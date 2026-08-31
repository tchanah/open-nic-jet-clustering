"""Cocotb tests for jc_setkin -- merged four-momentum back to coordinates.

Compared against float64 evaluations of FastJet's own formulas, not against
a Python re-implementation of the fixed-point path. The question is whether
the hardware agrees with the physics, and a mirror of the arithmetic would
repeat any mistake in it.

Tolerances come from what consumes each output:

  phi, rapidity   jc_dist truncates by JC_DELTA_SHIFT before squaring, so one
                  delta LSB (1.87e-7 rad) is the natural bound
  weight          ranked in Q7.25, so its own LSB (3e-8) is the floor; the
                  log unit contributes ~1.1e-8

Rapidity deliberately uses FastJet's stable form. The interesting inputs are
therefore jets close to the beam, where E - pz is small and the textbook
0.5*log((E+pz)/(E-pz)) loses its significance -- test_forward_jets covers it.
"""

import json
import math
import pathlib
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

LUTS = json.loads(
    (pathlib.Path(__file__).resolve().parents[3] / "model" / "luts.json")
    .read_text())

P4_W = LUTS["formats"]["jc_lut_energy"]["width"]
P4_FRAC = LUTS["formats"]["jc_lut_energy"]["frac"]
WGT_W = LUTS["formats"]["jc_lut_neg2log2e"]["width"]
WGT_FRAC = LUTS["formats"]["jc_lut_neg2log2e"]["frac"]
COORD_W = 32
COORD_SCALE = LUTS["coord_scale"]

P4_ONE = 1 << P4_FRAC
P4_MASK = (1 << P4_W) - 1
COORD_MASK = (1 << COORD_W) - 1
WGT_MASK = (1 << WGT_W) - 1

DELTA_LSB_RAD = (1 << LUTS["delta_shift"]) / COORD_SCALE
TOL_RAD = DELTA_LSB_RAD
TOL_WGT = 1e-6            # well above the Q7.25 LSB and the log's 1.1e-8


def to_signed(word, width):
    return word - (1 << width) if word >> (width - 1) else word


def q(x):
    """GeV to Q14.34."""
    return int(round(x * P4_ONE))


def fastjet_kin(e, px, py, pz):
    """FastJet's own rapidity/phi/weight, in float64."""
    pt_sq = px * px + py * py
    phi = math.atan2(py, px) % (2 * math.pi)
    m2 = max(0.0, (e + pz) * (e - pz) - pt_sq)
    e_plus = e + abs(pz)
    rap = 0.5 * math.log((pt_sq + m2) / (e_plus * e_plus))
    if pz > 0:
        rap = -rap
    return rap, phi, -math.log2(pt_sq)


def massless_cell(pt, y, phi):
    """A massless four-momentum, which is what cells and their sums are."""
    return (pt * math.cosh(y), pt * math.cos(phi), pt * math.sin(phi),
            pt * math.sinh(y))


async def start_dut(dut):
    cocotb.start_soon(Clock(dut.aclk, 4, units="ns").start())
    dut.start.value = 0
    for s in ("in_e", "in_px", "in_py", "in_pz", "ext_log_valid", "ext_log_x"):
        getattr(dut, s).value = 0
    dut.aresetn.value = 0
    for _ in range(5):
        await RisingEdge(dut.aclk)
    dut.aresetn.value = 1
    await RisingEdge(dut.aclk)


async def run_one(dut, e, px, py, pz):
    """Feed a float four-momentum in GeV; return (rap, phi, wgt) as floats."""
    dut.in_e.value = q(e) & P4_MASK
    dut.in_px.value = q(px) & P4_MASK
    dut.in_py.value = q(py) & P4_MASK
    dut.in_pz.value = q(pz) & P4_MASK
    dut.start.value = 1
    await RisingEdge(dut.aclk)
    dut.start.value = 0
    for _ in range(300):
        await RisingEdge(dut.aclk)
        if dut.done.value:
            return (to_signed(int(dut.out_y.value), COORD_W) / COORD_SCALE,
                    int(dut.out_phi.value) / COORD_SCALE,
                    to_signed(int(dut.out_wgt.value), WGT_W) / (1 << WGT_FRAC))
    raise AssertionError(f"setkin never finished for ({e}, {px}, {py}, {pz})")


def check(got, e, px, py, pz, label):
    rap, phi, wgt = got
    want_rap, want_phi, want_wgt = fastjet_kin(e, px, py, pz)

    dphi = abs(phi - want_phi) % (2 * math.pi)
    dphi = min(dphi, 2 * math.pi - dphi)

    assert abs(rap - want_rap) < TOL_RAD, (
        f"{label}: rapidity {rap:.9f} vs FastJet {want_rap:.9f} "
        f"(err {abs(rap-want_rap):.2e})")
    assert dphi < TOL_RAD, (
        f"{label}: phi {phi:.9f} vs {want_phi:.9f} (err {dphi:.2e})")
    assert abs(wgt - want_wgt) < TOL_WGT, (
        f"{label}: weight {wgt:.9f} vs {want_wgt:.9f} "
        f"(err {abs(wgt-want_wgt):.2e})")
    return abs(rap - want_rap), dphi, abs(wgt - want_wgt)


@cocotb.test()
async def test_massless_jets_across_the_grid(dut):
    """Single cells: massless, so mT2 is exactly pt_sq and m2 clamps to zero.

    This is the common case -- most merges combine nearly-collinear cells and
    stay close to massless.
    """
    await start_dut(dut)
    worst = [0.0, 0.0, 0.0]
    for pt in (0.5, 5.0, 50.0, 500.0):
        for y in (-2.45, -1.05, 0.05, 1.35, 2.45):
            for phi in (0.0, 1.1, 3.0, 4.7, 6.1):
                e, px, py, pz = massless_cell(pt, y, phi)
                got = await run_one(dut, e, px, py, pz)
                errs = check(got, e, px, py, pz, f"pt={pt} y={y} phi={phi}")
                worst = [max(a, b) for a, b in zip(worst, errs)]
    dut._log.info("worst massless: rap %.3e rad, phi %.3e rad, wgt %.3e",
                  *worst)


@cocotb.test()
async def test_massive_jets(dut):
    """Merged jets carry real mass, which is what mT2 exists for.

    E-scheme recombination of two cells at different angles gives a jet with
    m > 0, so mT2 > pt_sq and the (E+pz)(E-pz) branch is the live one.
    """
    await start_dut(dut)
    rng = random.Random(81)
    worst = [0.0, 0.0, 0.0]
    for _ in range(40):
        # Two massless cells within a plausible jet radius, summed.
        pt1 = rng.uniform(1.0, 200.0)
        y1 = rng.uniform(-2.4, 2.4)
        p1 = rng.uniform(0, 2 * math.pi)
        a = massless_cell(pt1, y1, p1)
        b = massless_cell(rng.uniform(0.5, pt1),
                          y1 + rng.uniform(-0.4, 0.4),
                          p1 + rng.uniform(-0.4, 0.4))
        e, px, py, pz = (a[i] + b[i] for i in range(4))
        got = await run_one(dut, e, px, py, pz)
        errs = check(got, e, px, py, pz, f"merged pt1={pt1:.1f}")
        worst = [max(x, y) for x, y in zip(worst, errs)]
    dut._log.info("worst massive: rap %.3e rad, phi %.3e rad, wgt %.3e", *worst)


@cocotb.test()
async def test_forward_jets(dut):
    """Close to the beam, where E - pz is small.

    This is exactly the case FastJet's stable form exists for: the textbook
    0.5*log((E+pz)/(E-pz)) loses its significance here, while dividing by
    (E+|pz|)^2 stays well conditioned. If the wrong formula had been used,
    this is the test that would say so.
    """
    await start_dut(dut)
    worst = 0.0
    for y in (2.0, 2.2, 2.4, 2.45, -2.45, -2.4):
        for pt in (0.5, 20.0, 300.0):
            e, px, py, pz = massless_cell(pt, y, 0.7)
            got = await run_one(dut, e, px, py, pz)
            errs = check(got, e, px, py, pz, f"forward y={y} pt={pt}")
            worst = max(worst, errs[0])
    dut._log.info("worst forward rapidity error: %.3e rad", worst)


@cocotb.test()
async def test_rapidity_sign_follows_pz(dut):
    """Positive pz must give positive rapidity, and symmetrically."""
    await start_dut(dut)
    for y in (0.35, 1.25, 2.05):
        e, px, py, pz = massless_cell(10.0, y, 2.0)
        pos = await run_one(dut, e, px, py, pz)
        neg = await run_one(dut, e, px, py, -pz)
        assert pos[0] > 0 and neg[0] < 0, (
            f"y={y}: got {pos[0]:.4f} for +pz and {neg[0]:.4f} for -pz")
        assert abs(pos[0] + neg[0]) < TOL_RAD, (
            f"y={y}: not antisymmetric, {pos[0]:.9f} vs {neg[0]:.9f}")


@cocotb.test()
async def test_weight_matches_the_ingest_convention(dut):
    """weight = log2(1/pt^2), the same quantity jc_ingest builds from tables.

    The two paths must agree or a merged jet would be ranked on a different
    scale from an unmerged cell.
    """
    await start_dut(dut)
    for pt in (0.2, 1.0, 10.0, 100.0, 1000.0):
        e, px, py, pz = massless_cell(pt, 0.85, 1.9)
        rap, phi, wgt = await run_one(dut, e, px, py, pz)
        want = -math.log2(pt * pt)
        assert abs(wgt - want) < TOL_WGT, (
            f"pt={pt}: weight {wgt:.9f}, expected {want:.9f}")


@cocotb.test()
async def test_shared_log_is_borrowable_when_idle(dut):
    """jc_ctrl uses the same log unit for the nn_dist_log refresh.

    The response must come back tagged to the borrower, and the unit must
    still be correct for its own work afterwards.
    """
    await start_dut(dut)
    assert dut.ext_log_ready.value == 1, "log not offered while idle"

    x = 1 << 40
    dut.ext_log_valid.value = 1
    dut.ext_log_x.value = x
    await RisingEdge(dut.aclk)
    dut.ext_log_valid.value = 0

    for _ in range(20):
        await RisingEdge(dut.aclk)
        if dut.ext_log_rsp_valid.value:
            got = int(dut.ext_log_rsp.value) / (1 << LUTS["log2_out_frac"])
            assert abs(got - 40.0) < 1e-6, f"borrowed log2(2^40) = {got}"
            break
    else:
        raise AssertionError("borrowed log request never answered")

    e, px, py, pz = massless_cell(25.0, -1.15, 4.2)
    check(await run_one(dut, e, px, py, pz), e, px, py, pz, "after borrow")
