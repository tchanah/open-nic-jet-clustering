"""Cocotb tests for jc_dist -- the replicated distance lane.

Two independent checks run here, and they catch different things:

  * bit-exactness against a Python re-implementation of the fixed-point
    arithmetic, which catches width, shift and saturation mistakes;
  * closure against a floating-point dR in radians, computed from the real
    grid via luts.json, which catches the thing bit-exactness cannot -- that
    the shared coordinate scale actually means what jc_defs.vh says it does.

The second is the one that matters for the scale unification: rapidity and
azimuth were put on a common 2^32-per-turn scale specifically so dy^2 +
dphi^2 is a real dR^2, and only a comparison in radians proves it.
"""

import json
import math
import pathlib
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

LUTS_PATH = pathlib.Path(__file__).resolve().parents[3] / "model" / "luts.json"
if not LUTS_PATH.exists():
    raise RuntimeError(
        f"{LUTS_PATH} missing -- run model/gen_luts.py before this bench")
LUTS = json.loads(LUTS_PATH.read_text())

COORD_W = 32
COORD_MASK = (1 << COORD_W) - 1
DELTA_W = LUTS["delta_w"]
SHIFT = LUTS["delta_shift"]
GEO_W = LUTS["geo_w"]
COORD_SCALE = LUTS["coord_scale"]
PHI_SHIFT = LUTS["phi_bin_shift"]
RAP_BINS = LUTS["rap_bins"]
PHI_BINS = LUTS["phi_bins"]

# Deltas are unsigned magnitudes, so the full width is available.
SAT_MAG = (1 << DELTA_W) - 1

LATENCY = 3


def to_signed(word, width):
    return word - (1 << width) if word >> (width - 1) else word


def saturate(mag):
    return min(SAT_MAG, mag)


def model(y_q, phi_q, y_k, phi_k):
    """The RTL's arithmetic, in Python. Inputs are raw 32-bit words."""
    dy = to_signed(y_q & COORD_MASK, COORD_W) - to_signed(y_k & COORD_MASK, COORD_W)
    # A full turn is exactly 2^32, so the wrapped difference read as signed
    # is already the shortest separation.
    dphi = to_signed((phi_q - phi_k) & COORD_MASK, COORD_W)
    # Magnitude first, so the shift truncates toward zero and the result is
    # symmetric in its arguments.
    dy_s = saturate(abs(dy) >> SHIFT)
    dphi_s = saturate(abs(dphi) >> SHIFT)
    return dy_s * dy_s + dphi_s * dphi_s


def rap_word(iy):
    return LUTS["words"]["jc_lut_rapidity"][iy]


def rap_radians(iy):
    return to_signed(rap_word(iy), COORD_W) / COORD_SCALE


async def start_dut(dut):
    cocotb.start_soon(Clock(dut.aclk, 4, units="ns").start())  # 250 MHz
    dut.y_q.value = 0
    dut.phi_q.value = 0
    dut.y_k.value = 0
    dut.phi_k.value = 0
    dut.en.value = 1
    dut.aresetn.value = 0
    for _ in range(5):
        await RisingEdge(dut.aclk)
    dut.aresetn.value = 1
    await RisingEdge(dut.aclk)


async def run_pairs(dut, pairs):
    """Stream (y_q, phi_q, y_k, phi_k) tuples through, return the results."""
    out = []
    for i in range(len(pairs) + LATENCY):
        if i < len(pairs):
            y_q, phi_q, y_k, phi_k = pairs[i]
            dut.y_q.value = y_q & COORD_MASK
            dut.phi_q.value = phi_q & COORD_MASK
            dut.y_k.value = y_k & COORD_MASK
            dut.phi_k.value = phi_k & COORD_MASK
        await RisingEdge(dut.aclk)
        if i >= LATENCY:
            out.append(int(dut.geo_dist_sq.value))
    return out


@cocotb.test()
async def test_zero_distance(dut):
    """A pseudojet against itself is exactly zero, at several grid points."""
    await start_dut(dut)
    pairs = []
    for iy in (0, 25, 49):
        for iphi in (0, 17, 63):
            y, p = rap_word(iy), iphi << PHI_SHIFT
            pairs.append((y, p, y, p))

    got = await run_pairs(dut, pairs)
    assert all(g == 0 for g in got), f"self-distance not zero: {got}"


@cocotb.test()
async def test_random_words_bit_exact(dut):
    """Randomised over the whole 32-bit coordinate space."""
    await start_dut(dut)
    rng = random.Random(31)
    pairs = [tuple(rng.randrange(1 << COORD_W) for _ in range(4))
             for _ in range(500)]

    got = await run_pairs(dut, pairs)
    for p, g in zip(pairs, got):
        exp = model(*p)
        assert g == exp, f"pair={p}: got {g}, expected {exp}"


@cocotb.test()
async def test_phi_wraps_the_short_way(dut):
    """The 0/2*pi seam must cost nothing: neighbours across it are close.

    This is the property the binary-angle scale was chosen for. Bin 63 and
    bin 0 are adjacent on the detector, so their separation must be one bin,
    not sixty-three.
    """
    await start_dut(dut)
    y = rap_word(25)
    adjacent = (63 << PHI_SHIFT, 0)                  # across the seam
    interior = (30 << PHI_SHIFT, 31 << PHI_SHIFT)    # nowhere near it

    pairs = [(y, adjacent[0], y, adjacent[1]),
             (y, interior[0], y, interior[1]),
             (y, 0, y, 32 << PHI_SHIFT)]             # exactly opposite, pi

    got = await run_pairs(dut, pairs)
    assert got[0] == got[1], (
        f"seam is not free: adjacent across 0/2pi gave {got[0]}, "
        f"the same gap in the interior gave {got[1]}")
    assert got[2] > got[0], "half a turn should be further than one bin"


@cocotb.test()
async def test_saturation(dut):
    """Deltas past ~pi saturate rather than wrap.

    A wrapped delta would read as SMALL and could win an argmin it should
    have lost -- a merge between two objects that are nowhere near each other.
    Saturating is lossless here: anything at the limit already exceeds R^2.
    """
    await start_dut(dut)
    # Straddle the coordinate range so dy reaches 2^31 without either operand
    # itself wrapping into the sign bit.
    pairs = [
        (1 << 30, 0, -(1 << 30), 0),     # dy = +2^31
        (-(1 << 30), 0, 1 << 30, 0),     # dy = -2^31, must give the same
        (0, 0, 0, 1 << 31),              # dphi exactly half a turn
    ]
    got = await run_pairs(dut, pairs)

    for p, g in zip(pairs, got):
        assert g == model(*p), f"pair={p}: got {g}, model {model(*p)}"
    assert got[0] == SAT_MAG * SAT_MAG, f"positive dy did not saturate: {got[0]}"
    assert got[1] == got[0], (
        f"saturation is not symmetric: +dy gave {got[0]}, -dy gave {got[1]}")

    # Whatever it saturates to must still read as further than any usable R.
    r_max_sq = (int(round(1.0 * COORD_SCALE)) >> SHIFT) ** 2
    assert min(got) > r_max_sq, "a saturated delta must exceed R^2 for R = 1"


@cocotb.test()
async def test_symmetry(dut):
    """dR^2(a,b) == dR^2(b,a) -- required for a deterministic argmin."""
    await start_dut(dut)
    rng = random.Random(32)
    base = [(rng.randrange(RAP_BINS), rng.randrange(PHI_BINS),
             rng.randrange(RAP_BINS), rng.randrange(PHI_BINS))
            for _ in range(150)]

    fwd = [(rap_word(a), b << PHI_SHIFT, rap_word(c), d << PHI_SHIFT)
           for a, b, c, d in base]
    rev = [(rap_word(c), d << PHI_SHIFT, rap_word(a), b << PHI_SHIFT)
           for a, b, c, d in base]

    got_f = await run_pairs(dut, fwd)
    got_r = await run_pairs(dut, rev)
    for t, f, r in zip(base, got_f, got_r):
        assert f == r, f"asymmetric at {t}: {f} vs {r}"


@cocotb.test()
async def test_matches_radians_on_the_real_grid(dut):
    """dR^2 in fixed point must be dR^2 in radians, on actual grid points.

    Bit-exactness cannot catch a wrong SCALE -- the Python model would be
    wrong the same way. This converts the hardware result back to radians and
    compares against the geometry, which is what proves rapidity and azimuth
    really do share one scale.
    """
    await start_dut(dut)
    rng = random.Random(33)
    quads = [(rng.randrange(RAP_BINS), rng.randrange(PHI_BINS),
              rng.randrange(RAP_BINS), rng.randrange(PHI_BINS))
             for _ in range(300)]

    pairs = [(rap_word(a), b << PHI_SHIFT, rap_word(c), d << PHI_SHIFT)
             for a, b, c, d in quads]
    got = await run_pairs(dut, pairs)

    unit = (1 << SHIFT) / COORD_SCALE        # radians per delta LSB
    worst = 0.0
    for (iy1, ip1, iy2, ip2), g in zip(quads, got):
        dy = rap_radians(iy1) - rap_radians(iy2)
        dphi = 2 * math.pi * (ip1 - ip2) / PHI_BINS
        dphi = (dphi + math.pi) % (2 * math.pi) - math.pi     # shortest way
        true_sq = dy * dy + dphi * dphi

        if math.sqrt(true_sq) > math.pi:
            continue                          # saturation territory, by design

        hw_sq = g * unit * unit
        err = abs(hw_sq - true_sq)
        worst = max(worst, err)
        # Each delta truncates by up to one LSB, so dR^2 carries up to
        # 2*|d|*LSB per term -- about 1.2e-6 rad^2 at the pi/2 limit. A wrong
        # SCALE, the thing this test exists to catch, would be off by 27% of
        # the value itself, so this bound still fails it decisively.
        assert err < 5e-6, (
            f"({iy1},{ip1})-({iy2},{ip2}): hw {hw_sq:.9f} rad^2, "
            f"true {true_sq:.9f} rad^2, err {err:.2e}")
    dut._log.info("worst dR^2 error vs radians: %.3e rad^2", worst)


@cocotb.test()
async def test_r_squared_threshold_is_representable(dut):
    """R = 0.4 and its square must sit comfortably inside the formats.

    jc_sweep clamps the per-row minimum against an R^2 register in exactly
    these units, so if R^2 did not fit, every distance would read as beam.
    """
    await start_dut(dut)
    for r in (0.2, 0.4, 0.7, 1.0):
        r_units = int(round(r * COORD_SCALE)) >> SHIFT
        r_sq = r_units * r_units
        assert r_units <= SAT_MAG, (
            f"R = {r} is {r_units} delta units, past the saturation limit "
            f"{SAT_MAG}")
        assert r_sq < (1 << GEO_W), f"R^2 for R = {r} overflows JC_GEO_W"
    dut._log.info("R = 0.4 is %d delta units, R^2 = %d of %d bits",
                  int(round(0.4 * COORD_SCALE)) >> SHIFT,
                  (int(round(0.4 * COORD_SCALE)) >> SHIFT) ** 2, GEO_W)
