"""Cocotb tests for jc_cordic -- atan2(py, px) as a binary angle.

Checked against math.atan2 rather than against a Python CORDIC, because a
re-implementation would repeat any mistake in the rotation sequence and the
whole point is the angle being right.

The bound is one delta LSB, 1.87e-7 rad: phi feeds jc_dist, which truncates
by JC_DELTA_SHIFT before squaring, so error below that disappears. 28
iterations should give ~2^-28 rad of residual plus the table quantisation.

Magnitudes are swept deliberately across the dynamic range a merged jet can
have -- the pt floor puts max(|px|,|py|) near 2^31 in Q14.34 terms, a TeV jet
near 2^44 -- because the normalisation step is exactly what stops small
inputs from losing the late rotations.
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
COORD_W = 32
COORD_SCALE = LUTS["coord_scale"]
P4_MASK = (1 << P4_W) - 1

DELTA_LSB_RAD = (1 << LUTS["delta_shift"]) / COORD_SCALE
TOL_RAD = DELTA_LSB_RAD          # one delta LSB; see the module docstring


def to_signed(word, width):
    return word - (1 << width) if word >> (width - 1) else word


def angle_error(got_units, px, py):
    """Shortest angular distance between the DUT result and atan2, in rad."""
    want = math.atan2(py, px) % (2 * math.pi)
    have = (got_units / COORD_SCALE) % (2 * math.pi)
    d = abs(have - want) % (2 * math.pi)
    return min(d, 2 * math.pi - d)


async def start_dut(dut):
    cocotb.start_soon(Clock(dut.aclk, 4, units="ns").start())
    dut.start.value = 0
    dut.in_px.value = 0
    dut.in_py.value = 0
    dut.aresetn.value = 0
    for _ in range(5):
        await RisingEdge(dut.aclk)
    dut.aresetn.value = 1
    await RisingEdge(dut.aclk)


async def run_one(dut, px, py):
    dut.in_px.value = px & P4_MASK
    dut.in_py.value = py & P4_MASK
    dut.start.value = 1
    await RisingEdge(dut.aclk)
    dut.start.value = 0
    for _ in range(200):
        await RisingEdge(dut.aclk)
        if dut.done.value:
            return int(dut.out_phi.value)
    raise AssertionError(f"cordic never finished for ({px}, {py})")


@cocotb.test()
async def test_cardinal_directions(dut):
    """The four axes must land on exact binary-angle quarters.

    These are the cases where a quadrant-fold mistake shows up as a clean
    90-degree error rather than as noise.
    """
    await start_dut(dut)
    m = 1 << 40
    cases = [
        ((m, 0), 0),                    # +x  -> 0
        ((0, m), 1 << 30),              # +y  -> pi/2
        ((-m, 0), 1 << 31),             # -x  -> pi
        ((0, -m), 3 << 30),             # -y  -> 3pi/2
    ]
    for (px, py), want in cases:
        got = await run_one(dut, px, py)
        # Wrap-aware comparison: 3pi/2 and -pi/2 are the same angle.
        diff = min((got - want) % (1 << COORD_W), (want - got) % (1 << COORD_W))
        assert diff < 64, (
            f"atan2({py},{px}): got {got} ({360*got/2**32:.4f} deg), "
            f"expected {want} ({360*want/2**32:.4f} deg)")


@cocotb.test()
async def test_diagonals(dut):
    """45-degree cases exercise the very first rotation, atan(1) = pi/4."""
    await start_dut(dut)
    m = 1 << 38
    for px, py in ((m, m), (-m, m), (-m, -m), (m, -m)):
        got = await run_one(dut, px, py)
        err = angle_error(got, px, py)
        assert err < TOL_RAD, f"atan2({py},{px}): {err:.3e} rad"


@cocotb.test()
async def test_all_quadrants_randomised(dut):
    """Randomised over sign combinations at a fixed, comfortable magnitude."""
    await start_dut(dut)
    rng = random.Random(71)
    worst = 0.0
    for _ in range(60):
        px = rng.randrange(1, 1 << 42) * rng.choice([1, -1])
        py = rng.randrange(1, 1 << 42) * rng.choice([1, -1])
        got = await run_one(dut, px, py)
        err = angle_error(got, px, py)
        worst = max(worst, err)
        assert err < TOL_RAD, f"atan2({py},{px}): {err:.3e} rad"
    dut._log.info("worst angle error: %.3e rad (%.1f%% of a delta LSB)",
                  worst, 100 * worst / DELTA_LSB_RAD)


@cocotb.test()
async def test_small_magnitudes_keep_precision(dut):
    """The normalisation step is what this checks.

    A jet at the pt floor has max(|px|,|py|) near 2^31 in Q14.34 integer
    terms. Without normalising, the late rotations would shift that down to a
    handful of bits and the angle would degrade exactly where soft jets live.
    """
    await start_dut(dut)
    rng = random.Random(72)
    worst, worst_case = 0.0, None
    for shift in (31, 28, 24, 20, 16):
        for _ in range(8):
            px = rng.randrange(1, 1 << shift) * rng.choice([1, -1])
            py = rng.randrange(1, 1 << shift) * rng.choice([1, -1])
            got = await run_one(dut, px, py)
            err = angle_error(got, px, py)
            if err > worst:
                worst, worst_case = err, (px, py)
            assert err < TOL_RAD, (
                f"atan2({py},{px}) at 2^{shift}: {err:.3e} rad -- "
                f"normalisation is not preserving small inputs")
    dut._log.info("worst small-magnitude error: %.3e rad at %s",
                  worst, worst_case)


@cocotb.test()
async def test_ratio_invariance(dut):
    """Only the ratio matters: scaling both inputs must not move the angle.

    This is the property normalisation relies on, so it is worth asserting
    rather than assuming.
    """
    await start_dut(dut)
    base_px, base_py = 3, 7
    results = []
    for k in range(20, 45, 4):
        got = await run_one(dut, base_px << k, base_py << k)
        results.append(got)
    spread = max(results) - min(results)
    assert spread < 256, (
        f"angle moved by {spread} units across 2^20..2^44 scaling: {results}")


@cocotb.test()
async def test_near_axis_angles(dut):
    """Very small angles: py tiny against a large px.

    atan2 is ~py/px here, so this is where an unnormalised CORDIC would
    quantise the answer to zero.
    """
    await start_dut(dut)
    px = 1 << 44
    for k in range(10, 30, 3):
        py = 1 << k
        got = await run_one(dut, px, py)
        err = angle_error(got, px, py)
        assert err < TOL_RAD, (
            f"atan2(2^{k}, 2^44) = {math.atan2(py, px):.3e} rad, "
            f"error {err:.3e}")


@cocotb.test()
async def test_zero_input_completes(dut):
    """px = py = 0 has no angle; it must finish rather than hang.

    jc_setkin guards on pt_sq before starting, so this only protects against
    a stray request -- but a hang would be far worse than a wrong angle.
    """
    await start_dut(dut)
    got = await run_one(dut, 0, 0)
    assert got == 0, f"zero input gave {got}"

    # And the unit must still work afterwards.
    m = 1 << 40
    got = await run_one(dut, m, m)
    assert angle_error(got, m, m) < TOL_RAD, "did not recover after zero input"
