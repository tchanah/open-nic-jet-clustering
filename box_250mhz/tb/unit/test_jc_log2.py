"""Cocotb tests for jc_log2 -- the engine's only logarithm.

Two things are under test and they are different questions:

  * bit-exactness against a Python model of the same table-and-interpolate
    arithmetic, which catches width, shift and indexing mistakes;
  * ACCURACY against math.log2, which is what actually matters, because the
    result feeds rapidity and the beam weight and nobody downstream can
    recover from a log that is merely self-consistent.

The accuracy bound is set by rapidity: an error eps in log2 becomes roughly
eps radians of rapidity, and one delta LSB is 1.87e-7 rad.

Two error terms contribute, and conflating them cost a round here. The
interpolation residual is 0.18 * 2^-2*IDX_BITS = 1.1e-8. The mantissa
truncation is 1.44 * 2^-(IDX_BITS+FRAC_BITS), which at FRAC_BITS=12 was
8.7e-8 -- eight times larger and easy to forget, because it is set by the
interpolation width rather than the table size. FRAC_BITS=20 puts it at
3.4e-10 and leaves the residual dominant, as intended.
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

IN_W = LUTS["log2_in_w"]
IDX_BITS = LUTS["log2_idx_bits"]
FRC_BITS = LUTS["log2_frac_bits"]
OUT_FRAC = LUTS["log2_out_frac"]
TAB = LUTS["words"]["jc_lut_log2m"]
TAB_FRAC = LUTS["formats"]["jc_lut_log2m"]["frac"]

MANT_W = IDX_BITS + FRC_BITS
# Six, not five: step 8t gave the table subtract its own stage. The unit has
# no handshake, so this number is the contract -- if it disagrees with the RTL
# every test here fails on out_valid rather than on a wrong answer.
LATENCY = 6

# Rapidity is the demanding consumer; see the module docstring.
DELTA_LSB_RAD = (1 << LUTS["delta_shift"]) / LUTS["coord_scale"]


def model(x):
    """The RTL's arithmetic in Python: encode, normalise, table, interpolate."""
    if x == 0:
        return 0
    e = x.bit_length() - 1
    norm = (x << (IN_W - 1 - e)) & ((1 << IN_W) - 1)
    mant = (norm >> (IN_W - 1 - MANT_W)) & ((1 << MANT_W) - 1)
    idx = mant >> FRC_BITS
    frc = mant & ((1 << FRC_BITS) - 1)
    slope = TAB[idx + 1] - TAB[idx]
    mant_log = TAB[idx] + ((slope * frc) >> FRC_BITS)
    return (e << OUT_FRAC) + (mant_log << (OUT_FRAC - TAB_FRAC))


async def start_dut(dut):
    cocotb.start_soon(Clock(dut.aclk, 4, units="ns").start())
    dut.in_valid.value = 0
    dut.in_x.value = 0
    dut.aresetn.value = 0
    for _ in range(5):
        await RisingEdge(dut.aclk)
    dut.aresetn.value = 1
    await RisingEdge(dut.aclk)


async def run(dut, values):
    """Stream values through and collect (log2, zero) per input."""
    out = []
    for i in range(len(values) + LATENCY):
        if i < len(values):
            dut.in_valid.value = 1
            dut.in_x.value = values[i]
        else:
            dut.in_valid.value = 0
        await RisingEdge(dut.aclk)
        if i >= LATENCY:
            assert dut.out_valid.value, f"out_valid dropped at result {i-LATENCY}"
            out.append((int(dut.out_log2.value), int(dut.out_zero.value)))
    dut.in_valid.value = 0
    return out


@cocotb.test()
async def test_exact_powers_of_two(dut):
    """log2(2^e) must be exactly e, with no interpolation residue.

    The mantissa is exactly 1.0 at a power of two, so the table's first entry
    is used verbatim; anything other than a clean integer here means the
    exponent and mantissa terms are misaligned.
    """
    await start_dut(dut)
    exps = list(range(IN_W))
    got = await run(dut, [1 << e for e in exps])

    for e, (val, zero) in zip(exps, got):
        assert zero == 0, f"2^{e} flagged zero"
        assert val == (e << OUT_FRAC), (
            f"log2(2^{e}) = {val / (1 << OUT_FRAC):.9f}, expected {e}")


@cocotb.test()
async def test_zero_flagged(dut):
    """x = 0 has no representable log; it must raise the flag, not wrap."""
    await start_dut(dut)
    got = await run(dut, [0, 1, 0, 1 << 40])
    assert got[0] == (0, 1), f"log2(0) gave {got[0]}"
    assert got[1] == (0, 0), "log2(1) should be 0 without the zero flag"
    assert got[2] == (0, 1)
    assert got[3][1] == 0


@cocotb.test()
async def test_bit_exact_against_model(dut):
    """Randomised across the full input width."""
    await start_dut(dut)
    rng = random.Random(61)
    vals = [1, 2, 3, (1 << IN_W) - 1]
    for _ in range(400):
        w = rng.randrange(1, IN_W + 1)
        vals.append(rng.randrange(1 << (w - 1), 1 << w))

    got = await run(dut, vals)
    for x, (val, zero) in zip(vals, got):
        assert zero == 0
        assert val == model(x), (
            f"x={x:#x}: got {val:#x}, model {model(x):#x}")


@cocotb.test()
async def test_accuracy_against_real_log2(dut):
    """The number that matters: how far from math.log2 is it?

    Bit-exactness cannot see a wrong table -- the Python model reads the same
    one. This compares against the mathematical answer.
    """
    await start_dut(dut)
    rng = random.Random(62)
    vals = []
    for _ in range(600):
        w = rng.randrange(1, IN_W + 1)
        vals.append(rng.randrange(1 << (w - 1), 1 << w))

    got = await run(dut, vals)
    worst, worst_x = 0.0, 0
    for x, (val, _z) in zip(vals, got):
        err = abs(val / (1 << OUT_FRAC) - math.log2(x))
        if err > worst:
            worst, worst_x = err, x
    dut._log.info("worst log2 error: %.3e at x=%#x  (%.1f%% of a delta LSB)",
                  worst, worst_x, 100 * worst / DELTA_LSB_RAD)
    # Interpolation residual is ~1.1e-8; this leaves headroom for the table
    # and product quantisation without admitting a mantissa-width mistake.
    assert worst < 3e-8, f"worst log2 error {worst:.3e} at x={worst_x:#x}"


@cocotb.test()
async def test_monotonic_across_an_octave(dut):
    """log2 must never decrease as x grows.

    A non-monotonic log would reorder nn_dist_log and change which pair
    merges -- the failure would look like a clustering bug, not a numeric one,
    so it is worth ruling out directly. The seam between table entries is
    where interpolation would break it, so the sweep steps across many.
    """
    await start_dut(dut)
    base = 1 << 60
    step = base >> (IDX_BITS + 4)
    vals = [base + i * step for i in range(400)]

    got = await run(dut, vals)
    prev = -1
    for x, (val, _z) in zip(vals, got):
        assert val >= prev, f"log2 decreased at x={x:#x}: {val} < {prev}"
        prev = val


@cocotb.test()
async def test_back_to_back_and_bubbles(dut):
    """Fully pipelined: results must not depend on input spacing."""
    await start_dut(dut)
    rng = random.Random(63)
    vals = [rng.randrange(1, 1 << 70) for _ in range(60)]

    dense = await run(dut, vals)

    sparse = []
    for x in vals:
        got = await run(dut, [x])
        sparse.append(got[0])
        for _ in range(rng.randrange(0, 4)):
            await RisingEdge(dut.aclk)

    assert dense == sparse, "results changed with input spacing"
