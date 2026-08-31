"""Cocotb tests for jc_fp32 -- Q14.34 to IEEE-754 binary32.

The reference is struct.pack('>f', v / 2**34), which is what the host will see
and what the end-to-end bench builds its expected frames from. That division
is exact -- a 48-bit integer fits a float64 mantissa with five bits to spare --
so the ONLY rounding in the reference is the float64 -> float32 narrowing, done
round-to-nearest-even. The RTL has to match it bit for bit, not approximately.

Rounding is where a fixed-to-float converter actually goes wrong, and random
vectors are bad at finding it: a tie needs the guard bit set with nothing
below, which random values essentially never produce. So the ties are
constructed by hand, both parities, at several exponents.

jc_fp32 is a ONE-CYCLE pipeline as of step 8t, not combinational: the
leading-one detect and the normalising shift sit on one side of a register,
rounding and the exponent on the other. Nothing about the arithmetic changed,
so every expected value below is the one it always was.
"""

import random
import struct

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

P4_W = 48
P4_FRAC = 34
P4_MASK = (1 << P4_W) - 1


def ref_fp32(v_int):
    """The contract: exact scale-down, then one RNE narrowing."""
    return struct.unpack(">I", struct.pack(">f", v_int / (1 << P4_FRAC)))[0]


def as_float(bits):
    return struct.unpack(">f", struct.pack(">I", bits))[0]


async def start_dut(dut):
    """Start aclk, once per test.

    PER TEST, not once for the file. cocotb cancels every coroutine a test
    started with start_soon when that test ends, so a clock started in the
    first test is dead by the second: the next await RisingEdge never fires,
    the simulation runs out of events, and it reports as "Simulator shut down
    prematurely" rather than as a hang or a wrong answer.

    The edge before returning matters too. A value written after the current
    timestep's ReadWrite phase lands AFTER the edge the DUT samples, so the
    first conversion of each test would clock in stale fx and read as a
    rounding bug. Same trap as send_frame's in test_jet_clustering.py.
    """
    cocotb.start_soon(Clock(dut.aclk, 4, units="ns").start())
    await RisingEdge(dut.aclk)


async def convert(dut, v_int):
    dut.fx.value = v_int & P4_MASK
    await RisingEdge(dut.aclk)      # the pipeline register captures fx here
    await Timer(1, units="ns")      # and the second half settles after it
    return int(dut.fp.value)


async def check(dut, v_int, label):
    # The DUT reads its input as SIGNED, so a test vector at or above 2^47 is
    # not the number the reference thinks it is -- bit 47 is the sign. Caught
    # here rather than reported as an RTL failure, which is how the first
    # version of test_rounding_ties wasted a run.
    assert -(1 << (P4_W - 1)) <= v_int < (1 << (P4_W - 1)), (
        f"{label}: test vector {v_int} is outside signed Q14.34, so the "
        f"comparison would be meaningless")
    got = await convert(dut, v_int)
    want = ref_fp32(v_int)
    assert got == want, (
        f"{label}: fx={v_int} ({v_int / (1 << P4_FRAC):.12g})\n"
        f"  got  0x{got:08x} = {as_float(got):.12g}\n"
        f"  want 0x{want:08x} = {as_float(want):.12g}")


@cocotb.test()
async def test_zero_and_units(dut):
    """Zero, one LSB either way, and exactly 1.0."""
    await start_dut(dut)
    for v, label in [(0, "zero"),
                     (1, "+1 LSB"),
                     (-1, "-1 LSB"),
                     (1 << P4_FRAC, "+1.0"),
                     (-(1 << P4_FRAC), "-1.0")]:
        await check(dut, v, label)

    # Zero must be +0, not -0: a negative zero would be a legal float that
    # compares equal but does not round-trip through a byte comparison.
    assert await convert(dut, 0) == 0, "zero is not +0"


@cocotb.test()
async def test_format_extremes(dut):
    """The ends of Q14.34, including the one asymmetric value.

    -2^47 is its own two's complement, which is exactly the case a magnitude
    taken in W bits rather than W+1 has to get right.
    """
    await start_dut(dut)
    for v, label in [((1 << (P4_W - 1)) - 1, "largest positive"),
                     (-(1 << (P4_W - 1)), "most negative"),
                     (-(1 << (P4_W - 1)) + 1, "most negative + 1")]:
        await check(dut, v, label)


@cocotb.test()
async def test_rounding_ties(dut):
    """Exactly-half cases, both parities, at several exponents.

    Ties to even means the tie breaks on the last KEPT bit, so the same
    fraction with the low bit clear rounds down and with it set rounds up.
    Random stimulus does not reach these.
    """
    await start_dut(dut)
    # Below msb=24 there is no bit under the guard, so no exact tie exists.
    # Above msb=46 there is no headroom either: bit 47 is the sign, so 2^47
    # is not a positive input and its negation is out of range too.
    for msb in (46, 44, 40, 34, 30, 25, 24):
        base = 1 << msb
        guard = 1 << (msb - 24)
        lsb = 1 << (msb - 23)

        await check(dut, base | guard, f"tie, even, msb={msb}")
        await check(dut, base | lsb | guard, f"tie, odd, msb={msb}")
        # Just above and just below half, which must not tie-break at all.
        if msb > 24:
            await check(dut, base | guard | 1, f"above half, msb={msb}")
            await check(dut, base | (guard - 1), f"below half, msb={msb}")
        await check(dut, -(base | guard), f"tie, even, negative, msb={msb}")
        await check(dut, -(base | lsb | guard), f"tie, odd, negative, msb={msb}")


@cocotb.test()
async def test_exact_at_every_exponent(dut):
    """One clean power of two per reachable exponent, both signs.

    Walks the leading-one detect across its whole range: every msb from 0 to
    47 is a different shift amount and a different exponent.
    """
    await start_dut(dut)
    for msb in range(P4_W - 1):
        await check(dut, 1 << msb, f"2^{msb - P4_FRAC}")
        await check(dut, -(1 << msb), f"-2^{msb - P4_FRAC}")


@cocotb.test()
async def test_randomised(dut):
    """Broad coverage, including magnitudes concentrated near each exponent."""
    await start_dut(dut)
    rng = random.Random(4242)
    for _ in range(400):
        v = rng.randrange(-(1 << (P4_W - 1)), 1 << (P4_W - 1))
        await check(dut, v, "random full range")
    for _ in range(400):
        msb = rng.randrange(P4_W - 1)
        v = rng.randrange(1 << msb, 1 << (msb + 1)) if msb else 1
        if rng.random() < 0.5:
            v = -v
        await check(dut, v, f"random near 2^{msb}")


@cocotb.test()
async def test_physical_jet_energies(dut):
    """Values the thing will actually see: GeV-scale four-momenta."""
    await start_dut(dut)
    for gev in (0.2, 1.0, 5.0, 20.0, 50.0, 64.9, 100.0, 1000.0, 8191.0):
        v = int(round(gev * (1 << P4_FRAC)))
        await check(dut, v, f"{gev} GeV")
        await check(dut, -v, f"-{gev} GeV")
