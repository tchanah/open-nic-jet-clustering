"""Cocotb tests for jc_ingest -- calorimeter cell to pseudojet record.

Expected values come from model/luts.json, the same artefact gen_luts.py
emits jc_luts.vh from, so this bench cannot drift from the RTL's tables. It
re-implements only the *arithmetic* -- the two multiply stages and their
rounding -- which is the part actually under test.

Run model/gen_luts.py first; without it neither the RTL nor this bench has
tables.

Conversion under test, all fixed point:

    energy   = E[ecode]                          Q14.34
    pt       = energy * sech(y)  >> 30           Q14.34, internal
    pz       = energy * tanh(y)  >> 30           Q14.34
    px       = pt     * cos(phi) >> 30           Q14.34
    py       = pt     * sin(phi) >> 30           Q14.34
    rapidity = y[iy]                             Q3.29,  exact
    phi      = iphi << 26                        binary angle, exact
    weight   = -2log2(E) + -2log2(sech(y))       Q7.25,  one add
"""

import json
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

WORDS = LUTS["words"]
FMT = LUTS["formats"]
RAP_BINS = LUTS["rap_bins"]
PHI_BINS = LUTS["phi_bins"]
PHI_SHIFT = LUTS["phi_bin_shift"]

P4_W, P4_FRAC = FMT["jc_lut_energy"]["width"], FMT["jc_lut_energy"]["frac"]
TRIG_FRAC = FMT["jc_lut_sech"]["frac"]
ECODE_DEPTH = FMT["jc_lut_energy"]["depth"]
BIN_DEPTH = FMT["jc_lut_sech"]["depth"]
HALF_LSB = 1 << (TRIG_FRAC - 1)


def to_signed(word, width):
    return word - (1 << width) if word >> (width - 1) else word


def trunc(value, width):
    """Keep the low `width` bits and reinterpret as signed, as the RTL does."""
    value &= (1 << width) - 1
    return value - (1 << width) if value >> (width - 1) else value


def lut(name, index):
    fmt = FMT[name]
    return to_signed(WORDS[name][index], fmt["width"])


def make_cell(iy, iphi, ecode):
    return ((iy & 0xFF) << 24) | ((iphi & 0xFF) << 16) | (ecode & 0xFFFF)


def expect(iy, iphi, ecode):
    """Bit-exact expectation for one cell, masking exactly as the RTL does."""
    iy6, iphi6, ec12 = iy & 0x3F, iphi & 0x3F, ecode & 0x0FFF

    energy = lut("jc_lut_energy", ec12)
    pt = trunc((energy * lut("jc_lut_sech", iy6) + HALF_LSB) >> TRIG_FRAC, P4_W)
    pz = trunc((energy * lut("jc_lut_tanh", iy6) + HALF_LSB) >> TRIG_FRAC, P4_W)
    px = trunc((pt * lut("jc_lut_cos", iphi6) + HALF_LSB) >> TRIG_FRAC, P4_W)
    py = trunc((pt * lut("jc_lut_sin", iphi6) + HALF_LSB) >> TRIG_FRAC, P4_W)

    weight = trunc(lut("jc_lut_neg2log2e", ec12)
                   + lut("jc_lut_neg2logsech", iy6), FMT["jc_lut_neg2log2e"]["width"])

    return {
        "energy": energy,
        "px": px, "py": py, "pz": pz,
        "rapidity": lut("jc_lut_rapidity", iy6),
        "phi": iphi6 << PHI_SHIFT,
        "weight": weight,
        "err": int(iy >= RAP_BINS or iphi >= PHI_BINS or ecode >= ECODE_DEPTH),
    }


def observe(dut):
    """Sample the pseudojet outputs as signed integers."""
    return {
        "energy": to_signed(int(dut.pj_energy.value), P4_W),
        "px": to_signed(int(dut.pj_px.value), P4_W),
        "py": to_signed(int(dut.pj_py.value), P4_W),
        "pz": to_signed(int(dut.pj_pz.value), P4_W),
        "rapidity": to_signed(int(dut.pj_rapidity.value),
                              FMT["jc_lut_rapidity"]["width"]),
        "phi": int(dut.pj_phi.value),
        "weight": to_signed(int(dut.pj_beam_weight_log.value),
                            FMT["jc_lut_neg2log2e"]["width"]),
        "err": int(dut.pj_err.value),
    }


async def start_dut(dut):
    cocotb.start_soon(Clock(dut.aclk, 4, units="ns").start())  # 250 MHz
    dut.cell_valid.value = 0
    dut.cell_data.value = 0
    dut.cell_start.value = 0
    dut.cell_last.value = 0
    # jc_deframe's abort flag. Every test here drives well-formed cells and
    # exercises the range check instead, so it stays low; the path it feeds is
    # the same pj_err, covered by test_jc_deframe's length-error tests.
    dut.cell_err.value = 0
    dut.cell_event_seq.value = 0
    dut.pj_ready.value = 0
    dut.aresetn.value = 0
    for _ in range(5):
        await RisingEdge(dut.aclk)
    dut.aresetn.value = 1
    await RisingEdge(dut.aclk)


async def drive(dut, cells, gap=lambda: 0, err_at=()):
    """Feed cells, honouring cell_ready. First and last get start/last.

    err_at names the positions where jc_deframe would raise cell_err -- the
    sideband that fails an event whose length disagreed with its header.
    """
    for i, c in enumerate(cells):
        for _ in range(gap()):
            dut.cell_valid.value = 0
            await RisingEdge(dut.aclk)
        dut.cell_valid.value = 1
        dut.cell_data.value = c
        dut.cell_start.value = 1 if i == 0 else 0
        dut.cell_last.value = 1 if i == len(cells) - 1 else 0
        dut.cell_err.value = 1 if i in err_at else 0
        while True:
            await RisingEdge(dut.aclk)
            if dut.cell_ready.value:
                break
        dut.cell_valid.value = 0
    dut.cell_start.value = 0
    dut.cell_last.value = 0
    dut.cell_err.value = 0


async def collect(dut, n, stall=lambda: False):
    """Pull n pseudojet records, optionally back-pressuring."""
    out, flags = [], []
    while len(out) < n:
        ready = not stall()
        dut.pj_ready.value = 1 if ready else 0
        await RisingEdge(dut.aclk)
        if ready and dut.pj_valid.value:
            out.append(observe(dut))
            flags.append((bool(dut.pj_start.value), bool(dut.pj_last.value)))
    dut.pj_ready.value = 0
    return out, flags


def compare(got, exp, label):
    for key in exp:
        assert got[key] == exp[key], (
            f"{label}: {key} = {got[key]}, expected {exp[key]}")


@cocotb.test()
async def test_energy_sweep(dut):
    """Every energy code at one grid point: the 4096-entry tables end to end."""
    await start_dut(dut)
    iy, iphi = 25, 10
    cells = [make_cell(iy, iphi, e) for e in range(ECODE_DEPTH)]

    cocotb.start_soon(drive(dut, cells))
    got, _ = await collect(dut, len(cells))

    for e, g in enumerate(got):
        compare(g, expect(iy, iphi, e), f"ecode={e}")
    assert not any(g["err"] for g in got), "legal codes flagged"


@cocotb.test()
async def test_grid_sweep(dut):
    """Every legal (iy, iphi) at one energy: the position tables."""
    await start_dut(dut)
    ecode = 2048
    pairs = [(iy, iphi) for iy in range(RAP_BINS) for iphi in range(PHI_BINS)]
    cells = [make_cell(iy, iphi, ecode) for iy, iphi in pairs]

    cocotb.start_soon(drive(dut, cells))
    got, _ = await collect(dut, len(cells))

    for (iy, iphi), g in zip(pairs, got):
        compare(g, expect(iy, iphi, ecode), f"iy={iy} iphi={iphi}")
    assert not any(g["err"] for g in got), "legal grid points flagged"


@cocotb.test()
async def test_random_cells(dut):
    """Randomised over the whole legal space, all outputs bit-exact."""
    await start_dut(dut)
    rng = random.Random(11)
    triples = [(rng.randrange(RAP_BINS), rng.randrange(PHI_BINS),
                rng.randrange(ECODE_DEPTH)) for _ in range(400)]
    cells = [make_cell(*t) for t in triples]

    cocotb.start_soon(drive(dut, cells))
    got, flags = await collect(dut, len(cells))

    for t, g in zip(triples, got):
        compare(g, expect(*t), f"cell={t}")
    assert flags[0] == (True, False), f"cell_start not propagated: {flags[0]}"
    assert flags[-1] == (False, True), f"cell_last not propagated: {flags[-1]}"
    assert not any(s or l for s, l in flags[1:-1]), "spurious start/last"


@cocotb.test()
async def test_backpressure_and_bubbles(dut):
    """pj_ready freezes the whole pipeline; no cell may be lost or repeated."""
    await start_dut(dut)
    rng = random.Random(12)
    triples = [(rng.randrange(RAP_BINS), rng.randrange(PHI_BINS),
                rng.randrange(ECODE_DEPTH)) for _ in range(200)]
    cells = [make_cell(*t) for t in triples]

    gap_rng, stall_rng = random.Random(13), random.Random(14)
    cocotb.start_soon(drive(dut, cells, gap=lambda: gap_rng.randrange(0, 4)))
    got, _ = await collect(dut, len(cells),
                           stall=lambda: stall_rng.random() < 0.4)

    for t, g in zip(triples, got):
        compare(g, expect(*t), f"cell={t} under back-pressure")


@cocotb.test()
async def test_out_of_range_flagged(dut):
    """Illegal indices set pj_err and still produce finite numbers.

    A wrapped index would read a neighbouring tower and look entirely
    plausible downstream, which is why this is flagged rather than masked
    silently. The record stays finite so nothing propagates an infinity.
    """
    await start_dut(dut)
    bad = [
        (RAP_BINS, 0, 100),          # first illegal iy
        (63, 0, 100),                # top of the padded iy range
        (0, PHI_BINS, 100),          # iphi past the grid, wraps to 0
        (0, 0, ECODE_DEPTH),         # energy code past the table
        (200, 200, 60000),           # everything wrong at once
    ]
    good = [(10, 20, 300), (49, 63, 4095)]
    triples = bad + good
    cells = [make_cell(*t) for t in triples]

    cocotb.start_soon(drive(dut, cells))
    got, _ = await collect(dut, len(cells))

    for t, g in zip(triples, got):
        compare(g, expect(*t), f"cell={t}")

    assert [g["err"] for g in got] == [1] * len(bad) + [0] * len(good), \
        f"err flags wrong: {[g['err'] for g in got]}"


@cocotb.test()
async def test_cell_err_propagates_to_pj_err(dut):
    """jc_deframe's abort sideband must reach pj_err on that cell alone.

    The path was previously only exercised end to end -- jc_deframe proves it
    raises cell_err, jc_evbuf proves it drops on pj_err, and the middle was
    assumed. This asserts the middle directly: a perfectly legal cell carrying
    cell_err must come out flagged, and its neighbours must not.
    """
    await start_dut(dut)
    triples = [(10, 20, 300), (11, 21, 400), (12, 22, 500), (13, 23, 600)]
    cells = [make_cell(*t) for t in triples]

    cocotb.start_soon(drive(dut, cells, err_at={1}))
    got, _ = await collect(dut, len(cells))

    for i, (t, g) in enumerate(zip(triples, got)):
        exp = expect(*t)
        # The record itself is unaffected -- only the flag moves.
        exp["err"] = 1 if i == 1 else 0
        compare(g, exp, f"cell {i} of {triples}")

    assert [g["err"] for g in got] == [0, 1, 0, 0], (
        f"cell_err did not land on the right cell: {[g['err'] for g in got]}")


@cocotb.test()
async def test_cell_err_on_an_out_of_range_cell(dut):
    """cell_err and a range violation share one flag, so both together is
    still one flag -- there is no second failure path to get out of step."""
    await start_dut(dut)
    triples = [(50, 0, 100), (10, 20, 300)]      # first is out of range too
    cells = [make_cell(*t) for t in triples]

    cocotb.start_soon(drive(dut, cells, err_at={0, 1}))
    got, _ = await collect(dut, len(cells))

    assert [g["err"] for g in got] == [1, 1], (
        f"err flags {[g['err'] for g in got]}")


@cocotb.test()
async def test_massless_consistency(dut):
    """E^2 == px^2 + py^2 + pz^2, which is the physics the tables encode.

    Independent of the Python model -- it checks the LUT contents themselves
    are a consistent massless four-momentum, not merely that the RTL agrees
    with gen_luts.py. sech^2 + tanh^2 = 1 is what makes this hold.
    """
    await start_dut(dut)
    rng = random.Random(15)
    triples = [(rng.randrange(RAP_BINS), rng.randrange(PHI_BINS),
                rng.randrange(ECODE_DEPTH)) for _ in range(300)]
    cells = [make_cell(*t) for t in triples]

    cocotb.start_soon(drive(dut, cells))
    got, _ = await collect(dut, len(cells))

    scale = float(1 << P4_FRAC)
    worst = 0.0
    for t, g in zip(triples, got):
        energy = g["energy"] / scale
        mag = ((g["px"] / scale) ** 2 + (g["py"] / scale) ** 2
               + (g["pz"] / scale) ** 2) ** 0.5
        rel = abs(mag - energy) / energy
        worst = max(worst, rel)
        assert rel < 1e-6, f"cell={t}: |p| = {mag}, E = {energy}, rel {rel:.2e}"
    dut._log.info("worst |p| vs E relative error: %.2e", worst)
