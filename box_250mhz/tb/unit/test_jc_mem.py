"""Cocotb tests for jc_mem -- the banked active list.

The property under test is the banking: entry k must live in bank k % LANES
at offset k // LANES, so that a lane-parallel read at offset o returns
entries {o*LANES .. o*LANES+LANES-1} and lane l's write-back lands on entry
o*LANES + l and nowhere else.

That mapping is what lets jc_sweep run without a crossbar, so an off-by-one
in it would not fail loudly -- it would quietly cluster the wrong pairs.
Every test here reads back through a different port than it wrote through.
"""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

NMAX = 128
LANES = 16
DEPTH = NMAX // LANES
IDX_W = 7
COORD_W = 32
WGT_W = 32
GEO_W = 49
NNLOG_W = 40
P4_W = 48

COORD_MASK = (1 << COORD_W) - 1
GEO_MASK = (1 << GEO_W) - 1
NNLOG_MASK = (1 << NNLOG_W) - 1
P4_MASK = (1 << P4_W) - 1
IDX_MASK = (1 << IDX_W) - 1


def bank_of(idx):
    return idx % LANES


def off_of(idx):
    return idx // LANES


def lane(vec, l, width):
    """Slice lane l out of a flattened bus."""
    return (int(vec) >> (l * width)) & ((1 << width) - 1)


def payload(idx):
    """Distinguishable per-entry values, so a bank mix-up shows as a mismatch."""
    return {
        "y": (0x1000000 + idx * 7) & COORD_MASK,
        "phi": (0x2000000 + idx * 11) & COORD_MASK,
        "wgt": (0x3000000 + idx * 13) & COORD_MASK,
        "e": (0x40000000 + idx * 17) & P4_MASK,
        "px": (0x50000000 + idx * 19) & P4_MASK,
        "py": (0x60000000 + idx * 23) & P4_MASK,
        "pz": (0x70000000 + idx * 29) & P4_MASK,
    }


async def start_dut(dut):
    cocotb.start_soon(Clock(dut.aclk, 4, units="ns").start())
    for sig in ("init_en", "set_en", "p4_wr_en", "kill_en", "nn_wr_en",
                "log_wr_en"):
        getattr(dut, sig).value = 0
    for sig in ("init_idx", "set_idx", "p4_wr_idx", "kill_idx", "nn_wr_idx",
                "log_wr_idx", "log_wr_val",
                "p4_rd_idx", "rd_off", "wb_off", "wb_en",
                "nn_wr_beam", "nn_wr_index", "nn_wr_geo", "nn_wr_log",
                "wb_nn_index", "wb_nn_geo",
                "init_e", "init_px", "init_py", "init_pz",
                "init_y", "init_phi", "init_wgt",
                "set_y", "set_phi", "set_wgt",
                "p4_wr_e", "p4_wr_px", "p4_wr_py", "p4_wr_pz"):
        getattr(dut, sig).value = 0
    dut.aresetn.value = 0
    for _ in range(5):
        await RisingEdge(dut.aclk)
    dut.aresetn.value = 1
    await RisingEdge(dut.aclk)


async def init_entry(dut, idx):
    p = payload(idx)
    dut.init_en.value = 1
    dut.init_idx.value = idx
    dut.init_y.value = p["y"]
    dut.init_phi.value = p["phi"]
    dut.init_wgt.value = p["wgt"]
    dut.init_e.value = p["e"]
    dut.init_px.value = p["px"]
    dut.init_py.value = p["py"]
    dut.init_pz.value = p["pz"]
    await RisingEdge(dut.aclk)
    dut.init_en.value = 0


async def read_offset(dut, off):
    """Present an offset and return the registered lane data one cycle later."""
    dut.rd_off.value = off
    await RisingEdge(dut.aclk)
    await RisingEdge(dut.aclk)
    return {
        "y": [lane(dut.rd_y.value, l, COORD_W) for l in range(LANES)],
        "phi": [lane(dut.rd_phi.value, l, COORD_W) for l in range(LANES)],
        "wgt": [lane(dut.rd_wgt.value, l, WGT_W) for l in range(LANES)],
        "active": [(int(dut.rd_active.value) >> l) & 1 for l in range(LANES)],
        "beam": [(int(dut.rd_beam.value) >> l) & 1 for l in range(LANES)],
        "nn_index": [lane(dut.rd_nn_index.value, l, IDX_W) for l in range(LANES)],
        "nn_geo": [lane(dut.rd_nn_geo.value, l, GEO_W) for l in range(LANES)],
        "nn_log": [lane(dut.rd_nn_log.value, l, NNLOG_W) for l in range(LANES)],
    }


async def read_p4(dut, idx):
    dut.p4_rd_idx.value = idx
    await RisingEdge(dut.aclk)
    await RisingEdge(dut.aclk)
    return (int(dut.p4_rd_e.value) & P4_MASK,
            int(dut.p4_rd_px.value) & P4_MASK,
            int(dut.p4_rd_py.value) & P4_MASK,
            int(dut.p4_rd_pz.value) & P4_MASK)


@cocotb.test()
async def test_banking_maps_index_to_lane(dut):
    """Fill all 128 entries by index, read them back by (offset, lane).

    This is the test that matters: it writes through the single-entry port
    and reads through the lane-parallel one, so the two address decodes must
    agree or nothing lines up.
    """
    await start_dut(dut)
    for idx in range(NMAX):
        await init_entry(dut, idx)

    for off in range(DEPTH):
        got = await read_offset(dut, off)
        for l in range(LANES):
            idx = off * LANES + l
            exp = payload(idx)
            assert got["y"][l] == exp["y"], (
                f"offset {off} lane {l} should hold entry {idx}: "
                f"y = {got['y'][l]:#x}, expected {exp['y']:#x}")
            assert got["phi"][l] == exp["phi"], f"phi at {idx}"
            assert got["wgt"][l] == exp["wgt"], f"wgt at {idx}"


@cocotb.test()
async def test_init_sets_no_neighbour_state(dut):
    """A fresh entry must be active, beam, and lose every later comparison.

    The initial all-pairs scan relies on this: nn_geo all ones and nn_log at
    its most positive means the first real candidate wins outright.
    """
    await start_dut(dut)
    for idx in range(NMAX):
        await init_entry(dut, idx)

    for off in range(DEPTH):
        got = await read_offset(dut, off)
        for l in range(LANES):
            idx = off * LANES + l
            assert got["active"][l] == 1, f"entry {idx} not active after init"
            assert got["beam"][l] == 1, f"entry {idx} not marked beam after init"
            assert got["nn_geo"][l] == GEO_MASK, (
                f"entry {idx} nn_geo = {got['nn_geo'][l]:#x}, expected all ones")
            assert got["nn_log"][l] == (NNLOG_MASK >> 1), (
                f"entry {idx} nn_log = {got['nn_log'][l]:#x}, "
                f"expected max positive {NNLOG_MASK >> 1:#x}")


@cocotb.test()
async def test_four_momentum_round_trip(dut):
    """The central four-momentum store is addressed by full index."""
    await start_dut(dut)
    for idx in range(NMAX):
        await init_entry(dut, idx)

    rng = random.Random(41)
    for idx in [0, 1, 15, 16, 127] + [rng.randrange(NMAX) for _ in range(20)]:
        p = payload(idx)
        got = await read_p4(dut, idx)
        assert got == (p["e"], p["px"], p["py"], p["pz"]), (
            f"entry {idx}: p4 {got} != {(p['e'], p['px'], p['py'], p['pz'])}")

    # An accumulate must land on the addressed entry and disturb no other.
    dut.p4_wr_en.value = 1
    dut.p4_wr_idx.value = 20
    dut.p4_wr_e.value = 0xAAAA
    dut.p4_wr_px.value = 0xBBBB
    dut.p4_wr_py.value = 0xCCCC
    dut.p4_wr_pz.value = 0xDDDD
    await RisingEdge(dut.aclk)
    dut.p4_wr_en.value = 0

    assert await read_p4(dut, 20) == (0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD)
    p = payload(21)
    assert await read_p4(dut, 21) == (p["e"], p["px"], p["py"], p["pz"]), \
        "the neighbouring entry was disturbed"


@cocotb.test()
async def test_lane_write_back_is_isolated(dut):
    """Lane l's write-back must touch entry off*LANES+l and nothing else.

    Conflict-freedom is the reason the sweep needs no arbitration, so this
    checks it directly: enable a scattered subset of lanes and confirm the
    disabled ones kept their previous contents.

    It also pins down what the write-back deliberately does NOT do: it leaves
    nn_dist_log alone. Storing w_k + log2(g) there would need a logarithm in
    every lane, so the value is left stale and refreshed later through
    log_wr_*, and this test fails if that ever changes silently.
    """
    await start_dut(dut)
    for idx in range(NMAX):
        await init_entry(dut, idx)

    off = 3
    enabled = [0, 1, 4, 9, 15]
    mask = sum(1 << l for l in enabled)
    stale_log = NNLOG_MASK >> 1          # what init left behind

    wb_index = wb_geo = 0
    for l in enabled:
        wb_index |= ((off * LANES + l) & IDX_MASK) << (l * IDX_W)
        wb_geo |= ((0x1234 + l) & GEO_MASK) << (l * GEO_W)

    dut.wb_off.value = off
    dut.wb_en.value = mask
    dut.wb_nn_index.value = wb_index
    dut.wb_nn_geo.value = wb_geo
    await RisingEdge(dut.aclk)
    dut.wb_en.value = 0

    got = await read_offset(dut, off)
    for l in range(LANES):
        if l in enabled:
            assert got["nn_index"][l] == (off * LANES + l) & IDX_MASK, \
                f"lane {l} nn_index"
            assert got["nn_geo"][l] == 0x1234 + l, f"lane {l} nn_geo"
            assert got["nn_log"][l] == stale_log, (
                f"lane {l} nn_log was written by the write-back; it must be "
                f"left for the shared log unit")
            assert got["beam"][l] == 0, \
                f"lane {l} still marked beam after being claimed"
            assert got["active"][l] == 1, f"lane {l} lost its active bit"
        else:
            assert got["nn_geo"][l] == GEO_MASK, (
                f"lane {l} was written despite wb_en low -- write-back is not "
                f"isolated")

    # Other offsets untouched.
    other = await read_offset(dut, 4)
    assert all(g == GEO_MASK for g in other["nn_geo"]), \
        "the write-back leaked into another offset"


@cocotb.test()
async def test_log_refresh_touches_only_that_field(dut):
    """The shared log unit writes nn_dist_log and nothing else."""
    await start_dut(dut)
    for idx in range(NMAX):
        await init_entry(dut, idx)

    idx = 45                          # bank 13, offset 2
    l, off = bank_of(idx), off_of(idx)

    # Give the row a neighbour first, so there is state to preserve.
    dut.nn_wr_en.value = 1
    dut.nn_wr_idx.value = idx
    dut.nn_wr_beam.value = 0
    dut.nn_wr_index.value = 88
    dut.nn_wr_geo.value = 0x2468
    dut.nn_wr_log.value = 0
    await RisingEdge(dut.aclk)
    dut.nn_wr_en.value = 0

    dut.log_wr_en.value = 1
    dut.log_wr_idx.value = idx
    dut.log_wr_val.value = 0x1BEEF
    await RisingEdge(dut.aclk)
    dut.log_wr_en.value = 0

    got = await read_offset(dut, off)
    assert got["nn_log"][l] == 0x1BEEF, f"nn_log = {got['nn_log'][l]:#x}"
    assert got["nn_index"][l] == 88, "the log refresh disturbed nn_index"
    assert got["nn_geo"][l] == 0x2468, "the log refresh disturbed nn_geo"
    assert got["active"][l] == 1 and got["beam"][l] == 0, \
        "the log refresh disturbed the row's own flags"
    nb = (l + 1) % LANES
    assert got["nn_geo"][nb] == GEO_MASK, "the log refresh hit another bank"


@cocotb.test()
async def test_single_entry_nn_write_and_kill(dut):
    """The scanned row writes its own result; kill clears only active."""
    await start_dut(dut)
    for idx in range(NMAX):
        await init_entry(dut, idx)

    dut.nn_wr_en.value = 1
    dut.nn_wr_idx.value = 37          # bank 5, offset 2
    dut.nn_wr_beam.value = 0
    dut.nn_wr_index.value = 99
    dut.nn_wr_geo.value = 0xABCDE
    dut.nn_wr_log.value = 0x13579
    await RisingEdge(dut.aclk)
    dut.nn_wr_en.value = 0

    got = await read_offset(dut, 2)
    assert got["nn_index"][5] == 99, f"nn_index = {got['nn_index'][5]}"
    assert got["nn_geo"][5] == 0xABCDE
    assert got["nn_log"][5] == 0x13579
    assert got["beam"][5] == 0
    assert got["active"][5] == 1
    assert got["nn_geo"][6] == GEO_MASK, "the adjacent lane was disturbed"

    dut.kill_en.value = 1
    dut.kill_idx.value = 37
    await RisingEdge(dut.aclk)
    dut.kill_en.value = 0

    got = await read_offset(dut, 2)
    assert got["active"][5] == 0, "kill did not clear active"
    assert got["nn_index"][5] == 99, "kill disturbed the neighbour fields"
    assert got["active"][6] == 1, "kill hit the wrong entry"


@cocotb.test()
async def test_setkin_updates_coordinates_only(dut):
    """A merge moves an entry's coordinates without touching its nn state."""
    await start_dut(dut)
    for idx in range(NMAX):
        await init_entry(dut, idx)

    idx = 70                          # bank 6, offset 4
    dut.set_en.value = 1
    dut.set_idx.value = idx
    dut.set_y.value = 0x0BADF00D
    dut.set_phi.value = 0x0C0FFEE0
    dut.set_wgt.value = 0x0DEADBEE
    await RisingEdge(dut.aclk)
    dut.set_en.value = 0

    got = await read_offset(dut, off_of(idx))
    l = bank_of(idx)
    assert got["y"][l] == 0x0BADF00D, f"y = {got['y'][l]:#x}"
    assert got["phi"][l] == 0x0C0FFEE0
    assert got["wgt"][l] == 0x0DEADBEE
    assert got["active"][l] == 1 and got["beam"][l] == 1, \
        "setkin disturbed the nn state"

    nb = (l + 1) % LANES
    exp = payload(off_of(idx) * LANES + nb)
    assert got["y"][nb] == exp["y"], "setkin hit the neighbouring bank"


@cocotb.test()
async def test_single_entry_write_beats_write_back(dut):
    """Priority is a definition, not an expectation -- pin it down.

    The controller keeps these in separate phases of a round, but if they
    ever do overlap the single-entry write must win, because it carries the
    scanned row's own freshly computed result.
    """
    await start_dut(dut)
    for idx in range(NMAX):
        await init_entry(dut, idx)

    idx = 21                          # bank 5, offset 1
    l, off = bank_of(idx), off_of(idx)

    dut.nn_wr_en.value = 1
    dut.nn_wr_idx.value = idx
    dut.nn_wr_beam.value = 0
    dut.nn_wr_index.value = 7
    dut.nn_wr_geo.value = 0x111
    dut.nn_wr_log.value = 0x222

    dut.wb_off.value = off
    dut.wb_en.value = 1 << l
    dut.wb_nn_index.value = 0x3F << (l * IDX_W)
    dut.wb_nn_geo.value = 0x999 << (l * GEO_W)

    await RisingEdge(dut.aclk)
    dut.nn_wr_en.value = 0
    dut.wb_en.value = 0

    got = await read_offset(dut, off)
    assert got["nn_geo"][l] == 0x111, (
        f"write-back won over the single-entry write: nn_geo = "
        f"{got['nn_geo'][l]:#x}")
    assert got["nn_index"][l] == 7
    assert got["nn_log"][l] == 0x222, (
        "the single-entry write carries a complete record including its own "
        f"nn_dist_log; got {got['nn_log'][l]:#x}")
