"""Cocotb tests for jc_sweep, wired to the real jc_mem.

Three modes share one pipeline, so each is checked against an independent
Python model over the same loaded active list:

    NN_SCAN   nearest active neighbour of a query row, plus the write-back
              that lets the query claim rows it is now nearer to
    ARGMIN    the active row with the smallest nn_dist_log
    MARK      active rows whose cached neighbour was one of a merged pair

The tie-break is the point of several of these. On a regular grid 2-6% of
row scans end in a bit-exact distance tie, so "smallest index wins" has to
hold in the lane tree AND across offsets, and jc_model.py fixes the same
convention. A test that only used random coordinates would almost never
exercise it, so ties are constructed deliberately.
"""

import json
import pathlib
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

LUTS = json.loads(
    (pathlib.Path(__file__).resolve().parents[3] / "model" / "luts.json")
    .read_text())

NMAX = 128
LANES = 16
DEPTH = NMAX // LANES
IDX_W = 7
COORD_W = 32
GEO_W = 49
NNLOG_W = 40

COORD_MASK = (1 << COORD_W) - 1
GEO_MASK = (1 << GEO_W) - 1
NNLOG_MASK = (1 << NNLOG_W) - 1

DELTA_W = LUTS["delta_w"]
SHIFT = LUTS["delta_shift"]
COORD_SCALE = LUTS["coord_scale"]
SAT_MAG = (1 << DELTA_W) - 1

MODE_NN_SCAN, MODE_ARGMIN, MODE_MARK = 0, 1, 2

R_RAD = 0.4
R2 = (int(round(R_RAD * COORD_SCALE)) >> SHIFT) ** 2


def to_signed(word, width):
    return word - (1 << width) if word >> (width - 1) else word


def geo(ya, pa, yb, pb):
    """dR^2 exactly as jc_dist computes it."""
    dy = to_signed(ya & COORD_MASK, COORD_W) - to_signed(yb & COORD_MASK, COORD_W)
    dphi = to_signed((pa - pb) & COORD_MASK, COORD_W)
    u = min(SAT_MAG, abs(dy) >> SHIFT)
    v = min(SAT_MAG, abs(dphi) >> SHIFT)
    return u * u + v * v


def lane(vec, l, width):
    return (int(vec) >> (l * width)) & ((1 << width) - 1)


class Model:
    """The active list as the bench believes it to be."""

    def __init__(self):
        self.y = {}
        self.phi = {}
        self.active = {}
        self.nn_index = {}
        self.nn_geo = {}
        self.beam = {}

    def add(self, idx, y, phi):
        self.y[idx] = y & COORD_MASK
        self.phi[idx] = phi & COORD_MASK
        self.active[idx] = True
        self.nn_index[idx] = 0
        self.nn_geo[idx] = GEO_MASK
        self.beam[idx] = True

    def nn_scan(self, q):
        """Nearest active neighbour of q, smallest index on a tie."""
        best_g, best_k = None, None
        for k in sorted(self.y):
            if k == q or not self.active[k]:
                continue
            g = geo(self.y[q], self.phi[q], self.y[k], self.phi[k])
            if best_g is None or g < best_g:
                best_g, best_k = g, k
        return best_g, best_k

    def claims(self, q):
        """Rows q would take over during its own scan."""
        out = set()
        for k in sorted(self.y):
            if k == q or not self.active[k]:
                continue
            g = geo(self.y[q], self.phi[q], self.y[k], self.phi[k])
            if g < R2 and (g < self.nn_geo[k]
                           or (g == self.nn_geo[k] and q < self.nn_index[k])):
                out.add(k)
        return out


async def start_dut(dut):
    cocotb.start_soon(Clock(dut.aclk, 4, units="ns").start())
    for sig in ("init_en", "kill_en", "nn_wr_en", "start"):
        getattr(dut, sig).value = 0
    for sig in ("init_idx", "init_e", "init_px", "init_py", "init_pz",
                "init_y", "init_phi", "init_wgt", "kill_idx",
                "nn_wr_idx", "nn_wr_beam", "nn_wr_index", "nn_wr_geo",
                "nn_wr_log", "mode", "query_idx", "query_y", "query_phi",
                "mark_a", "mark_b", "obs_off"):
        getattr(dut, sig).value = 0
    dut.r_squared.value = R2
    dut.aresetn.value = 0
    for _ in range(5):
        await RisingEdge(dut.aclk)
    dut.aresetn.value = 1
    await RisingEdge(dut.aclk)


async def load(dut, model, entries):
    """entries: {idx: (y, phi)}"""
    for idx, (y, phi) in sorted(entries.items()):
        dut.init_en.value = 1
        dut.init_idx.value = idx
        dut.init_y.value = y & COORD_MASK
        dut.init_phi.value = phi & COORD_MASK
        dut.init_wgt.value = 0
        dut.init_e.value = 0
        dut.init_px.value = 0
        dut.init_py.value = 0
        dut.init_pz.value = 0
        await RisingEdge(dut.aclk)
        model.add(idx, y, phi)
    dut.init_en.value = 0
    await RisingEdge(dut.aclk)


async def set_nn(dut, model, idx, nn_index, nn_geo, nn_log=0, beam=0):
    dut.nn_wr_en.value = 1
    dut.nn_wr_idx.value = idx
    dut.nn_wr_beam.value = beam
    dut.nn_wr_index.value = nn_index
    dut.nn_wr_geo.value = nn_geo
    dut.nn_wr_log.value = nn_log & NNLOG_MASK
    await RisingEdge(dut.aclk)
    dut.nn_wr_en.value = 0
    await RisingEdge(dut.aclk)
    model.nn_index[idx] = nn_index
    model.nn_geo[idx] = nn_geo
    model.beam[idx] = bool(beam)


async def run_sweep(dut, mode, query_idx=0, query_y=0, query_phi=0,
                    mark_a=0, mark_b=0):
    dut.mode.value = mode
    dut.query_idx.value = query_idx
    dut.query_y.value = query_y & COORD_MASK
    dut.query_phi.value = query_phi & COORD_MASK
    dut.mark_a.value = mark_a
    dut.mark_b.value = mark_b
    dut.start.value = 1
    await RisingEdge(dut.aclk)
    dut.start.value = 0
    for _ in range(200):
        await RisingEdge(dut.aclk)
        if dut.done.value:
            break
    else:
        raise AssertionError("sweep never asserted done")
    return {
        "valid": bool(dut.result_valid.value),
        "index": int(dut.result_index.value),
        "geo": int(dut.result_geo.value),
        "beam": bool(dut.result_beam.value),
        "log": to_signed(int(dut.result_log.value) & NNLOG_MASK, NNLOG_W),
        "claimed": int(dut.claimed_mask.value),
        "stale": int(dut.stale_mask.value),
    }


def mask_to_set(m):
    return {i for i in range(NMAX) if (m >> i) & 1}


# A coordinate that is a whole number of delta LSBs, so constructed ties are
# exact rather than nearly exact.
def coord(n_lsb):
    return (n_lsb << SHIFT) & COORD_MASK


@cocotb.test()
async def test_nn_scan_finds_nearest(dut):
    """Randomised active list; the scan must match the Python model exactly."""
    await start_dut(dut)
    rng = random.Random(51)
    model = Model()
    entries = {i: (coord(rng.randrange(-20000, 20000)),
                   coord(rng.randrange(0, 40000)))
               for i in range(NMAX)}
    await load(dut, model, entries)

    for q in (0, 1, 15, 16, 63, 127):
        got = await run_sweep(dut, MODE_NN_SCAN, query_idx=q,
                              query_y=entries[q][0], query_phi=entries[q][1])
        exp_g, exp_k = model.nn_scan(q)
        assert got["valid"], f"query {q}: no candidate found"
        assert got["geo"] == exp_g, (
            f"query {q}: geo {got['geo']} != {exp_g}")
        assert got["index"] == exp_k, (
            f"query {q}: nearest {got['index']} != {exp_k}")
        assert got["beam"] == (exp_g >= R2), f"query {q}: beam flag"


@cocotb.test()
async def test_nn_scan_tie_takes_smallest_index(dut):
    """Exact distance ties must resolve to the lowest index, every time.

    Placed deliberately: candidates equidistant from the query in each of the
    four reflections, spread across banks AND offsets so both the lane tree
    and the cross-offset accumulator have to break a tie.
    """
    await start_dut(dut)
    model = Model()
    qy, qp = coord(0), coord(10000)
    d = coord(500)

    # Query at 0; four candidates at exactly +/-d in y and phi.
    entries = {0: (qy, qp)}
    tied = [3, 20, 37, 70]              # banks 3,4,5,6 and offsets 0,1,2,4
    offsets = [(d, 0), (-d, 0), (0, d), (0, -d)]
    for idx, (dy, dp) in zip(tied, offsets):
        entries[idx] = ((qy + dy) & COORD_MASK, (qp + dp) & COORD_MASK)
    # A strictly further candidate, to prove the tie set is what won.
    entries[9] = ((qy + coord(4000)) & COORD_MASK, qp)
    await load(dut, model, entries)

    got = await run_sweep(dut, MODE_NN_SCAN, query_idx=0,
                          query_y=qy, query_phi=qp)
    exp_g, exp_k = model.nn_scan(0)
    assert exp_k == min(tied), "model itself did not pick the smallest index"
    assert got["index"] == min(tied), (
        f"tie went to {got['index']}, expected {min(tied)}")
    assert got["geo"] == exp_g


@cocotb.test()
async def test_nn_scan_skips_self_and_inactive(dut):
    """The query must not match itself, and killed rows must not compete."""
    await start_dut(dut)
    model = Model()
    qy, qp = coord(0), coord(0)
    entries = {0: (qy, qp),
               5: (coord(100), coord(0)),      # nearest, will be killed
               33: (coord(300), coord(0)),
               64: (coord(900), coord(0))}
    await load(dut, model, entries)

    got = await run_sweep(dut, MODE_NN_SCAN, query_idx=0,
                          query_y=qy, query_phi=qp)
    assert got["index"] == 5, f"expected 5, got {got['index']}"
    assert got["geo"] != 0, "the query matched itself"

    dut.kill_en.value = 1
    dut.kill_idx.value = 5
    await RisingEdge(dut.aclk)
    dut.kill_en.value = 0
    model.active[5] = False
    await RisingEdge(dut.aclk)

    got = await run_sweep(dut, MODE_NN_SCAN, query_idx=0,
                          query_y=qy, query_phi=qp)
    assert got["index"] == 33, (
        f"a killed row still competed: got {got['index']}")


@cocotb.test()
async def test_write_back_claims_only_inside_r(dut):
    """The query claims rows it beats -- and only rows strictly inside R.

    Clamping to R^2 before comparing would let a query BEYOND R capture a
    row, which then merges on its turn instead of being emitted as a jet.
    That was a real bug in jc_model.py, so it is pinned here.
    """
    await start_dut(dut)
    model = Model()
    r_lsb = int(round(R_RAD * COORD_SCALE)) >> SHIFT

    qy, qp = coord(0), coord(0)
    entries = {
        0: (qy, qp),
        1: (coord(r_lsb // 4), qp),        # well inside R  -> claimed
        2: (coord(r_lsb // 2), qp),        # inside R       -> claimed
        3: (coord(r_lsb * 2), qp),         # beyond R       -> NOT claimed
        4: (coord(r_lsb * 4), qp),         # far beyond R   -> NOT claimed
    }
    await load(dut, model, entries)

    got = await run_sweep(dut, MODE_NN_SCAN, query_idx=0,
                          query_y=qy, query_phi=qp)
    claimed = mask_to_set(got["claimed"])
    assert claimed == model.claims(0), (
        f"claimed {sorted(claimed)}, model says {sorted(model.claims(0))}")
    assert 3 not in claimed and 4 not in claimed, (
        "a row beyond R was claimed -- the R test is clamping, not comparing")

    # And the claim must actually be in memory.
    dut.obs_off.value = 0
    await RisingEdge(dut.aclk)
    await RisingEdge(dut.aclk)
    for k in (1, 2):
        assert lane(dut.obs_nn_index.value, k, IDX_W) == 0, \
            f"row {k} does not point at the query"
        assert ((int(dut.obs_beam.value) >> k) & 1) == 0, \
            f"row {k} is still flagged beam after being claimed"
    for k in (3, 4):
        assert lane(dut.obs_nn_geo.value, k, GEO_W) == GEO_MASK, \
            f"row {k} was written despite being outside R"


@cocotb.test()
async def test_write_back_respects_an_existing_better_neighbour(dut):
    """A row already closer to someone else keeps that neighbour."""
    await start_dut(dut)
    model = Model()
    qy, qp = coord(0), coord(0)
    entries = {0: (qy, qp), 1: (coord(2000), qp), 2: (coord(3000), qp)}
    await load(dut, model, entries)

    g01 = geo(qy, qp, entries[1][0], entries[1][1])
    # Row 1 already has a strictly better neighbour than the query offers.
    await set_nn(dut, model, 1, nn_index=2, nn_geo=g01 - 1, beam=0)
    # Row 2's cached distance is worse, so the query should take it.
    g02 = geo(qy, qp, entries[2][0], entries[2][1])
    await set_nn(dut, model, 2, nn_index=1, nn_geo=g02 + 1, beam=0)

    got = await run_sweep(dut, MODE_NN_SCAN, query_idx=0,
                          query_y=qy, query_phi=qp)
    claimed = mask_to_set(got["claimed"])
    assert claimed == model.claims(0), (
        f"claimed {sorted(claimed)}, model says {sorted(model.claims(0))}")
    assert 1 not in claimed, "a row with a better neighbour was stolen"
    assert 2 in claimed, "a row with a worse neighbour was not claimed"


@cocotb.test()
async def test_argmin_over_nn_dist_log(dut):
    """ARGMIN ranks a signed log; smallest wins, ties to the lowest index."""
    await start_dut(dut)
    model = Model()
    entries = {i: (coord(i * 100), coord(0)) for i in range(NMAX)}
    await load(dut, model, entries)

    rng = random.Random(52)
    logs = {}
    for i in range(NMAX):
        v = rng.randrange(-(1 << 30), 1 << 30)
        logs[i] = v
        await set_nn(dut, model, i, nn_index=0, nn_geo=1, nn_log=v, beam=0)

    got = await run_sweep(dut, MODE_ARGMIN)
    best = min(logs, key=lambda i: (logs[i], i))
    assert got["index"] == best, (
        f"argmin picked {got['index']} (log {logs[got['index']]}), "
        f"expected {best} (log {logs[best]})")
    assert got["log"] == logs[best], f"log {got['log']} != {logs[best]}"

    # A deliberate tie between two rows must go to the smaller index.
    lo = min(logs.values()) - 5
    await set_nn(dut, model, 100, nn_index=0, nn_geo=1, nn_log=lo, beam=0)
    await set_nn(dut, model, 42, nn_index=0, nn_geo=1, nn_log=lo, beam=0)
    got = await run_sweep(dut, MODE_ARGMIN)
    assert got["index"] == 42, f"tie went to {got['index']}, expected 42"


@cocotb.test()
async def test_argmin_ignores_inactive(dut):
    """A killed row must not win, however small its stale log."""
    await start_dut(dut)
    model = Model()
    entries = {i: (coord(i * 100), coord(0)) for i in range(32)}
    await load(dut, model, entries)

    for i in range(32):
        await set_nn(dut, model, i, nn_index=0, nn_geo=1,
                     nn_log=1000 + i, beam=0)
    await set_nn(dut, model, 7, nn_index=0, nn_geo=1, nn_log=-9999, beam=0)

    got = await run_sweep(dut, MODE_ARGMIN)
    assert got["index"] == 7

    dut.kill_en.value = 1
    dut.kill_idx.value = 7
    await RisingEdge(dut.aclk)
    dut.kill_en.value = 0
    await RisingEdge(dut.aclk)

    got = await run_sweep(dut, MODE_ARGMIN)
    assert got["index"] == 0, (
        f"an inactive row won the argmin: {got['index']}")


@cocotb.test()
async def test_mark_finds_rows_pointing_at_the_merged_pair(dut):
    """MARK flags exactly the rows whose neighbour was one of the pair."""
    await start_dut(dut)
    model = Model()
    entries = {i: (coord(i * 100), coord(0)) for i in range(NMAX)}
    await load(dut, model, entries)

    rng = random.Random(53)
    pointing = {}
    for i in range(NMAX):
        tgt = rng.randrange(NMAX)
        pointing[i] = tgt
        await set_nn(dut, model, i, nn_index=tgt, nn_geo=1, beam=0)

    a, b = 11, 90
    got = await run_sweep(dut, MODE_MARK, mark_a=a, mark_b=b)
    exp = {i for i, t in pointing.items() if t in (a, b)}
    assert mask_to_set(got["stale"]) == exp, (
        f"stale {sorted(mask_to_set(got['stale']))} != {sorted(exp)}")
    assert got["claimed"] == 0, "MARK must not claim anything"


@cocotb.test()
async def test_modes_do_not_leak_into_each_other(dut):
    """Each mode leaves the other modes' outputs alone."""
    await start_dut(dut)
    model = Model()
    entries = {i: (coord(i * 50), coord(0)) for i in range(NMAX)}
    await load(dut, model, entries)
    for i in range(NMAX):
        await set_nn(dut, model, i, nn_index=(i + 1) % NMAX, nn_geo=1000 + i,
                     nn_log=i, beam=0)

    got = await run_sweep(dut, MODE_ARGMIN)
    assert got["claimed"] == 0, "ARGMIN wrote the claimed mask"
    assert got["stale"] == 0, "ARGMIN wrote the stale mask"

    got = await run_sweep(dut, MODE_MARK, mark_a=1, mark_b=2)
    assert got["claimed"] == 0, "MARK wrote the claimed mask"

    got = await run_sweep(dut, MODE_NN_SCAN, query_idx=0,
                          query_y=entries[0][0], query_phi=entries[0][1])
    assert got["stale"] == 0, "NN_SCAN wrote the stale mask"
