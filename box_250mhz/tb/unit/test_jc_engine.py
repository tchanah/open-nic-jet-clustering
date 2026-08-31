"""Cocotb tests for jc_engine -- a whole event, clustered.

The bench plays jc_evbuf: it presents an event, answers ev_addr with the
pseudojet record one cycle later (matching jc_evbuf's registered read port),
and collects the jets.

Expected jets come from model/jc_model.py clustering the SAME pseudojets, so
this compares the RTL's control flow against a reference that is itself
bit-identical to FastJet on the float path.

THE JETS ARE COMPARED BIT-EXACTLY; THE ROUTE TO THEM IS NOT. jc_model's
set_kin is still float64 while the RTL uses jc_setkin's fixed point, and the
two agree to ~5e-9 rad rather than exactly. That cannot show up in a jet:
merges are exact integer adds, so a jet is the exact sum of its constituent
cells, and setkin's precision reaches the output only through the merge
ORDERING it influences. So the two sides either agree on which cells group
together -- in which case every four-momentum matches to the bit -- or they
do not, in which case the partition itself differs and no tolerance would
have been the right answer anyway.

The real-event tests therefore assert exact equality of the four-momenta,
which is the claim CLAUDE.md makes. A failure there means the model's float
set_kin reordered a merge, not that the arithmetic drifted; replacing it with
a mirror of jc_log2 + jc_cordic is what would close that last gap. The
tolerance checks are kept ahead of it because they fail more legibly.

Comparison on the small synthetic events stays tolerance-based: they exist to
catch control bugs, which move jets by O(1), not by 1e-9.

Events are built small and synthetic before anything real is tried: a
128-cell event is ~15k cycles of simulation, and a control bug found on four
cells is a bug you can still read a waveform for.
"""

import json
import math
import pathlib
import random
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "model"))
import jc_model as M                                    # noqa: E402

FMT = M.Formats(ROOT / "model" / "luts.json")
LUTS = json.loads((ROOT / "model" / "luts.json").read_text())

P4_W = FMT.p4_w
P4_FRAC = FMT.p4_frac
COORD_W = FMT.coord_w
WGT_W = FMT.wgt_w
COORD_SCALE = FMT.coord_scale
P4_MASK = (1 << P4_W) - 1
COORD_MASK = (1 << COORD_W) - 1
WGT_MASK = (1 << WGT_W) - 1

R_RAD = 0.4
R2 = FMT.r_squared(R_RAD)

# Jet floor as pt^2 in Q28.68, the form jc_ctrl compares against.
def pt_sq_floor(gev):
    return int(round(gev * gev * (1 << (2 * P4_FRAC))))


def to_signed(word, width):
    return word - (1 << width) if word >> (width - 1) else word


async def start_dut(dut, pt_min_gev):
    cocotb.start_soon(Clock(dut.aclk, 4, units="ns").start())
    dut.ev_valid.value = 0
    dut.ev_count.value = 0
    dut.ev_seq.value = 0
    for s in ("ev_energy", "ev_px", "ev_py", "ev_pz",
              "ev_rapidity", "ev_phi", "ev_beam_weight_log"):
        getattr(dut, s).value = 0
    dut.cfg_r_squared.value = R2
    dut.cfg_pt_sq_floor.value = pt_sq_floor(pt_min_gev)
    dut.jet_ready.value = 1
    dut.aresetn.value = 0
    for _ in range(5):
        await RisingEdge(dut.aclk)
    dut.aresetn.value = 1
    await RisingEdge(dut.aclk)


async def evbuf(dut, jets, seq, done_flag):
    """Stand in for jc_evbuf: offer the event, serve reads, take the release.

    The record must appear one cycle after ev_addr, because jc_ctrl is
    written against jc_evbuf's registered read port. Answering combinationally
    would let the RTL pass here and fail against the real buffer.
    """
    dut.ev_valid.value = 1
    dut.ev_count.value = len(jets)
    dut.ev_seq.value = seq

    while True:
        await RisingEdge(dut.aclk)
        # ev_addr is X until jc_ctrl first drives it; a real jc_evbuf would
        # simply return whatever that address held.
        raw = dut.ev_addr.value
        addr = int(raw) if raw.is_resolvable else 0
        if addr < len(jets):
            j = jets[addr]
            dut.ev_energy.value = j.e & P4_MASK
            dut.ev_px.value = j.px & P4_MASK
            dut.ev_py.value = j.py & P4_MASK
            dut.ev_pz.value = j.pz & P4_MASK
            dut.ev_rapidity.value = j.y & COORD_MASK
            dut.ev_phi.value = j.phi & COORD_MASK
            dut.ev_beam_weight_log.value = j.wgt & WGT_MASK
        if dut.ev_accept.value:
            dut.ev_valid.value = 0
        if dut.ev_release.value:
            pass
        if done_flag[0]:
            return


async def run_event(dut, cells, seq=1, limit=400000):
    """Cluster one event; return the jets as float four-momenta."""
    jets_in = M.ingest(cells, FMT)
    done = [False]
    cocotb.start_soon(evbuf(dut, jets_in, seq, done))

    out = []
    for _ in range(limit):
        await RisingEdge(dut.aclk)
        if dut.jet_valid.value and dut.jet_ready.value:
            out.append((
                to_signed(int(dut.jet_e.value), P4_W) / (1 << P4_FRAC),
                to_signed(int(dut.jet_px.value), P4_W) / (1 << P4_FRAC),
                to_signed(int(dut.jet_py.value), P4_W) / (1 << P4_FRAC),
                to_signed(int(dut.jet_pz.value), P4_W) / (1 << P4_FRAC)))
        if dut.jet_eoe.value:
            done[0] = True
            return out, int(dut.cycle_count.value)
    done[0] = True
    raise AssertionError(f"engine never finished (>{limit} cycles)")


def cell(iy, iphi, ecode):
    return (iy, iphi, ecode)


def compare(got, want, label, tol=1e-6):
    """Pair jets by proximity and check pt; report count first."""
    assert len(got) == len(want), (
        f"{label}: {len(got)} jets, model says {len(want)}\n"
        f"  got  {[f'{math.hypot(j[1], j[2]):.4f}' for j in sorted(got)]}\n"
        f"  want {[f'{math.hypot(j[1], j[2]):.4f}' for j in sorted(want)]}")
    g = sorted(math.hypot(j[1], j[2]) for j in got)
    w = sorted(math.hypot(j[1], j[2]) for j in want)
    for a, b in zip(g, w):
        assert abs(a - b) <= tol * max(b, 1.0), (
            f"{label}: jet pt {a:.9f} vs model {b:.9f}")


def assert_bit_exact(got, want, label):
    """Every four-momentum identical, not merely close. See the module docstring.

    Both sides are 48-bit integers divided by 2^P4_FRAC, and 48 bits fit a
    float64 mantissa, so this equality is the integer equality it looks like.
    Jets leave in completion order, which is not the model's order, so the
    comparison is over the sorted multiset.
    """
    bad = [(a, b) for a, b in zip(sorted(got), sorted(want)) if a != b]
    assert not bad, (
        f"{label}: {len(bad)} of {len(got)} jets differ from the model.\n"
        f"  A merge was ordered differently -- the four-momenta themselves\n"
        f"  carry no rounding. First: {bad[0][0]} vs {bad[0][1]}")


@cocotb.test()
async def test_single_cell(dut):
    """One cell in, one jet out. The smallest thing that exercises a round."""
    await start_dut(dut, 0.0)
    cells = [cell(25, 10, 3000)]
    got, cycles = await run_event(dut, cells)
    want = M.cluster_fixed(cells, FMT, R_RAD, 0.0)
    dut._log.info("1 cell: %d jets in %d cycles", len(got), cycles)
    compare(got, want, "single cell")


@cocotb.test()
async def test_two_cells_far_apart(dut):
    """Beyond R: no merge, two jets. Exercises the beam path twice."""
    await start_dut(dut, 0.0)
    cells = [cell(10, 5, 3000), cell(40, 40, 3200)]
    got, cycles = await run_event(dut, cells)
    want = M.cluster_fixed(cells, FMT, R_RAD, 0.0)
    dut._log.info("2 far cells: %d jets in %d cycles", len(got), cycles)
    compare(got, want, "two far cells")


@cocotb.test()
async def test_two_cells_close(dut):
    """Inside R: one merge, then one jet. The first real merge path."""
    await start_dut(dut, 0.0)
    cells = [cell(25, 10, 3000), cell(26, 10, 2900)]
    got, cycles = await run_event(dut, cells)
    want = M.cluster_fixed(cells, FMT, R_RAD, 0.0)
    dut._log.info("2 close cells: %d jets in %d cycles", len(got), cycles)
    compare(got, want, "two close cells")


@cocotb.test()
async def test_small_cluster(dut):
    """A handful of cells around one seed, plus an isolated one.

    Small enough to read a waveform, large enough to exercise the stale-row
    rescans and the write-back claims that the two-cell cases cannot.
    """
    await start_dut(dut, 0.0)
    cells = [cell(25, 10, 3200), cell(26, 10, 3000), cell(25, 11, 2900),
             cell(27, 11, 2800), cell(40, 40, 3100)]
    got, cycles = await run_event(dut, cells)
    want = M.cluster_fixed(cells, FMT, R_RAD, 0.0)
    dut._log.info("5 cells: %d jets in %d cycles", len(got), cycles)
    compare(got, want, "small cluster")


@cocotb.test()
async def test_randomised_small_events(dut):
    """Randomised, still small. Catches ordering the hand cases do not."""
    await start_dut(dut, 0.0)
    rng = random.Random(91)
    for trial in range(6):
        n = rng.randrange(3, 10)
        cells = [cell(rng.randrange(50), rng.randrange(64),
                      rng.randrange(2000, 4000)) for _ in range(n)]
        got, cycles = await run_event(dut, cells, seq=trial)
        want = M.cluster_fixed(cells, FMT, R_RAD, 0.0)
        dut._log.info("trial %d: n=%d -> %d jets in %d cycles",
                      trial, n, len(got), cycles)
        compare(got, want, f"trial {trial} n={n}")


@cocotb.test()
async def test_jet_floor_suppresses_output(dut):
    """Below the floor a row is still removed, just not reported."""
    # Q28.68 tops out at pt ~16 TeV, so this is "nothing can pass" without
    # overflowing the port.
    await start_dut(dut, 1.0e4)
    cells = [cell(25, 10, 3000), cell(40, 40, 3200)]
    got, cycles = await run_event(dut, cells)
    assert got == [], f"floor ignored: {len(got)} jets emitted"
    dut._log.info("floor test finished in %d cycles", cycles)


# The committed fixture first, the full dataset only as a fallback. A missing
# file used to raise TestSuccess, which is a PASS -- so on any machine without
# /scratch the headline result of step 7 silently evaporated into green. It is
# an error now, and model/make_fixture.py is how you fix it.
FIXTURE = pathlib.Path(__file__).resolve().parent / "data" / "events.pkt.bin"
PKT = pathlib.Path("/scratch/chettige/cells1k.pkt.bin")

NO_DATA = (
    f"no real-event data.\n"
    f"  looked for {FIXTURE}\n"
    f"           and {PKT}\n"
    f"  build the fixture with:\n"
    f"    python3 model/pixelize.py   # if the dataset is missing too\n"
    f"    python3 model/make_fixture.py {PKT} {FIXTURE}")


def real_events(want_sizes, limit=400):
    """Pull events out of the pixelised dataset, one per requested size band.

    Real events, not synthetic ones: the cell positions come from actual
    Pythia showers, so the distance distribution and therefore the stale-row
    count per merge are what the hardware will really see.
    """
    src = FIXTURE if FIXTURE.exists() else PKT
    picked = {}
    if not src.exists():
        return []
    for i, (seq, _total, cells) in enumerate(M.read_packets(src)):
        if i >= limit:
            break
        for lo, hi in want_sizes:
            if lo <= len(cells) <= hi and (lo, hi) not in picked:
                picked[(lo, hi)] = (seq, cells)
        if len(picked) == len(want_sizes):
            break
    return [picked[k] for k in want_sizes if k in picked]


def all_events(limit=400):
    """Every event in whichever source is present, in file order."""
    src = FIXTURE if FIXTURE.exists() else PKT
    if not src.exists():
        return []
    out = []
    for i, (seq, _total, cells) in enumerate(M.read_packets(src)):
        if i >= limit:
            break
        out.append((seq, cells))
    return out


@cocotb.test()
async def test_real_events_full_scale(dut):
    """Real pixelised events, up to the full 128 cells.

    Everything above is <= 8 cells. The stale-row count per merge grows with
    N, so the repair paths that dominate a real event are precisely the ones
    the small cases barely touch -- and this is the first honest cycles/event
    number, which is what step 9 exists to confirm on hardware.
    """
    await start_dut(dut, 0.0)
    events = real_events([(20, 40), (60, 80), (100, 128)])
    assert events, NO_DATA

    for seq, cells in events:
        got, cycles = await run_event(dut, cells, seq=seq)
        want = M.cluster_fixed(cells, FMT, R_RAD, 0.0)
        worst, unmatched = M.match(got, want)
        dut._log.info(
            "seq %d: n=%d -> %d jets in %d cycles (%.1f/cell), "
            "worst rel pt %.2e, unmatched %d",
            seq, len(cells), len(got), cycles, cycles / len(cells),
            worst, unmatched)
        assert len(got) == len(want), (
            f"seq {seq} n={len(cells)}: {len(got)} jets, model says {len(want)}")
        assert unmatched == 0, f"seq {seq}: {unmatched} jets did not pair up"
        assert worst < 1e-6, f"seq {seq}: worst relative pt {worst:.3e}"
        assert_bit_exact(got, want, f"seq {seq} n={len(cells)}")


@cocotb.test()
async def test_real_event_at_the_trigger_floor(dut):
    """The same clustering, reported at the 50 GeV floor it ships with.

    Only the jets that would actually leave the card, which is the comparison
    the physics side cares about.

    The event is chosen for HAVING a jet above the floor, not for its size.
    This sample is soft -- only 3 of its first 400 events carry a jet over 50
    GeV, and the hardest anywhere in them is 65 GeV -- so selecting by size
    band handed this test an event whose hardest jet was 12 GeV, and it passed
    by asserting that no jets equalled no jets. model/make_fixture.py puts a
    qualifying event in the fixture precisely so this cannot happen again.
    """
    await start_dut(dut, 50.0)
    events = all_events()
    assert events, NO_DATA

    picked = None
    for seq, cells in events:
        want = M.cluster_fixed(cells, FMT, R_RAD, 50.0)
        if want:
            picked = (seq, cells, want)
            break
    assert picked is not None, (
        "no event in the fixture has a jet above 50 GeV, so this test would "
        "prove nothing. Rebuild it with model/make_fixture.py, which selects "
        "one on purpose.")

    seq, cells, want = picked
    got, cycles = await run_event(dut, cells, seq=seq)
    worst, unmatched = M.match(got, want)
    dut._log.info("seq %d at 50 GeV: n=%d -> %d jets, worst %.2e, unmatched %d",
                  seq, len(cells), len(got), worst, unmatched)
    assert got, "the event was chosen for clearing the floor but emitted nothing"
    assert len(got) == len(want) and unmatched == 0
    assert worst < 1e-6
    assert_bit_exact(got, want, f"seq {seq} at 50 GeV")


@cocotb.test()
async def test_back_to_back_events(dut):
    """State must not leak between events -- active bits, masks, counters."""
    await start_dut(dut, 0.0)
    a = [cell(25, 10, 3000), cell(26, 10, 2900)]
    b = [cell(5, 50, 3300), cell(45, 20, 3100), cell(45, 21, 3000)]

    got_a, _ = await run_event(dut, a, seq=10)
    compare(got_a, M.cluster_fixed(a, FMT, R_RAD, 0.0), "event A")

    got_b, _ = await run_event(dut, b, seq=11)
    compare(got_b, M.cluster_fixed(b, FMT, R_RAD, 0.0), "event B")

    assert int(dut.event_count.value) == 2, (
        f"event_count = {int(dut.event_count.value)}")
