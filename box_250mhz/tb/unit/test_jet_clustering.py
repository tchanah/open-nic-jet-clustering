"""Cocotb tests for jet_clustering -- packets in, jets out, the whole chain.

THIS IS THE TEST STEP 8 EXISTS FOR. Every module below has its own bench, and
every one of those benches stands in for the module's neighbours -- encoding
the same assumptions the RTL does. A misunderstanding shared by a module and
its bench survives all of them. Running jc_deframe -> jc_ingest -> jc_evbuf ->
jc_engine -> jc_reframe as one thing, against packets built from the wire
format and jets checked against jc_model.py, is what can catch it.

The comparison is BIT-EXACT, not tolerance-based, in both directions:

  * the input bytes are the same bytes model/pixelize.py writes, so the model
    and the RTL cluster identical cells;
  * jets leave as fp32, and the model's four-momenta are integers divided by
    2^34, so the expected frame payload is a plain float32 round-trip of the
    model's answer. Any difference is a real disagreement, not rounding.

AXI-Lite is exercised here too rather than in isolation, because the thing
worth testing is not that a register holds a value -- it is that writing R
through the 125 MHz bus changes what the 250 MHz datapath does, across the
clock crossing, and that the counters read back consistently.
"""

import json
import pathlib
import struct
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "model"))
import jc_model as M                                    # noqa: E402
from jc_frames import (build_event_frame, to_beats, fp32,   # noqa: E402
                       expected_jets, parse_jets_frame,
                       ETH_TYPE_IN, ETH_TYPE_OUT)

FMT = M.Formats(ROOT / "model" / "luts.json")
LUTS = json.loads((ROOT / "model" / "luts.json").read_text())

P4_FRAC = FMT.p4_frac

# The reset values gen_luts.py baked into jc_consts.vh. Read, not repeated, so
# regenerating with a different default cannot leave this asserting the old.
DEFAULT_R = LUTS["default_r"]
DEFAULT_FLOOR = LUTS["default_pt_floor"]

# jc_regs map, byte offsets.
REG_ID = 0x00
REG_SCRATCH = 0x04
REG_STATUS = 0x08
REG_RSQ_LO, REG_RSQ_HI = 0x10, 0x14
REG_FLR_0, REG_FLR_1, REG_FLR_2 = 0x18, 0x1C, 0x20
REG_FRAMES_IN = 0x40
REG_BAD_HEADER = 0x44
REG_BAD_LENGTH = 0x48
REG_ACCEPT = 0x4C
REG_DROP_FULL = 0x50
REG_DROP_ERR = 0x54
REG_EVENTS = 0x58
REG_JETS_OUT = 0x5C
REG_FRAMES_OUT = 0x60
REG_SUPPRESSED = 0x64
REG_LAST_CYCLES = 0x68


# Frame building and parsing live in model/jc_frames.py, shared with the
# p2p-level bench so the two cannot drift.
build_frame = build_event_frame


# ------------------------------------------------------------------ driving --
async def start_dut(dut):
    cocotb.start_soon(Clock(dut.aclk, 4, units="ns").start())
    cocotb.start_soon(Clock(dut.axil_aclk, 8, units="ns").start())
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tdata.value = 0
    dut.s_axis_tkeep.value = 0
    dut.s_axis_tlast.value = 0
    dut.s_axis_tuser.value = 0
    dut.m_axis_tready.value = 0
    for s in ("s_axil_awvalid", "s_axil_wvalid", "s_axil_bready",
              "s_axil_arvalid", "s_axil_rready"):
        getattr(dut, s).value = 0
    dut.s_axil_awaddr.value = 0
    dut.s_axil_wdata.value = 0
    dut.s_axil_araddr.value = 0
    dut.aresetn.value = 0
    dut.axil_aresetn.value = 0
    for _ in range(10):
        await RisingEdge(dut.axil_aclk)
    dut.aresetn.value = 1
    dut.axil_aresetn.value = 1
    for _ in range(5):
        await RisingEdge(dut.axil_aclk)


# Every handshake wait below is bounded. An unbounded `while True` on tready
# does not fail when the datapath stalls -- vvp keeps advancing time at 100%
# CPU and the run is indistinguishable from a slow compile until someone
# checks `ps -eo pid,etime,pcpu`. These bounds are generous against the ~16k
# cycles an event takes; they exist to turn a hang into a named failure.
HANDSHAKE_LIMIT = 100_000


async def wait_for(dut, clk, cond, what):
    for _ in range(HANDSHAKE_LIMIT):
        await RisingEdge(clk)
        if cond():
            return
    raise AssertionError(
        f"{what} never asserted in {HANDSHAKE_LIMIT} cycles -- the handshake "
        f"is stalled, not slow")


async def send_frame(dut, frame):
    # ALIGN TO aclk BEFORE DRIVING ANYTHING. Without this the first beat is
    # silently lost: cocotb applies a signal write at the next ReadWrite phase,
    # and if this coroutine is entered past that point in the timestep, the
    # write lands AFTER the edge the DUT samples. tready is high anyway, so the
    # loop below concludes the beat was taken and moves on -- the header beat
    # vanishes and jc_deframe correctly rejects the cell beat that follows it
    # as a bad header. jc_deframe's own bench never hit this only because its
    # reset() happens to end on an aclk edge.
    await RisingEdge(dut.aclk)
    for data, keep, last in to_beats(frame):
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tdata.value = data
        dut.s_axis_tkeep.value = keep
        dut.s_axis_tlast.value = 1 if last else 0
        await wait_for(dut, dut.aclk, lambda: dut.s_axis_tready.value,
                       "s_axis_tready")
        dut.s_axis_tvalid.value = 0
    dut.s_axis_tlast.value = 0


LADDER = [
    ("frames_in", REG_FRAMES_IN), ("bad_header", REG_BAD_HEADER),
    ("bad_length", REG_BAD_LENGTH), ("accept", REG_ACCEPT),
    ("drop_full", REG_DROP_FULL), ("drop_err", REG_DROP_ERR),
    ("events", REG_EVENTS), ("jets_out", REG_JETS_OUT),
    ("frames_out", REG_FRAMES_OUT), ("suppressed", REG_SUPPRESSED),
    ("last_cycles", REG_LAST_CYCLES),
]


async def dump_counters(dut, label=""):
    """Read the whole ladder. This is what the ladder is FOR: the first stage
    whose count is zero is the stage that failed."""
    vals = {}
    for name, addr in LADDER:
        vals[name] = await axil_read(dut, addr)
    dut._log.info("counters %s: %s", label,
                  "  ".join(f"{k}={v}" for k, v in vals.items()))
    return vals


# One event is ~16k cycles, so 60k is generous. 400k merely turned every
# failure into an 80-second wait.
async def collect_frame(dut, limit=60000):
    """Wait for a jets frame. None if nothing arrives inside the limit."""
    beats = []
    dut.m_axis_tready.value = 1
    for _ in range(limit):
        await RisingEdge(dut.aclk)
        if dut.m_axis_tvalid.value:
            beats.append((int(dut.m_axis_tdata.value),
                          int(dut.m_axis_tkeep.value),
                          bool(dut.m_axis_tlast.value),
                          int(dut.m_axis_tuser.value)))
            if beats[-1][2]:
                dut.m_axis_tready.value = 0
                await RisingEdge(dut.aclk)
                return beats
    dut.m_axis_tready.value = 0
    return None


async def axil_write(dut, addr, data):
    dut.s_axil_awaddr.value = addr
    dut.s_axil_awvalid.value = 1
    dut.s_axil_wdata.value = data
    dut.s_axil_wvalid.value = 1
    await wait_for(dut, dut.axil_aclk,
                   lambda: dut.s_axil_awready.value and dut.s_axil_wready.value,
                   "s_axil_awready/wready")
    dut.s_axil_awvalid.value = 0
    dut.s_axil_wvalid.value = 0
    dut.s_axil_bready.value = 1
    await wait_for(dut, dut.axil_aclk, lambda: dut.s_axil_bvalid.value,
                   "s_axil_bvalid")
    dut.s_axil_bready.value = 0


async def axil_read(dut, addr):
    dut.s_axil_araddr.value = addr
    dut.s_axil_arvalid.value = 1
    await wait_for(dut, dut.axil_aclk, lambda: dut.s_axil_arready.value,
                   "s_axil_arready")
    dut.s_axil_arvalid.value = 0
    dut.s_axil_rready.value = 1
    await wait_for(dut, dut.axil_aclk, lambda: dut.s_axil_rvalid.value,
                   "s_axil_rvalid")
    val = int(dut.s_axil_rdata.value)
    dut.s_axil_rready.value = 0
    return val


async def set_floor(dut, gev):
    v = int(round(gev * gev * (1 << (2 * P4_FRAC))))
    await axil_write(dut, REG_FLR_0, v & 0xFFFFFFFF)
    await axil_write(dut, REG_FLR_1, (v >> 32) & 0xFFFFFFFF)
    await axil_write(dut, REG_FLR_2, (v >> 64) & 0xFFFFFFFF)
    # The config crossing is a handshake; give it time to land before the
    # datapath is expected to honour the new value.
    for _ in range(40):
        await RisingEdge(dut.axil_aclk)


async def set_r(dut, r_rad):
    v = FMT.r_squared(r_rad)
    await axil_write(dut, REG_RSQ_LO, v & 0xFFFFFFFF)
    await axil_write(dut, REG_RSQ_HI, (v >> 32) & 0xFFFFFFFF)
    for _ in range(40):
        await RisingEdge(dut.axil_aclk)


# ------------------------------------------------------------------- data ----
FIXTURE = pathlib.Path(__file__).resolve().parent / "data" / "events.pkt.bin"
PKT = pathlib.Path("/scratch/chettige/cells1k.pkt.bin")

NO_DATA = (f"no real-event data.\n  looked for {FIXTURE}\n           and {PKT}\n"
           f"  build it with: python3 model/make_fixture.py {PKT} {FIXTURE}")


def real_events(limit=400):
    src = FIXTURE if FIXTURE.exists() else PKT
    if not src.exists():
        return []
    out = []
    for i, (seq, _total, cells) in enumerate(M.read_packets(src)):
        if i >= limit:
            break
        out.append((seq, cells))
    return out


def expect(cells, floor_gev, r_rad=None):
    """The model's jets, as the fp32 the host will actually receive."""
    return expected_jets(
        M.cluster_fixed(cells, FMT, DEFAULT_R if r_rad is None else r_rad,
                        floor_gev))


# ------------------------------------------------------------------ tests ----
@cocotb.test()
async def test_axil_identity_and_scratch(dut):
    """The bus answers, and a register holds what was written."""
    await start_dut(dut)
    ident = await axil_read(dut, REG_ID)
    assert ident == 0x4A430001, f"ID reads {ident:#010x}"

    await axil_write(dut, REG_SCRATCH, 0xDEADBEEF)
    got = await axil_read(dut, REG_SCRATCH)
    assert got == 0xDEADBEEF, f"scratch reads {got:#010x}"

    # Everything is quiet, so every counter must still be zero.
    for name, addr in [("frames_in", REG_FRAMES_IN), ("accept", REG_ACCEPT),
                       ("events", REG_EVENTS), ("frames_out", REG_FRAMES_OUT)]:
        v = await axil_read(dut, addr)
        assert v == 0, f"{name} = {v} before any traffic"


@cocotb.test()
async def test_header_arrives_intact(dut):
    """The bytes the bench drives are the bytes jc_deframe decodes.

    Cheap and worth keeping: it isolates "the frame is malformed" from "the
    frame never arrived", which are indistinguishable from the counters alone
    and were confused for each other once already.
    """
    await start_dut(dut)
    events = real_events()
    assert events, NO_DATA
    seq, cells = min(events, key=lambda e: len(e[1]))
    data, keep, last = to_beats(build_frame(cells, seq))[0]

    dut.s_axis_tvalid.value = 1
    dut.s_axis_tdata.value = data
    dut.s_axis_tkeep.value = keep
    dut.s_axis_tlast.value = 0
    await Timer(1, units="ns")          # settle the combinational decode

    d = dut.u_deframe
    seen = int(d.s_axis_tdata.value)
    dut._log.info("bytes 12..17 at the DUT: %s",
                  " ".join("%02x" % ((seen >> (8 * n)) & 0xFF)
                           for n in range(12, 18)))
    dut._log.info("bench sent            : %s",
                  " ".join("%02x" % ((data >> (8 * n)) & 0xFF)
                           for n in range(12, 18)))
    for name in ("state", "hdr_ethtype", "hdr_version", "hdr_count",
                 "hdr_ok", "s_axis_tvalid", "s_axis_tready", "s_axis_tlast"):
        try:
            dut._log.info("  %-14s = %s", name, getattr(d, name).value)
        except Exception as exc:                       # noqa: BLE001
            dut._log.info("  %-14s unreadable (%s)", name, exc)

    dut.s_axis_tvalid.value = 0
    assert seen == data, "tdata at jc_deframe is not what the bench drove"


@cocotb.test()
async def test_trace_first_frame(dut):
    """One frame through deframe and ingest, traced cycle by cycle.

    The trace is kept because it is the cheapest way to read the handoffs when
    something upstream breaks -- cell_start at the first cell, pj_start six
    cycles later per jc_ingest's fixed latency, ev_valid once the event is
    whole. The assertions at the end are what make it a test rather than a
    log.
    """
    await start_dut(dut)
    events = real_events()
    assert events, NO_DATA
    seq, cells = min(events, key=lambda e: len(e[1]))
    nbeats = len(to_beats(build_frame(cells, seq)))
    dut._log.info("tracing seq %d: %d cells, %d beats", seq, len(cells), nbeats)

    frame = build_frame(cells, seq)
    beats = to_beats(frame)
    for i, (bd, bk, bl) in enumerate(beats):
        dut._log.info("  beat %d: eth-bytes=%04x tkeep=%#x tlast=%s", i,
                      ((bd >> (8 * 12)) & 0xFFFF), bk, bl)
    cocotb.start_soon(send_frame(dut, frame))

    d, ing, buf = dut.u_deframe, dut.u_ingest, dut.u_evbuf
    names = ("cyc st ok eth  bl left idx last | tv tr tl | "
             "cv cr cs cl ce | pv ps pl pe | ev")

    def bit(sig):
        try:
            v = sig.value
            return str(int(v)) if v.is_resolvable else "x"
        except Exception:                              # noqa: BLE001
            return "?"

    dut._log.info(names)
    prev = None
    for cyc in range(80):
        await RisingEdge(dut.aclk)
        try:
            eth = "%04x" % ((int(dut.s_axis_tdata.value) >> 96) & 0xFFFF)
        except Exception:                              # noqa: BLE001
            eth = "????"
        row = (f"{cyc:3d} {bit(d.state):>2} {bit(d.hdr_ok)} {eth} "
               f"{bit(d.beat_loaded)} "
               f"{bit(d.cells_left):>4} {bit(d.cell_idx):>3} "
               f"{bit(d.cell_idx_last):>4} | "
               f"{bit(dut.s_axis_tvalid)} {bit(dut.s_axis_tready)} "
               f"{bit(dut.s_axis_tlast)} | "
               f"{bit(d.cell_valid)} {bit(d.cell_ready)} {bit(d.cell_start)} "
               f"{bit(d.cell_last)} {bit(d.cell_err)} | "
               f"{bit(ing.pj_valid)} {bit(ing.pj_start)} {bit(ing.pj_last)} "
               f"{bit(ing.pj_err)} | {bit(buf.ev_valid)}")
        # Only print changes plus the first few cycles, to keep it readable.
        key = row[4:]
        if cyc < 6 or key != prev:
            dut._log.info(row)
        prev = key

    c = await dump_counters(dut, "after the traced frame")
    # Not just a trace: a well-formed frame must survive deframe intact. The
    # first version of this bench lost the header beat in the driver, and
    # every frame was counted as a bad header.
    assert c["frames_in"] == 1, "the frame never arrived"
    assert c["bad_header"] == 0 and c["bad_length"] == 0, (
        "a well-formed frame was rejected by jc_deframe")
    assert c["accept"] == 1, "the event never assembled in jc_evbuf"


@cocotb.test()
async def test_config_defaults_and_crossing(dut):
    """The generated defaults reach the engine, and a write crosses to it.

    Both halves matter. An unconfigured card must cluster correctly from the
    first packet, which is why both sides of the CDC reset to the same
    gen_luts.py constants rather than waiting for a transfer. And a write must
    actually arrive: this reads the aclk-side registers directly, so it cannot
    pass on the AXI-Lite side alone holding the value.
    """
    await start_dut(dut)
    want_r2 = LUTS["default_r_squared"]
    want_flr = LUTS["default_pt_sq_floor"]

    def rd(sig):
        v = sig.value
        return int(v) if v.is_resolvable else None

    r2_reset = rd(dut.u_regs.cfg_r_squared)
    flr_reset = rd(dut.u_regs.cfg_pt_sq_floor)
    dut._log.info("after reset: cfg_r_squared=%s (want %d)", r2_reset, want_r2)
    dut._log.info("after reset: cfg_pt_sq_floor=%s (want %d)", flr_reset, want_flr)
    assert r2_reset == want_r2, "the aclk-side R^2 default is wrong"
    assert flr_reset == want_flr, "the aclk-side floor default is wrong"

    # Now write the floor to zero and watch both sides.
    await set_floor(dut, 0.0)
    lo = await axil_read(dut, REG_FLR_0)
    mid = await axil_read(dut, REG_FLR_1)
    hi = await axil_read(dut, REG_FLR_2)
    dut._log.info("axil side after write: %08x_%08x_%08x", hi, mid, lo)
    assert (lo, mid, hi) == (0, 0, 0), "the AXI-Lite register did not take it"

    flr_after = rd(dut.u_regs.cfg_pt_sq_floor)
    r2_after = rd(dut.u_regs.cfg_r_squared)
    dut._log.info("aclk side after write: floor=%s r2=%s", flr_after, r2_after)
    assert flr_after == 0, "the config CDC did not carry the new floor across"
    assert r2_after == want_r2, "the CDC corrupted R^2 while carrying the floor"


@cocotb.test()
async def test_engine_hands_every_jet_to_reframe(dut):
    """Count jet_valid/jet_ready handshakes against the model, and log the FSM.

    Sits between "the engine clustered" and "a frame went out": if the jet
    count is right here but no frame appears, the fault is in jc_reframe, and
    if it is wrong here the engine is at fault. The state histogram is the
    quickest read on what the round FSM actually did -- EMIT and MERGE counts
    should be the model's jet and merge counts exactly.
    """
    await start_dut(dut)
    events = real_events()
    assert events, NO_DATA
    seq, cells = min(events, key=lambda e: len(e[1]))
    want = expect(cells, 0.0)
    await set_floor(dut, 0.0)

    ctrl = dut.u_engine.u_ctrl

    def rd(sig):
        v = sig.value
        return int(v) if v.is_resolvable else None

    dut._log.info("model: %d jets at floor 0 for seq %d (%d cells)",
                  len(want), seq, len(cells))
    dut._log.info("engine sees cfg_pt_sq_floor = %s", rd(ctrl.cfg_pt_sq_floor))

    cocotb.start_soon(send_frame(dut, build_frame(cells, seq)))

    names = {0: "IDLE", 1: "LOAD_RD", 2: "LOAD", 3: "SETUP_RD", 4: "SETUP_GO",
             5: "SETUP_W", 6: "ARGMIN", 7: "ARGMIN_W", 8: "DECIDE_RD",
             9: "DECIDE", 10: "EMIT", 11: "EMIT_MARK", 12: "EMIT_MARK_W",
             13: "MERGE_I", 14: "MERGE_J", 15: "MERGE_WR", 16: "KIN",
             17: "KIN_W", 18: "SCAN_I", 19: "SCAN_I_W", 20: "STALE_PICK",
             21: "STALE_RD", 22: "STALE_GO", 23: "STALE_W", 24: "REFRESH",
             25: "REFRESH_RD", 26: "REFRESH_GO", 27: "FINISH"}

    seen, handshakes, n_seen, mask_seen = {}, 0, None, None
    # A FIXED WINDOW, so it must outlast the event by a margin. It counts
    # handshakes and then compares the total to the model: an event that has
    # not finished when the window closes fails as "too few jets", which reads
    # like a datapath fault and is not one. Step 8t added ~3 cycles per merge.
    for _ in range(12000):
        await RisingEdge(dut.aclk)
        st = rd(ctrl.state)
        seen[st] = seen.get(st, 0) + 1
        if st == 2 and n_seen is None:                 # S_LOAD
            n_seen = rd(ctrl.n_cells)
        if st == 6 and mask_seen is None:              # S_ARGMIN
            m = rd(dut.u_engine.active_mask)
            mask_seen = bin(m).count("1") if m is not None else None
        if rd(dut.u_engine.jet_valid) and rd(dut.u_reframe.jet_ready):
            handshakes += 1

    dut._log.info("n_cells at S_LOAD        : %s (event has %d cells)",
                  n_seen, len(cells))
    dut._log.info("active rows at S_ARGMIN  : %s", mask_seen)
    dut._log.info("sw_result_valid at end   : %s",
                  rd(dut.u_engine.sw_result_valid))
    dut._log.info("states visited:")
    for st, n in sorted(seen.items(), key=lambda kv: -kv[1]):
        dut._log.info("    %-12s %6d", names.get(st, f"?{st}"), n)
    dut._log.info("jet handshakes %d; model wants %d", handshakes, len(want))
    c = await dump_counters(dut, "jet emission probe")
    assert handshakes == len(want), (
        f"{handshakes} jets handed to jc_reframe, model says {len(want)}")


@cocotb.test()
async def test_argmin_valid_pipeline_is_never_x(dut):
    """jc_sweep's valid chain must resolve, and ARGMIN must find a candidate.

    Written as a diagnostic and kept as a regression, because the bug it found
    is invisible by every other means. jc_sweep held its metadata in an
    UNPACKED array with `wire m = meta_dly[LAT_DIST-1]`, and in the full
    chain's elaboration Icarus stopped tracking that assignment: the array
    read back correctly while m sat at its time-zero value, all 1556 bits X.

    Nothing failed loudly. b_valid lives in that word, so key_valid_r,
    mid_valid and t_valid went X; `if (t_valid && ...)` is false for X, so
    acc_valid never latched; ARGMIN reported no candidate among 35 active
    rows; and jc_ctrl finished every event having emitted nothing. The module
    passed its own bench throughout -- it only misbehaved once instantiated
    inside jet_clustering.
    """
    await start_dut(dut)
    events = real_events()
    assert events, NO_DATA
    seq, cells = min(events, key=lambda e: len(e[1]))
    await set_floor(dut, 0.0)

    ctrl = dut.u_engine.u_ctrl
    sw = dut.u_engine.u_sweep

    def rd(sig):
        try:
            v = sig.value
            return int(v) if v.is_resolvable else None
        except Exception:                              # noqa: BLE001
            return "?"

    cocotb.start_soon(send_frame(dut, build_frame(cells, seq)))

    for _ in range(8000):
        await RisingEdge(dut.aclk)
        if rd(ctrl.state) == 6:                        # S_ARGMIN
            break
    else:
        assert False, "never reached S_ARGMIN"

    dut._log.info("rstn at sweep = %s, at top = %s",
                  rd(sw.aresetn), rd(dut.aresetn))

    # Where does the X enter the metadata line? meta_in's top bit IS a_valid,
    # so if that is clean and m's top bit is not, the delay line is the
    # culprit; if meta_in's top bit is already X, the concatenation is.
    def top(sig, name):
        try:
            s = sig.value.binstr
            dut._log.info("  %-10s width=%4d top=%s  x=%d  z=%d",
                          name, len(s), s[0], s.count("x") + s.count("X"),
                          s.count("z") + s.count("Z"))
        except Exception as exc:                       # noqa: BLE001
            dut._log.info("  %-10s unreadable (%s)", name, exc)

    top(sw.meta_in, "meta_in")
    top(sw.m, "m")
    for k in range(3):
        try:
            top(sw.meta_dly[k], f"meta_dly[{k}]")
        except Exception as exc:                       # noqa: BLE001
            dut._log.info("  meta_dly[%d] unindexable (%s)", k, exc)
    dut._log.info("i  st run cyc a_v b_v kvr mv tv av rv")
    unresolved, saw_result = [], False
    for i in range(28):
        vals = {"b_valid": rd(sw.b_valid), "key_valid_r": rd(sw.key_valid_r),
                "mid_valid": rd(sw.mid_valid), "t_valid": rd(sw.t_valid),
                "acc_valid": rd(sw.acc_valid)}
        dut._log.info("%2d %3s %3s %3s %3s %3s %3s %2s %2s %2s %2s",
                      i, rd(ctrl.state), rd(sw.running), rd(sw.cyc),
                      rd(sw.a_valid), vals["b_valid"], vals["key_valid_r"],
                      vals["mid_valid"], vals["t_valid"], vals["acc_valid"],
                      rd(dut.u_engine.sw_result_valid))
        unresolved += [n for n, v in vals.items() if v is None]
        saw_result |= (rd(dut.u_engine.sw_result_valid) == 1)
        await RisingEdge(dut.aclk)

    # THE REGRESSION. Every one of these was X, and the whole engine still
    # "ran": jc_ctrl saw no candidate among 35 active rows and finished each
    # event without emitting a jet. An `if (X)` is false, so nothing errored
    # and nothing warned -- 27 jets per event simply evaporated.
    assert not unresolved, (
        f"unresolvable valid bits in jc_sweep: {sorted(set(unresolved))}. "
        f"This is the unpacked-array regression -- a wire continuously "
        f"assigned from an array element stopped tracking it, freezing the "
        f"metadata word at its all-X reset value.")
    assert saw_result, "ARGMIN found no candidate among 35 active rows"


@cocotb.test()
async def test_one_event_reaches_every_stage(dut):
    """Walk one good event down the ladder, asserting stage by stage.

    Deliberately the first substantive test and deliberately cheap: when the
    chain is broken, the point is to learn WHERE in one run rather than to
    watch six frame comparisons time out. Each assertion names the stage that
    stopped, and the full ladder is logged either way.
    """
    await start_dut(dut)
    events = real_events()
    assert events, NO_DATA
    seq, cells = min(events, key=lambda e: len(e[1]))
    dut._log.info("diagnostic event: seq %d, %d cells", seq, len(cells))

    await set_floor(dut, 0.0)
    # Drain the egress while the event runs. Without this the frame sits in
    # ST_HDR waiting for tready and frames_out never moves -- which is correct
    # behaviour, but it made this test read as a datapath failure.
    collector = cocotb.start_soon(collect_frame(dut))
    await send_frame(dut, build_frame(cells, seq))
    beats = await collector.join()

    c = await dump_counters(dut, "after one good event")

    assert c["frames_in"] == 1, "jc_deframe never saw a complete frame"
    assert c["bad_header"] == 0, "a well-formed header was rejected"
    assert c["bad_length"] == 0, "the frame's length disagreed with its header"
    assert c["drop_err"] == 0, (
        "jc_evbuf dropped the event on a flagged cell -- jc_ingest's range "
        "check fired, so the cell fields are being decoded wrongly")
    assert c["drop_full"] == 0, "dropped for want of a slot, with an idle engine"
    assert c["accept"] == 1, "the event never reached jc_evbuf intact"
    assert c["events"] == 1, "jc_engine accepted the event but never finished it"
    assert c["last_cycles"] > 0, "the engine reported zero cycles"
    # The floor is zero, so this event fires: a frame, not a suppression.
    assert c["jets_out"] == len(expect(cells, 0.0)), (
        f"{c['jets_out']} jets reached jc_reframe, model says "
        f"{len(expect(cells, 0.0))}")
    assert c["suppressed"] == 0, "an event with jets was counted as suppressed"
    assert c["frames_out"] == 1, "the jets were buffered but no frame went out"
    assert beats is not None, "no frame collected"


@cocotb.test()
async def test_defaults_need_no_configuration(dut):
    """An unconfigured card must cluster correctly at the generated defaults.

    Nothing is written over AXI-Lite here at all. The event is chosen for
    having a jet above the default floor, so this cannot pass by emitting
    nothing.
    """
    await start_dut(dut)
    events = real_events()
    assert events, NO_DATA

    picked = None
    for seq, cells in events:
        if expect(cells, DEFAULT_FLOOR):
            picked = (seq, cells)
            break
    assert picked is not None, (
        f"no fixture event has a jet above the default {DEFAULT_FLOOR} GeV "
        f"floor, so this test would assert nothing == nothing")

    seq, cells = picked
    want = expect(cells, DEFAULT_FLOOR)
    cocotb.start_soon(send_frame(dut, build_frame(cells, seq)))
    beats = await collect_frame(dut)
    assert beats is not None, "no jets frame emitted"

    f = parse_jets_frame(beats)
    assert f["ethertype"] == ETH_TYPE_OUT, f"ethertype {f['ethertype']:#06x}"
    assert f["seq"] == seq, f"seq {f['seq']} vs {seq}"
    assert sorted(f["jets"]) == want, (
        f"jets differ from the model at the default floor\n"
        f"  got  {sorted(f['jets'])}\n  want {want}")
    dut._log.info("seq %d: n=%d -> %d jets in %d cycles at the defaults",
                  seq, len(cells), f["njets"], f["cycles"])


@cocotb.test()
async def test_floor_over_axil_changes_the_datapath(dut):
    """Writing the floor at 125 MHz must change what the 250 MHz engine emits.

    Same event twice: at the default floor and at zero. The second must emit
    strictly more jets, which is what proves the crossing carried the value
    rather than the datapath keeping its reset default.
    """
    await start_dut(dut)
    events = real_events()
    assert events, NO_DATA
    seq, cells = next((e for e in events if 30 <= len(e[1]) <= 80), events[0])

    await set_floor(dut, 0.0)
    want = expect(cells, 0.0)
    cocotb.start_soon(send_frame(dut, build_frame(cells, seq)))
    beats = await collect_frame(dut)
    assert beats is not None, "no frame at a zero floor"
    f = parse_jets_frame(beats)

    assert sorted(f["jets"]) == want, (
        f"floor=0: {len(f['jets'])} jets, model says {len(want)}")
    assert len(want) > len(expect(cells, DEFAULT_FLOOR)), (
        "this event emits the same jets at both floors, so it cannot show "
        "that the AXI-Lite write did anything")
    dut._log.info("seq %d: %d jets at floor 0 (vs %d at the default)",
                  seq, len(f["jets"]), len(expect(cells, DEFAULT_FLOOR)))


@cocotb.test()
async def test_r_over_axil(dut):
    """A different R must change the partition, not merely the jet count."""
    await start_dut(dut)
    events = real_events()
    assert events, NO_DATA
    seq, cells = next((e for e in events if 30 <= len(e[1]) <= 80), events[0])

    await set_floor(dut, 0.0)
    await set_r(dut, 0.8)
    want = expect(cells, 0.0, r_rad=0.8)
    cocotb.start_soon(send_frame(dut, build_frame(cells, seq)))
    beats = await collect_frame(dut)
    assert beats is not None, "no frame at R=0.8"
    f = parse_jets_frame(beats)
    assert sorted(f["jets"]) == want, (
        f"R=0.8: {len(f['jets'])} jets, model says {len(want)}")
    assert want != expect(cells, 0.0, r_rad=DEFAULT_R), (
        "R=0.8 and the default give the same jets on this event, so the "
        "write cannot be shown to have taken effect")


@cocotb.test()
async def test_suppressed_event_emits_nothing(dut):
    """A soft event at the trigger floor: silence, and a counter that says so."""
    await start_dut(dut)
    events = real_events()
    assert events, NO_DATA

    picked = next(((s, c) for s, c in events if not expect(c, DEFAULT_FLOOR)),
                  None)
    assert picked is not None, "every fixture event clears the default floor"
    seq, cells = picked

    await send_frame(dut, build_frame(cells, seq))
    beats = await collect_frame(dut)
    assert beats is None, "a jetless event emitted a frame"

    c = await dump_counters(dut, "suppressed event")
    assert c["events"] == 1, "the event was never clustered"
    assert c["suppressed"] == 1, "clustered, but not counted as suppressed"
    assert c["frames_out"] == 0


@cocotb.test()
async def test_bad_header_is_counted_and_dropped(dut):
    """Wrong ethertype: rejected before a cell is emitted, counted as header."""
    await start_dut(dut)
    events = real_events()
    assert events, NO_DATA
    seq, cells = events[0]

    await send_frame(dut, build_frame(cells, seq, ethertype=0x1234))
    await send_frame(dut, build_frame(cells, seq, version=0x99))
    for _ in range(200):
        await RisingEdge(dut.aclk)

    assert await axil_read(dut, REG_BAD_HEADER) == 2, "bad headers not counted"
    assert await axil_read(dut, REG_BAD_LENGTH) == 0, "counted as a length error"
    assert await axil_read(dut, REG_ACCEPT) == 0, "a bad frame reached the engine"
    assert await axil_read(dut, REG_FRAMES_IN) == 2

    # A GOOD frame now, or this test cannot tell "rejects bad headers" from
    # "rejects everything". It could not: the first version of this bench lost
    # the header beat in the driver, every frame was counted as a bad header,
    # and the two assertions above still passed.
    await send_frame(dut, build_frame(cells, seq))
    for _ in range(2000):
        await RisingEdge(dut.aclk)

    c = await dump_counters(dut, "after a good frame")
    assert c["frames_in"] == 3
    assert c["bad_header"] == 2, "the well-formed frame was rejected too"
    assert c["accept"] == 1, "the well-formed frame never reached the engine"


@cocotb.test()
async def test_counter_ladder_adds_up(dut):
    """Several events, then the whole ladder read as one coherent snapshot."""
    await start_dut(dut)
    events = real_events()
    assert events, NO_DATA
    await set_floor(dut, 0.0)

    chosen = [e for e in events if len(e[1]) <= 60][:3]
    assert len(chosen) >= 2, "need at least two small events"

    total_jets = 0
    for seq, cells in chosen:
        want = expect(cells, 0.0)
        total_jets += len(want)
        cocotb.start_soon(send_frame(dut, build_frame(cells, seq)))
        beats = await collect_frame(dut)
        assert beats is not None, f"seq {seq}: no frame"
        f = parse_jets_frame(beats)
        assert sorted(f["jets"]) == want, f"seq {seq}: jets differ"

    n = len(chosen)
    frames_in = await axil_read(dut, REG_FRAMES_IN)
    bad_h = await axil_read(dut, REG_BAD_HEADER)
    bad_l = await axil_read(dut, REG_BAD_LENGTH)
    accept = await axil_read(dut, REG_ACCEPT)
    drop_f = await axil_read(dut, REG_DROP_FULL)
    drop_e = await axil_read(dut, REG_DROP_ERR)
    ev = await axil_read(dut, REG_EVENTS)
    jets = await axil_read(dut, REG_JETS_OUT)
    frames_o = await axil_read(dut, REG_FRAMES_OUT)
    supp = await axil_read(dut, REG_SUPPRESSED)
    cycles = await axil_read(dut, REG_LAST_CYCLES)

    dut._log.info("ladder: in=%d badh=%d badl=%d acc=%d dropf=%d drope=%d "
                  "ev=%d jets=%d out=%d supp=%d cycles=%d",
                  frames_in, bad_h, bad_l, accept, drop_f, drop_e,
                  ev, jets, frames_o, supp, cycles)

    assert frames_in == n, f"frames_in {frames_in} vs {n}"
    assert bad_h == 0 and bad_l == 0, "clean frames were counted as bad"
    assert drop_e == 0, "a clean event was flagged"
    # Nothing here is dropped for want of a slot: each event is collected
    # before the next is sent.
    assert accept + drop_f == n, f"accept {accept} + drop_full {drop_f} != {n}"
    assert accept == n and drop_f == 0
    assert ev == n, f"events clustered {ev} vs {n}"
    assert jets == total_jets, f"jets_out {jets} vs {total_jets}"
    assert frames_o + supp == n, f"frames {frames_o} + suppressed {supp} != {n}"
    assert cycles > 0, "cycle_count never moved"


@cocotb.test()
async def test_full_scale_event(dut):
    """A 128-cell event through the whole chain, bit-exact.

    The headline claim: real Pythia cells in as bytes, jets out as fp32,
    identical to jc_model.py.
    """
    await start_dut(dut)
    events = real_events()
    assert events, NO_DATA
    await set_floor(dut, 0.0)

    seq, cells = max(events, key=lambda e: len(e[1]))
    want = expect(cells, 0.0)
    cocotb.start_soon(send_frame(dut, build_frame(cells, seq)))
    beats = await collect_frame(dut)
    assert beats is not None, "no frame for the full-scale event"
    f = parse_jets_frame(beats)

    assert f["njets"] == len(want), f"{f['njets']} jets, model says {len(want)}"
    assert f["size"] == 64 + 16 * len(want), f"tuser_size {f['size']}"
    assert sorted(f["jets"]) == want, "jets differ from the model"
    dut._log.info("seq %d: n=%d -> %d jets in %d cycles (%.1f/cell)",
                  seq, len(cells), f["njets"], f["cycles"],
                  f["cycles"] / len(cells))
