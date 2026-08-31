"""Cocotb tests for jc_reframe -- jets in, one output frame per firing event.

Checks the frame against the format in jc_defs.vh byte by byte, not just the
jet payload: a header field in the wrong place is exactly the kind of thing a
tolerance-based check waves through and a host parser then blames on the
clustering.

Two behaviours here are deliberate and are asserted as such rather than
assumed:

  * an event with no jet above the floor emits NOTHING, and bumps `suppressed`
    instead. On a soft sample that is the common case, so the counter is the
    only evidence the engine ran at all.
  * jet_ready falls for the whole time a frame is draining, which is what lets
    a single buffer be safe. The test drives jets at a frame while it streams
    and requires them to be held off.
"""

import random
import struct

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

P4_W = 48
P4_FRAC = 34
P4_MASK = (1 << P4_W) - 1

ETH_TYPE = 0x88B7
FMT_VERSION = 0x01
NMAX = 128


def fp32(v_int):
    """Q14.34 integer -> the float the host will read. Exact, then one RNE."""
    return struct.unpack(">f", struct.pack(">f", v_int / (1 << P4_FRAC)))[0]


def gev(x):
    return int(round(x * (1 << P4_FRAC)))


async def start_dut(dut):
    cocotb.start_soon(Clock(dut.aclk, 4, units="ns").start())
    dut.jet_valid.value = 0
    dut.jet_eoe.value = 0
    for s in ("jet_e", "jet_px", "jet_py", "jet_pz"):
        getattr(dut, s).value = 0
    dut.jet_seq.value = 0
    dut.ev_cycles.value = 0
    dut.cnt_drop_full.value = 0
    dut.cnt_drop_err.value = 0
    dut.cnt_bad_frame.value = 0
    dut.cfg_dst_mac.value = 0
    dut.cfg_src_mac.value = 0
    dut.cfg_tuser_src.value = 0
    dut.m_axis_tready.value = 0
    dut.aresetn.value = 0
    for _ in range(5):
        await RisingEdge(dut.aclk)
    dut.aresetn.value = 1
    await RisingEdge(dut.aclk)


async def send_jet(dut, jet, seq=0):
    """Offer one jet and wait for it to be taken."""
    e, px, py, pz = jet
    dut.jet_e.value = e & P4_MASK
    dut.jet_px.value = px & P4_MASK
    dut.jet_py.value = py & P4_MASK
    dut.jet_pz.value = pz & P4_MASK
    dut.jet_seq.value = seq
    dut.jet_valid.value = 1
    while True:
        await RisingEdge(dut.aclk)
        if dut.jet_ready.value:
            break
    dut.jet_valid.value = 0


async def pulse_eoe(dut, seq=0, cycles=0):
    dut.jet_seq.value = seq
    dut.ev_cycles.value = cycles
    dut.jet_eoe.value = 1
    await RisingEdge(dut.aclk)
    dut.jet_eoe.value = 0


async def collect_frame(dut, stall=lambda: False, limit=400):
    """Pull beats until tlast. Returns None if nothing was emitted."""
    beats = []
    for _ in range(limit):
        ready = not stall()
        dut.m_axis_tready.value = 1 if ready else 0
        await RisingEdge(dut.aclk)
        if ready and dut.m_axis_tvalid.value:
            beats.append((int(dut.m_axis_tdata.value),
                          int(dut.m_axis_tkeep.value),
                          bool(dut.m_axis_tlast.value),
                          int(dut.m_axis_tuser.value)))
            if beats[-1][2]:
                dut.m_axis_tready.value = 0
                return beats
    dut.m_axis_tready.value = 0
    return None


def parse_frame(beats):
    """Frame bytes -> fields, per jc_defs.vh. Byte n of a beat is tdata[8n]."""
    hdr = beats[0][0].to_bytes(64, "little")
    assert beats[0][1] == (1 << 64) - 1, "header beat must be fully valid"

    out = {
        "dst_mac": hdr[0:6],
        "src_mac": hdr[6:12],
        "ethertype": int.from_bytes(hdr[12:14], "big"),
        "version": hdr[14],
        "njets": int.from_bytes(hdr[16:18], "big"),
        "seq": int.from_bytes(hdr[18:22], "big"),
        "cycles": int.from_bytes(hdr[22:26], "big"),
        "drop_full": int.from_bytes(hdr[26:30], "big"),
        "drop_err": int.from_bytes(hdr[30:34], "big"),
        "bad_frame": int.from_bytes(hdr[34:38], "big"),
        "reserved": hdr[38:64],
        "size": beats[0][3] & 0xFFFF,
        "src": (beats[0][3] >> 16) & 0xFFFF,
        "jets": [],
    }
    for data, keep, _last, _u in beats[1:]:
        raw = data.to_bytes(64, "little")
        nbytes = bin(keep).count("1")
        assert keep == (1 << nbytes) - 1, f"tkeep not a run of ones: {keep:#x}"
        assert nbytes % 16 == 0, f"tkeep {nbytes} bytes is not whole jets"
        for off in range(0, nbytes, 16):
            out["jets"].append(struct.unpack(">ffff", raw[off:off + 16]))
    return out


def expect_jets(jets):
    return [tuple(fp32(c) for c in j) for j in jets]


async def run_event(dut, jets, seq=1, cycles=0, stall=lambda: False):
    """Send an event's jets, end it, and return the parsed frame (or None)."""
    collector = cocotb.start_soon(collect_frame(dut, stall=stall))
    for j in jets:
        await send_jet(dut, j, seq=seq)
    await pulse_eoe(dut, seq=seq, cycles=cycles)
    beats = await collector.join()
    # collect_frame returns on the SAME edge that carried tlast, and
    # frames_out is registered on that edge -- so a counter read here would
    # see its pre-edge value. One more clock lets the end-of-frame updates
    # land before any test inspects them.
    await RisingEdge(dut.aclk)
    return parse_frame(beats) if beats else None


def check_frame(f, jets, seq, cycles, label):
    assert f is not None, f"{label}: no frame emitted"
    assert f["ethertype"] == ETH_TYPE, (
        f"{label}: ethertype {f['ethertype']:#06x}, want {ETH_TYPE:#06x} -- "
        f"a jets frame must not be re-ingestable as cells")
    assert f["version"] == FMT_VERSION, f"{label}: version {f['version']}"
    assert f["njets"] == len(jets), f"{label}: njets {f['njets']} vs {len(jets)}"
    assert f["seq"] == seq, f"{label}: seq {f['seq']} vs {seq}"
    assert f["cycles"] == cycles, f"{label}: cycles {f['cycles']} vs {cycles}"
    assert f["size"] == 64 + 16 * len(jets), (
        f"{label}: tuser_size {f['size']} vs {64 + 16 * len(jets)}")
    assert f["reserved"] == bytes(26), f"{label}: reserved bytes not zero"

    want = expect_jets(jets)
    assert f["jets"] == want, (
        f"{label}: jet payload differs\n  got  {f['jets'][:3]}\n"
        f"  want {want[:3]}")


@cocotb.test()
async def test_single_jet(dut):
    """One jet: a header beat and a 16-byte partial beat."""
    await start_dut(dut)
    jets = [(gev(60.0), gev(40.0), gev(-30.0), gev(10.0))]
    f = await run_event(dut, jets, seq=7, cycles=16186)
    check_frame(f, jets, 7, 16186, "single jet")
    assert len(f["jets"]) == 1
    assert int(dut.frames_out.value) == 1
    assert int(dut.jets_out.value) == 1
    assert int(dut.suppressed.value) == 0


@cocotb.test()
async def test_exactly_one_full_beat(dut):
    """Four jets fill a beat exactly -- the boundary the partial flush skips."""
    await start_dut(dut)
    jets = [(gev(50.0 + i), gev(10.0 * i), gev(-5.0 * i), gev(i))
            for i in range(4)]
    f = await run_event(dut, jets, seq=11, cycles=1234)
    check_frame(f, jets, 11, 1234, "four jets")
    assert f["reserved"] == bytes(26)


@cocotb.test()
async def test_partial_tail(dut):
    """Five jets: one full beat plus a one-jet tail, exercising the flush."""
    await start_dut(dut)
    jets = [(gev(50.0 + i), gev(3.0 * i), gev(-2.0 * i), gev(0.5 * i))
            for i in range(5)]
    f = await run_event(dut, jets, seq=12, cycles=99)
    check_frame(f, jets, 12, 99, "five jets")


@cocotb.test()
async def test_every_tail_length(dut):
    """1..9 jets, so every tkeep case and both flush paths are covered."""
    await start_dut(dut)
    for n in range(1, 10):
        jets = [(gev(50.0 + i), gev(1.0 + i), gev(-1.0 - i), gev(i))
                for i in range(n)]
        f = await run_event(dut, jets, seq=100 + n, cycles=n)
        check_frame(f, jets, 100 + n, n, f"{n} jets")


@cocotb.test()
async def test_zero_jets_emits_nothing(dut):
    """The suppression decision, and the counter that keeps it observable."""
    await start_dut(dut)
    await pulse_eoe(dut, seq=5, cycles=800)
    for _ in range(40):
        await RisingEdge(dut.aclk)
        assert not dut.m_axis_tvalid.value, "a jetless event emitted a frame"
    assert int(dut.suppressed.value) == 1, "suppressed did not move"
    assert int(dut.frames_out.value) == 0

    # And the module is still usable afterwards.
    jets = [(gev(70.0), gev(50.0), gev(20.0), gev(-5.0))]
    f = await run_event(dut, jets, seq=6, cycles=10)
    check_frame(f, jets, 6, 10, "after a suppressed event")
    assert int(dut.suppressed.value) == 1


@cocotb.test()
async def test_full_event(dut):
    """128 jets -- the one-jet-per-cell worst case the buffer is sized for."""
    await start_dut(dut)
    rng = random.Random(7)
    jets = [(gev(rng.uniform(1, 500)), gev(rng.uniform(-200, 200)),
             gev(rng.uniform(-200, 200)), gev(rng.uniform(-300, 300)))
            for _ in range(NMAX)]
    f = await run_event(dut, jets, seq=999, cycles=16186)
    check_frame(f, jets, 999, 16186, "128 jets")
    assert f["size"] == 64 + 16 * NMAX == 2112


@cocotb.test()
async def test_backpressure(dut):
    """Random tready stalls must not change a single byte."""
    await start_dut(dut)
    rng = random.Random(31)
    jets = [(gev(50.0 + i), gev(2.0 * i), gev(-3.0 * i), gev(i))
            for i in range(11)]
    f = await run_event(dut, jets, seq=42, cycles=7777,
                        stall=lambda: rng.random() < 0.6)
    check_frame(f, jets, 42, 7777, "back-pressured")


@cocotb.test()
async def test_header_metadata(dut):
    """Counters and MACs are sampled at end of event, not at frame start."""
    await start_dut(dut)
    dut.cfg_dst_mac.value = 0x001B21AABBCC
    dut.cfg_src_mac.value = 0x001B21DDEEFF
    dut.cfg_tuser_src.value = 0x0002
    dut.cnt_drop_full.value = 12345
    dut.cnt_drop_err.value = 7
    dut.cnt_bad_frame.value = 99

    jets = [(gev(80.0), gev(60.0), gev(-20.0), gev(5.0))]
    collector = cocotb.start_soon(collect_frame(dut))
    await send_jet(dut, jets[0], seq=3)
    await pulse_eoe(dut, seq=3, cycles=555)
    # Move the counters AFTER the event ended; the frame must show the old
    # values, because they describe the event it is reporting.
    dut.cnt_drop_full.value = 0
    dut.cnt_drop_err.value = 0
    dut.cnt_bad_frame.value = 0
    beats = await collector.join()
    f = parse_frame(beats) if beats else None

    check_frame(f, jets, 3, 555, "metadata")
    assert f["drop_full"] == 12345, f"drop_full {f['drop_full']}"
    assert f["drop_err"] == 7, f"drop_err {f['drop_err']}"
    assert f["bad_frame"] == 99, f"bad_frame {f['bad_frame']}"
    assert f["dst_mac"] == bytes.fromhex("001B21AABBCC"), f["dst_mac"].hex()
    assert f["src_mac"] == bytes.fromhex("001B21DDEEFF"), f["src_mac"].hex()
    assert f["src"] == 2, f"tuser_src {f['src']}"


@cocotb.test()
async def test_ready_low_while_draining(dut):
    """The single-buffer safety property, asserted rather than reasoned about.

    While a frame is on the wire the next event's jets must not be accepted --
    otherwise they land in the buffer being read.
    """
    await start_dut(dut)
    jets = [(gev(50.0 + i), gev(i), gev(-i), gev(i)) for i in range(8)]
    for j in jets:
        await send_jet(dut, j, seq=1)
    await pulse_eoe(dut, seq=1, cycles=10)

    # Frame is now queued. Offer a jet and hold tready low: it must not move.
    dut.jet_valid.value = 1
    dut.jet_e.value = gev(500.0)
    saw_busy = False
    for _ in range(20):
        await RisingEdge(dut.aclk)
        if dut.busy.value:
            saw_busy = True
            assert not dut.jet_ready.value, (
                "jet accepted while a frame was streaming -- the buffer being "
                "read would be overwritten")
    dut.jet_valid.value = 0
    assert saw_busy, "the frame never went out; test proved nothing"


@cocotb.test()
async def test_back_to_back_events(dut):
    """Three events in a row: no state, no counts and no jets leaking across."""
    await start_dut(dut)
    events = [
        [(gev(50.0), gev(30.0), gev(-10.0), gev(2.0))],
        [(gev(60.0 + i), gev(5.0 * i), gev(-4.0 * i), gev(i))
         for i in range(6)],
        [(gev(90.0), gev(70.0), gev(-30.0), gev(11.0)),
         (gev(55.0), gev(20.0), gev(40.0), gev(-9.0))],
    ]
    total = 0
    for n, jets in enumerate(events):
        f = await run_event(dut, jets, seq=200 + n, cycles=1000 + n)
        check_frame(f, jets, 200 + n, 1000 + n, f"event {n}")
        total += len(jets)

    assert int(dut.frames_out.value) == len(events), (
        f"frames_out = {int(dut.frames_out.value)}")
    assert int(dut.jets_out.value) == total, (
        f"jets_out = {int(dut.jets_out.value)} vs {total}")
    assert int(dut.suppressed.value) == 0
