"""Cocotb tests for jc_evbuf -- the two-slot event buffer.

The buffer's whole job is accept-or-drop: it must never assert back-pressure
(that would stall the shell's RX path) and must never store an event
partially (that would cluster into plausible wrong jets). So the tests are
mostly about what happens when it CANNOT take an event.

Records are synthetic, keyed on (seq, index), so a slot mix-up or an
off-by-one in the read port shows up as a value mismatch rather than as a
plausible-looking wrong number.
"""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

NMAX = 128
P4_MASK = (1 << 48) - 1
W32_MASK = (1 << 32) - 1

FIELDS = ("energy", "px", "py", "pz", "rapidity", "phi", "weight")


def record(seq, i):
    """A distinguishable payload for cell i of event seq."""
    base = ((seq & 0xFFFF) << 16) | (i & 0xFFFF)
    return {
        "energy": (base * 7 + 1) & P4_MASK,
        "px": (base * 7 + 2) & P4_MASK,
        "py": (base * 7 + 3) & P4_MASK,
        "pz": (base * 7 + 4) & P4_MASK,
        "rapidity": (base * 7 + 5) & W32_MASK,
        "phi": (base * 7 + 6) & W32_MASK,
        "weight": (base * 7 + 7) & W32_MASK,
    }


def sample(dut):
    return {
        "energy": int(dut.ev_energy.value) & P4_MASK,
        "px": int(dut.ev_px.value) & P4_MASK,
        "py": int(dut.ev_py.value) & P4_MASK,
        "pz": int(dut.ev_pz.value) & P4_MASK,
        "rapidity": int(dut.ev_rapidity.value) & W32_MASK,
        "phi": int(dut.ev_phi.value) & W32_MASK,
        "weight": int(dut.ev_beam_weight_log.value) & W32_MASK,
    }


async def start_dut(dut):
    cocotb.start_soon(Clock(dut.aclk, 4, units="ns").start())  # 250 MHz
    dut.pj_valid.value = 0
    dut.pj_start.value = 0
    dut.pj_last.value = 0
    dut.pj_err.value = 0
    dut.pj_event_seq.value = 0
    for f in ("pj_energy", "pj_px", "pj_py", "pj_pz",
              "pj_rapidity", "pj_phi", "pj_beam_weight_log"):
        getattr(dut, f).value = 0
    dut.ev_accept.value = 0
    dut.ev_release.value = 0
    dut.ev_addr.value = 0
    dut.aresetn.value = 0
    for _ in range(5):
        await RisingEdge(dut.aclk)
    dut.aresetn.value = 1
    await RisingEdge(dut.aclk)


async def push_event(dut, seq, n, err_at=None):
    """Drive one whole event. pj_ready is tied high, so there is no handshake."""
    for i in range(n):
        r = record(seq, i)
        dut.pj_valid.value = 1
        dut.pj_energy.value = r["energy"]
        dut.pj_px.value = r["px"]
        dut.pj_py.value = r["py"]
        dut.pj_pz.value = r["pz"]
        dut.pj_rapidity.value = r["rapidity"]
        dut.pj_phi.value = r["phi"]
        dut.pj_beam_weight_log.value = r["weight"]
        dut.pj_start.value = 1 if i == 0 else 0
        dut.pj_last.value = 1 if i == n - 1 else 0
        dut.pj_err.value = 1 if i == err_at else 0
        dut.pj_event_seq.value = seq
        assert dut.pj_ready.value == 1, "jc_evbuf asserted back-pressure"
        await RisingEdge(dut.aclk)
    dut.pj_valid.value = 0
    dut.pj_start.value = 0
    dut.pj_last.value = 0
    dut.pj_err.value = 0
    # One settling cycle so the last cell's non-blocking updates -- slot
    # state, ev_valid, the counters -- are visible to the caller's asserts.
    await RisingEdge(dut.aclk)


async def accept_event(dut):
    """Claim the waiting event; returns (count, seq)."""
    while not dut.ev_valid.value:
        await RisingEdge(dut.aclk)
    count, seq = int(dut.ev_count.value), int(dut.ev_seq.value)
    dut.ev_accept.value = 1
    await RisingEdge(dut.aclk)
    dut.ev_accept.value = 0
    return count, seq


async def read_back(dut, n):
    """Read n records out. The read port is registered, so the address for
    entry i+1 is presented while entry i is being sampled."""
    out = []
    dut.ev_addr.value = 0
    await RisingEdge(dut.aclk)
    for i in range(n):
        if i + 1 < n:
            dut.ev_addr.value = i + 1
        await RisingEdge(dut.aclk)
        out.append(sample(dut))
    return out


async def release(dut):
    dut.ev_release.value = 1
    await RisingEdge(dut.aclk)
    dut.ev_release.value = 0
    await RisingEdge(dut.aclk)


def check(got, seq, n, label):
    for i, g in enumerate(got):
        exp = record(seq, i)
        for f in FIELDS:
            assert g[f] == exp[f], (
                f"{label}: entry {i} field {f} = {g[f]:#x}, "
                f"expected {exp[f]:#x}")


@cocotb.test()
async def test_single_event(dut):
    """One event in, one event out, every field intact."""
    await start_dut(dut)
    await push_event(dut, seq=0xABCD, n=40)

    count, seq = await accept_event(dut)
    assert count == 40, f"ev_count = {count}"
    assert seq == 0xABCD, f"ev_seq = {seq:#x}"

    got = await read_back(dut, 40)
    check(got, 0xABCD, 40, "single event")
    await release(dut)

    assert int(dut.accept_count.value) == 1
    assert int(dut.drop_count.value) == 0


@cocotb.test()
async def test_full_and_minimal_events(dut):
    """NMAX cells and a single cell -- the two size extremes."""
    await start_dut(dut)
    for seq, n in ((1, NMAX), (2, 1)):
        await push_event(dut, seq=seq, n=n)
        count, got_seq = await accept_event(dut)
        assert count == n, f"n={n}: ev_count = {count}"
        assert got_seq == seq
        got = await read_back(dut, n)
        check(got, seq, n, f"n={n}")
        await release(dut)

    assert int(dut.accept_count.value) == 2
    assert int(dut.drop_count.value) == 0


@cocotb.test()
async def test_ping_pong_preserves_order(dut):
    """Both slots occupied at once; events must leave in arrival order."""
    await start_dut(dut)
    await push_event(dut, seq=10, n=20)
    await push_event(dut, seq=11, n=30)

    for seq, n in ((10, 20), (11, 30)):
        count, got_seq = await accept_event(dut)
        assert got_seq == seq, f"out of order: got seq {got_seq}, wanted {seq}"
        assert count == n
        got = await read_back(dut, n)
        check(got, seq, n, f"seq={seq}")
        await release(dut)

    assert int(dut.accept_count.value) == 2
    assert int(dut.drop_count.value) == 0


@cocotb.test()
async def test_drop_when_full_and_recover(dut):
    """A third event with both slots occupied is dropped whole, then the
    buffer must take the next one once a slot frees."""
    await start_dut(dut)
    await push_event(dut, seq=20, n=10)
    await push_event(dut, seq=21, n=10)
    await push_event(dut, seq=22, n=10)          # nowhere to put it

    assert int(dut.drop_count.value) == 1, (
        f"drop_count = {int(dut.drop_count.value)}, expected 1")
    assert int(dut.accept_count.value) == 2

    # The two stored events must be untouched by the dropped one.
    for seq in (20, 21):
        count, got_seq = await accept_event(dut)
        assert got_seq == seq, f"dropped event disturbed ordering: {got_seq}"
        got = await read_back(dut, count)
        check(got, seq, count, f"seq={seq} after a drop")
        await release(dut)

    # And the buffer still works afterwards.
    await push_event(dut, seq=23, n=15)
    count, got_seq = await accept_event(dut)
    assert got_seq == 23 and count == 15
    got = await read_back(dut, 15)
    check(got, 23, 15, "after recovery")
    await release(dut)

    assert int(dut.accept_count.value) == 3
    assert int(dut.drop_count.value) == 1


@cocotb.test()
async def test_err_event_dropped(dut):
    """One bad cell condemns the whole event and hands the slot straight back.

    A partially stored event is the one failure nothing downstream could
    detect, so this must drop rather than truncate.
    """
    await start_dut(dut)
    await push_event(dut, seq=30, n=25, err_at=12)

    assert int(dut.drop_count.value) == 1
    assert int(dut.accept_count.value) == 0
    assert not dut.ev_valid.value, "a flagged event was offered to the engine"

    # The slot must have been released, so two more events still fit.
    await push_event(dut, seq=31, n=8)
    await push_event(dut, seq=32, n=9)
    assert int(dut.drop_count.value) == 1, "slot was not handed back"

    for seq, n in ((31, 8), (32, 9)):
        count, got_seq = await accept_event(dut)
        assert (got_seq, count) == (seq, n)
        got = await read_back(dut, n)
        check(got, seq, n, f"seq={seq} after an err drop")
        await release(dut)


@cocotb.test()
async def test_err_on_first_and_last_cell(dut):
    """The flag must condemn the event wherever in it the bad cell falls."""
    await start_dut(dut)
    for seq, n, at in ((40, 12, 0), (41, 12, 11), (42, 1, 0)):
        await push_event(dut, seq=seq, n=n, err_at=at)
        assert not dut.ev_valid.value, f"seq={seq}: err at {at} not caught"
    assert int(dut.drop_count.value) == 3
    assert int(dut.accept_count.value) == 0


@cocotb.test()
async def test_overflow_dropped(dut):
    """More than NMAX cells must drop, not wrap over cell 0.

    jc_deframe rejects count > NMAX, so this is unreachable through a
    well-formed frame -- the buffer must not corrupt an event on the strength
    of an upstream guarantee.
    """
    await start_dut(dut)
    await push_event(dut, seq=50, n=NMAX + 1)

    assert int(dut.drop_count.value) == 1, "oversized event was stored"
    assert int(dut.accept_count.value) == 0
    assert not dut.ev_valid.value

    await push_event(dut, seq=51, n=NMAX)
    count, got_seq = await accept_event(dut)
    assert (got_seq, count) == (51, NMAX)
    got = await read_back(dut, NMAX)
    check(got, 51, NMAX, "full event after an overflow")
    await release(dut)


@cocotb.test()
async def test_sustained_load_never_blocks(dut):
    """Many events with the engine draining slowly: pj_ready stays high
    throughout and every event is either accepted or counted, never lost."""
    await start_dut(dut)
    rng = random.Random(21)

    async def engine():
        while True:
            if dut.ev_valid.value:
                count, seq = await accept_event(dut)
                got = await read_back(dut, count)
                check(got, seq, count, f"seq={seq} under load")
                for _ in range(rng.randrange(20, 60)):   # stand in for clustering
                    await RisingEdge(dut.aclk)
                await release(dut)
            else:
                await RisingEdge(dut.aclk)

    cocotb.start_soon(engine())

    total = 30
    for seq in range(100, 100 + total):
        await push_event(dut, seq=seq, n=rng.randrange(4, 40))
        for _ in range(rng.randrange(0, 8)):
            await RisingEdge(dut.aclk)

    for _ in range(200):
        await RisingEdge(dut.aclk)

    accepted = int(dut.accept_count.value)
    dropped = int(dut.drop_count.value)
    dut._log.info("accepted %d, dropped %d of %d", accepted, dropped, total)
    assert accepted + dropped == total, (
        f"{accepted} + {dropped} != {total} -- an event vanished")
    assert dropped > 0, "engine was not slow enough to exercise the drop path"
