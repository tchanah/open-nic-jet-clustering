"""Cocotb tests for jc_deframe -- 512-bit beats to a 1-cell-per-cycle stream.

Frame layout (see jc_defs.vh):
    beat 0        event header
                    bytes  0..11  dst/src MAC
                    bytes 12..13  ethertype 0x88B6
                    byte  14      format version
                    byte  15      pad
                    bytes 16..17  cell count N            (big-endian)
                    bytes 18..21  event sequence, 32-bit  (big-endian)
                    bytes 22..23  cells before truncation (big-endian)
                    bytes 24..63  reserved
    beats 1..     16 cells each, 4 bytes per cell, last beat partial

A cell is {iy[7:0], iphi[7:0], ecode[15:0]} packed little-endian into its
4 bytes, so ecode occupies the low half-word.

The header count is authoritative; padding past it in the final beat is
ignored. Malformed frames must be swallowed to tlast and counted, never
partially emitted -- that is what a wrong-but-plausible result would look
like downstream.

A length error is different from a bad header, because it is only visible
after cells have already left. Those cases are covered at the bottom of this
file: a short frame must fail the event with cell_err rather than sit waiting
for cells that will never arrive, and it must not swallow the next frame's
header beat. tkeep is assumed contiguous, which is what the shell delivers.
"""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

ETH_TYPE = 0x88B6
FMT_VERSION = 0x01
NMAX = 128
CELLS_PER_BEAT = 16


def make_cell(iy, iphi, ecode):
    """Pack one 4-byte cell, little-endian within the word."""
    return (iy << 24) | (iphi << 16) | (ecode & 0xFFFF)


def build_frame(cells, seq=0, ethtype=ETH_TYPE, count_override=None,
                pad_last_beat=False, version=FMT_VERSION, cells_total=None):
    """Return a list of (data:int, keep:int, last:bool) beats.

    cells_total defaults to the cell count, i.e. no truncation. Passing a
    larger value models the host having cut the event to the NMAX
    highest-pt cells.
    """
    count = len(cells) if count_override is None else count_override
    total = count if cells_total is None else cells_total

    hdr = bytearray(64)
    hdr[0:6] = b'\x02\x11\x22\x33\x44\x55'
    hdr[6:12] = b'\x02\xAA\xBB\xCC\xDD\xEE'
    hdr[12:14] = ethtype.to_bytes(2, 'big')
    hdr[14] = version
    # byte 15 is pad
    hdr[16:18] = count.to_bytes(2, 'big')
    hdr[18:22] = seq.to_bytes(4, 'big')
    hdr[22:24] = total.to_bytes(2, 'big')
    beats = [(int.from_bytes(hdr, 'little'), (1 << 64) - 1, len(cells) == 0)]

    for base in range(0, len(cells), CELLS_PER_BEAT):
        chunk = cells[base:base + CELLS_PER_BEAT]
        n = len(chunk)
        if pad_last_beat:
            chunk = chunk + [0xDEADBEEF] * (CELLS_PER_BEAT - n)
            n = CELLS_PER_BEAT
        data = 0
        for i, c in enumerate(chunk):
            data |= (c & 0xFFFFFFFF) << (32 * i)
        keep = (1 << (4 * n)) - 1
        beats.append((data, keep, base + CELLS_PER_BEAT >= len(cells)))
    return beats


def random_cells(n, rng):
    return [make_cell(rng.randrange(50), rng.randrange(64),
                      rng.randrange(1 << 12)) for _ in range(n)]


async def reset(dut):
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tdata.value = 0
    dut.s_axis_tkeep.value = 0
    dut.s_axis_tlast.value = 0
    dut.cell_ready.value = 0
    dut.aresetn.value = 0
    for _ in range(5):
        await RisingEdge(dut.aclk)
    dut.aresetn.value = 1
    await RisingEdge(dut.aclk)


async def drive_beats(dut, beats, gap=lambda: 0):
    """Feed beats honouring tready, with optional idle gaps between them."""
    for data, keep, last in beats:
        for _ in range(gap()):
            dut.s_axis_tvalid.value = 0
            await RisingEdge(dut.aclk)
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tdata.value = data
        dut.s_axis_tkeep.value = keep
        dut.s_axis_tlast.value = 1 if last else 0
        while True:
            await RisingEdge(dut.aclk)
            if dut.s_axis_tready.value:
                break
        dut.s_axis_tvalid.value = 0
    dut.s_axis_tlast.value = 0


async def collect_cells(dut, n, stall=lambda: False):
    """Pull n cells, optionally deasserting cell_ready to back-pressure."""
    out, starts, lasts = [], [], []
    while len(out) < n:
        ready = not stall()
        dut.cell_ready.value = 1 if ready else 0
        await RisingEdge(dut.aclk)
        if ready and dut.cell_valid.value:
            out.append(int(dut.cell_data.value))
            starts.append(bool(dut.cell_start.value))
            lasts.append(bool(dut.cell_last.value))
    dut.cell_ready.value = 0
    return out, starts, lasts


async def collect_stream(dut, cycles):
    """Sample the cell stream for a fixed window, ready held high.

    Unlike collect_cells this does not know how many cells to expect -- the
    length-error tests are precisely the ones where the count is in dispute --
    so it returns every beat of the handshake, cell_err included.
    """
    out = []
    dut.cell_ready.value = 1
    for _ in range(cycles):
        await RisingEdge(dut.aclk)
        if dut.cell_valid.value:
            out.append((int(dut.cell_data.value),
                        bool(dut.cell_start.value),
                        bool(dut.cell_last.value),
                        bool(dut.cell_err.value)))
    dut.cell_ready.value = 0
    return out


async def start_dut(dut):
    cocotb.start_soon(Clock(dut.aclk, 4, units="ns").start())  # 250 MHz
    await reset(dut)


@cocotb.test()
async def test_single_beat_event(dut):
    """An event smaller than one beat: count and tkeep both partial."""
    await start_dut(dut)
    rng = random.Random(1)
    cells = random_cells(5, rng)

    # cells_total > count models a truncated event: the host had 300 cells
    # above threshold and kept the 5 hardest.
    cocotb.start_soon(drive_beats(dut, build_frame(
        cells, seq=0xDEADBEEF, cells_total=300)))
    got, starts, lasts = await collect_cells(dut, len(cells))

    assert got == cells, f"cells mismatch:\n got {got}\n exp {cells}"
    assert starts == [True] + [False] * 4, f"cell_start wrong: {starts}"
    assert lasts == [False] * 4 + [True], f"cell_last wrong: {lasts}"
    assert int(dut.event_cell_count.value) == 5
    # 32-bit seq: a 16-bit field would have wrapped this to 0xBEEF
    assert int(dut.event_seq.value) == 0xDEADBEEF
    assert int(dut.event_cells_total.value) == 300, "truncation not visible"
    assert int(dut.bad_event_count.value) == 0


@cocotb.test()
async def test_full_event_multi_beat(dut):
    """NMAX cells = 8 full cell beats, the worst case the buffer must hold."""
    await start_dut(dut)
    rng = random.Random(2)
    cells = random_cells(NMAX, rng)

    cocotb.start_soon(drive_beats(dut, build_frame(cells, seq=0x1234)))
    got, starts, lasts = await collect_cells(dut, NMAX)

    assert got == cells, "cells mismatch on full event"
    assert starts[0] and not any(starts[1:])
    assert lasts[-1] and not any(lasts[:-1])
    assert int(dut.event_cell_count.value) == NMAX
    assert int(dut.event_seq.value) == 0x1234
    assert int(dut.bad_event_count.value) == 0


@cocotb.test()
async def test_partial_last_beat(dut):
    """Count not a multiple of 16: tkeep marks the tail of the final beat."""
    await start_dut(dut)
    rng = random.Random(3)
    for n in (17, 31, 33, 127):
        await reset(dut)
        cells = random_cells(n, rng)
        cocotb.start_soon(drive_beats(dut, build_frame(cells)))
        got, _, lasts = await collect_cells(dut, n)
        assert got == cells, f"cells mismatch at n={n}"
        assert lasts[-1], f"cell_last missing at n={n}"
        assert int(dut.bad_event_count.value) == 0


@cocotb.test()
async def test_padded_last_beat_ignored(dut):
    """Header count is authoritative: padding past it must not be emitted."""
    await start_dut(dut)
    rng = random.Random(4)
    cells = random_cells(19, rng)

    cocotb.start_soon(drive_beats(dut, build_frame(cells, pad_last_beat=True)))
    got, _, lasts = await collect_cells(dut, 19)

    assert got == cells, "padding leaked into the cell stream"
    assert lasts[-1]
    assert int(dut.bad_event_count.value) == 0


@cocotb.test()
async def test_backpressure_and_bubbles(dut):
    """Randomised stalls on both sides; beats are not contiguous."""
    await start_dut(dut)
    rng = random.Random(5)
    cells = random_cells(70, rng)

    gap_rng, stall_rng = random.Random(6), random.Random(7)
    cocotb.start_soon(drive_beats(dut, build_frame(cells),
                                  gap=lambda: gap_rng.randrange(0, 4)))
    got, _, lasts = await collect_cells(
        dut, len(cells), stall=lambda: stall_rng.random() < 0.4)

    assert got == cells, "cells corrupted under back-pressure"
    assert lasts[-1]
    assert int(dut.bad_event_count.value) == 0


@cocotb.test()
async def test_back_to_back_events(dut):
    """Two events in succession: state must return cleanly to the header."""
    await start_dut(dut)
    rng = random.Random(8)
    first, second = random_cells(20, rng), random_cells(9, rng)

    async def feed():
        await drive_beats(dut, build_frame(first, seq=1))
        await drive_beats(dut, build_frame(second, seq=2))

    cocotb.start_soon(feed())
    got_a, _, _ = await collect_cells(dut, 20)
    got_b, starts_b, lasts_b = await collect_cells(dut, 9)

    assert got_a == first, "first event corrupted"
    assert got_b == second, "second event corrupted"
    assert starts_b[0], "cell_start missing on the second event"
    assert lasts_b[-1]
    assert int(dut.event_seq.value) == 2
    assert int(dut.bad_event_count.value) == 0


@cocotb.test()
async def test_malformed_rejected(dut):
    """Bad ethertype, zero count and over-NMAX count emit nothing and count."""
    await start_dut(dut)
    rng = random.Random(9)

    bad_frames = [
        build_frame(random_cells(8, rng), ethtype=0x0800),        # wrong type
        build_frame(random_cells(8, rng), count_override=0),      # zero count
        build_frame(random_cells(8, rng), count_override=NMAX + 1),
        # Wrong format version: a host pixelising on a different grid than
        # the LUTs assume must be rejected, not silently misread.
        build_frame(random_cells(8, rng), version=FMT_VERSION + 1),
    ]

    for i, beats in enumerate(bad_frames, start=1):
        cocotb.start_soon(drive_beats(dut, beats))
        dut.cell_ready.value = 1
        emitted = 0
        for _ in range(80):
            await RisingEdge(dut.aclk)
            if dut.cell_valid.value:
                emitted += 1
        dut.cell_ready.value = 0
        assert emitted == 0, f"malformed frame {i} emitted {emitted} cells"
        assert int(dut.bad_event_count.value) == i, (
            f"bad_event_count = {int(dut.bad_event_count.value)}, expected {i}")

    # A good event must still work after the malformed ones.
    cells = random_cells(12, rng)
    cocotb.start_soon(drive_beats(dut, build_frame(cells, seq=99)))
    got, _, lasts = await collect_cells(dut, 12)
    assert got == cells, "recovery after malformed frames failed"
    assert lasts[-1]
    assert int(dut.event_seq.value) == 99
    assert int(dut.event_cells_total.value) == 12, "stale cells_total"
    assert int(dut.bad_event_count.value) == len(bad_frames), (
        "good event counted as bad")


# ---- Length errors ------------------------------------------------------
# The header count and the frame disagree. Unlike a bad header this is only
# discoverable after cells have already been emitted, so it cannot be handled
# by rejecting before the fact.


@cocotb.test()
async def test_short_frame_fails_the_event(dut):
    """tlast before the header count is satisfied.

    The cells already emitted cannot be recalled, so jc_deframe owes
    downstream one cell with cell_err. That is what makes jc_evbuf drop the
    whole event instead of leaving a slot half-filled forever, waiting on a
    pj_last that would never come.
    """
    await start_dut(dut)
    rng = random.Random(21)
    cells = random_cells(16, rng)      # header claims 40, frame carries 16

    cocotb.start_soon(drive_beats(dut, build_frame(cells, seq=7,
                                                   count_override=40)))
    got = await collect_stream(dut, 80)

    real = [g for g in got if not g[3]]
    errs = [g for g in got if g[3]]
    assert [g[0] for g in real] == cells, (
        f"emitted {len(real)} real cells, expected the 16 that were sent")
    assert len(errs) == 1, f"{len(errs)} error cells, expected exactly one"
    assert errs[0][2], "the abort cell must carry cell_last"
    assert not errs[0][1], "the abort cell must not carry cell_start"
    assert not any(g[2] for g in real), "a real cell claimed to end the event"
    assert int(dut.bad_event_count.value) == 1


@cocotb.test()
async def test_short_frame_does_not_swallow_the_next_header(dut):
    """The frame after a short one must still decode as its own event.

    This is the failure the abort path exists for. With tready left high and
    cells still owed, the next frame's HEADER beat was loaded and emitted as
    sixteen cells -- two frames merged into one event that looks entirely
    plausible downstream.
    """
    await start_dut(dut)
    rng = random.Random(22)
    short = random_cells(16, rng)
    good = random_cells(12, rng)

    async def feed():
        await drive_beats(dut, build_frame(short, seq=1, count_override=40))
        await drive_beats(dut, build_frame(good, seq=55))

    cocotb.start_soon(feed())
    got = await collect_stream(dut, 200)

    real = [g for g in got if not g[3]]
    assert [g[0] for g in real[:16]] == short, "short frame's cells changed"
    assert [g[0] for g in real[16:]] == good, (
        f"{len(real) - 16} cells after the abort, expected the 12 of the "
        f"next event -- a swallowed header shows up here as 16 extra")
    assert real[16][1], "the recovered event lost cell_start"
    assert real[-1][2], "the recovered event lost cell_last"
    assert int(dut.event_seq.value) == 55, "header of the second frame not decoded"
    assert int(dut.bad_event_count.value) == 1


@cocotb.test()
async def test_long_frame_emits_the_counted_cells(dut):
    """The count runs out before tlast: the header count is authoritative.

    Not symmetric with the short case. What was emitted is exactly what the
    header promised and nothing is partial, so the event is passed on and the
    surplus flushed -- but the frame is still counted bad, because a sender
    whose framing disagrees with its own header is broken. Failing this one
    would need lookahead: tlast arrives after the final cell has left.
    """
    await start_dut(dut)
    rng = random.Random(23)
    cells = random_cells(32, rng)      # header claims 8, frame carries 32

    cocotb.start_soon(drive_beats(dut, build_frame(cells, seq=3,
                                                   count_override=8)))
    got = await collect_stream(dut, 120)

    assert [g[0] for g in got] == cells[:8], (
        f"{len(got)} cells emitted, expected the 8 the header counted")
    assert not any(g[3] for g in got), "a long frame must not raise cell_err"
    assert got[0][1] and got[-1][2], "start/last wrong on the counted event"
    assert int(dut.bad_event_count.value) == 1

    # The surplus beats must have been flushed, not left to be read as cells.
    good = random_cells(6, rng)
    cocotb.start_soon(drive_beats(dut, build_frame(good, seq=77)))
    tail = await collect_stream(dut, 80)
    assert [g[0] for g in tail] == good, "recovery after a long frame failed"
    assert int(dut.event_seq.value) == 77


@cocotb.test()
async def test_zero_keep_beat_is_not_sixteen_cells(dut):
    """A beat with no valid bytes carries no cells.

    cells_in_beat is popcount(tkeep)/4 and cell_idx_last is one less, so
    tkeep = 0 underflows to 15 and emits sixteen words of a beat that holds
    nothing. Illegal on the wire, but the cost of not checking is an event
    built from padding.
    """
    await start_dut(dut)
    rng = random.Random(24)
    cells = random_cells(16, rng)

    # Mid-event: cells have already left, so this must fail the event.
    beats = build_frame(cells, seq=9, count_override=40)
    beats[1] = (beats[1][0], beats[1][1], False)
    beats.append((0, 0, True))
    cocotb.start_soon(drive_beats(dut, beats))
    got = await collect_stream(dut, 120)

    real = [g for g in got if not g[3]]
    errs = [g for g in got if g[3]]
    assert [g[0] for g in real] == cells, (
        f"{len(real)} real cells, expected 16 -- an empty beat became cells")
    assert len(errs) == 1 and errs[0][2], "empty beat did not fail the event"
    assert int(dut.bad_event_count.value) == 1

    # As the first cell beat: nothing has escaped, so it is rejected outright
    # the way a bad header is, with no abort cell at all.
    hdr_data, hdr_keep, _ = build_frame([], seq=10, count_override=40)[0]
    cocotb.start_soon(drive_beats(dut, [(hdr_data, hdr_keep, False),
                                        (0, 0, True)]))
    got = await collect_stream(dut, 80)
    assert got == [], f"{len(got)} cells emitted from a frame with none"
    assert int(dut.bad_event_count.value) == 2

    good = random_cells(9, rng)
    cocotb.start_soon(drive_beats(dut, build_frame(good, seq=11)))
    tail = await collect_stream(dut, 80)
    assert [g[0] for g in tail] == good, "recovery after an empty beat failed"
