#!/usr/bin/env python3
"""Cut a handful of real events out of a pixelised dataset into a fixture.

The engine bench's headline result -- exact agreement with the model, and the
cycles/event number that step 9 goes to hardware to confirm -- came from real
Pythia showers, because the distance distribution and therefore the stale-row
count per merge are nothing like a synthetic event's. But it read them from
/scratch, so on any other machine the result silently evaporated.

This writes the same events, byte for byte, into a file small enough to commit
next to the bench. Packets are copied whole rather than re-serialised: the
reserved header bytes and the MACs are part of what the deframe path will one
day see, and a fixture that quietly normalises them is a fixture that stops
being a sample of the real thing.

    python3 model/make_fixture.py \\
        /scratch/chettige/cells1k.pkt.bin \\
        box_250mhz/tb/unit/data/events.pkt.bin

Regenerate it whenever JC_FMT_VERSION changes -- the version byte is the only
thing standing between a stale fixture and a bench that tests the wrong grid.
"""

import argparse
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import jc_model as M          # noqa: E402  (needs the path above)

# One event per size band, matching what test_jc_engine.py asks for. The top
# band is the one that matters -- stale-row count grows with N, so the repair
# paths that dominate a real event barely show up below it.
BANDS = [(20, 40), (60, 80), (100, 128)]

# And one event that actually has a jet above the trigger floor, which is NOT
# implied by any of the bands: this sample is soft, and only 3 of its first 400
# events have a jet over 50 GeV at all. Picking by size gave the floor test an
# event whose hardest jet was 12 GeV, so it asserted that nothing equalled
# nothing. Finding this one means clustering, which is why it is the slow pass.
TRIGGER_FLOOR_GEV = 50.0
R_RAD = 0.4


def packet_spans(path, limit):
    """Walk the file yielding (seq, n_cells, byte_slice) for each packet.

    read_packets decodes but does not report offsets, and the point here is to
    copy bytes rather than re-encode them, so the walk is repeated with the
    one extra piece of bookkeeping it needs.
    """
    blob = path.read_bytes()
    off = 0
    for _ in range(limit):
        if off >= len(blob):
            break
        hdr = blob[off:off + M.HDR_BYTES]
        if len(hdr) < M.HDR_BYTES:
            raise ValueError(f"truncated header at byte {off}")
        if int.from_bytes(hdr[12:14], "big") != M.ETH_TYPE:
            raise ValueError(f"bad ethertype at byte {off}")
        if hdr[14] != M.FMT_VERSION:
            raise ValueError(
                f"format version {hdr[14]} at byte {off}, expected "
                f"{M.FMT_VERSION} -- regenerate the dataset")
        count = int.from_bytes(hdr[16:18], "big")
        seq = int.from_bytes(hdr[18:22], "big")
        end = off + M.HDR_BYTES + 4 * count
        yield seq, count, blob[off:end]
        off = end


def cells_of(raw):
    """Decode one packet's cells, the same unpacking read_packets does."""
    out = []
    for off in range(M.HDR_BYTES, len(raw), 4):
        word = int.from_bytes(raw[off:off + 4], "little")
        out.append(((word >> 24) & 0xFF, (word >> 16) & 0xFF, word & 0xFFFF))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("source", type=pathlib.Path)
    ap.add_argument("dest", type=pathlib.Path)
    ap.add_argument("--limit", type=int, default=400,
                    help="events to scan for the bands (default 400)")
    ap.add_argument("--floor", type=float, default=TRIGGER_FLOOR_GEV,
                    help="jet pt the extra event must exceed (default 50 GeV)")
    # Named events, for when hardware or a longer run disagrees with the model
    # on ONE event and the question is whether simulation disagrees too. The
    # bands cannot express "that one", and cutting the specific event is the
    # difference between debugging it offline and re-injecting on the card.
    ap.add_argument("--seq", type=int, nargs="+", default=None,
                    help="cut exactly these sequence numbers instead of the "
                         "size bands, e.g. --seq 18")
    args = ap.parse_args()

    spans = list(packet_spans(args.source, args.limit))

    if args.seq is not None:
        by_seq = {s: (s, c, raw) for s, c, raw in spans}
        missing = [s for s in args.seq if s not in by_seq]
        if missing:
            sys.exit(f"seq {missing} not in the first {args.limit} events of "
                     f"{args.source} -- raise --limit")
        chosen = [by_seq[s] for s in args.seq]
        args.dest.parent.mkdir(parents=True, exist_ok=True)
        args.dest.write_bytes(b"".join(raw for _s, _c, raw in chosen))
        for s, c, _raw in chosen:
            print(f"  seq {s}: {c} cells")
        print(f"wrote {args.dest} ({args.dest.stat().st_size} B)")
        return

    picked = {}
    for seq, count, raw in spans:
        for band in BANDS:
            if band[0] <= count <= band[1] and band not in picked:
                picked[band] = (seq, count, raw)
        if len(picked) == len(BANDS):
            break

    missing = [b for b in BANDS if b not in picked]
    if missing:
        raise SystemExit(
            f"no event in {missing} within the first {args.limit} of "
            f"{args.source} -- raise --limit or use a larger dataset")

    # The slow pass. Clustering in Python is not fast and most of the sample
    # is too soft to qualify, so this walks a long way before it hits one.
    fmt = M.Formats(HERE / "luts.json")
    already = {v[0] for v in picked.values()}
    hard = None
    print(f"scanning for an event with a jet over {args.floor:g} GeV ...")
    for seq, count, raw in spans:
        if seq in already:
            continue
        if M.cluster_fixed(cells_of(raw), fmt, R_RAD, args.floor):
            hard = (seq, count, raw)
            break
    if hard is None:
        raise SystemExit(
            f"no event with a jet over {args.floor:g} GeV in the first "
            f"{args.limit} of {args.source} -- raise --limit, or lower "
            f"--floor and change the bench to match")

    args.dest.parent.mkdir(parents=True, exist_ok=True)
    with args.dest.open("wb") as f:
        for band in BANDS:
            seq, count, raw = picked[band]
            f.write(raw)
            print(f"  band {band[0]:3d}-{band[1]:3d}: seq {seq}, "
                  f"n={count}, {len(raw)} bytes")
        seq, count, raw = hard
        f.write(raw)
        print(f"  over {args.floor:g} GeV: seq {seq}, n={count}, "
              f"{len(raw)} bytes")
    print(f"wrote {args.dest} ({args.dest.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
