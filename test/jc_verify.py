#!/usr/bin/env python3
"""Check captured jet frames against jc_model.py. Run as root (pcap perms).

    sudo tcpdump -i enp193s0f0 -w /tmp/jets.pcap 'ether proto 0x88b7' &
    sudo ./jc_inject.py -i enp37s0f0 --dst <B f0 MAC> -n 100
    sudo ./jc_verify.py -f /tmp/jets.pcap --floor 0

THE COMPARISON IS THE SAME ONE THE BENCHES MAKE. Frames are decoded by
model/jc_frames.py's parse_jets_bytes and compared against
jc_model.cluster_fixed over the identical cells from the same .pkt.bin, so a
hardware pass means exactly what a simulation pass means. Bit-exact, not
tolerance-based: the model's four-momenta are integers over 2^34 and the card
sends fp32, so the expected payload is a plain float32 round-trip.

Matched by SEQUENCE NUMBER, not arrival order. Events can be dropped -- that
is jc_evbuf working as designed, not a failure -- so the check is that every
frame which DID arrive is correct, reported alongside how many never came.

pcap is parsed by hand rather than with scapy: `sudo` runs root's Python, and
one less root-side dependency is one less thing to install before a result.
"""

import argparse
import pathlib
import struct
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "model"))

import jc_model as M                                          # noqa: E402
from jc_frames import (parse_jets_bytes, expected_jets,       # noqa: E402
                       ETH_TYPE_OUT, ShortFrame)

DEFAULT_PKT = pathlib.Path("/scratch/chettige/cells1k.pkt.bin")
FMT = M.Formats(ROOT / "model" / "luts.json")
import json                                                   # noqa: E402
LUTS = json.loads((ROOT / "model" / "luts.json").read_text())
DEFAULT_R = LUTS["default_r"]


def read_pcap(path):
    """Yield packet payloads from a classic pcap file."""
    b = path.read_bytes()
    if len(b) < 24:
        sys.exit("%s is too short to be a pcap" % path)
    magic = b[:4]
    if magic in (b"\xa1\xb2\xc3\xd4", b"\xa1\xb2\x3c\x4d"):
        end = ">"
    elif magic in (b"\xd4\xc3\xb2\xa1", b"\x4d\x3c\xb2\xa1"):
        end = "<"
    elif magic == b"\x0a\x0d\x0d\x0a":
        sys.exit("%s is pcapng, not classic pcap. Re-capture with "
                 "`tcpdump -w` (which writes pcap), or convert it." % path)
    else:
        sys.exit("%s is not a pcap (magic %s)" % (path, magic.hex()))

    off = 24
    while off + 16 <= len(b):
        _ts, _us, incl, _orig = struct.unpack(end + "IIII", b[off:off + 16])
        off += 16
        yield b[off:off + incl]
        off += incl


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-f", "--file", type=pathlib.Path, required=True,
                    help="pcap from the sink")
    ap.add_argument("--pkt", type=pathlib.Path, default=DEFAULT_PKT,
                    help="the .pkt.bin that was injected")
    ap.add_argument("--first", type=int, default=0,
                    help="first event injected -- must match jc_inject.py")
    ap.add_argument("-n", "--count", type=int, default=None,
                    help="how many were injected. Without it, every event in "
                         "the file counts as expected and the missing-frame "
                         "line reports the whole dataset, which is noise")
    ap.add_argument("--floor", type=float, default=0.0,
                    help="jet pt floor in GeV, must match what jc_regs holds")
    ap.add_argument("--r", type=float, default=None,
                    help="R in radians (default: the generated %.2f)"
                         % DEFAULT_R)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    r_rad = DEFAULT_R if args.r is None else args.r

    if not args.pkt.exists():
        sys.exit("no such packet file: %s" % args.pkt)

    cells_by_seq = {}
    for i, (seq, _t, cells) in enumerate(M.read_packets(args.pkt)):
        if i < args.first:
            continue
        cells_by_seq[seq] = cells
        if args.count is not None and len(cells_by_seq) >= args.count:
            break

    # A truncated frame is a CAPTURE artifact, not a card fault -- tcpdump
    # returns before flushing its last record, so killing it promptly clips
    # one frame. Report those and carry on rather than discarding a run that
    # may have taken half an hour to inject.
    frames, wrong_type, short = [], 0, []
    for pkt in read_pcap(args.file):
        if len(pkt) < 14:
            continue                     # too short to even carry an ethertype
        if int.from_bytes(pkt[12:14], "big") != ETH_TYPE_OUT:
            wrong_type += 1
            continue
        try:
            frames.append(parse_jets_bytes(pkt))
        except ShortFrame as e:
            short.append(str(e))

    if not frames:
        sys.exit("no complete 0x%04X frames in %s (%d of another ethertype, "
                 "%d truncated).\n"
                 "  If the capture is empty: check the ladder with "
                 "`jc_regs.py <B> dump` -- the first counter reading zero is "
                 "the stage that stopped." % (ETH_TYPE_OUT, args.file,
                                              wrong_type, len(short)))

    bad, ok, unknown, cycles = [], 0, [], []
    for f in frames:
        seq = f["seq"]
        if seq not in cells_by_seq:
            unknown.append(seq)
            continue
        want = expected_jets(
            M.cluster_fixed(cells_by_seq[seq], FMT, r_rad, args.floor))
        got = sorted(f["jets"])
        cycles.append((len(cells_by_seq[seq]), f["cycles"]))
        if got == want:
            ok += 1
            if args.verbose:
                print("seq %-6d n=%-4d %2d jets  %6d cycles  OK"
                      % (seq, len(cells_by_seq[seq]), f["njets"], f["cycles"]))
        else:
            bad.append((seq, got, want))

    print("\n%d frames captured, %d matched the model, %d differed"
          % (len(frames), ok, len(bad)))
    if unknown:
        print("  %d frames carried a seq not in %s: %s"
              % (len(unknown), args.pkt.name, unknown[:8]))
    if short:
        print("  %d truncated frame(s) skipped -- almost always tcpdump "
              "killed before it flushed, not a card fault:" % len(short))
        for s in short[:3]:
            print("    %s" % s)

    sent_seqs = set(cells_by_seq)
    seen = {f["seq"] for f in frames}
    # Silence is not necessarily loss: an event whose jets are all below the
    # floor emits nothing by design. Only the model can tell the two apart.
    missing_with_jets = [s for s in sorted(sent_seqs - seen)
                         if M.cluster_fixed(cells_by_seq[s], FMT, r_rad,
                                            args.floor)]
    if missing_with_jets:
        print("  %d events had jets above the floor but no frame arrived: %s"
              % (len(missing_with_jets), missing_with_jets[:8]))
        print("    -> check drop_full/drop_err in `jc_regs.py <B> dump`; "
              "if drop_full is high the injector outran the engine (--gap)")

    if cycles:
        per_cell = [c / n for n, c in cycles if n]
        print("  cycles/event: min %d, max %d, %.1f/cell mean"
              % (min(c for _, c in cycles), max(c for _, c in cycles),
                 sum(per_cell) / len(per_cell)))

    # These ride in the frame header, sampled on the aclk side at jet_eoe, so
    # they are readable even when jc_regs' snapshot crossing is not. drop_full
    # against accepted events IS the duty cycle -- the figure step 10 sizes
    # engine replication from.
    last = frames[-1]
    print("  from the last frame's header: drop_full=%d drop_err=%d "
          "bad_frame=%d" % (last["drop_full"], last["drop_err"],
                            last["bad_frame"]))
    if last["drop_full"]:
        print("    -> %d events found no free slot. Expected if --gap is small;"
              " that ratio is the duty cycle, not a fault."
              % last["drop_full"])

    for seq, got, want in bad[:3]:
        print("\nseq %d DIFFERS" % seq)
        print("  got  %d jets: %s" % (len(got), got[:4]))
        print("  want %d jets: %s" % (len(want), want[:4]))

    if bad:
        sys.exit(1)
    print("\nPASS -- every captured frame is bit-exact against jc_model.py")


if __name__ == "__main__":
    main()
