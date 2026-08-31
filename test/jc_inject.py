#!/usr/bin/env python3
"""Inject cell events onto the wire from the source card. Run as root.

    sudo ./jc_inject.py -i enp37s0f0 --dst 00:0a:35:6f:09:8e -n 100

Frames are built by model/jc_frames.py's build_event_frame -- THE SAME
FUNCTION THE COCOTB BENCHES USE. That is the point: if the hardware disagrees
with the model, it cannot be because the host and the simulation disagreed
about the wire format. pixelize.py wrote those bytes and this replays them.

Raw AF_PACKET, no scapy. `sudo` runs root's Python, not yours, so a scapy
dependency is one more thing to install as root before anything works.

PACING MATTERS AND THE DEFAULT IS DELIBERATE. One event is ~16.5k cycles at
250 MHz, about 66 us, and jc_evbuf has two slots and never blocks -- it drops
whole events instead, because a device in the network path cannot stall the
shell's RX. Firing faster than the engine clusters is therefore a legitimate
measurement (drop_full over accept IS the duty cycle, and what sizes engine
count for step 10) but it is NOT a correctness test. The default gap leaves
the engine idle between events so every one is accepted.
"""

import argparse
import pathlib
import socket
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "model"))

import jc_model as M                                          # noqa: E402
from jc_frames import build_event_frame, ETH_TYPE_IN          # noqa: E402

DEFAULT_PKT = pathlib.Path("/scratch/chettige/cells1k.pkt.bin")


def mac_bytes(s):
    parts = s.replace("-", ":").split(":")
    if len(parts) != 6:
        raise argparse.ArgumentTypeError("expected aa:bb:cc:dd:ee:ff")
    return bytes(int(p, 16) for p in parts)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-i", "--iface", required=True,
                    help="source interface, e.g. enp37s0f0")
    ap.add_argument("--dst", type=mac_bytes, required=True,
                    help="the PLUGIN card's ingest port MAC (B's f0)")
    ap.add_argument("--src", type=mac_bytes, default=b"\x00" * 6,
                    help="source MAC written into the frame (cosmetic)")
    ap.add_argument("-n", "--count", type=int, default=100,
                    help="events to send (default 100)")
    ap.add_argument("--first", type=int, default=0,
                    help="skip this many events from the file first")
    ap.add_argument("--pkt", type=pathlib.Path, default=DEFAULT_PKT,
                    help="pixelize.py .pkt.bin (default %s)" % DEFAULT_PKT)
    ap.add_argument("--gap", type=float, default=0.002,
                    help="seconds between events (default 0.002 -- ~30x one "
                         "event, so nothing is dropped). 0 = as fast as "
                         "possible, which measures duty cycle, not correctness")
    args = ap.parse_args()

    if not args.pkt.exists():
        sys.exit("no such packet file: %s\n"
                 "  build one with: python3 model/pixelize.py <pythia.dat> ..."
                 % args.pkt)

    events = []
    for i, (seq, _total, cells) in enumerate(M.read_packets(args.pkt)):
        if i < args.first:
            continue
        events.append((seq, cells))
        if len(events) >= args.count:
            break
    if not events:
        sys.exit("no events read from %s" % args.pkt)

    try:
        s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW)
        s.bind((args.iface, 0))
    except PermissionError:
        sys.exit("AF_PACKET needs root -- run under sudo")
    except OSError as e:
        sys.exit("cannot bind %s (%s)" % (args.iface, e))

    nbytes = 0
    t0 = time.time()
    for seq, cells in events:
        frame = build_event_frame(cells, seq, dst_mac=args.dst,
                                  src_mac=args.src)
        s.send(frame)
        nbytes += len(frame)
        if args.gap:
            time.sleep(args.gap)
    dt = time.time() - t0

    sizes = [len(c) for _, c in events]
    print("sent %d events (%d B) on %s in %.2f s"
          % (len(events), nbytes, args.iface, dt))
    print("  ethertype 0x%04X, dst %s"
          % (ETH_TYPE_IN, ":".join("%02x" % b for b in args.dst)))
    print("  seq %d..%d, cells min/med/max %d/%d/%d"
          % (events[0][0], events[-1][0], min(sizes),
             sorted(sizes)[len(sizes) // 2], max(sizes)))
    print("\nNow read the ladder on the plugin card:")
    print("  sudo ./jc_regs.py <B bdf> dump      # frames_in should be %d"
          % len(events))


if __name__ == "__main__":
    main()
