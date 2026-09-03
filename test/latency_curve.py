#!/usr/bin/env python3
"""Engine latency against cell count, from a capture already on disk.

    ./latency_curve.py -f /tmp/jet_v3_1k.pcap --pkt /scratch/chettige/cells1k.pkt.bin

WHY THIS NEEDS NO CARD TIME. The obvious way to ask "what would N=64 cost?" is
to re-pixelise at --nmax 64 and inject. That is not necessary: NMAX is a build
parameter but the LIVE cell count is not, and a normal dataset already spans
the range. cells1k runs n=5..128 with at least 50 events in every 16-wide bin,
and every jets frame carries its own `cycles` in the header. So one ordinary
1000-event capture is already a thousand samples of the latency-versus-n curve,
measured on silicon.

WHAT IT CANNOT TELL YOU, and the distinction matters. An event that naturally
has 64 cells and an event truncated from 100 to 64 look identical to the
engine, so this curve answers the LATENCY half of the N question exactly. It
says nothing about the PHYSICS half -- how much pt is thrown away by
truncating -- which comes from pixelize.py's stats.json at the same --nmax and
is a separate run. Both halves are needed before anyone chooses an N.

THE FIT IS THE POINT, not the bins. Cycles per cell rises with n (118.9 at
n=35, 128.9 at n=128), which means the total is superlinear and a single
"cycles per cell" number is misleading. Fitting a + b*n + c*n^2 separates the
fixed per-event cost from the per-cell cost from whatever genuinely scales with
n^2, and only the last of those would make halving N worth more than half.
"""

import argparse
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "model"))
sys.path.insert(0, str(ROOT / "test"))

import jc_model as M                                            # noqa: E402
from jc_frames import parse_jets_bytes, ETH_TYPE_OUT, ShortFrame  # noqa: E402
from jc_verify import read_pcap                                 # noqa: E402

CLK_MHZ = 250.0


def solve3(rows, rhs):
    """Tiny Gaussian elimination -- numpy is not installed for every account."""
    n = len(rhs)
    m = [list(r) + [v] for r, v in zip(rows, rhs)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-12:
            return None
        m[c], m[p] = m[p], m[c]
        for r in range(n):
            if r == c:
                continue
            f = m[r][c] / m[c][c]
            for k in range(c, n + 1):
                m[r][k] -= f * m[c][k]
    return [m[i][n] / m[i][i] for i in range(n)]


def polyfit2(xs, ys):
    """Least squares a + b*x + c*x^2, and the R^2 that says whether to believe it."""
    s = [sum(x ** k for x in xs) for k in range(5)]
    rhs = [sum(y * x ** k for x, y in zip(xs, ys)) for k in range(3)]
    coef = solve3([[s[i + j] for j in range(3)] for i in range(3)], rhs)
    if coef is None:
        return None, 0.0
    a, b, c = coef
    mean = sum(ys) / len(ys)
    ss_tot = sum((y - mean) ** 2 for y in ys)
    ss_res = sum((y - (a + b * x + c * x * x)) ** 2 for x, y in zip(xs, ys))
    return coef, (1.0 - ss_res / ss_tot if ss_tot else 1.0)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-f", "--file", type=pathlib.Path, required=True,
                    help="pcap from the sink")
    ap.add_argument("--pkt", type=pathlib.Path,
                    default=pathlib.Path("/scratch/chettige/cells1k.pkt.bin"),
                    help="the .pkt.bin that was injected")
    ap.add_argument("--bin", type=int, default=16, help="histogram bin width")
    ap.add_argument("--min-events", type=int, default=20,
                    help="hide bins thinner than this. They stay in the fit -- "
                         "they are real events -- but a bin mean over a handful "
                         "of them is noise, and printing it next to bins of 150 "
                         "invites reading a trend that is not there")
    ap.add_argument("--at", type=int, nargs="*", default=[64, 96, 128],
                    help="cell counts to project the fit at")
    args = ap.parse_args()

    if not args.pkt.exists():
        sys.exit("no such packet file: %s" % args.pkt)

    n_by_seq = {seq: len(cells) for seq, _t, cells in M.read_packets(args.pkt)}

    pts, short = [], 0
    for pkt in read_pcap(args.file):
        if len(pkt) < 14 or int.from_bytes(pkt[12:14], "big") != ETH_TYPE_OUT:
            continue
        try:
            f = parse_jets_bytes(pkt)
        except ShortFrame:
            short += 1
            continue
        n = n_by_seq.get(f["seq"])
        if n:
            pts.append((n, f["cycles"], f["njets"]))

    if len(pts) < 8:
        sys.exit("only %d usable frames -- need a real capture" % len(pts))

    xs = [n for n, _c, _j in pts]
    ys = [c for _n, c, _j in pts]

    print("\n%d frames paired with their cell count%s"
          % (len(pts), ", %d truncated and skipped" % short if short else ""))

    # ---- the curve -------------------------------------------------------
    print("\n%6s %7s %10s %9s %8s" % ("n", "events", "cycles", "cyc/cell", "us"))
    bins = {}
    for n, c, _j in pts:
        bins.setdefault((n // args.bin) * args.bin, []).append(c)
    hidden = 0
    for lo in sorted(bins):
        v = bins[lo]
        if len(v) < args.min_events:
            hidden += len(v)
            continue
        mean = sum(v) / len(v)
        mid = lo + args.bin / 2.0
        print("%3d-%-3d %6d %10.0f %9.1f %8.1f"
              % (lo, lo + args.bin - 1, len(v), mean, mean / mid,
                 mean / CLK_MHZ))
    if hidden:
        print("  (%d events in bins under %d, hidden from the table but kept "
              "in the fit)" % (hidden, args.min_events))

    # ---- the fit ---------------------------------------------------------
    coef, r2 = polyfit2(xs, ys)
    if coef is None:
        sys.exit("fit failed -- degenerate data")
    a, b, c = coef
    print("\ncycles ~ %.1f %+.2f*n %+.5f*n^2      R^2 = %.5f" % (a, b, c, r2))
    quad = abs(c) * 128 * 128
    print("  fixed per event : %8.0f cycles" % a)
    print("  linear in n     : %8.0f at n=128" % (b * 128))
    print("  quadratic in n  : %8.0f at n=128  (%.1f%% of the total)"
          % (quad, 100.0 * quad / max(a + b * 128 + c * 128 * 128, 1)))
    if quad < 0.05 * (a + b * 128):
        print("  -> essentially LINEAR: halving N halves the latency, no more.")
    else:
        print("  -> superlinear: halving N is worth MORE than half.")

    # ---- what N would buy ------------------------------------------------
    print("\nprojected from the fit:")
    print("%6s %10s %9s" % ("n", "cycles", "us"))
    for n in sorted(args.at):
        pred = a + b * n + c * n * n
        print("%6d %10.0f %9.1f" % (n, pred, pred / CLK_MHZ))

    print("\nThis is the LATENCY half of the N question only. The pt cost of\n"
          "truncating comes from pixelize.py --nmax <N> and its stats.json.")


if __name__ == "__main__":
    main()
