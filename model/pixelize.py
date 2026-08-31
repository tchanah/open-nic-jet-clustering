#!/usr/bin/env python3
"""Pythia particles -> calorimeter cells -> the two aligned test inputs.

Emits both halves of every comparative test from ONE decision path, so the
hardware and FastJet cluster bit-identical inputs and any difference between
them is arithmetic rather than data:

    <out>.cells.dat   cells as four-momenta, in the SAME binary format as the
                      input, so jc_ref.cc / reference_fastjet.cc read it with
                      no changes
    <out>.pkt.bin     the same cells in our wire format, for cocotb and
                      eventually the wire
    <out>.stats.json  per-event bookkeeping, including what truncation cost

Input format (confirmed against the bytes, little-endian throughout):
    uint32                 number of events
    per event:
        uint16             number of particles n
        n x 4 x float32    E, px, py, pz

Pixelisation
------------
Particles carry real mass -- pions at 0.14 GeV, kaons at 0.494. Cells do not,
and that is the point: a tower sums deposited ENERGY, sits at its geometric
centre, and its four-momentum is rebuilt massless from (E, eta, phi). The mass
disappears in the binning, which is why "inputs are massless" holds for the
hardware even though the generator's particles are not.

Binning is by eta, not y, because a detector is segmented geometrically. For
the massless tower that comes out, y == eta anyway.

Energies round-trip through the REAL energy table in luts.json rather than an
analytic formula, so the cells written for FastJet are exactly the values
jc_ingest will produce from the same codes. Quantisation is then common to
both sides and cancels out of the comparison.

Truncation
----------
Events over NMAX cells keep the NMAX highest-pt, deterministically ordered by
(-pt, iy, iphi) so ties cannot make the dataset depend on dict ordering. On
the 1k file this fires for ~13% of events and costs a median 3% of event pt --
a bound on truth, not a verification problem, since FastJet sees the same
surviving cells.
"""

import argparse
import json
import math
import pathlib
import struct

HERE = pathlib.Path(__file__).resolve().parent

ETH_TYPE = 0x88B6
FMT_VERSION = 0x01
HDR_BYTES = 64
DST_MAC = b"\x02\x11\x22\x33\x44\x55"
SRC_MAC = b"\x02\xAA\xBB\xCC\xDD\xEE"


def load_luts(path):
    luts = json.loads(path.read_text())
    fmt = luts["formats"]["jc_lut_energy"]
    width, frac = fmt["width"], fmt["frac"]

    def to_signed(w):
        return w - (1 << width) if w >> (width - 1) else w

    # The decoded table IS what jc_ingest emits for each code.
    energy = [to_signed(w) / float(1 << frac)
              for w in luts["words"]["jc_lut_energy"]]
    return luts, energy


def encode_energy(e_gev, energy_table):
    """Nearest code in log space, which is how the table is spaced."""
    lo, hi = 0, len(energy_table) - 1
    if e_gev <= energy_table[0]:
        return 0
    if e_gev >= energy_table[hi]:
        return hi
    # Binary search for the bracketing pair, then pick the nearer in log space.
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if energy_table[mid] <= e_gev:
            lo = mid
        else:
            hi = mid
    d_lo = math.log(e_gev / energy_table[lo])
    d_hi = math.log(energy_table[hi] / e_gev)
    return lo if d_lo <= d_hi else hi


def read_events(path, limit=None):
    b = path.read_bytes()
    nev = struct.unpack_from("<I", b, 0)[0]
    off = 4
    for i in range(nev):
        n = struct.unpack_from("<H", b, off)[0]
        off += 2
        vals = struct.unpack_from("<%df" % (4 * n), b, off)
        off += 16 * n
        if limit is not None and i >= limit:
            return
        yield i, vals, n


def pixelize(vals, n, cfg):
    """Particles -> {(iy, iphi): summed E}, applying the acceptance cut."""
    grid = {}
    for k in range(n):
        e, px, py, pz = vals[4 * k:4 * k + 4]
        p = math.sqrt(px * px + py * py + pz * pz)
        if p <= abs(pz):
            continue                      # exactly along the beam, no eta
        eta = 0.5 * math.log((p + pz) / (p - pz))
        if abs(eta) >= cfg["eta_max"]:
            continue
        phi = math.atan2(py, px) % (2.0 * math.pi)
        iy = min(cfg["rap_bins"] - 1,
                 int((eta + cfg["eta_max"]) / cfg["deta"]))
        iphi = int(phi / (2.0 * math.pi / cfg["phi_bins"])) % cfg["phi_bins"]
        key = (iy, iphi)
        grid[key] = grid.get(key, 0.0) + e
    return grid


def make_cells(grid, cfg, energy_table):
    """Quantise, threshold and truncate. Returns (cells, total_above_thresh).

    A cell is (iy, iphi, ecode, pt, E) where E is the DECODED energy -- the
    value the hardware will reconstruct, and therefore the value FastJet must
    be given.
    """
    cells = []
    for (iy, iphi), e_sum in grid.items():
        y = cfg["y_lo"] + cfg["deta"] * iy          # bin centre
        code = encode_energy(e_sum, energy_table)
        e_dec = energy_table[code]
        pt = e_dec / math.cosh(y)
        if pt <= cfg["pt_min"]:
            continue
        cells.append((iy, iphi, code, pt, e_dec))

    total = len(cells)
    pt_all = sum(c[3] for c in cells)
    # Deterministic: hardest first, ties broken by position so the dataset
    # never depends on dict iteration order.
    cells.sort(key=lambda c: (-c[3], c[0], c[1]))
    cells = cells[:cfg["nmax"]]
    pt_kept = sum(c[3] for c in cells)
    # Emit in grid order so the packet is a function of the event alone.
    cells.sort(key=lambda c: (c[0], c[1]))
    return cells, total, pt_kept, pt_all


def cell_four_momentum(cell, cfg):
    """Massless tower at its bin centre, built exactly as jc_ingest does."""
    iy, iphi, _code, _pt, e = cell
    y = cfg["y_lo"] + cfg["deta"] * iy
    phi = 2.0 * math.pi * iphi / cfg["phi_bins"]
    pt = e / math.cosh(y)
    return (e, pt * math.cos(phi), pt * math.sin(phi), e * math.tanh(y))


def build_packet(seq, cells, cells_total, cfg):
    hdr = bytearray(HDR_BYTES)
    hdr[0:6] = DST_MAC
    hdr[6:12] = SRC_MAC
    hdr[12:14] = ETH_TYPE.to_bytes(2, "big")
    hdr[14] = FMT_VERSION
    hdr[16:18] = len(cells).to_bytes(2, "big")
    hdr[18:22] = (seq & 0xFFFFFFFF).to_bytes(4, "big")
    hdr[22:24] = min(cells_total, 0xFFFF).to_bytes(2, "big")

    body = bytearray()
    for iy, iphi, code, _pt, _e in cells:
        word = ((iy & 0xFF) << 24) | ((iphi & 0xFF) << 16) | (code & 0xFFFF)
        body += word.to_bytes(4, "little")     # cell index is word index
    return bytes(hdr) + bytes(body)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", type=pathlib.Path,
                    help="pythia .dat, e.g. /scratch/chettige/pythiaEvents1k14TeV.dat")
    ap.add_argument("-o", "--out", type=pathlib.Path, required=True,
                    help="output prefix; .cells.dat/.pkt.bin/.stats.json are appended")
    ap.add_argument("--luts", type=pathlib.Path, default=HERE / "luts.json")
    ap.add_argument("--limit", type=int, default=None,
                    help="only process the first N events")
    ap.add_argument("--nmax", type=int, default=128)
    ap.add_argument("--pt-min", type=float, default=0.2,
                    help="cell pt floor in GeV (default: %(default)s)")
    args = ap.parse_args()

    luts, energy_table = load_luts(args.luts)
    cfg = {
        "rap_bins": luts["rap_bins"],
        "phi_bins": luts["phi_bins"],
        "eta_max": 2.5,
        "deta": 0.1,
        "y_lo": -2.45,          # centre of bin 0
        "pt_min": args.pt_min,
        "nmax": args.nmax,
    }

    cells_path = args.out.with_suffix(args.out.suffix + ".cells.dat")
    pkt_path = args.out.with_suffix(args.out.suffix + ".pkt.bin")
    stats_path = args.out.with_suffix(args.out.suffix + ".stats.json")

    events, per_event = [], []
    packets = bytearray()
    truncated = 0
    lost = []

    for seq, vals, n in read_events(args.input, args.limit):
        grid = pixelize(vals, n, cfg)
        cells, total, pt_kept, pt_all = make_cells(grid, cfg, energy_table)
        if not cells:
            per_event.append({"seq": seq, "raw": n, "cells": 0, "total": total,
                              "pt_lost": 0.0})
            continue

        p4 = [cell_four_momentum(c, cfg) for c in cells]
        events.append(p4)
        packets += build_packet(seq, cells, total, cfg)

        pt_lost = 1.0 - pt_kept / pt_all if pt_all > 0 else 0.0
        if total > len(cells):
            truncated += 1
            lost.append(pt_lost)
        per_event.append({"seq": seq, "raw": n, "cells": len(cells),
                          "total": total, "pt_lost": pt_lost})

    # cells.dat mirrors the input container exactly, so existing FastJet
    # tooling reads it with no changes.
    with cells_path.open("wb") as f:
        f.write(struct.pack("<I", len(events)))
        for p4 in events:
            f.write(struct.pack("<H", len(p4)))
            for e, px, py, pz in p4:
                f.write(struct.pack("<4f", e, px, py, pz))

    pkt_path.write_bytes(bytes(packets))

    sizes = [d["cells"] for d in per_event if d["cells"]]
    stats = {
        "source": str(args.input),
        "events_written": len(events),
        "events_empty": sum(1 for d in per_event if not d["cells"]),
        "events_truncated": truncated,
        "nmax": args.nmax,
        "pt_min": args.pt_min,
        "energy_min_gev": luts["energy_min_gev"],
        "energy_max_gev": luts["energy_max_gev"],
        "cells_min": min(sizes) if sizes else 0,
        "cells_max": max(sizes) if sizes else 0,
        "cells_mean": sum(sizes) / len(sizes) if sizes else 0,
        "pt_lost_median": sorted(lost)[len(lost) // 2] if lost else 0.0,
        "pt_lost_max": max(lost) if lost else 0.0,
        "per_event": per_event,
    }
    stats_path.write_text(json.dumps(stats, indent=1))

    print(f"read     {len(per_event)} events from {args.input}")
    print(f"wrote    {len(events)} events, {min(sizes)}..{max(sizes)} cells "
          f"(mean {stats['cells_mean']:.1f})")
    print(f"truncated {truncated} events at NMAX={args.nmax} "
          f"({100.0*truncated/max(len(per_event),1):.1f}%)")
    if lost:
        print(f"  pt lost in those: median {100*stats['pt_lost_median']:.2f}%, "
              f"max {100*stats['pt_lost_max']:.2f}%")
    print(f"  {cells_path}   for FastJet")
    print(f"  {pkt_path}   for cocotb / the wire")
    print(f"  {stats_path}")


if __name__ == "__main__":
    main()
