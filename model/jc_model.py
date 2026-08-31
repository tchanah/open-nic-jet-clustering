#!/usr/bin/env python3
"""Bit-accurate fixed-point model of the clustering engine.

This is verification layer 1: the reference the RTL is checked against, and
the cheapest place to find out whether our numerical choices change the jets.

It runs the same event two ways and compares:

    fixed   every quantity exactly as the hardware will hold it -- LUT ingest,
            truncated deltas, log-domain ranking, exact integer merges
    float   plain float64 anti-kt over the same cells, no tricks

The difference between them is attributable, and that is the point. Four
things could bite, and only a run like this separates them:

  * ingest rounding                      measured at 8e-10, negligible
  * delta truncation in jc_dist          ~1e-6 on dR^2, can flip near-ties
  * log-domain ranking                   Q7.25, LSB ~2e-8, finer than above
  * tie-breaking on a regular lattice    exact ties are ROUTINE by reflection
                                         symmetry, so the rule must be fixed

FastJet then validates the float path, not the fixed one -- the fixed path is
validated against float, and float against FastJet. Chaining it that way keeps
each comparison to one variable.

Input is the .pkt.bin written by pixelize.py, i.e. exactly the bytes the
hardware will receive.


Nearest-neighbour bookkeeping
-----------------------------
Each active row caches its nearest neighbour. After a merge, three sets of
rows need attention, and missing the third silently reorders merges:

  1. the survivor itself                  -- it moved
  2. rows whose NN was the survivor or    -- their cached answer is gone
     the absorbed jet
  3. rows the moved survivor is now       -- handled as a write-back DURING
     nearer to than their cached NN          the survivor's own row scan

For 3 the natural test is w_k + log2(min(g,R2)) < nn_dist_log[k], which would
need a logarithm per lane. Keeping nn_geo[k] alongside nn_dist_log[k] reduces
it by monotonicity to min(g,R2) < nn_geo[k] -- an integer compare. Only rows
whose nn_geo actually changed need a fresh log.

Those three obligations are the least-verified claim in the whole design, so
--audit checks them rather than arguing them: after the seeding scan and after
every round it recomputes each active row's nearest neighbour from scratch and
demands the maintained table match. It is a check on the RULES, independent of
the RTL -- which matters because RTL-versus-model agreement cannot catch a
misunderstanding that both sides share.
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

# log2 of a zero distance, MIRRORING THE HARDWARE rather than the maths.
#
# jc_log2 returns zero for a zero argument and jc_ctrl adds it to the weight
# unchanged, so the RTL ranks a zero-distance pair as though the distance were
# 1 -- the smallest representable nonzero geometry. A -inf sentinel here would
# be more principled and would disagree with the hardware, and agreement is
# the entire point of this model. The two choices are physically
# indistinguishable anyway: the alternative to "as close as anything can be"
# is "as close as anything can be, bar one unit of 1.9e-7 rad", and either
# still beats the beam term by log2(R^2) ~ 42.
#
# Unreachable from real data. It needs two rows within 1.9e-7 rad in BOTH
# coordinates after DELTA_SHIFT, and distinct towers are at least 0.1 in y or
# 2pi/64 in phi apart -- six orders of magnitude away. Only two merged jets
# could ever collide this closely, by coincidence.
LOG_ZERO_DIST = 0


def to_signed(word, width):
    return word - (1 << width) if word >> (width - 1) else word


def trunc(value, width):
    value &= (1 << width) - 1
    return value - (1 << width) if value >> (width - 1) else value


class Formats:
    """Everything the hardware's widths imply, read from luts.json."""

    def __init__(self, path):
        j = json.loads(path.read_text())
        self.j = j
        f = j["formats"]
        self.p4_w = f["jc_lut_energy"]["width"]
        self.p4_frac = f["jc_lut_energy"]["frac"]
        self.wgt_w = f["jc_lut_neg2log2e"]["width"]
        self.wgt_frac = f["jc_lut_neg2log2e"]["frac"]
        self.trig_frac = f["jc_lut_sech"]["frac"]
        self.coord_w = f["jc_lut_rapidity"]["width"]
        self.coord_scale = j["coord_scale"]
        self.delta_w = j["delta_w"]
        self.delta_shift = j["delta_shift"]
        self.phi_shift = j["phi_bin_shift"]
        self.sat_mag = (1 << self.delta_w) - 1
        self.half_lsb = 1 << (self.trig_frac - 1)
        # Respect each table's signedness. jc_lut_log2m is unsigned and its
        # top entry has the high bit set, so a blanket to_signed would turn
        # log2(2) = 1.0 into a negative number.
        self.lut = {k: ([to_signed(w, f[k]["width"]) for w in v]
                        if f[k].get("signed", True) else list(v))
                    for k, v in j["words"].items()}

    def p4_to_float(self, v):
        return v / float(1 << self.p4_frac)

    def q_wgt(self, x):
        return int(round(x * (1 << self.wgt_frac)))

    def r_squared(self, r_rad):
        """R^2 in the same delta units jc_dist produces."""
        return (int(round(r_rad * self.coord_scale)) >> self.delta_shift) ** 2


def read_packets(path):
    """Yield (seq, cells_total, [(iy, iphi, ecode), ...]) from pixelize.py."""
    b = path.read_bytes()
    off = 0
    while off < len(b):
        hdr = b[off:off + HDR_BYTES]
        if len(hdr) < HDR_BYTES:
            raise ValueError(f"truncated header at byte {off}")
        ethtype = int.from_bytes(hdr[12:14], "big")
        version = hdr[14]
        if ethtype != ETH_TYPE or version != FMT_VERSION:
            raise ValueError(f"bad header at {off}: type {ethtype:#x} "
                             f"version {version}")
        count = int.from_bytes(hdr[16:18], "big")
        seq = int.from_bytes(hdr[18:22], "big")
        total = int.from_bytes(hdr[22:24], "big")
        off += HDR_BYTES
        cells = []
        for _ in range(count):
            word = int.from_bytes(b[off:off + 4], "little")
            off += 4
            cells.append(((word >> 24) & 0xFF, (word >> 16) & 0xFF,
                          word & 0xFFFF))
        yield seq, total, cells


class Jet:
    """One active entry. Four-momentum is exact; coordinates are quantised."""

    __slots__ = ("e", "px", "py", "pz", "y", "phi", "wgt",
                 "nn", "nn_geo", "nn_log", "active")

    def __init__(self, e, px, py, pz, y, phi, wgt):
        self.e, self.px, self.py, self.pz = e, px, py, pz
        self.y, self.phi, self.wgt = y, phi, wgt
        self.nn, self.nn_geo, self.nn_log = -1, None, None
        self.active = True


def ingest(cells, fmt):
    """Cell -> pseudojet, mirroring jc_ingest's arithmetic exactly."""
    out = []
    for iy, iphi, ecode in cells:
        i6, p6, ec = iy & 0x3F, iphi & 0x3F, ecode & 0x0FFF
        e = fmt.lut["jc_lut_energy"][ec]
        s, t = fmt.lut["jc_lut_sech"][i6], fmt.lut["jc_lut_tanh"][i6]
        c, n = fmt.lut["jc_lut_cos"][p6], fmt.lut["jc_lut_sin"][p6]
        sh, w = fmt.trig_frac, fmt.p4_w
        pt = trunc((e * s + fmt.half_lsb) >> sh, w)
        pz = trunc((e * t + fmt.half_lsb) >> sh, w)
        px = trunc((pt * c + fmt.half_lsb) >> sh, w)
        py = trunc((pt * n + fmt.half_lsb) >> sh, w)
        wgt = trunc(fmt.lut["jc_lut_neg2log2e"][ec]
                    + fmt.lut["jc_lut_neg2logsech"][i6], fmt.wgt_w)
        out.append(Jet(e, px, py, pz,
                       fmt.lut["jc_lut_rapidity"][i6],
                       p6 << fmt.phi_shift, wgt))
    return out


def geo_dist_sq(a, b, fmt):
    """dR^2 exactly as jc_dist computes it: magnitude, shift, saturate."""
    dy = a.y - b.y
    dphi = to_signed((a.phi - b.phi) & 0xFFFFFFFF, fmt.coord_w)
    u = min(fmt.sat_mag, abs(dy) >> fmt.delta_shift)
    v = min(fmt.sat_mag, abs(dphi) >> fmt.delta_shift)
    return u * u + v * v


def q_log2(geo, fmt):
    if geo <= 0:
        return LOG_ZERO_DIST
    return int(round(math.log2(geo) * (1 << fmt.wgt_frac)))


def set_kin(jet, fmt):
    """Recompute coordinates after a merge.

    FastJet's numerically stable rapidity, not the naive ratio: pt_sq +
    mass_sq is mT^2, so E - |pz| is never formed. Bit-agreement with FastJet
    depends on using this exact form.

    Done in float64 here and quantised on the way out. jc_setkin (step 6) will
    replace this with a CORDIC and a log chain; isolating it this way keeps
    the ranking questions separate from the setkin-precision question.
    """
    e = fmt.p4_to_float(jet.e)
    px, py = fmt.p4_to_float(jet.px), fmt.p4_to_float(jet.py)
    pz = fmt.p4_to_float(jet.pz)

    pt_sq = px * px + py * py
    if pt_sq <= 0.0:
        jet.y, jet.phi, jet.wgt = 0, 0, fmt.q_wgt(64.0)
        return

    phi = math.atan2(py, px) % (2.0 * math.pi)
    jet.phi = int(round(phi * fmt.coord_scale)) & 0xFFFFFFFF

    mass_sq = (e + pz) * (e - pz) - pt_sq
    e_plus = e + abs(pz)
    half_log = 0.5 * math.log(max(pt_sq + mass_sq, 1e-300) / (e_plus * e_plus))
    y = -half_log if pz > 0 else half_log

    lim = (1 << (fmt.coord_w - 1)) - 1
    jet.y = max(-lim, min(lim, int(round(y * fmt.coord_scale))))
    jet.wgt = trunc(fmt.q_wgt(-math.log2(pt_sq)), fmt.wgt_w)


def row_scan(jets, i, r2, fmt, write_back=True):
    """Find row i's nearest neighbour, and let i claim rows it is nearer to.

    The write-back is what keeps the table honest after the survivor moves;
    without it a row keeps pointing at a neighbour that is no longer nearest
    and merges come out in the wrong order. See the module docstring.
    """
    ji = jets[i]
    best_geo, best_k = None, -1
    for k, jk in enumerate(jets):
        if k == i or not jk.active:
            continue
        g = geo_dist_sq(ji, jk, fmt)
        if best_geo is None or g < best_geo or (g == best_geo and k < best_k):
            best_geo, best_k = g, k
        # Only a neighbour strictly inside R can be claimed. Clamping to r2
        # and comparing would let a row BEYOND R capture k, and k would then
        # merge on its turn instead of being emitted as a jet.
        if write_back and g < r2 and jk.nn_geo is not None:
            # Monotonic in log, so the linear compare is the same decision --
            # and needs no logarithm in the lane.
            if g < jk.nn_geo or (g == jk.nn_geo and i < jk.nn):
                jk.nn_geo, jk.nn = g, i
                jk.nn_log = trunc(jk.wgt + q_log2(g, fmt), 64)

    if best_geo is None or best_geo >= r2:
        ji.nn, ji.nn_geo = -1, r2          # the beam wins this row
    else:
        ji.nn, ji.nn_geo = best_k, best_geo
    ji.nn_log = trunc(ji.wgt + q_log2(ji.nn_geo, fmt), 64)


def nn_reference(jets, i, r2, fmt):
    """Row i's nearest neighbour from scratch -- no cache, no write-back.

    The same decision row_scan makes, reached independently: brute force over
    every active row, smallest index on a tie, the beam when nothing is inside
    R. Nothing in the clustering path calls this; it exists so the maintained
    table has something to be wrong against.
    """
    ji = jets[i]
    best_geo, best_k = None, -1
    for k, jk in enumerate(jets):
        if k == i or not jk.active:
            continue
        g = geo_dist_sq(ji, jk, fmt)
        if best_geo is None or g < best_geo or (g == best_geo and k < best_k):
            best_geo, best_k = g, k

    if best_geo is None or best_geo >= r2:
        return -1, r2                      # the beam wins this row
    return best_k, best_geo


def audit_nn_table(jets, r2, fmt, where):
    """Assert the incrementally maintained NN table equals a full recompute.

    THE MAINTENANCE RULES ARE THE LEAST-CHECKED PART OF THE DESIGN. Rescanning
    the survivor, rescanning every row that pointed at either merged row, and
    letting the survivor's own scan claim rows it is now nearest to -- three
    obligations, and missing one silently reorders later merges. Two review
    passes never touched them, and RTL-versus-model agreement cannot check them
    because the same reasoning wrote both sides.

    This checks the rules against arithmetic instead of against a reading. It
    recomputes every active row's nearest neighbour from scratch and demands
    the cached answer match in all three fields the engine actually uses: the
    index that says who merges, the linear nn_geo the sweep write-back compares
    on, and the log-domain nn_dist_log the global argmin ranks on.

    A missed obligation surfaces here as a stale row at the round it went
    stale, rather than as a jet that differs three merges downstream.
    """
    for i, ji in enumerate(jets):
        if not ji.active:
            continue
        ref_nn, ref_geo = nn_reference(jets, i, r2, fmt)
        ref_log = trunc(ji.wgt + q_log2(ref_geo, fmt), 64)
        if (ji.nn, ji.nn_geo, ji.nn_log) != (ref_nn, ref_geo, ref_log):
            raise AssertionError(
                f"NN table stale at {where}: row {i} caches "
                f"nn={ji.nn} geo={ji.nn_geo} log={ji.nn_log}, "
                f"recompute says nn={ref_nn} geo={ref_geo} log={ref_log}")


def cluster_fixed(cells, fmt, r_rad, pt_floor, audit=False):
    """The engine, in exactly the arithmetic the hardware will use.

    With audit set, the nearest-neighbour table is checked against a full
    recompute after the seeding scan and after every round. That is O(n^3) per
    event against the O(n^2) the clustering itself costs, so it is off by
    default and meant for a few tens of events, not a dataset.
    """
    jets = ingest(cells, fmt)
    r2 = fmt.r_squared(r_rad)
    floor_sq = pt_floor * pt_floor

    for i in range(len(jets)):
        row_scan(jets, i, r2, fmt, write_back=False)

    if audit:
        audit_nn_table(jets, r2, fmt, "setup")

    out = []
    rounds = 0
    remaining = len(jets)
    while remaining:
        # Global argmin, smallest index on a tie. The lattice makes exact ties
        # routine, so this rule is load-bearing for reproducibility.
        best_i, best_d = -1, None
        for i, j in enumerate(jets):
            if not j.active:
                continue
            if best_d is None or j.nn_log < best_d:
                best_i, best_d = i, j.nn_log

        ji = jets[best_i]
        j_idx = ji.nn

        if j_idx < 0 or not jets[j_idx].active:
            px, py = fmt.p4_to_float(ji.px), fmt.p4_to_float(ji.py)
            if px * px + py * py >= floor_sq:
                out.append((fmt.p4_to_float(ji.e), px, py,
                            fmt.p4_to_float(ji.pz)))
            ji.active = False
            remaining -= 1
            action = f"emit {best_i}"
            # Anyone pointing at the departed jet has lost its answer.
            for k, jk in enumerate(jets):
                if jk.active and jk.nn == best_i:
                    row_scan(jets, k, r2, fmt, write_back=False)
        else:
            jj = jets[j_idx]
            # E-scheme: exact integer adds, so a jet is independent of the
            # order its constituents were combined in.
            ji.e += jj.e
            ji.px += jj.px
            ji.py += jj.py
            ji.pz += jj.pz
            jj.active = False
            remaining -= 1
            action = f"merge {best_i} <- {j_idx}"
            set_kin(ji, fmt)

            # Rows whose cached neighbour was either of the merged pair must
            # be rescanned in full -- being near the survivor does not make
            # the survivor their NEAREST neighbour, so a write-back cannot
            # stand in for the scan.
            stale = [k for k, jk in enumerate(jets)
                     if jk.active and k != best_i
                     and jk.nn in (best_i, j_idx)]
            for k in stale:
                row_scan(jets, k, r2, fmt, write_back=False)
            # Then let the moved survivor claim any row it is now nearest to.
            row_scan(jets, best_i, r2, fmt, write_back=True)

        rounds += 1
        if audit:
            audit_nn_table(jets, r2, fmt, f"round {rounds} after {action}")
    return out


def cluster_float(cells, fmt, r_rad, pt_floor):
    """Plain float64 anti-kt over the same cells -- the control."""
    objs = []
    for iy, iphi, ecode in cells:
        e = fmt.p4_to_float(fmt.lut["jc_lut_energy"][ecode & 0x0FFF])
        y = fmt.lut["jc_lut_rapidity"][iy & 0x3F] / fmt.coord_scale
        phi = 2.0 * math.pi * (iphi & 0x3F) / fmt.j["phi_bins"]
        pt = e / math.cosh(y)
        objs.append([e, pt * math.cos(phi), pt * math.sin(phi),
                     e * math.tanh(y)])

    r2 = r_rad * r_rad
    out = []
    while objs:
        n = len(objs)
        kin = []
        for e, px, py, pz in objs:
            pt_sq = px * px + py * py
            p = math.sqrt(pt_sq + pz * pz)
            y = 0.5 * math.log((e + pz) / (e - pz)) if abs(pz) < p else 0.0
            kin.append((pt_sq, y, math.atan2(py, px)))

        best, bi, bj = None, -1, -1
        for i in range(n):
            pti, yi, phii = kin[i]
            wi = 1.0 / pti if pti > 0 else float("inf")
            if best is None or wi < best:
                best, bi, bj = wi, i, -1
            for k in range(i + 1, n):
                ptk, yk, phik = kin[k]
                dphi = abs(phii - phik) % (2 * math.pi)
                dphi = min(dphi, 2 * math.pi - dphi)
                dr2 = (yi - yk) ** 2 + dphi * dphi
                w = min(wi, 1.0 / ptk if ptk > 0 else float("inf"))
                d = w * dr2 / r2
                if d < best:
                    best, bi, bj = d, i, k

        if bj < 0:
            e, px, py, pz = objs[bi]
            if px * px + py * py >= pt_floor * pt_floor:
                out.append((e, px, py, pz))
            objs.pop(bi)
        else:
            for c in range(4):
                objs[bi][c] += objs[bj][c]
            objs.pop(bj)
    return out


def cluster_fastjet(cells, fmt, r_rad, pt_floor):
    """The same cells through FastJet itself.

    Validates the float control, which in turn validates the fixed path --
    one variable per comparison. Cells are built identically to
    cluster_float, so any difference here is FastJet's clustering versus
    ours, not the inputs.
    """
    import fastjet                      # optional: only needed with --fastjet

    pjs = []
    for iy, iphi, ecode in cells:
        e = fmt.p4_to_float(fmt.lut["jc_lut_energy"][ecode & 0x0FFF])
        y = fmt.lut["jc_lut_rapidity"][iy & 0x3F] / fmt.coord_scale
        phi = 2.0 * math.pi * (iphi & 0x3F) / fmt.j["phi_bins"]
        pt = e / math.cosh(y)
        pjs.append(fastjet.PseudoJet(pt * math.cos(phi), pt * math.sin(phi),
                                     e * math.tanh(y), e))

    jetdef = fastjet.JetDefinition(fastjet.antikt_algorithm, r_rad)
    seq = fastjet.ClusterSequence(pjs, jetdef)
    return [(j.e(), j.px(), j.py(), j.pz())
            for j in seq.inclusive_jets(pt_floor)]


def match(a, b):
    """Pair jets by proximity in (y, phi); return worst relative pt error."""
    def kin(j):
        e, px, py, pz = j
        pt = math.hypot(px, py)
        p = math.sqrt(pt * pt + pz * pz)
        y = 0.5 * math.log((e + pz) / (e - pz)) if abs(pz) < p else 0.0
        return pt, y, math.atan2(py, px)

    ka, kb = [kin(j) for j in a], [kin(j) for j in b]
    worst, unmatched = 0.0, 0
    used = set()
    for pta, ya, pa in ka:
        best, bk = None, -1
        for k, (ptb, yb, pb) in enumerate(kb):
            if k in used:
                continue
            dphi = abs(pa - pb) % (2 * math.pi)
            dphi = min(dphi, 2 * math.pi - dphi)
            d = (ya - yb) ** 2 + dphi * dphi
            if best is None or d < best:
                best, bk = d, k
        if bk < 0 or best > 0.01:
            unmatched += 1
            continue
        used.add(bk)
        worst = max(worst, abs(pta - kb[bk][0]) / max(pta, 1e-12))
    unmatched += len(kb) - len(used)
    return worst, unmatched


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("packets", type=pathlib.Path,
                    help=".pkt.bin from pixelize.py")
    ap.add_argument("--luts", type=pathlib.Path, default=HERE / "luts.json")
    ap.add_argument("--events", type=int, default=50)
    ap.add_argument("-R", type=float, default=0.4)
    ap.add_argument("--pt-min", type=float, default=5.0,
                    help="jet floor in GeV; the trigger case is 50")
    ap.add_argument("--fastjet", action="store_true",
                    help="also cluster with FastJet itself and compare both "
                         "paths against it")
    ap.add_argument("--audit", action="store_true",
                    help="after every round, check the incrementally "
                         "maintained nearest-neighbour table against a full "
                         "recompute. O(n^3) per event -- use a few tens of "
                         "events, not a dataset")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    fmt = Formats(args.luts)
    # Each comparison isolates one variable: fixed-vs-float is our numerics,
    # float-vs-FastJet is our clustering, fixed-vs-FastJet is the end-to-end
    # claim the physics side cares about.
    pairs = {"fixed vs float": [0.0, 0]}
    if args.fastjet:
        pairs["float vs fastjet"] = [0.0, 0]
        pairs["fixed vs fastjet"] = [0.0, 0]

    bad_events, total_jets = 0, 0

    for i, (seq, total, cells) in enumerate(read_packets(args.packets)):
        if i >= args.events:
            break
        try:
            fx = cluster_fixed(cells, fmt, args.R, args.pt_min,
                               audit=args.audit)
        except AssertionError as exc:
            raise SystemExit(f"AUDIT FAILED on event seq {seq} "
                             f"(n={len(cells)}):\n  {exc}")
        fl = cluster_float(cells, fmt, args.R, args.pt_min)
        total_jets += len(fl)

        results = {"fixed vs float": match(fx, fl)}
        if args.fastjet:
            fj = cluster_fastjet(cells, fmt, args.R, args.pt_min)
            results["float vs fastjet"] = match(fl, fj)
            results["fixed vs fastjet"] = match(fx, fj)

        failed = False
        for key, (worst, unmatched) in results.items():
            pairs[key][0] = max(pairs[key][0], worst)
            pairs[key][1] += unmatched
            if unmatched or worst > 1e-4:
                failed = True
        bad_events += failed
        if args.verbose:
            detail = "  ".join(f"{k}: {w:.2e}/{u}"
                               for k, (w, u) in results.items())
            print(f"  seq {seq:6d}  n={len(cells):3d}  jets {len(fl)}  {detail}")

    n = min(args.events, i + 1)
    print(f"events compared     : {n}")
    print(f"jets (float)        : {total_jets}")
    for key, (worst, unmatched) in pairs.items():
        print(f"{key:20s}: worst rel pt {worst:.3e}   unmatched {unmatched}")
    print(f"events failing      : {bad_events}   (criterion 1e-4)")
    if args.audit:
        print(f"nn table audited    : {n} events, every round, table matched "
              f"a full recompute")
    print("PASS" if bad_events == 0 else "FAIL")


if __name__ == "__main__":
    main()
