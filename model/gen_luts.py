#!/usr/bin/env python3
"""Generate the jc_ingest lookup tables.

Emits two artefacts from one computation, so the RTL and the Python model
cannot drift:

    ../jc_luts.vh   initial blocks, `include-d inside jc_ingest
    luts.json       the same integers, for jc_model.py and the cocotb bench

Widths and depths are parsed out of ../jc_defs.vh rather than restated here,
so a format change is a one-place edit.

Ingest is pure table lookup by design. Cells arrive as grid indices, so:

    energy   = E[ecode]
    pt       = energy * sech(y)                 } the only multiplies,
    pz       = energy * tanh(y)                 } four of them, in a unit
    px       = pt     * cos(phi)                } shared by every engine
    py       = pt     * sin(phi)
    rapidity = y[iy]                            exact, no arithmetic
    phi      = iphi << JC_PHI_BIN_SHIFT         exact, a shift
    weight   = -2*log2(E) + -2*log2(sech(y))    one add, both from tables

log2(1/pt^2) = -2*log2(pt) = -2*log2(E) - 2*log2(sech(y)), which is why the
beam weight needs no logarithm in hardware -- the split is exact because pt
factorises into an energy term and a rapidity term.

Energy mapping
--------------
Log-spaced over [--emin, --emax] across 4096 codes. This is a PLACEHOLDER:
what the codes mean is an open physics item (CLAUDE.md), and answering it is
a rerun of this script, not a design change.

Note what the encoding itself costs. 4096 log-spaced codes over 0.2-2000 GeV
is ~0.225% per step, so cell energy carries ~0.11% quantisation -- an order
of magnitude coarser than the 1e-4 match criterion. That is not a problem
for verification, because FastJet is fed the same decoded energies: 1e-4 is
RTL vs model vs FastJet on identical decoded inputs, not RTL vs truth. It is
worth the physics side knowing, because it does bound truth.
"""

import argparse
import json
import math
import pathlib
import re

HERE = pathlib.Path(__file__).resolve().parent
PLUGIN_ROOT = HERE.parent
# Generated RTL lives apart from hand-written RTL. It is ~13k lines against
# the design's ~2.6k, so mixing them makes the repo root unreadable.
GEN_DIR = PLUGIN_ROOT / "gen"

# Arrays gen_luts.py owns, as declared in jc_defs.vh. Renaming one without
# renaming it there fails at elaboration, which is the intent.
ARRAY_DECLS = [
    # (verilog array,       width macro,  frac macro,       depth macro)
    ("jc_lut_energy",       "JC_P4_W",    "JC_P4_FRAC",     "JC_LUT_DEPTH_E"),
    ("jc_lut_neg2log2e",    "JC_WGT_W",   "JC_WGT_FRAC",    "JC_LUT_DEPTH_E"),
    ("jc_lut_sech",         "JC_TRIG_W",  "JC_TRIG_FRAC",   "JC_LUT_DEPTH_BIN"),
    ("jc_lut_tanh",         "JC_TRIG_W",  "JC_TRIG_FRAC",   "JC_LUT_DEPTH_BIN"),
    ("jc_lut_rapidity",     "JC_RAP_W",   None,             "JC_LUT_DEPTH_BIN"),
    ("jc_lut_neg2logsech",  "JC_WGT_W",   "JC_WGT_FRAC",    "JC_LUT_DEPTH_BIN"),
    ("jc_lut_cos",          "JC_TRIG_W",  "JC_TRIG_FRAC",   "JC_LUT_DEPTH_BIN"),
    ("jc_lut_sin",          "JC_TRIG_W",  "JC_TRIG_FRAC",   "JC_LUT_DEPTH_BIN"),
    # log2 mantissa table for jc_log2. One extra entry so interpolation can
    # read i and i+1 from a dual-port BRAM instead of storing a slope table.
    ("jc_lut_log2m",        "JC_LOG2_TAB_W", "JC_LOG2_TAB_FRAC", "JC_LOG2_DEPTH"),
    # CORDIC rotation angles, in the same binary-angle units as phi itself,
    # so the accumulated result needs no final scaling.
    ("jc_lut_atan",         "JC_COORD_W", None,             "JC_LUT_ATAN_DEPTH"),
]

DEFINE_RE = re.compile(r"^\s*`define\s+(JC_\w+)\s+(.+?)\s*(?://.*)?$")
SIZED_RE = re.compile(r"^\d+'[hbd]", re.IGNORECASE)


def parse_defs(path):
    """Pull the plain-integer `define values out of jc_defs.vh."""
    out = {}
    for line in path.read_text().splitlines():
        m = DEFINE_RE.match(line)
        if not m:
            continue
        name, value = m.group(1), m.group(2).strip()
        if SIZED_RE.match(value):      # 16'h88B6 and friends are not ours
            continue
        if value.startswith("`"):
            # An alias for another macro. Resolving these is what lets
            # jc_defs.vh say a depth IS an iteration count rather than
            # restating the number and letting the two drift apart.
            ref = value[1:].strip()
            if ref in out:
                out[name] = out[ref]
            continue
        try:
            out[name] = int(value, 0)
        except ValueError:
            pass                       # strings, expressions: not needed here
    return out


# Shared coordinate scale: 2^32 per full turn, used by BOTH rapidity and
# azimuth so that dy^2 + dphi^2 is meaningful. See jc_defs.vh -- azimuth sets
# the scale because it is the one that makes wrapping free.
COORD_SCALE = 2.0 ** 32 / (2.0 * math.pi)          # units per radian


# Tables whose values cannot be negative. Declaring them unsigned is not a
# nicety: jc_lut_log2m ends at log2(2) = 1.0, which in Q1.31 is exactly 2^31
# and does not fit a signed 32-bit word -- but fits an unsigned one exactly.
UNSIGNED_ARRAYS = {"jc_lut_log2m", "jc_lut_atan"}


def to_fixed(value, multiplier, width, name, index, is_signed=True):
    """Round to a fixed-point word and return its two's complement.

    Range is checked rather than wrapped: a format too narrow for its table
    is a design error, and silently wrapping it would look like data.
    """
    scaled = int(round(value * multiplier))
    if is_signed:
        lo, hi = -(1 << (width - 1)), (1 << (width - 1)) - 1
    else:
        lo, hi = 0, (1 << width) - 1
    if not lo <= scaled <= hi:
        raise ValueError(
            f"{name}[{index}] = {value!r} scales to {scaled}, which overflows "
            f"{width} bits {'signed' if is_signed else 'unsigned'}")
    return scaled & ((1 << width) - 1)


def build(defs, emin, emax):
    """Return {array name: [python float]} in physical units."""
    depth_e = defs["JC_LUT_DEPTH_E"]
    depth_bin = defs["JC_LUT_DEPTH_BIN"]
    rap_bins = defs["JC_RAP_BINS"]
    phi_bins = defs["JC_PHI_BINS"]

    # Energy: log-spaced, code 0 -> emin, code depth-1 -> emax.
    energy = [emin * (emax / emin) ** (c / (depth_e - 1)) for c in range(depth_e)]

    # Rapidity bin centres. Tables are depth_bin deep for a clean address
    # width; indices at or past rap_bins repeat the last legal bin so an
    # out-of-range cell still yields finite numbers next to its error flag.
    def y_of(iy):
        i = min(iy, rap_bins - 1)
        return -2.5 + 0.1 * (i + 0.5)

    # Phi bin centres sit on multiples of the bin, so phi = iphi << 26 is
    # exact. Every index below phi_bins is legal; the guard is for the wider
    # field, and phi_bins == depth_bin today so it never triggers.
    def phi_of(iphi):
        return 2.0 * math.pi * min(iphi, phi_bins - 1) / phi_bins

    ys = [y_of(i) for i in range(depth_bin)]
    phis = [phi_of(i) for i in range(depth_bin)]

    # log2(1 + i/2^k) over one octave, plus the endpoint log2(2) = 1.0.
    log2_depth = defs["JC_LOG2_DEPTH"]
    idx_span = 1 << defs["JC_LOG2_IDX_BITS"]
    log2m = [math.log2(1.0 + i / idx_span) for i in range(log2_depth)]

    # CORDIC rotation angles. Emitted in radians and scaled by COORD_SCALE
    # like the rapidity table, which is exactly what makes them binary angles.
    atan_tab = [math.atan(2.0 ** -i) for i in range(defs["JC_LUT_ATAN_DEPTH"])]

    return {
        "jc_lut_log2m": log2m,
        "jc_lut_atan": atan_tab,
        "jc_lut_energy": energy,
        "jc_lut_neg2log2e": [-2.0 * math.log2(e) for e in energy],
        "jc_lut_sech": [1.0 / math.cosh(y) for y in ys],
        "jc_lut_tanh": [math.tanh(y) for y in ys],
        "jc_lut_rapidity": ys,
        "jc_lut_neg2logsech": [-2.0 * math.log2(1.0 / math.cosh(y)) for y in ys],
        "jc_lut_cos": [math.cos(p) for p in phis],
        "jc_lut_sin": [math.sin(p) for p in phis],
    }


def quantise(defs, real):
    """Convert the physical tables to two's-complement words."""
    words, formats = {}, {}
    for name, w_macro, f_macro, d_macro in ARRAY_DECLS:
        width, depth = defs[w_macro], defs[d_macro]
        # f_macro None means the array is not a power-of-two Q format: the
        # coordinate tables carry the shared 2^32-per-turn scale instead.
        frac = defs[f_macro] if f_macro else None
        multiplier = (1 << frac) if frac is not None else COORD_SCALE
        is_signed = name not in UNSIGNED_ARRAYS
        values = real[name]
        assert len(values) == depth, f"{name}: {len(values)} values, depth {depth}"
        words[name] = [to_fixed(v, multiplier, width, name, i, is_signed)
                       for i, v in enumerate(values)]
        formats[name] = {"width": width, "frac": frac, "depth": depth,
                         "scale": multiplier, "signed": is_signed}
    return words, formats


def emit_consts(path, consts, provenance):
    """Emit scalar `define constants.

    These belong here for the same reason the tables do. JC_K_RAP was hand
    computed once and came out 5.8e-5 high, which showed up as every rapidity
    being wrong by that exact relative amount -- a constant no test of the
    arithmetic could catch, because the arithmetic was right.
    """
    lines = [
        "// *********************************************************************",
        "//",
        "// jc_consts.vh -- GENERATED by model/gen_luts.py. Do not edit.",
        "//",
        f"// {provenance}",
        "//",
        "// *********************************************************************",
        "`ifndef JC_CONSTS_VH",
        "`define JC_CONSTS_VH",
        "",
    ]
    for name, (value, width, comment) in consts.items():
        lines.append(f"// {comment}")
        lines.append(f"`define {name} {width}'d{value}")
        lines.append("")
    lines.append("`endif")
    path.write_text("\n".join(lines))


def emit_verilog(path, words, formats, provenance, names):
    """Emit the initial blocks for `names` only.

    Split by consumer, not by convenience: each file is `include-d inside a
    module that declares exactly those arrays, so an array landing in the
    wrong file fails at elaboration instead of silently initialising nothing.
    """
    lines = [
        "// *********************************************************************",
        "//",
        "// jc_luts.vh -- GENERATED by model/gen_luts.py. Do not edit.",
        "//",
        f"// {provenance}",
        "//",
        "// Included inside jc_ingest, which declares the arrays. Plain indexed",
        "// assignment rather than $readmemh: no runtime file path to resolve, so",
        "// Icarus and Vivado initialise identically and a cocotb run needs no",
        "// working-directory setup.",
        "//",
        "// *********************************************************************",
        "",
    ]
    for name in names:
        fmt = formats[name]
        width, depth = fmt["width"], fmt["depth"]
        digits = (width + 3) // 4
        idx_w = len(str(depth - 1))
        fmt_desc = (f"Q{width - fmt['frac']}.{fmt['frac']}" if fmt["frac"] is not None
                    else f"{width}b, {fmt['scale']:.6f} units per radian")
        lines.append(f"// {name}: {fmt_desc}, {depth} entries")
        lines.append("initial begin")
        for i, word in enumerate(words[name]):
            lines.append(f"  {name}[{i:{idx_w}d}] = "
                         f"{width}'h{word:0{digits}x};")
        lines.append("end")
        lines.append("")
    path.write_text("\n".join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--emin", type=float, default=0.2,
                    help="GeV at energy code 0 (default: the pt transmission floor)")
    ap.add_argument("--emax", type=float, default=2000.0,
                    help="GeV at the top code (default: %(default)s)")
    ap.add_argument("--defs", type=pathlib.Path,
                    default=PLUGIN_ROOT / "jc_defs.vh")
    ap.add_argument("--vh", type=pathlib.Path,
                    default=GEN_DIR / "jc_luts.vh")
    ap.add_argument("--log2-vh", type=pathlib.Path,
                    default=GEN_DIR / "jc_luts_log2.vh")
    ap.add_argument("--cordic-vh", type=pathlib.Path,
                    default=GEN_DIR / "jc_luts_cordic.vh")
    ap.add_argument("--consts-vh", type=pathlib.Path,
                    default=GEN_DIR / "jc_consts.vh")
    ap.add_argument("--default-r", type=float, default=0.4,
                    help="reset value of the runtime R, in rapidity-azimuth "
                         "units (AXI-Lite overrides it)")
    ap.add_argument("--default-pt-floor", type=float, default=50.0,
                    help="reset value of the jet pt floor in GeV; the trigger "
                         "case is 50 (AXI-Lite overrides it)")
    ap.add_argument("--json", type=pathlib.Path, default=HERE / "luts.json")
    # phi is not tabulated -- iphi << JC_PHI_BIN_SHIFT is exact.
    args = ap.parse_args()

    for p in {args.vh.parent, args.log2_vh.parent,
              args.cordic_vh.parent, args.consts_vh.parent}:
        p.mkdir(parents=True, exist_ok=True)

    defs = parse_defs(args.defs)
    real = build(defs, args.emin, args.emax)
    words, formats = quantise(defs, real)

    step = (args.emax / args.emin) ** (1.0 / (defs["JC_LUT_DEPTH_E"] - 1))
    provenance = (f"energy: log-spaced {args.emin} - {args.emax} GeV over "
                  f"{defs['JC_LUT_DEPTH_E']} codes, {(step - 1) * 100:.4f}% per step")

    log2_arrays = ["jc_lut_log2m"]
    cordic_arrays = ["jc_lut_atan"]
    ingest_arrays = [n for n, _, _, _ in ARRAY_DECLS
                     if n not in log2_arrays + cordic_arrays]
    emit_verilog(args.vh, words, formats, provenance, ingest_arrays)
    emit_verilog(args.log2_vh, words, formats, provenance, log2_arrays)
    emit_verilog(args.cordic_vh, words, formats, provenance, cordic_arrays)

    # jc_setkin turns a difference of base-2 logarithms into binary-angle
    # rapidity: y_units = d_log * ln2 * COORD_SCALE / 2^(LOG_FRAC+1). Folding
    # the 2^K_RAP_F into the constant keeps the shift a power of two.
    k_rap_f = 3
    k_rap = int(round(math.log(2.0) * COORD_SCALE * (1 << k_rap_f)))
    assert k_rap < (1 << 32), "JC_K_RAP no longer fits 32 bits"
    # Reset values for the AXI-Lite config registers. Generated rather than
    # written into the RTL by hand for the same reason JC_K_RAP is: R^2 is
    # R scaled by COORD_SCALE, shifted and squared, and getting any of those
    # three steps wrong yields a plausible number that quietly changes which
    # cells cluster together. They are DEFAULTS, not limits -- the host
    # overwrites either at run time -- but a card that is never configured
    # must still cluster correctly, which is what makes them load-bearing.
    r2_default = (int(round(args.default_r * COORD_SCALE))
                  >> defs["JC_DELTA_SHIFT"]) ** 2
    assert r2_default < (1 << defs["JC_GEO_W"]), "default R^2 exceeds JC_GEO_W"
    pt_sq_default = int(round(args.default_pt_floor ** 2
                              * (1 << (2 * defs["JC_P4_FRAC"]))))
    assert pt_sq_default < (1 << (2 * defs["JC_P4_W"])), "default floor overflows"

    emit_consts(args.consts_vh, {
        "JC_K_RAP": (k_rap, 32,
                     f"ln2 * 2^32/(2*pi) * 2^{k_rap_f}; divide by "
                     f"2^(JC_LOG2_OUT_FRAC + {k_rap_f} + 1) after multiplying"),
        "JC_K_RAP_F": (k_rap_f, 8, "extra fractional bits folded into JC_K_RAP"),
        "JC_DEFAULT_R_SQUARED": (
            r2_default, defs["JC_GEO_W"],
            f"reset value of cfg_r_squared: R = {args.default_r} in delta units"),
        "JC_DEFAULT_PT_SQ_FLOOR": (
            pt_sq_default, 2 * defs["JC_P4_W"],
            f"reset value of cfg_pt_sq_floor: "
            f"({args.default_pt_floor} GeV)^2 in Q28.68"),
    }, provenance)

    args.json.write_text(json.dumps({
        "provenance": provenance,
        "energy_min_gev": args.emin,
        "energy_max_gev": args.emax,
        "formats": formats,
        "phi_bin_shift": defs["JC_PHI_BIN_SHIFT"],
        "coord_scale": COORD_SCALE,
        # Benches read these rather than repeating the numbers, so a rerun
        # with a different default cannot leave a test asserting the old one.
        "default_r": args.default_r,
        "default_pt_floor": args.default_pt_floor,
        "default_r_squared": r2_default,
        "default_pt_sq_floor": pt_sq_default,
        "delta_w": defs["JC_DELTA_W"],
        "delta_shift": defs["JC_DELTA_SHIFT"],
        "geo_w": defs["JC_GEO_W"],
        "log2_idx_bits": defs["JC_LOG2_IDX_BITS"],
        "log2_frac_bits": defs["JC_LOG2_FRAC_BITS"],
        "log2_in_w": defs["JC_LOG2_IN_W"],
        "log2_out_w": defs["JC_LOG2_OUT_W"],
        "log2_out_frac": defs["JC_LOG2_OUT_FRAC"],
        "k_rap": k_rap,
        "k_rap_f": k_rap_f,
        "rap_bins": defs["JC_RAP_BINS"],
        "phi_bins": defs["JC_PHI_BINS"],
        "words": words,
    }, indent=1))

    print(provenance)
    print(f"wrote {args.vh}")
    print(f"wrote {args.log2_vh}")
    print(f"wrote {args.cordic_vh}")
    print(f"wrote {args.consts_vh}   (JC_K_RAP = {k_rap})")
    print(f"  defaults: R = {args.default_r} -> R^2 = {r2_default}, "
          f"floor = {args.default_pt_floor} GeV -> pt^2 = {pt_sq_default}")
    print(f"wrote {args.json}")


if __name__ == "__main__":
    main()
