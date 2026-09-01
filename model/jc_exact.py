"""Bit-accurate Python of the two RTL units jc_model.py used to idealise.

WHY THIS EXISTS. `cluster_fixed` reproduces the hardware's arithmetic
everywhere except `jc_log2` and `jc_setkin`: `q_log2` was `math.log2` in
float64 and `set_kin` was float64 trigonometry quantised on the way out. So
"bit-exact against the model" was conditional, not structural -- it held only
while no merged jet's coordinate landed near a truncation boundary.

That conditional broke twice, measurably:

  * The jc_log2 index wrap was invisible to every model layer, because the
    model never indexed the table at all. It took a synthesis report to find.
  * seq 870 of cells1k differs between card and model for no other reason:
    rows 15 and 17 are BIT-IDENTICALLY equidistant from a merged row 18, the
    model breaks the tie by smallest index, and jc_setkin's coordinate --
    4.6e-9 rad away, 2.5% of a delta LSB -- lands on the other side of a
    truncation boundary. One delta LSB, geo differs by 2d-1 = 3,145,727, the
    tie becomes a strict inequality, and one merge becomes an emit.

Exact ties are routine here (2-6% of row scans, by reflection symmetry on the
cell lattice), which is what makes a sub-LSB difference decide jets.

Everything below mirrors the RTL statement for statement. Where the RTL slices
a signed vector, so does this -- via two's complement on an explicit width,
never Python's unbounded integers.
"""

import json
import pathlib

_HERE = pathlib.Path(__file__).resolve().parent


def _tc(value, width):
    """Two's-complement bit pattern of `value` in `width` bits."""
    return value & ((1 << width) - 1)


def _signed(bits, width):
    """Interpret `width` bits as signed."""
    bits &= (1 << width) - 1
    return bits - (1 << width) if bits >> (width - 1) else bits


def _slice(value, width, hi, lo):
    """The RTL's `value[hi:lo]` on a `width`-bit two's-complement word."""
    return (_tc(value, width) >> lo) & ((1 << (hi - lo + 1)) - 1)


class Exact:
    """jc_log2 and jc_setkin, bit-for-bit, driven by the generated tables."""

    def __init__(self, luts_path=None):
        L = json.loads((pathlib.Path(luts_path) if luts_path
                        else _HERE / "luts.json").read_text())
        self.L = L

        # ---- jc_log2 ----------------------------------------------------
        self.LOG_IN_W = L["log2_in_w"]
        self.IDX_BITS = L["log2_idx_bits"]
        self.FRC_BITS = L["log2_frac_bits"]
        self.OUT_FRAC = L["log2_out_frac"]
        self.LOG_W = L["log2_out_w"]
        self.MANT_W = self.IDX_BITS + self.FRC_BITS
        self.TAB = L["words"]["jc_lut_log2m"]
        self.TAB_FRAC = L["formats"]["jc_lut_log2m"]["frac"]

        # ---- jc_cordic --------------------------------------------------
        self.ATAN = L["words"]["jc_lut_atan"]
        self.CORDIC_ITERS = len(self.ATAN)
        self.CORDIC_W = 56          # JC_CORDIC_W
        self.CORDIC_MSB = 50        # JC_CORDIC_MSB
        self.COORD_W = 32

        # ---- jc_setkin --------------------------------------------------
        self.P4_W = 48
        self.P4_FRAC = 34
        self.PROD_W = 2 * self.P4_W + 2          # 98
        self.ACC_W = self.LOG_W + 3              # 42
        self.WGT_W = L["formats"]["jc_lut_neg2log2e"]["width"]
        self.WGT_FRAC = L["formats"]["jc_lut_neg2log2e"]["frac"]
        self.K_RAP = L["k_rap"]
        self.K_RAP_SHIFT = self.OUT_FRAC + L["k_rap_f"] + 1
        self.LOG_TO_WGT = self.OUT_FRAC - self.WGT_FRAC
        self.WGT_BIAS = (1 << self.OUT_FRAC) * (2 * self.P4_FRAC)

    # ------------------------------------------------------------ jc_log2 --
    def log2(self, x):
        """jc_log2: log2 of the INTEGER x, Q7.32. Zero in, zero out.

        The upper interpolation point is TAB[idx + 1] with idx + 1 computed
        WIDE -- the RTL wrapped this at IDX_BITS until the index was widened,
        which read log2(1)=0 instead of log2(2)=1.0 on one call in 2^IDX_BITS.
        """
        if x <= 0:
            return 0
        e = x.bit_length() - 1
        norm = (x << (self.LOG_IN_W - 1 - e)) & ((1 << self.LOG_IN_W) - 1)
        mant = (norm >> (self.LOG_IN_W - 1 - self.MANT_W)) \
            & ((1 << self.MANT_W) - 1)
        idx = mant >> self.FRC_BITS
        frc = mant & ((1 << self.FRC_BITS) - 1)
        slope = self.TAB[idx + 1] - self.TAB[idx]
        mant_log = self.TAB[idx] + ((slope * frc) >> self.FRC_BITS)
        return (e << self.OUT_FRAC) \
            + (mant_log << (self.OUT_FRAC - self.TAB_FRAC))

    # ----------------------------------------------------------- jc_cordic --
    def cordic_phi(self, px, py):
        """Vectoring CORDIC: atan2(py, px) as a binary angle, 2^32 per turn."""
        W, MSB = self.CORDIC_W, self.CORDIC_MSB

        # Quadrant fold: negating both components is exactly +pi = 2^31.
        fold = px < 0
        fx, fy = (-px, -py) if fold else (px, py)
        quad = (1 << (self.COORD_W - 1)) if fold else 0

        x = _signed(_tc(fx, W), W)
        y = _signed(_tc(fy, W), W)

        # Normalisation, from the LATCHED values as the RTL does.
        mag = _tc(abs(x), W) | _tc(abs(y), W)
        if mag == 0:
            return 0
        lead = mag.bit_length() - 1
        shift = MSB - lead if lead < MSB else 0
        x = _signed(_tc(x << shift, W), W)
        y = _signed(_tc(y << shift, W), W)

        z = 0
        for it in range(self.CORDIC_ITERS):
            step = self.ATAN[it]
            # z and the x/y update all read the PRE-iteration values, exactly
            # as non-blocking assignment does in the RTL.
            if y < 0:
                z_next = z - step
                nx, ny = x - (y >> it), y + (x >> it)
            else:
                z_next = z + step
                nx, ny = x + (y >> it), y - (x >> it)
            x = _signed(_tc(nx, W), W)
            y = _signed(_tc(ny, W), W)
            z = _tc(z_next, self.COORD_W)

        return _tc(z + quad, self.COORD_W)

    # ----------------------------------------------------------- jc_setkin --
    def set_kin(self, e, px, py, pz):
        """Merged four-momentum -> (y, phi, weight), all as the RTL holds them.

        Returns (y_val signed COORD_W, phi unsigned COORD_W, wgt unsigned
        WGT_W). Inputs are the Q14.34 integers, not floats.
        """
        P4_W, PROD_W, ACC_W = self.P4_W, self.PROD_W, self.ACC_W

        # ST_MUL1
        e_plus_pz = e + pz
        e_minus_pz = e - pz
        px2 = px * px
        py2 = py * py
        eplus = e + abs(pz)
        pz_pos = pz > 0

        # ST_MUL2 / ST_MUL3
        prod = e_plus_pz * e_minus_pz
        pt_sq = _tc(px2 + py2, PROD_W)

        # ST_MUL4: FastJet's max(0, m2) collapsed to a compare.
        prod_u = _tc(prod, PROD_W)
        prod_neg = (prod_u >> (PROD_W - 1)) & 1
        mt2 = pt_sq if (prod_neg or prod_u < pt_sq) else prod_u
        pt_zero = (pt_sq == 0)

        # Shared jc_log2, three issues.
        log_ptsq = self.log2(pt_sq)
        log_mt2 = self.log2(mt2)
        log_eplus = self.log2(eplus)

        # ST_RAP1: weight. Saturate rather than wrap on pt = 0, so such a row
        # loses every argmin instead of winning them all.
        if pt_zero:
            wgt = (1 << (self.WGT_W - 1)) - 1
        else:
            wgt_q32 = _signed(_tc(self.WGT_BIAS - log_ptsq, ACC_W), ACC_W)
            wgt_rnd = wgt_q32 + (1 << (self.LOG_TO_WGT - 1))
            # out_wgt is declared signed, and jc_model's trunc() returns a
            # signed value too -- slicing unsigned here costs exactly 2^WGT_W.
            wgt = _signed(
                _slice(wgt_rnd, ACC_W,
                       self.LOG_TO_WGT + self.WGT_W - 1, self.LOG_TO_WGT),
                self.WGT_W)

        # ST_RAP2/3: d = log2(mT2) - 2*log2(E+|pz|), always <= 0.
        d_log = _signed(_tc(log_mt2 - (log_eplus << 1), ACC_W), ACC_W)
        y_scaled = d_log * self.K_RAP
        y_mag = _signed(
            _slice(y_scaled, ACC_W + 33,
                   self.K_RAP_SHIFT + self.COORD_W - 1, self.K_RAP_SHIFT),
            self.COORD_W)
        if pt_zero:
            y_val = 0
        else:
            y_val = _signed(_tc(-y_mag if pz_pos else y_mag, self.COORD_W),
                            self.COORD_W)

        phi = 0 if pt_zero else self.cordic_phi(px, py)
        return y_val, phi, wgt
