// *************************************************************************
//
// jc_defs.vh -- shared constants for the jet clustering datapath.
//
// Plain `define rather than a SystemVerilog package: Icarus 11 does not
// resolve package identifiers inside an ANSI port list, so a port width like
// [CELL_W-1:0] fails to parse even with `import jc_pkg::*` in the header.
// Macros are Verilog-2001 and work in every tool in the flow.
//
// Convention: every macro is JC_ prefixed, and modules mirror the ones they
// use into localparams so the body stays readable.
//
// Every width here is fixed by CLAUDE.md's input contract and numerical
// formats section. Nothing is a free choice at build time except JC_LANES.
//
// *************************************************************************
`ifndef JC_DEFS_VH
`define JC_DEFS_VH

// =======================================================================
// Event sizing
// =======================================================================
`define JC_NMAX   128        // max calorimeter cells resident per event
`define JC_LANES  16         // distance lanes per engine
`define JC_IDX_W  7          // $clog2(JC_NMAX), addresses 0..127
`define JC_CNT_W  8          // JC_IDX_W+1, counts 0..128

// Active-list banking. Entries are spread cyclically over LANES banks, so
// entry k lives in bank k[LANE_W-1:0] at offset k[IDX_W-1:LANE_W]. Keeping
// LANES a power of two makes that split a wire split rather than a divide,
// and means lane l addresses only bank l -- no crossbar on the read, and no
// possible collision on the write-back.
`define JC_DEPTH   8         // JC_NMAX / JC_LANES, entries per bank
`define JC_LANE_W  4         // $clog2(JC_LANES), selects the bank
`define JC_OFF_W   3         // $clog2(JC_DEPTH), selects within a bank

// =======================================================================
// Calorimeter grid -- LHC trigger-tower granularity
// =======================================================================
`define JC_RAP_BINS 50       // dy = 0.1 over |y| < 2.5
`define JC_PHI_BINS 64       // dphi = 2*pi/64

// A bin index names its CENTRE, and the host pixeliser must agree -- this is
// the one convention that cannot be checked in hardware, only versioned.
//
//   rapidity(iy)  = -2.45 + 0.1*iy          iy   = 0..49
//   phi(iphi)     = 2*pi*iphi/64            iphi = 0..63
//
// The two are offset differently on purpose. Rapidity is a bounded range, so
// 50 centres inside |y| < 2.5 sit at half-bin offsets and never land on
// y = 0. Phi is periodic, so putting centre 0 at phi = 0 costs nothing and
// keeps phi = iphi << JC_PHI_BIN_SHIFT exact with no additive term.
//
// Index range is NOT implied by the field width: iy has 50 legal values in an
// 8-bit field and iphi has 64. jc_ingest flags anything above and jc_evbuf
// drops the event -- a silent wrap would read a neighbouring tower.

// =======================================================================
// Wire format
// =======================================================================
// Cell, 4 bytes. 4 B divides a 64 B beat exactly: 16 cells per beat, always,
// so cell index is word index on every beat with no offset term.
`define JC_CELL_W         32
`define JC_CELLS_PER_BEAT 16

// Field positions within a cell word: {iy[7:0], iphi[7:0], ecode[15:0]}.
// Spelled out so the bit layout is visible rather than implied by
// declaration order.
`define JC_CELL_ECODE_LSB 0
`define JC_CELL_ECODE_W   16
`define JC_CELL_IPHI_LSB  16
`define JC_CELL_IPHI_W    8
`define JC_CELL_IY_LSB    24
`define JC_CELL_IY_W      8

// Energy code: the field is 16 bits so the format is byte-aligned and has
// room to grow, but only the low JC_ECODE_BITS index the energy LUT.
// Widening this multiplies LUT depth, so it is deliberately not the field
// width.
`define JC_ECODE_BITS  12
`define JC_ECODE_DEPTH 4096

// -----------------------------------------------------------------------
// Event header, occupying beat 0 entirely
// -----------------------------------------------------------------------
//   bytes  0..5   destination MAC
//   bytes  6..11  source MAC
//   bytes 12..13  ethertype, JC_ETH_TYPE
//   byte  14      format version, JC_FMT_VERSION
//   byte  15      pad, keeps the count 2-byte aligned
//   bytes 16..17  cell count N            (big-endian, network order)
//   bytes 18..21  event sequence number   (big-endian, 32-bit)
//   bytes 22..23  cells before truncation (big-endian)
//   bytes 24..63  reserved, zero
//
// Cells start at beat 1, 16 per beat, last beat partial per tkeep.
//
// Giving the header a whole beat sometimes costs one extra beat on the wire
// (7 -> 8 at N=100; equal at N=44 and N=128). That buys nothing back because
// deframe emits one cell per cycle, so ingest is cell-limited, not
// beat-limited. What it does buy:
//   - cell index IS word index on every beat, so extraction never carries an
//     offset term and cells-per-beat from tkeep stays uniform;
//   - header decode and cell extraction are separate states that cannot
//     alias on tdata;
//   - count, sequence and total are latched a full beat before the first
//     cell exists, so jc_evbuf can decide drop-or-accept and the engine can
//     clear its active list with no lookahead or stall.
//
// No truncation flag: cells_total > N is the flag, and it additionally says
// by how much, which is what tells you whether NMAX is right at all.
//
// Deliberately absent: R and the jet pt floor live in AXI-Lite registers --
// properties of an analysis, not of an event, so a per-event field would
// spend bytes forever on something set once per run.
`define JC_HDR_BEATS 1
`define JC_ETH_TYPE  16'h88B6   // local experimental; graph plugin owns 88B5

// Bump whenever the cell encoding OR the grid changes. Without this a host
// pixelising on a different grid than the LUTs assume would be silently
// misread rather than rejected.
`define JC_FMT_VERSION 8'h01

// Byte offsets within beat 0
`define JC_HDR_OFF_ETHTYPE 12
`define JC_HDR_OFF_VERSION 14
`define JC_HDR_OFF_COUNT   16
`define JC_HDR_OFF_SEQ     18
`define JC_HDR_OFF_TOTAL   22

// -----------------------------------------------------------------------
// Jets frame -- what leaves the card
// -----------------------------------------------------------------------
//   bytes  0..5   destination MAC
//   bytes  6..11  source MAC
//   bytes 12..13  ethertype, JC_JET_ETH_TYPE
//   byte  14      format version, JC_JET_FMT_VERSION
//   byte  15      pad
//   bytes 16..17  jet count                (big-endian)
//   bytes 18..21  event sequence number    (big-endian, echoes the input)
//   bytes 22..25  clustering cycles for this event
//   bytes 26..29  events dropped, no free slot
//   bytes 30..33  events dropped, bad cell or aborted frame
//   bytes 34..37  frames rejected by jc_deframe
//   bytes 38..63  reserved, zero
//
// Then 4 jets per beat, 16 B each, big-endian fp32 {E, px, py, pz}.
//
// A DIFFERENT ETHERTYPE FROM THE INPUT, and that is the whole point of
// spending a second one. A jets frame looped back into an RX port must be
// rejected by jc_deframe rather than parsed as sixteen cells per beat, which
// is exactly what reusing 0x88B6 would produce -- plausible jets from jet
// bytes, the failure mode nothing downstream detects.
//
// The event sequence number is echoed from the input header, so a host pairs
// jets with the event that produced them without relying on arrival order.
//
// COUNTERS RIDE ALONG because the header has the room and a host watching
// only the network would otherwise never see them. They are a bonus channel,
// not the measurement: an event that produces no jet emits no frame, so at a
// 50 GeV floor on a soft sample the card is silent most of the time and these
// bytes go with it. AXI-Lite is where the counters are actually read.
//
// cycle_count is the engine's own measurement of the event just finished --
// the step 9 number, reported per event with no instrumentation to add.
`define JC_JET_ETH_TYPE     16'h88B7
// Version 2 adds port-to-port cycles and the per-event stale/refresh counts.
// Bumped so a host tool from before them reads a frame it does not understand
// and says so, rather than silently reporting zeros as measurements.
`define JC_JET_FMT_VERSION  8'h02
`define JC_JET_BYTES        16      // fp32 x 4
`define JC_JETS_PER_BEAT    4       // 64 / JC_JET_BYTES, so jet index IS word index

`define JC_JHDR_OFF_ETHTYPE 12
`define JC_JHDR_OFF_VERSION 14
`define JC_JHDR_OFF_NJETS   16
`define JC_JHDR_OFF_SEQ     18
`define JC_JHDR_OFF_CYCLES  22
`define JC_JHDR_OFF_DROP_FULL 26
`define JC_JHDR_OFF_DROP_ERR  30
`define JC_JHDR_OFF_BAD_FRAME 34
// ---- Per-event instrumentation, added at jet format version 2 -----------
// PORT-TO-PORT is the honest latency number. JC_JHDR_OFF_CYCLES counts only
// while jc_ctrl is out of S_IDLE, so it excludes CMAC RX, deframe, ingest,
// evbuf queuing, reframe and CMAC TX -- about 1.5 us. Noise against the
// original 66 us, but several percent once the engine is under 45, and the
// difference between a measured port-to-port figure and an estimate.
//
// STALE and REFRESH are the two DATA-DEPENDENT terms in the engine's cost.
// Inferring them rather than counting them had them wrong by 5.4x and 27%,
// which hid the entire stale-rescan cost. Simulation can sample them on one
// event in ~20 s; the card samples a thousand in seconds, so they ride here
// and every hardware run becomes a census over real data.
`define JC_JHDR_OFF_P2P       38
`define JC_JHDR_OFF_STALE     42
`define JC_JHDR_OFF_REFRESH   44

// =======================================================================
// Fixed-point formats
// =======================================================================
// Four-momentum: Q14.34 signed. 14 integer bits from maximum cell energy,
// 34 fractional. Adds are exact, so a merged jet carries no accumulated
// rounding and is independent of merge order.
`define JC_P4_W    48
`define JC_P4_FRAC 34

// -----------------------------------------------------------------------
// Coordinates -- ONE SHARED SCALE for rapidity and azimuth
// -----------------------------------------------------------------------
// dR^2 = dy^2 + dphi^2 is only meaningful if both deltas carry the same
// scale, so both use binary-angle units: 2^32 per full turn, i.e.
//
//     JC_COORD_SCALE = 2^32 / (2*pi) = 683565275.576...  units per radian
//
// Azimuth sets the scale rather than rapidity because it is the one that
// buys something: phi wraps EXACTLY on 32-bit overflow, so the 0/2*pi seam
// needs no compare-and-correct in any lane, and 64 phi bins land on exactly
// 2^32/64 = 2^26 so phi = iphi << 26 stays a shift. Putting phi on a Q-format
// instead would cost an explicit wrap fixup per lane per cycle, and matching
// the two scales any other way costs a multiply by 8/(2*pi) in the same
// place. Both are exactly what the per-row asymmetric distance exists to
// avoid.
//
// The absolute scale is arbitrary -- R^2 is a register and scales with it.
// Only the SHARED part matters.
//
// Rapidity fits: |y| < 2.5 -> |y_int| < 1.71e9, inside int32's 2.15e9.
// The tables are emitted at this scale by model/gen_luts.py.
`define JC_COORD_W       32
`define JC_RAP_W         32     // alias, same coordinate word
`define JC_PHI_W         32     // alias, same coordinate word
`define JC_PHI_BIN_SHIFT 26

// Deltas are reduced to an UNSIGNED MAGNITUDE before the shift, because the
// squarer does not care about sign and carrying it costs correctness:
// an arithmetic shift floors, and floor(-x/2^s) != -floor(x/2^s) unless x
// divides evenly, so dR^2(a,b) would differ from dR^2(b,a). Taking |d| first
// makes the shift a truncation toward zero, which IS symmetric -- and row i
// and row j must agree on their mutual distance or the nearest-neighbour
// table is inconsistent with itself.
//
// Dropping the sign also buys a bit: JC_DELTA_W is a magnitude, so the same
// multiplier width now reaches 2^24-1 instead of 2^23-1.
//
// dy spans +/-4.9 rad and overflows int32 at this scale, so the subtract is
// done 34 bits wide, then shifted and SATURATED. Saturation is lossless for
// the decision: the largest representable magnitude is
// (2^JC_DELTA_W - 1) << JC_DELTA_SHIFT ~= 2^31 units = pi, so a saturated
// delta already has dR^2 > R^2 for every usable R and routes to the beam
// whatever its true value was. |dphi| <= pi always, so only dy can saturate
// in practice.
//
// DELTA_SHIFT is the main DSP lever -- it sets multiplier width directly.
// At 7, the LSB is 128 units = 1.87e-7 rad, so dR^2 near R = 0.4 carries
// about 1e-6 relative. That is a ranking input, not an arithmetic one: the
// merge itself is exact integer adds either way.
`define JC_DELTA_W     24
`define JC_DELTA_SHIFT 7

// Each square is at most (2^24-1)^2 < 2^48, so their sum needs 49 bits and
// carries zero rounding beyond the delta truncation above.
`define JC_GEO_W   49

// Nearest-neighbour distance in the log domain: beam_weight_log + log2(geo).
// The weight spans about -22..+10 and log2(geo) spans 0..49, so the sum needs
// 7 integer bits on top of JC_WGT_FRAC, plus a sign. 40 leaves margin.
//
// The SAME distance is also kept linearly as JC_GEO_W bits (nn_geo). That
// looks redundant and is not: the write-back during a row scan has to ask
// "is this closer than row k's current best", and in the log domain that
// costs a logarithm in every lane. Monotonicity turns it into a plain
// integer compare on nn_geo. See CLAUDE.md, active-list record.
`define JC_NNLOG_W 40

// Per-entry sweep state: {active, beam, nn_index, nn_geo, nn_dist_log}.
// beam is a separate bit rather than a reserved nn_index value, so every
// 7-bit index stays a legal neighbour.
`define JC_NN_W (1 + 1 + `JC_IDX_W + `JC_GEO_W + `JC_NNLOG_W)

// Beam weight, log2(1/pt^2): Q7.25 signed, range +/-64. pt from 0.2 GeV
// upward spans several decades, which no linear fixed-point word holds.
// Reachable range is about -22 .. +10, so this has wide headroom.
`define JC_WGT_W    32
`define JC_WGT_FRAC 25

// Trig LUT entries -- sech, tanh, cos, sin: Q2.30 signed, range +/-2.
// Two integer bits, not one: every value is in [-1,1] but cos(0) = 1 exactly
// and Q1.31 cannot represent 1.0. LSB 9.3e-10, so the ingest conversion sits
// far below the 1e-4 match criterion.
`define JC_TRIG_W    32
`define JC_TRIG_FRAC 30

// =======================================================================
// Shared log2 unit
// =======================================================================
// Every logarithm in the engine goes through one jc_log2: the beam weight
// and the rapidity in jc_setkin, and the nn_dist_log refresh for rows the
// sweep write-back claimed. Keeping it in one place is what allows jc_sweep's
// lanes to be pure geometry.
//
// log2(x) = e + log2(m), with e the leading-one position and m in [1,2).
// log2(m) comes from a table on the top JC_LOG2_IDX_BITS mantissa bits with
// linear interpolation over the next JC_LOG2_FRAC_BITS.
//
// TWO error terms, and they are set by different knobs:
//
//   interpolation residual   0.18 * 2^-2*IDX_BITS   = 1.1e-8 at 12
//   mantissa truncation      1.44 * 2^-MANT_BITS    = 3.4e-10 at 32
//
// where MANT_BITS = IDX_BITS + FRAC_BITS. Only the first depends on table
// size; the second is the mantissa simply being cut off, and costs a wider
// interpolation multiply rather than more memory -- which is why FRAC_BITS
// is 20 and not 12. At 12 the mantissa was 24 bits and truncation alone
// gave 8.7e-8, eight times the residual it was paired with.
//
// The 1.1e-8 total matters for rapidity, not the weight: a rapidity error of
// 1.1e-8 rad is 6% of one delta LSB (1.87e-7 rad), so it disappears into the
// distance quantisation. IDX_BITS=10 would have been 1.7e-7, a whole LSB.
//
// The table has DEPTH = 2^k + 1 entries so the interpolation can read i and
// i+1 from a true dual-port BRAM and subtract, rather than storing a second
// slope table. Entries are UNSIGNED Q1.31: the values are log2(1+u) over
// [0,1], and the last one is exactly 1.0 = 2^31, which fits an unsigned
// 32-bit word exactly but overflows a signed one.
`define JC_LOG2_IDX_BITS  12
`define JC_LOG2_FRAC_BITS 20
`define JC_LOG2_DEPTH     4097          // 2^JC_LOG2_IDX_BITS + 1
`define JC_LOG2_TAB_W     32            // Q1.31
`define JC_LOG2_TAB_FRAC  31

// Widest thing ever fed to it: (E+pz)*(E-pz) with both 49 bits.
`define JC_LOG2_IN_W  98
`define JC_LOG2_EXP_W 7                 // 0..97

// Result is Q7.32: 7 integer bits for the exponent, 32 fractional.
`define JC_LOG2_OUT_W    39
`define JC_LOG2_OUT_FRAC 32

// =======================================================================
// CORDIC, vectoring mode -- azimuth of a merged jet
// =======================================================================
// atan2(py, px) once per merge. CORDIC rather than a log-domain ratio and
// table: it needs no division, no exponential and no large table, and it
// handles every magnitude uniformly, which matters because a merged jet's
// px or py can sit anywhere from the pt floor to a TeV.
//
// Iterative, one rotation per cycle. Latency is what costs here, not
// throughput -- this runs once per merge, against roughly a hundred cycles
// of sweeps -- so an unrolled array would buy nothing and cost 28x the area.
//
// Each iteration resolves about one more bit of angle, so N sets precision:
// the residual is ~2^-N rad and the target is one delta LSB, 1.87e-7 rad =
// 2^-22.3. 28 leaves comfortable margin.
//
// Inputs are normalised first. The pt floor puts max(|px|,|py|) at about
// 2^31 in Q14.34 integer terms, and shifting right by 27 in the last
// iterations would leave four significant bits; normalising to the top of
// the internal word keeps every rotation well resolved.
// Width and normalisation target are coupled, and getting it wrong overflows
// silently on the diagonals. After normalising the LARGER component to bit
// MSB it can reach 2^(MSB+1); the vector magnitude is then up to sqrt(2)
// times that, and CORDIC's 1.647 gain multiplies it again:
//
//     MSB + 1 + 0.5 (sqrt 2) + 0.72 (gain)  <  W - 1   (signed)
//
// so MSB <= W - 4.3. At W=56, MSB=50 leaves about 1.7 bits spare.
`define JC_CORDIC_ITERS 28
`define JC_CORDIC_W     56
`define JC_CORDIC_MSB   50    // normalise the larger input to here

// atan(2^-i) in the SAME units as phi -- radians * 2^32/(2*pi) -- so the
// accumulated angle is already a binary angle and needs no final scaling.
// atan(1) = pi/4 maps to exactly 2^29.
`define JC_LUT_ATAN_DEPTH `JC_CORDIC_ITERS

// =======================================================================
// Ingest LUT array names -- the contract with model/gen_luts.py
// =======================================================================
// gen_luts.py emits jc_luts.vh as initial blocks assigning these arrays by
// name, so a rename here without a rename there fails at elaboration rather
// than silently producing a zero table.
//
//   jc_lut_energy    [0:4095]  Q14.34   E in GeV
//   jc_lut_neg2log2e [0:4095]  Q7.25    -2*log2(E)
//   jc_lut_sech      [0:63]    Q2.30    sech(y)
//   jc_lut_tanh      [0:63]    Q2.30    tanh(y)
//   jc_lut_rapidity  [0:63]    Q3.29    y
//   jc_lut_neg2logsech[0:63]   Q7.25    -2*log2(sech(y))
//   jc_lut_cos       [0:63]    Q2.30    cos(phi)
//   jc_lut_sin       [0:63]    Q2.30    sin(phi)
//
// The iy tables are 64 deep for a natural address width; entries 50..63 are
// filled with the iy=49 values so an out-of-range index still yields finite
// numbers alongside its error flag, rather than a pt of zero whose log is
// negative infinity.
//
// Note: if the final energy mapping stays log-spaced, jc_lut_neg2log2e is
// affine in ecode and collapses to one constant multiply. It is kept as a
// table because the mapping is still an open physics item and a table costs
// nothing in a unit shared by the whole device.
`define JC_LUT_DEPTH_E   4096
`define JC_LUT_DEPTH_BIN 64

// Generated scalar constants (JC_K_RAP and friends) ride along here, so a
// module that includes jc_defs.vh has everything and there is no second
// include to forget.
`include "jc_consts.vh"

`endif
