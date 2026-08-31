// *************************************************************************
//
// jc_ingest -- calorimeter cell to pseudojet record, one cell per cycle.
//
// Cells arrive as grid indices, so nothing here needs a CORDIC, a logarithm
// or a square root. Everything is a table read plus four multiplies:
//
//   energy   = E[ecode]
//   pt       = energy * sech(y)          pt is internal; nothing downstream
//   pz       = energy * tanh(y)          needs it, setkin recomputes it
//   px       = pt     * cos(phi)
//   py       = pt     * sin(phi)
//   rapidity = y[iy]                     exact -- the bin IS the coordinate
//   phi      = iphi << JC_PHI_BIN_SHIFT  exact -- a shift, not a lookup
//   weight   = -2*log2(E) + -2*log2(sech(y))    one add, two tables
//
// The weight is log2(1/pt^2). It needs no logarithm in hardware because pt
// factorises into an energy term and a rapidity term, so its log splits into
// a sum of two quantities each indexed by a field that already arrived.
//
// Inputs are massless (E = |p|), which is what makes E = sqrt(pt^2 + pz^2)
// consistent with pt = E*sech(y) and pz = E*tanh(y): sech^2 + tanh^2 = 1.
//
// ONE SHARED INSTANCE serves every engine. Ingest is O(N) at ~128 cycles
// against a clustering loop in the tens of thousands, so its tables are paid
// once for the device rather than once per engine. That is the whole reason
// the LUT approach is affordable at this depth.
//
// EACH MULTIPLY OWNS A CYCLE, and the rounding add owns the next one. The
// four products are two dependent pairs -- pt feeds px and py -- and doing a
// pair's multiply and its round-and-shift together left the second pair's
// operand arriving through a whole DSP cascade plus a 48-bit carry chain:
// 3.819 ns, a 243 MHz unit in a 250 MHz design. Ingest is O(N) at ~128 cycles
// against a clustering loop in the tens of thousands, so the two extra stages
// are the cheapest fix anywhere in the design.
//
// Back-pressure freezes the pipeline wholesale rather than skidding: pj_ready
// is the clock enable and propagates straight back to cell_ready. jc_evbuf
// writes at one cell per cycle into BRAM and stalls only when both event
// slots are full, so a skid buffer here would cost registers to save nothing.
//
// LUT contents come from model/gen_luts.py; jc_luts.vh must be generated
// before this file will elaborate.
//
// *************************************************************************
`include "jc_defs.vh"
`timescale 1ns/1ps
module jc_ingest (
  // Cell stream from jc_deframe.
  input                          cell_valid,
  input  [`JC_CELL_W-1:0]        cell_data,
  input                          cell_start,
  input                          cell_last,
  // jc_deframe already knows this event is unsalvageable (a frame whose
  // length disagrees with its header count). Folded into the same error the
  // range check produces, so there is one failure path, not two.
  input                          cell_err,
  // Constant for the whole event, piped alongside rather than sampled at the
  // far end. jc_deframe can latch the NEXT event's header while a short event
  // is still inside these four stages, so by the time pj_last emerges its
  // event_seq output has already moved on. 128 flops in a device-shared unit
  // is the cheapest correct answer.
  input                   [31:0] cell_event_seq,
  output logic                   cell_ready,

  // Pseudojet record. Fixed 6-cycle latency; start/last ride along unchanged.
  output logic                          pj_valid,
  output logic signed [`JC_P4_W-1:0]    pj_energy,
  output logic signed [`JC_P4_W-1:0]    pj_px,
  output logic signed [`JC_P4_W-1:0]    pj_py,
  output logic signed [`JC_P4_W-1:0]    pj_pz,
  output logic signed [`JC_RAP_W-1:0]   pj_rapidity,
  output logic        [`JC_PHI_W-1:0]   pj_phi,
  output logic signed [`JC_WGT_W-1:0]   pj_beam_weight_log,
  output logic                          pj_start,
  output logic                          pj_last,
  output logic                   [31:0] pj_event_seq,
  // This cell's indices were outside the legal grid, or jc_deframe raised
  // cell_err on it. The record is still finite (the tables clamp), but the
  // event is untrustworthy -- jc_evbuf accumulates this across the event and
  // drops the whole thing, because a wrapped index silently reads a
  // neighbouring tower.
  output logic                          pj_err,
  input                                 pj_ready,

  input                                 aclk,
  input                                 aresetn
);

  localparam int RAP_BINS   = `JC_RAP_BINS;
  localparam int PHI_BINS   = `JC_PHI_BINS;
  localparam int TRIG_FRAC  = `JC_TRIG_FRAC;
  localparam int ECODE_BITS = `JC_ECODE_BITS;

  // pt = energy * trig, with energy in Q14.34 and trig in Q2.30, gives a
  // product with 64 fractional bits; Q14.34 wants 34, so every product is
  // shifted right by TRIG_FRAC. Same shift for px/py, since pt is also Q14.34.
  localparam int PROD_W = `JC_P4_W + `JC_TRIG_W;      // 80

  // ---- Stage 1: field decode and range check ---------------------------
  // Field widths are 8/8/16 but the legal ranges are 50/64/4096, so the
  // field being in range is not implied by it fitting. Check, then mask --
  // masking alone would turn an illegal index into a plausible neighbour.
  wire  [7:0] iy_raw    = cell_data[`JC_CELL_IY_LSB    +: `JC_CELL_IY_W];
  wire  [7:0] iphi_raw  = cell_data[`JC_CELL_IPHI_LSB  +: `JC_CELL_IPHI_W];
  wire [15:0] ecode_raw = cell_data[`JC_CELL_ECODE_LSB +: `JC_CELL_ECODE_W];

  // Any bit at or above ECODE_BITS means the code is past the table.
  wire range_err = cell_err
                || (iy_raw   >= RAP_BINS)
                || (iphi_raw >= PHI_BINS)
                || ((ecode_raw >> ECODE_BITS) != 0);

  logic  [5:0] iy_s1, iphi_s1;
  logic [11:0] ecode_s1;
  logic        valid_s1, start_s1, last_s1, err_s1;
  logic [31:0] seq_s1;

  // ---- Stage 2: table outputs ------------------------------------------
  logic signed [`JC_P4_W-1:0]   energy_s2;
  logic signed [`JC_TRIG_W-1:0] sech_s2, tanh_s2, cos_s2, sin_s2;
  logic signed [`JC_RAP_W-1:0]  rapidity_s2;
  logic signed [`JC_WGT_W-1:0]  neg2log2e_s2, neg2logsech_s2;
  logic        [`JC_PHI_W-1:0]  phi_s2;
  logic                         valid_s2, start_s2, last_s2, err_s2;
  logic                  [31:0] seq_s2;

  // ---- Stage 3a: the raw pt and pz products ----------------------------
  logic signed [PROD_W-1:0]     ptmul_s3a, pzmul_s3a;
  logic signed [`JC_P4_W-1:0]   energy_s3a;
  logic signed [`JC_TRIG_W-1:0] cos_s3a, sin_s3a;
  logic signed [`JC_RAP_W-1:0]  rapidity_s3a;
  logic signed [`JC_WGT_W-1:0]  weight_s3a;
  logic        [`JC_PHI_W-1:0]  phi_s3a;
  logic                         valid_s3a, start_s3a, last_s3a, err_s3a;
  logic                  [31:0] seq_s3a;

  // ---- Stage 3: pt, pz, weight -----------------------------------------
  logic signed [`JC_P4_W-1:0]   energy_s3, pt_s3, pz_s3;
  logic signed [`JC_TRIG_W-1:0] cos_s3, sin_s3;
  logic signed [`JC_RAP_W-1:0]  rapidity_s3;
  logic signed [`JC_WGT_W-1:0]  weight_s3;
  logic        [`JC_PHI_W-1:0]  phi_s3;
  logic                         valid_s3, start_s3, last_s3, err_s3;
  logic                  [31:0] seq_s3;

  // ---- Stage 4a: the raw px and py products ----------------------------
  logic signed [PROD_W-1:0]     pxmul_s4a, pymul_s4a;
  logic signed [`JC_P4_W-1:0]   energy_s4a, pz_s4a;
  logic signed [`JC_RAP_W-1:0]  rapidity_s4a;
  logic signed [`JC_WGT_W-1:0]  weight_s4a;
  logic        [`JC_PHI_W-1:0]  phi_s4a;
  logic                         valid_s4a, start_s4a, last_s4a, err_s4a;
  logic                  [31:0] seq_s4a;

  // ---- Lookup tables ---------------------------------------------------
  // Names are the contract with model/gen_luts.py -- see jc_defs.vh.
  logic signed [`JC_P4_W-1:0]   jc_lut_energy      [0:`JC_LUT_DEPTH_E-1];
  logic signed [`JC_WGT_W-1:0]  jc_lut_neg2log2e   [0:`JC_LUT_DEPTH_E-1];
  logic signed [`JC_TRIG_W-1:0] jc_lut_sech        [0:`JC_LUT_DEPTH_BIN-1];
  logic signed [`JC_TRIG_W-1:0] jc_lut_tanh        [0:`JC_LUT_DEPTH_BIN-1];
  logic signed [`JC_RAP_W-1:0]  jc_lut_rapidity    [0:`JC_LUT_DEPTH_BIN-1];
  logic signed [`JC_WGT_W-1:0]  jc_lut_neg2logsech [0:`JC_LUT_DEPTH_BIN-1];
  logic signed [`JC_TRIG_W-1:0] jc_lut_cos         [0:`JC_LUT_DEPTH_BIN-1];
  logic signed [`JC_TRIG_W-1:0] jc_lut_sin         [0:`JC_LUT_DEPTH_BIN-1];

  `include "jc_luts.vh"

  // ---- Products --------------------------------------------------------
  // Both operands signed, so these elaborate as signed multiplies. Rounding
  // is add-half-then-shift; the true result always fits JC_P4_W, since
  // |trig| <= 1 and energy is bounded by the top of the energy table.
  localparam logic signed [PROD_W-1:0] PROD_ONE  = 1;
  localparam logic signed [PROD_W-1:0] HALF_LSB  = PROD_ONE <<< (TRIG_FRAC - 1);

  // Bit range of the shifted result: (prod >>> TRIG_FRAC) truncated to P4_W.
  localparam int PROD_MSB = TRIG_FRAC + `JC_P4_W - 1;   // 77

  // The multiply and the rounding add are evaluated a cycle apart, but in the
  // same PROD_W context they always were, so every product and every rounded
  // result is bit-identical to the single-cycle version.
  wire signed [PROD_W-1:0] mul_pt = energy_s2 * sech_s2;
  wire signed [PROD_W-1:0] mul_pz = energy_s2 * tanh_s2;
  wire signed [PROD_W-1:0] mul_px = pt_s3     * cos_s3;
  wire signed [PROD_W-1:0] mul_py = pt_s3     * sin_s3;

  wire signed [PROD_W-1:0] prod_pt = ptmul_s3a + HALF_LSB;
  wire signed [PROD_W-1:0] prod_pz = pzmul_s3a + HALF_LSB;
  wire signed [PROD_W-1:0] prod_px = pxmul_s4a + HALF_LSB;
  wire signed [PROD_W-1:0] prod_py = pymul_s4a + HALF_LSB;

  // ---- Flow ------------------------------------------------------------
  wire en = pj_ready;
  assign cell_ready = pj_ready;

  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      valid_s1  <= 1'b0;
      valid_s2  <= 1'b0;
      valid_s3a <= 1'b0;
      valid_s3  <= 1'b0;
      valid_s4a <= 1'b0;
      pj_valid  <= 1'b0;
    end
    else if (en) begin
      // Stage 1 -- decode and mask.
      iy_s1    <= iy_raw[5:0];
      iphi_s1  <= iphi_raw[5:0];
      ecode_s1 <= ecode_raw[ECODE_BITS-1:0];
      valid_s1 <= cell_valid;
      start_s1 <= cell_start;
      last_s1  <= cell_last;
      err_s1   <= range_err;
      seq_s1   <= cell_event_seq;

      // Stage 2 -- table reads. Registered output, so these infer ROMs.
      energy_s2       <= jc_lut_energy     [ecode_s1];
      neg2log2e_s2    <= jc_lut_neg2log2e  [ecode_s1];
      sech_s2         <= jc_lut_sech       [iy_s1];
      tanh_s2         <= jc_lut_tanh       [iy_s1];
      rapidity_s2     <= jc_lut_rapidity   [iy_s1];
      neg2logsech_s2  <= jc_lut_neg2logsech[iy_s1];
      cos_s2          <= jc_lut_cos        [iphi_s1];
      sin_s2          <= jc_lut_sin        [iphi_s1];
      // phi needs no table: 64 bins into a 32-bit binary angle is exactly
      // 2^26 per bin, so the index is the top 6 bits of the angle.
      phi_s2          <= {iphi_s1, {`JC_PHI_BIN_SHIFT{1'b0}}};
      valid_s2 <= valid_s1;
      start_s2 <= start_s1;
      last_s2  <= last_s1;
      err_s2   <= err_s1;
      seq_s2   <= seq_s1;

      // Stage 3a -- the pt and pz products, and the weight as a single add.
      // Nothing but the multiply happens here; the rounding is next cycle.
      ptmul_s3a    <= mul_pt;
      pzmul_s3a    <= mul_pz;
      weight_s3a   <= neg2log2e_s2 + neg2logsech_s2;
      energy_s3a   <= energy_s2;
      rapidity_s3a <= rapidity_s2;
      phi_s3a      <= phi_s2;
      cos_s3a      <= cos_s2;
      sin_s3a      <= sin_s2;
      valid_s3a <= valid_s2;
      start_s3a <= start_s2;
      last_s3a  <= last_s2;
      err_s3a   <= err_s2;
      seq_s3a   <= seq_s2;

      // Stage 3 -- round the pt and pz products back into Q14.34.
      pt_s3       <= prod_pt[PROD_MSB:TRIG_FRAC];
      pz_s3       <= prod_pz[PROD_MSB:TRIG_FRAC];
      weight_s3   <= weight_s3a;
      energy_s3   <= energy_s3a;
      rapidity_s3 <= rapidity_s3a;
      phi_s3      <= phi_s3a;
      cos_s3      <= cos_s3a;
      sin_s3      <= sin_s3a;
      valid_s3 <= valid_s3a;
      start_s3 <= start_s3a;
      last_s3  <= last_s3a;
      err_s3   <= err_s3a;
      seq_s3   <= seq_s3a;

      // Stage 4a -- the px and py products, off the rounded pt.
      pxmul_s4a    <= mul_px;
      pymul_s4a    <= mul_py;
      pz_s4a       <= pz_s3;
      energy_s4a   <= energy_s3;
      rapidity_s4a <= rapidity_s3;
      phi_s4a      <= phi_s3;
      weight_s4a   <= weight_s3;
      valid_s4a <= valid_s3;
      start_s4a <= start_s3;
      last_s4a  <= last_s3;
      err_s4a   <= err_s3;
      seq_s4a   <= seq_s3;

      // Stage 4 -- round px and py, and out.
      pj_px              <= prod_px[PROD_MSB:TRIG_FRAC];
      pj_py              <= prod_py[PROD_MSB:TRIG_FRAC];
      pj_pz              <= pz_s4a;
      pj_energy          <= energy_s4a;
      pj_rapidity        <= rapidity_s4a;
      pj_phi             <= phi_s4a;
      pj_beam_weight_log <= weight_s4a;
      pj_valid <= valid_s4a;
      pj_start <= start_s4a;
      pj_last  <= last_s4a;
      pj_err   <= err_s4a;
      pj_event_seq <= seq_s4a;
    end
  end

endmodule: jc_ingest
