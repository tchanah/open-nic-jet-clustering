// *************************************************************************
//
// jc_dist -- one distance lane: dR^2 between two pseudojets.
//
// This is the ONLY replicated arithmetic in the design, LANES x ENGINES of
// it, so its width and depth set both the DSP budget and Fmax. Everything
// else -- ingest, setkin, reframe -- runs once per cell or once per merge.
//
// Pure geometry, no weight and no logarithm. Within one row scan the anti-kt
// weight min(1/pt_i^2, 1/pt_j^2) collapses to the constant w_i, so ranking by
// w_i*dR^2 is ranking by dR^2 alone; the weight is applied once per sweep in
// jc_sweep, not once per comparison here. That is what lets this be five
// operations instead of a multiply and a log.
//
//   dy   = y_q   - y_k                subtract
//   dphi = phi_q - phi_k              subtract, wraps free
//   dR^2 = trunc(dy)^2 + trunc(dphi)^2
//
// Both coordinates share one scale, 2^32 per full turn -- see jc_defs.vh.
// phi therefore wraps EXACTLY on 32-bit overflow: reinterpreting the raw
// 32-bit difference as signed already gives the shortest separation, so the
// 0/2*pi seam costs nothing here. That is the whole reason rapidity was moved
// onto azimuth's scale rather than the other way round.
//
// Deltas saturate rather than wrap after the shift. That is lossless for the
// decision being made: the largest representable delta is exactly pi/2, so a
// saturated lane already has dR^2 > R^2 for any R < pi/2 and routes to the
// beam whatever its true value was.
//
// Fixed 3-cycle latency, no handshake. Validity is jc_sweep's business --
// this block is fed unconditionally and its result masked downstream, which
// keeps the replicated logic to arithmetic only.
//
// *************************************************************************
`include "jc_defs.vh"
`timescale 1ns/1ps
module jc_dist (
  input  signed [`JC_COORD_W-1:0] y_q,
  input         [`JC_COORD_W-1:0] phi_q,
  input  signed [`JC_COORD_W-1:0] y_k,
  input         [`JC_COORD_W-1:0] phi_k,

  output logic  [`JC_GEO_W-1:0]   geo_dist_sq,

  input                           en,
  input                           aclk,
  input                           aresetn
);

  localparam int COORD_W = `JC_COORD_W;
  localparam int DELTA_W = `JC_DELTA_W;
  localparam int SHIFT   = `JC_DELTA_SHIFT;

  // Two extra bits: one for the subtract's growth, one so the shift and the
  // saturation compares have room to work in.
  localparam int WIDE_W = COORD_W + 2;

  // ---- Stage 1: deltas, magnitude, shift, saturate ---------------------
  wire signed [WIDE_W-1:0] dy_wide =
      $signed({{2{y_q[COORD_W-1]}}, y_q}) - $signed({{2{y_k[COORD_W-1]}}, y_k});

  // Modulo-2^32 subtract. A full turn IS 2^32, so the raw difference read as
  // signed is already the shortest separation -- no compare, no correction.
  wire [COORD_W-1:0] dphi_raw = phi_q - phi_k;
  wire signed [COORD_W-1:0] dphi_signed = $signed(dphi_raw);

  // Magnitude BEFORE the shift. A signed >>> floors, and floor(-x/2^s) is not
  // -floor(x/2^s), so shifting first would make dR^2(a,b) differ from
  // dR^2(b,a) and leave the nearest-neighbour table disagreeing with itself.
  // On a magnitude the shift is a truncation toward zero, which is symmetric.
  wire [WIDE_W-1:0]  dy_abs   = dy_wide[WIDE_W-1]        ? -dy_wide     : dy_wide;
  wire [COORD_W-1:0] dphi_abs = dphi_signed[COORD_W-1]   ? -dphi_signed : dphi_signed;

  wire [WIDE_W-1:0]  dy_sh   = dy_abs   >> SHIFT;
  wire [COORD_W-1:0] dphi_sh = dphi_abs >> SHIFT;

  // Full-scale magnitude: no sign bit to give up, so this reaches ~pi.
  localparam [DELTA_W-1:0] SAT_MAG = {DELTA_W{1'b1}};

  wire [DELTA_W-1:0] dy_sat   = (dy_sh   > SAT_MAG) ? SAT_MAG : dy_sh[DELTA_W-1:0];
  wire [DELTA_W-1:0] dphi_sat = (dphi_sh > SAT_MAG) ? SAT_MAG : dphi_sh[DELTA_W-1:0];

  logic [DELTA_W-1:0] dy_s1, dphi_s1;

  // ---- Stage 2: squares ------------------------------------------------
  // Unsigned magnitudes, so these are unsigned squares and the sum needs no
  // sign bit. This is the DSP cost: two DELTA_W x DELTA_W per lane, which is
  // why DELTA_W is the lever rather than the coordinate width.
  logic [2*DELTA_W-1:0] sq_y_s2, sq_phi_s2;

  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      dy_s1       <= '0;
      dphi_s1     <= '0;
      sq_y_s2     <= '0;
      sq_phi_s2   <= '0;
      geo_dist_sq <= '0;
    end
    else if (en) begin
      dy_s1   <= dy_sat;
      dphi_s1 <= dphi_sat;

      sq_y_s2   <= dy_s1   * dy_s1;
      sq_phi_s2 <= dphi_s1 * dphi_s1;

      // Each square is at most 2^(2*DELTA_W-2), so the sum fits JC_GEO_W
      // with no rounding beyond the delta truncation above.
      geo_dist_sq <= sq_y_s2 + sq_phi_s2;
    end
  end

endmodule: jc_dist
