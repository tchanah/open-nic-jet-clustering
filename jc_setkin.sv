// *************************************************************************
//
// jc_setkin -- recompute a merged jet's coordinates from its four-momentum.
//
// Runs once per merge. The four-momentum add itself is exact integer
// arithmetic in jc_ctrl; this is the part that has to turn (E,px,py,pz) back
// into the (rapidity, phi, weight) the next round's sweeps rank by.
//
//   pt_sq  = px^2 + py^2                       exact, two multiplies
//   mT2    = max(pt_sq, (E+pz)(E-pz))          see below
//   phi    = atan2(py, px)                     jc_cordic
//   y      = +/- ln2 * (log2(mT2)/2 - log2(E+|pz|))
//   weight = log2(1/pt^2) = -log2(pt_sq)
//
// RAPIDITY USES FASTJET'S STABLE FORM, and bit-agreement depends on it.
// FastJet computes 0.5*log((kt2 + m2)/(E+|pz|)^2) with m2 clamped at zero,
// rather than the textbook 0.5*log((E+pz)/(E-pz)). Dividing by (E+|pz|)^2 is
// well conditioned because it is a sum; forming E-pz alone is not, and a
// merged jet close to the beam has E-pz small.
//
// The clamp collapses to a compare. m2 = (E+pz)(E-pz) - pt_sq clamped at 0,
// and mT2 = pt_sq + m2, so mT2 = max(pt_sq, (E+pz)(E-pz)) -- no subtraction
// and no separate clamp.
//
// The fixed-point offsets cancel exactly, which is worth stating because it
// looks like they should not. pt_sq and mT2 are Q28.68 and E+|pz| is Q15.34,
// so working from integer logs:
//
//   y = 0.5*(log2(mT2_int) - 68) - (log2(Eplus_int) - 34)
//     = 0.5*log2(mT2_int) - log2(Eplus_int)
//
// -- the 68 and the 34 annihilate, so no bias term is needed on this path.
//
// One jc_log2 serves all three logarithms, issued back to back into its
// pipeline, and is exposed to jc_ctrl while idle so the nn_dist_log refresh
// for rows the sweep claimed shares the same unit. That is the whole reason
// jc_sweep's sixteen lanes can be pure geometry. Responses are routed by a
// tag pipeline matched to the log's latency, so an external request already
// in flight when a merge starts still comes back to its owner.
//
// Latency is set by the CORDIC, ~31 cycles; the logs finish long before and
// wait. Against roughly a hundred cycles of sweeps per merge, and with
// jc_ctrl able to run MARK concurrently, that is affordable.
//
// THE mT2 CHAIN IS ONE ARITHMETIC STEP PER STATE, and that is a timing fix,
// not a structural preference. Doing the whole of it in a single cycle -- the
// two 49-bit sums off pz, the 49x49 product they feed, the parallel px^2+py^2,
// then a 98-bit compare and mux -- measured 5.80 ns over 26 logic levels and
// held the entire plugin to 172 MHz against a 250 MHz requirement. It is the
// cheapest possible thing to fix, because jc_ctrl waits on the sk_done
// HANDSHAKE rather than counting cycles: the three extra states cost ~3 cycles
// per merge, some 380 in an event of 16,000, and no change anywhere else.
//
// *************************************************************************
`include "jc_defs.vh"
`timescale 1ns/1ps
module jc_setkin (
  input                                 start,
  input  signed [`JC_P4_W-1:0]          in_e,
  input  signed [`JC_P4_W-1:0]          in_px,
  input  signed [`JC_P4_W-1:0]          in_py,
  input  signed [`JC_P4_W-1:0]          in_pz,

  output logic                          busy,
  output logic                          done,          // one-cycle pulse
  output logic signed [`JC_COORD_W-1:0] out_y,
  output logic        [`JC_COORD_W-1:0] out_phi,
  output logic signed [`JC_WGT_W-1:0]   out_wgt,

  // ---- Shared log2, borrowed by jc_ctrl while this unit is idle ---------
  input                                 ext_log_valid,
  input  [`JC_LOG2_IN_W-1:0]            ext_log_x,
  output                                ext_log_ready,
  output                                ext_log_rsp_valid,
  output [`JC_LOG2_OUT_W-1:0]           ext_log_rsp,

  input                                 aclk,
  input                                 aresetn
);

  localparam int P4_W     = `JC_P4_W;
  localparam int P4_FRAC  = `JC_P4_FRAC;
  localparam int COORD_W  = `JC_COORD_W;
  localparam int WGT_W    = `JC_WGT_W;
  localparam int WGT_FRAC = `JC_WGT_FRAC;
  localparam int LOG_W    = `JC_LOG2_OUT_W;
  localparam int LOG_FRAC = `JC_LOG2_OUT_FRAC;
  localparam int LOG_IN_W = `JC_LOG2_IN_W;

  localparam int PROD_W  = 2 * P4_W + 2;         // 98
  localparam int ACC_W   = LOG_W + 3;            // 42, room for -2*log2 term
  localparam int LOG_LAT = 6;                    // jc_log2 pipeline depth

  // The one constant that turns a difference of base-2 logarithms into
  // binary-angle rapidity: y_units = d_log * ln2 * COORD_SCALE / 2^(LOG_FRAC+1),
  // with 2^JC_K_RAP_F folded in so the divide stays a shift.
  //
  // GENERATED, not written here. Computing it by hand gave a value 5.8e-5
  // high, which made every rapidity wrong by exactly that relative amount --
  // invisible to any test of the arithmetic, because the arithmetic was fine.
  localparam logic [31:0] K_RAP       = `JC_K_RAP;
  localparam int          K_RAP_SHIFT = LOG_FRAC + `JC_K_RAP_F + 1;   // 36
  localparam int          LOG_TO_WGT  = LOG_FRAC - WGT_FRAC;  // 7

  // 2*P4_FRAC in the log unit's Q7.32. Built by shifting a correctly sized
  // one: (1 << 32) evaluates to zero in Verilog's 32-bit integer arithmetic.
  localparam logic signed [ACC_W-1:0] ACC_ONE  = 1;
  localparam logic signed [ACC_W-1:0] WGT_BIAS = (ACC_ONE <<< LOG_FRAC)
                                                 * (2 * P4_FRAC);

  localparam logic [3:0] ST_IDLE  = 4'd0,
                         ST_MUL1  = 4'd1,
                         ST_MUL2  = 4'd2,
                         ST_MUL3  = 4'd3,
                         ST_MUL4  = 4'd4,
                         ST_ISSUE = 4'd5,
                         ST_WAIT  = 4'd6,
                         ST_RAP1  = 4'd7,
                         ST_RAP2  = 4'd8,
                         ST_RAP3  = 4'd9,
                         ST_FIN   = 4'd10;

  logic [3:0] state;
  logic [1:0] issue_cnt, rsp_cnt;

  logic signed [P4_W-1:0] e_r, px_r, py_r, pz_r;

  // The mT2 chain's stage registers, one per ST_MUL* state. prodm_r and
  // prod_r are two register levels on one product on purpose: it gives the
  // DSP cascade both an M and a P register to absorb, rather than only P.
  logic signed   [P4_W:0]    epz_r, emz_r;
  logic signed [2*P4_W-1:0]  px2_r, py2_r;
  logic signed [PROD_W-1:0]  prodm_r, prod_r;

  logic [PROD_W-1:0] pt_sq_r, mt2_r;
  logic   [P4_W:0]   eplus_r;
  logic              pz_pos_r, pt_zero_r;
  logic [LOG_W-1:0]  log_ptsq, log_mt2, log_eplus;
  logic [COORD_W-1:0] phi_r;
  logic              cordic_seen;

  // ---- Products ----------------------------------------------------------
  // Grouped by the state that registers each one. The arithmetic is unchanged
  // from the single-cycle version -- same widths, same signedness, same
  // truncation -- so the result is bit-identical; only the register boundaries
  // moved.

  // ST_MUL1: sums and squares, all straight off the ST_IDLE captures.
  wire signed   [P4_W:0]   e_plus_pz  = {e_r[P4_W-1], e_r} + {pz_r[P4_W-1], pz_r};
  wire signed   [P4_W:0]   e_minus_pz = {e_r[P4_W-1], e_r} - {pz_r[P4_W-1], pz_r};
  wire signed [2*P4_W-1:0] px2_c = px_r * px_r;
  wire signed [2*P4_W-1:0] py2_c = py_r * py_r;

  wire [P4_W-1:0] abs_pz  = pz_r[P4_W-1] ? -pz_r : pz_r;
  wire   [P4_W:0] eplus_c = {1'b0, e_r} + {1'b0, abs_pz};

  // ST_MUL2: the 49x49 product, its operands now registered.
  wire signed [PROD_W-1:0] prod_c = epz_r * emz_r;

  // ST_MUL3: the pt^2 sum.
  wire [PROD_W-1:0] ptsq_c = px2_r + py2_r;

  // ST_MUL4: the clamp. A negative product means a spacelike m2 from
  // rounding; falling back to pt_sq is FastJet's max(0.0, m2()).
  wire [PROD_W-1:0] prod_u = prod_r;
  wire [PROD_W-1:0] mt2_c  =
       (prod_r[PROD_W-1] || (prod_u < pt_sq_r)) ? pt_sq_r : prod_u;

  // ---- Shared log2 -------------------------------------------------------
  wire                 int_log_valid = (state == ST_ISSUE);
  logic [LOG_IN_W-1:0] int_log_x;

  always_comb begin
    case (issue_cnt)
      2'd0:    int_log_x = {{(LOG_IN_W-PROD_W){1'b0}}, pt_sq_r};
      2'd1:    int_log_x = {{(LOG_IN_W-PROD_W){1'b0}}, mt2_r};
      default: int_log_x = {{(LOG_IN_W-P4_W-1){1'b0}}, eplus_r};
    endcase
  end

  // Borrowing is only safe while this unit has none of its own in flight.
  assign ext_log_ready = (state == ST_IDLE) && !start;

  wire                 log_in_valid = int_log_valid
                                    | (ext_log_valid & ext_log_ready);
  wire [LOG_IN_W-1:0]  log_in_x     = int_log_valid ? int_log_x : ext_log_x;
  wire                 log_out_valid;
  wire [LOG_W-1:0]     log_out;

  jc_log2 u_log2 (
    .in_valid  (log_in_valid),
    .in_x      (log_in_x),
    .out_valid (log_out_valid),
    .out_zero  (),
    .out_log2  (log_out),
    .aclk      (aclk),
    .aresetn   (aresetn)
  );

  // Tag pipeline, matched to the log's latency: a response belongs to
  // whoever issued it, whatever has happened to the FSM since.
  logic [LOG_LAT-1:0] tag_mine;
  always_ff @(posedge aclk) begin
    if (!aresetn) tag_mine <= '0;
    else          tag_mine <= {tag_mine[LOG_LAT-2:0], int_log_valid};
  end
  wire rsp_mine = tag_mine[LOG_LAT-1];

  assign ext_log_rsp_valid = log_out_valid && !rsp_mine;
  assign ext_log_rsp       = log_out;

  // ---- CORDIC ------------------------------------------------------------
  wire               cordic_done;
  wire [COORD_W-1:0] cordic_phi;

  jc_cordic u_cordic (
    .start   (start && (state == ST_IDLE)),
    .in_px   (in_px),
    .in_py   (in_py),
    .busy    (),
    .done    (cordic_done),
    .out_phi (cordic_phi),
    .aclk    (aclk),
    .aresetn (aresetn)
  );

  // ---- Rapidity and weight ----------------------------------------------
  // d = log2(mT2) - 2*log2(E+|pz|), which is <= 0 by construction because
  // mT2 <= (E+|pz|)^2. Rapidity is K_RAP * d, sign taken from pz.
  //
  // Split ST_RAP1/2/3, and unlike the mT2 chain this one is FREE rather than
  // merely cheap. The CORDIC needs ~31 cycles and the logs are all in by
  // about 14, so ST_FIN was already sitting idle for the rest; three states
  // move into that gap and the merge takes exactly as long as it did. Doing
  // the subtract, the 42x32 multiply, the negate and the mux in one cycle
  // measured 4.75 ns over 22 logic levels -- the path that replaced mT2's as
  // the module's worst.
  wire signed [ACC_W-1:0] d_log =
       $signed({{(ACC_W-LOG_W){1'b0}}, log_mt2})
     - ($signed({{(ACC_W-LOG_W){1'b0}}, log_eplus}) <<< 1);

  logic signed [ACC_W-1:0]   d_log_r;
  logic signed [ACC_W+32:0]  y_scaled_r;
  logic signed [COORD_W-1:0] y_val_r;
  logic signed [WGT_W-1:0]   wgt_val_r;

  wire signed [ACC_W+32:0]  y_scaled = d_log_r * $signed({1'b0, K_RAP});
  wire signed [COORD_W-1:0] y_mag = y_scaled_r[K_RAP_SHIFT + COORD_W - 1 -: COORD_W];

  wire signed [ACC_W-1:0] wgt_q32 =
       WGT_BIAS - $signed({{(ACC_W-LOG_W){1'b0}}, log_ptsq});
  wire signed [ACC_W-1:0] wgt_rnd = wgt_q32 + (ACC_ONE <<< (LOG_TO_WGT - 1));

  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      state       <= ST_IDLE;
      busy        <= 1'b0;
      done        <= 1'b0;
      cordic_seen <= 1'b0;
    end
    else begin
      done <= 1'b0;

      // Capture the angle whenever it lands -- the CORDIC can finish before
      // the FSM reaches ST_FIN, and a result only latched there would be lost.
      if (cordic_done) begin
        phi_r       <= cordic_phi;
        cordic_seen <= 1'b1;
      end

      case (state)
        ST_IDLE: begin
          if (start) begin
            e_r  <= in_e;   px_r <= in_px;
            py_r <= in_py;  pz_r <= in_pz;
            busy        <= 1'b1;
            cordic_seen <= 1'b0;
            state       <= ST_MUL1;
          end
        end

        ST_MUL1: begin
          epz_r    <= e_plus_pz;
          emz_r    <= e_minus_pz;
          px2_r    <= px2_c;
          py2_r    <= py2_c;
          eplus_r  <= eplus_c;
          pz_pos_r <= (pz_r > 0);
          state    <= ST_MUL2;
        end

        ST_MUL2: begin
          prodm_r <= prod_c;
          state   <= ST_MUL3;
        end

        ST_MUL3: begin
          prod_r  <= prodm_r;
          pt_sq_r <= ptsq_c;
          state   <= ST_MUL4;
        end

        ST_MUL4: begin
          mt2_r     <= mt2_c;
          pt_zero_r <= (pt_sq_r == {PROD_W{1'b0}});
          issue_cnt <= '0;
          rsp_cnt   <= '0;
          state     <= ST_ISSUE;
        end

        ST_ISSUE: begin
          issue_cnt <= issue_cnt + 1'b1;
          if (issue_cnt == 2'd2) state <= ST_WAIT;
        end

        ST_WAIT: begin
          if (log_out_valid && rsp_mine) begin
            case (rsp_cnt)
              2'd0:    log_ptsq  <= log_out;
              2'd1:    log_mt2   <= log_out;
              default: log_eplus <= log_out;
            endcase
            rsp_cnt <= rsp_cnt + 1'b1;
            if (rsp_cnt == 2'd2) state <= ST_RAP1;
          end
        end

        // log_eplus lands on the cycle ST_WAIT exits, so d_log is ready here.
        ST_RAP1: begin
          d_log_r   <= d_log;
          // pt = 0 makes 1/pt^2 unbounded; saturate rather than wrap, so such
          // a row loses every argmin instead of winning them all.
          wgt_val_r <= pt_zero_r ? {1'b0, {(WGT_W-1){1'b1}}}
                                 : wgt_rnd[LOG_TO_WGT +: WGT_W];
          state     <= ST_RAP2;
        end

        ST_RAP2: begin
          y_scaled_r <= y_scaled;
          state      <= ST_RAP3;
        end

        ST_RAP3: begin
          y_val_r <= pt_zero_r ? '0 : (pz_pos_r ? -y_mag : y_mag);
          state   <= ST_FIN;
        end

        ST_FIN: begin
          // Logs and rapidity are long done; only the CORDIC can still run.
          if (cordic_seen) begin
            out_phi <= phi_r;
            out_y   <= y_val_r;
            out_wgt <= wgt_val_r;
            busy    <= 1'b0;
            done    <= 1'b1;
            state   <= ST_IDLE;
          end
        end

        default: state <= ST_IDLE;
      endcase
    end
  end

endmodule: jc_setkin
