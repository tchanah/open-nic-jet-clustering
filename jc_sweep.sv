// *************************************************************************
//
// jc_sweep -- one streaming pass over the active list, three modes.
//
// The engine's only wide datapath. LANES copies of jc_dist plus a reduction
// tree, walking the banked active list one offset per cycle: DEPTH cycles of
// work behind a fixed drain, which is the whole cost model of the design.
//
//   NN_SCAN   geometry. Find the query row's nearest active neighbour, and
//             let the query CLAIM rows it is now nearer to than their cached
//             best. That write-back is what keeps the nn table honest after
//             a merge moves the survivor -- see CLAUDE.md.
//   ARGMIN    no geometry. Reduce nn_dist_log over active rows to find the
//             globally closest pair, which is the round's merge decision.
//   MARK      no geometry, no reduction. Flag active rows whose cached
//             neighbour was either of the two rows just merged.
//
// All three share the sequencing, the delay line and the tree; only NN_SCAN
// lights up the lanes. That is why they are one module rather than three.
//
// NO LOGARITHM LIVES HERE. Claiming a row means storing w_k + log2(g), and
// doing that per lane would cost roughly 16 DSPs and thousands of LUTs per
// engine for something used a handful of times per merge. Instead the
// write-back stores nn_geo linearly -- monotonicity makes the "is this
// closer" test a plain integer compare -- and reports the claimed rows in
// claimed_mask. One shared log unit refreshes them afterwards.
//
// Ties break to the SMALLEST INDEX, in the lane tree (<= keeps the left,
// lower-numbered lane) and across offsets (strict < keeps the earlier one).
// This is load-bearing, not cosmetic: 2-6% of row scans on a regular grid
// end in a bit-exact tie, and jc_model.py fixes the same convention.
//
// Timing, from the cycle rd_off is presented:
//   +1  jc_mem registered read
//   +4  jc_dist result (3 stages)
//   +5  lane keys registered
//   +6  first half of the reduction tree
//   +7  second half
//   +8  accumulated into the running minimum
// so a sweep is DEPTH + 8 cycles: 8 of work behind 8 of drain at LANES=16.
//
// The tree is three registers deep because out-of-context synthesis said so.
// One cycle for the whole thing measured 29 logic levels and 5.6 ns against
// a 4 ns budget -- WNS -1.625, Fmax 178 MHz. Splitting it costs two cycles of
// drain, which is around 2k cycles/event against a budget in the tens of
// thousands, and is still nowhere near the ~50 the HLS port paid.
//
// *************************************************************************
`include "jc_defs.vh"
`timescale 1ns/1ps
module jc_sweep (
  // ---- Control ---------------------------------------------------------
  input                            start,
  input                      [1:0] mode,
  input        [`JC_IDX_W-1:0]     query_idx,     // NN_SCAN: the row scanned
  input signed [`JC_COORD_W-1:0]   query_y,
  input        [`JC_COORD_W-1:0]   query_phi,
  input        [`JC_IDX_W-1:0]     mark_a,        // MARK: the merged pair
  input        [`JC_IDX_W-1:0]     mark_b,
  input        [`JC_GEO_W-1:0]     r_squared,     // R^2 in delta units
  output logic                     busy,
  output logic                     done,          // one-cycle pulse

  // ---- Results, valid with done ----------------------------------------
  output logic                     result_valid,  // any candidate at all
  output logic [`JC_IDX_W-1:0]     result_index,
  output logic [`JC_GEO_W-1:0]     result_geo,    // NN_SCAN
  output logic                     result_beam,   // NN_SCAN: none inside R
  output logic signed [`JC_NNLOG_W-1:0] result_log,  // ARGMIN
  output logic [`JC_NMAX-1:0]      claimed_mask,  // NN_SCAN
  output logic [`JC_NMAX-1:0]      stale_mask,    // MARK

  // ---- jc_mem ----------------------------------------------------------
  output logic [`JC_OFF_W-1:0]           mem_rd_off,
  input  [`JC_LANES*`JC_COORD_W-1:0]     mem_rd_y,
  input  [`JC_LANES*`JC_COORD_W-1:0]     mem_rd_phi,
  input  [`JC_LANES-1:0]                 mem_rd_active,
  input  [`JC_LANES*`JC_IDX_W-1:0]       mem_rd_nn_index,
  input  [`JC_LANES*`JC_GEO_W-1:0]       mem_rd_nn_geo,
  input  [`JC_LANES*`JC_NNLOG_W-1:0]     mem_rd_nn_log,

  output logic [`JC_OFF_W-1:0]           mem_wb_off,
  output logic [`JC_LANES-1:0]           mem_wb_en,
  output logic [`JC_LANES*`JC_IDX_W-1:0] mem_wb_nn_index,
  output logic [`JC_LANES*`JC_GEO_W-1:0] mem_wb_nn_geo,

  input                                  aclk,
  input                                  aresetn
);

  localparam int LANES   = `JC_LANES;
  localparam int DEPTH   = `JC_DEPTH;
  localparam int IDX_W   = `JC_IDX_W;
  localparam int LANE_W  = `JC_LANE_W;
  localparam int OFF_W   = `JC_OFF_W;
  localparam int COORD_W = `JC_COORD_W;
  localparam int GEO_W   = `JC_GEO_W;
  localparam int NNLOG_W = `JC_NNLOG_W;

  localparam logic [1:0] MODE_NN_SCAN = 2'd0,
                         MODE_ARGMIN  = 2'd1,
                         MODE_MARK    = 2'd2;

  // One key width for both reductions, so the tree is built once. Smaller
  // always means better; invalid lanes are forced to all ones so they lose.
  localparam int KEY_W = GEO_W + 1;

  localparam int LAT_DIST = 3;              // jc_dist pipeline depth
  localparam int TOTAL    = DEPTH + 8;      // read..accumulate, see header

  // ---- Sequencing ------------------------------------------------------
  localparam int CYC_W = 5;
  logic [CYC_W-1:0] cyc;
  logic             running;

  wire read_phase = running && (cyc < DEPTH);

  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      running <= 1'b0;
      cyc     <= '0;
      done    <= 1'b0;
    end
    else begin
      done <= 1'b0;
      if (!running) begin
        if (start) begin
          running <= 1'b1;
          cyc     <= '0;
        end
      end
      else if (cyc == TOTAL - 1) begin
        running <= 1'b0;
        done    <= 1'b1;
      end
      else begin
        cyc <= cyc + 1'b1;
      end
    end
  end

  assign busy       = running;
  assign mem_rd_off = cyc[OFF_W-1:0];

  // ---- Stage A: jc_mem output, one cycle after rd_off ------------------
  logic             a_valid;
  logic [OFF_W-1:0] a_off;
  always_ff @(posedge aclk) begin
    if (!aresetn) a_valid <= 1'b0;
    else begin
      a_valid <= read_phase;
      a_off   <= cyc[OFF_W-1:0];
    end
  end

  // ---- Lanes -----------------------------------------------------------
  logic [LANES*GEO_W-1:0] lane_geo;

  genvar l, lv, n;
  generate
    for (l = 0; l < LANES; l = l + 1) begin : g_dist
      jc_dist u_dist (
        .y_q         (query_y),
        .phi_q       (query_phi),
        .y_k         (mem_rd_y  [l*COORD_W +: COORD_W]),
        .phi_k       (mem_rd_phi[l*COORD_W +: COORD_W]),
        .geo_dist_sq (lane_geo  [l*GEO_W   +: GEO_W]),
        .en          (1'b1),
        .aclk        (aclk),
        .aresetn     (aresetn)
      );
    end
  endgenerate

  // ---- Metadata delay, aligned to the lane results ---------------------
  // Everything the claim test and the reductions need, carried alongside the
  // geometry rather than re-read, so jc_mem needs only one read port.
  localparam int META_W = 1 + OFF_W + LANES*(1 + IDX_W + GEO_W + NNLOG_W);

  wire [META_W-1:0] meta_in = {a_valid, a_off, mem_rd_active,
                               mem_rd_nn_index, mem_rd_nn_geo, mem_rd_nn_log};

  // ONE PACKED VECTOR, NOT AN UNPACKED ARRAY, and that is not a style choice.
  // As `logic [META_W-1:0] meta_dly [0:LAT_DIST-1]` with
  // `wire m = meta_dly[LAT_DIST-1]`, Icarus does not keep the continuous
  // assignment in step with the array: every stage of the array read back
  // correctly while m sat at its time-zero value, all 1556 bits X. b_valid
  // is inside that word, so key_valid_r, mid_valid and t_valid all went X,
  // acc_valid never latched (an `if (X)` is false), and ARGMIN reported no
  // candidate on a list of 35 active rows -- the engine finished every event
  // without emitting a single jet.
  //
  // It survived this module's own bench and only appeared once jc_sweep was
  // instantiated inside the full chain, so it is elaboration-context
  // sensitive. A part-select of a packed vector has no such ambiguity in any
  // tool, and synthesises to exactly the same flops.
  logic [LAT_DIST*META_W-1:0] meta_dly;
  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      // Clear outright: the valid bit lives inside this word, so leaving it
      // undriven would let X propagate into the claim test for three cycles.
      meta_dly <= '0;
    end
    else begin
      meta_dly <= {meta_dly[(LAT_DIST-1)*META_W-1:0], meta_in};
    end
  end

  wire [META_W-1:0] m = meta_dly[(LAT_DIST-1)*META_W +: META_W];

  localparam int M_LOG = 0;
  localparam int M_GEO = M_LOG + LANES*NNLOG_W;
  localparam int M_IDX = M_GEO + LANES*GEO_W;
  localparam int M_ACT = M_IDX + LANES*IDX_W;
  localparam int M_OFF = M_ACT + LANES;
  localparam int M_VLD = M_OFF + OFF_W;

  wire             b_valid = m[M_VLD];
  wire [OFF_W-1:0] b_off   = m[M_OFF +: OFF_W];

  // ---- Keys and the claim test -----------------------------------------
  logic [LANES-1:0]       lane_claim;
  logic [LANES-1:0]       lane_stale;
  logic [LANES*KEY_W-1:0] lane_key;

  generate
    for (l = 0; l < LANES; l = l + 1) begin : g_key
      // A genvar cannot be part-selected, so widen it through a localparam.
      localparam logic [LANE_W-1:0] LID = l;

      wire             act    = m[M_ACT + l];
      wire [IDX_W-1:0] gidx   = {b_off, LID};
      wire             is_self = (gidx == query_idx);
      wire             usable = b_valid && act && !is_self;

      wire [GEO_W-1:0]   geo    = lane_geo[l*GEO_W +: GEO_W];
      wire [GEO_W-1:0]   nn_geo = m[M_GEO + l*GEO_W +: GEO_W];
      wire [IDX_W-1:0]   nn_idx = m[M_IDX + l*IDX_W +: IDX_W];
      wire [NNLOG_W-1:0] nn_log = m[M_LOG + l*NNLOG_W +: NNLOG_W];

      // ARGMIN ranks a signed log; flipping the sign bit makes unsigned
      // order agree with signed order, so one tree serves both modes.
      wire [KEY_W-1:0] key_geo = {1'b0, geo};
      wire [KEY_W-1:0] key_log =
           {{(KEY_W-NNLOG_W){1'b0}}, ~nn_log[NNLOG_W-1], nn_log[NNLOG_W-2:0]};

      // ARGMIN must consider every active row, including beam rows; NN_SCAN
      // must skip the query itself.
      wire lane_ok = (mode == MODE_ARGMIN) ? (b_valid && act) : usable;

      assign lane_key[l*KEY_W +: KEY_W] =
             !lane_ok       ? {KEY_W{1'b1}} :
             (mode == MODE_ARGMIN) ? key_log : key_geo;

      // Claim k only if strictly inside R. Clamping to R^2 first would let a
      // query BEYOND R capture k, and k would then merge on its turn instead
      // of being emitted as a jet -- a real bug jc_model.py caught.
      assign lane_claim[l] = (mode == MODE_NN_SCAN) && usable
                          && (geo < r_squared)
                          && ((geo < nn_geo)
                              || ((geo == nn_geo) && (query_idx < nn_idx)));

      assign lane_stale[l] = (mode == MODE_MARK) && b_valid && act
                          && ((nn_idx == mark_a) || (nn_idx == mark_b));

      assign mem_wb_nn_index[l*IDX_W +: IDX_W] = query_idx;
      assign mem_wb_nn_geo  [l*GEO_W +: GEO_W] = geo;
    end
  endgenerate

  assign mem_wb_off = b_off;
  assign mem_wb_en  = lane_claim;

  // ---- Reduction tree, smallest index wins -----------------------------
  // Split across three registers, not one. Out-of-context synthesis put the
  // whole LANE_W-level tree at 29 logic levels and 5.6 ns -- sixteen CARRY8
  // in series, because every 50-bit compare is a ~7-block carry chain and
  // four levels stack them. No stage below carries more than two levels.
  localparam int LVL   = LANE_W;
  localparam int LVL_A = LVL / 2;             // levels in the first half
  localparam int LVL_B = LVL - LVL_A;
  localparam int MID   = LANES >> LVL_A;      // survivors between the halves

  // Stage 1: the keys themselves, off the metadata delay line.
  logic [LANES*KEY_W-1:0] key_r;
  logic     [OFF_W-1:0]   key_off_r;
  logic                   key_valid_r;

  always_ff @(posedge aclk) begin
    if (!aresetn) key_valid_r <= 1'b0;
    else begin
      key_r       <= lane_key;
      key_off_r   <= b_off;
      key_valid_r <= b_valid;
    end
  end

  logic [KEY_W-1:0]  a_key [0:LVL_A][0:LANES-1];
  logic [LANE_W-1:0] a_idx [0:LVL_A][0:LANES-1];

  generate
    for (l = 0; l < LANES; l = l + 1) begin : g_leaf
      localparam logic [LANE_W-1:0] LID = l;
      assign a_key[0][l] = key_r[l*KEY_W +: KEY_W];
      assign a_idx[0][l] = LID;
    end
    for (lv = 0; lv < LVL_A; lv = lv + 1) begin : g_lvl_a
      for (n = 0; n < (LANES >> (lv+1)); n = n + 1) begin : g_node
        // <= keeps the left child, which carries the lower lane index.
        wire take_left = a_key[lv][2*n] <= a_key[lv][2*n+1];
        assign a_key[lv+1][n] = take_left ? a_key[lv][2*n] : a_key[lv][2*n+1];
        assign a_idx[lv+1][n] = take_left ? a_idx[lv][2*n] : a_idx[lv][2*n+1];
      end
    end
  endgenerate

  // Stage 2: the half-tree survivors.
  logic [MID*KEY_W-1:0]  mid_key;
  logic [MID*LANE_W-1:0] mid_idx;
  logic    [OFF_W-1:0]   mid_off;
  logic                  mid_valid;

  generate
    for (n = 0; n < MID; n = n + 1) begin : g_mid
      always_ff @(posedge aclk) begin
        mid_key[n*KEY_W  +: KEY_W]  <= a_key[LVL_A][n];
        mid_idx[n*LANE_W +: LANE_W] <= a_idx[LVL_A][n];
      end
    end
  endgenerate

  always_ff @(posedge aclk) begin
    if (!aresetn) mid_valid <= 1'b0;
    else begin
      mid_valid <= key_valid_r;
      mid_off   <= key_off_r;
    end
  end

  logic [KEY_W-1:0]  b_key [0:LVL_B][0:MID-1];
  logic [LANE_W-1:0] b_idx [0:LVL_B][0:MID-1];

  generate
    for (n = 0; n < MID; n = n + 1) begin : g_mid_leaf
      assign b_key[0][n] = mid_key[n*KEY_W  +: KEY_W];
      assign b_idx[0][n] = mid_idx[n*LANE_W +: LANE_W];
    end
    for (lv = 0; lv < LVL_B; lv = lv + 1) begin : g_lvl_b
      for (n = 0; n < (MID >> (lv+1)); n = n + 1) begin : g_node
        wire take_left = b_key[lv][2*n] <= b_key[lv][2*n+1];
        assign b_key[lv+1][n] = take_left ? b_key[lv][2*n] : b_key[lv][2*n+1];
        assign b_idx[lv+1][n] = take_left ? b_idx[lv][2*n] : b_idx[lv][2*n+1];
      end
    end
  endgenerate

  // ---- Stage 3: registered tree output ---------------------------------
  logic [KEY_W-1:0]  t_key;
  logic [IDX_W-1:0]  t_idx;
  logic              t_valid;

  always_ff @(posedge aclk) begin
    if (!aresetn) t_valid <= 1'b0;
    else begin
      t_key   <= b_key[LVL_B][0];
      t_idx   <= {mid_off, b_idx[LVL_B][0]};
      t_valid <= mid_valid && (b_key[LVL_B][0] != {KEY_W{1'b1}});
    end
  end

  // ---- Running minimum across offsets ----------------------------------
  logic [KEY_W-1:0] acc_key;
  logic [IDX_W-1:0] acc_idx;
  logic             acc_valid;

  always_ff @(posedge aclk) begin
    if (!aresetn || start) begin
      acc_valid <= 1'b0;
      acc_key   <= {KEY_W{1'b1}};
      acc_idx   <= '0;
    end
    // Strict <, so the earliest offset survives a tie and the winning global
    // index is the smallest.
    else if (t_valid && (!acc_valid || t_key < acc_key)) begin
      acc_valid <= 1'b1;
      acc_key   <= t_key;
      acc_idx   <= t_idx;
    end
  end

  // ---- Bitmaps ---------------------------------------------------------
  // Lane l at offset b_off IS global index b_off*LANES + l, so the whole lane
  // vector drops into place with a single shift. Doing it per lane would put
  // sixteen always blocks on one variable, which is not a legal driver set.
  wire [`JC_NMAX-1:0] claim_set =
       {{(`JC_NMAX-LANES){1'b0}}, lane_claim} << (b_off * LANES);
  wire [`JC_NMAX-1:0] stale_set =
       {{(`JC_NMAX-LANES){1'b0}}, lane_stale} << (b_off * LANES);

  always_ff @(posedge aclk) begin
    if (!aresetn || start) begin
      claimed_mask <= '0;
      stale_mask   <= '0;
    end
    else begin
      claimed_mask <= claimed_mask | claim_set;
      stale_mask   <= stale_mask   | stale_set;
    end
  end

  // ---- Result decode ---------------------------------------------------
  assign result_valid = acc_valid;
  assign result_index = acc_idx;
  assign result_geo   = acc_key[GEO_W-1:0];
  // Nothing inside R means this row's own best is the beam.
  assign result_beam  = !acc_valid || (acc_key[GEO_W-1:0] >= r_squared);
  assign result_log   = {~acc_key[NNLOG_W-1], acc_key[NNLOG_W-2:0]};

endmodule: jc_sweep
