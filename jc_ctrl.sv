// *************************************************************************
//
// jc_ctrl -- the round FSM. Sequences one event through to its jets.
//
// A SEQUENCER OVER PIPELINES, not a substitute for them. Each sweep is
// already an II=1 pass over the active list; this only chooses which one
// runs. Rounds cannot be pipelined against each other because round k's
// ARGMIN reads nn_dist_log that round k-1's rescans just rewrote, and which
// pair merges determines the whole next active set. That is a true serial
// dependency, so cross-event parallelism is engine replication.
//
//   LOAD    pull the event from jc_evbuf into jc_mem
//   SETUP   NN_SCAN every row, then one log each, to seed the nn table
//   ROUND   until nothing is active:
//             ARGMIN                the globally closest pair
//             beam?  -> emit the jet, deactivate, repair its pointers
//             else   -> merge, then repair the nn table
//
// Repairing after a merge has THREE obligations, and missing the third
// silently reorders later merges:
//
//   1. the survivor moved            -> NN_SCAN it
//   2. rows that pointed at either   -> rescan in full; being near the
//      merged row lost their answer     survivor does not make it their
//                                       NEAREST neighbour
//   3. rows the moved survivor is    -> its own scan's write-back claims
//      now nearer to                    them, reported in claimed_mask
//
// Both 1 and 3 leave nn_dist_log stale, because the write-back stores only
// the linear nn_geo -- a logarithm in sixteen lanes was never affordable.
// jc_ctrl walks the affected rows through the single log unit it borrows
// from jc_setkin, which is what makes that trade work.
//
// EVERY MERGE IS AN EXACT INTEGER ADD. Four-momenta are Q14.34, so a jet is
// independent of the order its constituents were combined in: only WHICH
// cells group together can differ.
//
// TIMING NOTE THAT SHAPES THE STATE COUNT. jc_mem's lane read is registered,
// so an index presented in one cycle yields its fields in the next. Every
// place that inspects a row therefore takes two states -- one to address it,
// one to use it. Collapsing them reads the previous row's data, which is the
// kind of bug that produces plausible wrong jets rather than a crash.
//
// *************************************************************************
`include "jc_defs.vh"
`timescale 1ns/1ps
module jc_ctrl (
  // ---- Event source: jc_evbuf ------------------------------------------
  input                                 ev_valid,
  input  [`JC_CNT_W-1:0]                ev_count,
  input                          [31:0] ev_seq,
  output logic                          ev_accept,
  output logic [`JC_IDX_W-1:0]          ev_addr,
  input  signed [`JC_P4_W-1:0]          ev_energy,
  input  signed [`JC_P4_W-1:0]          ev_px,
  input  signed [`JC_P4_W-1:0]          ev_py,
  input  signed [`JC_P4_W-1:0]          ev_pz,
  input  signed [`JC_COORD_W-1:0]       ev_rapidity,
  input         [`JC_COORD_W-1:0]       ev_phi,
  input  signed [`JC_WGT_W-1:0]         ev_beam_weight_log,
  output logic                          ev_release,

  // ---- Configuration ----------------------------------------------------
  input  [`JC_GEO_W-1:0]                cfg_r_squared,
  input  [2*`JC_P4_W-1:0]               cfg_pt_sq_floor,   // Q28.68

  // ---- jc_mem: writes ---------------------------------------------------
  output logic                          mem_init_en,
  output logic [`JC_IDX_W-1:0]          mem_init_idx,
  output logic signed [`JC_P4_W-1:0]    mem_init_e, mem_init_px,
                                        mem_init_py, mem_init_pz,
  output logic signed [`JC_COORD_W-1:0] mem_init_y,
  output logic        [`JC_COORD_W-1:0] mem_init_phi,
  output logic signed [`JC_WGT_W-1:0]   mem_init_wgt,

  output logic                          mem_set_en,
  output logic [`JC_IDX_W-1:0]          mem_set_idx,
  output logic signed [`JC_COORD_W-1:0] mem_set_y,
  output logic        [`JC_COORD_W-1:0] mem_set_phi,
  output logic signed [`JC_WGT_W-1:0]   mem_set_wgt,

  output logic [`JC_IDX_W-1:0]          mem_p4_rd_idx,
  input  signed [`JC_P4_W-1:0]          mem_p4_rd_e, mem_p4_rd_px,
                                        mem_p4_rd_py, mem_p4_rd_pz,
  output logic                          mem_p4_wr_en,
  output logic [`JC_IDX_W-1:0]          mem_p4_wr_idx,
  output logic signed [`JC_P4_W-1:0]    mem_p4_wr_e, mem_p4_wr_px,
                                        mem_p4_wr_py, mem_p4_wr_pz,

  output logic                          mem_kill_en,
  output logic [`JC_IDX_W-1:0]          mem_kill_idx,

  output logic                          mem_nn_wr_en,
  output logic [`JC_IDX_W-1:0]          mem_nn_wr_idx,
  output logic                          mem_nn_wr_beam,
  output logic [`JC_IDX_W-1:0]          mem_nn_wr_index,
  output logic [`JC_GEO_W-1:0]          mem_nn_wr_geo,
  output logic signed [`JC_NNLOG_W-1:0] mem_nn_wr_log,

  output logic                          mem_log_wr_en,
  output logic [`JC_IDX_W-1:0]          mem_log_wr_idx,
  output logic signed [`JC_NNLOG_W-1:0] mem_log_wr_val,

  // ---- jc_mem: single-row inspection ------------------------------------
  // jc_engine hands ctrl the read port whenever the sweep is idle; ctrl
  // presents an offset and selects the lane itself.
  output logic [`JC_OFF_W-1:0]          mem_rd_off,
  input  [`JC_LANES*`JC_COORD_W-1:0]    mem_rd_y,
  input  [`JC_LANES*`JC_COORD_W-1:0]    mem_rd_phi,
  input  [`JC_LANES*`JC_WGT_W-1:0]      mem_rd_wgt,
  input  [`JC_LANES-1:0]                mem_rd_beam,
  input  [`JC_LANES*`JC_IDX_W-1:0]      mem_rd_nn_index,
  input  [`JC_LANES*`JC_GEO_W-1:0]      mem_rd_nn_geo,
  input  [`JC_NMAX-1:0]                 mem_active_mask,

  // ---- jc_sweep ---------------------------------------------------------
  output logic                          sw_start,
  output logic                    [1:0] sw_mode,
  output logic [`JC_IDX_W-1:0]          sw_query_idx,
  output logic signed [`JC_COORD_W-1:0] sw_query_y,
  output logic        [`JC_COORD_W-1:0] sw_query_phi,
  output logic [`JC_IDX_W-1:0]          sw_mark_a, sw_mark_b,
  input                                 sw_done,
  input                                 sw_result_valid,
  input  [`JC_IDX_W-1:0]                sw_result_index,
  input  [`JC_GEO_W-1:0]                sw_result_geo,
  input                                 sw_result_beam,
  input  [`JC_NMAX-1:0]                 sw_claimed_mask,
  input  [`JC_NMAX-1:0]                 sw_stale_mask,

  // ---- jc_setkin --------------------------------------------------------
  output logic                          sk_start,
  output logic signed [`JC_P4_W-1:0]    sk_e, sk_px, sk_py, sk_pz,
  input                                 sk_done,
  input  signed [`JC_COORD_W-1:0]       sk_y,
  input         [`JC_COORD_W-1:0]       sk_phi,
  input  signed [`JC_WGT_W-1:0]         sk_wgt,
  output logic                          sk_ext_log_valid,
  output logic [`JC_LOG2_IN_W-1:0]      sk_ext_log_x,
  input                                 sk_ext_log_ready,
  input                                 sk_ext_log_rsp_valid,
  input  [`JC_LOG2_OUT_W-1:0]           sk_ext_log_rsp,

  // ---- Jets out ---------------------------------------------------------
  output logic                          jet_valid,
  output logic signed [`JC_P4_W-1:0]    jet_e, jet_px, jet_py, jet_pz,
  output logic                   [31:0] jet_seq,
  output logic                          jet_eoe,      // end of event
  input                                 jet_ready,

  // ---- Status -----------------------------------------------------------
  output logic                          idle,
  output logic                   [31:0] event_count,
  output logic                   [31:0] cycle_count,  // of the event just done

  input                                 aclk,
  input                                 aresetn
);

  localparam int NMAX     = `JC_NMAX;
  localparam int IDX_W    = `JC_IDX_W;
  localparam int LANE_W   = `JC_LANE_W;
  localparam int COORD_W  = `JC_COORD_W;
  localparam int GEO_W    = `JC_GEO_W;
  localparam int WGT_W    = `JC_WGT_W;
  localparam int NNLOG_W  = `JC_NNLOG_W;
  localparam int P4_W     = `JC_P4_W;
  localparam int LOG_OUT_W= `JC_LOG2_OUT_W;
  localparam int LOG_IN_W = `JC_LOG2_IN_W;
  localparam int LOG_TO_NN= `JC_LOG2_OUT_FRAC - `JC_WGT_FRAC;   // 7

  localparam logic [1:0] MODE_NN_SCAN = 2'd0,
                         MODE_ARGMIN  = 2'd1,
                         MODE_MARK    = 2'd2;

  localparam logic [4:0]
      S_IDLE       = 5'd0,  S_LOAD_RD    = 5'd1,  S_LOAD      = 5'd2,
      S_SETUP_RD   = 5'd3,  S_SETUP_GO   = 5'd4,  S_SETUP_W   = 5'd5,
      S_ARGMIN     = 5'd6,  S_ARGMIN_W   = 5'd7,
      S_DECIDE_RD  = 5'd8,  S_DECIDE     = 5'd9,
      S_EMIT       = 5'd10, S_EMIT_MARK  = 5'd11, S_EMIT_MARK_W = 5'd12,
      S_MERGE_I    = 5'd13, S_MERGE_J    = 5'd14, S_MERGE_WR  = 5'd15,
      S_KIN        = 5'd16, S_KIN_W      = 5'd17,
      S_SCAN_I     = 5'd18, S_SCAN_I_W   = 5'd19,
      S_STALE_PICK = 5'd20, S_STALE_RD   = 5'd21, S_STALE_GO  = 5'd22,
      S_STALE_W    = 5'd23,
      S_REFRESH    = 5'd24, S_REFRESH_RD = 5'd25, S_REFRESH_GO = 5'd26,
      S_FINISH     = 5'd27;

  logic [4:0] state;

  logic [`JC_CNT_W-1:0] n_cells;
  logic          [31:0] seq_r;
  logic   [IDX_W-1:0]   load_idx, setup_idx;
  logic   [IDX_W-1:0]   surv, absorbed, scan_row;
  logic signed [P4_W-1:0] acc_e, acc_px, acc_py, acc_pz;
  logic                 in_setup;

  // Rows needing a full rescan, and rows needing only a fresh log.
  logic [NMAX-1:0] todo_scan, todo_log;

  // S_KIN launches MARK and jc_setkin together and must see BOTH finish; they
  // are independent units with unequal latency, so their done pulses are
  // latched rather than assumed to arrive in a fixed order. See S_KIN_W.
  logic kin_sw_done, kin_sk_done;

  // ---- Priority pick, lowest index first --------------------------------
  // Lowest-first keeps the repair order a function of the active set alone,
  // so a run is reproducible.
  logic [NMAX-1:0]  pick_mask;
  logic [IDX_W-1:0] pick_idx;
  logic             pick_valid;
  // p is an integer, so no part-select on it: Icarus rejects constant selects
  // inside always_* and the truncation to IDX_W happens on assignment anyway.
  integer p;
  always_comb begin
    pick_valid = 1'b0;
    pick_idx   = '0;
    for (p = NMAX-1; p >= 0; p = p - 1)
      if (pick_mask[p]) begin
        pick_valid = 1'b1;
        pick_idx   = p;
      end
  end

  always_comb begin
    case (state)
      S_STALE_PICK: pick_mask = todo_scan;
      S_REFRESH:    pick_mask = todo_log;
      default:      pick_mask = '0;
    endcase
  end

  // ---- Single-row field select ------------------------------------------
  // The read is registered: probe_idx set this cycle, probe_* valid next.
  logic [IDX_W-1:0] probe_idx;
  wire [LANE_W-1:0] probe_lane = probe_idx[LANE_W-1:0];
  assign mem_rd_off = probe_idx[IDX_W-1:LANE_W];

  wire signed [COORD_W-1:0] probe_y     = mem_rd_y  [probe_lane*COORD_W +: COORD_W];
  wire        [COORD_W-1:0] probe_phi   = mem_rd_phi[probe_lane*COORD_W +: COORD_W];
  wire signed   [WGT_W-1:0] probe_wgt   = mem_rd_wgt[probe_lane*WGT_W   +: WGT_W];
  wire                      probe_beam  = mem_rd_beam[probe_lane];
  wire        [IDX_W-1:0]   probe_nnidx = mem_rd_nn_index[probe_lane*IDX_W +: IDX_W];
  wire        [GEO_W-1:0]   probe_nngeo = mem_rd_nn_geo[probe_lane*GEO_W   +: GEO_W];

  // ---- nn_dist_log = weight + log2(distance) ----------------------------
  // The sweep leaves nn_geo linear and nn_dist_log stale; this is where the
  // two are reconciled, one row at a time through the borrowed log unit.
  // The log arrives Q7.32 and nn_dist_log is Q7.25, so it sheds LOG_TO_NN.
  logic  [IDX_W-1:0]      log_row;
  logic signed [WGT_W-1:0] log_wgt;
  logic                    log_pending;

  wire [LOG_OUT_W-LOG_TO_NN-1:0] log_q25 = sk_ext_log_rsp[LOG_OUT_W-1:LOG_TO_NN];
  wire signed [NNLOG_W-1:0] nn_log_result =
       $signed({{(NNLOG_W-WGT_W){log_wgt[WGT_W-1]}}, log_wgt})
     + $signed({{(NNLOG_W-(LOG_OUT_W-LOG_TO_NN)){1'b0}}, log_q25});

  // ---- Jet gate ---------------------------------------------------------
  // Two 48x48 multiplies, a 96-bit sum and a 96-bit compare against the floor.
  // The MULTIPLIES alone fill a cycle: S_DECIDE registers the two squares,
  // S_EMIT does the sum and the compare together. That last pair is two carry
  // chains in series and comfortable; the multiplies with the sum bolted on
  // measured 4.41 ns, and with the compare as well, 5.06 ns.
  //
  // No extra state, so no extra cycles: S_DECIDE already existed as the wait
  // for jc_mem's registered read, and the sum is free to share S_EMIT with a
  // compare that was going to happen there anyway.
  //
  // Splitting the sum from the multiply does not change a bit. Both squares
  // are non-negative and exact in 96 bits, and two's-complement addition is
  // identical whether the operands are read as signed or unsigned, so the
  // unsigned sum below is the same 96 bits the signed one produced.
  //
  // NONE OF THIS SHOWED UP IN jc_ctrl's OWN OUT-OF-CONTEXT RUN, which reported
  // +2.010 ns. syn/ooc.tcl writes a create_clock and no set_input_delay, so
  // a path starting at an input port is never timed -- and standalone,
  // mem_p4_rd_px IS a port. Inside jc_engine it is a register in jc_mem,
  // Vivado absorbs it into the DSP, and the path becomes reg-to-reg and real.
  // Per-module Fmax cannot see anything that crosses a module boundary; only
  // the jc_engine and jet_clustering rows can.
  wire signed [2*P4_W-1:0] jet_px_sq = mem_p4_rd_px * mem_p4_rd_px;
  wire signed [2*P4_W-1:0] jet_py_sq = mem_p4_rd_py * mem_p4_rd_py;
  logic [2*P4_W-1:0] jet_px_sq_r, jet_py_sq_r;

  wire [2*P4_W-1:0] jet_pt_sq = jet_px_sq_r + jet_py_sq_r;
  wire jet_above_floor = (jet_pt_sq >= cfg_pt_sq_floor);

  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      state            <= S_IDLE;
      idle             <= 1'b1;
      ev_accept        <= 1'b0;
      ev_release       <= 1'b0;
      mem_init_en      <= 1'b0;
      mem_set_en       <= 1'b0;
      mem_p4_wr_en     <= 1'b0;
      mem_kill_en      <= 1'b0;
      mem_nn_wr_en     <= 1'b0;
      mem_log_wr_en    <= 1'b0;
      sw_start         <= 1'b0;
      sk_start         <= 1'b0;
      sk_ext_log_valid <= 1'b0;
      jet_valid        <= 1'b0;
      jet_eoe          <= 1'b0;
      event_count      <= 32'd0;
      cycle_count      <= 32'd0;
      log_pending      <= 1'b0;
      kin_sw_done      <= 1'b0;
      kin_sk_done      <= 1'b0;
      todo_scan        <= '0;
      todo_log         <= '0;
    end
    else begin
      // Single-cycle strobes; each state re-raises what it needs.
      ev_accept        <= 1'b0;
      ev_release       <= 1'b0;
      mem_init_en      <= 1'b0;
      mem_set_en       <= 1'b0;
      mem_p4_wr_en     <= 1'b0;
      mem_kill_en      <= 1'b0;
      mem_nn_wr_en     <= 1'b0;
      mem_log_wr_en    <= 1'b0;
      sw_start         <= 1'b0;
      sk_start         <= 1'b0;
      sk_ext_log_valid <= 1'b0;
      jet_eoe          <= 1'b0;

      if (jet_valid && jet_ready) jet_valid <= 1'b0;
      if (state != S_IDLE) cycle_count <= cycle_count + 32'd1;

      // The borrowed log's answer can land in any state; catch it once.
      if (log_pending && sk_ext_log_rsp_valid) begin
        mem_log_wr_en  <= 1'b1;
        mem_log_wr_idx <= log_row;
        mem_log_wr_val <= nn_log_result;
        log_pending    <= 1'b0;
      end

      case (state)

        // ---- Take an event -------------------------------------------
        S_IDLE: begin
          idle <= 1'b1;
          if (ev_valid) begin
            ev_accept   <= 1'b1;
            n_cells     <= ev_count;
            seq_r       <= ev_seq;
            load_idx    <= '0;
            ev_addr     <= '0;
            cycle_count <= 32'd0;
            todo_scan   <= '0;
            todo_log    <= '0;
            idle        <= 1'b0;
            state       <= S_LOAD_RD;
          end
        end

        // jc_evbuf's read port is registered too.
        S_LOAD_RD: state <= S_LOAD;

        S_LOAD: begin
          mem_init_en  <= 1'b1;
          mem_init_idx <= load_idx;
          mem_init_e   <= ev_energy;
          mem_init_px  <= ev_px;
          mem_init_py  <= ev_py;
          mem_init_pz  <= ev_pz;
          mem_init_y   <= ev_rapidity;
          mem_init_phi <= ev_phi;
          mem_init_wgt <= ev_beam_weight_log;

          if (load_idx + 1'b1 >= n_cells) begin
            ev_release <= 1'b1;
            setup_idx  <= '0;
            probe_idx  <= '0;
            in_setup   <= 1'b1;
            state      <= S_SETUP_RD;
          end
          else begin
            load_idx <= load_idx + 1'b1;
            ev_addr  <= load_idx + 1'b1;
            state    <= S_LOAD_RD;
          end
        end

        // ---- Seed every row's nearest neighbour ----------------------
        // Two states because the query's own coordinates come from memory.
        S_SETUP_RD: state <= S_SETUP_GO;

        S_SETUP_GO: begin
          sw_start     <= 1'b1;
          sw_mode      <= MODE_NN_SCAN;
          sw_query_idx <= setup_idx;
          sw_query_y   <= probe_y;
          sw_query_phi <= probe_phi;
          scan_row     <= setup_idx;
          state        <= S_SETUP_W;
        end

        S_SETUP_W: begin
          if (sw_done) begin
            mem_nn_wr_en    <= 1'b1;
            mem_nn_wr_idx   <= scan_row;
            mem_nn_wr_beam  <= sw_result_beam;
            mem_nn_wr_index <= sw_result_index;
            mem_nn_wr_geo   <= sw_result_beam ? cfg_r_squared : sw_result_geo;
            mem_nn_wr_log   <= '0;
            todo_log[scan_row] <= 1'b1;

            if (setup_idx + 1'b1 >= n_cells) state <= S_REFRESH;
            else begin
              setup_idx <= setup_idx + 1'b1;
              probe_idx <= setup_idx + 1'b1;
              state     <= S_SETUP_RD;
            end
          end
        end

        // ---- One round -----------------------------------------------
        S_ARGMIN: begin
          sw_start <= 1'b1;
          sw_mode  <= MODE_ARGMIN;
          state    <= S_ARGMIN_W;
        end

        S_ARGMIN_W: begin
          if (sw_done) begin
            if (!sw_result_valid) state <= S_FINISH;
            else begin
              surv      <= sw_result_index;
              probe_idx <= sw_result_index;
              // Address the four-momentum here, not in S_DECIDE: that read is
              // registered too, and both branches out of S_DECIDE consume it
              // immediately. Setting it one state later hands S_EMIT the
              // PREVIOUS row -- X on the first round, so the first jet of
              // every event simply vanished.
              mem_p4_rd_idx <= sw_result_index;
              state     <= S_DECIDE_RD;
            end
          end
        end

        S_DECIDE_RD: state <= S_DECIDE;

        // probe_* and the four-momentum are both settled by now, which is what
        // makes this the place to take the pt^2 products. Unconditional: the
        // merge path ignores the register, and guarding it would only add a
        // mux to the thing being made cheaper.
        S_DECIDE: begin
          absorbed    <= probe_nnidx;
          jet_px_sq_r <= jet_px_sq;
          jet_py_sq_r <= jet_py_sq;
          state       <= probe_beam ? S_EMIT : S_MERGE_I;
        end

        // ---- Beam: this row is a finished jet ------------------------
        S_EMIT: begin
          if (!jet_valid || jet_ready) begin
            // Below the floor it is still removed, just not reported.
            jet_valid <= jet_above_floor;
            jet_e     <= mem_p4_rd_e;
            jet_px    <= mem_p4_rd_px;
            jet_py    <= mem_p4_rd_py;
            jet_pz    <= mem_p4_rd_pz;
            jet_seq   <= seq_r;

            mem_kill_en  <= 1'b1;
            mem_kill_idx <= surv;
            state        <= S_EMIT_MARK;
          end
        end

        // Rows pointing at the departed jet have lost their answer. MARK
        // finds them the same way it does after a merge, with both targets
        // set to the one index.
        S_EMIT_MARK: begin
          sw_start  <= 1'b1;
          sw_mode   <= MODE_MARK;
          sw_mark_a <= surv;
          sw_mark_b <= surv;
          state     <= S_EMIT_MARK_W;
        end

        S_EMIT_MARK_W: begin
          if (sw_done) begin
            todo_scan <= sw_stale_mask;
            state     <= S_STALE_PICK;
          end
        end

        // ---- Merge: exact integer add --------------------------------
        S_MERGE_I: begin
          acc_e  <= mem_p4_rd_e;   acc_px <= mem_p4_rd_px;
          acc_py <= mem_p4_rd_py;  acc_pz <= mem_p4_rd_pz;
          mem_p4_rd_idx <= absorbed;
          state  <= S_MERGE_J;
        end

        S_MERGE_J: state <= S_MERGE_WR;

        S_MERGE_WR: begin
          mem_p4_wr_en  <= 1'b1;
          mem_p4_wr_idx <= surv;
          mem_p4_wr_e   <= acc_e  + mem_p4_rd_e;
          mem_p4_wr_px  <= acc_px + mem_p4_rd_px;
          mem_p4_wr_py  <= acc_py + mem_p4_rd_py;
          mem_p4_wr_pz  <= acc_pz + mem_p4_rd_pz;

          mem_kill_en   <= 1'b1;
          mem_kill_idx  <= absorbed;

          sk_e  <= acc_e  + mem_p4_rd_e;
          sk_px <= acc_px + mem_p4_rd_px;
          sk_py <= acc_py + mem_p4_rd_py;
          sk_pz <= acc_pz + mem_p4_rd_pz;
          state <= S_KIN;
        end

        // ---- Recompute coordinates, and mark the orphaned rows -------
        // Concurrent on purpose: MARK reads only nn_index and active, which
        // the merge has already settled, so it needs nothing from setkin.
        S_KIN: begin
          sk_start    <= 1'b1;
          sw_start    <= 1'b1;
          sw_mode     <= MODE_MARK;
          sw_mark_a   <= surv;
          sw_mark_b   <= absorbed;
          kin_sw_done <= 1'b0;
          kin_sk_done <= 1'b0;
          state       <= S_KIN_W;
        end

        // Leave only when BOTH have finished. MARK is DEPTH+8 = 16 cycles and
        // jc_setkin is CORDIC-bound at ~31, so sw_done has always landed
        // first -- but leaving on sk_done alone would make that ordering a
        // silent dependency of the control logic on two other modules'
        // latencies. If it ever inverted, S_SCAN_I would pulse sw_start into
        // a still-running sweep; jc_sweep ignores start while running, so the
        // scan would never happen and S_SCAN_I_W would write the MARK sweep's
        // results into the survivor's nn record. Wrong jets, no symptom.
        S_KIN_W: begin
          if (sw_done) begin
            kin_sw_done <= 1'b1;
            // The survivor gets a full scan of its own, so exclude it here.
            todo_scan <= sw_stale_mask & ~({{(NMAX-1){1'b0}}, 1'b1} << surv);
          end
          if (sk_done) begin
            kin_sk_done  <= 1'b1;
            mem_set_en   <= 1'b1;
            mem_set_idx  <= surv;
            mem_set_y    <= sk_y;
            mem_set_phi  <= sk_phi;
            mem_set_wgt  <= sk_wgt;
            sw_query_y   <= sk_y;
            sw_query_phi <= sk_phi;
          end
          if ((sw_done || kin_sw_done) && (sk_done || kin_sk_done))
            state <= S_SCAN_I;
        end

        // ---- The survivor's own scan, with the write-back ------------
        S_SCAN_I: begin
          sw_start     <= 1'b1;
          sw_mode      <= MODE_NN_SCAN;
          sw_query_idx <= surv;
          state        <= S_SCAN_I_W;
        end

        S_SCAN_I_W: begin
          if (sw_done) begin
            mem_nn_wr_en    <= 1'b1;
            mem_nn_wr_idx   <= surv;
            mem_nn_wr_beam  <= sw_result_beam;
            mem_nn_wr_index <= sw_result_index;
            mem_nn_wr_geo   <= sw_result_beam ? cfg_r_squared : sw_result_geo;
            mem_nn_wr_log   <= '0;
            // The survivor needs a log, and so does every row it claimed --
            // the write-back could not compute one.
            todo_log <= sw_claimed_mask | ({{(NMAX-1){1'b0}}, 1'b1} << surv);
            state    <= S_STALE_PICK;
          end
        end

        // ---- Rows whose cached neighbour is gone ---------------------
        S_STALE_PICK: begin
          if (!pick_valid) state <= S_REFRESH;
          else begin
            probe_idx <= pick_idx;
            scan_row  <= pick_idx;
            state     <= S_STALE_RD;
          end
        end

        S_STALE_RD: state <= S_STALE_GO;

        S_STALE_GO: begin
          sw_start     <= 1'b1;
          sw_mode      <= MODE_NN_SCAN;
          sw_query_idx <= scan_row;
          sw_query_y   <= probe_y;
          sw_query_phi <= probe_phi;
          state        <= S_STALE_W;
        end

        S_STALE_W: begin
          if (sw_done) begin
            mem_nn_wr_en    <= 1'b1;
            mem_nn_wr_idx   <= scan_row;
            mem_nn_wr_beam  <= sw_result_beam;
            mem_nn_wr_index <= sw_result_index;
            mem_nn_wr_geo   <= sw_result_beam ? cfg_r_squared : sw_result_geo;
            mem_nn_wr_log   <= '0;
            todo_scan[scan_row] <= 1'b0;
            todo_log[scan_row]  <= 1'b1;
            state <= S_STALE_PICK;
          end
        end

        // ---- Give every touched row a fresh nn_dist_log --------------
        S_REFRESH: begin
          if (!pick_valid && !log_pending) begin
            in_setup <= 1'b0;
            state    <= S_ARGMIN;
          end
          else if (pick_valid && sk_ext_log_ready && !log_pending) begin
            probe_idx <= pick_idx;
            log_row   <= pick_idx;
            state     <= S_REFRESH_RD;
          end
        end

        // The read is registered like every other probe site: without this
        // wait, nn_geo and the weight both come from the PREVIOUS row and
        // every nn_dist_log in the event is computed from the wrong pair.
        S_REFRESH_RD: state <= S_REFRESH_GO;

        // The ready check belongs HERE, on the cycle the request is actually
        // issued -- S_REFRESH tested it two states ago. jc_setkin only lends
        // its log unit while idle, so a request issued into a busy setkin is
        // dropped on the floor while log_pending stays set and S_REFRESH
        // waits for an answer that will never come. Unreachable today, since
        // jc_ctrl is itself the only thing that starts setkin and it is
        // sitting here, but that is an argument, not a guarantee. probe_idx
        // holds, so probe_nngeo and probe_wgt stay valid while we wait.
        S_REFRESH_GO: begin
          if (sk_ext_log_ready) begin
            sk_ext_log_valid <= 1'b1;
            sk_ext_log_x     <= {{(LOG_IN_W-GEO_W){1'b0}}, probe_nngeo};
            log_wgt          <= probe_wgt;
            log_pending      <= 1'b1;
            todo_log[log_row] <= 1'b0;
            state            <= S_REFRESH;
          end
        end

        // ---- Event done -----------------------------------------------
        S_FINISH: begin
          if (!jet_valid || jet_ready) begin
            jet_eoe     <= 1'b1;
            event_count <= event_count + 32'd1;
            state       <= S_IDLE;
          end
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule: jc_ctrl
