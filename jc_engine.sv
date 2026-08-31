// *************************************************************************
//
// jc_engine -- one clustering engine: memory, sweep, setkin, control.
//
// Structural. All the reasoning lives in the four modules it wires together;
// the only decision made here is who owns jc_mem's read port.
//
// THE READ PORT IS SHARED, NOT DUPLICATED. jc_sweep streams offsets through
// it for the whole of a sweep; jc_ctrl needs single-row lookups between
// sweeps. They never overlap -- ctrl issues a sweep and waits for done --
// so a second port would be idle hardware. The mux follows sw_busy, which
// is the sweep's own statement of when it is using it.
//
// Everything else is a straight connection. jc_setkin owns the single
// jc_log2 and lends it to jc_ctrl while idle, which is how the nn_dist_log
// refresh gets a logarithm without putting one in sixteen lanes.
//
// *************************************************************************
`include "jc_defs.vh"
`timescale 1ns/1ps
module jc_engine (
  // ---- Event source: jc_evbuf ------------------------------------------
  input                                 ev_valid,
  input  [`JC_CNT_W-1:0]                ev_count,
  input                          [31:0] ev_seq,
  output                                ev_accept,
  output [`JC_IDX_W-1:0]                ev_addr,
  input  signed [`JC_P4_W-1:0]          ev_energy, ev_px, ev_py, ev_pz,
  input  signed [`JC_COORD_W-1:0]       ev_rapidity,
  input         [`JC_COORD_W-1:0]       ev_phi,
  input  signed [`JC_WGT_W-1:0]         ev_beam_weight_log,
  output                                ev_release,

  // ---- Configuration ----------------------------------------------------
  input  [`JC_GEO_W-1:0]                cfg_r_squared,
  input  [2*`JC_P4_W-1:0]               cfg_pt_sq_floor,

  // ---- Jets out ---------------------------------------------------------
  output                                jet_valid,
  output signed [`JC_P4_W-1:0]          jet_e, jet_px, jet_py, jet_pz,
  output                         [31:0] jet_seq,
  output                                jet_eoe,
  input                                 jet_ready,

  // ---- Status -----------------------------------------------------------
  output                                idle,
  output                         [31:0] event_count,
  output                         [31:0] cycle_count,

  input                                 aclk,
  input                                 aresetn
);

  // ---- jc_mem write ports (all driven by jc_ctrl) ----------------------
  wire                          init_en;
  wire [`JC_IDX_W-1:0]          init_idx;
  wire signed [`JC_P4_W-1:0]    init_e, init_px, init_py, init_pz;
  wire signed [`JC_COORD_W-1:0] init_y;
  wire        [`JC_COORD_W-1:0] init_phi;
  wire signed [`JC_WGT_W-1:0]   init_wgt;

  wire                          set_en;
  wire [`JC_IDX_W-1:0]          set_idx;
  wire signed [`JC_COORD_W-1:0] set_y;
  wire        [`JC_COORD_W-1:0] set_phi;
  wire signed [`JC_WGT_W-1:0]   set_wgt;

  wire [`JC_IDX_W-1:0]          p4_rd_idx;
  wire signed [`JC_P4_W-1:0]    p4_rd_e, p4_rd_px, p4_rd_py, p4_rd_pz;
  wire                          p4_wr_en;
  wire [`JC_IDX_W-1:0]          p4_wr_idx;
  wire signed [`JC_P4_W-1:0]    p4_wr_e, p4_wr_px, p4_wr_py, p4_wr_pz;

  wire                          kill_en;
  wire [`JC_IDX_W-1:0]          kill_idx;

  wire                          nn_wr_en;
  wire [`JC_IDX_W-1:0]          nn_wr_idx;
  wire                          nn_wr_beam;
  wire [`JC_IDX_W-1:0]          nn_wr_index;
  wire [`JC_GEO_W-1:0]          nn_wr_geo;
  wire signed [`JC_NNLOG_W-1:0] nn_wr_log;

  wire                          log_wr_en;
  wire [`JC_IDX_W-1:0]          log_wr_idx;
  wire signed [`JC_NNLOG_W-1:0] log_wr_val;

  // ---- jc_mem read, shared ---------------------------------------------
  wire [`JC_OFF_W-1:0] sw_rd_off, ct_rd_off;
  wire                 sw_busy;
  wire [`JC_OFF_W-1:0] mem_rd_off = sw_busy ? sw_rd_off : ct_rd_off;

  wire [`JC_LANES*`JC_COORD_W-1:0] rd_y, rd_phi;
  wire [`JC_LANES*`JC_WGT_W-1:0]   rd_wgt;
  wire [`JC_LANES-1:0]             rd_active, rd_beam;
  wire [`JC_LANES*`JC_IDX_W-1:0]   rd_nn_index;
  wire [`JC_LANES*`JC_GEO_W-1:0]   rd_nn_geo;
  wire [`JC_LANES*`JC_NNLOG_W-1:0] rd_nn_log;
  wire [`JC_NMAX-1:0]              active_mask;

  wire [`JC_OFF_W-1:0]             wb_off;
  wire [`JC_LANES-1:0]             wb_en;
  wire [`JC_LANES*`JC_IDX_W-1:0]   wb_nn_index;
  wire [`JC_LANES*`JC_GEO_W-1:0]   wb_nn_geo;

  jc_mem u_mem (
    .init_en (init_en), .init_idx (init_idx),
    .init_e  (init_e),  .init_px  (init_px),
    .init_py (init_py), .init_pz  (init_pz),
    .init_y  (init_y),  .init_phi (init_phi), .init_wgt (init_wgt),

    .set_en  (set_en),  .set_idx  (set_idx),
    .set_y   (set_y),   .set_phi  (set_phi),  .set_wgt  (set_wgt),

    .p4_rd_idx (p4_rd_idx),
    .p4_rd_e (p4_rd_e), .p4_rd_px (p4_rd_px),
    .p4_rd_py(p4_rd_py),.p4_rd_pz (p4_rd_pz),
    .p4_wr_en(p4_wr_en),.p4_wr_idx(p4_wr_idx),
    .p4_wr_e (p4_wr_e), .p4_wr_px (p4_wr_px),
    .p4_wr_py(p4_wr_py),.p4_wr_pz (p4_wr_pz),

    .kill_en (kill_en), .kill_idx (kill_idx),

    .nn_wr_en(nn_wr_en),.nn_wr_idx(nn_wr_idx),
    .nn_wr_beam (nn_wr_beam), .nn_wr_index (nn_wr_index),
    .nn_wr_geo  (nn_wr_geo),  .nn_wr_log   (nn_wr_log),

    .log_wr_en (log_wr_en), .log_wr_idx (log_wr_idx), .log_wr_val (log_wr_val),

    .rd_off      (mem_rd_off),
    .rd_y        (rd_y),      .rd_phi   (rd_phi),  .rd_wgt (rd_wgt),
    .rd_active   (rd_active), .rd_beam  (rd_beam),
    .rd_nn_index (rd_nn_index),
    .rd_nn_geo   (rd_nn_geo), .rd_nn_log(rd_nn_log),

    .wb_off      (wb_off),    .wb_en    (wb_en),
    .wb_nn_index (wb_nn_index), .wb_nn_geo (wb_nn_geo),

    .active_mask (active_mask),
    .aclk (aclk), .aresetn (aresetn)
  );

  // ---- Sweep ------------------------------------------------------------
  wire                    sw_start, sw_done;
  wire              [1:0] sw_mode;
  wire [`JC_IDX_W-1:0]    sw_query_idx, sw_mark_a, sw_mark_b;
  wire signed [`JC_COORD_W-1:0] sw_query_y;
  wire        [`JC_COORD_W-1:0] sw_query_phi;
  wire                    sw_result_valid, sw_result_beam;
  wire [`JC_IDX_W-1:0]    sw_result_index;
  wire [`JC_GEO_W-1:0]    sw_result_geo;
  wire [`JC_NMAX-1:0]     sw_claimed_mask, sw_stale_mask;

  jc_sweep u_sweep (
    .start (sw_start), .mode (sw_mode),
    .query_idx (sw_query_idx), .query_y (sw_query_y), .query_phi (sw_query_phi),
    .mark_a (sw_mark_a), .mark_b (sw_mark_b),
    .r_squared (cfg_r_squared),
    .busy (sw_busy), .done (sw_done),

    .result_valid (sw_result_valid), .result_index (sw_result_index),
    .result_geo   (sw_result_geo),   .result_beam  (sw_result_beam),
    .result_log   (),
    .claimed_mask (sw_claimed_mask), .stale_mask   (sw_stale_mask),

    .mem_rd_off      (sw_rd_off),
    .mem_rd_y        (rd_y),        .mem_rd_phi (rd_phi),
    .mem_rd_active   (rd_active),
    .mem_rd_nn_index (rd_nn_index), .mem_rd_nn_geo (rd_nn_geo),
    .mem_rd_nn_log   (rd_nn_log),

    .mem_wb_off      (wb_off),      .mem_wb_en  (wb_en),
    .mem_wb_nn_index (wb_nn_index), .mem_wb_nn_geo (wb_nn_geo),

    .aclk (aclk), .aresetn (aresetn)
  );

  // ---- Merge reconversion, and the engine's only logarithm --------------
  wire                    sk_start, sk_done;
  wire signed [`JC_P4_W-1:0] sk_e, sk_px, sk_py, sk_pz;
  wire signed [`JC_COORD_W-1:0] sk_y;
  wire        [`JC_COORD_W-1:0] sk_phi;
  wire signed [`JC_WGT_W-1:0]   sk_wgt;
  wire                    sk_ext_log_valid, sk_ext_log_ready, sk_ext_log_rsp_valid;
  wire [`JC_LOG2_IN_W-1:0]  sk_ext_log_x;
  wire [`JC_LOG2_OUT_W-1:0] sk_ext_log_rsp;

  jc_setkin u_setkin (
    .start (sk_start),
    .in_e (sk_e), .in_px (sk_px), .in_py (sk_py), .in_pz (sk_pz),
    .busy (), .done (sk_done),
    .out_y (sk_y), .out_phi (sk_phi), .out_wgt (sk_wgt),

    .ext_log_valid     (sk_ext_log_valid),
    .ext_log_x         (sk_ext_log_x),
    .ext_log_ready     (sk_ext_log_ready),
    .ext_log_rsp_valid (sk_ext_log_rsp_valid),
    .ext_log_rsp       (sk_ext_log_rsp),

    .aclk (aclk), .aresetn (aresetn)
  );

  // ---- The round FSM ----------------------------------------------------
  jc_ctrl u_ctrl (
    .ev_valid (ev_valid), .ev_count (ev_count), .ev_seq (ev_seq),
    .ev_accept (ev_accept), .ev_addr (ev_addr),
    .ev_energy (ev_energy), .ev_px (ev_px), .ev_py (ev_py), .ev_pz (ev_pz),
    .ev_rapidity (ev_rapidity), .ev_phi (ev_phi),
    .ev_beam_weight_log (ev_beam_weight_log),
    .ev_release (ev_release),

    .cfg_r_squared (cfg_r_squared), .cfg_pt_sq_floor (cfg_pt_sq_floor),

    .mem_init_en (init_en), .mem_init_idx (init_idx),
    .mem_init_e (init_e), .mem_init_px (init_px),
    .mem_init_py (init_py), .mem_init_pz (init_pz),
    .mem_init_y (init_y), .mem_init_phi (init_phi), .mem_init_wgt (init_wgt),

    .mem_set_en (set_en), .mem_set_idx (set_idx),
    .mem_set_y (set_y), .mem_set_phi (set_phi), .mem_set_wgt (set_wgt),

    .mem_p4_rd_idx (p4_rd_idx),
    .mem_p4_rd_e (p4_rd_e), .mem_p4_rd_px (p4_rd_px),
    .mem_p4_rd_py (p4_rd_py), .mem_p4_rd_pz (p4_rd_pz),
    .mem_p4_wr_en (p4_wr_en), .mem_p4_wr_idx (p4_wr_idx),
    .mem_p4_wr_e (p4_wr_e), .mem_p4_wr_px (p4_wr_px),
    .mem_p4_wr_py (p4_wr_py), .mem_p4_wr_pz (p4_wr_pz),

    .mem_kill_en (kill_en), .mem_kill_idx (kill_idx),

    .mem_nn_wr_en (nn_wr_en), .mem_nn_wr_idx (nn_wr_idx),
    .mem_nn_wr_beam (nn_wr_beam), .mem_nn_wr_index (nn_wr_index),
    .mem_nn_wr_geo (nn_wr_geo), .mem_nn_wr_log (nn_wr_log),

    .mem_log_wr_en (log_wr_en), .mem_log_wr_idx (log_wr_idx),
    .mem_log_wr_val (log_wr_val),

    .mem_rd_off (ct_rd_off),
    .mem_rd_y (rd_y), .mem_rd_phi (rd_phi), .mem_rd_wgt (rd_wgt),
    .mem_rd_beam (rd_beam), .mem_rd_nn_index (rd_nn_index),
    .mem_rd_nn_geo (rd_nn_geo), .mem_active_mask (active_mask),

    .sw_start (sw_start), .sw_mode (sw_mode),
    .sw_query_idx (sw_query_idx), .sw_query_y (sw_query_y),
    .sw_query_phi (sw_query_phi),
    .sw_mark_a (sw_mark_a), .sw_mark_b (sw_mark_b),
    .sw_done (sw_done),
    .sw_result_valid (sw_result_valid), .sw_result_index (sw_result_index),
    .sw_result_geo (sw_result_geo), .sw_result_beam (sw_result_beam),
    .sw_claimed_mask (sw_claimed_mask), .sw_stale_mask (sw_stale_mask),

    .sk_start (sk_start),
    .sk_e (sk_e), .sk_px (sk_px), .sk_py (sk_py), .sk_pz (sk_pz),
    .sk_done (sk_done),
    .sk_y (sk_y), .sk_phi (sk_phi), .sk_wgt (sk_wgt),
    .sk_ext_log_valid (sk_ext_log_valid), .sk_ext_log_x (sk_ext_log_x),
    .sk_ext_log_ready (sk_ext_log_ready),
    .sk_ext_log_rsp_valid (sk_ext_log_rsp_valid),
    .sk_ext_log_rsp (sk_ext_log_rsp),

    .jet_valid (jet_valid),
    .jet_e (jet_e), .jet_px (jet_px), .jet_py (jet_py), .jet_pz (jet_pz),
    .jet_seq (jet_seq), .jet_eoe (jet_eoe), .jet_ready (jet_ready),

    .idle (idle), .event_count (event_count), .cycle_count (cycle_count),

    .aclk (aclk), .aresetn (aresetn)
  );

endmodule: jc_engine
