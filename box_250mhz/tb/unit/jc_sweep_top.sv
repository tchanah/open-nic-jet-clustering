// *************************************************************************
//
// jc_sweep_top -- test harness: jc_sweep wired to the real jc_mem.
//
// Not production RTL. jc_engine will instantiate the same pair, but this
// exists so the sweep is exercised against the actual memory rather than a
// Python stand-in -- the read latency, the banking and the write-back path
// are exactly the things a mock would get wrong in the same way the RTL did.
//
// The bench loads the active list through jc_mem's single-entry ports, then
// drives jc_sweep and reads the results.
//
// *************************************************************************
`include "jc_defs.vh"
`timescale 1ns/1ps
module jc_sweep_top (
  // ---- Active-list loading (straight through to jc_mem) ----------------
  input                            init_en,
  input  [`JC_IDX_W-1:0]           init_idx,
  input  signed [`JC_P4_W-1:0]     init_e,
  input  signed [`JC_P4_W-1:0]     init_px,
  input  signed [`JC_P4_W-1:0]     init_py,
  input  signed [`JC_P4_W-1:0]     init_pz,
  input  signed [`JC_COORD_W-1:0]  init_y,
  input         [`JC_COORD_W-1:0]  init_phi,
  input  signed [`JC_WGT_W-1:0]    init_wgt,

  input                            kill_en,
  input  [`JC_IDX_W-1:0]           kill_idx,

  input                            nn_wr_en,
  input  [`JC_IDX_W-1:0]           nn_wr_idx,
  input                            nn_wr_beam,
  input  [`JC_IDX_W-1:0]           nn_wr_index,
  input  [`JC_GEO_W-1:0]           nn_wr_geo,
  input  signed [`JC_NNLOG_W-1:0]  nn_wr_log,

  // ---- Sweep control ---------------------------------------------------
  input                            start,
  input                      [1:0] mode,
  input  [`JC_IDX_W-1:0]           query_idx,
  input  signed [`JC_COORD_W-1:0]  query_y,
  input         [`JC_COORD_W-1:0]  query_phi,
  input  [`JC_IDX_W-1:0]           mark_a,
  input  [`JC_IDX_W-1:0]           mark_b,
  input  [`JC_GEO_W-1:0]           r_squared,
  output                           busy,
  output                           done,

  output                           result_valid,
  output [`JC_IDX_W-1:0]           result_index,
  output [`JC_GEO_W-1:0]           result_geo,
  output                           result_beam,
  output signed [`JC_NNLOG_W-1:0]  result_log,
  output [`JC_NMAX-1:0]            claimed_mask,
  output [`JC_NMAX-1:0]            stale_mask,

  // ---- Observation: read the stored nn state back ----------------------
  input  [`JC_OFF_W-1:0]                 obs_off,
  output [`JC_LANES*`JC_IDX_W-1:0]       obs_nn_index,
  output [`JC_LANES*`JC_GEO_W-1:0]       obs_nn_geo,
  output [`JC_LANES-1:0]                 obs_beam,
  output [`JC_LANES-1:0]                 obs_active,
  output [`JC_NMAX-1:0]                  active_mask,

  input                                  aclk,
  input                                  aresetn
);

  // jc_sweep owns the read port while it runs; the bench borrows it when
  // idle to inspect what the write-back left behind.
  wire [`JC_OFF_W-1:0] sweep_rd_off;
  wire [`JC_OFF_W-1:0] mem_rd_off = busy ? sweep_rd_off : obs_off;

  wire [`JC_LANES*`JC_COORD_W-1:0] rd_y, rd_phi;
  wire [`JC_LANES*`JC_WGT_W-1:0]   rd_wgt;
  wire [`JC_LANES-1:0]             rd_active, rd_beam;
  wire [`JC_LANES*`JC_IDX_W-1:0]   rd_nn_index;
  wire [`JC_LANES*`JC_GEO_W-1:0]   rd_nn_geo;
  wire [`JC_LANES*`JC_NNLOG_W-1:0] rd_nn_log;

  wire [`JC_OFF_W-1:0]             wb_off;
  wire [`JC_LANES-1:0]             wb_en;
  wire [`JC_LANES*`JC_IDX_W-1:0]   wb_nn_index;
  wire [`JC_LANES*`JC_GEO_W-1:0]   wb_nn_geo;

  assign obs_nn_index = rd_nn_index;
  assign obs_nn_geo   = rd_nn_geo;
  assign obs_beam     = rd_beam;
  assign obs_active   = rd_active;

  jc_mem u_mem (
    .init_en     (init_en),   .init_idx (init_idx),
    .init_e      (init_e),    .init_px  (init_px),
    .init_py     (init_py),   .init_pz  (init_pz),
    .init_y      (init_y),    .init_phi (init_phi), .init_wgt (init_wgt),

    // Sized zeros: a bare '0 elaborates one bit wide in Icarus and buries the
    // real width mismatches in a wall of padding warnings.
    .set_en      (1'b0),      .set_idx  (`JC_IDX_W'd0),
    .set_y       (`JC_COORD_W'd0), .set_phi (`JC_COORD_W'd0),
    .set_wgt     (`JC_WGT_W'd0),

    .p4_rd_idx   (`JC_IDX_W'd0),
    .p4_rd_e     (),          .p4_rd_px (),         .p4_rd_py (), .p4_rd_pz (),
    .p4_wr_en    (1'b0),      .p4_wr_idx(`JC_IDX_W'd0),
    .p4_wr_e     (`JC_P4_W'd0), .p4_wr_px (`JC_P4_W'd0),
    .p4_wr_py    (`JC_P4_W'd0), .p4_wr_pz (`JC_P4_W'd0),

    .kill_en     (kill_en),   .kill_idx (kill_idx),

    .nn_wr_en    (nn_wr_en),  .nn_wr_idx(nn_wr_idx),
    .nn_wr_beam  (nn_wr_beam),.nn_wr_index(nn_wr_index),
    .nn_wr_geo   (nn_wr_geo), .nn_wr_log(nn_wr_log),

    .log_wr_en   (1'b0),      .log_wr_idx(`JC_IDX_W'd0),
    .log_wr_val  (`JC_NNLOG_W'd0),

    .active_mask (active_mask),

    .rd_off      (mem_rd_off),
    .rd_y        (rd_y),      .rd_phi   (rd_phi),   .rd_wgt   (rd_wgt),
    .rd_active   (rd_active), .rd_beam  (rd_beam),
    .rd_nn_index (rd_nn_index),
    .rd_nn_geo   (rd_nn_geo), .rd_nn_log(rd_nn_log),

    .wb_off      (wb_off),    .wb_en    (wb_en),
    .wb_nn_index (wb_nn_index), .wb_nn_geo (wb_nn_geo),

    .aclk        (aclk),      .aresetn  (aresetn)
  );

  jc_sweep u_sweep (
    .start        (start),      .mode      (mode),
    .query_idx    (query_idx),  .query_y   (query_y), .query_phi (query_phi),
    .mark_a       (mark_a),     .mark_b    (mark_b),
    .r_squared    (r_squared),
    .busy         (busy),       .done      (done),

    .result_valid (result_valid), .result_index (result_index),
    .result_geo   (result_geo),   .result_beam  (result_beam),
    .result_log   (result_log),
    .claimed_mask (claimed_mask), .stale_mask   (stale_mask),

    .mem_rd_off      (sweep_rd_off),
    .mem_rd_y        (rd_y),        .mem_rd_phi (rd_phi),
    .mem_rd_active   (rd_active),
    .mem_rd_nn_index (rd_nn_index), .mem_rd_nn_geo (rd_nn_geo),
    .mem_rd_nn_log   (rd_nn_log),

    .mem_wb_off      (wb_off),      .mem_wb_en  (wb_en),
    .mem_wb_nn_index (wb_nn_index), .mem_wb_nn_geo (wb_nn_geo),

    .aclk (aclk), .aresetn (aresetn)
  );

endmodule: jc_sweep_top
