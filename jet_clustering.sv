// *************************************************************************
//
// jet_clustering -- top of the anti-kt jet clustering datapath.
//
// A BUMP IN THE WIRE: calorimeter-cell events arrive from the network on
// CMAC port 0 RX and jets leave on CMAC port 1 TX, to be captured on a third
// machine. Nothing goes to the local host -- QDMA C2H is tied off, so the
// PCIe side carries control and status only.
// Single 250 MHz domain for the datapath, 512-bit AXI-Stream,
// 48-bit tuser = {dst[47:32], src[31:16], size[15:0]}. AXI-Lite arrives on its
// own 125 MHz clock and is crossed inside jc_regs.
//
//   s_axis --> jc_deframe --> jc_ingest --> jc_evbuf --> jc_engine
//                                                            |
//   m_axis <-------------------------------------------- jc_reframe
//
// Each stage's reasoning lives in its own file; what is decided HERE is the
// counter ladder and where back-pressure is allowed to reach.
//
// BACK-PRESSURE STOPS AT jc_evbuf, BY DESIGN. Its pj_ready is tied high, so a
// busy engine can never reach s_axis_tready -- a device in the network path
// must not stall the shell's RX. Events that arrive with no free slot are
// counted and discarded whole.
//
// jc_deframe DOES lower s_axis_tready, and that is not a contradiction: it
// emits one cell per cycle out of a sixteen-cell beat, so it stalls its input
// about sixteen cycles per beat. Worth being explicit about the consequence,
// because it bounds what the counters below can see. Frames dropped by the
// packet adapter while jc_deframe is stalled never reach frames_in, so
// accept/(accept+drop_full) measures the duty cycle of THIS BLOCK, not the
// fraction of offered traffic that was clustered. The host knows the offered
// rate; the card cannot. At one event per ~16k cycles against a line rate of
// millions of packets per second, over-subscription is the normal condition
// and not a fault.
//
// ONE COUNTER PER STAGE BOUNDARY, so a zero says which stage died rather than
// merely that nothing came out. See jc_regs for the map and the reasoning.
//
// *************************************************************************
`include "jc_defs.vh"
`timescale 1ns/1ps
module jet_clustering (
  // ---- AXI-Lite control, axil_aclk domain ------------------------------
  input          s_axil_awvalid,
  input   [31:0] s_axil_awaddr,
  output         s_axil_awready,
  input          s_axil_wvalid,
  input   [31:0] s_axil_wdata,
  output         s_axil_wready,
  output         s_axil_bvalid,
  output   [1:0] s_axil_bresp,
  input          s_axil_bready,
  input          s_axil_arvalid,
  input   [31:0] s_axil_araddr,
  output         s_axil_arready,
  output         s_axil_rvalid,
  output  [31:0] s_axil_rdata,
  output   [1:0] s_axil_rresp,
  input          s_axil_rready,

  // ---- Cell events in ---------------------------------------------------
  input          s_axis_tvalid,
  input  [511:0] s_axis_tdata,
  input   [63:0] s_axis_tkeep,
  input          s_axis_tlast,
  input   [47:0] s_axis_tuser,
  output         s_axis_tready,

  // ---- Jets out ---------------------------------------------------------
  output         m_axis_tvalid,
  output [511:0] m_axis_tdata,
  output  [63:0] m_axis_tkeep,
  output         m_axis_tlast,
  output  [47:0] m_axis_tuser,
  input          m_axis_tready,

  input          axil_aclk,
  input          axil_aresetn,
  input          aclk,
  input          aresetn
);

  // ---- Configuration, driven by jc_regs in the aclk domain -------------
  wire [`JC_GEO_W-1:0]  cfg_r_squared;
  wire [2*`JC_P4_W-1:0] cfg_pt_sq_floor;
  wire         [47:0]   cfg_dst_mac, cfg_src_mac;
  wire         [15:0]   cfg_tuser_src;

  // ---- Deframe ----------------------------------------------------------
  wire [`JC_CELL_W-1:0] cell_data;
  wire                  cell_valid, cell_start, cell_last, cell_err, cell_ready;
  wire [`JC_CNT_W-1:0]  event_cell_count;
  wire         [31:0]   event_seq;
  wire         [15:0]   event_cells_total;
  wire         [31:0]   bad_event_count, bad_header_count, bad_length_count;

  jc_deframe u_deframe (
    .s_axis_tvalid (s_axis_tvalid), .s_axis_tdata (s_axis_tdata),
    .s_axis_tkeep  (s_axis_tkeep),  .s_axis_tlast (s_axis_tlast),
    .s_axis_tready (s_axis_tready),

    .cell_data (cell_data), .cell_valid (cell_valid),
    .cell_start (cell_start), .cell_last (cell_last), .cell_err (cell_err),
    .cell_ready (cell_ready),

    .event_cell_count  (event_cell_count),
    .event_seq         (event_seq),
    .event_cells_total (event_cells_total),
    .bad_event_count   (bad_event_count),
    .bad_header_count  (bad_header_count),
    .bad_length_count  (bad_length_count),

    .aclk (aclk), .aresetn (aresetn)
  );

  // ---- Ingest -----------------------------------------------------------
  wire                          pj_valid, pj_start, pj_last, pj_err, pj_ready;
  wire signed [`JC_P4_W-1:0]    pj_energy, pj_px, pj_py, pj_pz;
  wire signed [`JC_RAP_W-1:0]   pj_rapidity;
  wire        [`JC_PHI_W-1:0]   pj_phi;
  wire signed [`JC_WGT_W-1:0]   pj_beam_weight_log;
  wire                 [31:0]   pj_event_seq;

  jc_ingest u_ingest (
    .cell_valid (cell_valid), .cell_data (cell_data),
    .cell_start (cell_start), .cell_last (cell_last), .cell_err (cell_err),
    .cell_event_seq (event_seq), .cell_ready (cell_ready),

    .pj_valid (pj_valid),
    .pj_energy (pj_energy), .pj_px (pj_px), .pj_py (pj_py), .pj_pz (pj_pz),
    .pj_rapidity (pj_rapidity), .pj_phi (pj_phi),
    .pj_beam_weight_log (pj_beam_weight_log),
    .pj_start (pj_start), .pj_last (pj_last),
    .pj_event_seq (pj_event_seq), .pj_err (pj_err), .pj_ready (pj_ready),

    .aclk (aclk), .aresetn (aresetn)
  );

  // ---- Event buffer -----------------------------------------------------
  wire                        ev_valid, ev_accept, ev_release;
  wire [`JC_CNT_W-1:0]        ev_count;
  wire               [31:0]   ev_seq;
  wire [`JC_IDX_W-1:0]        ev_addr;
  wire signed [`JC_P4_W-1:0]  ev_energy, ev_px, ev_py, ev_pz;
  wire signed [`JC_RAP_W-1:0] ev_rapidity;
  wire        [`JC_PHI_W-1:0] ev_phi;
  wire signed [`JC_WGT_W-1:0] ev_beam_weight_log;
  wire               [31:0]   accept_count, drop_count;
  wire               [31:0]   drop_full_count, drop_err_count;

  jc_evbuf u_evbuf (
    .pj_valid (pj_valid),
    .pj_energy (pj_energy), .pj_px (pj_px), .pj_py (pj_py), .pj_pz (pj_pz),
    .pj_rapidity (pj_rapidity), .pj_phi (pj_phi),
    .pj_beam_weight_log (pj_beam_weight_log),
    .pj_start (pj_start), .pj_last (pj_last), .pj_err (pj_err),
    .pj_event_seq (pj_event_seq), .pj_ready (pj_ready),

    .ev_valid (ev_valid), .ev_count (ev_count), .ev_seq (ev_seq),
    .ev_accept (ev_accept), .ev_addr (ev_addr),
    .ev_energy (ev_energy), .ev_px (ev_px), .ev_py (ev_py), .ev_pz (ev_pz),
    .ev_rapidity (ev_rapidity), .ev_phi (ev_phi),
    .ev_beam_weight_log (ev_beam_weight_log), .ev_release (ev_release),

    .accept_count (accept_count), .drop_count (drop_count),
    .drop_full_count (drop_full_count), .drop_err_count (drop_err_count),

    .aclk (aclk), .aresetn (aresetn)
  );

  // ---- Engine -----------------------------------------------------------
  wire                       jet_valid, jet_eoe, jet_ready;
  wire signed [`JC_P4_W-1:0] jet_e, jet_px, jet_py, jet_pz;
  wire              [31:0]   jet_seq;
  wire                       engine_idle;
  wire              [31:0]   engine_events, engine_cycles;

  jc_engine u_engine (
    .ev_valid (ev_valid), .ev_count (ev_count), .ev_seq (ev_seq),
    .ev_accept (ev_accept), .ev_addr (ev_addr),
    .ev_energy (ev_energy), .ev_px (ev_px), .ev_py (ev_py), .ev_pz (ev_pz),
    .ev_rapidity (ev_rapidity), .ev_phi (ev_phi),
    .ev_beam_weight_log (ev_beam_weight_log), .ev_release (ev_release),

    .cfg_r_squared (cfg_r_squared), .cfg_pt_sq_floor (cfg_pt_sq_floor),

    .jet_valid (jet_valid),
    .jet_e (jet_e), .jet_px (jet_px), .jet_py (jet_py), .jet_pz (jet_pz),
    .jet_seq (jet_seq), .jet_eoe (jet_eoe), .jet_ready (jet_ready),

    .idle (engine_idle),
    .event_count (engine_events), .cycle_count (engine_cycles),

    .aclk (aclk), .aresetn (aresetn)
  );

  // ---- Reframe ----------------------------------------------------------
  wire [31:0] frames_out, jets_out, suppressed;

  jc_reframe u_reframe (
    .jet_valid (jet_valid),
    .jet_e (jet_e), .jet_px (jet_px), .jet_py (jet_py), .jet_pz (jet_pz),
    .jet_seq (jet_seq), .jet_eoe (jet_eoe), .jet_ready (jet_ready),

    .ev_cycles (engine_cycles),
    .cnt_drop_full (drop_full_count), .cnt_drop_err (drop_err_count),
    .cnt_bad_frame (bad_event_count),

    .cfg_dst_mac (cfg_dst_mac), .cfg_src_mac (cfg_src_mac),
    .cfg_tuser_src (cfg_tuser_src),

    .m_axis_tvalid (m_axis_tvalid), .m_axis_tdata (m_axis_tdata),
    .m_axis_tkeep  (m_axis_tkeep),  .m_axis_tlast (m_axis_tlast),
    .m_axis_tuser  (m_axis_tuser),  .m_axis_tready (m_axis_tready),

    .frames_out (frames_out), .jets_out (jets_out),
    .suppressed (suppressed), .busy (),

    .aclk (aclk), .aresetn (aresetn)
  );

  // ---- The one counter no submodule owns --------------------------------
  // Frames offered to the block, counted before any validation, so
  // frames_in - bad_header - bad_length is what jc_deframe accepted.
  logic [31:0] frames_in;
  always_ff @(posedge aclk) begin
    if (!aresetn) frames_in <= 32'd0;
    else if (s_axis_tvalid && s_axis_tready && s_axis_tlast)
      frames_in <= frames_in + 32'd1;
  end

  // ---- Control and status ----------------------------------------------
  jc_regs u_regs (
    .s_axil_awvalid (s_axil_awvalid), .s_axil_awaddr (s_axil_awaddr),
    .s_axil_awready (s_axil_awready),
    .s_axil_wvalid  (s_axil_wvalid),  .s_axil_wdata  (s_axil_wdata),
    .s_axil_wready  (s_axil_wready),
    .s_axil_bvalid  (s_axil_bvalid),  .s_axil_bresp  (s_axil_bresp),
    .s_axil_bready  (s_axil_bready),
    .s_axil_arvalid (s_axil_arvalid), .s_axil_araddr (s_axil_araddr),
    .s_axil_arready (s_axil_arready),
    .s_axil_rvalid  (s_axil_rvalid),  .s_axil_rdata  (s_axil_rdata),
    .s_axil_rresp   (s_axil_rresp),   .s_axil_rready (s_axil_rready),

    .cfg_r_squared (cfg_r_squared), .cfg_pt_sq_floor (cfg_pt_sq_floor),
    .cfg_dst_mac (cfg_dst_mac), .cfg_src_mac (cfg_src_mac),
    .cfg_tuser_src (cfg_tuser_src),

    .cnt_frames_in   (frames_in),
    .cnt_bad_header  (bad_header_count),
    .cnt_bad_length  (bad_length_count),
    .cnt_accept      (accept_count),
    .cnt_drop_full   (drop_full_count),
    .cnt_drop_err    (drop_err_count),
    .cnt_events      (engine_events),
    .cnt_jets_out    (jets_out),
    .cnt_frames_out  (frames_out),
    .cnt_suppressed  (suppressed),
    .cnt_last_cycles (engine_cycles),
    .stat_idle       (engine_idle),

    .axil_aclk (axil_aclk), .axil_aresetn (axil_aresetn),
    .aclk (aclk), .aresetn (aresetn)
  );

  // event_cells_total and drop_count are deliberately not routed out. The
  // first is per-event truncation context that belongs in the jets header if
  // anywhere, and the second is drop_full + drop_err, which the host can add.
  wire _unused = &{1'b0, event_cells_total, event_cell_count, drop_count,
                   s_axis_tuser, 1'b0};

endmodule: jet_clustering
