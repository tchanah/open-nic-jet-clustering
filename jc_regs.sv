// *************************************************************************
//
// jc_regs -- AXI-Lite control and status for the jet clustering datapath.
//
// Two jobs, and the second is the one that justifies the block.
//
// CONFIGURATION is the obvious one: R and the jet pt floor are properties of
// an analysis, not of an event, so they live here rather than in the packet
// header (jc_defs.vh says so, and a per-event field would spend bytes forever
// on something set once per run). Changing R without this is a Vivado build,
// a JTAG program and a warm reboot -- hours, per value.
//
// STATUS IS WHY IT EXISTS AT ALL. At bring-up the datapath is the thing most
// likely to be broken, so a control path that runs THROUGH the datapath can
// tell you nothing exactly when you need it. "No jets came out" has at least
// five causes -- link down, wrong ethertype, version mismatch, every event
// dropped for want of a slot, every jet below the floor -- and the counters
// that separate them already exist in the RTL. They were simply unreadable.
// One counter per stage boundary, so a zero localises the failure to one
// stage:
//
//   frames_in -> bad_header/bad_length -> accept/drop_full/drop_err
//             -> events -> jets_out -> frames_out/suppressed
//
// FREE-RUNNING, WRAPPING, NO CLEAR BIT. Cumulative since reset means a single
// late read still carries the whole run, and differencing two reads gives an
// interval rate without a clear that would race with the thing being counted.
// frames_in can wrap in about 160 s at line rate, so the host differences.
//
// THE COUNTERS CROSS AS ONE SNAPSHOT. Read one at a time over a 125 MHz bus
// they would not agree with each other -- traffic moves between reads and
// frames_in - bad - accept - drop would not balance. A single handshake
// latches all of them on one 250 MHz edge, so the arithmetic is trustworthy.
// The transfer free-runs rather than waiting to be asked: there is nothing to
// request, and the result is at most a few hundred nanoseconds stale, which
// for cumulative counters is nothing.
//
// CLOCK CROSSING IS xpm_cdc_handshake, NOT HAND-ROLLED, and that is a
// constraints decision rather than a taste one. A hand-written synchroniser
// needs set_max_delay on its crossing, and neither build_box_250mhz.tcl reads
// an XDC -- the only place to put one would be the shell's own timing.xdc,
// which this plugin does not touch. XPM macros carry their own scoped
// constraints, so they need no XDC at all, and the shell already uses
// xpm_cdc_single in generic_reset.sv.
//
// Config crosses the other way on the same primitive. It is quasi-static, and
// BOTH SIDES RESET TO THE GENERATED DEFAULTS -- so a card that is never
// configured clusters correctly from the first packet, and no start-up
// transfer has to be sequenced before the datapath is trustworthy.
//
// Register map, byte offsets in the plugin's 0x80 window:
//
//   0x00  ID              RO  0x4A430001, "JC" and the map version
//   0x04  SCRATCH         RW  proves the bus works, nothing reads it
//   0x08  STATUS          RO  bit 0 = engine idle
//   0x10  R_SQUARED_LO    RW  cfg_r_squared[31:0]
//   0x14  R_SQUARED_HI    RW  cfg_r_squared[48:32]
//   0x18  PT_FLOOR_0      RW  cfg_pt_sq_floor[31:0]     Q28.68
//   0x1C  PT_FLOOR_1      RW  cfg_pt_sq_floor[63:32]
//   0x20  PT_FLOOR_2      RW  cfg_pt_sq_floor[95:64]
//   0x24  DST_MAC_LO      RW  frame header, bytes 2..5
//   0x28  DST_MAC_HI      RW  frame header, bytes 0..1
//   0x2C  SRC_MAC_LO      RW
//   0x30  SRC_MAC_HI      RW
//   0x34  TUSER_SRC       RW  the src field driven on egress tuser
//   0x40  CNT_FRAMES_IN   RO  ingress frames seen
//   0x44  CNT_BAD_HEADER  RO  wrong ethertype / version / count
//   0x48  CNT_BAD_LENGTH  RO  framing disagrees with its own header
//   0x4C  CNT_ACCEPT      RO  events buffered for an engine
//   0x50  CNT_DROP_FULL   RO  no free slot -- the duty cycle, for step 10
//   0x54  CNT_DROP_ERR    RO  bad cell index or aborted frame
//   0x58  CNT_EVENTS      RO  events clustered to completion
//   0x5C  CNT_JETS_OUT    RO  jets above the floor
//   0x60  CNT_FRAMES_OUT  RO  jets frames emitted
//   0x64  CNT_SUPPRESSED  RO  events that fired nothing
//   0x68  CNT_LAST_CYCLES RO  clustering cycles of the most recent event
//
// *************************************************************************
`include "jc_defs.vh"
`timescale 1ns/1ps
module jc_regs #(
  parameter int ADDR_W = 12
) (
  // ---- AXI-Lite, axil_aclk domain --------------------------------------
  input                 s_axil_awvalid,
  input          [31:0] s_axil_awaddr,
  output                s_axil_awready,
  input                 s_axil_wvalid,
  input          [31:0] s_axil_wdata,
  output                s_axil_wready,
  output                s_axil_bvalid,
  output          [1:0] s_axil_bresp,
  input                 s_axil_bready,
  input                 s_axil_arvalid,
  input          [31:0] s_axil_araddr,
  output                s_axil_arready,
  output                s_axil_rvalid,
  output         [31:0] s_axil_rdata,
  output          [1:0] s_axil_rresp,
  input                 s_axil_rready,

  // ---- Configuration out, aclk domain ----------------------------------
  output logic [`JC_GEO_W-1:0]    cfg_r_squared,
  output logic [2*`JC_P4_W-1:0]   cfg_pt_sq_floor,
  output logic          [47:0]    cfg_dst_mac,
  output logic          [47:0]    cfg_src_mac,
  output logic          [15:0]    cfg_tuser_src,

  // ---- Status in, aclk domain ------------------------------------------
  input          [31:0] cnt_frames_in,
  input          [31:0] cnt_bad_header,
  input          [31:0] cnt_bad_length,
  input          [31:0] cnt_accept,
  input          [31:0] cnt_drop_full,
  input          [31:0] cnt_drop_err,
  input          [31:0] cnt_events,
  input          [31:0] cnt_jets_out,
  input          [31:0] cnt_frames_out,
  input          [31:0] cnt_suppressed,
  input          [31:0] cnt_last_cycles,
  input                 stat_idle,

  input                 axil_aclk,
  input                 axil_aresetn,
  input                 aclk,
  input                 aresetn
);

  localparam int GEO_W  = `JC_GEO_W;          // 49
  localparam int FLR_W  = 2 * `JC_P4_W;       // 96
  localparam int CFG_W  = GEO_W + FLR_W + 48 + 48 + 16;   // 257
  localparam int NCNT   = 11;
  localparam int SNAP_W = NCNT * 32 + 1;      // + stat_idle

  // Register indices: byte address >> 2.
  localparam [5:0]
    R_ID        = 6'h00, R_SCRATCH   = 6'h01, R_STATUS   = 6'h02,
    R_RSQ_LO    = 6'h04, R_RSQ_HI    = 6'h05,
    R_FLR_0     = 6'h06, R_FLR_1     = 6'h07, R_FLR_2    = 6'h08,
    R_DMAC_LO   = 6'h09, R_DMAC_HI   = 6'h0A,
    R_SMAC_LO   = 6'h0B, R_SMAC_HI   = 6'h0C,
    R_TUSER_SRC = 6'h0D,
    R_FRAMES_IN = 6'h10, R_BAD_HDR   = 6'h11, R_BAD_LEN  = 6'h12,
    R_ACCEPT    = 6'h13, R_DROP_FULL = 6'h14, R_DROP_ERR = 6'h15,
    R_EVENTS    = 6'h16, R_JETS_OUT  = 6'h17, R_FRAMES_O = 6'h18,
    R_SUPPRESS  = 6'h19, R_LAST_CYC  = 6'h1A;

  // ---- AXI-Lite to a register interface --------------------------------
  // The shell's block, common_clock, so everything below runs on axil_aclk.
  wire              reg_en, reg_we;
  wire [ADDR_W-1:0] reg_addr;
  wire       [31:0] reg_din;
  logic      [31:0] reg_dout;

  axi_lite_register #(
    .CLOCKING_MODE ("common_clock"),
    .ADDR_W        (ADDR_W),
    .DATA_W        (32)
  ) u_axil (
    .s_axil_awvalid (s_axil_awvalid), .s_axil_awaddr (s_axil_awaddr[ADDR_W-1:0]),
    .s_axil_awready (s_axil_awready),
    .s_axil_wvalid  (s_axil_wvalid),  .s_axil_wdata  (s_axil_wdata),
    .s_axil_wready  (s_axil_wready),
    .s_axil_bvalid  (s_axil_bvalid),  .s_axil_bresp  (s_axil_bresp),
    .s_axil_bready  (s_axil_bready),
    .s_axil_arvalid (s_axil_arvalid), .s_axil_araddr (s_axil_araddr[ADDR_W-1:0]),
    .s_axil_arready (s_axil_arready),
    .s_axil_rvalid  (s_axil_rvalid),  .s_axil_rdata  (s_axil_rdata),
    .s_axil_rresp   (s_axil_rresp),   .s_axil_rready (s_axil_rready),

    .reg_en   (reg_en),   .reg_we   (reg_we),
    .reg_addr (reg_addr), .reg_din  (reg_din), .reg_dout (reg_dout),

    .axil_aclk (axil_aclk), .axil_aresetn (axil_aresetn),
    .reg_clk   (axil_aclk), .reg_rstn     (axil_aresetn)
  );

  wire [5:0] idx = reg_addr[7:2];
  wire       wr  = reg_en && reg_we;

  // ---- Held configuration, axil domain ---------------------------------
  logic [GEO_W-1:0] rsq_q;
  logic [FLR_W-1:0] flr_q;
  logic     [47:0]  dmac_q, smac_q;
  logic     [15:0]  tsrc_q;
  logic     [31:0]  scratch_q;
  logic             cfg_dirty;

  wire cfg_write = wr && (idx == R_RSQ_LO  || idx == R_RSQ_HI
                       || idx == R_FLR_0   || idx == R_FLR_1 || idx == R_FLR_2
                       || idx == R_DMAC_LO || idx == R_DMAC_HI
                       || idx == R_SMAC_LO || idx == R_SMAC_HI
                       || idx == R_TUSER_SRC);

  logic cfg_send;
  wire  cfg_rcv;
  wire  cfg_done = cfg_send && cfg_rcv;

  always_ff @(posedge axil_aclk) begin
    if (!axil_aresetn) begin
      rsq_q     <= `JC_DEFAULT_R_SQUARED;
      flr_q     <= `JC_DEFAULT_PT_SQ_FLOOR;
      dmac_q    <= 48'hFFFFFFFFFFFF;      // broadcast until told otherwise
      smac_q    <= 48'd0;
      tsrc_q    <= 16'd0;
      scratch_q <= 32'd0;
      cfg_dirty <= 1'b0;
    end
    else begin
      // Clear first so a write landing on the same cycle as a completed
      // transfer still re-arms: the last value written must always get across.
      if (cfg_done)  cfg_dirty <= 1'b0;
      if (cfg_write) cfg_dirty <= 1'b1;

      if (wr) begin
        case (idx)
          R_SCRATCH:   scratch_q          <= reg_din;
          R_RSQ_LO:    rsq_q[31:0]        <= reg_din;
          R_RSQ_HI:    rsq_q[GEO_W-1:32]  <= reg_din[GEO_W-33:0];
          R_FLR_0:     flr_q[31:0]        <= reg_din;
          R_FLR_1:     flr_q[63:32]       <= reg_din;
          R_FLR_2:     flr_q[FLR_W-1:64]  <= reg_din;
          R_DMAC_LO:   dmac_q[31:0]       <= reg_din;
          R_DMAC_HI:   dmac_q[47:32]      <= reg_din[15:0];
          R_SMAC_LO:   smac_q[31:0]       <= reg_din;
          R_SMAC_HI:   smac_q[47:32]      <= reg_din[15:0];
          R_TUSER_SRC: tsrc_q             <= reg_din[15:0];
          default: ;
        endcase
      end
    end
  end

  // Hold src_send until the far side acknowledges, then re-arm if another
  // write has landed meanwhile.
  // The !cfg_rcv guard is the same protocol requirement the snapshot side
  // above explains at length. Latent rather than observed here: this re-arms
  // only on cfg_dirty, and two host writes are microseconds apart, so cfg_rcv
  // has always fallen by then. It costs nothing to make the property hold by
  // construction rather than by the host bus being slow.
  always_ff @(posedge axil_aclk) begin
    if (!axil_aresetn)          cfg_send <= 1'b0;
    else if (cfg_send && cfg_rcv) cfg_send <= 1'b0;
    else if (!cfg_send && !cfg_rcv && cfg_dirty) cfg_send <= 1'b1;
  end

  wire [CFG_W-1:0] cfg_flat = {tsrc_q, smac_q, dmac_q, flr_q, rsq_q};
  wire [CFG_W-1:0] cfg_dest;
  wire             cfg_req;

  xpm_cdc_handshake #(
    .DEST_EXT_HSK   (0),      // dest_req is a pulse; no ack to drive
    .DEST_SYNC_FF   (4),
    .INIT_SYNC_FF   (0),
    .SIM_ASSERT_CHK (0),
    .SRC_SYNC_FF    (4),
    .WIDTH          (CFG_W)
  ) u_cfg_cdc (
    .src_clk  (axil_aclk), .src_in   (cfg_flat),
    .src_send (cfg_send),  .src_rcv  (cfg_rcv),
    .dest_clk (aclk),      .dest_out (cfg_dest),
    .dest_req (cfg_req),   .dest_ack (1'b0)
  );

  // ---- Live configuration, aclk domain ---------------------------------
  // Same reset values as the axil side, so the datapath is correct before any
  // transfer has happened.
  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      cfg_r_squared   <= `JC_DEFAULT_R_SQUARED;
      cfg_pt_sq_floor <= `JC_DEFAULT_PT_SQ_FLOOR;
      cfg_dst_mac     <= 48'hFFFFFFFFFFFF;
      cfg_src_mac     <= 48'd0;
      cfg_tuser_src   <= 16'd0;
    end
    else if (cfg_req) begin
      {cfg_tuser_src, cfg_src_mac, cfg_dst_mac,
       cfg_pt_sq_floor, cfg_r_squared} <= cfg_dest;
    end
  end

  // ---- Counter snapshot, aclk -> axil ----------------------------------
  wire [SNAP_W-1:0] snap_live = {stat_idle,
                                 cnt_last_cycles, cnt_suppressed,
                                 cnt_frames_out, cnt_jets_out, cnt_events,
                                 cnt_drop_err, cnt_drop_full, cnt_accept,
                                 cnt_bad_length, cnt_bad_header,
                                 cnt_frames_in};

  logic [SNAP_W-1:0] snap_hold;    // stable for the whole transfer
  logic              snap_send;
  wire               snap_rcv;

  // RE-ARM ONLY ONCE snap_rcv HAS FALLEN, and that guard is load-bearing.
  // xpm_cdc_handshake requires src_send not be asserted again until src_rcv
  // has DEASSERTED (UG974). src_rcv needs several src_clk edges to fall after
  // src_send drops, so re-asserting on the very next cycle -- which is what
  // this did -- violates the protocol and wedges the macro's FSM.
  //
  // ON HARDWARE THAT MEANS EXACTLY ONE TRANSFER EVER COMPLETES: the snapshot
  // taken at reset. The whole ladder then reads zero forever while the
  // datapath runs perfectly, which is a spectacularly misleading failure --
  // the counters exist to localise faults, and a frozen ladder blames the
  // first stage for something that is not wrong.
  //
  // Simulation did not catch it. Icarus's XPM model is permissive about the
  // re-assert, so the benches saw a live ladder throughout; only the real
  // macro wedges. The config CDC below survived by luck of timing -- it
  // re-arms on cfg_dirty, and host writes are microseconds apart, so src_rcv
  // has always long since fallen. Same guard applied there anyway.
  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      snap_send <= 1'b0;
      snap_hold <= '0;
    end
    else if (snap_send) begin
      if (snap_rcv) snap_send <= 1'b0;
    end
    else if (!snap_rcv) begin
      // Every counter sampled on ONE edge -- that is what makes the set
      // mutually consistent when the host reads them one at a time.
      snap_hold <= snap_live;
      snap_send <= 1'b1;
    end
  end

  wire [SNAP_W-1:0] snap_dest;
  wire              snap_req;

  xpm_cdc_handshake #(
    .DEST_EXT_HSK   (0),
    .DEST_SYNC_FF   (4),
    .INIT_SYNC_FF   (0),
    .SIM_ASSERT_CHK (0),
    .SRC_SYNC_FF    (4),
    .WIDTH          (SNAP_W)
  ) u_snap_cdc (
    .src_clk  (aclk),       .src_in   (snap_hold),
    .src_send (snap_send),  .src_rcv  (snap_rcv),
    .dest_clk (axil_aclk),  .dest_out (snap_dest),
    .dest_req (snap_req),   .dest_ack (1'b0)
  );

  logic [SNAP_W-1:0] snap_q;
  always_ff @(posedge axil_aclk) begin
    if (!axil_aresetn)  snap_q <= '0;
    else if (snap_req)  snap_q <= snap_dest;
  end

  function automatic [31:0] snap_word (input int n);
    snap_word = snap_q[32*n +: 32];
  endfunction

  // ---- Read decode ------------------------------------------------------
  // Unmapped addresses read zero rather than echoing the address: a host that
  // reads a register this build does not have should see "nothing", not a
  // plausible value.
  always_comb begin
    reg_dout = 32'd0;
    case (idx)
      R_ID:        reg_dout = 32'h4A43_0001;
      R_SCRATCH:   reg_dout = scratch_q;
      R_STATUS:    reg_dout = {31'd0, snap_q[SNAP_W-1]};
      R_RSQ_LO:    reg_dout = rsq_q[31:0];
      R_RSQ_HI:    reg_dout = {{(64-GEO_W){1'b0}}, rsq_q[GEO_W-1:32]};
      R_FLR_0:     reg_dout = flr_q[31:0];
      R_FLR_1:     reg_dout = flr_q[63:32];
      R_FLR_2:     reg_dout = flr_q[FLR_W-1:64];
      R_DMAC_LO:   reg_dout = dmac_q[31:0];
      R_DMAC_HI:   reg_dout = {16'd0, dmac_q[47:32]};
      R_SMAC_LO:   reg_dout = smac_q[31:0];
      R_SMAC_HI:   reg_dout = {16'd0, smac_q[47:32]};
      R_TUSER_SRC: reg_dout = {16'd0, tsrc_q};
      R_FRAMES_IN: reg_dout = snap_word(0);
      R_BAD_HDR:   reg_dout = snap_word(1);
      R_BAD_LEN:   reg_dout = snap_word(2);
      R_ACCEPT:    reg_dout = snap_word(3);
      R_DROP_FULL: reg_dout = snap_word(4);
      R_DROP_ERR:  reg_dout = snap_word(5);
      R_EVENTS:    reg_dout = snap_word(6);
      R_JETS_OUT:  reg_dout = snap_word(7);
      R_FRAMES_O:  reg_dout = snap_word(8);
      R_SUPPRESS:  reg_dout = snap_word(9);
      R_LAST_CYC:  reg_dout = snap_word(10);
      default:     reg_dout = 32'd0;
    endcase
  end

endmodule: jc_regs
