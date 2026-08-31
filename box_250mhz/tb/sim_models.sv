// *************************************************************************
//
// Simulation-only stand-ins for Vivado-generated components, so the
// cocotb testbench runs on open-source simulators (Icarus/Verilator)
// without the Vivado IP / XPM simulation libraries.
//
// These files are NOT read by build_box_250mhz.tcl: synthesis uses the
// real Xilinx axis_register_slice IP (src/utility/vivado_ip/
// axi_stream_pipeline.tcl) and the real XPM macros.
//
// *************************************************************************
`timescale 1ns/1ps

// Behavioral model of the "axi_stream_pipeline" axis_register_slice IP
// (512-bit TDATA, 64-bit TKEEP, 48-bit TUSER). Two-deep skid buffer with
// full AXI-Stream handshake compliance.
module axi_stream_pipeline (
  input          s_axis_tvalid,
  input  [511:0] s_axis_tdata,
  input   [63:0] s_axis_tkeep,
  input          s_axis_tlast,
  input   [47:0] s_axis_tuser,
  output         s_axis_tready,

  output         m_axis_tvalid,
  output [511:0] m_axis_tdata,
  output  [63:0] m_axis_tkeep,
  output         m_axis_tlast,
  output  [47:0] m_axis_tuser,
  input          m_axis_tready,

  input          aclk,
  input          aresetn
);

  localparam W = 48 + 1 + 64 + 512;  // {tuser, tlast, tkeep, tdata}

  reg [W-1:0] buf0;  // output stage
  reg [W-1:0] buf1;  // skid stage
  reg v0, v1;

  wire [W-1:0] s_pay = {s_axis_tuser, s_axis_tlast, s_axis_tkeep, s_axis_tdata};

  assign s_axis_tready = !v1;
  assign m_axis_tvalid = v0;
  assign {m_axis_tuser, m_axis_tlast, m_axis_tkeep, m_axis_tdata} = buf0;

  always @(posedge aclk) begin
    if (!aresetn) begin
      v0 <= 1'b0;
      v1 <= 1'b0;
    end
    else begin
      if (!v0 || m_axis_tready) begin
        // Output stage drains (or is empty): refill from skid, then input
        if (v1) begin
          buf0 <= buf1;
          v0   <= 1'b1;
          v1   <= 1'b0;
        end
        else if (s_axis_tvalid && !v1) begin
          buf0 <= s_pay;
          v0   <= 1'b1;
        end
        else begin
          v0 <= 1'b0;
        end
      end
      else if (s_axis_tvalid && !v1) begin
        // Output stage stalled and full: catch the in-flight beat
        buf1 <= s_pay;
        v1   <= 1'b1;
      end
    end
  end

endmodule

// Behavioral model of the XPM single-bit CDC synchronizer used by
// generic_reset. Parameter names match the real XPM macro.
module xpm_cdc_single #(
  parameter int DEST_SYNC_FF   = 4,
  parameter int INIT_SYNC_FF   = 0,
  parameter int SIM_ASSERT_CHK = 0,
  parameter int SRC_INPUT_REG  = 1
) (
  input  src_clk,
  input  src_in,
  input  dest_clk,
  output dest_out
);

  reg                    src_q  = 1'b0;
  reg [DEST_SYNC_FF-1:0] sync_q = '0;

  always @(posedge src_clk) begin
    src_q <= src_in;
  end

  wire src_w = (SRC_INPUT_REG != 0) ? src_q : src_in;

  always @(posedge dest_clk) begin
    sync_q <= {sync_q[DEST_SYNC_FF-2:0], src_w};
  end

  assign dest_out = sync_q[DEST_SYNC_FF-1];

endmodule

// Behavioral model of the XPM bus-CDC handshake used by jc_regs, for the two
// quasi-static transfers: config down to the datapath, a counter snapshot
// back up. Parameter and port names match the real macro.
//
// The four-phase protocol is modelled honestly, because jc_regs depends on
// its shape rather than just on data arriving: src_send is held until src_rcv
// answers, dest_req pulses for exactly one dest_clk when dest_out is valid,
// and the reply toggles back. What is NOT modelled is metastability -- the
// data register is written in one domain and read in the other, which is safe
// here only because the protocol guarantees src_in is stable for the whole
// transfer. That is the same assumption the real macro makes of its user, so
// a design that violates it is wrong in hardware AND wrong here; this stub
// just will not be the thing that tells you.
//
// DEST_EXT_HSK = 0 only: jc_regs drives dest_ack low and takes the pulse.
module xpm_cdc_handshake #(
  parameter int DEST_EXT_HSK   = 1,
  parameter int DEST_SYNC_FF   = 4,
  parameter int INIT_SYNC_FF   = 0,
  parameter int SIM_ASSERT_CHK = 0,
  parameter int SRC_SYNC_FF    = 4,
  parameter int WIDTH          = 1
) (
  input        [WIDTH-1:0] src_in,
  input                    src_send,
  output                   src_rcv,
  input                    src_clk,
  input                    dest_clk,
  input                    dest_ack,
  output logic [WIDTH-1:0] dest_out,
  output logic             dest_req
);

  reg [WIDTH-1:0] payload = '0;
  reg             src_tog = 1'b0;    // flips once per transfer started
  reg             ack_tog = 1'b0;    // flips once per transfer taken

  reg [SRC_SYNC_FF-1:0]  ack_sync = '0;
  reg [DEST_SYNC_FF-1:0] req_sync = '0;
  reg                    req_seen = 1'b0;
  reg                    src_send_q = 1'b0;

  // Source: capture on the rising edge of src_send, then flip the request.
  always @(posedge src_clk) begin
    src_send_q <= src_send;
    if (src_send && !src_send_q) begin
      payload <= src_in;
      src_tog <= ~src_tog;
    end
    ack_sync <= {ack_sync[SRC_SYNC_FF-2:0], ack_tog};
  end

  // Both toggles start equal and each flips once per transfer, so they agree
  // again exactly when the round trip has completed.
  assign src_rcv = src_send && (ack_sync[SRC_SYNC_FF-1] == src_tog);

  // Destination: one dest_clk pulse when the synchronised request changes.
  always @(posedge dest_clk) begin
    req_sync <= {req_sync[DEST_SYNC_FF-2:0], src_tog};
    dest_req <= 1'b0;
    if (req_sync[DEST_SYNC_FF-1] != req_seen) begin
      req_seen <= req_sync[DEST_SYNC_FF-1];
      dest_out <= payload;
      dest_req <= 1'b1;
      ack_tog  <= ~ack_tog;
    end
  end

endmodule
