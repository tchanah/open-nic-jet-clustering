// *************************************************************************
//
// Copyright 2020 Xilinx, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// *************************************************************************
`include "open_nic_shell_macros.vh"
`timescale 1ns/1ps
module p2p_250mhz #(
  parameter int NUM_QDMA = 1,
  parameter int NUM_INTF = 1
) (
  input        [NUM_INTF*2-1:0] s_axil_awvalid,
  input     [32*NUM_INTF*2-1:0] s_axil_awaddr,
  output       [NUM_INTF*2-1:0] s_axil_awready,
  input        [NUM_INTF*2-1:0] s_axil_wvalid,
  input     [32*NUM_INTF*2-1:0] s_axil_wdata,
  output       [NUM_INTF*2-1:0] s_axil_wready,
  output       [NUM_INTF*2-1:0] s_axil_bvalid,
  output     [2*NUM_INTF*2-1:0] s_axil_bresp,
  input        [NUM_INTF*2-1:0] s_axil_bready,
  input        [NUM_INTF*2-1:0] s_axil_arvalid,
  input     [32*NUM_INTF*2-1:0] s_axil_araddr,
  output       [NUM_INTF*2-1:0] s_axil_arready,
  output       [NUM_INTF*2-1:0] s_axil_rvalid,
  output    [32*NUM_INTF*2-1:0] s_axil_rdata,
  output     [2*NUM_INTF*2-1:0] s_axil_rresp,
  input        [NUM_INTF*2-1:0] s_axil_rready,

  input      [NUM_INTF*NUM_QDMA-1:0] s_axis_qdma_h2c_tvalid,
  input  [512*NUM_INTF*NUM_QDMA-1:0] s_axis_qdma_h2c_tdata,
  input   [64*NUM_INTF*NUM_QDMA-1:0] s_axis_qdma_h2c_tkeep,
  input      [NUM_INTF*NUM_QDMA-1:0] s_axis_qdma_h2c_tlast,
  input   [16*NUM_INTF*NUM_QDMA-1:0] s_axis_qdma_h2c_tuser_size,
  input   [16*NUM_INTF*NUM_QDMA-1:0] s_axis_qdma_h2c_tuser_src,
  input   [16*NUM_INTF*NUM_QDMA-1:0] s_axis_qdma_h2c_tuser_dst,
  output     [NUM_INTF*NUM_QDMA-1:0] s_axis_qdma_h2c_tready,

  output     [NUM_INTF*NUM_QDMA-1:0] m_axis_qdma_c2h_tvalid,
  output [512*NUM_INTF*NUM_QDMA-1:0] m_axis_qdma_c2h_tdata,
  output  [64*NUM_INTF*NUM_QDMA-1:0] m_axis_qdma_c2h_tkeep,
  output     [NUM_INTF*NUM_QDMA-1:0] m_axis_qdma_c2h_tlast,
  output  [16*NUM_INTF*NUM_QDMA-1:0] m_axis_qdma_c2h_tuser_size,
  output  [16*NUM_INTF*NUM_QDMA-1:0] m_axis_qdma_c2h_tuser_src,
  output  [16*NUM_INTF*NUM_QDMA-1:0] m_axis_qdma_c2h_tuser_dst,
  input      [NUM_INTF*NUM_QDMA-1:0] m_axis_qdma_c2h_tready,

  output     [NUM_INTF-1:0] m_axis_adap_tx_250mhz_tvalid,
  output [512*NUM_INTF-1:0] m_axis_adap_tx_250mhz_tdata,
  output  [64*NUM_INTF-1:0] m_axis_adap_tx_250mhz_tkeep,
  output     [NUM_INTF-1:0] m_axis_adap_tx_250mhz_tlast,
  output  [16*NUM_INTF-1:0] m_axis_adap_tx_250mhz_tuser_size,
  output  [16*NUM_INTF-1:0] m_axis_adap_tx_250mhz_tuser_src,
  output  [16*NUM_INTF-1:0] m_axis_adap_tx_250mhz_tuser_dst,
  input      [NUM_INTF-1:0] m_axis_adap_tx_250mhz_tready,

  input      [NUM_INTF-1:0] s_axis_adap_rx_250mhz_tvalid,
  input  [512*NUM_INTF-1:0] s_axis_adap_rx_250mhz_tdata,
  input   [64*NUM_INTF-1:0] s_axis_adap_rx_250mhz_tkeep,
  input      [NUM_INTF-1:0] s_axis_adap_rx_250mhz_tlast,
  input   [16*NUM_INTF-1:0] s_axis_adap_rx_250mhz_tuser_size,
  input   [16*NUM_INTF-1:0] s_axis_adap_rx_250mhz_tuser_src,
  input   [16*NUM_INTF-1:0] s_axis_adap_rx_250mhz_tuser_dst,
  output     [NUM_INTF-1:0] s_axis_adap_rx_250mhz_tready,

  input                     mod_rstn,
  output                    mod_rst_done,

  input                     axil_aclk,

`ifdef __au55n__
  input                     ref_clk_100mhz,
`elsif __au55c__
  input                     ref_clk_100mhz,
`elsif __au50__
  input                     ref_clk_100mhz,
`elsif __au280__
  input                     ref_clk_100mhz,
`endif
  input                     axis_aclk
);

  wire axil_aresetn;

  // Reset is clocked by the 125MHz AXI-Lite clock
  generic_reset #(
    .NUM_INPUT_CLK  (1),
    .RESET_DURATION (100)
  ) reset_inst (
    .mod_rstn     (mod_rstn),
    .mod_rst_done (mod_rst_done),
    .clk          (axil_aclk),
    .rstn         (axil_aresetn)
  );

  // The box routes TWO AXI-Lite channels per interface -- 0x0000 "ingress"
  // and 0x0080 "egress" (box_250mhz_address_map.v). Stock p2p connected a
  // single sink to the whole vector, so only channel 0 was answered and
  // channel 1's *ready outputs were left undriven: a host read at 0x0080
  // would hang the bus waiting for arready. Both are driven below, inside the
  // per-interface loop -- channel 0 by jet_clustering's register file, and
  // channel 1 by the stock sink, since nothing needs it yet.

  // Everything below is the STOCK multi-QDMA path, untouched. The tuser
  // plumbing lives inside the branch rather than at loop level because the
  // bump-in-wire block further down drives the same adap_tx tuser signals --
  // at loop level they would be a second driver on every port.
  generate for (genvar i = 0; i < NUM_INTF; i++) begin
    if (NUM_QDMA > 1) begin
      wire          [16*3-1:0] axis_adap_tx_250mhz_tuser;
      wire          [16*3-1:0] axis_adap_rx_250mhz_tuser;
      wire          [16*3*NUM_QDMA-1:0] axis_qdma_c2h_tuser;

      assign axis_adap_rx_250mhz_tuser[0+:16]                 = s_axis_adap_rx_250mhz_tuser_size[`getvec(16, i)];
      assign axis_adap_rx_250mhz_tuser[16+:16]                = s_axis_adap_rx_250mhz_tuser_src[`getvec(16, i)];
      assign axis_adap_rx_250mhz_tuser[32+:16]                = s_axis_adap_rx_250mhz_tuser_dst[`getvec(16, i)];

      assign m_axis_adap_tx_250mhz_tuser_size[`getvec(16, i)] = axis_adap_tx_250mhz_tuser[0+:16];
      assign m_axis_adap_tx_250mhz_tuser_src[`getvec(16, i)]  = axis_adap_tx_250mhz_tuser[16+:16];
      assign m_axis_adap_tx_250mhz_tuser_dst[`getvec(16, i)]  = 16'h1 << (6 + i);

      wire      [NUM_QDMA-1:0] axis_qdma_h2c_tvalid;
      wire  [512*NUM_QDMA-1:0] axis_qdma_h2c_tdata;
      wire   [64*NUM_QDMA-1:0] axis_qdma_h2c_tkeep;
      wire      [NUM_QDMA-1:0] axis_qdma_h2c_tlast;
      wire [16*3*NUM_QDMA-1:0] axis_qdma_h2c_tuser;
      wire      [NUM_QDMA-1:0] axis_qdma_h2c_tready;

      wire      [NUM_QDMA-1:0] axis_qdma_c2h_tvalid;
      wire  [512*NUM_QDMA-1:0] axis_qdma_c2h_tdata;
      wire   [64*NUM_QDMA-1:0] axis_qdma_c2h_tkeep;
      wire      [NUM_QDMA-1:0] axis_qdma_c2h_tlast;
      wire      [NUM_QDMA-1:0] axis_qdma_c2h_tready;

      for (genvar ii = 0; ii < NUM_QDMA; ii++) begin
        assign axis_qdma_h2c_tvalid[ii]                 = s_axis_qdma_h2c_tvalid[2*ii+i];
        assign axis_qdma_h2c_tdata[`getvec(512, ii)]    = s_axis_qdma_h2c_tdata[`getvec(512, 2*ii+i)];
        assign axis_qdma_h2c_tkeep[`getvec(64, ii)]     = s_axis_qdma_h2c_tkeep[`getvec(64, 2*ii+i)];
        assign axis_qdma_h2c_tlast[ii]                  = s_axis_qdma_h2c_tlast[2*ii+i];
        assign axis_qdma_h2c_tuser[`getvec(16, 3*ii)]   = s_axis_qdma_h2c_tuser_size[`getvec(16, 2*ii+i)];
        assign axis_qdma_h2c_tuser[`getvec(16, 3*ii+1)] = s_axis_qdma_h2c_tuser_src[`getvec(16, 2*ii+i)];
        assign axis_qdma_h2c_tuser[`getvec(16, 3*ii+2)] = s_axis_qdma_h2c_tuser_dst[`getvec(16, 2*ii+i)];
        assign s_axis_qdma_h2c_tready[2*ii+i]           = axis_qdma_h2c_tready[ii];

        assign m_axis_qdma_c2h_tvalid[2*ii+i]                  = axis_qdma_c2h_tvalid[ii];
        assign m_axis_qdma_c2h_tdata[`getvec(512, 2*ii+i)]     = axis_qdma_c2h_tdata[`getvec(512, ii)];
        assign m_axis_qdma_c2h_tkeep[`getvec(64, 2*ii+i)]      = axis_qdma_c2h_tkeep[`getvec(64, ii)];
        assign m_axis_qdma_c2h_tlast[2*ii+i]                   = axis_qdma_c2h_tlast[ii];
        assign m_axis_qdma_c2h_tuser_size[`getvec(16, 2*ii+i)] = axis_qdma_c2h_tuser[`getvec(16, 3*ii)];
        assign m_axis_qdma_c2h_tuser_src[`getvec(16, 2*ii+i)]  = axis_qdma_c2h_tuser[`getvec(16, 3*ii+1)];
        assign m_axis_qdma_c2h_tuser_dst[`getvec(16, 2*ii+i)]  = axis_qdma_c2h_tuser[`getvec(16, 3*ii+2)];
        assign axis_qdma_c2h_tready[ii]                        = m_axis_qdma_c2h_tready[2*ii+i];
      end

      box_250mhz_egress_axi_switch box_250mhz_egress_axi_switch_inst (
        .aclk                (axis_aclk),
        .aresetn             (axil_aresetn),
        .s_axis_tvalid       (axis_qdma_h2c_tvalid),
        .s_axis_tready       (axis_qdma_h2c_tready),
        .s_axis_tdata        (axis_qdma_h2c_tdata),
        .s_axis_tkeep        (axis_qdma_h2c_tkeep),
        .s_axis_tlast        (axis_qdma_h2c_tlast),
        .s_axis_tuser        (axis_qdma_h2c_tuser),
        .m_axis_tvalid       (m_axis_adap_tx_250mhz_tvalid[i]),
        .m_axis_tready       (m_axis_adap_tx_250mhz_tready[i]),
        .m_axis_tdata        (m_axis_adap_tx_250mhz_tdata[`getvec(512, i)]),
        .m_axis_tkeep        (m_axis_adap_tx_250mhz_tkeep[`getvec(64, i)]),
        .m_axis_tlast        (m_axis_adap_tx_250mhz_tlast[i]),
        .m_axis_tuser        (axis_adap_tx_250mhz_tuser),
        .s_axi_ctrl_aclk     (axil_aclk),
        .s_axi_ctrl_aresetn  (axil_aresetn),
        .s_axi_ctrl_awvalid  (s_axil_awvalid[2*i+1]),
        .s_axi_ctrl_awready  (s_axil_awready[2*i+1]),
        .s_axi_ctrl_awaddr   (s_axil_awaddr[`getvec(32, 2*i+1)]),
        .s_axi_ctrl_wvalid   (s_axil_wvalid[2*i+1]),
        .s_axi_ctrl_wready   (s_axil_wready[2*i+1]),
        .s_axi_ctrl_wdata    (s_axil_wdata[`getvec(32, 2*i+1)]),
        .s_axi_ctrl_bvalid   (s_axil_bvalid[2*i+1]),
        .s_axi_ctrl_bready   (s_axil_bready[2*i+1]),
        .s_axi_ctrl_bresp    (s_axil_bresp[`getvec(2, 2*i+1)]),
        .s_axi_ctrl_arvalid  (s_axil_arvalid[2*i+1]),
        .s_axi_ctrl_arready  (s_axil_arready[2*i+1]),
        .s_axi_ctrl_araddr   (s_axil_araddr[`getvec(32, 2*i+1)]),
        .s_axi_ctrl_rvalid   (s_axil_rvalid[2*i+1]),
        .s_axi_ctrl_rready   (s_axil_rready[2*i+1]),
        .s_axi_ctrl_rdata    (s_axil_rdata[`getvec(32, 2*i+1)]),
        .s_axi_ctrl_rresp    (s_axil_rresp[`getvec(2, 2*i+1)])
      );

      box_250mhz_ingress_axi_switch box_250mhz_ingress_axi_switch_inst (
        .aclk                (axis_aclk),
        .aresetn             (axil_aresetn),
        .s_axis_tvalid       (s_axis_adap_rx_250mhz_tvalid[i]),
        .s_axis_tready       (s_axis_adap_rx_250mhz_tready[i]),
        .s_axis_tdata        (s_axis_adap_rx_250mhz_tdata[`getvec(512, i)]),
        .s_axis_tkeep        (s_axis_adap_rx_250mhz_tkeep[`getvec(64, i)]),
        .s_axis_tlast        (s_axis_adap_rx_250mhz_tlast[i]),
        .s_axis_tuser        (axis_adap_rx_250mhz_tuser),
        .m_axis_tvalid       (axis_qdma_c2h_tvalid),
        .m_axis_tready       (axis_qdma_c2h_tready),
        .m_axis_tdata        (axis_qdma_c2h_tdata),
        .m_axis_tkeep        (axis_qdma_c2h_tkeep),
        .m_axis_tlast        (axis_qdma_c2h_tlast),
        .m_axis_tuser        (axis_qdma_c2h_tuser),
        .s_axi_ctrl_aclk     (axil_aclk),
        .s_axi_ctrl_aresetn  (axil_aresetn),
        .s_axi_ctrl_awvalid  (s_axil_awvalid[2*i]),
        .s_axi_ctrl_awready  (s_axil_awready[2*i]),
        .s_axi_ctrl_awaddr   (s_axil_awaddr[`getvec(32, 2*i)]),
        .s_axi_ctrl_wvalid   (s_axil_wvalid[2*i]),
        .s_axi_ctrl_wready   (s_axil_wready[2*i]),
        .s_axi_ctrl_wdata    (s_axil_wdata[`getvec(32, 2*i)]),
        .s_axi_ctrl_bvalid   (s_axil_bvalid[2*i]),
        .s_axi_ctrl_bready   (s_axil_bready[2*i]),
        .s_axi_ctrl_bresp    (s_axil_bresp[`getvec(2, 2*i)]),
        .s_axi_ctrl_arvalid  (s_axil_arvalid[2*i]),
        .s_axi_ctrl_arready  (s_axil_arready[2*i]),
        .s_axi_ctrl_araddr   (s_axil_araddr[`getvec(32, 2*i)]),
        .s_axi_ctrl_rvalid   (s_axil_rvalid[2*i]),
        .s_axi_ctrl_rready   (s_axil_rready[2*i]),
        .s_axi_ctrl_rdata    (s_axil_rdata[`getvec(32, 2*i)]),
        .s_axi_ctrl_rresp    (s_axil_rresp[`getvec(2, 2*i)])
      );
    end
  end
  endgenerate

  // =====================================================================
  // BUMP IN THE WIRE -- cells in one port, jets out the other
  // =====================================================================
  //   CMAC port 0 RX  ->  jet_clustering  ->  CMAC port 1 TX
  //
  // The same shape the graph plugin proved on this card and host. Cell
  // events arrive from the network and jets go back out to the network, to
  // be captured on a third machine -- NOT up to the local host over QDMA
  // C2H, which is where the stock p2p RX path would have sent them and
  // where the step-2 scaffold inherited its wiring from.
  //
  // ONE ENGINE FOR THE CARD, not one per interface. Instantiating inside
  // the per-interface loop would have built a second complete clustering
  // engine -- another sixteen lanes, another 64 DSPs, another set of ingest
  // tables -- for a port that has nothing to cluster. It also keeps the
  // "one shared jc_ingest for the device" property CLAUDE.md claims.
  //
  // The destination MAC is an AXI-Lite register, not a parameter: the graph
  // plugin hard-coded its sink card at synthesis time and had to rebuild to
  // move it. It defaults to broadcast, which any capture host on the switch
  // will see; set it to a directed unicast once the sink is chosen.
  localparam int JET_RX_PORT = 0;
  localparam int JET_TX_PORT = 1;

  generate if (NUM_QDMA <= 1) begin : g_bump_in_wire
    if (NUM_INTF < 2) begin : g_need_two_ports
      // A one-port build has nowhere to send jets. Fail loudly at
      // elaboration rather than quietly tying the egress off.
      // One string, not two: Verilog does not concatenate adjacent literals.
      initial $fatal(1, "jet_clustering needs NUM_INTF >= 2 -- cells arrive on CMAC port 0, jets leave on CMAC port 1");
    end

    wire [16*3-1:0] axis_jets_tuser;
    wire [16*3-1:0] axis_cells_tuser;

    assign axis_cells_tuser[0+:16]  = s_axis_adap_rx_250mhz_tuser_size[`getvec(16, JET_RX_PORT)];
    assign axis_cells_tuser[16+:16] = s_axis_adap_rx_250mhz_tuser_src[`getvec(16, JET_RX_PORT)];
    assign axis_cells_tuser[32+:16] = s_axis_adap_rx_250mhz_tuser_dst[`getvec(16, JET_RX_PORT)];

    assign m_axis_adap_tx_250mhz_tuser_size[`getvec(16, JET_TX_PORT)] = axis_jets_tuser[0+:16];
    assign m_axis_adap_tx_250mhz_tuser_src [`getvec(16, JET_TX_PORT)] = axis_jets_tuser[16+:16];
    assign m_axis_adap_tx_250mhz_tuser_dst [`getvec(16, JET_TX_PORT)] = 16'h1 << (6 + JET_TX_PORT);

    jet_clustering jet_clustering_inst (
      // AXI-Lite channel 0 -- the "ingress" window, base 0x0000 in the box map.
      .s_axil_awvalid (s_axil_awvalid[0]),
      .s_axil_awaddr  (s_axil_awaddr[`getvec(32, 0)]),
      .s_axil_awready (s_axil_awready[0]),
      .s_axil_wvalid  (s_axil_wvalid[0]),
      .s_axil_wdata   (s_axil_wdata[`getvec(32, 0)]),
      .s_axil_wready  (s_axil_wready[0]),
      .s_axil_bvalid  (s_axil_bvalid[0]),
      .s_axil_bresp   (s_axil_bresp[`getvec(2, 0)]),
      .s_axil_bready  (s_axil_bready[0]),
      .s_axil_arvalid (s_axil_arvalid[0]),
      .s_axil_araddr  (s_axil_araddr[`getvec(32, 0)]),
      .s_axil_arready (s_axil_arready[0]),
      .s_axil_rvalid  (s_axil_rvalid[0]),
      .s_axil_rdata   (s_axil_rdata[`getvec(32, 0)]),
      .s_axil_rresp   (s_axil_rresp[`getvec(2, 0)]),
      .s_axil_rready  (s_axil_rready[0]),

      .s_axis_tvalid (s_axis_adap_rx_250mhz_tvalid[JET_RX_PORT]),
      .s_axis_tdata  (s_axis_adap_rx_250mhz_tdata[`getvec(512, JET_RX_PORT)]),
      .s_axis_tkeep  (s_axis_adap_rx_250mhz_tkeep[`getvec(64, JET_RX_PORT)]),
      .s_axis_tlast  (s_axis_adap_rx_250mhz_tlast[JET_RX_PORT]),
      .s_axis_tuser  (axis_cells_tuser),
      .s_axis_tready (s_axis_adap_rx_250mhz_tready[JET_RX_PORT]),

      .m_axis_tvalid (m_axis_adap_tx_250mhz_tvalid[JET_TX_PORT]),
      .m_axis_tdata  (m_axis_adap_tx_250mhz_tdata[`getvec(512, JET_TX_PORT)]),
      .m_axis_tkeep  (m_axis_adap_tx_250mhz_tkeep[`getvec(64, JET_TX_PORT)]),
      .m_axis_tlast  (m_axis_adap_tx_250mhz_tlast[JET_TX_PORT]),
      .m_axis_tuser  (axis_jets_tuser),
      .m_axis_tready (m_axis_adap_tx_250mhz_tready[JET_TX_PORT]),

      .axil_aclk     (axil_aclk),
      .axil_aresetn  (axil_aresetn),
      .aclk          (axis_aclk),
      .aresetn       (axil_aresetn)
    );

    // ---- Host TX still reaches the wire on the free port ----------------
    // H2C on the jets port would collide with the jets themselves, so only
    // the RX port's TX side keeps its pass-through. That is what lets the
    // host inject its own frames out CMAC port 0 for bring-up.
    wire [16*3-1:0] axis_host_tx_tuser;
    wire     [47:0] axis_qdma_h2c_tuser;

    assign axis_qdma_h2c_tuser[0+:16]  = s_axis_qdma_h2c_tuser_size[`getvec(16, JET_RX_PORT)];
    assign axis_qdma_h2c_tuser[16+:16] = s_axis_qdma_h2c_tuser_src[`getvec(16, JET_RX_PORT)];
    assign axis_qdma_h2c_tuser[32+:16] = s_axis_qdma_h2c_tuser_dst[`getvec(16, JET_RX_PORT)];

    assign m_axis_adap_tx_250mhz_tuser_size[`getvec(16, JET_RX_PORT)] = axis_host_tx_tuser[0+:16];
    assign m_axis_adap_tx_250mhz_tuser_src [`getvec(16, JET_RX_PORT)] = axis_host_tx_tuser[16+:16];
    assign m_axis_adap_tx_250mhz_tuser_dst [`getvec(16, JET_RX_PORT)] = 16'h1 << (6 + JET_RX_PORT);

    axi_stream_pipeline host_tx_ppl_inst (
      .s_axis_tvalid (s_axis_qdma_h2c_tvalid[JET_RX_PORT]),
      .s_axis_tdata  (s_axis_qdma_h2c_tdata[`getvec(512, JET_RX_PORT)]),
      .s_axis_tkeep  (s_axis_qdma_h2c_tkeep[`getvec(64, JET_RX_PORT)]),
      .s_axis_tlast  (s_axis_qdma_h2c_tlast[JET_RX_PORT]),
      .s_axis_tuser  (axis_qdma_h2c_tuser),
      .s_axis_tready (s_axis_qdma_h2c_tready[JET_RX_PORT]),

      .m_axis_tvalid (m_axis_adap_tx_250mhz_tvalid[JET_RX_PORT]),
      .m_axis_tdata  (m_axis_adap_tx_250mhz_tdata[`getvec(512, JET_RX_PORT)]),
      .m_axis_tkeep  (m_axis_adap_tx_250mhz_tkeep[`getvec(64, JET_RX_PORT)]),
      .m_axis_tlast  (m_axis_adap_tx_250mhz_tlast[JET_RX_PORT]),
      .m_axis_tuser  (axis_host_tx_tuser),
      .m_axis_tready (m_axis_adap_tx_250mhz_tready[JET_RX_PORT]),

      .aclk          (axis_aclk),
      .aresetn       (axil_aresetn)
    );

    // ---- Nothing goes up to the host ------------------------------------
    assign m_axis_qdma_c2h_tvalid     = {NUM_INTF*NUM_QDMA{1'b0}};
    assign m_axis_qdma_c2h_tdata      = {512*NUM_INTF*NUM_QDMA{1'b0}};
    assign m_axis_qdma_c2h_tkeep      = {64*NUM_INTF*NUM_QDMA{1'b0}};
    assign m_axis_qdma_c2h_tlast      = {NUM_INTF*NUM_QDMA{1'b0}};
    assign m_axis_qdma_c2h_tuser_size = {16*NUM_INTF*NUM_QDMA{1'b0}};
    assign m_axis_qdma_c2h_tuser_src  = {16*NUM_INTF*NUM_QDMA{1'b0}};
    assign m_axis_qdma_c2h_tuser_dst  = {16*NUM_INTF*NUM_QDMA{1'b0}};

    // ---- Unused ingress: drain, never stall -----------------------------
    // An undriven tready would back-pressure the adapter and eventually the
    // CMAC, which a device in the network path must never do.
    assign s_axis_adap_rx_250mhz_tready[JET_TX_PORT] = 1'b1;
    assign s_axis_qdma_h2c_tready[JET_TX_PORT]       = 1'b1;

    // ---- The remaining AXI-Lite channels must still answer --------------
    // Channel 0 is the register file above; 1..2*NUM_INTF-1 are unused but a
    // read of any of them would hang the crossbar if nothing drove *ready.
    for (genvar c = 1; c < NUM_INTF*2; c++) begin : g_axil_sink
      axi_lite_slave #(
        .REG_ADDR_W (12),
        .REG_PREFIX (16'hB000)
      ) reg_sink (
        .s_axil_awvalid (s_axil_awvalid[c]),
        .s_axil_awaddr  (s_axil_awaddr[`getvec(32, c)]),
        .s_axil_awready (s_axil_awready[c]),
        .s_axil_wvalid  (s_axil_wvalid[c]),
        .s_axil_wdata   (s_axil_wdata[`getvec(32, c)]),
        .s_axil_wready  (s_axil_wready[c]),
        .s_axil_bvalid  (s_axil_bvalid[c]),
        .s_axil_bresp   (s_axil_bresp[`getvec(2, c)]),
        .s_axil_bready  (s_axil_bready[c]),
        .s_axil_arvalid (s_axil_arvalid[c]),
        .s_axil_araddr  (s_axil_araddr[`getvec(32, c)]),
        .s_axil_arready (s_axil_arready[c]),
        .s_axil_rvalid  (s_axil_rvalid[c]),
        .s_axil_rdata   (s_axil_rdata[`getvec(32, c)]),
        .s_axil_rresp   (s_axil_rresp[`getvec(2, c)]),
        .s_axil_rready  (s_axil_rready[c]),

        .aclk           (axil_aclk),
        .aresetn        (axil_aresetn)
      );
    end
  end
  endgenerate

endmodule: p2p_250mhz
