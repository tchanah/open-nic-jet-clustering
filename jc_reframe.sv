// *************************************************************************
//
// jc_reframe -- jets to an output frame, and the end of the datapath.
//
// Buffers an event's jets as they trickle out of jc_engine, then emits one
// frame when the event completes. See jc_defs.vh for the wire format.
//
// BUFFER, THEN SEND -- because tuser_size must be right on the FIRST beat.
// The shell wants the byte count up front and the count is 64 + 16*njets, so
// the last jet has to be known before the header leaves. That rules out
// streaming beats as they fill. It costs nothing: jets arrive tens of cycles
// apart while a frame drains in at most 33 beats.
//
// AN EVENT WITH NO JETS EMITS NOTHING. That is the trigger behaviour asked
// for -- silence when nothing fires -- and it is why `suppressed` exists.
// Without that counter, "the trigger works and this sample is soft" and "the
// trigger is broken" look identical from outside the card, and on this dataset
// only 3 events in 400 clear 50 GeV, so silence is the NORMAL case. The
// counter is the only thing that distinguishes them.
//
// SINGLE BUFFER, AND THE BACK-PRESSURE IS WHAT MAKES IT SAFE. jet_ready falls
// for the whole time a frame is streaming, so the next event's jets cannot
// land in a buffer still being read. That is not a timing argument -- it holds
// even if an event finished instantly. Its consequence is worth stating: a
// jet_eoe arriving mid-stream necessarily belongs to an event that produced no
// jets, because any jet would have stalled jc_ctrl in S_EMIT long before
// S_FINISH. So that case is a suppressed event, not a lost frame.
//
// The store is FOUR BANKS BY POSITION IN A BEAT, and that is a synthesis
// result rather than a preference -- the first version cost 57k LUTs. See the
// bank declaration below for what happened and why this shape fixes it.
//
// Four jets fill a beat exactly, which is the same property the input side
// gets from a 4-byte cell: jet index IS word index, no rotation state and no
// offset term.
//
// *************************************************************************
`include "jc_defs.vh"
`timescale 1ns/1ps
module jc_reframe (
  // ---- Jets from jc_engine ---------------------------------------------
  // jet_eoe is a bare one-cycle pulse, NOT qualified by jet_ready, and it
  // fires for every completed event including one with no jets. It must
  // always be accepted; there is no handshake to stall it with.
  input                             jet_valid,
  input  signed [`JC_P4_W-1:0]      jet_e,
  input  signed [`JC_P4_W-1:0]      jet_px,
  input  signed [`JC_P4_W-1:0]      jet_py,
  input  signed [`JC_P4_W-1:0]      jet_pz,
  input                      [31:0] jet_seq,
  input                             jet_eoe,
  output                            jet_ready,

  // ---- Sampled into the header when an event completes ------------------
  // Live values, latched at jet_eoe. cycle_count is the engine's measurement
  // of the event just finished -- the step 9 number, per event, free.
  input                      [31:0] ev_cycles,
  input                      [31:0] cnt_drop_full,
  input                      [31:0] cnt_drop_err,
  input                      [31:0] cnt_bad_frame,

  // ---- Frame identity ---------------------------------------------------
  // From the register file. Cosmetic on the QDMA C2H path, load-bearing the
  // moment jets are steered out a CMAC port instead.
  input                      [47:0] cfg_dst_mac,
  input                      [47:0] cfg_src_mac,
  input                      [15:0] cfg_tuser_src,

  // ---- AXI-Stream out ---------------------------------------------------
  output logic                      m_axis_tvalid,
  output logic              [511:0] m_axis_tdata,
  output logic               [63:0] m_axis_tkeep,
  output logic                      m_axis_tlast,
  output logic               [47:0] m_axis_tuser,
  input                             m_axis_tready,

  // ---- Status, cumulative since reset -----------------------------------
  output logic               [31:0] frames_out,
  output logic               [31:0] jets_out,
  output logic               [31:0] suppressed,
  output logic                      busy,

  input                             aclk,
  input                             aresetn
);

  localparam int NMAX  = `JC_NMAX;
  localparam int IDX_W = `JC_IDX_W;                 // 7
  localparam int CNT_W = `JC_CNT_W;                 // 8
  localparam int JPB   = `JC_JETS_PER_BEAT;         // 4
  localparam int BEATS = NMAX / JPB;                // 32
  localparam int BIDX_W = 5;                        // $clog2(BEATS)

  localparam logic [15:0] ETH_TYPE    = `JC_JET_ETH_TYPE;
  localparam logic  [7:0] FMT_VERSION = `JC_JET_FMT_VERSION;

  // ---- fp32 conversion, on the way in ----------------------------------
  // Converted once per jet at jet-arrival rate rather than once per beat on
  // the way out, so the egress path is a pure mux and the converters run at
  // one conversion per hundred-odd cycles.
  //
  // ONE CYCLE, not combinational: jc_ctrl's jet register through the whole
  // conversion into a bank write was 4.291 ns, the last path under 250 MHz.
  // The bank writes below are delayed to match.
  wire [31:0] fp_e, fp_px, fp_py, fp_pz;
  jc_fp32 u_fp_e  (.fx (jet_e),  .fp (fp_e),  .aclk (aclk));
  jc_fp32 u_fp_px (.fx (jet_px), .fp (fp_px), .aclk (aclk));
  jc_fp32 u_fp_py (.fx (jet_py), .fp (fp_py), .aclk (aclk));
  jc_fp32 u_fp_pz (.fx (jet_pz), .fp (fp_pz), .aclk (aclk));

  // Big-endian on the wire, so every multi-byte field is byte-reversed going
  // into tdata: byte n of a beat is tdata[8*n +: 8].
  function automatic [31:0] bswap32 (input [31:0] x);
    bswap32 = {x[7:0], x[15:8], x[23:16], x[31:24]};
  endfunction

  function automatic [15:0] bswap16 (input [15:0] x);
    bswap16 = {x[7:0], x[15:8]};
  endfunction

  // A MAC is transmitted most significant byte first, same as every other
  // field here.
  function automatic [47:0] bswap48 (input [47:0] x);
    bswap48 = {x[7:0], x[15:8], x[23:16], x[31:24], x[39:32], x[47:40]};
  endfunction

  // One jet, 16 B: E at bytes 0..3, then px, py, pz.
  wire [127:0] jet_word = {bswap32(fp_pz), bswap32(fp_py),
                           bswap32(fp_px), bswap32(fp_e)};

  // ---- Jet store, four banks by lane -----------------------------------
  // ONE BANK PER POSITION IN A BEAT, not one memory of 512-bit beats. Both
  // hold the same 16 kbit; the difference is what they synthesise to, and it
  // is not small.
  //
  // The single-memory version assembled each beat in a 512-bit accumulator
  // and wrote it in two places -- a full group during a jet, and a partial
  // tail at end of event. Two writes to one array in one process is exactly
  // what blocks RAM inference: Vivado reported "RAM beat_mem_reg dissolved
  // into registers" and produced 16,384 flops behind three 16384-bit muxes,
  // 57k LUTs for a buffer, against 7.2k for the whole sixteen-lane sweep.
  //
  // Banking by lane gives each array exactly one write port and no
  // accumulator: a jet goes straight to bank wp[1:0] at row wp>>2, and a beat
  // is four parallel reads concatenated. The partial-tail flush disappears
  // with it, because there is no group to be half-assembled -- which also
  // removes the trickiest piece of control in the module.
  //
  // Same reasoning as jc_mem's cyclic banking, and the same payoff: lane
  // index is a wire split, so there is no crossbar and no arbitration.
  //
  // A tail beat leaves stale jets from a previous event in the banks it does
  // not fill. They never leave the card: tkeep marks them invalid and
  // tuser_size excludes them, and both the packet adapter and QDMA honour
  // that. Zeroing them would cost the accumulator back.
  logic [127:0]      bank0 [0:BEATS-1];
  logic [127:0]      bank1 [0:BEATS-1];
  logic [127:0]      bank2 [0:BEATS-1];
  logic [127:0]      bank3 [0:BEATS-1];

  logic [CNT_W-1:0]  wp;                // jets buffered for the current event
  logic [CNT_W-1:0]  njets;             // jets in the frame being sent
  logic [31:0]       seq_r, cyc_r, dropf_r, drope_r, bad_r;
  logic [BIDX_W-1:0] beat_idx;

  typedef enum logic [1:0] {
    ST_FILL,      // taking jets
    ST_HDR,       // driving beat 0
    ST_JETS       // driving the jet beats
  } state_e;

  state_e state;

  wire accept = jet_valid && jet_ready;
  wire [1:0] lane = wp[1:0];
  wire [BIDX_W-1:0] wr_row = wp[IDX_W-1:2];

  // jc_ctrl raises jet_eoe the cycle AFTER the last jet is taken, so these
  // never coincide. Counted correctly anyway rather than assumed: a bench
  // standing in for jc_ctrl is exactly where a wrong belief about that would
  // hide, and the cost is one adder. The jet itself needs no special case --
  // its bank write happens on this edge regardless.
  wire [CNT_W-1:0] wp_final = accept ? wp + 1'b1 : wp;

  assign jet_ready = (state == ST_FILL) && (wp < NMAX[CNT_W-1:0]);
  assign busy      = (state != ST_FILL);

  // ---- Frame geometry ---------------------------------------------------
  wire [BIDX_W-1:0] last_beat = (njets - 1'b1) >> 2;
  wire              on_last   = (beat_idx == last_beat);
  wire        [2:0] jets_here = on_last
                              ? ((njets[1:0] == 2'd0) ? 3'd4 : {1'b0, njets[1:0]})
                              : 3'd4;

  function automatic [63:0] keep_for (input [2:0] n);
    case (n)
      3'd1:    keep_for = 64'h0000_0000_0000_FFFF;
      3'd2:    keep_for = 64'h0000_0000_FFFF_FFFF;
      3'd3:    keep_for = 64'h0000_FFFF_FFFF_FFFF;
      default: keep_for = 64'hFFFF_FFFF_FFFF_FFFF;
    endcase
  endfunction

  // ---- Header beat ------------------------------------------------------
  // Byte positions spelled out rather than implied by declaration order, the
  // same way jc_deframe decodes the input header.
  logic [511:0] hdr;
  always_comb begin
    hdr = 512'd0;
    hdr[8*0 +: 48] = bswap48(cfg_dst_mac);
    hdr[8*6 +: 48] = bswap48(cfg_src_mac);
    // Whole-word byte swaps rather than four constant part-selects each.
    // Same bytes, and it keeps Icarus quiet: it cannot narrow a constant
    // select inside always_* for sensitivity ("all bits will be included"),
    // so twenty of those became twenty warnings.
    hdr[8*`JC_JHDR_OFF_ETHTYPE   +: 16] = bswap16(ETH_TYPE);
    hdr[8*`JC_JHDR_OFF_VERSION   +:  8] = FMT_VERSION;
    hdr[8*`JC_JHDR_OFF_NJETS     +: 16] = bswap16({8'd0, njets});
    hdr[8*`JC_JHDR_OFF_SEQ       +: 32] = bswap32(seq_r);
    hdr[8*`JC_JHDR_OFF_CYCLES    +: 32] = bswap32(cyc_r);
    hdr[8*`JC_JHDR_OFF_DROP_FULL +: 32] = bswap32(dropf_r);
    hdr[8*`JC_JHDR_OFF_DROP_ERR  +: 32] = bswap32(drope_r);
    hdr[8*`JC_JHDR_OFF_BAD_FRAME +: 32] = bswap32(bad_r);
  end

  // ---- Egress -----------------------------------------------------------
  // tvalid never depends on tready, so the handshake cannot deadlock.
  wire [15:0] frame_bytes = 16'd64 + ({8'd0, njets} << 4);

  always_comb begin
    m_axis_tvalid = (state != ST_FILL);
    m_axis_tuser  = {16'd0, cfg_tuser_src, frame_bytes};
    if (state == ST_HDR) begin
      m_axis_tdata = hdr;
      m_axis_tkeep = 64'hFFFF_FFFF_FFFF_FFFF;
      m_axis_tlast = 1'b0;                    // njets >= 1, so a jet beat follows
    end
    else begin
      // Four parallel reads, lane 0 in the low bytes so jet index is word
      // index on every beat.
      m_axis_tdata = {bank3[beat_idx], bank2[beat_idx],
                      bank1[beat_idx], bank0[beat_idx]};
      m_axis_tkeep = keep_for(jets_here);
      m_axis_tlast = on_last;
    end
  end

  wire beat_fire = m_axis_tvalid && m_axis_tready;

  // ---- Write controls, one cycle behind the handshake -------------------
  // jc_fp32 is registered, so jet_word is the conversion of the jet accepted
  // LAST cycle. Only the bank write moves; wp, jets_out and jet_ready still
  // follow `accept` directly, so nothing about the handshake or the counting
  // changes.
  //
  // The last jet of an event is still safely stored before it can be read.
  // jc_ctrl raises jet_eoe the cycle after that jet is taken, so the delayed
  // write lands on the same edge that moves the state to ST_HDR; the header
  // beat is driven for a full cycle before ST_JETS reads a bank. One cycle of
  // margin, and it does not depend on m_axis_tready.
  logic              wr_en_d;
  logic        [1:0] lane_d;
  logic [BIDX_W-1:0] wr_row_d;

  always_ff @(posedge aclk) begin
    if (!aresetn) wr_en_d <= 1'b0;
    else begin
      wr_en_d  <= accept;
      lane_d   <= lane;
      wr_row_d <= wr_row;
    end
  end

  // The store. ONE write statement per bank, each in its own process, which
  // is what makes these infer as memory rather than as flops.
  //
  // Writing lane 0 also CLEARS the other three at that row, so a tail beat's
  // unused lanes read back as zero instead of as jets from a previous event.
  // Worth a mux for two reasons: those bytes sit outside tkeep and never reach
  // the wire, but they would still be whatever the RAM last held -- and in
  // simulation an unwritten bank is X, which would force every bench
  // downstream of here to mask tdata before it could read it.
  always_ff @(posedge aclk)
    if (wr_en_d && (lane_d == 2'd0))
      bank0[wr_row_d] <= jet_word;

  always_ff @(posedge aclk)
    if (wr_en_d && (lane_d == 2'd1 || lane_d == 2'd0))
      bank1[wr_row_d] <= (lane_d == 2'd1) ? jet_word : 128'd0;

  always_ff @(posedge aclk)
    if (wr_en_d && (lane_d == 2'd2 || lane_d == 2'd0))
      bank2[wr_row_d] <= (lane_d == 2'd2) ? jet_word : 128'd0;

  always_ff @(posedge aclk)
    if (wr_en_d && (lane_d == 2'd3 || lane_d == 2'd0))
      bank3[wr_row_d] <= (lane_d == 2'd3) ? jet_word : 128'd0;

  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      state      <= ST_FILL;
      wp         <= '0;
      njets      <= '0;
      beat_idx   <= '0;
      frames_out <= 32'd0;
      jets_out   <= 32'd0;
      suppressed <= 32'd0;
    end
    else begin
      // ---- Taking jets --------------------------------------------------
      if (accept) begin
        wp       <= wp + 1'b1;
        jets_out <= jets_out + 32'd1;
      end

      // ---- End of event -------------------------------------------------
      if (jet_eoe) begin
        if (wp_final != '0) begin
          njets   <= wp_final;
          seq_r   <= jet_seq;
          cyc_r   <= ev_cycles;
          dropf_r <= cnt_drop_full;
          drope_r <= cnt_drop_err;
          bad_r   <= cnt_bad_frame;
          // No flush: every jet reaches its bank on its own, one cycle behind
          // the handshake that took it. The last one lands on this very edge,
          // and the header beat below buys a full cycle before any bank is
          // read -- see the write-control block for why that is enough.
          wp       <= '0;
          beat_idx <= '0;
          state    <= ST_HDR;
        end
        else begin
          // No jet cleared the floor. See the header: mid-stream this is the
          // only thing jet_eoe can mean.
          suppressed <= suppressed + 32'd1;
        end
      end

      // ---- Draining -----------------------------------------------------
      if (beat_fire) begin
        if (state == ST_HDR) begin
          beat_idx <= '0;
          state    <= ST_JETS;
        end
        else if (on_last) begin
          frames_out <= frames_out + 32'd1;
          state      <= ST_FILL;
        end
        else begin
          beat_idx <= beat_idx + 1'b1;
        end
      end
    end
  end

endmodule: jc_reframe
