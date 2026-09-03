// *************************************************************************
//
// jc_evbuf -- two-slot event buffer between ingest and the clustering engine.
//
// ACCEPT OR DROP, NEVER BLOCK. pj_ready is tied high. Back-pressure here
// would propagate through jc_ingest into jc_deframe and out onto
// s_axis_tready, stalling the shell's whole RX path -- unacceptable for a
// device sitting in the network path. When there is nowhere to put an event
// it is consumed and counted instead, which is the correct behaviour for a
// trigger: process what you can, and report exactly how much you did not.
//
// drop_count against accept_count IS the duty cycle measurement. One engine
// clusters for tens of thousands of cycles per event while ingest fills a
// slot in ~128, so at any sustained input rate most events are dropped by
// design, and the ratio is what tells you how many engines step 11 needs.
//
// Ping-pong, not free-list: slots are claimed in strict alternation and
// released in the same order, so events reach the engine in arrival order.
// A first-free policy would reorder them whenever a slot freed early, and
// event_seq would stop being monotonic for no gain.
//
// An event is dropped as a WHOLE, never partially, on any of:
//   - no free slot when its first cell arrives
//   - any cell flagged pj_err by jc_ingest (index outside the legal grid, or
//     jc_deframe's abort cell, which is how a frame whose length disagrees
//     with its header count fails an event whose cells already left)
//   - more cells than NMAX, which jc_deframe should already have rejected
// A partially-stored event would cluster into plausible wrong jets, which is
// the one failure mode nothing downstream could detect.
//
// *************************************************************************
`include "jc_defs.vh"
`timescale 1ns/1ps
module jc_evbuf (
  // ---- Pseudojet stream from jc_ingest --------------------------------
  input                                 pj_valid,
  input  signed [`JC_P4_W-1:0]          pj_energy,
  input  signed [`JC_P4_W-1:0]          pj_px,
  input  signed [`JC_P4_W-1:0]          pj_py,
  input  signed [`JC_P4_W-1:0]          pj_pz,
  input  signed [`JC_RAP_W-1:0]         pj_rapidity,
  input         [`JC_PHI_W-1:0]         pj_phi,
  input  signed [`JC_WGT_W-1:0]         pj_beam_weight_log,
  input                                 pj_start,
  input                                 pj_last,
  input                                 pj_err,
  input                          [31:0] pj_event_seq,
  output                                pj_ready,

  // ---- Engine side ----------------------------------------------------
  // ev_valid means a complete event is waiting. The engine pulses ev_accept
  // to claim it, reads it out through ev_addr, then pulses ev_release. Only
  // one event is claimed at a time, so ev_count/ev_seq stay stable until it
  // is released.
  output                                ev_valid,
  output        [`JC_CNT_W-1:0]         ev_count,
  output                         [31:0] ev_seq,
  input                          [31:0] now,      // free-running, from the top
  output                         [31:0] ev_t0,    // when this event arrived
  input                                 ev_accept,

  input         [`JC_IDX_W-1:0]         ev_addr,
  output logic signed [`JC_P4_W-1:0]    ev_energy,
  output logic signed [`JC_P4_W-1:0]    ev_px,
  output logic signed [`JC_P4_W-1:0]    ev_py,
  output logic signed [`JC_P4_W-1:0]    ev_pz,
  output logic signed [`JC_RAP_W-1:0]   ev_rapidity,
  output logic        [`JC_PHI_W-1:0]   ev_phi,
  output logic signed [`JC_WGT_W-1:0]   ev_beam_weight_log,
  input                                 ev_release,

  // ---- Status, cumulative since reset ---------------------------------
  // drop_count is the total, and it is split because the two causes mean
  // OPPOSITE things. A full buffer is benign and expected -- it is the duty
  // cycle, and the ratio against accept_count is what sizes engine count. A
  // flagged cell is a data-integrity failure. Reported as one number, a
  // corrupt sender inflates the apparent duty cycle and step 10 replicates
  // engines to fix a problem that is not there.
  output logic                   [31:0] accept_count,
  output logic                   [31:0] drop_count,       // full + err
  output logic                   [31:0] drop_full_count,  // no free slot
  output logic                   [31:0] drop_err_count,   // bad cell, or aborted

  input                                 aclk,
  input                                 aresetn
);

  localparam int NMAX  = `JC_NMAX;
  localparam int IDX_W = `JC_IDX_W;
  localparam int CNT_W = `JC_CNT_W;

  // ---- Record packing --------------------------------------------------
  // One flat word per pseudojet so the two slots are a single memory. The
  // engine's jc_mem re-splits these -- four-momentum central, coordinates
  // lane-partitioned -- but that layout is its business, not the buffer's.
  localparam int OFF_ENERGY = 0;
  localparam int OFF_PX     = OFF_ENERGY + `JC_P4_W;    // 48
  localparam int OFF_PY     = OFF_PX     + `JC_P4_W;    // 96
  localparam int OFF_PZ     = OFF_PY     + `JC_P4_W;    // 144
  localparam int OFF_RAP    = OFF_PZ     + `JC_P4_W;    // 192
  localparam int OFF_PHI    = OFF_RAP    + `JC_RAP_W;   // 224
  localparam int OFF_WGT    = OFF_PHI    + `JC_PHI_W;   // 256
  localparam int REC_W      = OFF_WGT    + `JC_WGT_W;   // 288

  wire [REC_W-1:0] pj_record = {pj_beam_weight_log,
                                pj_phi,
                                pj_rapidity,
                                pj_pz, pj_py, pj_px, pj_energy};

  logic [REC_W-1:0] mem [0:2*NMAX-1];

  // ---- Slot bookkeeping ------------------------------------------------
  localparam logic [1:0] SL_EMPTY = 2'd0,   // nothing in it
                         SL_FILL  = 2'd1,   // ingest is writing
                         SL_READY = 2'd2,   // complete, waiting for an engine
                         SL_BUSY  = 2'd3;   // an engine is reading it

  logic [1:0]       slot_state [0:1];
  logic [CNT_W-1:0] slot_count [0:1];
  logic      [31:0] slot_seq   [0:1];
  // Arrival stamp, for the port-to-port measurement. PER SLOT, not global: a
  // second event can arrive while the first is still clustering, which is the
  // whole reason there are two slots.
  logic      [31:0] slot_t0    [0:1];

  logic             wr_ptr, rd_ptr;   // strict alternation, see header
  logic [IDX_W-1:0] wr_addr;
  logic             dropping;         // this event is being discarded
  logic             err_acc;          // any bad cell seen so far

  // ---- Current-event context ------------------------------------------
  // On the first cell the allocation decision is being made this cycle, so
  // the committed registers do not describe it yet -- these wires do.
  wire have_free = (slot_state[wr_ptr] == SL_EMPTY);

  wire                   cur_slot = wr_ptr;
  wire [IDX_W-1:0]       cur_addr = pj_start ? {IDX_W{1'b0}} : wr_addr;
  wire                   cur_drop = pj_start ? !have_free : dropping;

  // The 129th cell of an event cannot be stored: wr_addr would wrap and
  // overwrite cell 0. jc_deframe rejects count > NMAX so this is unreachable
  // through a well-formed frame, but the buffer must not corrupt an event on
  // the strength of an upstream guarantee.
  wire overflow = pj_valid && !cur_drop && !pj_last && (cur_addr == NMAX-1);

  wire cur_err  = (pj_start ? 1'b0 : err_acc) || pj_err || overflow;
  wire do_write = pj_valid && !cur_drop;

  // Never stall the RX path. See the header.
  assign pj_ready = 1'b1;

  // ---- Write side ------------------------------------------------------
  always_ff @(posedge aclk) begin
    if (do_write) mem[{cur_slot, cur_addr}] <= pj_record;
  end

  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      slot_state[0] <= SL_EMPTY;
      slot_state[1] <= SL_EMPTY;
      // Reset outright rather than relying on the write that precedes every
      // read: ev_t0 feeds a subtraction whose result reaches the output frame,
      // and an X there propagates silently -- it does not fail, it just makes
      // every beat unreadable.
      slot_t0[0]    <= 32'd0;
      slot_t0[1]    <= 32'd0;
      wr_ptr        <= 1'b0;
      rd_ptr        <= 1'b0;
      wr_addr       <= '0;
      dropping      <= 1'b0;
      err_acc       <= 1'b0;
      accept_count    <= 32'd0;
      drop_count      <= 32'd0;
      drop_full_count <= 32'd0;
      drop_err_count  <= 32'd0;
    end
    else begin
      if (pj_valid) begin
        if (pj_start) begin
          dropping <= !have_free;
          if (have_free) begin
            slot_state[wr_ptr] <= SL_FILL;
            // The first cell of the event: jc_deframe has just decoded the
            // header beat, so this is as close to "arrived" as the datapath
            // can see without reaching back into the CMAC.
            slot_t0[wr_ptr]    <= now;
          end
        end

        if (!cur_drop) begin
          wr_addr <= cur_addr + 1'b1;
          err_acc <= cur_err;
        end

        if (pj_last) begin
          dropping <= 1'b0;
          err_acc  <= 1'b0;
          if (cur_drop) begin
            // Never had a slot; nothing to release. Benign back-pressure.
            drop_count      <= drop_count + 32'd1;
            drop_full_count <= drop_full_count + 32'd1;
          end
          else if (cur_err) begin
            slot_state[cur_slot] <= SL_EMPTY;   // hand the slot straight back
            drop_count           <= drop_count + 32'd1;
            drop_err_count       <= drop_err_count + 32'd1;
          end
          else begin
            slot_state[cur_slot] <= SL_READY;
            slot_count[cur_slot] <= {1'b0, cur_addr} + 1'b1;
            slot_seq[cur_slot]   <= pj_event_seq;
            wr_ptr               <= ~wr_ptr;    // only a KEPT event advances
            accept_count         <= accept_count + 32'd1;
          end
        end
      end

      // ---- Read side ----------------------------------------------------
      if (ev_accept && ev_valid) slot_state[rd_ptr] <= SL_BUSY;

      if (ev_release) begin
        slot_state[rd_ptr] <= SL_EMPTY;
        rd_ptr             <= ~rd_ptr;
      end
    end
  end

  assign ev_valid = (slot_state[rd_ptr] == SL_READY);
  assign ev_count = slot_count[rd_ptr];
  assign ev_seq   = slot_seq[rd_ptr];
  assign ev_t0    = slot_t0[rd_ptr];

  // Registered read. rd_ptr only moves on release, so it is stable for the
  // whole time the engine holds the event.
  logic [REC_W-1:0] rd_record;
  always_ff @(posedge aclk) rd_record <= mem[{rd_ptr, ev_addr}];

  assign ev_energy          = rd_record[OFF_ENERGY +: `JC_P4_W];
  assign ev_px              = rd_record[OFF_PX     +: `JC_P4_W];
  assign ev_py              = rd_record[OFF_PY     +: `JC_P4_W];
  assign ev_pz              = rd_record[OFF_PZ     +: `JC_P4_W];
  assign ev_rapidity        = rd_record[OFF_RAP    +: `JC_RAP_W];
  assign ev_phi             = rd_record[OFF_PHI    +: `JC_PHI_W];
  assign ev_beam_weight_log = rd_record[OFF_WGT    +: `JC_WGT_W];

endmodule: jc_evbuf
