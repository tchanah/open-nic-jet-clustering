// *************************************************************************
//
// jc_deframe -- 512-bit AXI-Stream beats to a 1-cell-per-cycle stream.
//
// Beat 0 is the event header (see jc_defs.vh); beats 1.. carry 16 cells each.
// Because a cell is 4 bytes and a beat is 64, the split is fixed on every
// beat -- there is no rotation state, which a 3-byte cell would have needed.
//
// What still needs handling, and does:
//   - tkeep on the partial final beat. Cells are 4-byte aligned so tkeep is
//     a run of ones and cells_in_beat = popcount(tkeep)/4 exactly.
//   - tvalid bubbles. Beats are not guaranteed contiguous.
//   - back-pressure from downstream. One cell leaves per cycle while a beat
//     holds 16, so the input stalls 16 cycles per beat. Against thousands of
//     clustering cycles this is free.
//
// The header count is authoritative for how many cells an event has; padding
// beyond it in the final beat is ignored. Malformed events -- wrong
// ethertype, zero or over-NMAX count -- are rejected before a single cell is
// emitted, consumed to tlast and counted. Silent truncation would produce a
// wrong result that looks valid.
//
// A LENGTH ERROR IS FOUND MID-EVENT, so it cannot be handled that way. Two
// cases, and they are not symmetric:
//
//   - tlast arrives before the header count is satisfied. Cells have already
//     been emitted and cannot be recalled, so the event is FAILED instead:
//     ST_ABORT emits one more cell with cell_err set, jc_ingest folds that
//     into its own range check and jc_evbuf drops the whole event and hands
//     the slot back. Without this, s_axis_tready would stay high while the
//     module waits for cells that will never come, and the next frame's
//     HEADER beat would be loaded and emitted as sixteen cells -- two frames
//     silently merged into one plausible wrong event.
//
//   - the count runs out before tlast. The event is passed on, because the
//     cells emitted are exactly the ones the header promised and nothing is
//     partial; the surplus beats are flushed and the frame is counted bad.
//     Failing this one would need lookahead we do not have -- tlast arrives
//     after the final cell has already left.
//
// Either way bad_event_count moves, which is the signal that a sender's
// framing disagrees with its own header.
//
// *************************************************************************
`include "jc_defs.vh"
`timescale 1ns/1ps
module jc_deframe (
  input                s_axis_tvalid,
  input        [511:0] s_axis_tdata,
  input         [63:0] s_axis_tkeep,
  input                s_axis_tlast,
  output logic         s_axis_tready,

  // One cell per cycle. cell_start marks the first cell of an event,
  // cell_last the final one; both assert together for a 1-cell event.
  // Named cell_data, not cell: `cell` is a Verilog-2001 reserved word from
  // the config/liblist syntax, and Icarus enforces it under -g2012.
  output logic [`JC_CELL_W-1:0] cell_data,
  output logic             cell_valid,
  output logic             cell_start,
  output logic             cell_last,
  // This event is already unsalvageable -- see the length-error note above.
  // Rides the same handshake as a cell so the failure reaches jc_evbuf
  // through the drop path that already exists, rather than a second channel.
  output logic             cell_err,
  input                    cell_ready,

  // Latched from the header, stable while an event streams.
  // event_seq is the join key that pairs emitted jets with the event that
  // produced them: jets leave asynchronously and events can be dropped
  // downstream, so arrival order cannot be relied on.
  output logic [`JC_CNT_W-1:0] event_cell_count,
  output logic      [31:0] event_seq,
  // Cells above threshold before truncation. Greater than event_cell_count
  // means the host cut this event to the NMAX highest-pt cells, so a FastJet
  // comparison is only valid against the same NMAX -- and the difference
  // says how much was lost, which is what tells you whether NMAX is right.
  output logic      [15:0] event_cells_total,

  // Cumulative since reset. Split for the same reason jc_evbuf's drops are:
  // a bad header means the sender is speaking the wrong protocol entirely --
  // wrong ethertype, wrong format version, an impossible count -- while a
  // length error means a sender we understand whose framing disagrees with
  // its own header. At bring-up those point at completely different things,
  // and the total alone cannot tell them apart.
  output logic      [31:0] bad_event_count,   // header + length
  output logic      [31:0] bad_header_count,
  output logic      [31:0] bad_length_count,

  input                    aclk,
  input                    aresetn
);

  // Mirror the macros the body uses, so only the port list carries `JC_.
  localparam int NMAX             = `JC_NMAX;
  localparam int CNT_W            = `JC_CNT_W;
  localparam int HDR_OFF_ETHTYPE  = `JC_HDR_OFF_ETHTYPE;
  localparam int HDR_OFF_VERSION  = `JC_HDR_OFF_VERSION;
  localparam int HDR_OFF_COUNT    = `JC_HDR_OFF_COUNT;
  localparam int HDR_OFF_SEQ      = `JC_HDR_OFF_SEQ;
  localparam int HDR_OFF_TOTAL    = `JC_HDR_OFF_TOTAL;
  localparam logic [15:0] ETH_TYPE    = `JC_ETH_TYPE;
  localparam logic  [7:0] FMT_VERSION = `JC_FMT_VERSION;

  typedef enum logic [1:0] {
    ST_HEADER,   // waiting for beat 0
    ST_CELLS,    // streaming cells out of held beats
    ST_ABORT,    // length error mid-event: emit one cell_err and fail it
    ST_FLUSH     // malformed: consume to tlast, emit nothing
  } state_e;

  state_e state;

  // ---- Header decode, combinational on beat 0 --------------------------
  // Fields are big-endian on the wire; byte n of a beat is tdata[8*n +: 8].
  wire [15:0] hdr_ethtype = {s_axis_tdata[8*(HDR_OFF_ETHTYPE  ) +: 8],
                             s_axis_tdata[8*(HDR_OFF_ETHTYPE+1) +: 8]};
  wire  [7:0] hdr_version =  s_axis_tdata[8*(HDR_OFF_VERSION  ) +: 8];
  wire [15:0] hdr_count   = {s_axis_tdata[8*(HDR_OFF_COUNT    ) +: 8],
                             s_axis_tdata[8*(HDR_OFF_COUNT  +1) +: 8]};
  wire [31:0] hdr_seq     = {s_axis_tdata[8*(HDR_OFF_SEQ      ) +: 8],
                             s_axis_tdata[8*(HDR_OFF_SEQ    +1) +: 8],
                             s_axis_tdata[8*(HDR_OFF_SEQ    +2) +: 8],
                             s_axis_tdata[8*(HDR_OFF_SEQ    +3) +: 8]};
  wire [15:0] hdr_total   = {s_axis_tdata[8*(HDR_OFF_TOTAL    ) +: 8],
                             s_axis_tdata[8*(HDR_OFF_TOTAL  +1) +: 8]};

  // A version mismatch means the cell encoding or the grid differs from what
  // the LUTs assume. Rejecting is the entire point of carrying the field --
  // the alternative is misreading every cell and producing plausible jets.
  wire hdr_ok = (hdr_ethtype == ETH_TYPE)
             && (hdr_version == FMT_VERSION)
             && (hdr_count != 16'd0)
             && (hdr_count <= NMAX[15:0])
             && !s_axis_tlast;          // a header-only frame carries no cells

  // ---- Held cell beat --------------------------------------------------
  logic [511:0]     beat_data;
  logic             beat_loaded;     // beat_data holds cells not yet emitted
  logic [3:0]       cell_idx;        // 0..15 within the held beat
  logic [3:0]       cell_idx_last;   // index of the last valid cell in it
  logic             beat_was_last;   // held beat carried tlast
  logic [CNT_W-1:0] cells_left;      // still to emit for this event
  logic             abort_flush;     // after the abort cell, still drain to tlast

  // Cells present in a beat, from tkeep. Cells are 4-byte aligned, so tkeep
  // is a run of ones and each cell contributes exactly 4 set bits.
  function automatic [4:0] cells_from_keep (input [63:0] keep);
    int unsigned n;
    begin
      n = 0;
      for (int b = 0; b < 64; b++) if (keep[b]) n++;
      cells_from_keep = n[6:2];      // popcount / 4
    end
  endfunction

  wire [4:0] beat_cells = cells_from_keep(s_axis_tkeep);

  // ---- Handshakes ------------------------------------------------------
  // Two sources of cell_valid, kept apart: real cells out of a held beat, and
  // the single synthetic cell ST_ABORT emits to fail an event. Only the first
  // advances cells_left, so every counter below is qualified on cells_valid.
  wire cells_valid = (state == ST_CELLS) && beat_loaded;
  wire abort_valid = (state == ST_ABORT);

  assign cell_data  = beat_data[32*cell_idx +: 32];
  assign cell_valid = cells_valid || abort_valid;
  assign cell_start = cells_valid && (cells_left == event_cell_count);
  assign cell_last  = abort_valid
                   || (cells_valid && (cells_left == {{(CNT_W-1){1'b0}}, 1'b1}));
  // The abort cell's data is whatever the stale beat holds; jc_evbuf drops
  // the event on the flag without ever looking at it.
  assign cell_err   = abort_valid;

  wire cell_fire   = cells_valid && cell_ready;
  wire abort_fire  = abort_valid && cell_ready;
  wire beat_retire = cell_fire && (cell_idx == cell_idx_last);

  // The event's final cell is leaving this cycle, so the next beat on the bus
  // belongs to the next frame and must be decoded as a header, not consumed
  // here. Depends on cell_ready, not on s_axis_tready, so there is no loop.
  wire event_done = cell_fire && (cells_left == {{(CNT_W-1){1'b0}}, 1'b1});

  // The held beat carried tlast and has just run dry with cells still owed.
  // The frame is over, so the next beat is a header -- same reason as
  // event_done for dropping tready, and the reason this must be detected at
  // all rather than left to a count that will never be reached.
  wire frame_short = beat_retire && beat_was_last && !event_done;

  // Nothing of this event has left yet, so there is nothing to fail -- the
  // frame can be rejected outright the way a bad header is. cell_fire counts:
  // cells_left does not fall until the next cycle, so the first cell of an
  // event is still leaving while this compare would otherwise read as none.
  wire emitted_none = (cells_left == event_cell_count) && !cell_fire;

  always_comb begin
    unique case (state)
      ST_HEADER: s_axis_tready = 1'b1;                    // always take a header
      // When the final cell of the held beat retires the next beat may land
      // in the same cycle -- unless the event ends here, in which case
      // accepting it would swallow the next frame's header beat.
      ST_CELLS:  s_axis_tready = (!beat_loaded || beat_retire)
                                 && !event_done && !frame_short;
      ST_ABORT:  s_axis_tready = 1'b0;                    // next beat is a header
      ST_FLUSH:  s_axis_tready = 1'b1;                    // drain to tlast
      default:   s_axis_tready = 1'b1;
    endcase
  end

  wire beat_fire = s_axis_tvalid && s_axis_tready;

  // A beat with no valid bytes carries no cells: cell_idx_last would
  // underflow to 15 and sixteen words of it would be emitted as cells.
  wire beat_empty = beat_fire && (beat_cells == 5'd0);

  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      state            <= ST_HEADER;
      beat_loaded      <= 1'b0;
      cell_idx         <= 4'd0;
      cell_idx_last    <= 4'd0;
      beat_was_last    <= 1'b0;
      cells_left       <= '0;
      abort_flush      <= 1'b0;
      event_cell_count <= '0;
      event_seq         <= 32'd0;
      event_cells_total <= 16'd0;
      bad_event_count   <= 32'd0;
      bad_header_count  <= 32'd0;
      bad_length_count  <= 32'd0;
    end
    else begin
      unique case (state)

        // ---- beat 0: header ------------------------------------------
        ST_HEADER: begin
          if (beat_fire) begin
            if (hdr_ok) begin
              event_cell_count <= hdr_count[CNT_W-1:0];
              event_seq         <= hdr_seq;
              event_cells_total <= hdr_total;
              cells_left       <= hdr_count[CNT_W-1:0];
              beat_loaded      <= 1'b0;      // no cell beat held yet
              state            <= ST_CELLS;
            end
            else begin
              bad_event_count  <= bad_event_count + 32'd1;
              bad_header_count <= bad_header_count + 32'd1;
              // A single-beat malformed frame is already complete.
              state <= s_axis_tlast ? ST_HEADER : ST_FLUSH;
            end
          end
        end

        // ---- beats 1..: cells ----------------------------------------
        ST_CELLS: begin
          if (cell_fire) begin
            cells_left <= cells_left - 1'b1;
            cell_idx   <= cell_idx + 1'b1;
            if (beat_retire) beat_loaded <= 1'b0;
          end

          // Loading a beat takes precedence: when a beat retires and the
          // next arrives in the same cycle, the new one is held. This cannot
          // collide with event_done -- s_axis_tready is low on that cycle.
          if (beat_fire) begin
            beat_data     <= s_axis_tdata;
            cell_idx_last <= beat_cells[3:0] - 4'd1;
            beat_was_last <= s_axis_tlast;
            cell_idx      <= 4'd0;
            beat_loaded   <= 1'b1;
          end

          // Length errors first: both of these mean the count and the frame
          // disagree, and neither can reach event_done on its own.
          if (beat_empty) begin
            beat_loaded <= 1'b0;
            if (emitted_none) begin
              // Nothing escaped -- reject it outright, like a bad header.
              // Counted as a LENGTH error even so: the header was fine, the
              // frame just ran out of beats.
              bad_event_count  <= bad_event_count + 32'd1;
              bad_length_count <= bad_length_count + 32'd1;
              state <= s_axis_tlast ? ST_HEADER : ST_FLUSH;
            end
            else begin
              abort_flush <= !s_axis_tlast;
              state       <= ST_ABORT;
            end
          end
          else if (frame_short) begin
            beat_loaded <= 1'b0;
            abort_flush <= 1'b0;              // tlast was on the beat just drained
            state       <= ST_ABORT;
          end
          // Event complete when the final cell leaves.
          else if (event_done) begin
            beat_loaded <= 1'b0;
            if (beat_was_last) begin
              state <= ST_HEADER;             // clean end
            end
            else begin
              bad_event_count  <= bad_event_count + 32'd1;
              bad_length_count <= bad_length_count + 32'd1;
              state <= ST_FLUSH;              // count ran out before tlast
            end
          end
        end

        // ---- length error: fail the event downstream ------------------
        // One cell with cell_err, then the frame's own tail if any. The
        // count moves here, not at the trigger, so an aborted event is
        // counted exactly once however it got here.
        ST_ABORT: begin
          if (abort_fire) begin
            bad_event_count  <= bad_event_count + 32'd1;
            bad_length_count <= bad_length_count + 32'd1;
            state <= abort_flush ? ST_FLUSH : ST_HEADER;
          end
        end

        // ---- malformed: consume to tlast -----------------------------
        ST_FLUSH: begin
          beat_loaded <= 1'b0;
          if (beat_fire && s_axis_tlast) state <= ST_HEADER;
        end

        default: state <= ST_HEADER;
      endcase
    end
  end

endmodule: jc_deframe
