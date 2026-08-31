// *************************************************************************
//
// jc_mem -- the active list: 128 entries, split by how they are accessed.
//
// The split is the whole point. Three groups, three access patterns:
//
//   four-momentum   read/written ONCE PER MERGE, one entry at a time, so it
//                   stays central and 192 bits wide. Nothing sweeps it.
//   coordinates     y, phi, beam weight -- read by every lane on every sweep
//                   cycle, so BANKED cyclically by LANES.
//   nn state        active, beam, nn index, nn_geo, nn_dist_log -- same
//                   access pattern as coordinates, banked the same way, but
//                   written far more often, so kept in its own array rather
//                   than forcing a read-modify-write of a 194-bit word.
//
// Cyclic banking with LANES a power of two makes the address split free:
// entry k lives in bank k[3:0] at offset k[6:4]. Lane l therefore addresses
// ONLY bank l, on both the read and the write-back, so the sweep needs no
// crossbar and the write-back cannot collide between lanes.
//
// Reads are registered -- one cycle from rd_off to the data. jc_sweep must
// present wb_off aligned to the beat whose data is arriving back.
//
// Single-entry writes (init, setkin, kill, the scanned row's own result)
// take priority over the lane write-back. They belong to different phases of
// a round, so the controller must not overlap them; the priority is a
// definition rather than an expectation.
//
// *************************************************************************
`include "jc_defs.vh"
`timescale 1ns/1ps
module jc_mem (
  // ---- Initialise one entry from jc_evbuf ------------------------------
  // Also clears the nn state to "active, no neighbour yet", so the initial
  // all-pairs scan starts from a state every real neighbour beats.
  input                          init_en,
  input  [`JC_IDX_W-1:0]         init_idx,
  input  signed [`JC_P4_W-1:0]   init_e,
  input  signed [`JC_P4_W-1:0]   init_px,
  input  signed [`JC_P4_W-1:0]   init_py,
  input  signed [`JC_P4_W-1:0]   init_pz,
  input  signed [`JC_COORD_W-1:0] init_y,
  input         [`JC_COORD_W-1:0] init_phi,
  input  signed [`JC_WGT_W-1:0]  init_wgt,

  // ---- Coordinate update after a merge (from jc_setkin) ----------------
  input                          set_en,
  input  [`JC_IDX_W-1:0]         set_idx,
  input  signed [`JC_COORD_W-1:0] set_y,
  input         [`JC_COORD_W-1:0] set_phi,
  input  signed [`JC_WGT_W-1:0]  set_wgt,

  // ---- Four-momentum, one entry at a time ------------------------------
  input  [`JC_IDX_W-1:0]         p4_rd_idx,
  output signed [`JC_P4_W-1:0]   p4_rd_e,
  output signed [`JC_P4_W-1:0]   p4_rd_px,
  output signed [`JC_P4_W-1:0]   p4_rd_py,
  output signed [`JC_P4_W-1:0]   p4_rd_pz,
  input                          p4_wr_en,
  input  [`JC_IDX_W-1:0]         p4_wr_idx,
  input  signed [`JC_P4_W-1:0]   p4_wr_e,
  input  signed [`JC_P4_W-1:0]   p4_wr_px,
  input  signed [`JC_P4_W-1:0]   p4_wr_py,
  input  signed [`JC_P4_W-1:0]   p4_wr_pz,

  // ---- Deactivate one entry (merged away, or emitted as a jet) ---------
  input                          kill_en,
  input  [`JC_IDX_W-1:0]         kill_idx,

  // ---- Write one entry's own nn result (the scanned row) ---------------
  input                          nn_wr_en,
  input  [`JC_IDX_W-1:0]         nn_wr_idx,
  input                          nn_wr_beam,
  input  [`JC_IDX_W-1:0]         nn_wr_index,
  input  [`JC_GEO_W-1:0]         nn_wr_geo,
  input  signed [`JC_NNLOG_W-1:0] nn_wr_log,

  // ---- Lane-parallel read, one offset across all banks -----------------
  input  [`JC_OFF_W-1:0]                 rd_off,
  output [`JC_LANES*`JC_COORD_W-1:0]     rd_y,
  output [`JC_LANES*`JC_COORD_W-1:0]     rd_phi,
  output [`JC_LANES*`JC_WGT_W-1:0]       rd_wgt,
  output [`JC_LANES-1:0]                 rd_active,
  output [`JC_LANES-1:0]                 rd_beam,
  output [`JC_LANES*`JC_IDX_W-1:0]       rd_nn_index,
  output [`JC_LANES*`JC_GEO_W-1:0]       rd_nn_geo,
  output [`JC_LANES*`JC_NNLOG_W-1:0]     rd_nn_log,

  // ---- Refresh one entry's nn_dist_log ---------------------------------
  // Separate from the write-back on purpose. Claiming a row means storing
  // w_k + log2(g), and doing that in the lanes would put a logarithm in each
  // of them -- roughly 16 DSPs and several thousand LUTs per engine, for
  // something needed a handful of times per merge. The write-back therefore
  // leaves nn_dist_log stale and jc_sweep reports which rows it claimed; one
  // shared log unit walks that list through this port.
  input                          log_wr_en,
  input  [`JC_IDX_W-1:0]         log_wr_idx,
  input  signed [`JC_NNLOG_W-1:0] log_wr_val,

  // ---- Lane-parallel nn write-back -------------------------------------
  // Lane l writes bank l only, so these never conflict with each other.
  input  [`JC_OFF_W-1:0]                 wb_off,
  input  [`JC_LANES-1:0]                 wb_en,
  input  [`JC_LANES*`JC_IDX_W-1:0]       wb_nn_index,
  input  [`JC_LANES*`JC_GEO_W-1:0]       wb_nn_geo,

  // Every entry's active bit at once. jc_ctrl needs it to know when the list
  // is empty, and it is free: active lives in a flat register, not in the
  // banked record.
  output [`JC_NMAX-1:0]                  active_mask,

  input                                  aclk,
  input                                  aresetn
);

  localparam int NMAX    = `JC_NMAX;
  localparam int LANES   = `JC_LANES;
  localparam int DEPTH   = `JC_DEPTH;
  localparam int IDX_W   = `JC_IDX_W;
  localparam int LANE_W  = `JC_LANE_W;
  localparam int OFF_W   = `JC_OFF_W;
  localparam int COORD_W = `JC_COORD_W;
  localparam int WGT_W   = `JC_WGT_W;
  localparam int GEO_W   = `JC_GEO_W;
  localparam int NNLOG_W = `JC_NNLOG_W;
  localparam int P4_W    = `JC_P4_W;

  // ---- Record layouts --------------------------------------------------
  localparam int COORD_REC_W = 3 * COORD_W;             // y, phi, wgt
  localparam int C_Y   = 0;
  localparam int C_PHI = COORD_W;
  localparam int C_WGT = 2 * COORD_W;

  localparam int NN_LOG  = 0;
  localparam int NN_GEO  = NNLOG_W;
  localparam int NN_IDX  = NN_GEO + GEO_W;
  localparam int NN_BEAM = NN_IDX + IDX_W;
  localparam int NN_REC_W = NN_BEAM + 1;

  localparam int P4_REC_W = 4 * P4_W;

  // "No neighbour yet": nn_geo all ones and nn_log at its most positive, so
  // the first real candidate wins both the write-back compare and the argmin.
  // The initial all-pairs scan depends on this.
  localparam logic [NN_REC_W-1:0] NN_INIT =
      {1'b1, {IDX_W{1'b0}}, {GEO_W{1'b1}}, {1'b0, {(NNLOG_W-1){1'b1}}}};

  // ---- Storage ---------------------------------------------------------
  logic [COORD_REC_W-1:0] coord_bank [0:LANES-1][0:DEPTH-1];
  logic   [NN_REC_W-1:0]  nn_bank    [0:LANES-1][0:DEPTH-1];
  logic   [P4_REC_W-1:0]  p4mem      [0:NMAX-1];

  // Active is deliberately NOT in the banked record. One bit per entry in a
  // flat register resets in a single statement, which is what guarantees that
  // entries the controller never loaded read back inactive rather than X --
  // and it slices lane-parallel for free, because entry off*LANES+l means the
  // LANES bits starting at off*LANES are exactly that offset's lanes.
  logic [NMAX-1:0] active_vec;
  assign active_mask = active_vec;

  always_ff @(posedge aclk) begin
    if (!aresetn)        active_vec <= '0;
    else if (init_en)    active_vec[init_idx] <= 1'b1;
    else if (kill_en)    active_vec[kill_idx] <= 1'b0;
  end

  // ---- Address split, free because LANES is a power of two -------------
  wire [LANE_W-1:0] init_bank  = init_idx [LANE_W-1:0];
  wire  [OFF_W-1:0] init_off   = init_idx [IDX_W-1:LANE_W];
  wire [LANE_W-1:0] set_bank   = set_idx  [LANE_W-1:0];
  wire  [OFF_W-1:0] set_off    = set_idx  [IDX_W-1:LANE_W];
  wire [LANE_W-1:0] kill_bank  = kill_idx [LANE_W-1:0];
  wire  [OFF_W-1:0] kill_off   = kill_idx [IDX_W-1:LANE_W];
  wire [LANE_W-1:0] nnwr_bank  = nn_wr_idx [LANE_W-1:0];
  wire  [OFF_W-1:0] nnwr_off   = nn_wr_idx [IDX_W-1:LANE_W];
  wire [LANE_W-1:0] logwr_bank = log_wr_idx[LANE_W-1:0];
  wire  [OFF_W-1:0] logwr_off  = log_wr_idx[IDX_W-1:LANE_W];

  wire [COORD_REC_W-1:0] init_coord = {init_wgt, init_phi, init_y};
  wire [COORD_REC_W-1:0] set_coord  = {set_wgt,  set_phi,  set_y};
  wire   [NN_REC_W-1:0]  nnwr_rec   = {nn_wr_beam, nn_wr_index,
                                       nn_wr_geo, nn_wr_log};

  // ---- Four-momentum ---------------------------------------------------
  // Registered read, matching the banked arrays so jc_ctrl sees one latency.
  logic [P4_REC_W-1:0] p4_rd_rec;
  always_ff @(posedge aclk) p4_rd_rec <= p4mem[p4_rd_idx];

  always_ff @(posedge aclk) begin
    if (init_en)
      p4mem[init_idx] <= {init_pz, init_py, init_px, init_e};
    else if (p4_wr_en)
      p4mem[p4_wr_idx] <= {p4_wr_pz, p4_wr_py, p4_wr_px, p4_wr_e};
  end

  assign p4_rd_e  = p4_rd_rec[0      +: P4_W];
  assign p4_rd_px = p4_rd_rec[P4_W   +: P4_W];
  assign p4_rd_py = p4_rd_rec[2*P4_W +: P4_W];
  assign p4_rd_pz = p4_rd_rec[3*P4_W +: P4_W];

  // ---- Banked arrays ---------------------------------------------------
  logic [LANES*COORD_REC_W-1:0] coord_rd_r;
  logic [LANES*NN_REC_W-1:0]    nn_rd_r;

  // Registered to the same latency as the banked reads. One part-select
  // rather than sixteen bit-selects, because entry off*LANES+l puts an
  // offset's lanes in contiguous bits.
  logic [LANES-1:0] active_rd_r;
  always_ff @(posedge aclk) begin
    if (!aresetn) active_rd_r <= '0;
    else          active_rd_r <= active_vec[rd_off * LANES +: LANES];
  end
  assign rd_active = active_rd_r;

  genvar l;
  generate
    for (l = 0; l < LANES; l = l + 1) begin : g_lane
      // Coordinates: written only at init and after a merge.
      always_ff @(posedge aclk) begin
        if (init_en && init_bank == l)
          coord_bank[l][init_off] <= init_coord;
        else if (set_en && set_bank == l)
          coord_bank[l][set_off] <= set_coord;
      end

      // nn state: single-entry writes win over the lane write-back.
      always_ff @(posedge aclk) begin
        if (init_en && init_bank == l)
          nn_bank[l][init_off] <= NN_INIT;
        else if (nn_wr_en && nnwr_bank == l)
          nn_bank[l][nnwr_off] <= nnwr_rec;
        else if (log_wr_en && logwr_bank == l)
          nn_bank[l][logwr_off][NN_LOG +: NNLOG_W] <= log_wr_val;
        else if (wb_en[l])
          // Index and distance move; beam clears because a claimed row has a
          // neighbour by definition, and nn_dist_log is deliberately left
          // stale for the shared log unit.
          nn_bank[l][wb_off] <= {1'b0,
                                 wb_nn_index[l*IDX_W +: IDX_W],
                                 wb_nn_geo  [l*GEO_W +: GEO_W],
                                 nn_bank[l][wb_off][NN_LOG +: NNLOG_W]};
      end

      always_ff @(posedge aclk) begin
        coord_rd_r[l*COORD_REC_W +: COORD_REC_W] <= coord_bank[l][rd_off];
        nn_rd_r   [l*NN_REC_W    +: NN_REC_W]    <= nn_bank   [l][rd_off];
      end

      // Unpack the registered reads onto the flat output buses.
      assign rd_y      [l*COORD_W +: COORD_W] =
             coord_rd_r[l*COORD_REC_W + C_Y   +: COORD_W];
      assign rd_phi    [l*COORD_W +: COORD_W] =
             coord_rd_r[l*COORD_REC_W + C_PHI +: COORD_W];
      assign rd_wgt    [l*WGT_W   +: WGT_W]   =
             coord_rd_r[l*COORD_REC_W + C_WGT +: WGT_W];

      assign rd_beam   [l] = nn_rd_r[l*NN_REC_W + NN_BEAM];
      assign rd_nn_index[l*IDX_W   +: IDX_W]   =
             nn_rd_r[l*NN_REC_W + NN_IDX +: IDX_W];
      assign rd_nn_geo  [l*GEO_W   +: GEO_W]   =
             nn_rd_r[l*NN_REC_W + NN_GEO +: GEO_W];
      assign rd_nn_log  [l*NNLOG_W +: NNLOG_W] =
             nn_rd_r[l*NN_REC_W + NN_LOG +: NNLOG_W];
    end
  endgenerate

endmodule: jc_mem
