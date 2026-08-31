// *************************************************************************
//
// jc_fp32 -- Q14.34 fixed point to IEEE-754 binary32, round-to-nearest-even.
//
// The only place in the design where a number stops being exact. Everything
// upstream is integers: merges are exact adds, so a jet is the exact sum of
// its constituent cells. This is where that exact sum is rounded to 24 bits
// because the host wants floats, and it is deliberately the LAST thing that
// happens -- rounding here cannot influence any clustering decision.
//
// ROUND-TO-NEAREST-EVEN, AND THAT IS A CONTRACT, NOT A PREFERENCE. The model
// produces its expected bytes with struct.pack('>f', v_int / 2**34). A 48-bit
// integer divides exactly in float64 (48 < 53 mantissa bits), so that division
// is lossless and the float64 -> float32 narrowing is the only rounding, done
// RNE. Matching it here makes the bench a bit-exact comparison rather than a
// tolerance, which is the standard the rest of the datapath is held to.
//
// NO DENORMALS, NO INFINITIES, NO NaN, AND NO CODE FOR THEM. The input format
// bounds the output exponent, which is what keeps this module short:
//
//   biased exponent = msb + (127 - 34) = msb + 93,   msb in 0..47
//
// so 93..140, and 141 after a rounding carry. Comfortably inside 1..254. A
// general fixed-to-float converter spends most of its area on cases that
// Q14.34 cannot reach: the smallest nonzero magnitude is one LSB (2^-34, a
// perfectly ordinary normal) and the largest is 2^47 LSBs = 8192 GeV, which is
// the format's own ceiling. Only exact zero needs special handling.
//
// ONE CYCLE, SPLIT AFTER THE SHIFT -- and this module's own header used to say
// "combinational, and if it does not make timing, split it after the shift".
// It did not make timing, and this is that split.
//
// The whole of it -- magnitude, leading-one detect, normalising shift, then
// round-to-nearest-even and the exponent -- was 22 logic levels between
// jc_ctrl's jet register and jc_reframe's buffer write, 4.291 ns, and the last
// path holding the plugin under 250 MHz. Only 1.118 ns of that is logic; the
// rest is routing along a chain that crosses a module boundary, which is why
// halving the chain matters more than the level count suggests.
//
// NEITHER MODULE'S OWN Fmax COULD SEE IT. jc_reframe reported 676 MHz because
// standalone, jet_pz is an input port and syn/ooc.tcl constrains no ports; this
// module could not be synthesised alone at all, having had no clock to attach
// a create_clock to. It has one now, so it appears in the sweep on its own.
//
// Throughput was never the reason: jets arrive tens of cycles apart and this
// converts one per jet. The cost is a one-cycle delay on jc_reframe's buffer
// write, which is why jc_reframe delays its write controls to match rather
// than adding a wait state.
//
// *************************************************************************
`include "jc_defs.vh"
`timescale 1ns/1ps
module jc_fp32 (
  input  signed [`JC_P4_W-1:0] fx,     // Q14.34
  output        [31:0]         fp,     // IEEE-754 binary32, ONE CYCLE LATER
  input                        aclk
);

  localparam int W    = `JC_P4_W;        // 48
  localparam int FRAC = `JC_P4_FRAC;     // 34
  localparam int BIAS = 127 - FRAC;      // 93, see the header

  // ---- Magnitude -------------------------------------------------------
  // Taken in W bits, not W+1. The most negative input is -2^47, whose two's
  // complement negation is itself -- and read as UNSIGNED that bit pattern is
  // exactly 2^47, which is the magnitude wanted. Widening to catch an overflow
  // that cannot occur would only add a bit nothing reads.
  wire [W-1:0] fxu  = fx;
  wire         sign = fx[W-1];
  wire [W-1:0] mag  = sign ? (~fxu + {{(W-1){1'b0}}, 1'b1}) : fxu;
  wire         zero = (mag == {W{1'b0}});

  // ---- Leading one -----------------------------------------------------
  // Last assignment wins, so this yields the HIGHEST set bit. Synthesises to
  // a priority encoder; written as a loop so the intent survives the tool.
  function automatic [5:0] msb_pos (input [W-1:0] x);
    begin
      msb_pos = 6'd0;
      for (int i = 0; i < W; i++) if (x[i]) msb_pos = i[5:0];
    end
  endfunction

  wire [5:0]   msb  = msb_pos(mag);

  // Normalise so the leading one lands on bit W-1. That bit is the implicit
  // 1 of the significand and is discarded; the 23 below it are the fraction.
  wire [W-1:0] norm = mag << ((W-1) - msb);

  // ---- The pipeline register -------------------------------------------
  // Everything above is the leading-one detect and the shift it drives;
  // everything below is rounding and the exponent. Cutting here splits the
  // carry chains evenly and needs only the four values the second half reads.
  //
  // No reset: this is pure datapath, and jc_reframe gates its buffer write on
  // a delayed enable that IS reset, so an X here before the first jet cannot
  // reach a bank.
  logic [W-1:0] norm_r;
  logic   [5:0] msb_r;
  logic         sign_r, zero_r;

  always_ff @(posedge aclk) begin
    norm_r <= norm;
    msb_r  <= msb;
    sign_r <= sign;
    zero_r <= zero;
  end

  // ---- Round to nearest, ties to even ----------------------------------
  //   frac_raw  the 23 bits kept
  //   lsb       the last kept bit -- what "ties to even" is even ABOUT
  //   guard     the first bit dropped
  //   sticky    anything at all below the guard
  // Round up on guard && (sticky || lsb): strictly-above-half always, and
  // exactly-half only when it would otherwise leave an odd significand.
  wire [22:0] frac_raw = norm_r[W-2 -: 23];      // bits 46:24
  wire        lsb      = norm_r[24];
  wire        guard    = norm_r[23];
  wire        sticky   = |norm_r[22:0];
  wire        round_up = guard & (sticky | lsb);

  // 24 bits so the carry out of an all-ones fraction is visible rather than
  // wrapping: 0x7FFFFF + 1 must bump the exponent, not produce a zero
  // fraction at the same exponent.
  wire [23:0] frac_sum = {1'b0, frac_raw} + {23'd0, round_up};

  wire  [7:0] exp_base = {2'b00, msb_r} + BIAS[7:0];
  wire  [7:0] exp_out  = exp_base + {7'd0, frac_sum[23]};

  // Exact zero is the one input with no leading one to normalise on. Signed
  // zero falls out for free: fx == 0 has sign == 0, so it is always +0.
  assign fp = zero_r ? {sign_r, 31'd0} : {sign_r, exp_out, frac_sum[22:0]};

endmodule: jc_fp32
