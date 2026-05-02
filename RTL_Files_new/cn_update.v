// =============================================================================
// cn_update.v  —  Piecewise CN Update (one layer, one column hop)
// Purely combinational. No clocks, no latches.
// =============================================================================
`include "ldpc_defines.vh"

module cn_update (
  input  [`MSG_W-1:0] msg_in,   // incoming layer message from col t-1
  input  [`LVC_W-1:0] lvc,      // stored Lvc: [4]=sign, [3:0]=magnitude
  input               active,   // 0 = bypass (zero submatrix column)
  output [`MSG_W-1:0] msg_out   // updated layer message to col t+1
);

  wire               lvc_sign = lvc[`LVC_W-1];
  wire [`MAG_W-1:0]  lvc_mag  = lvc[`MAG_W-1:0];

  wire               in_pc   = msg_in[`PC_BIT];
  wire               in_sc   = msg_in[`SC_BIT];
  wire [`MAG_W-1:0]  in_min1 = msg_in[`MIN1_HI:`MIN1_LO];
  wire [`MAG_W-1:0]  in_min2 = msg_in[`MIN2_HI:`MIN2_LO];

  reg               out_sc;
  reg [`MAG_W-1:0]  out_min1, out_min2;

  always @(*) begin
    if (!active) begin
      out_sc   = in_sc;
      out_min1 = in_min1;
      out_min2 = in_min2;
    end else begin
      out_sc = in_sc ^ lvc_sign;

      if (lvc_mag <= in_min1) begin
        out_min1 = lvc_mag;
        out_min2 = in_min1;
      end else if (lvc_mag < in_min2) begin
        out_min1 = in_min1;
        out_min2 = lvc_mag;
      end else begin
        out_min1 = in_min1;
        out_min2 = in_min2;
      end
    end
  end

  // pc is not modified in CN phase — passed straight through
  assign msg_out = {in_pc, out_sc, out_min1, out_min2};

endmodule
