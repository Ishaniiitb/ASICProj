// =============================================================================
// permutation_network.v  —  Hard-wired cyclic permutation between columns
// Purely combinational. No clocks, no state.
// Parameterized by per-layer shift values from the 802.11ad H-matrix.
// =============================================================================
`include "ldpc_defines.vh"

module permutation_network #(
  parameter SHIFT_L0 = 0,
  parameter SHIFT_L1 = 0,
  parameter SHIFT_L2 = 0,
  parameter SHIFT_L3 = 0,
  parameter SHIFT_L4 = 0,
  parameter SHIFT_L5 = 0,
  parameter SHIFT_L6 = 0,
  parameter SHIFT_L7 = 0
)(
  input  [`Q*`N_LAYERS*`MSG_W-1:0] msg_in_flat,
  output [`Q*`N_LAYERS*`MSG_W-1:0] msg_out_flat
);

  wire [`MSG_W-1:0] msg_in  [0:`Q-1][0:`N_LAYERS-1];
  reg  [`MSG_W-1:0] msg_out [0:`Q-1][0:`N_LAYERS-1];

  // Unpack
  genvar vn, l;
  generate
    for (vn = 0; vn < `Q; vn = vn + 1) begin : unp_vn
      for (l = 0; l < `N_LAYERS; l = l + 1) begin : unp_l
        assign msg_in[vn][l] =
          msg_in_flat[(vn*`N_LAYERS + l)*`MSG_W +: `MSG_W];
      end
    end
  endgenerate

  // Shift lookup function
  function integer get_shift;
    input integer layer;
    begin
      case (layer)
        0: get_shift = SHIFT_L0;
        1: get_shift = SHIFT_L1;
        2: get_shift = SHIFT_L2;
        3: get_shift = SHIFT_L3;
        4: get_shift = SHIFT_L4;
        5: get_shift = SHIFT_L5;
        6: get_shift = SHIFT_L6;
        7: get_shift = SHIFT_L7;
        default: get_shift = 0;
      endcase
    end
  endfunction

  // Permute: for each source VN, route to destination using cyclic shift
  integer sv, sl, dst;
  always @(*) begin
    for (sv = 0; sv < `Q; sv = sv + 1)
      for (sl = 0; sl < `N_LAYERS; sl = sl + 1)
        msg_out[sv][sl] = {`MSG_W{1'b0}};

    for (sv = 0; sv < `Q; sv = sv + 1) begin
      for (sl = 0; sl < `N_LAYERS; sl = sl + 1) begin
        dst = (sv + get_shift(sl)) % `Q;
        msg_out[dst][sl] = msg_in[sv][sl];
      end
    end
  end

  // Repack
  generate
    for (vn = 0; vn < `Q; vn = vn + 1) begin : rep_vn
      for (l = 0; l < `N_LAYERS; l = l + 1) begin : rep_l
        assign msg_out_flat[(vn*`N_LAYERS + l)*`MSG_W +: `MSG_W] =
          msg_out[vn][l];
      end
    end
  endgenerate

endmodule
