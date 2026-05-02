// =============================================================================
// permutation_network.v  —  Hard-wired cyclic permutation between columns
// Purely combinational. No clocks, no state.
// Parameterized by per-layer shift values from the 802.11ad H-matrix.
//
// Synthesis-safe rewrite: runtime modulo (%) and integer loop variables
// replaced with genvar-based elaboration-time wiring. Since get_shift()
// returns a constant per layer, the permutation is fully resolved at
// compile time — zero logic, just wires.
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

  // Shift lookup function — returns a constant per layer at elaboration time
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

  // Static genvar-based wiring: for each source VN and layer, compute the
  // destination VN at elaboration time and generate a direct wire assignment.
  genvar sv, sl;
  generate
    for (sv = 0; sv < `Q; sv = sv + 1) begin : perm_vn
      for (sl = 0; sl < `N_LAYERS; sl = sl + 1) begin : perm_layer
        // Destination VN computed at elaboration time — no runtime modulo
        localparam integer DST = (sv + get_shift(sl)) % `Q;

        assign msg_out_flat[(DST*`N_LAYERS + sl)*`MSG_W +: `MSG_W] =
               msg_in_flat[(sv*`N_LAYERS + sl)*`MSG_W +: `MSG_W];
      end
    end
  endgenerate

endmodule
