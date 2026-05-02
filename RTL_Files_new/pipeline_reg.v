// =============================================================================
// pipeline_reg.v  —  Pipeline Register Bank
// Standard synchronous register with active-low reset.
// No clock gating in RTL — left to synthesis tool.
// =============================================================================
`include "ldpc_defines.vh"

module pipeline_reg #(
  parameter WIDTH = `Q*`N_LAYERS*`MSG_W
)(
  input               clk,
  input               rst_n,
  input  [WIDTH-1:0]  d,
  output reg [WIDTH-1:0] q
);

  always @(posedge clk) begin
    if (!rst_n)
      q <= {WIDTH{1'b0}};
    else
      q <= d;
  end

endmodule
