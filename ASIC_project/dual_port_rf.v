// =============================================================================
// dual_port_rf.v
// Generic Dual-Port Register File: 1 synchronous write, 1 synchronous read.
// ASIC-synthesizable: no initial blocks. Memory cleared via synchronous reset.
// Replace with foundry SRAM macro for tapeout.
// =============================================================================
module dual_port_rf #(
  parameter DEPTH = 8,
  parameter WIDTH = 5,
  parameter AW    = 3    // must satisfy AW == $clog2(DEPTH)
)(
  input              clk,
  input              rst_n,
  // Write port
  input              we,
  input  [AW-1:0]    waddr,
  input  [WIDTH-1:0] wdata,
  // Read port (registered output — matches SRAM read behaviour)
  input              re,
  input  [AW-1:0]    raddr,
  output reg [WIDTH-1:0] rdata
);

  reg [WIDTH-1:0] mem [0:DEPTH-1];

  integer i;

  // Synchronous write with synchronous reset
  always @(posedge clk) begin
    if (!rst_n) begin
      // Synchronous clear — synthesizes to reset mux on each cell
      for (i = 0; i < DEPTH; i = i + 1)
        mem[i] <= {WIDTH{1'b0}};
    end else begin
      if (we) mem[waddr] <= wdata;
    end
  end

  // Synchronous read with synchronous reset
  always @(posedge clk) begin
    if (!rst_n)
      rdata <= {WIDTH{1'b0}};
    else if (re)
      rdata <= mem[raddr];
  end

endmodule