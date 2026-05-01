// =============================================================================
// global_ctrl.v  —  Global Control FSM
// Manages phase, frame index, iteration counter, early termination.
// No clock gating signals — removed entirely.
// =============================================================================
`include "ldpc_defines.vh"

module global_ctrl (
  input                      clk,
  input                      rst_n,
  input                      start,
  input                      parity_ok,
  input  [1:0]               code_rate,
  output reg                 phase,
  output reg                 first_iter,
  output reg [`FRM_AW-1:0]   frame_idx,
  output reg [`N_LAYERS-1:0] layer_active,
  output reg                 done,
  output reg [`ITER_AW-1:0]  iter_count
);

  // FSM states
  localparam S_IDLE  = 3'd0;
  localparam S_CN    = 3'd1;
  localparam S_VN    = 3'd2;
  localparam S_CHECK = 3'd3;
  localparam S_DONE  = 3'd4;

  reg [2:0]          state, next_state;
  reg [`FRM_AW-1:0]  cycle_cnt;
  reg [`ITER_AW-1:0] iter_reg;

  // ---- Layer active mask ---------------------------------------------
  always @(*) begin
    case (code_rate)
      `RATE_1_2  : layer_active = 8'hFF;
      `RATE_5_8  : layer_active = 8'hFF;
      `RATE_3_4  : layer_active = 8'hFF;
      `RATE_13_16: layer_active = 8'h3F;
      default    : layer_active = 8'hFF;
    endcase
  end

  // ---- State register ------------------------------------------------
  always @(posedge clk) begin
    if (!rst_n) state <= S_IDLE;
    else        state <= next_state;
  end

  // ---- Counters ------------------------------------------------------
  always @(posedge clk) begin
    if (!rst_n) begin
      cycle_cnt <= {`FRM_AW{1'b0}};
      iter_reg  <= {`ITER_AW{1'b0}};
    end else begin
      case (state)
        S_IDLE: begin
          cycle_cnt <= {`FRM_AW{1'b0}};
          iter_reg  <= {`ITER_AW{1'b0}};
        end
        S_CN, S_VN: begin
          if (cycle_cnt == `N_FRAMES - 1) begin
            cycle_cnt <= {`FRM_AW{1'b0}};
            if (state == S_VN)
              iter_reg <= iter_reg + 1'b1;
          end else begin
            cycle_cnt <= cycle_cnt + 1'b1;
          end
        end
        default: ;
      endcase
    end
  end

  // ---- Next state ----------------------------------------------------
  always @(*) begin
    next_state = state;
    case (state)
      S_IDLE  : if (start)                               next_state = S_CN;
      S_CN    : if (cycle_cnt == `N_FRAMES - 1)          next_state = S_VN;
      S_VN    : if (cycle_cnt == `N_FRAMES - 1)          next_state = S_CHECK;
      S_CHECK : if (parity_ok || iter_reg >= `MAX_ITER)  next_state = S_DONE;
                else                                      next_state = S_CN;
      S_DONE  :                                           next_state = S_IDLE;
      default : next_state = S_IDLE;
    endcase
  end

  // ---- Output logic --------------------------------------------------
  always @(*) begin
    phase      = (state == S_CN) ? `PHASE_CN : `PHASE_VN;
    first_iter = (iter_reg == {`ITER_AW{1'b0}}) ? 1'b1 : 1'b0;
    frame_idx  = cycle_cnt;
    done       = (state == S_DONE) ? 1'b1 : 1'b0;
    iter_count = iter_reg;
  end

endmodule
