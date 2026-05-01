// =============================================================================
// column_slice.v  —  Column Slice
// Contains Q=42 CN+VN processing units and Qv/Lvc/Cv register-based memories.
// ASIC-clean: no generated clocks, no initial blocks.
// All state cleared by synchronous active-low rst_n.
// Clock gating deferred entirely to synthesis tool.
// =============================================================================
`include "ldpc_defines.vh"

module column_slice #(
  parameter COL_IDX = 0
)(
  input                              clk,
  input                              rst_n,
  input                              phase,
  input                              first_iter,
  input  [`N_LAYERS-1:0]             layer_active,
  input  [`FRM_AW-1:0]               frame_idx,
  // Layer message buses
  input  [`Q*`N_LAYERS*`MSG_W-1:0]   msg_in_flat,
  output [`Q*`N_LAYERS*`MSG_W-1:0]   msg_out_flat,
  // LLR load
  input                              qv_load_en,
  input  [`VN_AW-1:0]                qv_load_vn,
  input  [`IO_AW-1:0]                qv_load_frame,
  input  [`LLR_W-1:0]                qv_load_data,
  // Hard decision read
  input  [`VN_AW-1:0]                cv_read_vn,
  input  [`IO_AW-1:0]                cv_read_frame,
  output                             cv_read_data
);

  // Memory depth constants
  localparam QV_D  = `Q * `IO_DEPTH;             // 672
  localparam CV_D  = `Q * `IO_DEPTH;             // 672
  localparam LVC_D = `Q * `N_FRAMES * `N_LAYERS; // 2688

  // Memory arrays
  reg [`LLR_W-1:0] qv_mem  [0:QV_D-1];
  reg              cv_mem  [0:CV_D-1];
  reg [`LVC_W-1:0] lvc_mem [0:LVC_D-1];

  integer ri;

  // ------------------------------------------------------------------
  // Qv memory — synchronous write, synchronous reset
  // ------------------------------------------------------------------
  always @(posedge clk) begin
    if (!rst_n) begin
      for (ri = 0; ri < QV_D; ri = ri + 1)
        qv_mem[ri] <= {`LLR_W{1'b0}};
    end else if (qv_load_en) begin
      qv_mem[qv_load_frame * `Q + qv_load_vn] <= qv_load_data;
    end
  end

  // ------------------------------------------------------------------
  // Cv memory — synchronous reset; writes driven by processing units
  // ------------------------------------------------------------------
  // Cv write signals aggregated from all VN proc units
  wire [`Q-1:0] cv_we_arr;
  wire [`Q-1:0] cv_wd_arr;

  always @(posedge clk) begin
    if (!rst_n) begin
      for (ri = 0; ri < CV_D; ri = ri + 1)
        cv_mem[ri] <= 1'b0;
    end else begin
      // Each proc unit writes its own VN's hard decision
      // Write address = frame_idx * Q + vn
      // (Synthesises to Q write paths with shared frame_idx offset)
    end
  end

  genvar vn_w;
  generate
    for (vn_w = 0; vn_w < `Q; vn_w = vn_w + 1) begin : cv_wr_gen
      always @(posedge clk) begin
        if (!rst_n)
          cv_mem[0 * `Q + vn_w] <= 1'b0;  // reset only entry 0; full clear above
        else if (cv_we_arr[vn_w])
          cv_mem[frame_idx * `Q + vn_w] <= cv_wd_arr[vn_w];
      end
    end
  endgenerate

  // Cv read — asynchronous (combinational read from reg array)
  assign cv_read_data = cv_mem[cv_read_frame * `Q + cv_read_vn];

  // ------------------------------------------------------------------
  // Lvc memory — synchronous reset; writes driven by processing units
  // ------------------------------------------------------------------
  wire [`N_LAYERS*`LVC_W-1:0] lvc_we_arr  [0:`Q-1];  // per-VN write enable flags (1 bit each layer)
  wire [`N_LAYERS*`LVC_W-1:0] lvc_wd_arr  [0:`Q-1];
  wire [`Q-1:0]                lvc_we_flat;           // one bit per VN

  always @(posedge clk) begin
    if (!rst_n) begin
      for (ri = 0; ri < LVC_D; ri = ri + 1)
        lvc_mem[ri] <= {`LVC_W{1'b0}};
    end
  end

  genvar vn_l, ll;
  generate
    for (vn_l = 0; vn_l < `Q; vn_l = vn_l + 1) begin : lvc_wr_gen
      for (ll = 0; ll < `N_LAYERS; ll = ll + 1) begin : lvc_wr_layer
        always @(posedge clk) begin
          if (lvc_we_flat[vn_l])
            lvc_mem[(frame_idx*`Q*`N_LAYERS) + (vn_l*`N_LAYERS) + ll]
              <= lvc_wd_arr[vn_l][ll*`LVC_W +: `LVC_W];
        end
      end
    end
  endgenerate

  // ------------------------------------------------------------------
  // Q Processing Units
  // ------------------------------------------------------------------
  genvar vn;
  generate
    for (vn = 0; vn < `Q; vn = vn + 1) begin : gen_pu

      wire [`N_LAYERS*`MSG_W-1:0] pu_msg_in;
      wire [`N_LAYERS*`MSG_W-1:0] pu_msg_out;
      wire [`N_LAYERS*`LVC_W-1:0] pu_lvc_rd;
      wire [`LLR_W-1:0]           pu_qv;
      wire                        pu_lvc_we;
      wire                        pu_cv_we;

      // Slice message bus
      assign pu_msg_in =
        msg_in_flat[vn*`N_LAYERS*`MSG_W +: `N_LAYERS*`MSG_W];
      assign msg_out_flat[vn*`N_LAYERS*`MSG_W +: `N_LAYERS*`MSG_W] =
        pu_msg_out;

      // Qv combinational read
      assign pu_qv = qv_mem[frame_idx * `Q + vn];

      // Lvc combinational read per layer
      genvar rll;
      for (rll = 0; rll < `N_LAYERS; rll = rll + 1) begin : lvc_rd_conn
        assign pu_lvc_rd[rll*`LVC_W +: `LVC_W] =
          lvc_mem[(frame_idx*`Q*`N_LAYERS) + (vn*`N_LAYERS) + rll];
      end

      cn_vn_proc_unit #(
        .UNIT_IDX(vn),
        .COL_IDX (COL_IDX)
      ) u_pu (
        .clk            (clk),
        .rst_n          (rst_n),
        .phase          (phase),
        .first_iter     (first_iter),
        .layer_active   (layer_active),
        .frame_idx      (frame_idx),
        .msg_in_flat    (pu_msg_in),
        .msg_out_flat   (pu_msg_out),
        .lvc_we         (pu_lvc_we),
        .lvc_wdata_flat (lvc_wd_arr[vn]),
        .lvc_rdata_flat (pu_lvc_rd),
        .qv_rdata       (pu_qv),
        .cv_we          (pu_cv_we),
        .cv_wdata       (cv_wd_arr[vn])
      );

      assign lvc_we_flat[vn] = pu_lvc_we;
      assign cv_we_arr[vn]   = pu_cv_we;
    end
  endgenerate

endmodule
