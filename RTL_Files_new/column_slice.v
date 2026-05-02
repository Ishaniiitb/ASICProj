// =============================================================================
// column_slice.v  —  Column Slice
// Contains Q=42 CN+VN processing units and Qv/Lvc/Cv register-based memories.
// ASIC-clean: no generated clocks, no initial blocks.
// All state cleared by synchronous active-low rst_n.
// Clock gating deferred entirely to synthesis tool.
//
// Synthesis-safe rewrite: large flat arrays replaced with per-VN generate-
// based register banks, indexed by frame_idx via small muxes.
// Eliminates variable-indexed flat arrays and dual-always-block conflicts.
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

  // ------------------------------------------------------------------
  // Cv write signals aggregated from all VN proc units
  // ------------------------------------------------------------------
  wire [`Q-1:0] cv_we_arr;
  wire [`Q-1:0] cv_wd_arr;

  // ------------------------------------------------------------------
  // Lvc write signals from processing units
  // ------------------------------------------------------------------
  wire [`N_LAYERS*`LVC_W-1:0] lvc_wd_arr  [0:`Q-1];
  wire [`Q-1:0]                lvc_we_flat;

  // ------------------------------------------------------------------
  // Cv read mux — select the read-addressed VN's register bank
  // ------------------------------------------------------------------
  reg cv_read_mux;
  reg [`Q-1:0] cv_rd_per_vn;  // one bit per VN, muxed by cv_read_frame inside generate

  always @(*) begin : cv_read_vn_mux
    cv_read_mux = cv_rd_per_vn[cv_read_vn];
  end
  assign cv_read_data = cv_read_mux;

  // ------------------------------------------------------------------
  // Per-VN Generate Blocks — qv, cv, lvc register banks
  // ------------------------------------------------------------------
  genvar vn;
  generate
    for (vn = 0; vn < `Q; vn = vn + 1) begin : gen_pu

      // ================================================================
      // Qv Register Bank: IO_DEPTH entries per VN
      // ================================================================
      reg [`LLR_W-1:0] qv_bank [0:`IO_DEPTH-1];

      // Write: addressed by qv_load_frame when qv_load_en && qv_load_vn matches
      genvar qf;
      for (qf = 0; qf < `IO_DEPTH; qf = qf + 1) begin : qv_frame
        always @(posedge clk) begin
          if (!rst_n)
            qv_bank[qf] <= {`LLR_W{1'b0}};
          else if (qv_load_en && (qv_load_vn == vn[`VN_AW-1:0]) && (qv_load_frame == qf[`IO_AW-1:0]))
            qv_bank[qf] <= qv_load_data;
        end
      end

      // Combinational read: select by frame_idx
      wire [`LLR_W-1:0] pu_qv = qv_bank[frame_idx];

      // ================================================================
      // Cv Register Bank: IO_DEPTH entries per VN
      // ================================================================
      reg cv_bank [0:`IO_DEPTH-1];

      genvar cf;
      for (cf = 0; cf < `IO_DEPTH; cf = cf + 1) begin : cv_frame
        always @(posedge clk) begin
          if (!rst_n)
            cv_bank[cf] <= 1'b0;
          else if (cv_we_arr[vn] && (frame_idx == cf[`FRM_AW-1:0]))
            cv_bank[cf] <= cv_wd_arr[vn];
        end
      end

      // Read for HD output — select by cv_read_frame
      always @(*) begin : cv_rd_mux
        cv_rd_per_vn[vn] = cv_bank[cv_read_frame];
      end

      // ================================================================
      // Lvc Register Bank: N_FRAMES × N_LAYERS entries per VN
      // ================================================================
      reg [`LVC_W-1:0] lvc_bank [0:`N_FRAMES-1][0:`N_LAYERS-1];

      genvar lf, ll;
      for (lf = 0; lf < `N_FRAMES; lf = lf + 1) begin : lvc_frame
        for (ll = 0; ll < `N_LAYERS; ll = ll + 1) begin : lvc_layer
          always @(posedge clk) begin
            if (!rst_n)
              lvc_bank[lf][ll] <= {`LVC_W{1'b0}};
            else if (lvc_we_flat[vn] && (frame_idx == lf[`FRM_AW-1:0]))
              lvc_bank[lf][ll] <= lvc_wd_arr[vn][ll*`LVC_W +: `LVC_W];
          end
        end
      end

      // Lvc read — pack all layers for current frame_idx
      wire [`N_LAYERS*`LVC_W-1:0] pu_lvc_rd;
      genvar rll;
      for (rll = 0; rll < `N_LAYERS; rll = rll + 1) begin : lvc_rd_conn
        assign pu_lvc_rd[rll*`LVC_W +: `LVC_W] = lvc_bank[frame_idx][rll];
      end

      // ================================================================
      // Processing Unit Instantiation
      // ================================================================
      wire [`N_LAYERS*`MSG_W-1:0] pu_msg_in;
      wire [`N_LAYERS*`MSG_W-1:0] pu_msg_out;
      wire                        pu_lvc_we;
      wire                        pu_cv_we;

      // Slice message bus
      assign pu_msg_in =
        msg_in_flat[vn*`N_LAYERS*`MSG_W +: `N_LAYERS*`MSG_W];
      assign msg_out_flat[vn*`N_LAYERS*`MSG_W +: `N_LAYERS*`MSG_W] =
        pu_msg_out;

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
