// =============================================================================
// cn_vn_proc_unit.v  —  Combined CN+VN Processing Unit
// One instance per VN per column slice (42 per slice).
// Processes N_LAYERS=8 layers in parallel each clock cycle.
// No clock gating in RTL — enable-based register control only.
// =============================================================================
`include "ldpc_defines.vh"

module cn_vn_proc_unit #(
  parameter UNIT_IDX = 0,
  parameter COL_IDX  = 0
)(
  input                           clk,
  input                           rst_n,
  input                           phase,
  input                           first_iter,
  input  [`N_LAYERS-1:0]          layer_active,
  input  [`FRM_AW-1:0]            frame_idx,
  // Packed layer message buses
  input  [`N_LAYERS*`MSG_W-1:0]   msg_in_flat,
  output [`N_LAYERS*`MSG_W-1:0]   msg_out_flat,
  // Lvc memory interface
  output                          lvc_we,
  output [`N_LAYERS*`LVC_W-1:0]   lvc_wdata_flat,
  input  [`N_LAYERS*`LVC_W-1:0]   lvc_rdata_flat,
  // Qv and Cv
  input  [`LLR_W-1:0]             qv_rdata,
  output                          cv_we,
  output                          cv_wdata
);

  // ------------------------------------------------------------------
  // Per-layer wiring
  // ------------------------------------------------------------------
  wire [`MSG_W-1:0] msg_in  [0:`N_LAYERS-1];
  wire [`LVC_W-1:0] lvc_rd  [0:`N_LAYERS-1];
  wire [`MSG_W-1:0] cn_out  [0:`N_LAYERS-1];
  wire [`MSG_W-1:0] vn_out  [0:`N_LAYERS-1];
  wire [`LVC_W-1:0] vn_lvc  [0:`N_LAYERS-1];
  wire [`LLR_W-1:0] lv_chain[0:`N_LAYERS];  // lv_chain[0]=qv; chain[l+1]=after layer l
  wire              vn_cv   [0:`N_LAYERS-1];

  assign lv_chain[0] = qv_rdata;

  genvar l;
  generate
    for (l = 0; l < `N_LAYERS; l = l + 1) begin : g_layer

      assign msg_in[l] = msg_in_flat [l*`MSG_W  +: `MSG_W];
      assign lvc_rd[l] = lvc_rdata_flat[l*`LVC_W +: `LVC_W];

      cn_update u_cn (
        .msg_in  (msg_in[l]),
        .lvc     (lvc_rd[l]),
        .active  (layer_active[l]),
        .msg_out (cn_out[l])
      );

      vn_update u_vn (
        .msg_in     (msg_in[l]),
        .lvc_in     (lvc_rd[l]),
        .qv         (qv_rdata),
        .lv_acc_in  (lv_chain[l]),
        .active     (layer_active[l]),
        .first_iter (first_iter),
        .lvc_out    (vn_lvc[l]),
        .lv_acc_out (lv_chain[l+1]),
        .cv_out     (vn_cv[l]),
        .msg_out    (vn_out[l])
      );

      // Output mux: CN or VN result
      assign msg_out_flat[l*`MSG_W +: `MSG_W] =
        (phase == `PHASE_CN) ? cn_out[l] : vn_out[l];

      // Lvc write data
      assign lvc_wdata_flat[l*`LVC_W +: `LVC_W] = vn_lvc[l];
    end
  endgenerate

  // Write enables — only in VN phase
  assign lvc_we   = (phase == `PHASE_VN);
  assign cv_we    = (phase == `PHASE_VN);

  // Hard decision: sign of final accumulated LLR across all active layers
  assign cv_wdata = lv_chain[`N_LAYERS][`LLR_W-1];

endmodule
