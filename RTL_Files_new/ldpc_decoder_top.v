// =============================================================================
// ldpc_decoder_top.v  —  Top-Level LDPC Decoder
// IEEE 802.11ad: N=672, Q=42, 16 column slices, 8 frames, 8 layers
// ASIC-clean: no generated clocks, no initial blocks.
// =============================================================================
`include "ldpc_defines.vh"

module ldpc_decoder_top (
  input              clk,
  input              rst_n,
  input              start,
  input  [1:0]       code_rate,
  // LLR load
  input              llr_valid,
  input  [`VN_AW-1:0]   llr_vn,
  input  [`COL_AW-1:0]  llr_col,
  input  [`IO_AW-1:0]   llr_frame,
  input  [`LLR_W-1:0]   llr_data,
  // Hard decision read
  input              hd_req,
  input  [`VN_AW-1:0]   hd_vn,
  input  [`COL_AW-1:0]  hd_col,
  input  [`IO_AW-1:0]   hd_frame,
  output             hd_data,
  // Status
  output             done,
  output [`ITER_AW-1:0] iter_count
);

  localparam MBUS_W = `Q * `N_LAYERS * `MSG_W;  // 3360

  // ---- Control signals -----------------------------------------------
  wire                 phase;
  wire                 first_iter;
  wire [`FRM_AW-1:0]   frame_idx;
  wire [`N_LAYERS-1:0] layer_active;

  // ---- Message buses -------------------------------------------------
  // msg_bus_flat [0..15] : column slice outputs
  // msg_perm_flat[0..14] : permutation outputs  (15 networks, after col 0..14)
  // msg_pipe_flat[0.. 7] : pipeline reg outputs (8 registers)
  wire [MBUS_W-1:0] msg_bus_flat  [0:15];
  wire [MBUS_W-1:0] msg_perm_flat [0:14];
  wire [MBUS_W-1:0] msg_pipe_flat [0: 7];

  // ---- Hard decision per-column -------------------------------------
  wire [15:0] cv_data_arr;

  // Direct bit-select mux — no integer loop needed
  assign hd_data = cv_data_arr[hd_col];

  // ---- Parity aggregation -------------------------------------------
  // Per-column parity extraction using genvar, then OR-reduce
  wire [`N_COLS-1:0] col_parity;

  genvar pc_c, pc_v, pc_l;
  generate
    for (pc_c = 0; pc_c < `N_COLS; pc_c = pc_c + 1) begin : par_col

      // Extract parity bit from each active VN/layer in this column
      wire [`Q*`N_LAYERS-1:0] pc_bits;

      for (pc_v = 0; pc_v < `Q; pc_v = pc_v + 1) begin : par_vn
        for (pc_l = 0; pc_l < `N_LAYERS; pc_l = pc_l + 1) begin : par_layer
          assign pc_bits[pc_v*`N_LAYERS + pc_l] =
            layer_active[pc_l] &
            msg_bus_flat[pc_c][(pc_v*`N_LAYERS + pc_l)*`MSG_W + `PC_BIT];
        end
      end

      // OR-reduce all parity bits for this column
      assign col_parity[pc_c] = |pc_bits;
    end
  endgenerate

  wire parity_ok = ~(|col_parity);

  // ---- Global Control -----------------------------------------------
  global_ctrl u_ctrl (
    .clk          (clk),
    .rst_n        (rst_n),
    .start        (start),
    .parity_ok    (parity_ok),
    .code_rate    (code_rate),
    .phase        (phase),
    .first_iter   (first_iter),
    .frame_idx    (frame_idx),
    .layer_active (layer_active),
    .done         (done),
    .iter_count   (iter_count)
  );

  // ---- Slice input routing ------------------------------------------
  // col 0        : wraps from msg_pipe_flat[7] (ring)
  // col odd  c   : from msg_perm_flat[c-1]
  // col even c>0 : from msg_pipe_flat[c/2 - 1]
  wire [MBUS_W-1:0] slice_in [0:15];

  assign slice_in[ 0] = msg_pipe_flat[7];   // ring wrap
  assign slice_in[ 1] = msg_perm_flat[0];
  assign slice_in[ 2] = msg_pipe_flat[0];
  assign slice_in[ 3] = msg_perm_flat[2];
  assign slice_in[ 4] = msg_pipe_flat[1];
  assign slice_in[ 5] = msg_perm_flat[4];
  assign slice_in[ 6] = msg_pipe_flat[2];
  assign slice_in[ 7] = msg_perm_flat[6];
  assign slice_in[ 8] = msg_pipe_flat[3];
  assign slice_in[ 9] = msg_perm_flat[8];
  assign slice_in[10] = msg_pipe_flat[4];
  assign slice_in[11] = msg_perm_flat[10];
  assign slice_in[12] = msg_pipe_flat[5];
  assign slice_in[13] = msg_perm_flat[12];
  assign slice_in[14] = msg_pipe_flat[6];
  assign slice_in[15] = msg_perm_flat[14];

  // ---- Column Slices -------------------------------------------------
  genvar cs;
  generate
    for (cs = 0; cs < `N_COLS; cs = cs + 1) begin : gen_col_slice
      wire slice_llr_en;
      assign slice_llr_en = llr_valid && (llr_col == cs[`COL_AW-1:0]);

      column_slice #(.COL_IDX(cs)) u_slice (
        .clk           (clk),
        .rst_n         (rst_n),
        .phase         (phase),
        .first_iter    (first_iter),
        .layer_active  (layer_active),
        .frame_idx     (frame_idx),
        .msg_in_flat   (slice_in[cs]),
        .msg_out_flat  (msg_bus_flat[cs]),
        .qv_load_en    (slice_llr_en),
        .qv_load_vn    (llr_vn),
        .qv_load_frame (llr_frame),
        .qv_load_data  (llr_data),
        .cv_read_vn    (hd_vn),
        .cv_read_frame (hd_frame),
        .cv_read_data  (cv_data_arr[cs])
      );
    end
  endgenerate

  // ---- Permutation Networks (15 instances, one per column boundary) --
  // All shifts set to 0 (identity) — replace with actual 802.11ad values
  // from IEEE 802.11ad-2012 Annex V for functional BER simulation.
  permutation_network #(.SHIFT_L0( 0),.SHIFT_L1( 0),.SHIFT_L2( 0),.SHIFT_L3( 0),
                        .SHIFT_L4( 0),.SHIFT_L5( 0),.SHIFT_L6( 0),.SHIFT_L7( 0))
    u_perm_0  (.msg_in_flat(msg_bus_flat[ 0]),.msg_out_flat(msg_perm_flat[ 0]));

  permutation_network #(.SHIFT_L0(19),.SHIFT_L1( 0),.SHIFT_L2( 0),.SHIFT_L3( 0),
                        .SHIFT_L4( 0),.SHIFT_L5( 0),.SHIFT_L6( 0),.SHIFT_L7( 0))
    u_perm_1  (.msg_in_flat(msg_bus_flat[ 1]),.msg_out_flat(msg_perm_flat[ 1]));

  permutation_network #(.SHIFT_L0( 0),.SHIFT_L1( 0),.SHIFT_L2( 0),.SHIFT_L3( 0),
                        .SHIFT_L4( 0),.SHIFT_L5( 0),.SHIFT_L6( 0),.SHIFT_L7( 0))
    u_perm_2  (.msg_in_flat(msg_bus_flat[ 2]),.msg_out_flat(msg_perm_flat[ 2]));

  permutation_network #(.SHIFT_L0( 2),.SHIFT_L1( 0),.SHIFT_L2( 0),.SHIFT_L3( 0),
                        .SHIFT_L4( 0),.SHIFT_L5( 0),.SHIFT_L6( 0),.SHIFT_L7( 0))
    u_perm_3  (.msg_in_flat(msg_bus_flat[ 3]),.msg_out_flat(msg_perm_flat[ 3]));

  permutation_network #(.SHIFT_L0( 0),.SHIFT_L1( 0),.SHIFT_L2( 0),.SHIFT_L3( 0),
                        .SHIFT_L4( 0),.SHIFT_L5( 0),.SHIFT_L6( 0),.SHIFT_L7( 0))
    u_perm_4  (.msg_in_flat(msg_bus_flat[ 4]),.msg_out_flat(msg_perm_flat[ 4]));

  permutation_network #(.SHIFT_L0( 0),.SHIFT_L1( 0),.SHIFT_L2( 0),.SHIFT_L3( 0),
                        .SHIFT_L4( 0),.SHIFT_L5( 0),.SHIFT_L6( 0),.SHIFT_L7( 0))
    u_perm_5  (.msg_in_flat(msg_bus_flat[ 5]),.msg_out_flat(msg_perm_flat[ 5]));

  permutation_network #(.SHIFT_L0( 0),.SHIFT_L1( 0),.SHIFT_L2( 0),.SHIFT_L3( 0),
                        .SHIFT_L4( 0),.SHIFT_L5( 0),.SHIFT_L6( 0),.SHIFT_L7( 0))
    u_perm_6  (.msg_in_flat(msg_bus_flat[ 6]),.msg_out_flat(msg_perm_flat[ 6]));

  permutation_network #(.SHIFT_L0( 0),.SHIFT_L1( 0),.SHIFT_L2( 0),.SHIFT_L3( 0),
                        .SHIFT_L4( 0),.SHIFT_L5( 0),.SHIFT_L6( 0),.SHIFT_L7( 0))
    u_perm_7  (.msg_in_flat(msg_bus_flat[ 7]),.msg_out_flat(msg_perm_flat[ 7]));

  permutation_network #(.SHIFT_L0( 0),.SHIFT_L1( 0),.SHIFT_L2( 0),.SHIFT_L3( 0),
                        .SHIFT_L4( 0),.SHIFT_L5( 0),.SHIFT_L6( 0),.SHIFT_L7( 0))
    u_perm_8  (.msg_in_flat(msg_bus_flat[ 8]),.msg_out_flat(msg_perm_flat[ 8]));

  permutation_network #(.SHIFT_L0( 0),.SHIFT_L1( 0),.SHIFT_L2( 0),.SHIFT_L3( 0),
                        .SHIFT_L4( 0),.SHIFT_L5( 0),.SHIFT_L6( 0),.SHIFT_L7( 0))
    u_perm_9  (.msg_in_flat(msg_bus_flat[ 9]),.msg_out_flat(msg_perm_flat[ 9]));

  permutation_network #(.SHIFT_L0( 0),.SHIFT_L1( 0),.SHIFT_L2( 0),.SHIFT_L3( 0),
                        .SHIFT_L4( 0),.SHIFT_L5( 0),.SHIFT_L6( 0),.SHIFT_L7( 0))
    u_perm_10 (.msg_in_flat(msg_bus_flat[10]),.msg_out_flat(msg_perm_flat[10]));

  permutation_network #(.SHIFT_L0( 0),.SHIFT_L1( 0),.SHIFT_L2( 0),.SHIFT_L3( 0),
                        .SHIFT_L4( 0),.SHIFT_L5( 0),.SHIFT_L6( 0),.SHIFT_L7( 0))
    u_perm_11 (.msg_in_flat(msg_bus_flat[11]),.msg_out_flat(msg_perm_flat[11]));

  permutation_network #(.SHIFT_L0( 0),.SHIFT_L1( 0),.SHIFT_L2( 0),.SHIFT_L3( 0),
                        .SHIFT_L4( 0),.SHIFT_L5( 0),.SHIFT_L6( 0),.SHIFT_L7( 0))
    u_perm_12 (.msg_in_flat(msg_bus_flat[12]),.msg_out_flat(msg_perm_flat[12]));

  permutation_network #(.SHIFT_L0( 0),.SHIFT_L1( 0),.SHIFT_L2( 0),.SHIFT_L3( 0),
                        .SHIFT_L4( 0),.SHIFT_L5( 0),.SHIFT_L6( 0),.SHIFT_L7( 0))
    u_perm_13 (.msg_in_flat(msg_bus_flat[13]),.msg_out_flat(msg_perm_flat[13]));

  permutation_network #(.SHIFT_L0( 0),.SHIFT_L1( 0),.SHIFT_L2( 0),.SHIFT_L3( 0),
                        .SHIFT_L4( 0),.SHIFT_L5( 0),.SHIFT_L6( 0),.SHIFT_L7( 0))
    u_perm_14 (.msg_in_flat(msg_bus_flat[14]),.msg_out_flat(msg_perm_flat[14]));

  // ---- Pipeline Registers -------------------------------------------
  // pipe[p] is placed after the odd column of each 2-column pair.
  // pipe[0] after col  1 → msg_perm_flat[ 1]
  // pipe[1] after col  3 → msg_perm_flat[ 3]
  // pipe[2] after col  5 → msg_perm_flat[ 5]
  // pipe[3] after col  7 → msg_perm_flat[ 7]
  // pipe[4] after col  9 → msg_perm_flat[ 9]
  // pipe[5] after col 11 → msg_perm_flat[11]
  // pipe[6] after col 13 → msg_perm_flat[13]
  // pipe[7] after col 14 → msg_perm_flat[14]
  //   (col 15 is the last slice — no permutation follows it,
  //    so pipe[7] uses the last available permutation output, perm[14])
  pipeline_reg #(.WIDTH(MBUS_W)) u_pipe_0 (.clk(clk),.rst_n(rst_n),
    .d(msg_perm_flat[ 1]),.q(msg_pipe_flat[0]));
  pipeline_reg #(.WIDTH(MBUS_W)) u_pipe_1 (.clk(clk),.rst_n(rst_n),
    .d(msg_perm_flat[ 3]),.q(msg_pipe_flat[1]));
  pipeline_reg #(.WIDTH(MBUS_W)) u_pipe_2 (.clk(clk),.rst_n(rst_n),
    .d(msg_perm_flat[ 5]),.q(msg_pipe_flat[2]));
  pipeline_reg #(.WIDTH(MBUS_W)) u_pipe_3 (.clk(clk),.rst_n(rst_n),
    .d(msg_perm_flat[ 7]),.q(msg_pipe_flat[3]));
  pipeline_reg #(.WIDTH(MBUS_W)) u_pipe_4 (.clk(clk),.rst_n(rst_n),
    .d(msg_perm_flat[ 9]),.q(msg_pipe_flat[4]));
  pipeline_reg #(.WIDTH(MBUS_W)) u_pipe_5 (.clk(clk),.rst_n(rst_n),
    .d(msg_perm_flat[11]),.q(msg_pipe_flat[5]));
  pipeline_reg #(.WIDTH(MBUS_W)) u_pipe_6 (.clk(clk),.rst_n(rst_n),
    .d(msg_perm_flat[13]),.q(msg_pipe_flat[6]));
  pipeline_reg #(.WIDTH(MBUS_W)) u_pipe_7 (.clk(clk),.rst_n(rst_n),
    .d(msg_perm_flat[14]),.q(msg_pipe_flat[7]));

endmodule
