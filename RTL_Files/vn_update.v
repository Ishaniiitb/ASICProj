// =============================================================================
// vn_update.v  —  VN Update (one layer, one column hop)
// Purely combinational. No clocks, no latches.
// Synthesisable: no $signed() casts, no unary minus, explicit widths only.
// =============================================================================
`include "ldpc_defines.vh"

module vn_update (
  input [`MSG_W-1:0] msg_in,      // final {pc,sc,min1,min2} from CN pass
  input [`LVC_W-1:0] lvc_in,      // current Lvc (sign-magnitude)
  input [`LLR_W-1:0] qv,          // channel LLR for this VN
  input [`LLR_W-1:0] lv_acc_in,   // accumulated LLR from previous layers
  input active,
  input first_iter,  // use qv as base LLR on iteration 0
  output [`LVC_W-1:0] lvc_out,     // updated Lvc
  output [`LLR_W-1:0] lv_acc_out,  // updated accumulated LLR
  output cv_out,       // hard decision bit
  output [`MSG_W-1:0] msg_out       // pc updated; sc/min1/min2 pass through
);


  wire               lvc_sign = lvc_in[`LVC_W-1];
  wire [`MAG_W-1:0]  lvc_mag  = lvc_in[`MAG_W-1:0];

  wire               in_pc   = msg_in[`PC_BIT];
  wire               in_sc   = msg_in[`SC_BIT];
  wire [`MAG_W-1:0]  in_min1 = msg_in[`MIN1_HI:`MIN1_LO];
  wire [`MAG_W-1:0]  in_min2 = msg_in[`MIN2_HI:`MIN2_LO];

  localparam [`LLR_W-1:0] SAT_MAX = 5'b00111;  // +7
  localparam [`LLR_W-1:0] SAT_MIN = 5'b11000;  // -8

  // ---- mcv: computed in always block ----------------------------------
  reg [`MAG_W-1:0]  mcv_mag;
  reg               mcv_sign;

  always @(*) begin
    if (active) begin
      mcv_mag  = (lvc_mag == in_min1) ? in_min2 : in_min1;
      mcv_sign = in_sc ^ lvc_sign;
    end else begin
      mcv_mag  = {`MAG_W{1'b0}};
      mcv_sign = 1'b0;
    end
  end

  // ---- Sign-magnitude -> 2's complement (explicit, no unary minus) ----
  wire [`LLR_W-1:0] mcv_pos_form = {1'b0, mcv_mag};
  wire [`LLR_W-1:0] mcv_neg_form = (~mcv_pos_form) + {{(`LLR_W-1){1'b0}}, 1'b1};
  wire [`LLR_W-1:0] mcv_2c       = mcv_sign ? mcv_neg_form : mcv_pos_form;

  // ---- Base LLR -------------------------------------------------------
  wire [`LLR_W-1:0] lv_base = first_iter ? qv : lv_acc_in;

  // ---- Addition with overflow detection (LLR_W+1 bits) ---------------
  wire [`LLR_W:0] lv_base_ext = {lv_base[`LLR_W-1], lv_base};
  wire [`LLR_W:0] mcv_2c_ext  = {mcv_2c[`LLR_W-1],  mcv_2c};
  wire [`LLR_W:0] lv_raw_ext  = lv_base_ext + mcv_2c_ext;

  // Overflow: sign bits of extended result differ from truncated result
  // positive overflow: top bit=0, next bit=1  -> clamp to +7
  // negative overflow: top bit=1, next bit=0  -> clamp to -8
  wire overflow  = (~lv_raw_ext[`LLR_W]) & lv_raw_ext[`LLR_W-1];
  wire underflow =   lv_raw_ext[`LLR_W]  & (~lv_raw_ext[`LLR_W-1]);

  wire [`LLR_W-1:0] lv_sat = overflow  ? SAT_MAX :
                              underflow ? SAT_MIN :
                                          lv_raw_ext[`LLR_W-1:0];

  // ---- Hard decision --------------------------------------------------
  wire cv_wire = lv_sat[`LLR_W-1];

  // ---- 2's complement -> sign-magnitude for Lvc -----------------------
  wire [`LLR_W-1:0] lv_sat_neg = (~lv_sat) + {{(`LLR_W-1){1'b0}}, 1'b1};
  wire [`MAG_W-1:0] lv_sat_mag = cv_wire ? lv_sat_neg[`MAG_W-1:0]
                                          : lv_sat[`MAG_W-1:0];
  wire [`LVC_W-1:0] lvc_wire   = {cv_wire, lv_sat_mag};

  // ---- Parity chain ---------------------------------------------------
  wire out_pc = in_pc ^ cv_wire;

  // ---- Output mux -----------------------------------------------------
  assign lvc_out    = active ? lvc_wire  : lvc_in;
  assign lv_acc_out = active ? lv_sat    : lv_acc_in;
  assign cv_out     = active ? cv_wire   : lv_acc_in[`LLR_W-1];
  assign msg_out    = active ? {out_pc, in_sc, in_min1, in_min2} : msg_in;

endmodule