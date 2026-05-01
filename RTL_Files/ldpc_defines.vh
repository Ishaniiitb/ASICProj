// =============================================================================
// ldpc_defines.vh  —  IEEE 802.11ad LDPC Decoder Global Defines
// =============================================================================
`ifndef LDPC_DEFINES_VH
`define LDPC_DEFINES_VH

// Code parameters
`define N          672
`define Q          42
`define N_COLS     16
`define N_LAYERS   8
`define N_FRAMES   8
`define IO_DEPTH   16
`define MAX_ITER   10

// Bit widths
`define LLR_W      5
`define LVC_W      5
`define MAG_W      4
`define MSG_W      10

// Layer message bit positions  [9:0] = {pc, sc, min1[3:0], min2[3:0]}
`define PC_BIT     9
`define SC_BIT     8
`define MIN1_HI    7
`define MIN1_LO    4
`define MIN2_HI    3
`define MIN2_LO    0

// Code-rate encoding
`define RATE_1_2   2'b00
`define RATE_5_8   2'b01
`define RATE_3_4   2'b10
`define RATE_13_16 2'b11

// Phase encoding
`define PHASE_CN   1'b0
`define PHASE_VN   1'b1

// Address widths
`define COL_AW     4    // clog2(N_COLS)
`define VN_AW      6    // clog2(Q)=6
`define FRM_AW     3    // clog2(N_FRAMES)
`define IO_AW      4    // clog2(IO_DEPTH)
`define ITER_AW    4    // clog2(MAX_ITER+1)

`endif
