// =============================================================================
// ldpc_decoder_tb.v — Multi-Iteration Testbench
// Runs 4 test cases: 0, 1, 2, 3 injected bit errors
// Each test: full reset -> LLR load -> decode -> verify -> print metrics
// =============================================================================
`timescale 1ns/1ps
`include "ldpc_defines.vh"

module ldpc_decoder_tb;

  // ---- Clock ----------------------------------------------------------
  reg clk, rst_n;
  initial clk = 1'b0;
  always  #2.5 clk = ~clk;   // 200 MHz, T = 5 ns

  // ---- DUT ports ------------------------------------------------------
  reg         start;
  reg  [1:0]  code_rate;
  reg         llr_valid;
  reg  [`VN_AW-1:0]   llr_vn;
  reg  [`COL_AW-1:0]  llr_col;
  reg  [`IO_AW-1:0]   llr_frame;
  reg  [`LLR_W-1:0]   llr_data;
  reg         hd_req;
  reg  [`VN_AW-1:0]   hd_vn;
  reg  [`COL_AW-1:0]  hd_col;
  reg  [`IO_AW-1:0]   hd_frame;
  wire        hd_data;
  wire        done;
  wire [`ITER_AW-1:0] iter_count;

  // ---- Timing/metrics (module-level for task access) ------------------
  real    t_start_ns, t_done_ns;
  real    latency_ns;
  integer latency_cycles;
  real    throughput_gbps, fi_throughput;
  integer hd_errors;
  integer timed_out;
  integer completed_iters;   // iter_count is 0-indexed: add 1 for display

  localparam real CLK_NS = 5.0;
  localparam real N_BITS = 672.0;
  localparam real N_FRM  = 8.0;

  // Summary table storage (4 test cases)
  integer sum_nerr  [0:3];
  integer sum_iters [0:3];
  integer sum_cyc   [0:3];
  real    sum_lat   [0:3];
  real    sum_tput  [0:3];
  integer sum_hderr [0:3];
  integer sum_to    [0:3];

  // ---- DUT ------------------------------------------------------------
  ldpc_decoder_top dut (
    .clk        (clk),   .rst_n      (rst_n),
    .start      (start), .code_rate  (code_rate),
    .llr_valid  (llr_valid),
    .llr_vn     (llr_vn),    .llr_col    (llr_col),
    .llr_frame  (llr_frame), .llr_data   (llr_data),
    .hd_req     (hd_req),
    .hd_vn      (hd_vn),     .hd_col     (hd_col),
    .hd_frame   (hd_frame),  .hd_data    (hd_data),
    .done       (done),      .iter_count (iter_count)
  );

  integer col, vn, i;

  // ====================================================================
  // TASK: do_reset
  // ====================================================================
  task do_reset;
    begin
      rst_n     = 1'b0;
      start     = 1'b0;
      llr_valid = 1'b0;
      hd_req    = 1'b0;
      llr_vn    = 0; llr_col   = 0;
      llr_frame = 0; llr_data  = 0;
      hd_vn     = 0; hd_col    = 0; hd_frame = 0;
      repeat(4) @(posedge clk);
      rst_n = 1'b1;
      repeat(2) @(posedge clk);
    end
  endtask

  // ====================================================================
  // TASK: load_llrs(n_errors)
  // First n_errors positions get LLR=-7 (erroneous), rest get LLR=+7
  // ====================================================================
  task load_llrs;
    input integer n_errors;
    integer err_cnt;
    begin
      err_cnt = 0;
      for (col = 0; col < `N_COLS; col = col + 1) begin
        for (vn = 0; vn < `Q; vn = vn + 1) begin
          @(posedge clk);
          llr_valid = 1'b1;
          llr_col   = col[`COL_AW-1:0];
          llr_vn    = vn[`VN_AW-1:0];
          llr_frame = {`IO_AW{1'b0}};
          if (err_cnt < n_errors) begin
            llr_data = 5'b11001;   // -7 (erroneous bit)
            err_cnt  = err_cnt + 1;
          end else begin
            llr_data = 5'b00111;   // +7 (reliable zero)
          end
        end
      end
      @(posedge clk);
      llr_valid = 1'b0;
    end
  endtask

  // ====================================================================
  // TASK: run_decode
  // t_start captured one cycle after start (FSM entered S_CN).
  // t_done  captured at posedge done.
  // Extra @posedge clk after fork/join lets iter_count NBA settle.
  // ====================================================================
  task run_decode;
    begin
      timed_out = 0;
      @(posedge clk);
      start = 1'b1;
      @(posedge clk);
      t_start_ns = $realtime;   // FSM now in S_CN — start timing
      start = 1'b0;

      fork
        begin : f_done
          @(posedge done);
          t_done_ns = $realtime;
          disable f_timeout;
        end
        begin : f_timeout
          repeat(5000) @(posedge clk);
          timed_out = 1;
          t_done_ns = $realtime;
          disable f_done;
        end
      join

      @(posedge clk);   // let iter_count NBA update settle
      completed_iters = iter_count ;   // 0-indexed → add 1
    end
  endtask

  // ====================================================================
  // TASK: verify_hd
  // ====================================================================
  task verify_hd;
    begin
      hd_errors = 0;
      @(posedge clk);
      for (col = 0; col < `N_COLS; col = col + 1) begin
        for (vn = 0; vn < `Q; vn = vn + 1) begin
          @(posedge clk);
          hd_req   = 1'b1;
          hd_col   = col[`COL_AW-1:0];
          hd_vn    = vn[`VN_AW-1:0];
          hd_frame = {`IO_AW{1'b0}};
          @(posedge clk);
          hd_req = 1'b0;
          if (hd_data !== 1'b0)
            hd_errors = hd_errors + 1;
        end
      end
    end
  endtask

  // ====================================================================
  // TASK: print_metrics(test_idx, n_errors)
  // ====================================================================
  task print_metrics;
    input integer test_idx;
    input integer n_errors;
    begin
      latency_ns      = t_done_ns - t_start_ns;
      latency_cycles  = $rtoi(latency_ns / CLK_NS);
      throughput_gbps = N_BITS / latency_ns;
      fi_throughput   = throughput_gbps * N_FRM;

      sum_nerr [test_idx] = n_errors;
      sum_iters[test_idx] = completed_iters;
      sum_cyc  [test_idx] = latency_cycles;
      sum_lat  [test_idx] = latency_ns;
      sum_tput [test_idx] = throughput_gbps;
      sum_hderr[test_idx] = hd_errors;
      sum_to   [test_idx] = timed_out;

      $display("  Injected errors          : %0d", n_errors);
      if (timed_out) begin
        $display("  STATUS                   : TIMEOUT (no convergence in 5000 cycles)");
      end else begin
        $display("  Iterations to converge   : %0d", completed_iters);
        $display("  Decode latency           : %0d cycles  /  %.2f ns",
                 latency_cycles, latency_ns);
        $display("  Single-frame throughput  : %.3f Gb/s", throughput_gbps);
        $display("  Frame-interleaved tput   : %.3f Gb/s  (x%0d frames)",
                 fi_throughput, `N_FRAMES);
        $display("  Throughput / iteration   : %.3f Gb/s/iter",
                 throughput_gbps / $itor(completed_iters));
        if (hd_errors == 0)
          $display("  HD verification          : PASS (all %0d bits = 0)", `N);
        else
          $display("  HD verification          : FAIL (%0d residual errors)", hd_errors);
      end
    end
  endtask

  // ====================================================================
  // Main stimulus
  // ====================================================================
  initial begin
    code_rate = `RATE_13_16;

    $display("============================================================");
    $display("  IEEE 802.11ad LDPC Decoder — Multi-Iteration Test");
    $display("  Rate 13/16 | N=672 | Q=42 | MaxIter=%0d | f=200 MHz",
             `MAX_ITER);
    $display("============================================================");

    $display(""); $display("[ TEST 0 ] No errors — baseline");
    $display("------------------------------------------------------------");
    do_reset; load_llrs(0); run_decode; verify_hd; print_metrics(0, 0);

    $display(""); $display("[ TEST 1 ] 1 injected bit error");
    $display("------------------------------------------------------------");
    do_reset; load_llrs(1); run_decode; verify_hd; print_metrics(1, 1);

    $display(""); $display("[ TEST 2 ] 2 injected bit errors");
    $display("------------------------------------------------------------");
    do_reset; load_llrs(2); run_decode; verify_hd; print_metrics(2, 2);

    $display(""); $display("[ TEST 3 ] 3 injected bit errors");
    $display("------------------------------------------------------------");
    do_reset; load_llrs(3); run_decode; verify_hd; print_metrics(3, 3);

    $display("");
    $display("============================================================");
    $display("  SUMMARY TABLE");
    $display("------------------------------------------------------------");
    $display("  Test | Injected | Iters | Cycles | Latency  | Tput(Gb/s) | HD  ");
    $display("  -----|----------|-------|--------|----------|------------|-----");
    for (i = 0; i < 4; i = i + 1) begin
      if (sum_to[i])
        $display("  %4d | %8d | TIMEOUT                                    | FAIL",
                 i, sum_nerr[i]);
      else
        $display("  %4d | %8d | %5d | %6d | %5.1f ns | %6.3f     | %s",
                 i, sum_nerr[i], sum_iters[i], sum_cyc[i], sum_lat[i],
                 sum_tput[i], (sum_hderr[i]==0) ? "PASS" : "FAIL");
    end
    $display("============================================================");

    $dumpflush;
    $finish;
  end

  initial begin
    $dumpfile("ldpc_sim.vcd");
    $dumpvars(1, ldpc_decoder_tb);
  end

endmodule