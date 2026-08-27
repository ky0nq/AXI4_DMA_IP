`timescale 1ns / 1ps

module read_engine #(
    parameter ADDR_WIDTH      = 15,
    parameter DATA_WIDTH      = 32,
    parameter LEN_WIDTH       = 32,
    parameter BURST_WIDTH     = 8,
    parameter REGION_LSB      = 14,
    parameter MAX_BURST_BYTES = 16       // 16B = 4 beats (because, MicroBlaze cache line)
)(
    input                           clk,
    input                           rst_n,

    // Register Map
    //   SRC_ADDR and LENGTH must be 4-byte aligned. The register map should
    //   reject START otherwise; the datapath also raises cfg_err as a backstop.
    //   error_addr is the address of the beat that failed, ADDR_WIDTH wide.
    //   The register map places it into the ERROR register (0x1C) at [29:15];
    //   [14:0] belongs to the Write Engine.

    input                           start,
    input                           abort,      // CTRL.ABORT
    input       [ADDR_WIDTH-1:0]    src_addr,
    input       [LEN_WIDTH-1:0]     length,
    input       [BURST_WIDTH+1:0]   burst_cfg,  // [7:0] ARLEN, [9:8] burst type
    output                          busy,
    output                          done,
    output                          error,
    output      [ADDR_WIDTH-1:0]    error_addr,

    output                          fifo_wr_en,
    output      [DATA_WIDTH-1:0]    fifo_wr_data,
    input                           fifo_full,

    // AXI4-Full AR/R channel
    output      [3:0]               arid,
    output      [ADDR_WIDTH-1:0]    araddr,
    output      [BURST_WIDTH-1:0]   arlen,
    output      [2:0]               arsize,
    output      [1:0]               arburst,
    output                          arvalid,
    input                           arready,
    input       [DATA_WIDTH-1:0]    rdata,
    input                           rvalid,
    input                           rlast,
    input       [3:0]               rid,
    input       [1:0]               rresp,
    output                          rready
);

    wire                  en, init, r_hs, xfer_done;
    wire [ADDR_WIDTH-1:0] err_addr_w;
    wire                  err_valid_w, cfg_err_w;

    read_control #(
        .ADDR_WIDTH(ADDR_WIDTH)
    ) U_READ_CNTL (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start),
        .busy       (busy),
        .done       (done),
        .error      (error),
        .error_addr (error_addr),
        .fifo_full  (fifo_full),
        .r_hs       (r_hs),
        .xfer_done  (xfer_done),
        .err_valid  (err_valid_w), 
        .cfg_err    (cfg_err_w),
        .err_addr   (err_addr_w),
        .en         (en),
        .init       (init)
    );

    read_datapath #(
        .ADDR_WIDTH      (ADDR_WIDTH),
        .DATA_WIDTH      (DATA_WIDTH),
        .LEN_WIDTH       (LEN_WIDTH),
        .BURST_WIDTH     (BURST_WIDTH),
        .REGION_LSB      (REGION_LSB),
        .MAX_BURST_BYTES (MAX_BURST_BYTES)
    ) U_READ_DATAPATH (
        .clk          (clk),
        .rst_n        (rst_n),
        .en           (en),
        .init         (init),
        .abort        (abort),
        .src_addr     (src_addr),
        .length       (length),
        .burst_cfg    (burst_cfg),
        .r_hs         (r_hs),
        .xfer_done    (xfer_done),
        .err_addr     (err_addr_w),
        .err_valid    (err_valid_w),
        .cfg_err      (cfg_err_w),
        .fifo_wr_en   (fifo_wr_en),
        .fifo_wr_data (fifo_wr_data),
        .fifo_full    (fifo_full),
        .arid         (arid),
        .araddr       (araddr),
        .arlen        (arlen),
        .arsize       (arsize),
        .arburst      (arburst),
        .arvalid      (arvalid),
        .arready      (arready),
        .rdata        (rdata),
        .rvalid       (rvalid),
        .rid          (rid),
        .rlast        (rlast),
        .rresp        (rresp),
        .rready       (rready)
    );

endmodule
