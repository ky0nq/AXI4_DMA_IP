`timescale 1ns / 1ps

module read_engine #(
    parameter ADDR_WIDTH      = 15,
    parameter DATA_WIDTH      = 32,
    parameter LEN_WIDTH       = 32,
    parameter BURST_WIDTH     = 8,
    parameter REGION_LSB      = 14,
    parameter MAX_BURST_BYTES = 16       // 16B = 4 beats (MicroBlaze cache line)
)(
    input                           clk,
    input                           rst_n,

    // Register Map
    input                           start,
    input                           abort,      // CTRL.ABORT
    input       [ADDR_WIDTH-1:0]    src_addr,
    input       [LEN_WIDTH-1:0]     length,
    input       [BURST_WIDTH+1:0]   burst_cfg,  // [7:0] ARLEN cap, [9:8] burst type
    output                          busy,
    output                          done,
    output                          error,
    output      [31:0]              error_addr,
    output      [LEN_WIDTH-1:0]     error_offset, // valid bytes before the first failure
    output      [BURST_WIDTH-1:0]   error_cnt,    // failing beat count (saturating)

    // Sideband to the Write Engine (best-effort skip hint)
    //   Not routed through the register map : the Write Engine compares
    //   rd_err_offset against the byte offset it is about to write, and
    //   drives WSTRB = 0 for anything at or beyond it. If the Write Engine
    //   has already passed that offset (FIFO nearly empty) the zeros are
    //   already in DST, which software handles via STATUS.ERROR.
    output                          rd_err_valid,
    output      [LEN_WIDTH-1:0]     rd_err_offset,

    // FIFO : 32-bit, data only
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

    // Sideband taps : same source as the register-map path, separate wires
    assign rd_err_valid  = err_valid_w;
    assign rd_err_offset = error_offset;

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
        .err_valid  (err_valid_w),   // decided by the datapath, not re-derived here
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
        .err_offset   (error_offset),
        .err_cnt      (error_cnt),
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
