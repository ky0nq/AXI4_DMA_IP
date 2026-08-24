`timescale 1ns / 1ps

module read_engine #(
    parameter ADDR_WIDTH  = 15,
    parameter DATA_WIDTH  = 32,
    parameter LEN_WIDTH   = 32,
    parameter BURST_WIDTH = 8
)(
    input                         clk,
    input                         rst_n,

    // Register Map
    input                         start,
    input                         abort,     // CTRL.ABORT
    input  [ADDR_WIDTH-1:0]       src_addr,
    input  [LEN_WIDTH-1:0]        length,
    input  [BURST_WIDTH-1:0]      burst_cfg,
    output                        busy,
    output                        done,
    output                        error,
    output [31:0]                 error_addr,

    // FIFO
    output                        fifo_wr_en,
    output [DATA_WIDTH-1:0]       fifo_wr_data,
    input                         fifo_full,

    // AXI4-Full AR/R channel
    output [3:0]                  arid,
    output [ADDR_WIDTH-1:0]       araddr,
    output [BURST_WIDTH-1:0]      arlen,
    output [2:0]                  arsize,
    output [1:0]                  arburst,
    output                        arvalid,
    input                         arready,
    input  [DATA_WIDTH-1:0]       rdata,
    input                         rvalid,
    input                         rlast,
    input  [3:0]                  rid,
    input  [1:0]                  rresp,
    output                        rready
);

    wire                  en, init, r_hs, xfer_done;
    wire [ADDR_WIDTH-1:0] err_addr_w;

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
        .rresp      (rresp),
        .err_addr   (err_addr_w),
        .en         (en),
        .init       (init)
    );

    read_datapath #(
        .ADDR_WIDTH  (ADDR_WIDTH),
        .DATA_WIDTH  (DATA_WIDTH),
        .LEN_WIDTH   (LEN_WIDTH),
        .BURST_WIDTH (BURST_WIDTH)
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
        .rready       (rready)
    );

endmodule
