`timescale 1ns / 1ps

module read_engine #(
    parameter int ADDR_WIDTH  = 15,
    parameter int DATA_WIDTH  = 32,
    parameter int LEN_WIDTH   = 32,
    parameter int BURST_WIDTH = 8
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // Register Map
    input  logic                    start,
    input  logic                    abort,     // CTRL.ABORT
    input  logic [ADDR_WIDTH-1:0]   src_addr,
    input  logic [LEN_WIDTH-1:0]    length,
    input  logic [BURST_WIDTH-1:0]  burst_cfg,
    output logic                    busy,
    output logic                    done,
    output logic                    error,
    output logic [31:0]             error_addr,

    // FIFO
    output logic                    fifo_wr_en,
    output logic [DATA_WIDTH-1:0]   fifo_wr_data,
    input  logic                    fifo_full,

    // AXI4-Full AR/R channel
    output logic [3:0]              arid,
    output logic [ADDR_WIDTH-1:0]   araddr,
    output logic [BURST_WIDTH-1:0]  arlen,
    output logic [2:0]              arsize,
    output logic [1:0]              arburst,
    output logic                    arvalid,
    input  logic                    arready,
    input  logic [DATA_WIDTH-1:0]   rdata,
    input  logic                    rvalid,
    input  logic                    rlast,
    input  logic [3:0]              rid,
    input  logic [1:0]              rresp,
    output logic                    rready
);

    logic en, init, r_hs, xfer_done;
    logic [ADDR_WIDTH-1:0] err_addr_w;

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
