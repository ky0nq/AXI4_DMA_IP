//=====================================================================
// axi4_write_engine.v
//
// axi4_write_controller(제어)와 axi4_write_datapath(데이터패스)를
// 인스턴스화해서 연결하는 최상위 wrapper. axi4_read_engine.v와 대칭.
//=====================================================================
module axi4_write_engine #(
    parameter ADDR_WIDTH  = 15,
    parameter DATA_WIDTH  = 32,
    parameter LEN_WIDTH   = 32,
    parameter BURST_WIDTH = 8
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // ---- Register Map I/F ----
    input  wire                    start,
    input  wire [ADDR_WIDTH-1:0]   dst_addr,
    input  wire [LEN_WIDTH-1:0]    length,
    input  wire [BURST_WIDTH-1:0]  burst_cfg,
    output wire                    busy,
    output wire                    done,
    output wire                    error,
    output wire [31:0]             error_addr,

    // ---- FIFO I/F (consumer) ----
    output wire                    fifo_rd_en,
    input  wire [DATA_WIDTH-1:0]   fifo_rd_data,
    input  wire                    fifo_empty,

    // ---- AXI4 AW/W/B 채널 ----
    output wire [3:0]              awid,
    output wire [ADDR_WIDTH-1:0]   awaddr,
    output wire [BURST_WIDTH-1:0]  awlen,
    output wire [2:0]              awsize,
    output wire [1:0]              awburst,
    output wire                    awvalid,
    input  wire                    awready,

    output wire [DATA_WIDTH-1:0]   wdata,
    output wire [DATA_WIDTH/8-1:0] wstrb,
    output wire                    wlast,
    output wire                    wvalid,
    input  wire                    wready,

    input  wire [1:0]              bresp,
    input  wire                    bvalid,
    output wire                    bready
);

    wire en, init, b_hs, xfer_done;

    axi4_write_controller #(
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_ctrl (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start),
        .busy       (busy),
        .done       (done),
        .error      (error),
        .error_addr (error_addr),
        .b_hs       (b_hs),
        .xfer_done  (xfer_done),
        .bresp      (bresp),
        .awaddr     (awaddr),
        .en         (en),
        .init       (init)
    );

    axi4_write_datapath #(
        .ADDR_WIDTH  (ADDR_WIDTH),
        .DATA_WIDTH  (DATA_WIDTH),
        .LEN_WIDTH   (LEN_WIDTH),
        .BURST_WIDTH (BURST_WIDTH)
    ) u_dp (
        .clk          (clk),
        .rst_n        (rst_n),
        .en           (en),
        .init         (init),
        .dst_addr     (dst_addr),
        .length       (length),
        .burst_cfg    (burst_cfg),
        .b_hs         (b_hs),
        .xfer_done    (xfer_done),
        .fifo_rd_en   (fifo_rd_en),
        .fifo_rd_data (fifo_rd_data),
        .fifo_empty   (fifo_empty),
        .awid         (awid),
        .awaddr       (awaddr),
        .awlen        (awlen),
        .awsize       (awsize),
        .awburst      (awburst),
        .awvalid      (awvalid),
        .awready      (awready),
        .wdata        (wdata),
        .wstrb        (wstrb),
        .wlast        (wlast),
        .wvalid       (wvalid),
        .wready       (wready),
        .bresp        (bresp),
        .bvalid       (bvalid),
        .bready       (bready)
    );

endmodule
