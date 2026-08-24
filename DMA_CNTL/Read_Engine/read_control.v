`timescale 1ns / 1ps

module read_control #(
    parameter ADDR_WIDTH = 15
)(
    input                        clk,
    input                        rst_n,

    // DMA Register Map
    input                        start,
    output reg                   busy,
    output reg                   done,
    output reg                   error,
    output reg [31:0]            error_addr,

    // Datapath
    input                        fifo_full,
    input                        r_hs,
    input                        xfer_done,
    input      [1:0]             rresp,
    input      [ADDR_WIDTH-1:0]  err_addr,   // Error -> Where?

    // Datapath control signal
    output                       en,         // state == S_DATA
    output reg                   init        // IDLE -> DATA 진입 pulse
);

    // State
    localparam S_IDLE = 1'b0;
    localparam S_DATA = 1'b1;

    reg state;

    assign en = (state == S_DATA);

    // state register
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE;
            init  <= 1'b0;
        end
        else begin
            init <= 1'b0; // 1 pulse
            case (state)
                S_IDLE:
                    if (!fifo_full && start) begin  // remaining state & start signal
                        state <= S_DATA;            // state transition
                        init  <= 1'b1;              // initialize
                    end
                S_DATA:
                    if (xfer_done) state <= S_IDLE; // transfer done -> state transition
                default: state <= S_IDLE;
            endcase
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            busy       <= 1'b0;
            done       <= 1'b0;
            error      <= 1'b0;
            error_addr <= 32'd0;
        end
        else if (state == S_IDLE) begin
            if (!fifo_full && start) begin  // transfer start -> signal setting
                busy  <= 1'b1;
                done  <= 1'b0;
                error <= 1'b0;
            end
        end
        else begin
            if (r_hs && (rresp != 2'b00) && !error) begin             // handshake done -> error exists
                error      <= 1'b1;
                error_addr <= {{(32-ADDR_WIDTH){1'b0}}, err_addr};    // error address latch
            end
            if (xfer_done) begin                                      // handshake done -> done signal
                busy <= 1'b0;
                done <= 1'b1;
            end
        end
    end

endmodule
