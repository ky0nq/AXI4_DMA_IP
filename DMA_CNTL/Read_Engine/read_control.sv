`timescale 1ns / 1ps

module read_control #(
    parameter int ADDR_WIDTH = 15
)(
    input  logic                   clk,
    input  logic                   rst_n,

    // DMA Register Map
    input  logic                   start,
    output logic                   busy,
    output logic                   done,
    output logic                   error,
    output logic [31:0]            error_addr,

    // Datapath
    input  logic                   fifo_full,
    input  logic                   r_hs,
    input  logic                   xfer_done,
    input  logic [1:0]             rresp,
    input  logic [ADDR_WIDTH-1:0]  err_addr,   // Error -> Where? 

    // Datapath control signal
    output logic                   en,       // state == S_DATA
    output logic                   init      // IDLE -> DATA 진입 pulse
);

    // State 
    typedef enum logic {
        S_IDLE = 1'b0,
        S_DATA = 1'b1
    } state_e;
    state_e state;

    assign en = (state == S_DATA); 

    // state register
    always_ff @(posedge clk) begin
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

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            busy       <= 1'b0;
            done       <= 1'b0;
            error      <= 1'b0;
            error_addr <= 32'd0;
        end 
        else if (state == S_IDLE) begin
            if (!fifo_full && start) begin  // trasnfer start -> signal setting
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
