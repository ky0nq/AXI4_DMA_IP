//=====================================================================
// axi4_write_controller.v
//
// Write Engine의 Controller — IDLE/DATA 2-state FSM.
// Read Engine과 달리 IDLE 진입 조건이 "start"뿐이다 (!empty를 걸면
// FIFO가 비어있는 시작 시점에 Read/Write가 서로 기다리며 데드락).
//=====================================================================
module axi4_write_controller #(
    parameter ADDR_WIDTH = 15
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // ---- Register Map I/F ----
    input  wire                    start,
    output reg                     busy,
    output reg                     done,
    output reg                     error,
    output reg  [31:0]             error_addr,

    // ---- Datapath 상태 입력 ----
    input  wire                    b_hs,
    input  wire                    xfer_done,
    input  wire [1:0]              bresp,
    input  wire [ADDR_WIDTH-1:0]   awaddr,   // 에러 시점 주소 래치용

    // ---- Datapath 제어 출력 ----
    output wire                    en,       // state == S_DATA
    output reg                     init      // IDLE -> DATA 진입 pulse
);

    localparam S_IDLE = 1'b0;
    localparam S_DATA = 1'b1;

    reg state;
    assign en = (state == S_DATA);

    //-----------------------------------------------------------
    // state register
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            init  <= 1'b0;
        end else begin
            init <= 1'b0; // 기본 1-cycle pulse
            case (state)
                S_IDLE: if (start) begin
                    state <= S_DATA;
                    init  <= 1'b1;
                end
                S_DATA: if (xfer_done) state <= S_IDLE;
            endcase
        end
    end

    //-----------------------------------------------------------
    // status / error (STATUS 레지스터로 반영)
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy       <= 1'b0;
            done       <= 1'b0;
            error      <= 1'b0;
            error_addr <= 32'd0;
        end else if (state == S_IDLE) begin
            if (start) begin
                busy  <= 1'b1;
                done  <= 1'b0;
                error <= 1'b0;
            end
        end else begin // S_DATA
            if (b_hs && (bresp != 2'b00) && !error) begin
                error      <= 1'b1;
                error_addr <= {{(32-ADDR_WIDTH){1'b0}}, awaddr};
            end
            if (xfer_done) begin
                busy <= 1'b0;
                done <= 1'b1;
            end
        end
    end

endmodule
