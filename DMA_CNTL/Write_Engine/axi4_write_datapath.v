//=====================================================================
// axi4_write_datapath.v
//
// AXI4 DMA Write Engine의 Datapath.
//
// 목적지 주소, 전송 길이, Burst 설정을 기반으로 AXI4 Write Burst를
// 생성하고 AW/W/B 채널의 데이터 전송을 담당한다.
//
// Write Engine에서는 Master가 AWADDR/AWLEN 등의 Write Address 정보를
// 생성하고, WVALID/WLAST/WSTRB를 직접 제어한다. Write Transaction의
// 최종 완료 및 에러 여부는 B Channel의 Write Response를 통해 확인한다.
//
// 현재 설계에서는 Unaligned Address, 4KB Boundary, 가변 Burst Length,
// 동기 FIFO 연동 및 Outstanding Write 지원을 순차적으로 구현한다.
//=====================================================================
module axi4_write_datapath #(
    parameter ADDR_WIDTH  = 15,
    parameter DATA_WIDTH  = 32,
    parameter LEN_WIDTH   = 32,
    parameter BURST_WIDTH = 8
) (
    input wire clk,
    input wire rst_n,

    // ---- Controller -> Datapath ----
    input wire en,   // state == S_DATA
    input wire init, // IDLE -> DATA 진입 1cycle pulse

    // ---- Register Map 입력 ----
    input wire [ ADDR_WIDTH-1:0] dst_addr,
    input wire [  LEN_WIDTH-1:0] length,    // 총 전송 바이트 수
    input wire [BURST_WIDTH-1:0] burst_cfg,
    // 최대 Burst 설정값 (AWLEN encoding)
    // 0   -> 최대 1 beat
    // 3   -> 최대 4 beats
    // 15  -> 최대 16 beats
    // 255 -> 최대 256 beats

    // ---- Datapath -> Controller (status) ----
    output wire b_hs,
    output wire xfer_done, // aw_outstanding==0 & total>=length

    // ---- FIFO I/F (consumer) ----
    output reg                   fifo_rd_en,
    input  wire [DATA_WIDTH-1:0] fifo_rd_data,
    input  wire                  fifo_empty,

    // ---- AXI4 AW/W/B 채널 ----
    output reg  [            3:0] awid,
    output reg  [ ADDR_WIDTH-1:0] awaddr,
    output reg  [BURST_WIDTH-1:0] awlen,
    output wire [            2:0] awsize,
    output wire [            1:0] awburst,
    output reg                    awvalid,
    input  wire                   awready,

    output reg  [  DATA_WIDTH-1:0] wdata,
    output wire [DATA_WIDTH/8-1:0] wstrb,
    output reg                     wlast,
    output reg                     wvalid,
    input  wire                    wready,

    input  wire [1:0] bresp,
    input  wire       bvalid,
    output wire       bready
);

    localparam MASTER_ID = 1'b1;  // ID[3] : DMA = 1
    localparam integer BYTES_PER_BEAT = DATA_WIDTH / 8;
    localparam integer ADDR_LSB = $clog2(BYTES_PER_BEAT);

    assign awsize  = 3'b010;
    assign awburst = 2'b01;
    assign wstrb   = {(DATA_WIDTH / 8) {1'b1}};
    assign bready  = 1'b1;  // 항상 응답 수신 가능

    reg [ADDR_WIDTH-1:0] cur_addr;  // 다음 AW에 실을 주소
    reg [LEN_WIDTH-1:0] req_byte_cnt;  // 지금까지 "요청"한 바이트 수
    reg [LEN_WIDTH-1:0] total_byte_cnt;   // 지금까지 "송신"한 바이트 수
    reg aw_num;  // ID[0] : outstanding 슬롯 구분
    reg [1:0] aw_outstanding;  // 아직 B 응답 안 온 버스트 수 (0~2)
    reg [BURST_WIDTH-1:0] w_beat_cnt;       // 현재 버스트 안에서 몇 beat째인지
    reg [LEN_WIDTH-1:0] aw_payload_bytes;   // 현재 AW 요청이 실제로 담당하는 Payload Byte 수

    wire aw_hs = awvalid && awready;
    assign b_hs = bvalid && bready;
    wire w_hs = wvalid && wready;

    wire [1:0] slave_sel = cur_addr[14] ? 2'b01 : 2'b00;  // RAM:01 / ROM:00


    //-----------------------------------------------------------
    // Burst 계산
    //-----------------------------------------------------------

    // 아직 AW로 요청하지 않은 Payload Byte 수
    wire [LEN_WIDTH-1:0] remaining_bytes = (length > req_byte_cnt) ? (length - req_byte_cnt) : {LEN_WIDTH{1'b0}};


    // 현재 주소의 Word 내부 Offset
    // DATA_WIDTH = 32bit 기준 cur_addr[1:0]
    wire [ADDR_LSB-1:0] start_offset = cur_addr[ADDR_LSB-1:0];


    // 다음 4KB Boundary까지 남은 Byte 수
    // 예) cur_addr = 0x1FF0
    //     0x2000 - 0x1FF0 = 16 Byte
    wire [12:0] bytes_to_4k_raw = 13'd4096 - {1'b0, cur_addr[11:0]};

    wire [LEN_WIDTH-1:0] bytes_to_4k = {
        {(LEN_WIDTH - 13) {1'b0}}, bytes_to_4k_raw
    };


    // burst_cfg는 AWLEN encoding
    // 예) burst_cfg = 3 -> 최대 4 beats
    wire [BURST_WIDTH:0] max_burst_beats = {1'b0, burst_cfg} + 1'b1;


    // 최대 Burst가 차지할 수 있는 전체 Byte Lane
    // 예) 4 beats * 4 Byte = 16 Byte
    wire [LEN_WIDTH-1:0] max_burst_span_bytes =
        max_burst_beats * BYTES_PER_BEAT;


    // Start Unaligned를 고려한 실제 Payload 최대 크기
    //
    // 예)
    // cur_addr = 0x1002
    // max burst = 4 beats
    //
    // 전체 span = 16 Byte
    // start_offset = 2
    //
    // 실제 Payload 최대 = 16 - 2 = 14 Byte
    wire [LEN_WIDTH-1:0] max_burst_payload_bytes =
        max_burst_span_bytes - start_offset;


    // remaining / 4KB boundary 중 작은 값
    wire [LEN_WIDTH-1:0] burst_limit_1 =
        (remaining_bytes <= bytes_to_4k) ?
        remaining_bytes :
        bytes_to_4k;


    // 최종적으로 이번 Burst에서 전송할 실제 Payload Byte 수
    wire [LEN_WIDTH-1:0] actual_burst_bytes =
        (burst_limit_1 <= max_burst_payload_bytes) ?
        burst_limit_1 :
        max_burst_payload_bytes;


    // 필요한 실제 Beat 수 계산
    //
    // ceil((start_offset + actual_burst_bytes) / BYTES_PER_BEAT)
    //
    // 4Byte/Beat 기준
    // ceil(x / 4) = (x + 3) >> 2
    wire [LEN_WIDTH:0] beat_calc_value = {1'b0, actual_burst_bytes} + start_offset + (BYTES_PER_BEAT - 1);

    wire [LEN_WIDTH:0] actual_burst_beats_calc = beat_calc_value >> ADDR_LSB;


    // 최대 256 beat이므로 BURST_WIDTH+1 bit 필요
    wire [BURST_WIDTH:0] actual_burst_beats =
        actual_burst_beats_calc[BURST_WIDTH:0];


    // AXI AWLEN = Beat 수 - 1
    wire [BURST_WIDTH:0] actual_awlen_ext = actual_burst_beats - 1'b1;

    wire [BURST_WIDTH-1:0] actual_awlen = actual_awlen_ext[BURST_WIDTH-1:0];

    wire req_pending = (req_byte_cnt < length);
    wire aw_can_issue = en && !awvalid && req_pending && (aw_outstanding < 2);
    wire w_can_issue = en && !wvalid && !fifo_empty;

    assign xfer_done = b_hs && (aw_outstanding <= 2'd1) && (total_byte_cnt >= length);

    //-----------------------------------------------------------
    // cur_addr / req_byte_cnt / aw_num / aw_outstanding
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_addr       <= {ADDR_WIDTH{1'b0}};
            req_byte_cnt   <= {LEN_WIDTH{1'b0}};
            aw_num         <= 1'b0;
            aw_outstanding <= 2'd0;
        end else if (init) begin
            cur_addr       <= dst_addr;
            req_byte_cnt   <= {LEN_WIDTH{1'b0}};
            aw_num         <= 1'b0;
            aw_outstanding <= 2'd0;
        end else begin
            if (aw_hs) begin
                cur_addr     <= cur_addr + aw_payload_bytes[ADDR_WIDTH-1:0];
                req_byte_cnt <= req_byte_cnt + aw_payload_bytes;
                aw_num       <= ~aw_num;
            end
            case ({
                aw_hs, b_hs
            })
                2'b10: aw_outstanding <= aw_outstanding + 2'd1;
                2'b01: aw_outstanding <= aw_outstanding - 2'd1;
                default:
                aw_outstanding <= aw_outstanding; // 00, 11(둘 다 발생) 유지
            endcase
        end
    end

    //-----------------------------------------------------------
    // total_byte_cnt / w_beat_cnt (W 핸드셰이크마다 갱신, WLAST 생성)
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            total_byte_cnt <= {LEN_WIDTH{1'b0}};
            w_beat_cnt     <= {BURST_WIDTH{1'b0}};
        end else if (init) begin
            total_byte_cnt <= {LEN_WIDTH{1'b0}};
            w_beat_cnt     <= {BURST_WIDTH{1'b0}};
        end else if (w_hs) begin
            total_byte_cnt <= total_byte_cnt + (DATA_WIDTH / 8);
            w_beat_cnt     <= (w_beat_cnt == burst_cfg) ? {BURST_WIDTH{1'b0}}
                                                          : (w_beat_cnt + 1'b1);
        end
    end

    //-----------------------------------------------------------
    // AW 채널 구동
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            awvalid          <= 1'b0;
            awaddr           <= {ADDR_WIDTH{1'b0}};
            awlen            <= {BURST_WIDTH{1'b0}};
            awid             <= 4'd0;
            aw_payload_bytes <= {LEN_WIDTH{1'b0}};
        end else if (init) begin
            awvalid          <= 1'b0;
            awaddr           <= {ADDR_WIDTH{1'b0}};
            awlen            <= {BURST_WIDTH{1'b0}};
            awid             <= 4'd0;
            aw_payload_bytes <= {LEN_WIDTH{1'b0}};
        end else if (aw_hs) begin
            // AWVALID && AWREADY가 성립하면
            // 현재 AW 요청은 Slave에 전달 완료
            awvalid <= 1'b0;
        end else if (aw_can_issue) begin
            // Burst Calculator 결과를 AW Register에 저장
            awvalid          <= 1'b1;
            awaddr           <= cur_addr;
            awlen            <= actual_awlen;
            awid             <= {MASTER_ID, slave_sel, aw_num};

            // 이 AW가 실제 담당하는 Payload Byte 수도 같이 저장
            aw_payload_bytes <= actual_burst_bytes;
        end
    end

    //-----------------------------------------------------------
    // W 채널 구동 (FIFO -> W)
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wvalid     <= 1'b0;
            wdata      <= {DATA_WIDTH{1'b0}};
            wlast      <= 1'b0;
            fifo_rd_en <= 1'b0;
        end else begin
            fifo_rd_en <= 1'b0;  // 기본 1-cycle pulse
            if (w_hs) begin
                wvalid <= 1'b0;
            end else if (w_can_issue) begin
                wvalid <= 1'b1;
                wdata <= fifo_rd_data;
                wlast <= (w_beat_cnt == burst_cfg);
                fifo_rd_en <= 1'b1; // 이번에 꺼낸 데이터를 실었으니 다음 데이터 준비
            end
        end
    end

endmodule
