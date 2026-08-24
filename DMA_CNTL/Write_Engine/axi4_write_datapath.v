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
    input wire [BURST_WIDTH+1:0] burst_cfg,
    // burst_cfg[9:8] : Burst Type
    // 00 -> FIXED
    // 01 -> INCR
    // 10 -> WRAP
    // 11 -> Reserved
    //
    // burst_cfg[7:0] : 최대 Burst 설정값 (AWLEN encoding)
    // 0   -> 최대 1 beat
    // 3   -> 최대 4 beats
    // 15  -> 최대 16 beats
    // 255 -> 최대 256 beats

    // ---- Datapath -> Controller (status) ----
    output wire b_hs,
    output wire xfer_done, // LENGTH만큼 W Data 전송 완료 + 마지막 B Response 수신 완료

    // ---- FIFO I/F (consumer) ----
    output wire                  fifo_rd_en,
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
    output reg  [DATA_WIDTH/8-1:0] wstrb,
    output reg                     wlast,
    output reg                     wvalid,
    input  wire                    wready,

    input  wire [3:0] bid,
    // B Response가 어떤 Outstanding Transaction의 응답인지 식별
    // BID[0]으로 완료된 NUM(0/1)을 확인

    input wire [1:0] bresp,
    input wire bvalid,
    output wire bready
);

    localparam MASTER_ID = 1'b1;  // ID[3] : DMA = 1
    localparam integer BYTES_PER_BEAT = DATA_WIDTH / 8;  // 4
    localparam integer ADDR_LSB = $clog2(BYTES_PER_BEAT);  // 2

    //-----------------------------------------------------------
    // Burst Configuration Decode
    //-----------------------------------------------------------
    // burst_cfg[9:8] : Burst Type
    // 00 : FIXED
    // 01 : INCR
    // 10 : WRAP
    // 11 : Reserved
    //
    // burst_cfg[7:0] : 최대 Burst Length
    // AWLEN encoding
    //-----------------------------------------------------------

    localparam [1:0] BURST_FIXED = 2'b00;
    localparam [1:0] BURST_INCR  = 2'b01;
    localparam [1:0] BURST_WRAP  = 2'b10;

    wire [1:0] burst_type =
        burst_cfg[BURST_WIDTH+1:BURST_WIDTH];

    wire [BURST_WIDTH-1:0] burst_len_cfg =
        burst_cfg[BURST_WIDTH-1:0];

    assign awsize  = 3'b010;
    assign awburst = burst_type;
    assign bready  = 1'b1;  // 항상 응답 수신 가능

    reg [ADDR_WIDTH-1:0] cur_addr;  // 다음 AW에 실을 주소
    reg [LEN_WIDTH-1:0] req_byte_cnt;  // 지금까지 "요청"한 바이트 수
    reg [LEN_WIDTH-1:0] total_byte_cnt;   // 지금까지 "송신"한 바이트 수

    // Outstanding Transaction의 NUM 사용 상태
    // bit 0 : NUM 0
    // bit 1 : NUM 1
    // 0 = FREE, 1 = BUSY
    reg [1:0] num_busy;
    reg [1:0] aw_outstanding;  // 아직 B 응답 안 온 버스트 수 (0~2)

    reg [BURST_WIDTH-1:0] w_beat_cnt;       // 현재 버스트 안에서 몇 beat째인지
    reg [LEN_WIDTH-1:0] aw_payload_bytes;   // 현재 AW 요청이 실제로 담당하는 Payload Byte 수

    // 현재 Descriptor의 Burst 안에서 실제 전송 완료한 Payload Byte 수
    reg [LEN_WIDTH-1:0] burst_byte_cnt;
    //→ 현재 Burst 안에서 지금까지 실제로 전송 완료된 Byte 누적량

    // 현재 W Register에 실려 있는 Beat의 실제 유효 Byte 수
    reg [LEN_WIDTH-1:0] w_valid_bytes;
    //→ 지금 WVALID로 내보내고 있는 한 Beat에 몇 Byte가 유효한지

    //-----------------------------------------------------------
    // Burst Descriptor Queue
    //-----------------------------------------------------------
    // AW Handshake가 완료된 Burst 정보를 저장
    // W Channel은 저장된 Descriptor를 순서대로 사용
    // Outstanding 최대 2개이므로 Descriptor도 2-entry로 구성


    reg [BURST_WIDTH-1:0] desc_awlen         [0:1];
    //→ 실제 Burst 길이
    //desc_awlen
    //→ 나중에 WLAST 발생 위치 계산
    
    reg [LEN_WIDTH-1:0]   desc_payload_bytes [0:1];
    //desc_payload_bytes
    //→ 실제 Payload Byte 수
    //→ 나중에 WSTRB / 전송 Byte Counter 계산
    
    reg [ADDR_LSB-1:0]    desc_start_offset  [0:1];
    //desc_start_offset
    //→ 첫 Beat의 시작 위치
    //→ 나중에 Data Aligner / WSTRB 계산

    reg desc_wr_ptr;       // 다음 Descriptor 저장 위치
    //desc_wr_ptr
    //→ AW가 새 Descriptor를 어디에 쓸지
    
    reg desc_rd_ptr;       // 현재 W Channel이 사용할 Descriptor 위치
    //desc_rd_ptr
    //→ W가 현재 어떤 Descriptor를 읽을지
    
    reg [1:0] desc_count;  // 저장된 Descriptor 개수 (0~2)
    //desc_count
    //→ Queue에 Descriptor가 몇 개 있는지

    wire aw_hs = awvalid && awready;
    assign b_hs = bvalid && bready;
    wire w_hs = wvalid && wready;

    wire [1:0] slave_sel = cur_addr[14] ? 2'b01 : 2'b00;  // ROM:00 / RAM:01, UART:10은 추후 주소 Decode 확장

    //-----------------------------------------------------------
    // FIFO Read Pipeline / Data Alignment Buffer
    //-----------------------------------------------------------
    // 동기 FIFO는 rd_en 이후 다음 Clock에 rd_data가 유효하다.
    // 1 Beat/Clock 처리를 위해 FIFO 반환 데이터를 Alignment Buffer로
    // 바로 저장하고, W Channel이 사용할 Payload Byte Stream을 유지한다.
    //
    // Alignment Buffer는 3개의 FIFO Word를 저장할 수 있도록 구성한다.
    // DATA_WIDTH = 32bit 기준 12 Byte이며,
    // 현재 Payload와 이미 요청되어 다음 Clock에 들어올 FIFO Word까지
    // 수용할 수 있도록 여유 공간을 둔다.
    //-----------------------------------------------------------

    localparam integer ALIGN_BUF_WORDS = 3;
    localparam integer ALIGN_BUF_WIDTH = DATA_WIDTH * ALIGN_BUF_WORDS;
    localparam integer ALIGN_BUF_BYTES = BYTES_PER_BEAT * ALIGN_BUF_WORDS;
    localparam integer ALIGN_CNT_W = $clog2(ALIGN_BUF_BYTES + 1);

    // 이전 Clock의 FIFO Read 요청 여부
    // 1이면 현재 Clock의 fifo_rd_data가 유효하다.
    reg fifo_read_pending;

    // 이번 DMA에서 FIFO에 요청한 Word 수
    // 마지막 Partial Word도 FIFO에서는 32bit Word 하나로 읽는다.
    reg [LEN_WIDTH-1:0] fifo_word_req_cnt;

    // FIFO Payload Byte Stream 저장
    // align_buf[7:0]부터 가장 오래된 Payload Byte가 위치한다.
    reg [ALIGN_BUF_WIDTH-1:0] align_buf;

    // align_buf 안에 현재 유효한 Payload Byte 수
    // DATA_WIDTH = 32bit 기준 0~12 Byte
    reg [ALIGN_CNT_W-1:0] align_byte_count;

    //-----------------------------------------------------------
    // Burst 계산
    //-----------------------------------------------------------
    // Burst Type별 Calculator를 분리하고
    // 아래 Burst Mode Selector에서 실제 사용할 결과를 선택한다.
    //
    // 현재는 INCR Burst Calculator만 구현되어 있다.
    // FIXED / WRAP Burst Calculator는 추후 구현 예정이다.
    //-----------------------------------------------------------


    //-----------------------------------------------------------
    // INCR Burst Calculator
    //-----------------------------------------------------------

    // 아직 AW로 요청하지 않은 Payload Byte 수
    wire [LEN_WIDTH-1:0] incr_remaining_bytes = (length > req_byte_cnt) ? (length - req_byte_cnt) : {LEN_WIDTH{1'b0}};
    // 총 전송할 byte수가 지금까지 요청한 byte수 보다 크면? 총 전송할 byte - 요청한 byte, 아니면? zero extension

    // 현재 주소의 Word 내부 Offset
    // DATA_WIDTH = 32bit 기준 cur_addr[1:0]
    wire [ADDR_LSB-1:0] incr_start_offset = cur_addr[ADDR_LSB-1:0];


    // 다음 4KB Boundary까지 남은 Byte 수
    // 예) cur_addr = 0x1FF0
    //     0x2000 - 0x1FF0 = 16 Byte
    wire [12:0] incr_bytes_to_4k_raw = 13'd4096 - {1'b0, cur_addr[11:0]}; // 4096(4KB) - 다음 AW에 실을 주소(현재 주소)

    wire [LEN_WIDTH-1:0] incr_bytes_to_4k = {{(LEN_WIDTH - 13) {1'b0}}, incr_bytes_to_4k_raw};


    // burst_len_cfg는 AWLEN encoding
    // 예) burst_len_cfg = 3 -> 최대 4 beats
    wire [BURST_WIDTH:0] incr_max_burst_beats = {1'b0, burst_len_cfg} + 1'b1;


    // 최대 Burst가 차지할 수 있는 전체 Byte Lane
    // 예) 4 beats * 4 Byte = 16 Byte
    wire [LEN_WIDTH-1:0] incr_max_burst_span_bytes = incr_max_burst_beats * BYTES_PER_BEAT;


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
    wire [LEN_WIDTH-1:0] incr_max_burst_payload_bytes = incr_max_burst_span_bytes - incr_start_offset;


    // remaining / 4KB boundary 중 작은 값
    wire [LEN_WIDTH-1:0] incr_burst_limit_1 = (incr_remaining_bytes <= incr_bytes_to_4k) ? incr_remaining_bytes : incr_bytes_to_4k;


    // 최종적으로 이번 Burst에서 전송할 실제 Payload Byte 수
    wire [LEN_WIDTH-1:0] incr_actual_burst_bytes = (incr_burst_limit_1 <= incr_max_burst_payload_bytes) ? incr_burst_limit_1 : incr_max_burst_payload_bytes;


    // 필요한 실제 Beat 수 계산
    //
    // ceil((start_offset + actual_burst_bytes) / BYTES_PER_BEAT)
    //
    // 4Byte/Beat 기준
    // ceil(x / 4) = (x + 3) >> 2
    wire [LEN_WIDTH:0] incr_beat_calc_value = {1'b0, incr_actual_burst_bytes} + incr_start_offset + (BYTES_PER_BEAT - 1);

    wire [LEN_WIDTH:0] incr_actual_burst_beats_calc = incr_beat_calc_value >> ADDR_LSB;


    // 최대 256 beat이므로 BURST_WIDTH+1 bit 필요
    wire [BURST_WIDTH:0] incr_actual_burst_beats = incr_actual_burst_beats_calc[BURST_WIDTH:0];


    // AXI AWLEN = Beat 수 - 1
    wire [BURST_WIDTH:0] incr_actual_awlen_ext = incr_actual_burst_beats - 1'b1;

    wire [BURST_WIDTH-1:0] incr_actual_awlen = incr_actual_awlen_ext[BURST_WIDTH-1:0];


    //-----------------------------------------------------------
    // Burst Mode Selector
    //-----------------------------------------------------------
    // 각 Burst Type Calculator의 결과 중
    // 현재 burst_type에 해당하는 값을 선택한다.
    //
    // FIXED / WRAP은 아직 미구현이므로 burst_calc_valid = 0을 유지한다.
    // 따라서 해당 Mode에서는 AW Transaction이 발생하지 않는다.
    //-----------------------------------------------------------

    reg [LEN_WIDTH-1:0]   actual_burst_bytes;
    reg [BURST_WIDTH-1:0] actual_awlen;
    reg                   burst_calc_valid;

    always @(*) begin
        // Default
        actual_burst_bytes = {LEN_WIDTH{1'b0}};
        actual_awlen        = {BURST_WIDTH{1'b0}};
        burst_calc_valid    = 1'b0;

        case (burst_type)

            //---------------------------------------------------
            // FIXED Burst
            //---------------------------------------------------
            BURST_FIXED: begin
                // TODO:
                // FIXED Burst Calculator 구현 예정
            end


            //---------------------------------------------------
            // INCR Burst
            //---------------------------------------------------
            BURST_INCR: begin
                actual_burst_bytes = incr_actual_burst_bytes;
                actual_awlen        = incr_actual_awlen;
                burst_calc_valid    = 1'b1;
            end


            //---------------------------------------------------
            // WRAP Burst
            //---------------------------------------------------
            BURST_WRAP: begin
                // TODO:
                // WRAP Burst Calculator 구현 예정
            end


            //---------------------------------------------------
            // Reserved
            //---------------------------------------------------
            default: begin
                // 지원하지 않는 Burst Type
                // 현재는 AW 발행하지 않음
            end

        endcase
    end


    wire req_pending = (req_byte_cnt < length);

    wire num_available = (num_busy != 2'b11);
    // 하나 이상의 NUM이 비어 있는지 확인

    //-----------------------------------------------------------
    // Descriptor Queue 상태
    //-----------------------------------------------------------

    wire desc_empty = (desc_count == 2'd0);
    wire desc_full  = (desc_count == 2'd2);
    wire desc_push = aw_hs;
    wire desc_pop  = w_hs && wlast && !desc_empty;
    
    wire [BURST_WIDTH-1:0] current_desc_awlen;
    wire [LEN_WIDTH-1:0]   current_desc_payload_bytes;
    wire [ADDR_LSB-1:0]    current_desc_start_offset;

    assign current_desc_awlen = desc_awlen[desc_rd_ptr];

    assign current_desc_payload_bytes = desc_payload_bytes[desc_rd_ptr];

    assign current_desc_start_offset = desc_start_offset[desc_rd_ptr];

    //-----------------------------------------------------------
    // FIFO Read Word Count
    //-----------------------------------------------------------
    // FIFO는 32bit Word 단위로 데이터를 전달한다.
    // Read Engine은 마지막 Partial Word의 사용하지 않는 Byte를
    // Zero Padding해서 FIFO에 저장한다고 가정한다.
    //
    // 예)
    // length = 4 -> 1 Word
    // length = 5 -> 2 Words
    // length = 6 -> 2 Words
    // length = 8 -> 2 Words
    //-----------------------------------------------------------

    wire [LEN_WIDTH:0] fifo_total_words_calc =
        ({1'b0, length} + (BYTES_PER_BEAT - 1)) >> ADDR_LSB;

    wire [LEN_WIDTH-1:0] fifo_total_words =
        fifo_total_words_calc[LEN_WIDTH-1:0];

    wire fifo_words_remaining =
        (fifo_word_req_cnt < fifo_total_words);

    // 이전 Clock의 FIFO Read 요청에 대한 Data Return
    wire fifo_return = fifo_read_pending;


    //-----------------------------------------------------------
    // Next W Register Load Context
    //-----------------------------------------------------------
    // W Register가 비어 있거나 현재 W Beat가 Handshake되는 경우
    // 다음 Beat를 같은 Clock Edge에서 바로 적재할 수 있다.
    //-----------------------------------------------------------

    wire w_slot_available = !wvalid || w_hs;

    // 현재 W Beat가 Burst의 마지막 Beat이면 다음 Descriptor로 이동
    wire move_to_next_desc = w_hs && wlast;

    // 현재 Descriptor 뒤에 다음 Descriptor가 이미 Queue에 존재하는지 확인
    wire next_desc_queued = (desc_count >= 2'd2);

    // 현재 Descriptor 하나만 남은 상태에서 같은 Clock에 AW Handshake가
    // 발생하면 Queue Write 완료 전의 AW Register 값을 직접 사용한다.
    wire next_desc_bypass =
        (desc_count == 2'd1) && desc_push;

    wire load_desc_available =
        move_to_next_desc
        ? (next_desc_queued || next_desc_bypass)
        : !desc_empty;


    //-----------------------------------------------------------
    // 다음 W Beat가 사용할 Descriptor 정보
    //-----------------------------------------------------------

    wire [BURST_WIDTH-1:0] load_desc_awlen =
        move_to_next_desc
        ? (next_desc_queued ? desc_awlen[~desc_rd_ptr] : awlen)
        : current_desc_awlen;

    wire [LEN_WIDTH-1:0] load_desc_payload_bytes =
        move_to_next_desc
        ? (next_desc_queued ? desc_payload_bytes[~desc_rd_ptr]
                            : aw_payload_bytes)
        : current_desc_payload_bytes;

    wire [ADDR_LSB-1:0] load_desc_start_offset =
        move_to_next_desc
        ? (next_desc_queued ? desc_start_offset[~desc_rd_ptr]
                            : awaddr[ADDR_LSB-1:0])
        : current_desc_start_offset;


    //-----------------------------------------------------------
    // 다음 W Beat 번호 / Burst 내부 Payload 진행량
    //-----------------------------------------------------------

    wire [BURST_WIDTH-1:0] load_w_beat_cnt =
        move_to_next_desc
        ? {BURST_WIDTH{1'b0}}
        : (w_hs ? (w_beat_cnt + 1'b1) : w_beat_cnt);

    wire [LEN_WIDTH-1:0] load_burst_byte_cnt =
        move_to_next_desc
        ? {LEN_WIDTH{1'b0}}
        : (w_hs ? (burst_byte_cnt + w_valid_bytes)
                : burst_byte_cnt);


    //-----------------------------------------------------------
    // 다음 W Beat 정보
    //-----------------------------------------------------------

    // 현재 Burst의 첫 번째 Beat인지
    wire load_first_beat =
        (load_w_beat_cnt == {BURST_WIDTH{1'b0}});

    // 현재 Burst의 마지막 Beat인지
    wire load_last_beat =
        (load_w_beat_cnt == load_desc_awlen);

    // Descriptor의 Start Offset을 LEN_WIDTH로 확장
    wire [LEN_WIDTH-1:0] load_start_offset_ext =
        {{(LEN_WIDTH - ADDR_LSB) {1'b0}}, load_desc_start_offset};

    // 현재 Beat에서 데이터를 쓰기 시작할 Byte Lane
    // First Beat  : Descriptor의 Start Offset 사용
    // 그 이후    : Lane 0부터 사용
    wire [LEN_WIDTH-1:0] load_beat_start_lane =
        load_first_beat
        ? load_start_offset_ext
        : {LEN_WIDTH{1'b0}};

    // 현재 Beat에서 사용 가능한 Byte Lane 수
    //
    // DATA_WIDTH = 32bit 기준
    // Start Lane = 0 -> 4 Byte 사용 가능
    // Start Lane = 1 -> 3 Byte 사용 가능
    // Start Lane = 2 -> 2 Byte 사용 가능
    // Start Lane = 3 -> 1 Byte 사용 가능
    wire [LEN_WIDTH-1:0] load_beat_capacity_bytes =
        BYTES_PER_BEAT - load_beat_start_lane;

    // 현재 Descriptor에서 아직 실제 전송하지 않은 Payload Byte 수
    wire [LEN_WIDTH-1:0] load_burst_bytes_remaining =
        (load_desc_payload_bytes > load_burst_byte_cnt)
        ? (load_desc_payload_bytes - load_burst_byte_cnt)
        : {LEN_WIDTH{1'b0}};

    // 이번 W Beat에서 실제로 전송할 Payload Byte 수
    wire [LEN_WIDTH-1:0] load_valid_bytes =
        (load_burst_bytes_remaining <= load_beat_capacity_bytes)
        ? load_burst_bytes_remaining
        : load_beat_capacity_bytes;


    //-----------------------------------------------------------
    // Alignment Buffer Next-State 계산
    //-----------------------------------------------------------
    // 한 Clock에서 다음 두 동작을 동시에 처리한다.
    //
    // 1. 현재 W Beat Handshake
    //    -> 실제 사용한 Payload Byte 제거
    //
    // 2. 이전 Clock에 요청한 FIFO Data Return
    //    -> 새로운 4Byte Word를 Payload Stream 뒤에 추가
    //-----------------------------------------------------------

    wire [ALIGN_CNT_W-1:0] align_consume_bytes =
        w_hs
        ? w_valid_bytes[ALIGN_CNT_W-1:0]
        : {ALIGN_CNT_W{1'b0}};

    // W Handshake가 발생하면 먼저 사용한 Payload를 제거한다.
    wire [ALIGN_BUF_WIDTH-1:0] align_buf_after_consume =
        w_hs
        ? (align_buf >> (w_valid_bytes * 8))
        : align_buf;

    wire [ALIGN_CNT_W-1:0] align_count_after_consume =
        align_byte_count - align_consume_bytes;

    // FIFO Data Width를 Alignment Buffer Width로 확장
    wire [ALIGN_BUF_WIDTH-1:0] fifo_return_data_ext =
        {{(ALIGN_BUF_WIDTH - DATA_WIDTH){1'b0}}, fifo_rd_data};

    // Consume 후 남은 Payload 뒤에 FIFO Return Data를 추가한다.
    wire [ALIGN_BUF_WIDTH-1:0] align_buf_after_update =
        fifo_return
        ? (align_buf_after_consume |
           (fifo_return_data_ext << (align_count_after_consume * 8)))
        : align_buf_after_consume;

    wire [ALIGN_CNT_W-1:0] align_count_after_update =
        align_count_after_consume +
        (fifo_return ? BYTES_PER_BEAT : 0);


    //-----------------------------------------------------------
    // FIFO Prefetch / Buffer Credit
    //-----------------------------------------------------------
    // 현재 Clock의 Consume 및 FIFO Return을 반영한 이후에도
    // 다음 FIFO Word 4Byte를 받을 공간이 있으면 Read를 미리 요청한다.
    //
    // WVALID 여부와 관계없이 Prefetch할 수 있으므로
    // 현재 W Beat를 보내는 동안 다음 FIFO Word를 준비할 수 있다.
    //
    // 현재 구현된 Burst Mode에서만 FIFO를 소비하도록
    // burst_calc_valid가 1일 때 Read를 허용한다.
    //-----------------------------------------------------------

    wire fifo_credit_available =
        (align_count_after_update <=
         (ALIGN_BUF_BYTES - BYTES_PER_BEAT));

    wire fifo_read_req =
        en &&
        !init &&
        burst_calc_valid &&
        !fifo_empty &&
        fifo_words_remaining &&
        fifo_credit_available;

    assign fifo_rd_en = fifo_read_req;


    //-----------------------------------------------------------
    // Next WDATA Generator
    //-----------------------------------------------------------
    // 현재 W Beat Consume과 FIFO Return까지 반영된 최신 Payload Stream에서
    // 가장 오래된 DATA_WIDTH만 사용한다.
    //-----------------------------------------------------------

    wire [DATA_WIDTH-1:0] load_payload_word =
        align_buf_after_update[DATA_WIDTH-1:0];

    // 현재 Beat의 시작 Byte Lane에 맞게 Payload를 이동
    // 예) start_lane = 2이면 Payload를 16bit Shift
    wire [DATA_WIDTH-1:0] load_aligned_wdata =
        load_payload_word << (load_beat_start_lane * 8);

    // 다음 W Beat에 필요한 Payload Byte가 모두 준비됐는지 확인
    wire [LEN_WIDTH-1:0] align_count_after_update_ext =
        {{(LEN_WIDTH - ALIGN_CNT_W){1'b0}}, align_count_after_update};

    wire load_data_ready =
        (load_valid_bytes != {LEN_WIDTH{1'b0}}) &&
        (align_count_after_update_ext >= load_valid_bytes);

    wire w_load_req =
        en &&
        w_slot_available &&
        load_desc_available &&
        load_data_ready;


    wire aw_can_issue = en && !awvalid && req_pending && (aw_outstanding < 2) && num_available && !desc_full && burst_calc_valid;

    //-----------------------------------------------------------
    // DMA Write Transfer 완료 판단
    //-----------------------------------------------------------

    // 전체 LENGTH만큼 W Data 전송 완료
    wire payload_done = (total_byte_cnt >= length);

    // 현재 B Handshake가 마지막 Outstanding Transaction의 응답인지
    //
    // aw_outstanding는 Clock Edge 이전 값이므로
    // 현재 1개가 남아 있고 B Handshake가 발생하면
    // 이 응답 이후 Outstanding = 0이 된다.
    wire last_b_hs = b_hs && (aw_outstanding == 2'd1);

    // 전체 Payload 전송 완료
    // +
    // 마지막 Outstanding B Response 수신 완료
    assign xfer_done = payload_done && last_b_hs;

    //-----------------------------------------------------------
    // AW NUM Allocator
    //-----------------------------------------------------------

    // FREE 상태인 NUM을 선택
    // 둘 다 FREE인 경우 NUM 0을 우선 사용
    wire selected_num = (!num_busy[0]) ? 1'b0 : (!num_busy[1]) ? 1'b1 : 1'b0;

    //-----------------------------------------------------------
    // cur_addr / req_byte_cnt / NUM / aw_outstanding
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_addr       <= {ADDR_WIDTH{1'b0}};
            req_byte_cnt   <= {LEN_WIDTH{1'b0}};
            num_busy       <= 2'b00;
            aw_outstanding <= 2'd0;
        end else if (init) begin
            cur_addr       <= dst_addr;
            req_byte_cnt   <= {LEN_WIDTH{1'b0}};
            num_busy       <= 2'b00;
            aw_outstanding <= 2'd0;
        end else begin
            // AW Handshake 시 다음 Burst 주소 준비
            if (aw_hs) begin
                cur_addr     <= cur_addr + aw_payload_bytes[ADDR_WIDTH-1:0];
                req_byte_cnt <= req_byte_cnt + aw_payload_bytes;
            end
            // B Response가 완료된 NUM은 FREE
            if (b_hs) num_busy[bid[0]] <= 1'b0;
            // 새 AW가 사용한 NUM은 BUSY
            if (aw_hs) num_busy[awid[0]] <= 1'b1;
            // Outstanding 개수 관리
            case ({
                aw_hs, b_hs
            })
                2'b10:   aw_outstanding <= aw_outstanding + 2'd1;
                2'b01:   aw_outstanding <= aw_outstanding - 2'd1;
                default: aw_outstanding <= aw_outstanding;
            endcase
        end
    end

    //-----------------------------------------------------------
    // Burst Descriptor Queue
    //-----------------------------------------------------------
    // Push : AW Handshake
    // Pop  : 해당 Burst의 마지막 W Beat Handshake
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            desc_wr_ptr <= 1'b0;
            desc_rd_ptr <= 1'b0;
            desc_count  <= 2'd0;

            desc_awlen[0]         <= {BURST_WIDTH{1'b0}};
            desc_awlen[1]         <= {BURST_WIDTH{1'b0}};

            desc_payload_bytes[0] <= {LEN_WIDTH{1'b0}};
            desc_payload_bytes[1] <= {LEN_WIDTH{1'b0}};

            desc_start_offset[0]  <= {ADDR_LSB{1'b0}};
            desc_start_offset[1]  <= {ADDR_LSB{1'b0}};
        end
        else if (init) begin
            desc_wr_ptr <= 1'b0;
            desc_rd_ptr <= 1'b0;
            desc_count  <= 2'd0;

            desc_awlen[0]         <= {BURST_WIDTH{1'b0}};
            desc_awlen[1]         <= {BURST_WIDTH{1'b0}};

            desc_payload_bytes[0] <= {LEN_WIDTH{1'b0}};
            desc_payload_bytes[1] <= {LEN_WIDTH{1'b0}};

            desc_start_offset[0]  <= {ADDR_LSB{1'b0}};
            desc_start_offset[1]  <= {ADDR_LSB{1'b0}};
        end
        else begin

            //---------------------------------------------------
            // Descriptor Push
            //---------------------------------------------------
            if (desc_push) begin
                desc_awlen[desc_wr_ptr] <= awlen;

                desc_payload_bytes[desc_wr_ptr]
                    <= aw_payload_bytes;

                desc_start_offset[desc_wr_ptr]
                    <= awaddr[ADDR_LSB-1:0];

                desc_wr_ptr <= ~desc_wr_ptr;
            end

            //---------------------------------------------------
            // Descriptor Pop
            //---------------------------------------------------
            if (desc_pop) begin
                desc_rd_ptr <= ~desc_rd_ptr;
            end

            //---------------------------------------------------
            // Descriptor 개수 관리
            //---------------------------------------------------
            case ({desc_push, desc_pop})

                // Push만 발생
                2'b10:
                    desc_count <= desc_count + 2'd1;

                // Pop만 발생
                2'b01:
                    desc_count <= desc_count - 2'd1;

                // Push/Pop 동시 발생 또는 아무것도 없음
                default:
                    desc_count <= desc_count;
            endcase
        end
    end

    //-----------------------------------------------------------
    // Synchronous FIFO Read Pipeline
    //-----------------------------------------------------------
    // fifo_rd_en(N)
    //      ↓
    // fifo_rd_data(N+1)
    //
    // fifo_read_pending은 1Clock Pipeline Valid 역할을 한다.
    // 연속 Clock에서 fifo_rd_en이 1이면 FIFO Word도 매 Clock 반환될 수 있다.
    //-----------------------------------------------------------

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_read_pending <= 1'b0;
            fifo_word_req_cnt <= {LEN_WIDTH{1'b0}};
        end
        else if (init) begin
            fifo_read_pending <= 1'b0;
            fifo_word_req_cnt <= {LEN_WIDTH{1'b0}};
        end
        else begin

            //---------------------------------------------------
            // 현재 Read Request는 다음 Clock의 Return Valid
            //---------------------------------------------------
            fifo_read_pending <= fifo_read_req;

            //---------------------------------------------------
            // FIFO에 실제 요청한 Word 수
            //---------------------------------------------------
            if (fifo_read_req) begin
                fifo_word_req_cnt <= fifo_word_req_cnt + 1'b1;
            end
        end
    end

    //-----------------------------------------------------------
    // Data Alignment Buffer
    //-----------------------------------------------------------
    // FIFO Word Push:
    //   새로운 FIFO Word를 현재 Payload Byte Stream 뒤에 붙인다.
    //
    // W Handshake:
    //   실제 전송된 Payload Byte 수만큼 앞쪽 Byte를 제거한다.
    //
    // Push와 Consume은 같은 Clock에 동시에 처리할 수 있다.
    // Alignment Buffer는 Descriptor가 끝나더라도 초기화하지 않는다.
    // 남은 Payload Byte가 다음 Descriptor에서 계속 사용될 수 있기 때문이다.
    //-----------------------------------------------------------

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            align_buf        <= {ALIGN_BUF_WIDTH{1'b0}};
            align_byte_count <= {ALIGN_CNT_W{1'b0}};
        end
        else if (init) begin
            align_buf        <= {ALIGN_BUF_WIDTH{1'b0}};
            align_byte_count <= {ALIGN_CNT_W{1'b0}};
        end
        else begin
            align_buf        <= align_buf_after_update;
            align_byte_count <= align_count_after_update;
        end
    end

    //-----------------------------------------------------------
    // Next WSTRB Generator
    //-----------------------------------------------------------
    // load_beat_start_lane부터 load_valid_bytes 개수만큼
    // Byte Lane을 1로 설정한다.
    //
    // 예)
    // start=0, valid=4 -> 1111
    // start=2, valid=2 -> 1100
    // start=0, valid=2 -> 0011
    // start=1, valid=2 -> 0110
    //-----------------------------------------------------------

    integer strb_idx;

    reg [DATA_WIDTH/8-1:0] load_wstrb;

    always @(*) begin
        load_wstrb = {(DATA_WIDTH / 8) {1'b0}};

        for (
            strb_idx = 0;
            strb_idx < BYTES_PER_BEAT;
            strb_idx = strb_idx + 1
        ) begin

            if ((strb_idx >= load_beat_start_lane) &&
                (strb_idx <
                 (load_beat_start_lane + load_valid_bytes))) begin

                load_wstrb[strb_idx] = 1'b1;
            end
        end
    end

    //-----------------------------------------------------------
    // W Beat / Payload Byte Counter
    //-----------------------------------------------------------
    // Counter는 WVALID && WREADY Handshake가 실제 발생했을 때만
    // 증가한다.
    //
    // total_byte_cnt
    // → DMA 전체에서 실제 전송 완료한 Payload Byte 수
    //
    // burst_byte_cnt
    // → 현재 Descriptor 안에서 실제 전송 완료한 Payload Byte 수
    //
    // w_beat_cnt
    // → 현재 Burst의 Beat 번호
    //-----------------------------------------------------------

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            total_byte_cnt <= {LEN_WIDTH{1'b0}};
            burst_byte_cnt <= {LEN_WIDTH{1'b0}};
            w_beat_cnt     <= {BURST_WIDTH{1'b0}};
        end
        else if (init) begin
            total_byte_cnt <= {LEN_WIDTH{1'b0}};
            burst_byte_cnt <= {LEN_WIDTH{1'b0}};
            w_beat_cnt     <= {BURST_WIDTH{1'b0}};
        end
        else if (w_hs) begin

            //---------------------------------------------------
            // 전체 DMA Payload Byte Counter
            //---------------------------------------------------
            // 현재 W Beat에서 실제 유효했던 Byte 수만 증가
            total_byte_cnt <=
                total_byte_cnt + w_valid_bytes;


            //---------------------------------------------------
            // 현재 Burst의 마지막 Beat
            //---------------------------------------------------
            if (wlast) begin

                // 다음 Descriptor는 새로운 Burst이므로
                // Burst 내부 Counter 초기화
                burst_byte_cnt <= {LEN_WIDTH{1'b0}};
                w_beat_cnt     <= {BURST_WIDTH{1'b0}};
            end

            //---------------------------------------------------
            // 아직 현재 Burst가 끝나지 않음
            //---------------------------------------------------
            else begin

                burst_byte_cnt <=
                    burst_byte_cnt + w_valid_bytes;

                w_beat_cnt <=
                    w_beat_cnt + 1'b1;
            end
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
            awid             <= {MASTER_ID, slave_sel, selected_num};

            // 이 AW가 실제 담당하는 Payload Byte 수도 같이 저장
            aw_payload_bytes <= actual_burst_bytes;
        end
    end

    //-----------------------------------------------------------
    // W Channel
    //-----------------------------------------------------------
    // 현재 Beat가 Handshake되는 Clock에 다음 Beat가 준비되어 있으면
    // W Register를 바로 교체한다.
    //
    // WREADY=1이고 Payload/Descriptor가 충분한 경우
    // WVALID을 계속 1로 유지하면서 매 Clock 1 Beat 전송을 목표로 한다.
    //-----------------------------------------------------------

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wvalid        <= 1'b0;
            wdata         <= {DATA_WIDTH{1'b0}};
            wstrb         <= {(DATA_WIDTH / 8) {1'b0}};
            wlast         <= 1'b0;
            w_valid_bytes <= {LEN_WIDTH{1'b0}};
        end
        else if (init) begin
            wvalid        <= 1'b0;
            wdata         <= {DATA_WIDTH{1'b0}};
            wstrb         <= {(DATA_WIDTH / 8) {1'b0}};
            wlast         <= 1'b0;
            w_valid_bytes <= {LEN_WIDTH{1'b0}};
        end
        else begin

            //---------------------------------------------------
            // 다음 W Beat를 즉시 적재
            //---------------------------------------------------
            // 현재 W Beat Handshake와 동시에 발생할 수 있다.
            // 이 경우 WVALID을 0으로 내리지 않고 다음 Beat로 교체한다.
            //---------------------------------------------------
            if (w_load_req) begin

                // Payload Byte를 실제 AXI Byte Lane에 배치
                wdata <= load_aligned_wdata;

                // 해당 Byte Lane만 Write 허용
                wstrb <= load_wstrb;

                // Descriptor 기준 마지막 Beat
                wlast <= load_last_beat;

                // Handshake 시 실제 소비할 Payload Byte 수
                w_valid_bytes <= load_valid_bytes;

                wvalid <= 1'b1;
            end

            //---------------------------------------------------
            // 현재 Beat는 Handshake됐지만 다음 Beat가 아직 준비되지 않음
            //---------------------------------------------------
            else if (w_hs) begin
                wvalid        <= 1'b0;
                wlast         <= 1'b0;
                wstrb         <= {(DATA_WIDTH / 8) {1'b0}};
                w_valid_bytes <= {LEN_WIDTH{1'b0}};
            end
        end
    end

endmodule
