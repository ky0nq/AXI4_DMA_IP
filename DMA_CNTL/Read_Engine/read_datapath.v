`timescale 1ns / 1ps

module read_datapath #(
    parameter ADDR_WIDTH  = 15,
    parameter DATA_WIDTH  = 32,
    parameter LEN_WIDTH   = 32,
    parameter BURST_WIDTH = 8
)(
    input                           clk,
    input                           rst_n,

    // Control Unit -> Datapath
    input                           en,             // state == S_DATA
    input                           init,           // IDLE -> DATA transition = 1 pulse
    input                           abort,          // Force Shutdown

    // Register Map
    input       [ADDR_WIDTH-1:0]    src_addr,
    input       [LEN_WIDTH-1:0]     length,         // total transfer burst byte
    input       [BURST_WIDTH+1:0] burst_cfg,        // AXI ARLEN Maximum value (in Register Setting value) [7:0] & burst type [9:8]

    // Datapath -> Control Unit
    output                          r_hs,
    output                          xfer_done,      // transfer done or abort assertion
    output      [ADDR_WIDTH-1:0]    err_addr,

    // FIFO
    output reg                      fifo_wr_en,
    output reg  [DATA_WIDTH-1:0]    fifo_wr_data,
    input                           fifo_full,

    // AXI4 AR / R channel
    output reg  [3:0]               arid,
    output reg  [ADDR_WIDTH-1:0]    araddr,
    output reg  [BURST_WIDTH-1:0]   arlen,
    output      [2:0]               arsize,
    output      [1:0]               arburst,
    output reg                      arvalid,
    input                           arready,
    input       [DATA_WIDTH-1:0]    rdata,
    input                           rvalid,
    input                           rlast,
    input       [3:0]               rid,
    output                          rready
);

    localparam MASTER_ID      = 1'b1;                       // ID[3] : DMA = 1
    localparam BYTES_PER_BEAT = DATA_WIDTH/8;               // 1 beat (byte)  // ex) 32-byte/8 = 4-byte
    localparam ADDR_LSB       = $clog2(BYTES_PER_BEAT);     // byte offset    // 2
    localparam PAGE_BYTES     = 4096;                       // AXI4-Full Maximum burst size = 4KB boundary
    localparam PAGE_LSB       = $clog2(PAGE_BYTES);         // 12

    localparam [2:0] ARSIZE_VAL = ADDR_LSB;                 

    assign arsize  = ARSIZE_VAL;           
    /* 
    3'b000 Beat size = 1-Byte
    3'b001 Beat size = 2-Byte
    3'b010 Beat size = 4-Byte

    Bytes per Beat = 2^ARSIZE
    */
    wire [1:0]             burst_type_cfg;
    wire [BURST_WIDTH-1:0] burst_len_cfg;

    assign burst_type_cfg = burst_cfg[BURST_WIDTH+1:BURST_WIDTH];
    assign burst_len_cfg  = burst_cfg[BURST_WIDTH-1:0];

    assign arburst = burst_type_cfg;

    /*
    burst_cfg[9:8] = 00 → ARBURST = FIXED
    burst_cfg[9:8] = 01 → ARBURST = INCR
    burst_cfg[9:8] = 10 → ARBURST = WRAP
    */

    reg  [ADDR_WIDTH-1:0] cur_addr;         // Next AR address
    reg  [LEN_WIDTH-1:0]  req_byte_cnt;     // request byte
    reg  [LEN_WIDTH-1:0]  total_byte_cnt;   // total received byte

    reg  [1:0]            num_busy;
    reg  [ADDR_WIDTH-1:0] slot_addr [0:1];
    wire                  next_num;
    assign next_num = num_busy[0] ? 1'b1 : 1'b0;

    assign err_addr = slot_addr[rid[0]];

    wire ar_hs;
    assign ar_hs  = arvalid && arready;     // AR channel handshake
    assign r_hs   = rvalid  && rready;      // R channel handshake
    assign rready = !fifo_full;

    wire [1:0] slave_sel;
    assign slave_sel = cur_addr[ADDR_WIDTH-1] ? 2'b01 : 2'b00;  // ROM:00 / RAM:01

    wire req_pending;
    assign req_pending = (req_byte_cnt < length);   // Remain request

    reg [1:0] outstanding_cnt;
    reg [1:0] active_slave;

    always @(posedge clk) begin
        if (!rst_n) begin
            outstanding_cnt <= 2'd0;
        end
        else if (init) begin
            outstanding_cnt <= 2'd0;
        end
        else begin
            case ({ar_hs, (r_hs && rlast)})
                2'b10:   outstanding_cnt <= outstanding_cnt + 2'd1; // AR handshake => outstanding ++
                2'b01:   outstanding_cnt <= outstanding_cnt - 2'd1; // burst complete = R handshake + LAST signal => outstanding --
                default: outstanding_cnt <= outstanding_cnt;        // 00 = X , 11 = ++ & --
            endcase
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            active_slave <= 2'd0;
        end
        else if (init) begin
            active_slave <= 2'd0;
        end
        else if (ar_hs && (outstanding_cnt == 2'd0)) begin
            active_slave <= slave_sel;      // 0 -> 1 transition (New => outstanding == 0) => slave lock
        end
    end

    // NUM 0 1 assertion 
    always @(posedge clk) begin
        if (!rst_n) begin
            num_busy <= 2'b00;
        end else if (init) begin
            num_busy <= 2'b00;
        end else begin
            if (ar_hs)         num_busy[next_num] <= 1'b1;
            if (r_hs && rlast) num_busy[rid[0]]   <= 1'b0;
        end
    end

    reg abort_lat;
    always @(posedge clk) begin
        if (!rst_n) begin
            abort_lat <= 1'b0;
        end else if (init) begin
            abort_lat <= 1'b0;              // abort state reset
        end else if (abort) begin
            abort_lat <= 1'b1;
        end
    end

    wire same_slave_ok, outstanding_ok;
    assign same_slave_ok  = (outstanding_cnt == 2'd0) || (slave_sel == active_slave);
    assign outstanding_ok = (outstanding_cnt < 2'd2);

    // new AR possible condition check
    // -- Guard --
    // !init : init cycle = Before cur_addr/req_byte_cnt update
    wire ar_can_issue;
    assign ar_can_issue = en && !init && !arvalid && req_pending && !abort_lat && outstanding_ok && same_slave_ok;

    // AR channel Done = Not exists Request Or Abort assertion
    wire ar_done;   // AR request X
    assign ar_done = !req_pending || abort_lat; 

    // Transfer Done == Burst Response Done
    assign xfer_done = !init && ar_done && (outstanding_cnt == 2'd0);   // Do exists request != Transfer Done


    //-----------------------------------------------------------
    //  (1) desired_beats      : burst_cfg
    //  (2) beats_to_boundary  : cur_addr ~ 4KB boundary addr
    //                           -> Split burst before crossing 4KB boundary
    //                           -> Select the shortest valid burst length
    //  (3) remain_beats       : length - req_byte_cnt = #remained beat
    //                           -> last burst length < beat => burst length automatic setting (shortest)
    //-----------------------------------------------------------
    wire [LEN_WIDTH-1:0]   remain_bytes;
    wire [LEN_WIDTH-1:0]   remain_beats;
    wire [LEN_WIDTH-1:0]   bytes_to_boundary;
    wire [LEN_WIDTH-1:0]   beats_to_boundary;
    wire [LEN_WIDTH-1:0]   desired_beats;

    wire [LEN_WIDTH-1:0]   safe_beats_incr;
    wire [LEN_WIDTH-1:0]   safe_beats_fixed;
    wire [LEN_WIDTH-1:0]   safe_beats_wrap;
    wire [LEN_WIDTH-1:0]   safe_beats;        // Checking Burst Size  

    wire [LEN_WIDTH-1:0]   safe_bytes;        // Calculate Next Busrt address 
    wire [BURST_WIDTH-1:0] safe_arlen;

    // calculate remain
    assign remain_bytes      = length - req_byte_cnt;          
    assign remain_beats      = remain_bytes >> ADDR_LSB;    // Sending Beats per 4-Byte (1 Beat = 4-Byte)

    assign bytes_to_boundary = PAGE_BYTES -
                               {{(LEN_WIDTH-PAGE_LSB){1'b0}}, cur_addr[PAGE_LSB-1:0]}; // == bytes_to_boundary = 4096 - (cur_addr % 4096)
    assign beats_to_boundary = bytes_to_boundary >> ADDR_LSB;                          // transition bytes to beats

    assign desired_beats     = {{(LEN_WIDTH-BURST_WIDTH){1'b0}}, burst_len_cfg} + 1'b1;     // Beats = ARLEN + 1

    /* 
    ARLEN = 0 , Beat = 1
    ARLEN = 1 , Beat = 2
    ARLEN = 3 , Beat = 4
    */

    // ---------------------------------------------------------
    // FIXED Burst
    // AXI4 FIXED burst supports up to 16 beats.
    // ---------------------------------------------------------
    assign safe_beats_fixed =
        (remain_beats < desired_beats) ?
            ((remain_beats < 16) ? remain_beats : 16) :
            ((desired_beats < 16) ? desired_beats : 16); // FIXED: min(Max Beats, Remaining Beats, 16)


    // ---------------------------------------------------------
    // INCR Burst
    // Must not cross 4KB boundary.
    // ---------------------------------------------------------
    assign safe_beats_incr =
        (desired_beats < beats_to_boundary) ?
            ((desired_beats < remain_beats) ?
                desired_beats : remain_beats) :
            ((beats_to_boundary < remain_beats) ?
                beats_to_boundary : remain_beats); // INCR: min(Max Beats, Beats to 4KB Boundary, Remaining Beats)


    // ---------------------------------------------------------
    // WRAP Burst
    // AXI4 WRAP supports only 2 / 4 / 8 / 16 beats.
    // Choose the largest legal size that does not exceed
    // desired_beats and remain_beats.
    // ---------------------------------------------------------
    assign safe_beats_wrap =
        ((desired_beats >= 16) && (remain_beats >= 16)) ? 16 :
        ((desired_beats >=  8) && (remain_beats >=  8)) ?  8 :
        ((desired_beats >=  4) && (remain_beats >=  4)) ?  4 :
        ((desired_beats >=  2) && (remain_beats >=  2)) ?  2 :
                                                        0;


    // ---------------------------------------------------------
    // Select Burst calculation
    // ---------------------------------------------------------
    assign safe_beats =
        (burst_type_cfg == 2'b00) ? safe_beats_fixed :
        (burst_type_cfg == 2'b01) ? safe_beats_incr  :
        (burst_type_cfg == 2'b10) ? safe_beats_wrap  :
                                    0;

    // ======================= Using AXI Operation =========================
    // Convert the selected Burst Beat count to Byte count
    assign safe_bytes = safe_beats << ADDR_LSB;
    // Convert the selected Burst Beat count to AXI ARLEN (ARLEN = Beats - 1)
    assign safe_arlen = safe_beats - 1'b1;
    // ======================================================================

    // Register in Datapath Update
    always @(posedge clk) begin
        if (!rst_n) begin       // reset
            cur_addr       <= {ADDR_WIDTH{1'b0}};
            req_byte_cnt   <= {LEN_WIDTH{1'b0}};
            total_byte_cnt <= {LEN_WIDTH{1'b0}};
            slot_addr[0]   <= {ADDR_WIDTH{1'b0}};
            slot_addr[1]   <= {ADDR_WIDTH{1'b0}};
        end
        else if (init) begin    // initialize - DMA Read Transfer Start 
            cur_addr       <= src_addr;
            req_byte_cnt   <= {LEN_WIDTH{1'b0}};
            total_byte_cnt <= {LEN_WIDTH{1'b0}};
            slot_addr[0]   <= {ADDR_WIDTH{1'b0}};
            slot_addr[1]   <= {ADDR_WIDTH{1'b0}};
        end
        else begin
            if (r_hs)           // R channel handshake
                total_byte_cnt <= total_byte_cnt + BYTES_PER_BEAT;
            if (ar_hs) begin    // AR channel handshake
                case (burst_type_cfg)
                    2'b00: begin // FIXED
                        cur_addr <= cur_addr;
                    end
                    2'b01: begin // INCR
                        cur_addr <= cur_addr + safe_bytes[ADDR_WIDTH-1:0];
                    end
                    2'b10: begin // WRAP
                        // WRAP address calculation 별도 구현 필요
                    end
                    default: begin
                        cur_addr <= cur_addr;
                    end
                endcase

                req_byte_cnt <= req_byte_cnt + safe_bytes;
                slot_addr[next_num] <= cur_addr;
            end
        end
    end

    // AR channel
    always @(posedge clk) begin
        if (!rst_n) begin
            arvalid <= 1'b0;
            araddr  <= {ADDR_WIDTH{1'b0}};
            arlen   <= {BURST_WIDTH{1'b0}};
            arid    <= 4'd0;
        end
        else if (ar_hs) begin
            arvalid <= 1'b0;        // After handshake, ARVALID signal sets LOW
        end
        else if (ar_can_issue) begin
            arvalid <= 1'b1;
            araddr  <= cur_addr;
            arlen   <= safe_arlen;
            arid    <= {MASTER_ID, slave_sel, next_num}; // Generate ID for Arbiter table
        end
    end

    // R channel
    always @(*) begin
        fifo_wr_en   = r_hs;
        fifo_wr_data = rdata;  // FIFO write == Read Operation
    end

endmodule
