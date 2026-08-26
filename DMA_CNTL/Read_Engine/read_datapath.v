`timescale 1ns / 1ps

module read_datapath #(
    parameter ADDR_WIDTH   = 15,
    parameter DATA_WIDTH   = 32,
    parameter LEN_WIDTH    = 32,
    parameter BURST_WIDTH  = 8,
    // ROM 0x0000~0x3FFF / RAM 0x4000~0x7FFF
    // -> each region is 16KB, decoded by cur_addr[REGION_LSB].
    parameter REGION_LSB   = 14,

    // Burst size cap = 16B = 4 beats, matching a MicroBlaze cache line
    // (C_DCACHE_LINE_LEN = 4 words). Keeping the DMA burst equal to the
    // line size means a later cache-fill path needs no burst re-sizing.
    
    parameter MAX_BURST_BYTES = 16
)(
    input                            clk,
    input                            rst_n,

    // Control Unit -> Datapath
    input                            en,          // state == S_DATA
    input                            init,        // IDLE -> DATA transition = 1 pulse
    input                            abort,       // Force Shutdown

    // Register Map
    input      [ADDR_WIDTH-1:0]      src_addr,
    input      [LEN_WIDTH-1:0]       length,      // total transfer byte
    input      [BURST_WIDTH+1:0]     burst_cfg,   // AXI ARLEN Maximum value (in Register Setting value) [7:0] & burst type [9:8]

    // Datapath -> Control Unit
    output                           r_hs,
    output                           xfer_done,   // transfer done or abort assertion
    output     [ADDR_WIDTH-1:0]      err_addr,
    output reg                       err_valid,   // RRESP error latched
    output reg [LEN_WIDTH-1:0]       err_offset,  // byte offset where the first error occurred
    output reg [BURST_WIDTH-1:0]     err_cnt,     // number of failing beats (saturating)
    output reg                       cfg_err,     // illegal / unserviceable configuration

    // FIFO
    output reg                       fifo_wr_en,
    output reg [DATA_WIDTH-1:0]      fifo_wr_data,
    output reg [(DATA_WIDTH/8)-1:0]  fifo_wr_strb, // byte valid mask for Write Engine WSTRB
    input                            fifo_full,

    // AXI4 AR / R channel
    output reg [3:0]                 arid,
    output reg [ADDR_WIDTH-1:0]      araddr,
    output reg [BURST_WIDTH-1:0]     arlen,
    output     [2:0]                 arsize,
    output     [1:0]                 arburst,
    output reg                       arvalid,
    input                            arready,

    input      [DATA_WIDTH-1:0]      rdata,
    input                            rvalid,
    input                            rlast,
    input      [3:0]                 rid,
    input      [1:0]                 rresp,       
    output                           rready
);

    localparam [0:0] MASTER_ID      = 1'b1;                     // ID[3] : DMA = 1
    localparam BYTES_PER_BEAT       = DATA_WIDTH/8;             // 1 beat (byte)    // ex) 32-bit/8 = 4-byte
    localparam ADDR_LSB             = $clog2(BYTES_PER_BEAT);   // byte offset      // 2
    localparam PAGE_BYTES           = 4096;                     // AXI4-Full Maximum burst size = 4KB boundary
    localparam PAGE_LSB             = $clog2(PAGE_BYTES);       // 12
    localparam REGION_BYTES         = (1 << REGION_LSB);        // 16KB per slave region
    localparam MAX_BURST_BEATS      = MAX_BURST_BYTES / BYTES_PER_BEAT;  // 16/4 = 4 beats
    localparam [2:0] ARSIZE_VAL     = ADDR_LSB;

    assign arsize = ARSIZE_VAL;
    /*
        3'b000 Beat size = 1-Byte
        3'b001 Beat size = 2-Byte
        3'b010 Beat size = 4-Byte
        Bytes per Beat = 2^ARSIZE
    */

    wire [1:0]            burst_type_cfg;
    wire [BURST_WIDTH-1:0] burst_len_cfg;

    assign burst_type_cfg = burst_cfg[BURST_WIDTH+1:BURST_WIDTH];
    assign burst_len_cfg  = burst_cfg[BURST_WIDTH-1:0];
    assign arburst        = burst_type_cfg;
    /*
        burst_cfg[9:8] = 00 -> ARBURST = FIXED
        burst_cfg[9:8] = 01 -> ARBURST = INCR
        burst_cfg[9:8] = 10 -> ARBURST = WRAP
    */

    integer i;


    // Alignment pre-computation
    //   src_addr = 0x0002, length = 10  ->  head_off = 2, span = 12, beats = 3
    //
    //     0x0000   0x0001 | 0x0002   0x0003 | 0x0004 ~ 0x000B
    //    [ drop ] [ drop ]|[ keep ] [ keep ]|[      keep      ]
    //    |<----------- beat 0 ----------->|

    wire [ADDR_LSB-1:0]    head_off_c;      // leading bytes discarded (unaligned src_addr)
    wire [ADDR_WIDTH-1:0]  src_algn_c;      // driven address
    wire [LEN_WIDTH-1:0]   span_byte_c;     // total bytes actually fetched = head padding + payload
    wire [LEN_WIDTH-1:0]   total_beats_c;   // ceil(span_byte / BYTES_PER_BEAT)

    assign head_off_c    = src_addr[ADDR_LSB-1:0];      // byte
    assign src_algn_c    = {src_addr[ADDR_WIDTH-1:ADDR_LSB], {ADDR_LSB{1'b0}}};
    assign span_byte_c   = length + {{(LEN_WIDTH-ADDR_LSB){1'b0}}, head_off_c};
    assign total_beats_c = (length == {LEN_WIDTH{1'b0}}) ? {LEN_WIDTH{1'b0}}
                         : ((span_byte_c + (BYTES_PER_BEAT-1)) >> ADDR_LSB);

    // Latched at init so a register-map write mid-transfer cannot corrupt
    // the in-flight burst planning.
    reg [ADDR_LSB-1:0]   head_off_q;    
    reg [LEN_WIDTH-1:0]  span_byte_q;   
    reg [LEN_WIDTH-1:0]  total_beats_q; 

    wire [ADDR_LSB-1:0]  tail_off;
    assign tail_off = span_byte_q[ADDR_LSB-1:0];  // 0 => last beat is fully valid


    // Transfer State
    reg [ADDR_WIDTH-1:0] cur_addr;        // Next AR address (always beat-aligned)
    reg [LEN_WIDTH-1:0]  req_beat_cnt;    // requested beats
    reg [LEN_WIDTH-1:0]  rcv_beat_cnt;    // received beats -> Padding control purpose
    reg [LEN_WIDTH-1:0]  total_byte_cnt;  // bytes actually committed to the destination

    reg [1:0]            num_busy;
    reg [ADDR_WIDTH-1:0] slot_addr [0:1];

    wire next_num;
    assign next_num = num_busy[0] ? 1'b1 : 1'b0;

    wire ar_hs;
    assign ar_hs = arvalid && arready;    // AR channel handshake

    assign r_hs  = rvalid && rready;      // R channel handshake (raw, for Control Unit)
    assign rready = !fifo_full;

    // Only beats that actually belong to this master may retire a
    // slot, decrement outstanding_cnt or be pushed into the FIFO.
    wire r_mine, r_beat, r_burst_end;
    assign r_mine      = (rid[3] == MASTER_ID); // RID[3] == 1, DMA response
    assign r_beat      = r_hs && r_mine;        // Valid Data Beat 
    assign r_burst_end = r_beat && rlast;       // -> Beat's Burst Last

    wire [1:0] slave_sel;
    assign slave_sel = cur_addr[ADDR_WIDTH-1] ? 2'b01 : 2'b00;   // ROM:00 / RAM:01

    wire req_pending;
    assign req_pending = (req_beat_cnt < total_beats_q);          // Remain request

    reg [1:0] outstanding_cnt;
    reg [1:0] active_slave;

    // Values actually placed on ARID/ARADDR, held until handshake.
    reg       ar_num_q;
    reg [1:0] ar_slave_q;

    always @(posedge clk) begin
        if (!rst_n) begin
            outstanding_cnt <= 2'd0;
        end
        else if (init) begin
            outstanding_cnt <= 2'd0;
        end
        else begin
            case ({ar_hs, r_burst_end})
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
            active_slave <= ar_slave_q;   // use the slave actually issued
                                          // 0 -> 1 transition (New => outstanding == 0) => slave lock
        end
    end

    // NUM 0 1 assertion
    // set with ar_num_q (issued ID), clear with the qualified rlast.
    always @(posedge clk) begin
        if (!rst_n) begin
            num_busy <= 2'b00;
        end else if (init) begin
            num_busy <= 2'b00;
        end else begin
            if (ar_hs)       num_busy[ar_num_q] <= 1'b1;
            if (r_burst_end) num_busy[rid[0]]   <= 1'b0;
        end
    end

    reg abort_lat;
    always @(posedge clk) begin
        if (!rst_n) begin
            abort_lat <= 1'b0;
        end else if (init) begin
            abort_lat <= 1'b0;          // abort state reset
        end else if (abort) begin
            abort_lat <= 1'b1;
        end
    end

    wire same_slave_ok, outstanding_ok;
    assign same_slave_ok = (outstanding_cnt == 2'd0) || (slave_sel == active_slave);
    assign outstanding_ok = (outstanding_cnt < 2'd2);

    //----------------------------------------------------------
    // (1) desired_beats      : burst_cfg, capped at MAX_BURST_BEATS
    // (2) beats_to_boundary  : cur_addr ~ 4KB boundary addr
    //      -> Split burst before crossing 4KB boundary
    // (3) beats_to_region    : cur_addr ~ ROM/RAM region boundary
    //      -> A burst must never span two slaves. The interconnect decodes
    //         ARADDR once and routes the whole burst to that slave.
    // (4) remain_beats       : total_beats - req_beat_cnt
    //      -> last burst length < beat => burst length automatic setting
    //      -> Select the shortest valid burst length
    //----------------------------------------------------------
    wire [LEN_WIDTH-1:0] remain_beats;
    wire [LEN_WIDTH-1:0] bytes_to_boundary;
    wire [LEN_WIDTH-1:0] beats_to_boundary;
    wire [LEN_WIDTH-1:0] bytes_to_region;
    wire [LEN_WIDTH-1:0] beats_to_region;
    wire [LEN_WIDTH-1:0] desired_raw;
    wire [LEN_WIDTH-1:0] desired_beats;
    wire [LEN_WIDTH-1:0] limit_beats;

    wire [LEN_WIDTH-1:0] safe_beats_incr;
    wire [LEN_WIDTH-1:0] safe_beats_fixed;
    wire [LEN_WIDTH-1:0] safe_beats_wrap;
    wire [LEN_WIDTH-1:0] safe_beats;      // Checking Burst Size
    wire [LEN_WIDTH-1:0] safe_bytes;      // Calculate Next Burst address
    wire [BURST_WIDTH-1:0] safe_arlen;

    // calculate remain
    assign remain_beats      = total_beats_q - req_beat_cnt;

    assign bytes_to_boundary = PAGE_BYTES -
                               {{(LEN_WIDTH-PAGE_LSB){1'b0}}, cur_addr[PAGE_LSB-1:0]};
                               // == bytes_to_boundary = 4096 - (cur_addr % 4096)
    assign beats_to_boundary = bytes_to_boundary >> ADDR_LSB;   // transition bytes to beats

    assign bytes_to_region   = REGION_BYTES -
                               {{(LEN_WIDTH-REGION_LSB){1'b0}}, cur_addr[REGION_LSB-1:0]};
                               // == bytes_to_region = 16384 - (cur_addr % 16384)
    assign beats_to_region   = bytes_to_region >> ADDR_LSB;

    assign desired_raw       = {{(LEN_WIDTH-BURST_WIDTH){1'b0}}, burst_len_cfg} + 1'b1; // Beats = ARLEN + 1
    /*
        ARLEN = 0 , Beat = 1
        ARLEN = 1 , Beat = 2
        ARLEN = 3 , Beat = 4
    */

    // Hardware cap : a register-map value larger than MAX_BURST_BEATS is
    // clamped here, so no configuration can exceed one cache line per burst.
    assign desired_beats     = (desired_raw > MAX_BURST_BEATS) ? MAX_BURST_BEATS
                                                               : desired_raw;

    // min(4KB boundary, slave region boundary)
    assign limit_beats = (beats_to_boundary < beats_to_region) ? beats_to_boundary
                                                              : beats_to_region;

    // ---------------------------------------------------------
    // FIXED Burst
    // AXI4 FIXED burst supports up to 16 beats.
    // Address does not advance, so no boundary check is required.
    // ---------------------------------------------------------
    assign safe_beats_fixed =
        (remain_beats < desired_beats) ?
            ((remain_beats  < 16) ? remain_beats  : 16) :
            ((desired_beats < 16) ? desired_beats : 16);   // FIXED: min(Max Beats, Remaining Beats, 16)

    // ---------------------------------------------------------
    // INCR Burst
    // Must not cross a 4KB boundary, and must not cross the ROM/RAM region boundary.                                
    // INCR: min(Max Beats, Beats to 4KB Boundary,
    //           Beats to Region Boundary, Remaining Beats)
    // ---------------------------------------------------------
    assign safe_beats_incr =
        (desired_beats < limit_beats) ?
            ((desired_beats < remain_beats) ? desired_beats : remain_beats) :
            ((limit_beats   < remain_beats) ? limit_beats   : remain_beats);

    // ---------------------------------------------------------
    // WRAP Burst
    // AXI4 WRAP supports only 2 / 4 / 8 / 16 beats.
    // Choose the largest legal size that does not exceed
    // desired_beats and remain_beats.
    // With MAX_BURST_BEATS = 4 only the 4 / 2 arms are reachable; the
    // 16 / 8 arms stay for when the cap is raised.
    // NOTE: a remainder of 1 beat has no legal WRAP length, so
    //       safe_beats_wrap becomes 0 and cfg_err is raised.    
    // ---------------------------------------------------------
    assign safe_beats_wrap =
        ((desired_beats >= 16) && (remain_beats >= 16)) ? 16 :
        ((desired_beats >=  8) && (remain_beats >=  8)) ?  8 :
        ((desired_beats >=  4) && (remain_beats >=  4)) ?  4 :
        ((desired_beats >=  2) && (remain_beats >=  2)) ?  2 :
                                                           0;
    // Select Burst calculation
    assign safe_beats =
        (burst_type_cfg == 2'b00) ? safe_beats_fixed :
        (burst_type_cfg == 2'b01) ? safe_beats_incr  :
        (burst_type_cfg == 2'b10) ? safe_beats_wrap  :
                                    0;

    // ======================= Using AXI Operation =========================
    // Convert the selected Burst Beat count to Byte count
    assign safe_bytes = safe_beats << ADDR_LSB;
    // Convert the selected Burst Beat count to AXI ARLEN (ARLEN = Beats - 1)
    // Guarded by ar_can_issue: safe_beats == 0 would underflow to 8'hFF.
    assign safe_arlen = safe_beats[BURST_WIDTH-1:0] - 1'b1;
    // ======================================================================

    // new AR possible condition check
    // -- Guard --
    //  !init          : init cycle = Before cur_addr / req_beat_cnt update
    //  safe_beats!=0  : prevents the 8'hFF underflow / zero-progress lock
    //  !cfg_err       : an unserviceable configuration must not keep issuing
    //  NOTE: err_valid is deliberately NOT in this list. An RRESP error voids
    //        only the failing burst; the transfer continues with the next one.
    wire ar_can_issue;
    assign ar_can_issue = en && !init && !arvalid && req_pending && !abort_lat
                          && outstanding_ok && same_slave_ok
                          && (safe_beats != {LEN_WIDTH{1'b0}})
                          && !cfg_err;

    //======================================================================
    //   Configuration / progress error
    //   req_pending is asserted but no legal burst length can be formed.
    //   Typical causes: WRAP with a non power-of-two remainder,
    //                   WRAP with an unaligned start address (illegal in AXI),
    //                   an unsupported ARBURST encoding (2'b11).
    //
    //   cfg_err  : configuration fault    -> stops issuing new AR
    //   err_valid: transfer fault (RRESP) -> does NOT stop issuing
    //   abort_lat: software forced stop   -> stops issuing new AR
    //======================================================================
    wire wrap_illegal;  // WRAP type but, non-alignment == AXI violation
    assign wrap_illegal = (burst_type_cfg == 2'b10) && (head_off_q != {ADDR_LSB{1'b0}}); 

    wire no_progress;   // Transfer Remain & Legal Burst length don't make
    assign no_progress = en && !init && req_pending && !abort_lat
                         && (safe_beats == {LEN_WIDTH{1'b0}});

    always @(posedge clk) begin
        if (!rst_n) begin
            cfg_err <= 1'b0;
        end
        else if (init) begin
            cfg_err <= 1'b0;
        end
        else if (no_progress || wrap_illegal || (burst_type_cfg == 2'b11)) begin
            cfg_err <= 1'b1;
        end
    end

    // AR channel Done = Not exists Request Or Abort assertion
    wire ar_done; // AR request X
    assign ar_done = !req_pending || abort_lat || cfg_err;

    // Transfer Done == Burst Response Done
    assign xfer_done = !init && ar_done && (outstanding_cnt == 2'd0); // Do exists request != Transfer Done


    //======================================================================
    //   R channel : head / tail byte masking, zero padding
    //
    //   first beat : bytes below head_off_q belong to the previous aligned
    //                word and are not part of the payload
    //   last  beat : bytes at or above tail_off are past the end of LENGTH
    //                (tail_off == 0 means the last beat is fully valid)
    //
    //   Declared ahead of the datapath register block because the byte
    //   counter there consumes eff_strb.
    //======================================================================
    wire first_beat, last_beat;
    assign first_beat = (rcv_beat_cnt == {LEN_WIDTH{1'b0}});
    assign last_beat  = (rcv_beat_cnt == (total_beats_q - 1'b1));

    reg [BYTES_PER_BEAT-1:0] beat_strb_c; // Valid Byte check (alignment only)

    always @(*) begin
        for (i = 0; i < BYTES_PER_BEAT; i = i + 1) begin
            beat_strb_c[i] = 1'b1;
            if (first_beat && (i < head_off_q))
                beat_strb_c[i] = 1'b0;                       // head padding
            if (last_beat && (tail_off != {ADDR_LSB{1'b0}}) && (i >= tail_off))
                beat_strb_c[i] = 1'b0;                       // tail padding
        end
    end


    // Per-beat RRESP error handling
    //   Void the failing beat and the rest of its burst (strb = 0).
    //   Voided beats are still pushed so the beat count stays in step with
    //   the Write Engine; WSTRB = 0 leaves those DST bytes unmodified.
    //   Cleared at rlast -> next burst runs normally.

    wire beat_err;
    assign beat_err = r_beat && rresp[1];       // SLVERR(2'b10) / DECERR(2'b11)
    /*
        2'b00  OKAY    → rresp[1] = 0   OKAY 
        2'b01  EXOKAY  → rresp[1] = 0   OKAY = Exclusive Access Done -> Slave return
        2'b10  SLVERR  → rresp[1] = 1   Error
        2'b11  DECERR  → rresp[1] = 1   Error
    */

    reg burst_err_lat;                          // sticky until this burst ends
    always @(posedge clk) begin
        if (!rst_n)            burst_err_lat <= 1'b0;
        else if (init)         burst_err_lat <= 1'b0;
        else if (r_burst_end)  burst_err_lat <= 1'b0;   // released at rlast
        else if (beat_err)     burst_err_lat <= 1'b1;
    end

    wire beat_void;
    assign beat_void = beat_err || burst_err_lat;

    // effective strobe : alignment mask, forced to 0 on a voided beat
    wire [BYTES_PER_BEAT-1:0] eff_strb;
    assign eff_strb = beat_void ? {BYTES_PER_BEAT{1'b0}} : beat_strb_c;

    // R channel -> FIFO
    always @(*) begin
        fifo_wr_en   = r_beat;                  // every qualified beat is pushed
        fifo_wr_strb = eff_strb;
        for (i = 0; i < BYTES_PER_BEAT; i = i + 1)
            fifo_wr_data[i*8 +: 8] = eff_strb[i] ? rdata[i*8 +: 8] : 8'h00;  // masking
    end


    // Register in Datapath Update
    always @(posedge clk) begin
        if (!rst_n) begin                      
            cur_addr       <= {ADDR_WIDTH{1'b0}};
            req_beat_cnt   <= {LEN_WIDTH{1'b0}};
            rcv_beat_cnt   <= {LEN_WIDTH{1'b0}};
            total_byte_cnt <= {LEN_WIDTH{1'b0}};
            head_off_q     <= {ADDR_LSB{1'b0}};
            span_byte_q    <= {LEN_WIDTH{1'b0}};
            total_beats_q  <= {LEN_WIDTH{1'b0}};
            slot_addr[0]   <= {ADDR_WIDTH{1'b0}};
            slot_addr[1]   <= {ADDR_WIDTH{1'b0}};
        end
        else if (init) begin                        // initialize - DMA Read Transfer Start
            cur_addr       <= src_algn_c;           // aligned base, not raw src_addr
            req_beat_cnt   <= {LEN_WIDTH{1'b0}};
            rcv_beat_cnt   <= {LEN_WIDTH{1'b0}};
            total_byte_cnt <= {LEN_WIDTH{1'b0}};
            head_off_q     <= head_off_c;
            span_byte_q    <= span_byte_c;
            total_beats_q  <= total_beats_c;
            slot_addr[0]   <= {ADDR_WIDTH{1'b0}};
            slot_addr[1]   <= {ADDR_WIDTH{1'b0}};
        end
        else begin
            if (r_beat) begin                       // qualified R beat
                rcv_beat_cnt   <= rcv_beat_cnt + 1'b1;
                // eff_strb, not beat_strb_c : a voided beat contributes 0, so
                // total_byte_cnt reports the bytes actually written to DST
                total_byte_cnt <= total_byte_cnt + count_ones(eff_strb);
            end

            if (ar_hs) begin                        // AR channel handshake
                case (burst_type_cfg)
                    2'b00: begin                    // FIXED
                        cur_addr <= cur_addr;       // address is intentionally held
                                                    // (peripheral FIFO style access)
                    end
                    2'b01: begin                    // INCR
                        cur_addr <= cur_addr + safe_bytes[ADDR_WIDTH-1:0];
                    end
                    2'b10: begin                    // WRAP
                        // WRAP address calculation is still to be implemented.
                        // Until then WRAP is only legal for a single burst that
                        // covers the whole transfer; wrap_illegal / no_progress
                        // catch the unsupported cases.
                    end
                    default: begin
                        cur_addr <= cur_addr;
                    end
                endcase
                req_beat_cnt          <= req_beat_cnt + safe_beats;   // beats, not bytes
                slot_addr[ar_num_q]   <= araddr;                      // issued slot + issued address
            end
        end
    end

    //======================================================================
    // AR channel
    //======================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            arvalid    <= 1'b0;
            araddr     <= {ADDR_WIDTH{1'b0}};
            arlen      <= {BURST_WIDTH{1'b0}};
            arid       <= 4'd0;
            ar_num_q   <= 1'b0;
            ar_slave_q <= 2'd0;
        end
        else if (ar_hs) begin
            arvalid <= 1'b0;                        // After handshake, ARVALID signal sets LOW
                                                    // ARVALID is never withdrawn before ARREADY (AXI rule),
                                                    // so an abort only blocks *new* requests.
        end
        else if (ar_can_issue) begin
            arvalid    <= 1'b1;
            araddr     <= cur_addr;                 // already beat-aligned 
            arlen      <= safe_arlen;
            arid       <= {MASTER_ID, slave_sel, next_num}; // Generate ID for Arbiter table
            ar_num_q   <= next_num;                 // freeze what was actually issued
            ar_slave_q <= slave_sel;                
        end
    end


    // RRESP capture
    //   First failure latches the burst base address and err_offset
    //   (bytes copied so far = software resume point).
    //   err_cnt counts every failing beat, saturating.

    reg [ADDR_WIDTH-1:0] err_addr_q;
    assign err_addr = err_addr_q;

    always @(posedge clk) begin
        if (!rst_n) begin
            err_valid  <= 1'b0;
            err_addr_q <= {ADDR_WIDTH{1'b0}};
            err_offset <= {LEN_WIDTH{1'b0}};
            err_cnt    <= {BURST_WIDTH{1'b0}};
        end
        else if (init) begin
            err_valid  <= 1'b0;
            err_addr_q <= {ADDR_WIDTH{1'b0}};
            err_offset <= {LEN_WIDTH{1'b0}};
            err_cnt    <= {BURST_WIDTH{1'b0}};
        end
        else begin
            if (beat_err && !err_valid) begin              // first failure only
                err_valid  <= 1'b1;
                err_addr_q <= slot_addr[rid[0]];
                err_offset <= total_byte_cnt;
            end
            if (beat_err && (err_cnt != {BURST_WIDTH{1'b1}}))
                err_cnt <= err_cnt + 1'b1;                 // saturating
        end
    end

    // valid byte counter helper
    function [ADDR_LSB:0] count_ones;
        input [BYTES_PER_BEAT-1:0] s;
        integer k;
        begin
            count_ones = 0;
            for (k = 0; k < BYTES_PER_BEAT; k = k + 1)
                count_ones = count_ones + s[k];
        end
    endfunction

endmodule
