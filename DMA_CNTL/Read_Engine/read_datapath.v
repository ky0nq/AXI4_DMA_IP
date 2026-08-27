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
    output     [ADDR_WIDTH-1:0]      err_addr,    // address of the beat that failed
    output reg                       err_valid,   // RRESP error latched
    output reg                       cfg_err,     // illegal / unserviceable configuration

    // FIFO : 32-bit, data only.
    //   No strobe and no error flag are forwarded. SRC and LENGTH are forced
    //   4-byte aligned, so every beat carries 4 valid bytes and the Write
    //   Engine drives WSTRB = 4'b1111 unconditionally.
    output reg                       fifo_wr_en,
    output reg [DATA_WIDTH-1:0]      fifo_wr_data,
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

    //======================================================================
    //   Alignment policy : SRC_ADDR and LENGTH must be 4-byte aligned
    //
    //   Supporting an unaligned start would need head / tail byte masking,
    //   and an unaligned SRC/DST pair would additionally need a byte barrel
    //   shifter to merge adjacent beats. Both are out of scope, so the
    //   condition is checked here and reported as cfg_err instead.
    //   The register map should reject START on the same condition; this
    //   check is the hardware backstop.
    //======================================================================
    wire align_err_c;
    assign align_err_c = (src_addr[ADDR_LSB-1:0] != {ADDR_LSB{1'b0}}) ||
                         (length[ADDR_LSB-1:0]   != {ADDR_LSB{1'b0}});

    wire [LEN_WIDTH-1:0] total_beats_c;
    assign total_beats_c = length >> ADDR_LSB;   // exact : LENGTH is a multiple of 4

    reg                 align_err_q;
    reg [LEN_WIDTH-1:0] total_beats_q;           // latched at init


    // Transfer State
    reg [ADDR_WIDTH-1:0] cur_addr;        // Next AR address (always beat-aligned)
    reg [LEN_WIDTH-1:0]  req_beat_cnt;    // requested beats

    reg [1:0]             num_busy;
    reg [ADDR_WIDTH-1:0]  slot_addr [0:1];  // base address of the burst in each slot
    reg [BURST_WIDTH-1:0] slot_beat [0:1];  // beats already received in each slot

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
        ((desired_beats >=  2) && (remain_beats >=  2)) ?  2 : 0;
    // Select Burst calculation
    assign safe_beats =
        (burst_type_cfg == 2'b00) ? safe_beats_fixed :
        (burst_type_cfg == 2'b01) ? safe_beats_incr  :
        (burst_type_cfg == 2'b10) ? safe_beats_wrap  : 0;

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
    //  NOTE: err_valid is deliberately NOT in this list. An RRESP error is
    //        reported to software only; the transfer keeps running.
    wire ar_can_issue;
    assign ar_can_issue = en && !init && !arvalid && req_pending && !abort_lat
                          && outstanding_ok && same_slave_ok
                          && (safe_beats != {LEN_WIDTH{1'b0}})
                          && !cfg_err;

    //======================================================================
    //   Configuration / progress error
    //   Causes : SRC_ADDR or LENGTH not 4-byte aligned,
    //            no legal burst length can be formed (e.g. WRAP remainder),
    //            an unsupported ARBURST encoding (2'b11).
    //
    //   cfg_err  : configuration fault    -> stops issuing new AR
    //   err_valid: transfer fault (RRESP) -> does NOT stop issuing
    //   abort_lat: software forced stop   -> stops issuing new AR
    //======================================================================
    wire no_progress;   // Transfer Remain & Legal Burst length don't make
    assign no_progress = en && !init && req_pending && !abort_lat
                         && (safe_beats == {LEN_WIDTH{1'b0}});

    always @(posedge clk) begin
        if (!rst_n) begin
            cfg_err <= 1'b0;
        end
        else if (init) begin
            cfg_err <= align_err_c;                 // checked at start
        end
        else if (no_progress || align_err_q || (burst_type_cfg == 2'b11)) begin
            cfg_err <= 1'b1;
        end
    end

    // AR channel Done = Not exists Request Or Abort assertion
    wire ar_done; // AR request X
    assign ar_done = !req_pending || abort_lat || cfg_err;

    // Transfer Done == Burst Response Done
    assign xfer_done = !init && ar_done && (outstanding_cnt == 2'd0); // Do exists request != Transfer Done

    always @(*) begin
        fifo_wr_en   = r_beat;                  // every qualified beat is pushed
        fifo_wr_data = rdata;
    end

    // Register in Datapath Update
    always @(posedge clk) begin
        if (!rst_n) begin
            cur_addr       <= {ADDR_WIDTH{1'b0}};
            req_beat_cnt   <= {LEN_WIDTH{1'b0}};
            total_beats_q  <= {LEN_WIDTH{1'b0}};
            align_err_q    <= 1'b0;
            slot_addr[0]   <= {ADDR_WIDTH{1'b0}};
            slot_addr[1]   <= {ADDR_WIDTH{1'b0}};
            slot_beat[0]   <= {BURST_WIDTH{1'b0}};
            slot_beat[1]   <= {BURST_WIDTH{1'b0}};
        end
        else if (init) begin                        // initialize - DMA Read Transfer Start
            cur_addr       <= src_addr;             // already 4-byte aligned by policy
            req_beat_cnt   <= {LEN_WIDTH{1'b0}};
            total_beats_q  <= total_beats_c;
            align_err_q    <= align_err_c;
            slot_addr[0]   <= {ADDR_WIDTH{1'b0}};
            slot_addr[1]   <= {ADDR_WIDTH{1'b0}};
            slot_beat[0]   <= {BURST_WIDTH{1'b0}};
            slot_beat[1]   <= {BURST_WIDTH{1'b0}};
        end
        else begin
            // Calculate error address 
            if (ar_hs)      slot_beat[ar_num_q] <= {BURST_WIDTH{1'b0}};
            if (r_beat)     slot_beat[rid[0]]   <= slot_beat[rid[0]] + 1'b1;

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
                        // covers the whole transfer; no_progress catches the
                        // unsupported cases.
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
            araddr     <= cur_addr;                         // already beat-aligned
            arlen      <= safe_arlen;
            arid       <= {MASTER_ID, slave_sel, next_num}; // Generate ID for Arbiter table
            ar_num_q   <= next_num;                         // freeze what was actually issued
            ar_slave_q <= slave_sel;
        end
    end


    //======================================================================
    //   RRESP capture -> software only
    //   err_addr is the address of the beat that actually failed :
    //     burst base address + beat index * BYTES_PER_BEAT
    //   A FIXED burst holds its address, so no offset is added there.
    //   Only the first failure is latched.
    //======================================================================
    wire beat_err;
    assign beat_err = r_beat && rresp[1];       // SLVERR(2'b10) / DECERR(2'b11)
    /*
        2'b00  OKAY    -> rresp[1] = 0   OKAY
        2'b01  EXOKAY  -> rresp[1] = 0   OKAY = Exclusive Access Done -> Slave return
        2'b10  SLVERR  -> rresp[1] = 1   Error
        2'b11  DECERR  -> rresp[1] = 1   Error
    */

    wire [ADDR_WIDTH-1:0] beat_addr;
    assign beat_addr = (burst_type_cfg == 2'b00)
                     ? slot_addr[rid[0]]
                     : slot_addr[rid[0]] +
                       ({{(ADDR_WIDTH-BURST_WIDTH){1'b0}}, slot_beat[rid[0]]} << ADDR_LSB);

    reg [ADDR_WIDTH-1:0] err_addr_q;
    assign err_addr = err_addr_q;

    always @(posedge clk) begin
        if (!rst_n) begin
            err_valid  <= 1'b0;
            err_addr_q <= {ADDR_WIDTH{1'b0}};
        end
        else if (init) begin
            err_valid  <= 1'b0;
            err_addr_q <= {ADDR_WIDTH{1'b0}};
        end
        else if (beat_err && !err_valid) begin        // first failure only
            err_valid  <= 1'b1;
            err_addr_q <= beat_addr;
        end
    end

endmodule
