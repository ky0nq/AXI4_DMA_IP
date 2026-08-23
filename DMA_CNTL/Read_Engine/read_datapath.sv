`timescale 1ns / 1ps

module read_datapath #(
    parameter int ADDR_WIDTH  = 15,
    parameter int DATA_WIDTH  = 32,
    parameter int LEN_WIDTH   = 32,
    parameter int BURST_WIDTH = 8
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // Control Unit -> Datapath
    input  logic                    en,             // state == S_DATA
    input  logic                    init,           // IDLE -> DATA transition = 1 pulse
    input  logic                    abort,          // Force Shutdown

    // Register Map
    input  logic [ADDR_WIDTH-1:0]   src_addr,
    input  logic [LEN_WIDTH-1:0]    length,         // total transfer burst byte 
    input  logic [BURST_WIDTH-1:0]  burst_cfg,      // AXI ARLEN Maximum value (in Register Setting value)

    // Datapath -> Control Unit
    output logic                    r_hs,
    output logic                    xfer_done,      // transfer done or abort assertion 
    output logic [ADDR_WIDTH-1:0]   err_addr,

    // FIFO 
    output logic                    fifo_wr_en,
    output logic [DATA_WIDTH-1:0]   fifo_wr_data,
    input  logic                    fifo_full,

    // AXI4 AR/R channel
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
    output logic                    rready
);

    localparam logic MASTER_ID      = 1'b1;                     // ID[3] : DMA = 1
    localparam int   BYTES_PER_BEAT = DATA_WIDTH/8;             // 1 beat (byte)
    localparam int   ADDR_LSB       = $clog2(BYTES_PER_BEAT);   // byte offset 
    localparam int   PAGE_BYTES     = 4096;                     // AXI4-Full Maximum burst size = 4KB boundary
    localparam int   PAGE_LSB       = $clog2(PAGE_BYTES);       // 12

    assign arsize  = 3'b010;
    assign arburst = 2'b01;


    logic [ADDR_WIDTH-1:0] cur_addr;        // Next AR address
    logic [LEN_WIDTH-1:0]  req_byte_cnt;    // request byte
    logic [LEN_WIDTH-1:0]  total_byte_cnt;  // total received byte

    logic [1:0] slot_busy;
    logic [ADDR_WIDTH-1:0] slot_addr [0:1];
    logic next_num;
    assign next_num = slot_busy[0] ? 1'b1 : 1'b0;

    assign err_addr = slot_addr[rid[0]];   

    logic ar_hs;
    assign ar_hs  = arvalid && arready;     // AR channel handshake
    assign r_hs   = rvalid  && rready;      // R channel handshake
    assign rready = !fifo_full;

    logic [1:0] slave_sel;
    assign slave_sel = cur_addr[14] ? 2'b01 : 2'b00;    // ROM:00 / RAM:01

    logic req_pending;  
    assign req_pending = (req_byte_cnt < length);    

    logic [1:0] outstanding_cnt;  
    logic [1:0] active_slave;     

    always_ff @(posedge clk) begin
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

    always_ff @(posedge clk) begin
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

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            slot_busy <= 2'b00;
        end else if (init) begin
            slot_busy <= 2'b00;
        end else begin
            if (ar_hs)         slot_busy[next_num] <= 1'b1;
            if (r_hs && rlast) slot_busy[rid[0]]   <= 1'b0;
        end
    end

    logic abort_lat;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            abort_lat <= 1'b0;
        end else if (init) begin
            abort_lat <= 1'b0;              // abort state reset 
        end else if (abort) begin
            abort_lat <= 1'b1;
        end
    end

    logic same_slave_ok, outstanding_ok;
    assign same_slave_ok  = (outstanding_cnt == 2'd0) || (slave_sel == active_slave);
    assign outstanding_ok = (outstanding_cnt < 2'd2);

    // new AR possible condition check
    logic ar_can_issue;
    // -- Guard --
    // !init : init cycle = Before cur_addr/req_byte_cnt update
    assign ar_can_issue = en && !init && !arvalid && req_pending && !abort_lat &&
                          outstanding_ok && same_slave_ok;

    // AR channel Done = Not exists Request Or Abort assertion 
    logic ar_done;
    assign ar_done = !req_pending || abort_lat;
    // Transfer Done
    assign xfer_done = !init && ar_done && (outstanding_cnt == 2'd0);

    //-----------------------------------------------------------
    //  ① desired_beats      : burst_cfg 
    //  ② beats_to_boundary  : cur_addr ~ 4KB boundary addr
    //                         -> non-alignment burst -> burst length automatic setting (shortest)
    //                            && 4KB boundary check
    //  ③ remain_beats       : length - req_byte_cnt = #remained beat
    //                         -> last burst length < beat => burst length automatic setting (shortest)
    //-----------------------------------------------------------
    logic [LEN_WIDTH-1:0] remain_bytes;
    logic [LEN_WIDTH-1:0] remain_beats;
    logic [LEN_WIDTH-1:0] bytes_to_boundary;
    logic [LEN_WIDTH-1:0] beats_to_boundary;
    logic [LEN_WIDTH-1:0] desired_beats;
    logic [LEN_WIDTH-1:0] safe_beats;      
    // Current burst's #beat (req_pending=1, always 1~256 range)
    // req_pending is request remaining state
    logic [LEN_WIDTH-1:0] safe_bytes;      // safe_beats -> #bytes (beat converts byte)
    logic [BURST_WIDTH-1:0] safe_arlen;    // AXI ARLEN = #beat - 1 (ARLEN = 0 -> 1 beat)

    // calculate remain 
    assign remain_bytes      = length - req_byte_cnt;
    assign remain_beats      = remain_bytes >> ADDR_LSB;

    assign bytes_to_boundary = PAGE_BYTES -
                               {{(LEN_WIDTH-PAGE_LSB){1'b0}}, cur_addr[PAGE_LSB-1:0]};
    assign beats_to_boundary = bytes_to_boundary >> ADDR_LSB;

    assign desired_beats     = {{(LEN_WIDTH-BURST_WIDTH){1'b0}}, burst_cfg} + 1'b1;

    assign safe_beats = (desired_beats < beats_to_boundary) ?
                             ((desired_beats < remain_beats) ? desired_beats : remain_beats) :
                             ((beats_to_boundary < remain_beats) ? beats_to_boundary : remain_beats);

    assign safe_bytes = safe_beats << ADDR_LSB;
    // desired_beats <= 256 -> safe_beats <=256 -> 8bit truncate == safe
    // 32-bit value -> Use only LSB 8-bit 
    assign safe_arlen = safe_beats[BURST_WIDTH-1:0] - 1'b1;

    // Register in Datapath Update
    always_ff @(posedge clk) begin
        if (!rst_n) begin       // reset
            cur_addr       <= '0;
            req_byte_cnt   <= '0;
            total_byte_cnt <= '0;
        end 
        else if (init) begin    // initialize
            cur_addr       <= src_addr;
            req_byte_cnt   <= '0;
            total_byte_cnt <= '0;
        end 
        else begin
            if (r_hs)           // R channel handshake
                total_byte_cnt <= total_byte_cnt + (DATA_WIDTH/8);
            if (ar_hs) begin    // AR channel handshake
                cur_addr     <= cur_addr     + safe_bytes[ADDR_WIDTH-1:0];
                req_byte_cnt <= req_byte_cnt + safe_bytes;
                slot_addr[next_num] <= cur_addr;
            end
        end
    end

    // AR channel
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            arvalid <= 1'b0;
            araddr  <= '0;
            arlen   <= '0;
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
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            fifo_wr_en   <= 1'b0;
            fifo_wr_data <= '0;
        end 
        else begin
            fifo_wr_en   <= r_hs;
            fifo_wr_data <= rdata;  // FIFO write == Read Operation 
        end
    end

endmodule
