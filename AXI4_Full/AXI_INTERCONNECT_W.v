module AXI_INTERCONNECT_W #(
    parameter ID_WIDTH   = 5,
    parameter ADDR_WIDTH = 15,
    parameter DATA_WIDTH = 32
)(
    input  wire                     clk,
    input  wire                     RESETN,

    //======================================================
    // CPU Master -> Interconnect : AW Channel
    //======================================================
    input  wire [ID_WIDTH-1:0]      CPU_AWID,
    input  wire [ADDR_WIDTH-1:0]    CPU_AWADDR,
    input  wire [7:0]               CPU_AWLEN,
    input  wire [2:0]               CPU_AWSIZE,
    input  wire [1:0]               CPU_AWBURST,
    input  wire                     CPU_AWLOCK,
    input  wire [3:0]               CPU_AWCACHE,
    input  wire [2:0]               CPU_AWPROT,
    input  wire [3:0]               CPU_AWQOS,
    input  wire                     CPU_AWVALID,
    output reg                      CPU_AWREADY,

    //======================================================
    // DMA Master -> Interconnect : AW Channel
    //======================================================
    input  wire [ID_WIDTH-1:0]      DMA_AWID,
    input  wire [ADDR_WIDTH-1:0]    DMA_AWADDR,
    input  wire [7:0]               DMA_AWLEN,
    input  wire [2:0]               DMA_AWSIZE,
    input  wire [1:0]               DMA_AWBURST,
    input  wire                     DMA_AWLOCK,
    input  wire [3:0]               DMA_AWCACHE,
    input  wire [2:0]               DMA_AWPROT,
    input  wire [3:0]               DMA_AWQOS,
    input  wire                     DMA_AWVALID,
    output reg                      DMA_AWREADY,

    //======================================================
    // CPU Master -> Interconnect : W Channel
    //======================================================
    input  wire [DATA_WIDTH-1:0]       CPU_WDATA,
    input  wire [(DATA_WIDTH/8)-1:0]   CPU_WSTRB,
    input  wire                        CPU_WLAST,
    input  wire                        CPU_WVALID,
    output reg                         CPU_WREADY,

    //======================================================
    // DMA Master -> Interconnect : W Channel
    //======================================================
    input  wire [DATA_WIDTH-1:0]       DMA_WDATA,
    input  wire [(DATA_WIDTH/8)-1:0]   DMA_WSTRB,
    input  wire                        DMA_WLAST,
    input  wire                        DMA_WVALID,
    output reg                         DMA_WREADY,

    //======================================================
    // Interconnect -> CPU Master : B Channel
    //======================================================
    output reg  [ID_WIDTH-1:0]      CPU_BID,
    output reg  [1:0]               CPU_BRESP,
    output reg                      CPU_BVALID,
    input  wire                     CPU_BREADY,

    //======================================================
    // Interconnect -> DMA Master : B Channel
    //======================================================
    output reg  [ID_WIDTH-1:0]      DMA_BID,
    output reg  [1:0]               DMA_BRESP,
    output reg                      DMA_BVALID,
    input  wire                     DMA_BREADY,

    //======================================================
    // Interconnect -> Slave : AW Channel
    //======================================================
    output reg  [ID_WIDTH-1:0]      AWID,
    output reg  [ADDR_WIDTH-1:0]    AWADDR,
    output reg  [7:0]               AWLEN,
    output reg  [2:0]               AWSIZE,
    output reg  [1:0]               AWBURST,
    output reg                      AWLOCK,
    output reg  [3:0]               AWCACHE,
    output reg  [2:0]               AWPROT,
    output reg  [3:0]               AWQOS,
    output reg                      AW_MASTER_VALID,
    input  wire                     AW_SLAVE_READY,

    //======================================================
    // Interconnect -> Slave : W Channel
    //======================================================
    output reg  [DATA_WIDTH-1:0]       WDATA,
    output reg  [(DATA_WIDTH/8)-1:0]   WSTRB,
    output reg                         WLAST,
    output reg                         W_MASTER_VALID,
    input  wire                        W_SLAVE_READY,

    //======================================================
    // Slave -> Interconnect : B Channel
    //======================================================
    input  wire [ID_WIDTH-1:0]      BID,
    input  wire [1:0]               BRESP,
    input  wire                     SLAVE_BVALID,
    output reg                      SLAVE_BREADY
);

    //======================================================
    // Internal Control Signals
    //======================================================
    wire AW_HAND_SHAKE;
    wire W_HAND_SHAKE;
    wire B_HAND_SHAKE;

    // 00 : IDLE
    // 01 : CPU
    // 10 : DMA
    reg [1:0] GRANT_EN;

    // Outstanding write table, depth = 2
    reg [1:0] VALID;
    reg [ID_WIDTH-1:0] TABLE_ID [0:1];

    // W channel owner queue
    // 00 : NONE
    // 01 : CPU
    // 10 : DMA
    reg [1:0] OWNER;
    reg [1:0] NEXT_OWNER;
    reg [1:0] n_OWNER;
    reg [1:0] n_NEXT_OWNER;


    //======================================================
    // Handshake
    //======================================================
    assign AW_HAND_SHAKE = AW_MASTER_VALID && AW_SLAVE_READY;
    assign W_HAND_SHAKE  = W_MASTER_VALID  && W_SLAVE_READY;
    assign B_HAND_SHAKE  = SLAVE_BVALID  && SLAVE_BREADY;


    //======================================================
    // AWREADY Return Routing
    //======================================================
    always @(*) begin
        CPU_AWREADY = 1'b0;
        DMA_AWREADY = 1'b0;

        case (GRANT_EN)
            2'b01: CPU_AWREADY = AW_SLAVE_READY;
            2'b10: DMA_AWREADY = AW_SLAVE_READY;

            default: begin
                CPU_AWREADY = 1'b0;
                DMA_AWREADY = 1'b0;
            end
        endcase
    end


    //======================================================
    // AW Arbitration
    // Fixed priority : CPU > DMA
    //
    // RESETN polarity is kept the same as the original code:
    // RESETN == 1 -> reset
    //======================================================
    always @(posedge clk) begin
        if (RESETN) begin
            GRANT_EN        <= 2'b00;
            AW_MASTER_VALID <= 1'b0;

            AWID            <= {ID_WIDTH{1'b0}};
            AWADDR          <= {ADDR_WIDTH{1'b0}};
            AWLEN           <= 8'b0;
            AWSIZE          <= 3'b0;
            AWBURST         <= 2'b0;
            AWLOCK          <= 1'b0;
            AWCACHE         <= 4'b0;
            AWPROT          <= 3'b0;
            AWQOS           <= 4'b0;
        end
        else begin
            if (CPU_AWVALID && !(&VALID) && (GRANT_EN == 2'b00)) begin
                AW_MASTER_VALID <= 1'b1;
                GRANT_EN        <= 2'b01;

                AWID            <= CPU_AWID;
                AWADDR          <= CPU_AWADDR;
                AWLEN           <= CPU_AWLEN;
                AWSIZE          <= CPU_AWSIZE;
                AWBURST         <= CPU_AWBURST;
                AWLOCK          <= CPU_AWLOCK;
                AWCACHE         <= CPU_AWCACHE;
                AWPROT          <= CPU_AWPROT;
                AWQOS           <= CPU_AWQOS;
            end
            else if (DMA_AWVALID && !(&VALID) && (GRANT_EN == 2'b00)) begin
                AW_MASTER_VALID <= 1'b1;
                GRANT_EN        <= 2'b10;

                AWID            <= DMA_AWID;
                AWADDR          <= DMA_AWADDR;
                AWLEN           <= DMA_AWLEN;
                AWSIZE          <= DMA_AWSIZE;
                AWBURST         <= DMA_AWBURST;
                AWLOCK          <= DMA_AWLOCK;
                AWCACHE         <= DMA_AWCACHE;
                AWPROT          <= DMA_AWPROT;
                AWQOS           <= DMA_AWQOS;
            end
            else if (AW_HAND_SHAKE) begin
                AW_MASTER_VALID <= 1'b0;
                GRANT_EN        <= 2'b00;
            end
        end
    end


    //======================================================
    // Outstanding Write Table
    // Set   : AW handshake
    // Clear : B handshake
    //======================================================
    always @(posedge clk) begin
        if (RESETN) begin
            VALID       <= 2'b00;
            TABLE_ID[0] <= {ID_WIDTH{1'b0}};
            TABLE_ID[1] <= {ID_WIDTH{1'b0}};
        end
        else begin
            if (B_HAND_SHAKE) begin
                if (VALID[0] && (TABLE_ID[0] == BID)) begin
                    VALID[0] <= 1'b0;
                end
                else if (VALID[1] && (TABLE_ID[1] == BID)) begin
                    VALID[1] <= 1'b0;
                end
            end

            if (AW_HAND_SHAKE) begin
                if (!(&VALID)) begin
                    if (!VALID[0]) begin
                        TABLE_ID[0] <= AWID;
                        VALID[0]    <= 1'b1;
                    end
                    else if (!VALID[1]) begin
                        TABLE_ID[1] <= AWID;
                        VALID[1]    <= 1'b1;
                    end
                end
            end
        end
    end


    //======================================================
    // W Owner Queue Register
    //======================================================
    always @(posedge clk) begin
        if (RESETN) begin
            OWNER      <= 2'b00;
            NEXT_OWNER <= 2'b00;
        end
        else begin
            OWNER      <= n_OWNER;
            NEXT_OWNER <= n_NEXT_OWNER;
        end
    end


    //======================================================
    // W Owner Queue Next-State
    //
    // AWID[3] = 0 -> CPU -> 01
    // AWID[3] = 1 -> DMA -> 10
    //======================================================
    always @(*) begin
        n_OWNER      = OWNER;
        n_NEXT_OWNER = NEXT_OWNER;

        if (AW_HAND_SHAKE && (OWNER == 2'b00)) begin
            n_OWNER      = AWID[3] ? 2'b10 : 2'b01;
            n_NEXT_OWNER = 2'b00;
        end

        else if (AW_HAND_SHAKE &&
                 ((OWNER == 2'b01) || (OWNER == 2'b10)) &&
                 W_HAND_SHAKE && WLAST) begin

            if (NEXT_OWNER == 2'b00) begin
                n_OWNER      = AWID[3] ? 2'b10 : 2'b01;
                n_NEXT_OWNER = 2'b00;
            end
            else begin
                n_OWNER      = NEXT_OWNER;
                n_NEXT_OWNER = AWID[3] ? 2'b10 : 2'b01;
            end
        end

        else if (AW_HAND_SHAKE &&
                 ((OWNER == 2'b01) || (OWNER == 2'b10))) begin
            n_NEXT_OWNER = AWID[3] ? 2'b10 : 2'b01;
        end

        else if (W_HAND_SHAKE && WLAST && (OWNER != 2'b00)) begin
            n_OWNER      = NEXT_OWNER;
            n_NEXT_OWNER = 2'b00;
        end
    end


    //======================================================
    // W Channel MUX
    //======================================================
    always @(*) begin
        WDATA          = {DATA_WIDTH{1'b0}};
        WSTRB          = {(DATA_WIDTH/8){1'b0}};
        WLAST          = 1'b0;
        W_MASTER_VALID = 1'b0;

        CPU_WREADY   = 1'b0;
        DMA_WREADY   = 1'b0;

        case (OWNER)
            2'b01: begin
                WDATA          = CPU_WDATA;
                WSTRB          = CPU_WSTRB;
                WLAST          = CPU_WLAST;
                W_MASTER_VALID = CPU_WVALID;
                CPU_WREADY   = W_SLAVE_READY;
            end

            2'b10: begin
                WDATA          = DMA_WDATA;
                WSTRB          = DMA_WSTRB;
                WLAST          = DMA_WLAST;
                W_MASTER_VALID = DMA_WVALID;
                DMA_WREADY   = W_SLAVE_READY;
            end

            default: begin
                WDATA          = {DATA_WIDTH{1'b0}};
                WSTRB          = {(DATA_WIDTH/8){1'b0}};
                WLAST          = 1'b0;
                W_MASTER_VALID = 1'b0;
                CPU_WREADY   = 1'b0;
                DMA_WREADY   = 1'b0;
            end
        endcase
    end


    //======================================================
    // B Response Router
    //
    // BID[3] = 0 -> CPU
    // BID[3] = 1 -> DMA
    //======================================================
    always @(*) begin
        CPU_BID      = BID;
        CPU_BRESP    = BRESP;
        CPU_BVALID   = 1'b0;

        DMA_BID      = BID;
        DMA_BRESP    = BRESP;
        DMA_BVALID   = 1'b0;

        SLAVE_BREADY = 1'b0;

        if (SLAVE_BVALID) begin
            if (BID[3] == 1'b0) begin
                CPU_BVALID   = 1'b1;
                SLAVE_BREADY = CPU_BREADY;
            end
            else begin
                DMA_BVALID   = 1'b1;
                SLAVE_BREADY = DMA_BREADY;
            end
        end
    end

endmodule
