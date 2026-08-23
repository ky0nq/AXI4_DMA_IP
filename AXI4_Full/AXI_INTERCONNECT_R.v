module AXI_INTERCONNECT_R #(
    parameter ID_RIDTH   = 5,
    parameter ADDR_RIDTH = 15,
    parameter DATA_RIDTH = 32
)(
    input  wire                     clk,
    input  wire                     RESETN,

    //======================================================
    // CPU Master -> Interconnect : AR Channel
    //======================================================
    input  wire [ID_RIDTH-1:0]      CPU_ARID,
    input  wire [ADDR_RIDTH-1:0]    CPU_ARADDR,
    input  wire [7:0]               CPU_ARLEN,
    input  wire [2:0]               CPU_ARSIZE,
    input  wire [1:0]               CPU_ARBURST,
    input  wire                     CPU_ARLOCK,
    input  wire [3:0]               CPU_ARCACHE,
    input  wire [2:0]               CPU_ARPROT,
    input  wire [3:0]               CPU_ARQOS,
    input  wire                     CPU_ARVALID,
    output reg                      CPU_ARREADY,

    //======================================================
    // DMA Master -> Interconnect : AR Channel
    //======================================================
    input  wire [ID_RIDTH-1:0]      DMA_ARID,
    input  wire [ADDR_RIDTH-1:0]    DMA_ARADDR,
    input  wire [7:0]               DMA_ARLEN,
    input  wire [2:0]               DMA_ARSIZE,
    input  wire [1:0]               DMA_ARBURST,
    input  wire                     DMA_ARLOCK,
    input  wire [3:0]               DMA_ARCACHE,
    input  wire [2:0]               DMA_ARPROT,
    input  wire [3:0]               DMA_ARQOS,
    input  wire                     DMA_ARVALID,
    output reg                      DMA_ARREADY,

    //======================================================
    // CPU Master -> Interconnect : W Channel
    //======================================================
    input  wire [DATA_RIDTH-1:0]       CPU_RDATA,
    input  wire [(DATA_RIDTH/8)-1:0]   CPU_RSTRB,
    input  wire                        CPU_RLAST,
    input  wire                        CPU_RVALID,
    output reg                         CPU_RREADY,

    //======================================================
    // DMA Master -> Interconnect : W Channel
    //======================================================
    input  wire [DATA_RIDTH-1:0]       DMA_RDATA,
    input  wire [(DATA_RIDTH/8)-1:0]   DMA_RSTRB,
    input  wire                        DMA_RLAST,
    input  wire                        DMA_RVALID,
    output reg                         DMA_RREADY,

    //======================================================
    // Interconnect -> Slave : AR Channel
    //======================================================
    output reg  [ID_RIDTH-1:0]      ARID,
    output reg  [ADDR_RIDTH-1:0]    ARADDR,
    output reg  [7:0]               ARLEN,
    output reg  [2:0]               ARSIZE,
    output reg  [1:0]               ARBURST,
    output reg                      ARLOCK,
    output reg  [3:0]               ARCACHE,
    output reg  [2:0]               ARPROT,
    output reg  [3:0]               ARQOS,
    output reg                      AR_MASTER_VALID,
    input  wire                     AR_SLAVE_READY,

    //======================================================
    // Interconnect -> Slave : W Channel
    //======================================================
    output reg  [ID_RIDTH-1:0]         RID,
    output reg  [DATA_RIDTH-1:0]       WDATA,
    output reg  [(DATA_RIDTH/8)-1:0]   WSTRB,
    output reg                         WLAST,
    output reg                         R_MASTER_VALID,
    input  wire                        R_SLAVE_READY,

    //======================================================
    // Slave -> Interconnect : B Channel
    //======================================================
);

    //======================================================
    // Internal Control Signals
    //======================================================
    wire AR_HAND_SHAKE;
    wire R_HAND_SHAKE;

    // 00 : IDLE
    // 01 : CPU
    // 10 : DMA
    reg [1:0] GRANT_EN;

    // Outstanding write table, depth = 2
    reg [1:0] VALID;
    reg [ID_RIDTH-1:0] TABLE_ID [0:1];

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
    assign AR_HAND_SHAKE = AR_MASTER_VALID && AR_SLAVE_READY;
    assign R_HAND_SHAKE  = R_MASTER_VALID  && R_SLAVE_READY;


    //======================================================
    // ARREADY Return Routing
    //======================================================
    always @(*) begin
        CPU_ARREADY = 1'b0;
        DMA_ARREADY = 1'b0;

        case (GRANT_EN)
            2'b01: CPU_ARREADY = AR_SLAVE_READY;
            2'b10: DMA_ARREADY = AR_SLAVE_READY;

            default: begin
                CPU_ARREADY = 1'b0;
                DMA_ARREADY = 1'b0;
            end
        endcase
    end


    //======================================================
    // AR Arbitration
    // Fixed priority : CPU > DMA
    //
    // RESETN polarity is kept the same as the original code:
    // RESETN == 1 -> reset
    //======================================================
    always @(posedge clk) begin
        if (RESETN) begin
            GRANT_EN        <= 2'b00;
            AR_MASTER_VALID <= 1'b0;

            ARID            <= {ID_RIDTH{1'b0}};
            ARADDR          <= {ADDR_RIDTH{1'b0}};
            ARLEN           <= 8'b0;
            ARSIZE          <= 3'b0;
            ARBURST         <= 2'b0;
            ARLOCK          <= 1'b0;
            ARCACHE         <= 4'b0;
            ARPROT          <= 3'b0;
            ARQOS           <= 4'b0;
        end
        else begin
            if (CPU_ARVALID && !(&VALID) && (GRANT_EN == 2'b00)) begin
                AR_MASTER_VALID <= 1'b1;
                GRANT_EN        <= 2'b01;

                ARID            <= CPU_ARID;
                ARADDR          <= CPU_ARADDR;
                ARLEN           <= CPU_ARLEN;
                ARSIZE          <= CPU_ARSIZE;
                ARBURST         <= CPU_ARBURST;
                ARLOCK          <= CPU_ARLOCK;
                ARCACHE         <= CPU_ARCACHE;
                ARPROT          <= CPU_ARPROT;
                ARQOS           <= CPU_ARQOS;
            end
            else if (DMA_ARVALID && !(&VALID) && (GRANT_EN == 2'b00)) begin
                AR_MASTER_VALID <= 1'b1;
                GRANT_EN        <= 2'b10;

                ARID            <= DMA_ARID;
                ARADDR          <= DMA_ARADDR;
                ARLEN           <= DMA_ARLEN;
                ARSIZE          <= DMA_ARSIZE;
                ARBURST         <= DMA_ARBURST;
                ARLOCK          <= DMA_ARLOCK;
                ARCACHE         <= DMA_ARCACHE;
                ARPROT          <= DMA_ARPROT;
                ARQOS           <= DMA_ARQOS;
            end
            else if (AR_HAND_SHAKE) begin
                AR_MASTER_VALID <= 1'b0;
                GRANT_EN        <= 2'b00;
            end
        end
    end


    //======================================================
    // Outstanding Write Table
    // Set   : AR handshake
    // Clear : B handshake
    //======================================================
    always @(posedge clk) begin
        if (RESETN) begin
            VALID       <= 2'b00;
            TABLE_ID[0] <= {ID_RIDTH{1'b0}};
            TABLE_ID[1] <= {ID_RIDTH{1'b0}};
        end
        else begin
            if (R_HAND_SHAKE) begin
                if (VALID[0] && (TABLE_ID[0] == RID)) begin
                    VALID[0] <= 1'b0;
                end
                else if (VALID[1] && (TABLE_ID[1] == RID)) begin
                    VALID[1] <= 1'b0;
                end
            end

            if (AR_HAND_SHAKE) begin
                if (!(&VALID)) begin
                    if (!VALID[0]) begin
                        TABLE_ID[0] <= ARID;
                        VALID[0]    <= 1'b1;
                    end
                    else if (!VALID[1]) begin
                        TABLE_ID[1] <= ARID;
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
    // ARID[3] = 0 -> CPU -> 01
    // ARID[3] = 1 -> DMA -> 10
    //======================================================
    always @(*) begin
        n_OWNER      = OWNER;
        n_NEXT_OWNER = NEXT_OWNER;

        if (AR_HAND_SHAKE && (OWNER == 2'b00)) begin
            n_OWNER      = ARID[3] ? 2'b10 : 2'b01;
            n_NEXT_OWNER = 2'b00;
        end

        else if (AR_HAND_SHAKE &&
                 ((OWNER == 2'b01) || (OWNER == 2'b10)) &&
                 R_HAND_SHAKE && WLAST) begin

            if (NEXT_OWNER == 2'b00) begin
                n_OWNER      = ARID[3] ? 2'b10 : 2'b01;
                n_NEXT_OWNER = 2'b00;
            end
            else begin
                n_OWNER      = NEXT_OWNER;
                n_NEXT_OWNER = ARID[3] ? 2'b10 : 2'b01;
            end
        end

        else if (AR_HAND_SHAKE &&
                 ((OWNER == 2'b01) || (OWNER == 2'b10))) begin
            n_NEXT_OWNER = ARID[3] ? 2'b10 : 2'b01;
        end

        else if (R_HAND_SHAKE && WLAST && (OWNER != 2'b00)) begin
            n_OWNER      = NEXT_OWNER;
            n_NEXT_OWNER = 2'b00;
        end
    end


    //======================================================
    // W Channel MUX
    //======================================================
    always @(*) begin
        WDATA          = {DATA_RIDTH{1'b0}};
        WSTRB          = {(DATA_RIDTH/8){1'b0}};
        WLAST          = 1'b0;
        R_MASTER_VALID = 1'b0;

        CPU_RREADY   = 1'b0;
        DMA_RREADY   = 1'b0;

        case (OWNER)
            2'b01: begin
                WDATA          = CPU_RDATA;
                WSTRB          = CPU_RSTRB;
                WLAST          = CPU_RLAST;
                R_MASTER_VALID = CPU_RVALID;
                CPU_RREADY   = R_SLAVE_READY;
            end

            2'b10: begin
                WDATA          = DMA_RDATA;
                WSTRB          = DMA_RSTRB;
                WLAST          = DMA_RLAST;
                R_MASTER_VALID = DMA_RVALID;
                DMA_RREADY   = R_SLAVE_READY;
            end

            default: begin
                WDATA          = {DATA_RIDTH{1'b0}};
                WSTRB          = {(DATA_RIDTH/8){1'b0}};
                WLAST          = 1'b0;
                R_MASTER_VALID = 1'b0;
                CPU_RREADY   = 1'b0;
                DMA_RREADY   = 1'b0;
            end
        endcase
    end
endmodule
