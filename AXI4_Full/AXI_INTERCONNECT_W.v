module AXI_INTERCONNECT_W #(
    parameter ID_WIDTH   = 4,
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
    // Interconnect <-> ROM Slave : Write Channel
    //======================================================

    // AW Channel : Interconnect -> ROM
    output reg  [ID_WIDTH-1:0]         ROM_AWID,
    output reg  [ADDR_WIDTH-1:0]       ROM_AWADDR,
    output reg  [7:0]                  ROM_AWLEN,
    output reg  [2:0]                  ROM_AWSIZE,
    output reg  [1:0]                  ROM_AWBURST,
    output reg                         ROM_AWLOCK,
    output reg  [3:0]                  ROM_AWCACHE,
    output reg  [2:0]                  ROM_AWPROT,
    output reg  [3:0]                  ROM_AWQOS,
    output reg                         ROM_AWVALID,
    input  wire                        ROM_AWREADY,

    // W Channel : Interconnect -> ROM
    output reg  [DATA_WIDTH-1:0]       ROM_WDATA,
    output reg  [(DATA_WIDTH/8)-1:0]   ROM_WSTRB,
    output reg                         ROM_WLAST,
    output reg                         ROM_WVALID,
    input  wire                        ROM_WREADY,

    // B Channel : ROM -> Interconnect
    input  wire [ID_WIDTH-1:0]         ROM_BID,
    input  wire [1:0]                  ROM_BRESP,
    input  wire                        ROM_BVALID,
    output reg                         ROM_BREADY,

    //======================================================
    // Interconnect <-> RAM Slave : Write Channel
    //======================================================

    // AW Channel : Interconnect -> RAM
    output reg  [ID_WIDTH-1:0]         RAM_AWID,
    output reg  [ADDR_WIDTH-1:0]       RAM_AWADDR,
    output reg  [7:0]                  RAM_AWLEN,
    output reg  [2:0]                  RAM_AWSIZE,
    output reg  [1:0]                  RAM_AWBURST,
    output reg                         RAM_AWLOCK,
    output reg  [3:0]                  RAM_AWCACHE,
    output reg  [2:0]                  RAM_AWPROT,
    output reg  [3:0]                  RAM_AWQOS,
    output reg                         RAM_AWVALID,
    input  wire                        RAM_AWREADY,

    // W Channel : Interconnect -> RAM
    output reg  [DATA_WIDTH-1:0]       RAM_WDATA,
    output reg  [(DATA_WIDTH/8)-1:0]   RAM_WSTRB,
    output reg                         RAM_WLAST,
    output reg                         RAM_WVALID,
    input  wire                        RAM_WREADY,

    // B Channel : RAM -> Interconnect
    input  wire [ID_WIDTH-1:0]         RAM_BID,
    input  wire [1:0]                  RAM_BRESP,
    input  wire                        RAM_BVALID,
    output reg                         RAM_BREADY
);
	reg	[1:0]					QOS;
	reg						MASTER_AWVALID;
    reg                     MASTER_WVALID;
    reg                     MASTER_BREADY;

    reg                     SLAVE_AWREADY;
    reg                     SLAVE_WREADY;
    reg                     SLAVE_BVALID;

    //======================================================
    // Internal Common AW Channel
    //======================================================
    reg [ID_WIDTH-1:0]      AWID;
    reg [ADDR_WIDTH-1:0]    AWADDR;
    reg [7:0]               AWLEN;
    reg [2:0]               AWSIZE;
    reg [1:0]               AWBURST;
    reg                     AWLOCK;
    reg [3:0]               AWCACHE;
    reg [2:0]               AWPROT;
    reg [3:0]               AWQOS;

    //======================================================
    // Internal Common W Channel
    //======================================================
    reg [DATA_WIDTH-1:0]       WDATA;
    reg [(DATA_WIDTH/8)-1:0]   WSTRB;
    reg                        WLAST;

    //======================================================
    // Internal Common B Channel
    //======================================================
    reg [ID_WIDTH-1:0]      BID;
    reg [1:0]               BRESP;


    //======================================================
    // Slave -> Interconnect : B Channel
    //======================================================
    
    wire AW_HAND_SHAKE;
    wire W_HAND_SHAKE;
    wire B_HAND_SHAKE;

    reg [1:0] M_GRANT_EN;
    reg [1:0] S_GRANT_EN;

    reg [1:0] VALID;
    reg [ID_WIDTH-1:0] TABLE_ID [0:1];

    reg [1:0] MOWNER;
    reg [1:0] NEXT_MOWNER;
    reg [1:0] n_MOWNER;
    reg [1:0] n_NEXT_MOWNER;
    
	reg [1:0] SOWNER;
    reg [1:0] NEXT_SOWNER;
    reg [1:0] n_SOWNER;
    reg [1:0] n_NEXT_SOWNER;

    //======================================================
    // Handshake
    //======================================================
    assign AW_HAND_SHAKE = MASTER_AWVALID && SLAVE_AWREADY;
    assign W_HAND_SHAKE  = MASTER_WVALID  && SLAVE_WREADY;
    assign B_HAND_SHAKE  = SLAVE_BVALID  && MASTER_BREADY;

    //======================================================
    // AWREADY Return Routing
    //======================================================
    always @(*) begin
        CPU_AWREADY = 1'b0;
        DMA_AWREADY = 1'b0;

        case (M_GRANT_EN)
            2'b01: CPU_AWREADY = SLAVE_AWREADY;
            2'b10: DMA_AWREADY = SLAVE_AWREADY;

            default: begin
                CPU_AWREADY = 1'b0;
                DMA_AWREADY = 1'b0;
            end
        endcase
    end
    
	always @(*) begin
        SLAVE_AWREADY = 1'b0;

        case (S_GRANT_EN)
            2'b01: SLAVE_AWREADY = ROM_AWREADY;
            2'b10: SLAVE_AWREADY = RAM_AWREADY;

            default: begin
				SLAVE_AWREADY = 1'b0;
            end
        endcase
    end
	
	always @(*) begin
		if(CPU_AWVALID && DMA_AWVALID)begin
			if(CPU_AWQOS >= DMA_AWQOS)	QOS = 2'd1;	
			else						QOS = 2'd2;	
		end
		else QOS = 0;
    end

    always @(posedge clk) begin
        if (!RESETN) begin
            M_GRANT_EN        <= 2'b00;
            S_GRANT_EN        <= 2'b00;
            MASTER_AWVALID <= 1'b0;

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
            if (CPU_AWVALID && !(&VALID) && (M_GRANT_EN == 2'b00) && (QOS== 0 || QOS == 2'd1)) begin
                MASTER_AWVALID <= 1'b1;
                M_GRANT_EN        <= 2'b01;
                S_GRANT_EN        <= CPU_AWID[2:1];

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
            else if (DMA_AWVALID && !(&VALID) && (M_GRANT_EN == 2'b00) && (QOS== 0 || QOS == 2'd2)) begin
                MASTER_AWVALID <= 1'b1;
                M_GRANT_EN        <= 2'b10;
                S_GRANT_EN        <= DMA_AWID[2:1];
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
                MASTER_AWVALID <= 1'b0;
                S_GRANT_EN        <= 2'd0;
                M_GRANT_EN        <= 2'd0;
            end
        end
    end
    
    always @(posedge clk) begin
        if (!RESETN) begin
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
        if (!RESETN) begin
            MOWNER      <= 2'b00;
            NEXT_MOWNER <= 2'b00;
        end
        else begin
            MOWNER      <= n_MOWNER;
            NEXT_MOWNER <= n_NEXT_MOWNER;
        end
    end

    always @(*) begin
        n_MOWNER      = MOWNER;
        n_NEXT_MOWNER = NEXT_MOWNER;

        if (AW_HAND_SHAKE && (MOWNER == 2'b00)) begin
            n_MOWNER      = AWID[3] ? 2'b10 : 2'b01;
            n_NEXT_MOWNER = 2'b00;
        end

        else if (AW_HAND_SHAKE &&
                 ((MOWNER == 2'b01) || (MOWNER == 2'b10)) &&
                 W_HAND_SHAKE && WLAST) begin

            if (NEXT_MOWNER == 2'b00) begin
                n_MOWNER      = AWID[3] ? 2'b10 : 2'b01;
                n_NEXT_MOWNER = 2'b00;
            end
            else begin
                n_MOWNER      = NEXT_MOWNER;
                n_NEXT_MOWNER = AWID[3] ? 2'b10 : 2'b01;
            end
        end

        else if (AW_HAND_SHAKE &&
                 ((MOWNER == 2'b01) || (MOWNER == 2'b10))) begin
            n_NEXT_MOWNER = AWID[3] ? 2'b10 : 2'b01;
        end

        else if (W_HAND_SHAKE && WLAST && (MOWNER != 2'b00)) begin
            n_MOWNER      = NEXT_MOWNER;
            n_NEXT_MOWNER = 2'b00;
        end
    end

    always @(posedge clk) begin
        if (!RESETN) begin
            SOWNER      <= 2'b00;
            NEXT_SOWNER <= 2'b00;
        end
        else begin
            SOWNER      <= n_SOWNER;
            NEXT_SOWNER <= n_NEXT_SOWNER;
        end
    end

    always @(*) begin
        n_SOWNER      = SOWNER;
        n_NEXT_SOWNER = NEXT_SOWNER;

        if (AW_HAND_SHAKE && (SOWNER == 2'b00)) begin
            n_SOWNER      = AWID[2:1]+1;
            n_NEXT_SOWNER = 2'b00;
        end

        else if (AW_HAND_SHAKE && ((SOWNER == 2'b01) || (SOWNER == 2'b10)) &&
                 W_HAND_SHAKE && WLAST) begin

            if (NEXT_SOWNER == 2'b00) begin
                n_SOWNER      = AWID[2:1]+1;
                n_NEXT_SOWNER = 2'b00;
            end
            else begin
                n_SOWNER      = NEXT_SOWNER;
                n_NEXT_SOWNER = AWID[2:1]+1;
            end
        end

        else if (AW_HAND_SHAKE && ((SOWNER == 2'b01) || (SOWNER == 2'b10))) begin
            n_NEXT_SOWNER = AWID[2:1]+1;
        end

        else if (W_HAND_SHAKE && WLAST && (SOWNER != 2'b00)) begin
            n_SOWNER      = NEXT_SOWNER;
            n_NEXT_SOWNER = 2'b00;
        end
    end

    always @(*) begin
        WDATA          = {DATA_WIDTH{1'b0}};
        WSTRB          = {(DATA_WIDTH/8){1'b0}};
        WLAST          = 1'b0;
        MASTER_WVALID  = 1'b0;

        CPU_WREADY   = 1'b0;
        DMA_WREADY   = 1'b0;

        case (MOWNER)
            2'b01: begin
                  WDATA         =  CPU_WDATA     ;
                  WSTRB         =  CPU_WSTRB     ;
                  WLAST         =  CPU_WLAST     ;
                  MASTER_WVALID =  CPU_WVALID    ;
                  CPU_WREADY   = SLAVE_WREADY;
            end

            2'b10: begin
                  WDATA         = DMA_WDATA     ;
                  WSTRB         = DMA_WSTRB     ;
                  WLAST         = DMA_WLAST     ;
                  MASTER_WVALID = DMA_WVALID    ;
                  DMA_WREADY   = SLAVE_WREADY;
            end
            default: begin
                WDATA          = {DATA_WIDTH{1'b0}};
                WSTRB          = {(DATA_WIDTH/8){1'b0}};
                WLAST          = 1'b0;
                MASTER_WVALID = 1'b0;
                CPU_WREADY   = 1'b0;
                DMA_WREADY   = 1'b0;
            end
        endcase
    end
    
    always @(*) begin
        ROM_WDATA     = {DATA_WIDTH{1'b0}};
        ROM_WSTRB     = {(DATA_WIDTH/8){1'b0}};
        ROM_WLAST     = 1'b0;
        ROM_WVALID    = 1'b0;
        SLAVE_WREADY  = 1'b0;
        
        RAM_WDATA     = {DATA_WIDTH{1'b0}};
        RAM_WSTRB     = {(DATA_WIDTH/8){1'b0}};
        RAM_WLAST     = 1'b0;
        RAM_WVALID    = 1'b0;
        SLAVE_WREADY  = 1'b0;

        case (SOWNER)
            2'b01: begin
                  ROM_WDATA     =  WDATA        ;
                  ROM_WSTRB     =  WSTRB        ;
                  ROM_WLAST     =  WLAST        ;
                  ROM_WVALID    =  MASTER_WVALID;
                  SLAVE_WREADY  =  ROM_WREADY   ;
            end

            2'b10: begin
                  RAM_WDATA     =  WDATA        ;
                  RAM_WSTRB     =  WSTRB        ;
                  RAM_WLAST     =  WLAST        ;
                  RAM_WVALID    =  MASTER_WVALID;
                  SLAVE_WREADY  = RAM_WREADY    ;
            end
            default: begin
                  ROM_WDATA     = {DATA_WIDTH{1'b0}};
                  ROM_WSTRB     = {(DATA_WIDTH/8){1'b0}};
                  ROM_WLAST     = 1'b0;
                  ROM_WVALID    = 1'b0;
                  
                  RAM_WDATA     = {DATA_WIDTH{1'b0}};
                  RAM_WSTRB     = {(DATA_WIDTH/8){1'b0}};
                  RAM_WLAST     = 1'b0;
                  RAM_WVALID    = 1'b0;
                  
                  SLAVE_WREADY  = 1'b0;
            end
        endcase
    end

    always @(*) begin
        BID          = {ID_WIDTH{1'b0}};
        BRESP        = 2'b00;
        SLAVE_BVALID = 1'b0;

        ROM_BREADY   = 1'b0;
        RAM_BREADY   = 1'b0;

        if (ROM_BVALID) begin
            BID          = ROM_BID;
            BRESP        = ROM_BRESP;
            SLAVE_BVALID = 1'b1;
            ROM_BREADY   = MASTER_BREADY;
        end
        else if (RAM_BVALID) begin
            BID          = RAM_BID;
            BRESP        = RAM_BRESP;
            SLAVE_BVALID = 1'b1;
            RAM_BREADY   = MASTER_BREADY;
        end
    end

    always @(*) begin
        CPU_BID      = BID;
        CPU_BRESP    = BRESP;
        CPU_BVALID   = 1'b0;

        DMA_BID      = BID;
        DMA_BRESP    = BRESP;
        DMA_BVALID   = 1'b0;

        MASTER_BREADY = 1'b0;

        if (SLAVE_BVALID) begin
            if (BID[3] == 1'b0) begin
                CPU_BVALID   = 1'b1;
                MASTER_BREADY = CPU_BREADY;
            end
            else begin
                DMA_BVALID   = 1'b1;
                MASTER_BREADY = DMA_BREADY;
            end
        end
    end

    always @(*)begin
        ROM_AWID      = 0;
        ROM_AWADDR    = 0;
        ROM_AWLEN     = 0;
        ROM_AWSIZE    = 0;
        ROM_AWBURST   = 0;
        ROM_AWLOCK    = 0;
        ROM_AWCACHE   = 0;
        ROM_AWPROT    = 0;
        ROM_AWQOS     = 0;
        ROM_AWVALID   = 0;
        RAM_AWID      = 0;
        RAM_AWADDR    = 0;
        RAM_AWLEN     = 0;
        RAM_AWSIZE    = 0;
        RAM_AWBURST   = 0;
        RAM_AWLOCK    = 0;
        RAM_AWCACHE   = 0;
        RAM_AWPROT    = 0;
        RAM_AWQOS     = 0;
        RAM_AWVALID   = 0;
        if(S_GRANT_EN== 2'b01)begin
            ROM_AWID      = AWID     ;
            ROM_AWADDR    = AWADDR   ;
            ROM_AWLEN     = AWLEN    ;
            ROM_AWSIZE    = AWSIZE   ;
            ROM_AWBURST   = AWBURST  ;
            ROM_AWLOCK    = AWLOCK   ;
            ROM_AWCACHE   = AWCACHE  ;
            ROM_AWPROT    = AWPROT   ;
            ROM_AWQOS     = AWQOS    ;
            ROM_AWVALID   = MASTER_AWVALID;
        end 
        else if(S_GRANT_EN== 2'b10)begin
            RAM_AWID      = AWID     ;
            RAM_AWADDR    = AWADDR   ;
            RAM_AWLEN     = AWLEN    ;
            RAM_AWSIZE    = AWSIZE   ;
            RAM_AWBURST   = AWBURST  ;
            RAM_AWLOCK    = AWLOCK   ;
            RAM_AWCACHE   = AWCACHE  ;
            RAM_AWPROT    = AWPROT   ;
            RAM_AWQOS     = AWQOS    ;
            RAM_AWVALID   = MASTER_AWVALID;
        end 
    end

endmodule
