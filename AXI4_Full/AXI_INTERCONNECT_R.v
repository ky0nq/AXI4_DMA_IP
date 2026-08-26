module AXI_INTERCONNECT_R #(
    parameter ID_WIDTH   = 4,
    parameter ADDR_WIDTH = 15,
    parameter DATA_WIDTH = 32
)(
    input  wire                     clk,
    input  wire                     RESETN,

    //======================================================
    // CPU Master -> Interconnect : AR Channel
    //======================================================
    input  wire [ID_WIDTH-1:0]      CPU_ARID,
    input  wire [ADDR_WIDTH-1:0]    CPU_ARADDR,
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
    input  wire [ID_WIDTH-1:0]      DMA_ARID,
    input  wire [ADDR_WIDTH-1:0]    DMA_ARADDR,
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
    // Interconnect -> CPU Master : R Channel
    //======================================================
    output reg  [ID_WIDTH-1:0]      CPU_RID,
    output reg  [DATA_WIDTH-1:0]    CPU_RDATA,
    output reg  [1:0]               CPU_RRESP,
    output reg                      CPU_RLAST,
    output reg                      CPU_RVALID,
    input  wire                     CPU_RREADY,

    //======================================================
    // Interconnect -> DMA Master : R Channel
    //======================================================
    output reg  [ID_WIDTH-1:0]      DMA_RID,
    output reg  [DATA_WIDTH-1:0]    DMA_RDATA,
    output reg  [1:0]               DMA_RRESP,
    output reg                      DMA_RLAST,
    output reg                      DMA_RVALID,
    input  wire                     DMA_RREADY,

    //======================================================
    // Interconnect <-> ROM Slave : Read Channel
    //======================================================

    // AR Channel : Interconnect -> ROM
    output reg  [ID_WIDTH-1:0]      ROM_ARID,
    output reg  [ADDR_WIDTH-1:0]    ROM_ARADDR,
    output reg  [7:0]               ROM_ARLEN,
    output reg  [2:0]               ROM_ARSIZE,
    output reg  [1:0]               ROM_ARBURST,
    output reg                      ROM_ARLOCK,
    output reg  [3:0]               ROM_ARCACHE,
    output reg  [2:0]               ROM_ARPROT,
    output reg  [3:0]               ROM_ARQOS,
    output reg                      ROM_ARVALID,
    input  wire                     ROM_ARREADY,

    // R Channel : ROM -> Interconnect
    input  wire [ID_WIDTH-1:0]      ROM_RID,
    input  wire [DATA_WIDTH-1:0]    ROM_RDATA,
    input  wire [1:0]               ROM_RRESP,
    input  wire                     ROM_RLAST,
    input  wire                     ROM_RVALID,
    output reg                      ROM_RREADY,

    //======================================================
    // Interconnect <-> RAM Slave : Read Channel
    //======================================================

    // AR Channel : Interconnect -> RAM
    output reg  [ID_WIDTH-1:0]      RAM_ARID,
    output reg  [ADDR_WIDTH-1:0]    RAM_ARADDR,
    output reg  [7:0]               RAM_ARLEN,
    output reg  [2:0]               RAM_ARSIZE,
    output reg  [1:0]               RAM_ARBURST,
    output reg                      RAM_ARLOCK,
    output reg  [3:0]               RAM_ARCACHE,
    output reg  [2:0]               RAM_ARPROT,
    output reg  [3:0]               RAM_ARQOS,
    output reg                      RAM_ARVALID,
    input  wire                     RAM_ARREADY,

    // R Channel : RAM -> Interconnect
    input  wire [ID_WIDTH-1:0]      RAM_RID,
    input  wire [DATA_WIDTH-1:0]    RAM_RDATA,
    input  wire [1:0]               RAM_RRESP,
    input  wire                     RAM_RLAST,
    input  wire                     RAM_RVALID,
    output reg                      RAM_RREADY
);

    //======================================================
    // Internal Common AR Channel
    //======================================================
    reg [ID_WIDTH-1:0]      ARID;
    reg [ADDR_WIDTH-1:0]    ARADDR;
    reg [7:0]               ARLEN;
    reg [2:0]               ARSIZE;
    reg [1:0]               ARBURST;
    reg                     ARLOCK;
    reg [3:0]               ARCACHE;
    reg [2:0]               ARPROT;
    reg [3:0]               ARQOS;

    reg [1:0]                    QOS;
    reg                     AR_MASTER_VALID;
    reg                     AR_SLAVE_READY;

    //======================================================
    // Internal Common R Channel
    //======================================================
    reg [ID_WIDTH-1:0]      RID;
    reg [DATA_WIDTH-1:0]    RDATA;
    reg [1:0]               RRESP;
    reg                     RLAST;

    reg                     R_SLAVE_VALID;
    reg                     R_MASTER_READY;
    
	reg [1:0] OWNER;
    reg [1:0] NEXT_OWNER;
    reg [1:0] n_OWNER;
    reg [1:0] n_NEXT_OWNER;

    wire AR_HAND_SHAKE;
    wire R_HAND_SHAKE;

    reg [1:0] M_GRANT_EN;
    reg [1:0] S_GRANT_EN;

    reg [1:0] VALID;
    reg [ID_WIDTH-1:0] TABLE_ID [0:1];

    assign AR_HAND_SHAKE = AR_MASTER_VALID && AR_SLAVE_READY;
    assign R_HAND_SHAKE  = R_SLAVE_VALID && R_MASTER_READY;
    
	always @(*) begin
        CPU_ARREADY = 1'b0;
        DMA_ARREADY = 1'b0;

        case (M_GRANT_EN)
            2'b01: CPU_ARREADY = AR_SLAVE_READY;
            2'b10: DMA_ARREADY = AR_SLAVE_READY;

            default: begin
                CPU_ARREADY = 1'b0;
                DMA_ARREADY = 1'b0;
            end
        endcase
    end
	
 	always @(*) begin
		if(CPU_ARVALID && DMA_ARVALID)begin
			if(CPU_ARQOS >= DMA_ARQOS)	QOS = 2'd1;	
			else						QOS = 2'd2;	
		end
		else QOS = 0;
    end   
	always @(posedge clk) begin
        if (!RESETN) begin
            M_GRANT_EN        <= 2'b00;
            S_GRANT_EN        <= 2'b00;
            AR_MASTER_VALID <= 1'b0;
            ARID            <= {ID_WIDTH{1'b0}};
            ARADDR          <= {ADDR_WIDTH{1'b0}};
            ARLEN           <= 8'b0;
            ARSIZE          <= 3'b0;
            ARBURST         <= 2'b0;
            ARLOCK          <= 1'b0;
            ARCACHE         <= 4'b0;
            ARPROT          <= 3'b0;
            ARQOS           <= 4'b0;
        end
        else begin
            if (CPU_ARVALID && !(&VALID) && (M_GRANT_EN == 2'b00) && (QOS== 0 || QOS == 2'd1)) begin
                AR_MASTER_VALID <= 1'b1;
                M_GRANT_EN        <= 2'b01;
		        S_GRANT_EN        <= CPU_ARID[2:1] + 2'b01;
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
            else if (DMA_ARVALID && !(&VALID) && (M_GRANT_EN == 2'b00) && (QOS== 0 || QOS == 2'd2)) begin
                AR_MASTER_VALID <= 1'b1;
                M_GRANT_EN        <= 2'b10;
		        S_GRANT_EN        <= DMA_ARID[2:1] + 2'b01;
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
                M_GRANT_EN        <= 2'b00;
		        S_GRANT_EN        <= 2'b00;
            end
        end
    end
	
	always @(*) begin
	    // Default : ROM
	    ROM_ARID    = 0;
	    ROM_ARADDR  = 0;
	    ROM_ARLEN   = 0;
	    ROM_ARSIZE  = 0;
	    ROM_ARBURST = 0;
	    ROM_ARLOCK  = 0;
	    ROM_ARCACHE = 0;
	    ROM_ARPROT  = 0;
	    ROM_ARQOS   = 0;
	    ROM_ARVALID = 1'b0;
	
	    // Default : RAM
	    RAM_ARID    = 0;
	    RAM_ARADDR  = 0;
	    RAM_ARLEN   = 0;
	    RAM_ARSIZE  = 0;
	    RAM_ARBURST = 0;
	    RAM_ARLOCK  = 0;
	    RAM_ARCACHE = 0;
	    RAM_ARPROT  = 0;
	    RAM_ARQOS   = 0;
	    RAM_ARVALID = 0;

	    AR_SLAVE_READY = 1'b0;
	
	    case (S_GRANT_EN)
	
	        2'b01: begin
	            ROM_ARVALID    = AR_MASTER_VALID;
				ROM_ARID    = ARID;
	    		ROM_ARADDR  = ARADDR;
	    		ROM_ARLEN   = ARLEN;
	    		ROM_ARSIZE  = ARSIZE;
	    		ROM_ARBURST = ARBURST;
	    		ROM_ARLOCK  = ARLOCK;
	    		ROM_ARCACHE = ARCACHE;
	    		ROM_ARPROT  = ARPROT;
	    		ROM_ARQOS   = ARQOS;
	            AR_SLAVE_READY = ROM_ARREADY;
	        end
	
	        2'b10: begin
	            RAM_ARVALID    = AR_MASTER_VALID;
	            AR_SLAVE_READY = RAM_ARREADY;
				RAM_ARID    = ARID;
	    		RAM_ARADDR  = ARADDR;
	    		RAM_ARLEN   = ARLEN;
	    		RAM_ARSIZE  = ARSIZE;
	    		RAM_ARBURST = ARBURST;
	    		RAM_ARLOCK  = ARLOCK;
	    		RAM_ARCACHE = ARCACHE;
	    		RAM_ARPROT  = ARPROT;
	    		RAM_ARQOS   = ARQOS;
	        end
	
	        default: begin
	            ROM_ARVALID    = 1'b0;
	            RAM_ARVALID    = 1'b0;
	            AR_SLAVE_READY = 1'b0;
	        end
	
	    endcase
	end
	    
	always @(posedge clk) begin
        if (!RESETN) begin
            VALID       <= 2'b00;
            TABLE_ID[0] <= {ID_WIDTH{1'b0}};
            TABLE_ID[1] <= {ID_WIDTH{1'b0}};
        end
        else begin
            if (R_HAND_SHAKE && RLAST) begin
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
	
	always @(posedge clk) begin
        if (!RESETN) begin
            OWNER      <= 2'b00;
            NEXT_OWNER <= 2'b00;
        end
        else begin
            OWNER      <= n_OWNER;
            NEXT_OWNER <= n_NEXT_OWNER;
        end
    end
    
    always @(*) begin
        n_OWNER      = OWNER;
        n_NEXT_OWNER = NEXT_OWNER;
    
        if (AR_HAND_SHAKE && (OWNER == 2'b00)) begin
            n_OWNER      = ARID[2:1]+1;
            n_NEXT_OWNER = 2'b00;
        end
    
        else if (AR_HAND_SHAKE && ((OWNER == 2'b01) || (OWNER == 2'b10)) &&
                 R_HAND_SHAKE && RLAST) begin
    
            if (NEXT_OWNER == 2'b00) begin
				n_OWNER      = ARID[2:1]+1;
                n_NEXT_OWNER = 2'b00;
            end
            else begin
                n_OWNER      = NEXT_OWNER;
                n_NEXT_OWNER = ARID[2:1]+1;
            end
        end
    
        else if (AR_HAND_SHAKE &&
                 ((OWNER == 2'b01) || (OWNER == 2'b10))) begin
            n_NEXT_OWNER = ARID[2:1]+1;
        end
    
        else if (R_HAND_SHAKE && RLAST && (OWNER != 2'b00)) begin
            n_OWNER      = NEXT_OWNER;
            n_NEXT_OWNER = 2'b00;
        end
    end
	
	always @(*) begin
		RID           = 0;
    	RDATA         = 0;
    	RRESP         = 0;
    	RLAST         = 0;
    	R_SLAVE_VALID = 0;

    	ROM_RREADY    = 0;
    	RAM_RREADY    = 0;

    	if (ROM_RVALID && OWNER == 2'b01) begin
    	    RID           = ROM_RID;
    	    RDATA         = ROM_RDATA;
    	    RRESP         = ROM_RRESP;
    	    RLAST         = ROM_RLAST;
    	    R_SLAVE_VALID = 1'b1;

    	    ROM_RREADY    = R_MASTER_READY;
    	end
    	else if (RAM_RVALID && OWNER == 2'b10) begin
    	    RID           = RAM_RID;
    	    RDATA         = RAM_RDATA;
    	    RRESP         = RAM_RRESP;
    	    RLAST         = RAM_RLAST;
    	    R_SLAVE_VALID = 1'b1;

    	    RAM_RREADY    = R_MASTER_READY;
    	end
	end

	always @(*) begin
		DMA_RID			= 0;
    	DMA_RDATA		= 0;
    	DMA_RRESP		= 0;
    	DMA_RLAST		= 0;
    	DMA_RVALID		= 0;
    	R_MASTER_READY	= 0;
		CPU_RID			= 0;
    	CPU_RDATA		= 0;
    	CPU_RRESP		= 0;
    	CPU_RLAST		= 0;
    	CPU_RVALID		= 0;
    	R_MASTER_READY	= 0;

		if(RID[3])begin
			DMA_RID		= RID          ;
    		DMA_RDATA	= RDATA        ;
    		DMA_RRESP	= RRESP        ;
    		DMA_RLAST	= RLAST        ;
    		DMA_RVALID	= R_SLAVE_VALID;
    		R_MASTER_READY = DMA_RREADY	;
		end
		else begin
			CPU_RID		= RID          ;
    		CPU_RDATA	= RDATA        ;
    		CPU_RRESP	= RRESP        ;
    		CPU_RLAST	= RLAST        ;
    		CPU_RVALID	= R_SLAVE_VALID;
    		R_MASTER_READY = CPU_RREADY	;
		end
	end
    endmodule
