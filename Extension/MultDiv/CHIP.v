// Top module of your design, you cannot modify this module!!
module CHIP (	clk,
				rst_n,
//----------for slow_memD------------
				mem_read_D,
				mem_write_D,
				mem_addr_D,
				mem_wdata_D,
				mem_rdata_D,
				mem_ready_D,
//----------for slow_memI------------
				mem_read_I,
				mem_write_I,
				mem_addr_I,
				mem_wdata_I,
				mem_rdata_I,
				mem_ready_I,
//----------for TestBed--------------				
				DCACHE_addr, 
				DCACHE_wdata,
				DCACHE_wen   
			);
input			clk, rst_n;
//--------------------------

output			mem_read_D;
output			mem_write_D;
output	[31:4]	mem_addr_D;
output	[127:0]	mem_wdata_D;
input	[127:0]	mem_rdata_D;
input			mem_ready_D;
//--------------------------
output			mem_read_I;
output			mem_write_I;
output	[31:4]	mem_addr_I;
output	[127:0]	mem_wdata_I;
input	[127:0]	mem_rdata_I;
input			mem_ready_I;
//----------for TestBed--------------
output	[29:0]	DCACHE_addr;
output	[31:0]	DCACHE_wdata;
output			DCACHE_wen;
//--------------------------

// wire declaration
wire        ICACHE_ren;
wire        ICACHE_wen;
wire [29:0] ICACHE_addr;
wire [31:0] ICACHE_wdata;
wire        ICACHE_stall;
wire [31:0] ICACHE_rdata;

wire        DCACHE_ren;
wire        DCACHE_wen;
wire [29:0] DCACHE_addr;
wire [31:0] DCACHE_wdata;
wire        DCACHE_stall;
wire [31:0] DCACHE_rdata;

//=========================================
	// Note that the overall design of your MIPS includes:
	// 1. pipelined MIPS processor
	// 2. data cache
	// 3. instruction cache


	MIPS_Pipeline i_MIPS(
		// control interface
		.clk            (clk)           , 
		.rst_n          (rst_n)         ,
//----------I cache interface-------		
		.ICACHE_ren     (ICACHE_ren)    ,
		.ICACHE_wen     (ICACHE_wen)    ,
		.ICACHE_addr    (ICACHE_addr)   ,
		.ICACHE_wdata   (ICACHE_wdata)  ,
		.ICACHE_stall   (ICACHE_stall)  ,
		.ICACHE_rdata   (ICACHE_rdata)  ,
//----------D cache interface-------
		.DCACHE_ren     (DCACHE_ren)    ,
		.DCACHE_wen     (DCACHE_wen)    ,
		.DCACHE_addr    (DCACHE_addr)   ,
		.DCACHE_wdata   (DCACHE_wdata)  ,
		.DCACHE_stall   (DCACHE_stall)  ,
		.DCACHE_rdata   (DCACHE_rdata)
	);
	
	cache D_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (DCACHE_ren)  ,
        .proc_write (DCACHE_wen)  ,
        .proc_addr  (DCACHE_addr) ,
        .proc_rdata (DCACHE_rdata),
        .proc_wdata (DCACHE_wdata),
        .proc_stall (DCACHE_stall),
        .mem_read   (mem_read_D)  ,
        .mem_write  (mem_write_D) ,
        .mem_addr   (mem_addr_D)  ,
        .mem_wdata  (mem_wdata_D) ,
        .mem_rdata  (mem_rdata_D) ,
        .mem_ready  (mem_ready_D)
	);

	cache I_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (ICACHE_ren)  ,
        .proc_write (ICACHE_wen)  ,
        .proc_addr  (ICACHE_addr) ,
        .proc_rdata (ICACHE_rdata),
        .proc_wdata (ICACHE_wdata),
        .proc_stall (ICACHE_stall),
        .mem_read   (mem_read_I)  ,
        .mem_write  (mem_write_I) ,
        .mem_addr   (mem_addr_I)  ,
        .mem_wdata  (mem_wdata_I) ,
        .mem_rdata  (mem_rdata_I) ,
        .mem_ready  (mem_ready_I)
	);
endmodule

module alu(
    ctrl,
    shamt,
    x,
    y,
    out,
    carry 
);
    
    input  [3:0] ctrl;
    input  [4:0] shamt;
    input  [31:0] x;
    input  [31:0] y;
    output [31:0] out;
    output carry;
   
    reg [31:0] out;
    reg carry;
    wire [32:0] plus_out;
    wire [32:0] minus_out;

    assign  plus_out = {1'b0, x} + {1'b0, y};
    assign  minus_out = {1'b0, x} - {1'b0, y};

    always@(*) begin
        carry = 0;
        out = 0;
        case (ctrl)
            4'b0000: out = x & y;
            4'b0001: out = x | y;
            4'b0010: begin
                carry = plus_out[32];
                out = plus_out[31:0];
            end
            4'b0110: begin
                carry = minus_out[32];
                out = minus_out[31:0];
            end
            4'b1001: out = x ^ y;
            4'b1100: out = ~ (x | y);
            4'b0100: out = y << shamt;
            4'b0111: out = y >> shamt;
            4'b1000: out = $signed(y) >>> shamt;
            4'b1111: out = ($signed(x) < $signed(y)) ? 1 : 0;
        endcase
    end

endmodule

module register_file(
    Clk  ,
    rst  ,
    WEN  ,
    RW   ,
    busW ,
    RX   ,
    RY   ,
    busX ,
    busY
);

    input        Clk, rst, WEN;
    input  [4:0] RW, RX, RY;
    input  [31:0] busW;
    output [31:0] busX, busY;
    
    reg [31:0] r [0:31];
    reg [31:0] r_n [0:31];

    assign busX = r[RX];
    assign busY = r[RY];

    integer i;
    
    always@(*) begin
        r_n[0] = 0;
        for (i = 1; i < 32; i = i + 1)
            r_n[i] = r[i];
        if (WEN) r_n[RW] = busW;
    end

    always@(posedge Clk) begin
        if (!rst) begin
            for (i = 0; i < 32; i = i + 1)
                r[i] <= 0;
        end
        else begin
            for (i = 0; i < 32; i = i + 1)
                r[i] <= r_n[i];
        end
    end	

endmodule

module control(
    op,
    Funct_field,
    RegDst,
    Jump,
    Jr,
    Branch,
    bne,
    MemtoReg,
    ALUOp,
    REN,
    WEN,
    ALUSrc,
    RegWrite,
    ui
);

    input  [5:0] op;
    input  [5:0] Funct_field;
    output       Jump, Jr, Branch, bne, REN, WEN, ALUSrc, RegWrite, ui;
    output [1:0] RegDst, MemtoReg;
    output [2:0] ALUOp;

    reg       Jump, Jr, Branch, bne, REN, WEN, ALUSrc, RegWrite, ui;
    reg [1:0] RegDst, MemtoReg;
    reg [2:0] ALUOp;

    always@(*) begin
        Jump = 0;
        Jr = 0;
        Branch = 0;
        bne = 0;
        REN = 0;
        WEN = 0;
        ALUSrc = 0;
        RegWrite = 0;
        ui = 0;
        RegDst = 0;
        MemtoReg = 0;
        ALUOp = 3'b000;
        case (op) //nop(100...000) is default
            6'b000000: begin //R-type
                RegDst = 1;
                RegWrite = (Funct_field[5:0] != 6'b001000) 
                    | (Funct_field[5:0] != 6'b011000) 
                    | (Funct_field[5:0] != 6'b011010); //write excluding Jr, multdiv
                ALUOp = 3'b010;
                Jr = (Funct_field[5:1] == 5'b00100); //Jr,Jalr
                MemtoReg = (Funct_field[5:0] == 6'b001001) ? 2 : 0;
            end
            6'b001000: begin //addi
                ALUOp = 3'b000;
                ALUSrc = 1;
                RegWrite = 1;
            end
            6'b001100: begin //andi
                ALUOp = 3'b100;
                ALUSrc = 1;
                RegWrite = 1;
                ui = 1; //unsigned extension
            end
            6'b001101: begin //ori
                ALUOp = 3'b101;
                ALUSrc = 1;
                RegWrite = 1;
                ui = 1; //unsigned extension
            end
            6'b001110: begin //xori
                ALUOp = 3'b110;
                ALUSrc = 1;
                RegWrite = 1;
                ui = 1; //unsigned extension
            end
            6'b001010: begin //slti
                ALUOp = 3'b111;
                ALUSrc = 1;
                RegWrite = 1;
            end
            6'b100011: begin //lw
                REN = 1;
                MemtoReg = 1;
                ALUSrc = 1;
                RegWrite = 1;
            end
            6'b101011: begin //sw
                WEN = 1;
                ALUSrc = 1;
            end
            6'b000100: begin //beq
                ALUOp = 3'b001;
                Branch = 1;
            end
            6'b000101: begin//bne
                ALUOp = 3'b001;
                bne = 1;
            end
            6'b000010: //j
                Jump = 1;
            6'b000011: begin //jal
                RegDst = 2;
                Jump = 1;
                MemtoReg = 2;
                RegWrite = 1;
            end
        endcase
    end

endmodule

module alu_control(
    ALUOp,
    Funct_field,
    ALUOperation,
);

    input  [2:0] ALUOp;
    input  [5:0] Funct_field;
    output [3:0] ALUOperation;

    reg [3:0] ALUOperation;
    always@(*) begin
        case (ALUOp)
            3'b000: ALUOperation = 4'b0010; //lw,sw
            3'b001: ALUOperation = 4'b0110; //beq,bne
            3'b010: begin//R-type
                case (Funct_field[5:0])
                    6'b100000: ALUOperation = 4'b0010; //add
                    6'b100010: ALUOperation = 4'b0110; //subtract
                    6'b100100: ALUOperation = 4'b0000; //and
                    6'b100101: ALUOperation = 4'b0001; //or
                    6'b100110: ALUOperation = 4'b1001; //xor
                    6'b100111: ALUOperation = 4'b1100; //nor
                    6'b000000: ALUOperation = 4'b0100; //shift left logical
                    6'b000011: ALUOperation = 4'b1000; //shift right arithmetic
                    6'b000010: ALUOperation = 4'b0111; //shift right logical
                    6'b101010: ALUOperation = 4'b1111; //set on less than
                    6'b011000: ALUOperation = 4'b0011; //mult
                    6'b011010: ALUOperation = 4'b0101; //div
                    6'b010000: ALUOperation = 4'b1010; //mfhi
                    6'b010010: ALUOperation = 4'b1011; //mflo
                    default: ALUOperation = 0;
                endcase
            end
            3'b100: ALUOperation = 4'b0000; //andi
            3'b101: ALUOperation = 4'b0001; //ori
            3'b110: ALUOperation = 4'b1001; //xori
            3'b111: ALUOperation = 4'b1111; //slti
            default: ALUOperation = 0;
        endcase
    end

endmodule

module multdiv_control(clk, rst, ALUOperation, LO0, sign, stall, ALUctrl, shift_r, shift_l, write, init);
    
    input clk, rst;
    input [3:0] ALUOperation;
    input LO0, sign;
    output stall, shift_r, shift_l, write, init;
    output [3:0] ALUctrl;

    localparam IDLE = 2'b00;
    localparam MULT = 2'b10;
    localparam DIV = 2'b11;

    reg [1:0] state, next_state;
    reg [4:0] count, next_count;
    reg stall, shift_r, shift_l, write, init;
    reg [3:0] ALUctrl;

    always@(*) begin
        next_state = state;
        next_count = count;
        case(state)
            IDLE: begin
                if (ALUOperation == 4'b0011) begin
                    next_state = MULT;
                    next_count = 0;
                    stall = 1;
                    write = 0;
                    init = 1;
                    shift_r = 0;
                    shift_l = 0;
                    ALUctrl = 4'b0011; //nop
                end
                else if (ALUOperation == 4'b0101) begin 
                    next_state = DIV;
                    next_count = 0;
                    stall = 1;
                    write = 0;
                    init = 1;
                    shift_r = 0;
                    shift_l = 0;
                    ALUctrl = 4'b0011; //nop
                end
                else begin
                    next_state = IDLE;
                    next_count = 0;
                    stall = 0;
                    write = 0;
                    init = 0;
                    shift_r = 0;
                    shift_l = 0;
                    ALUctrl = 4'b0011; //nop
                end
            end
            MULT: begin
                if (count == 31) begin
                    next_state = IDLE;
                    next_count = 0;
                    stall = 0;
                    if (LO0) write = 1;
                    else write = 0;
                    init = 0;
                    shift_r = 1;
                    shift_l = 0;
                    ALUctrl = 4'b0010; //add
                end
                else begin
                    next_state = MULT;
                    next_count = count + 1;
                    stall = 1;
                    if (LO0) write = 1;
                    else write = 0;
                    init = 0;
                    shift_r = 1;
                    shift_l = 0;
                    ALUctrl = 4'b0010; //add
                end
            end
            DIV: begin
                if (count == 31) begin
                    next_state = IDLE;
                    next_count = 0;
                    stall = 0;
                    if (sign) write = 0;
                    else write = 1;
                    init = 0;
                    shift_r = 0;
                    shift_l = 1;
                    ALUctrl = 4'b0110; //sub
                end
                else begin
                    next_state = DIV;
                    next_count = count + 1;
                    stall = 1;
                    if (sign) write = 0;
                    else write = 1;
                    init = 0;
                    shift_r = 0;
                    shift_l = 1;
                    ALUctrl = 4'b0110; //sub
                end
            end
        endcase
    end

    always@(posedge clk) begin
        if( !rst ) begin
            state <= IDLE;
            count <= 0;
        end
        else begin
            state <= next_state;
            count <= next_count;
        end
    end

endmodule

module multdiv_reg(clk, rst, shift_r, shift_l, write, init, carry, rs_data, ALUout, HI, LO);

    input clk, rst, shift_r, shift_l, write, init, carry;
    input [31:0] rs_data, ALUout;
    output [31:0] HI, LO;

    reg [31:0] HI, LO, next_HI, next_LO;
    wire set_lsb;

    assign set_lsb = ~carry;

    always@(*) begin
        if (init) begin
            next_HI = 0;
            next_LO = rs_data;
        end
        else if (shift_r & write) begin
            next_HI = {carry, ALUout[31:1]};
            next_LO = {ALUout[0], LO[31:1]};
        end
        else if (shift_r & ~write) begin
            next_HI = {1'b0, HI[31:1]};
            next_LO = {HI[0], LO[31:1]};
        end
        else if (shift_l & write) begin
            next_HI = ALUout;
            next_LO = {LO[30:0], set_lsb};
        end
        else if (shift_l & ~write) begin
            next_HI = {HI[30:0], LO[31]};
            next_LO = {LO[30:0], 1'b0};
        end
        else begin
            next_HI = HI;
            next_LO = LO;
        end
    end

    always@(posedge clk) begin
        if( !rst ) begin
            HI <= 0;
            LO <= 0;
        end
        else begin
            HI <= next_HI;
            LO <= next_LO;
        end
    end

endmodule

module PC_adder(pc, PC_plus4);

    input  [31:0] pc;
    output [31:0] PC_plus4;

    assign PC_plus4 = pc + 4;

endmodule

module shift_and_add(
    imm, 
    PC_plus4,
    out
);

    input  [31:0] imm, PC_plus4;
    output [31:0] out;

    wire [31:0] address;

    assign address = imm << 2;
    assign out = address + PC_plus4;

endmodule

module jump_calc(j_imm, PC_plus4, jump_address);

    input  [25:0] j_imm;
    input  [31:0] PC_plus4;
    output [31:0] jump_address;

    wire [27:0] address;

    assign address = j_imm << 2;
    assign jump_address = {PC_plus4[31:28], address};

endmodule

module PC_mux(
    jump_address,
    PC_plus4, //pc+4 at that time
    adder_out,
    busX, //old busX
    Branch,
    bne,
    Zero,
    Jump,
    Jr,
    IF_Flush,
    newPC
);

    input  [31:0] jump_address;
    input  [31:0] PC_plus4, adder_out, busX;
    input Branch, bne, Zero, Jump, Jr;
    output IF_Flush;
    output [31:0] newPC;

    wire [27:0] address;
    wire [31:0] branch_out;
    wire [31:0] mux2_out;
    wire ctrl;

    //mux1
    assign ctrl = (Branch & Zero) | (bne & ~Zero);
    assign branch_out = ctrl ? adder_out : PC_plus4;
    //mux2
    assign mux2_out = Jump ? jump_address : branch_out;
    //mux3
    assign newPC = Jr ? busX : mux2_out;

    assign IF_Flush = ctrl | Jump | Jr;

endmodule

module alu_mux(x, y, ctrl, out);

    input  [31:0] x, y;
    input  ctrl;
    output [31:0] out;

    assign out = (ctrl) ? x : y;

endmodule

module WR_mux(x, y, ctrl, out);

    input  [4:0] x, y;
    input  [1:0] ctrl;
    output [4:0] out;

    assign out = (ctrl[1]) ? 31 : (ctrl[0] ? y : x);
endmodule

module busW_mux(x, y, PC_plus4, ctrl, out);

    input  [31:0] x, y, PC_plus4; //old_PCplus4 (for Jal)
    input  [1:0]  ctrl;
    output [31:0] out;

    assign out = (ctrl[1]) ? PC_plus4 : (ctrl[0] ? y : x);
endmodule

module sign_extend(x, ui, out);

    input  [15:0] x;
    input  ui;
    output [31:0] out;

    assign out = ui ? {{16{1'b0}}, x} : {{16{x[15]}}, x};

endmodule

module forwarding_unit(IDEXRs, IDEXRt, EXMEMRd, MEMWBRd, EXMEMRegWrite, MEMWBRegWrite, A_ctrl, B_ctrl);

    input  [4:0] IDEXRs, IDEXRt, EXMEMRd, MEMWBRd;
    input  EXMEMRegWrite, MEMWBRegWrite;
    output [1:0] A_ctrl, B_ctrl;

    reg [1:0] A_ctrl, B_ctrl;

    always@(*) begin
        A_ctrl = 0;
        B_ctrl = 0;
        if (EXMEMRegWrite & (EXMEMRd != 0) & (EXMEMRd == IDEXRs)) A_ctrl = 2'b10;
        if (EXMEMRegWrite & (EXMEMRd != 0) & (EXMEMRd == IDEXRt)) B_ctrl = 2'b10;
        if (MEMWBRegWrite & (MEMWBRd != 0) & (MEMWBRd == IDEXRs)) A_ctrl = 2'b01;
        if (MEMWBRegWrite & (MEMWBRd != 0) & (MEMWBRd == IDEXRt)) B_ctrl = 2'b01;
    end

endmodule

module forwarding_mux(ctrl, dataIDEX, dataEXMEM, dataMEMWB, out);

    input  [1:0] ctrl;
    input  [31:0] dataIDEX, dataEXMEM, dataMEMWB;
    output [31:0] out;

    assign out = (ctrl == 2'b01) ? dataMEMWB : 
            ((ctrl == 2'b10) ? dataEXMEM : dataIDEX);

endmodule

module hazard_detection_unit(IDEXRt, IFIDRs, IFIDRt, IDEXMemRead, stall_ctrl);

    input  [4:0] IDEXRt, IFIDRs, IFIDRt;
    input  IDEXMemRead;
    output stall_ctrl;

    reg stall_ctrl;

    always@(*) begin
        stall_ctrl = 0;
        if (IDEXMemRead & ((IDEXRt == IFIDRs) | (IDEXRt == IFIDRt))) stall_ctrl = 1'b1;
    end

endmodule

module stall_mux(stall,
    RegDst_in, MemtoReg_in, ALUOp_in, REN_in, WEN_in, ALUSrc_in, RegWrite_in,
    RegDst, MemtoReg, ALUOp, REN, WEN, ALUSrc, RegWrite,
);

    input stall;
    input       REN_in, WEN_in, ALUSrc_in, RegWrite_in;
    input [1:0] RegDst_in, MemtoReg_in;
    input [2:0] ALUOp_in;
    output       REN, WEN, ALUSrc, RegWrite;
    output [1:0] RegDst, MemtoReg;
    output [2:0] ALUOp;

    assign RegDst = stall ? 0 : RegDst_in;
    assign MemtoReg = stall ? 0 : MemtoReg_in;
    assign ALUOp = stall ? 0 : ALUOp_in;
    assign REN = stall ? 0 : REN_in;
    assign WEN = stall ? 0 : WEN_in;
    assign ALUSrc = stall ? 0 : ALUSrc_in;
    assign RegWrite = stall ? 0 : RegWrite_in;

endmodule

module branch_forwarding(IFIDRs, IFIDRt, IDEXRd, EXMEMRd, MEMWBRd, 
        IDEXRegWrite, EXMEMRegWrite, MEMWBRegWrite, x_ctrl, y_ctrl, stall);

    input  [4:0] IFIDRs, IFIDRt, IDEXRd, EXMEMRd, MEMWBRd;
    input  IDEXRegWrite, EXMEMRegWrite, MEMWBRegWrite;
    output [1:0] x_ctrl, y_ctrl;
    output stall;

    reg [1:0] x_ctrl, y_ctrl;
    reg stall;

    always@(*) begin
        x_ctrl = 0;
        y_ctrl = 0;
        stall = 0;
        if (IDEXRegWrite & (IDEXRd != 0) & (IDEXRd == IFIDRs)) stall = 1'b1;
        if (IDEXRegWrite & (IDEXRd != 0) & (IDEXRd == IFIDRt)) stall = 1'b1;
        if (EXMEMRegWrite & (EXMEMRd != 0) & (EXMEMRd == IFIDRs)) x_ctrl = 2'b01;
        if (EXMEMRegWrite & (EXMEMRd != 0) & (EXMEMRd == IFIDRt)) y_ctrl = 2'b01;
        if (MEMWBRegWrite & (MEMWBRd != 0) & (MEMWBRd == IFIDRs)) x_ctrl = 2'b11;
        if (MEMWBRegWrite & (MEMWBRd != 0) & (MEMWBRd == IFIDRt)) y_ctrl = 2'b11;
    end

endmodule

module branch_forwarding_mux(ctrl, orig, dataEXMEM, dataMEMWB, out);

    input  [1:0] ctrl;
    input  [31:0] orig, dataEXMEM, dataMEMWB;
    output [31:0] out;

    assign out = (ctrl == 2'b11) ? dataMEMWB : 
        ((ctrl == 2'b01) ? dataEXMEM : orig);

endmodule

module zero_unit(dataX, dataY, zero);

    input [31:0] dataX, dataY;
    output zero;

    assign zero = (dataX == dataY);

endmodule

module MIPS_Pipeline (
    // control interface
	clk            , 
	rst_n          ,
//----------I cache interface-------		
	ICACHE_ren     ,
	ICACHE_wen     ,
	ICACHE_addr    ,
	ICACHE_wdata   ,
	ICACHE_stall   ,
	ICACHE_rdata   ,
//----------D cache interface-------
	DCACHE_ren     ,
	DCACHE_wen     ,
	DCACHE_addr    ,
	DCACHE_wdata   ,
	DCACHE_stall   ,
	DCACHE_rdata   
);

    input  clk, rst_n, ICACHE_stall, DCACHE_stall;
    input  [31:0] ICACHE_rdata, DCACHE_rdata;
    output ICACHE_ren, ICACHE_wen, DCACHE_ren, DCACHE_wen;
    output [31:0] ICACHE_wdata, DCACHE_wdata;
    output [29:0] ICACHE_addr, DCACHE_addr;

    reg [31:0] PC;
    //IFID
    reg [31:0] instr_IFID_out;
    reg [31:0] PC_plus4_IFID_out;
    //IDEX
    reg [5:0] EXtype_IDEX_out;
    reg [5:0] MEMtype_IDEX_out;
    reg [2:0] WBtype_IDEX_out;
    reg [31:0] PC_plus4_IDEX_out;
    reg [31:0] dataX_IDEX_out;
    reg [31:0] dataY_IDEX_out;
    reg [31:0] extended_IDEX_out;
    reg [4:0] instrRs_IDEX_out;
    reg [4:0] instrRt_IDEX_out;
    //EXMEM
    reg [5:0] MEMtype_EXMEM_out;
    reg [2:0] WBtype_EXMEM_out;
    reg [31:0] PC_plus4_EXMEM_out;
    reg [31:0] ALUresult_EXMEM_out;
    reg [31:0] dataY_EXMEM_out;
    reg [4:0] RFwreg_EXMEM_out;
    //MEMWB
    reg [2:0] WBtype_MEMWB_out;
    reg [31:0] PC_plus4_MEMWB_out;
    reg [31:0] memdata_MEMWB_out;
    reg [31:0] ALUresult_MEMWB_out;
    reg [4:0] RFwreg_MEMWB_out;

    wire [31:0] PC_plus4_IF, PC_plus4_ID, PC_plus4_EX, PC_plus4_MEM, PC_plus4_WB;
    wire [31:0] instr_IF, instr_ID;
    wire [5:0] EXtype_ID, EXtype_EX;//EXtype
    wire [1:0] RegDst_c, RegDst_ID, RegDst_EX;
    wire [2:0] ALUOp_c, ALUOp_ID, ALUOp_EX;
    wire ALUSrc_c, ALUSrc_ID, ALUSrc_EX;
    wire [1:0] MEMtype_ID, MEMtype_EX, MEMtype_MEM;//MEMtype
    wire REN_c, REN_ID, REN_EX, REN_MEM;
    wire WEN_c, WEN_ID, WEN_EX, WEN_MEM;
    wire [2:0] WBtype_ID, WBtype_EX, WBtype_MEM, WBtype_WB;//WBtype
    wire RegWrite_c, RegWrite_ID, RegWrite_EX, RegWrite_MEM, RegWrite_WB;
    wire [1:0] MemtoReg_c, MemtoReg_ID, MemtoReg_EX, MemtoReg_MEM, MemtoReg_WB;
    wire Jump, Jr, Branch, bne;
    wire [31:0] dataX_ID, dataX_EX;
    wire [31:0] dataY_ID, dataY_EX, dataY_MEM;
    wire ui;
    wire [31:0] jaddr;
    wire [31:0] extended_ID, extended_EX;
    wire [31:0] adderout;
    wire [31:0] aluY;
    wire [3:0] alu_ctrl;
    wire zero;
    wire [31:0] ALUresult_EX, ALUresult_MEM, ALUresult_WB;
    wire [4:0] instrRs, instrRt, instrRd;
    wire [4:0] RFwreg_EX, RFwreg_MEM, RFwreg_WB;
    wire [31:0] newPC;
    wire [31:0] memdata_MEM, memdata_WB;
    wire [31:0] RFwdata_WB;
    wire [1:0] A_ctrl, B_ctrl;
    wire [31:0] A_out, B_out;
    wire stall_ctrl;
    wire [1:0] x_ctrl, y_ctrl;
    wire [31:0] x_out, y_out;
    wire IF_Flush;
    wire branch_stall;
    //multdiv
    wire multdiv_stall;
    wire carry;
    wire [31:0] HI_out, LO_out;
    wire shift_r, shift_l, write, init;
    wire [3:0] multdiv_ctrl;

    //IF
    assign ICACHE_ren = 1;
    assign ICACHE_wen = 0;
    assign ICACHE_addr = PC[31:2];
    assign ICACHE_wdata = 0;
    assign instr_IF = ICACHE_rdata;
    PC_adder pc_adder0(.pc({ICACHE_addr, 2'b00}), .PC_plus4(PC_plus4_IF));
    //ID
    assign EXtype_ID = {RegDst_ID, ALUOp_ID, ALUSrc_ID}; //bits:2,3,1
    assign MEMtype_ID = {REN_ID, WEN_ID}; //bits:1,1
    assign WBtype_ID = {RegWrite_ID, MemtoReg_ID}; //bits:1,2
    assign instr_ID = instr_IFID_out;
    assign PC_plus4_ID = PC_plus4_IFID_out;
    control control0(.op(instr_ID[31:26]), .Funct_field(instr_ID[5:0]), .RegDst(RegDst_c), 
            .Jump(Jump), .Jr(Jr), .Branch(Branch), .bne(bne),
            .MemtoReg(MemtoReg_c), .ALUOp(ALUOp_c), .REN(REN_c), .WEN(WEN_c),
            .ALUSrc(ALUSrc_c), .RegWrite(RegWrite_c), .ui(ui));
    register_file reg_file0(.Clk(clk), .rst(rst_n), .WEN(RegWrite_WB), .RW(RFwreg_WB), 
            .busW(RFwdata_WB), .RX(instr_ID[25:21]), .RY(instr_ID[20:16]), 
            .busX(dataX_ID), .busY(dataY_ID));
    sign_extend sign0(.x(instr_ID[15:0]), .ui(ui), .out(extended_ID));
    jump_calc jump_calc0(.j_imm(instr_ID[25:0]),
            .PC_plus4(PC_plus4_ID), .jump_address(jaddr));
    shift_and_add adder0(.imm(extended_ID), .PC_plus4(PC_plus4_ID), .out(adderout));
    PC_mux pm0(.jump_address(jaddr), .PC_plus4(PC_plus4_IF), .adder_out(adderout),
            .busX(x_out), .Branch(Branch), .bne(bne), .Zero(zero), 
            .Jump(Jump), .Jr(Jr), .IF_Flush(IF_Flush), .newPC(newPC));
    zero_unit z0(.dataX(x_out), .dataY(y_out), .zero(zero));
    //EX
    assign EXtype_EX = {RegDst_EX, ALUOp_EX, ALUSrc_EX}; //bits:2,3,1
    assign RegDst_EX = EXtype_IDEX_out[5:4];
    assign ALUOp_EX = EXtype_IDEX_out[3:1];
    assign ALUSrc_EX = EXtype_IDEX_out[0];
    assign MEMtype_EX = {REN_EX, WEN_EX}; //bits:1,1
    assign REN_EX = MEMtype_IDEX_out[1];
    assign WEN_EX = MEMtype_IDEX_out[0];
    assign WBtype_EX = {RegWrite_EX, MemtoReg_EX}; //bits:1,2
    assign RegWrite_EX = WBtype_IDEX_out[2];
    assign MemtoReg_EX = WBtype_IDEX_out[1:0];
    assign PC_plus4_EX = PC_plus4_IDEX_out;
    assign dataX_EX = dataX_IDEX_out;
    assign dataY_EX = dataY_IDEX_out;
    assign extended_EX = extended_IDEX_out; //extended instr[15:0]
    assign instrRs = instrRs_IDEX_out; //instr[25:21]
    assign instrRt = instrRt_IDEX_out; //instr[20:16]
    assign instrRd = extended_IDEX_out[15:11]; //instr[15:11]
    alu_mux am0(.x(extended_EX), .y(B_out), .ctrl(ALUSrc_EX), .out(aluY));
    alu alu0(.ctrl( (multdiv_stall) ? multdiv_ctrl : alu_ctrl), .shamt(extended_EX[10:6]), 
            .x( (multdiv_stall) ? HI_out : A_out), .y(aluY), .out(ALUresult_EX), .carry(carry));
    alu_control alu_control0(.ALUOp(ALUOp_EX), .Funct_field(extended_EX[5:0]), .ALUOperation(alu_ctrl));
    WR_mux wm0(.x(instrRt), .y(instrRd), .ctrl(RegDst_EX), .out(RFwreg_EX));
    multdiv_control mc0(.clk(clk), .rst(rst_n), .ALUOperation(alu_ctrl), .LO0(LO_out[0]), .sign(carry), .stall(multdiv_stall), 
            .ALUctrl(multdiv_ctrl), .shift_r(shift_r), .shift_l(shift_l), .write(write), .init(init));
    multdiv_reg mr0(.clk(clk), .rst(rst_n), .shift_r(shift_r), .shift_l(shift_l), .write(write), .init(init), 
            .carry(carry), .rs_data(A_out), .ALUout(ALUresult_EX), .HI(HI_out), .LO(LO_out));
    //MEM
    assign MEMtype_MEM = {REN_MEM, WEN_MEM}; //bits:1,1
    assign REN_MEM = MEMtype_EXMEM_out[1];
    assign WEN_MEM = MEMtype_EXMEM_out[0];
    assign WBtype_MEM = {RegWrite_MEM, MemtoReg_MEM}; //bits:1,2
    assign RegWrite_MEM = WBtype_EXMEM_out[2];
    assign MemtoReg_MEM = WBtype_EXMEM_out[1:0];
    assign PC_plus4_MEM = PC_plus4_EXMEM_out;
    assign ALUresult_MEM = ALUresult_EXMEM_out;
    assign dataY_MEM = dataY_EXMEM_out;
    assign RFwreg_MEM = RFwreg_EXMEM_out;
    assign DCACHE_ren = REN_MEM;
    assign DCACHE_wen = WEN_MEM;
    assign DCACHE_addr = ALUresult_MEM[31:2];
    assign DCACHE_wdata = dataY_MEM;
    assign memdata_MEM = DCACHE_rdata;
    //WB
    assign RegWrite_WB = WBtype_MEMWB_out[2];
    assign MemtoReg_WB = WBtype_MEMWB_out[1:0];
    assign PC_plus4_WB = PC_plus4_MEMWB_out;
    assign memdata_WB = memdata_MEMWB_out;
    assign ALUresult_WB = ALUresult_MEMWB_out;
    assign RFwreg_WB = RFwreg_MEMWB_out;
    busW_mux bum0(.x(ALUresult_WB), .y(memdata_WB), .PC_plus4(PC_plus4_WB), 
            .ctrl(MemtoReg_WB), .out(RFwdata_WB));
    //forwarding
    forwarding_unit forw0(.IDEXRs(instrRs), .IDEXRt(instrRt), .EXMEMRd(RFwreg_MEM), .MEMWBRd(RFwreg_WB),
            .EXMEMRegWrite(RegWrite_MEM), .MEMWBRegWrite(RegWrite_WB), .A_ctrl(A_ctrl), .B_ctrl(B_ctrl));
    forwarding_mux muxA(.ctrl(A_ctrl), .dataIDEX(dataX_EX), .dataEXMEM(ALUresult_MEM), .dataMEMWB(RFwdata_WB), .out(A_out));
    forwarding_mux muxB(.ctrl(B_ctrl), .dataIDEX(dataY_EX), .dataEXMEM(ALUresult_MEM), .dataMEMWB(RFwdata_WB), .out(B_out));
    //hazard detection
    //hazard_detection_unit hd0(.IDEXRt(instrRt), .IFIDRs(instr_ID[25:21]), .IFIDRt(instr_ID[20:16]),
    //        .IDEXMemRead(REN_EX), .stall_ctrl(stall_ctrl));
    stall_mux s_mux(.stall(branch_stall), .RegDst_in(RegDst_c), .MemtoReg_in(MemtoReg_c), .ALUOp_in(ALUOp_c),
            .REN_in(REN_c), .WEN_in(WEN_c), .ALUSrc_in(ALUSrc_c), .RegWrite_in(RegWrite_c),
            .RegDst(RegDst_ID), .MemtoReg(MemtoReg_ID), .ALUOp(ALUOp_ID), .REN(REN_ID), .WEN(WEN_ID),
            .ALUSrc(ALUSrc_ID), .RegWrite(RegWrite_ID));
    //branch forward
    branch_forwarding bforw0(.IFIDRs(instr_ID[25:21]), .IFIDRt(instr_ID[20:16]), .IDEXRd(RFwreg_EX), 
            .EXMEMRd(RFwreg_MEM), .MEMWBRd(RFwreg_WB), .IDEXRegWrite(RegWrite_EX), .EXMEMRegWrite(RegWrite_MEM), 
            .MEMWBRegWrite(RegWrite_WB), .x_ctrl(x_ctrl), .y_ctrl(y_ctrl), .stall(branch_stall));
    branch_forwarding_mux muxx(.ctrl(x_ctrl), .orig(dataX_ID), 
            .dataEXMEM(ALUresult_MEM), .dataMEMWB(RFwdata_WB), .out(x_out));
    branch_forwarding_mux muxy(.ctrl(y_ctrl), .orig(dataY_ID), 
            .dataEXMEM(ALUresult_MEM), .dataMEMWB(RFwdata_WB), .out(y_out));


    always@(posedge clk) begin
        if (!rst_n) begin
            PC <= 0;
            //IFID
            instr_IFID_out <= 0;
            PC_plus4_IFID_out <= 0;
            //IDEX
            EXtype_IDEX_out <= 0;
            MEMtype_IDEX_out <= 0;
            WBtype_IDEX_out <= 0;
            PC_plus4_IDEX_out <= 0;
            dataX_IDEX_out <= 0;
            dataY_IDEX_out <= 0;
            extended_IDEX_out <= 0;
            instrRs_IDEX_out <= 0;
            instrRt_IDEX_out <= 0;
            //EXMEM
            MEMtype_EXMEM_out <= 0;
            WBtype_EXMEM_out <= 0;
            PC_plus4_EXMEM_out <= 0;
            ALUresult_EXMEM_out <= 0;
            dataY_EXMEM_out <= 0;
            RFwreg_EXMEM_out <= 0;
            //MEMWB
            WBtype_MEMWB_out <= 0;
            PC_plus4_MEMWB_out <= 0;
            memdata_MEMWB_out <= 0;
            ALUresult_MEMWB_out <= 0;
            RFwreg_MEMWB_out <= 0;
        end
        else begin
            if (ICACHE_stall || DCACHE_stall || multdiv_stall) begin
                PC <= PC;
                //IFID
                instr_IFID_out <= instr_IFID_out;
                PC_plus4_IFID_out <= PC_plus4_IFID_out;
                //IDEX
                EXtype_IDEX_out <= EXtype_IDEX_out;
                MEMtype_IDEX_out <= MEMtype_IDEX_out;
                WBtype_IDEX_out <= WBtype_IDEX_out;
                PC_plus4_IDEX_out <= PC_plus4_IDEX_out;
                dataX_IDEX_out <= dataX_IDEX_out;
                dataY_IDEX_out <= dataY_IDEX_out;
                extended_IDEX_out <= extended_IDEX_out;
                instrRs_IDEX_out <= instrRs_IDEX_out;
                instrRt_IDEX_out <= instrRt_IDEX_out;
                //EXMEM
                MEMtype_EXMEM_out <= MEMtype_EXMEM_out;
                WBtype_EXMEM_out <= WBtype_EXMEM_out;
                PC_plus4_EXMEM_out <= PC_plus4_EXMEM_out;
                ALUresult_EXMEM_out <= ALUresult_EXMEM_out;
                dataY_EXMEM_out <= dataY_EXMEM_out;
                RFwreg_EXMEM_out <= RFwreg_EXMEM_out;
                //MEMWB
                WBtype_MEMWB_out <= WBtype_MEMWB_out;
                PC_plus4_MEMWB_out <= PC_plus4_MEMWB_out;
                memdata_MEMWB_out <= memdata_MEMWB_out;
                ALUresult_MEMWB_out <= ALUresult_MEMWB_out;
                RFwreg_MEMWB_out <= RFwreg_MEMWB_out;
            end
            else begin
                if (branch_stall) begin
                    PC <= PC;
                    //IFID
                    instr_IFID_out <= instr_IFID_out;
                    PC_plus4_IFID_out <= PC_plus4_IFID_out;
                end
                else begin
                    PC <= newPC;
                    //IFID
                    instr_IFID_out <= (IF_Flush) ? {1'b1, {31{1'b0}}} : instr_IF; //nop:100...000
                    PC_plus4_IFID_out <= PC_plus4_IF;
                end
                //IDEX
                EXtype_IDEX_out <= EXtype_ID;
                MEMtype_IDEX_out <= MEMtype_ID;
                WBtype_IDEX_out <= WBtype_ID;
                PC_plus4_IDEX_out <= PC_plus4_ID;
                dataX_IDEX_out <= x_out;
                dataY_IDEX_out <= y_out;
                extended_IDEX_out <= extended_ID;
                instrRs_IDEX_out <= instr_ID[25:21];
                instrRt_IDEX_out <= instr_ID[20:16];
                //EXMEM
                MEMtype_EXMEM_out <= MEMtype_EX;
                WBtype_EXMEM_out <= WBtype_EX;
                PC_plus4_EXMEM_out <= PC_plus4_EX;
                ALUresult_EXMEM_out <= (alu_ctrl == 4'b1010) ? HI_out : ( (alu_ctrl == 4'b1011) ? LO_out : ALUresult_EX);
                dataY_EXMEM_out <= dataY_EX;
                RFwreg_EXMEM_out <= RFwreg_EX;
                //MEMWB
                WBtype_MEMWB_out <= WBtype_MEM;
                PC_plus4_MEMWB_out <= PC_plus4_MEM;
                memdata_MEMWB_out <= memdata_MEM;
                ALUresult_MEMWB_out <= ALUresult_MEM;
                RFwreg_MEMWB_out <= RFwreg_MEM;
            end
        end
    end

endmodule

module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;
    

//==== Parameter ==========================================
    localparam IN_CACHE = 2'b00;
    localparam UPDATE_MEM = 2'b01;
    localparam READ_MEM = 2'b10;
    localparam WRITE_CACHE = 2'b11;

//==== wire/reg definition ================================
    wire [24:0] data_tag;
    wire [2:0] index;
    wire [1:0] block_offset;
    wire hit;
    reg proc_stall;
    reg mem_read, nxt_mem_read;
    reg mem_write, nxt_mem_write;
    reg [27:0] mem_addr, nxt_mem_addr;
    reg [1:0] state, nxt_state;
    reg [31:0] data [0:7][0:3];
    reg [31:0] nxt_data [0:7][0:3];
    reg [25:0] tag [0:7];
    reg [25:0] nxt_tag [0:7];
    reg valid [0:7];
    reg nxt_valid [0:7];
    reg dirty [0:7];
    reg nxt_dirty [0:7];

    integer i, j;
    
//==== combinational circuit ==============================
    assign data_tag = proc_addr[29:5];
    assign index = proc_addr[4:2];
    assign block_offset = proc_addr[1:0];
    assign hit = valid[index] & (tag[index] == data_tag);
    assign proc_rdata = data[index][block_offset];
    assign mem_wdata = {data[index][3], data[index][2], data[index][1], data[index][0]};

    always@(*) begin
        for (i = 0; i < 8; i = i + 1) nxt_valid[i] = valid[i];
        for (i = 0; i < 8; i = i + 1) nxt_tag[i] = tag[i];
        for (i = 0; i < 8; i = i + 1) nxt_dirty[i] = dirty[i];
        for (i = 0; i < 8; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                nxt_data[i][j] = data[i][j];
            end
        end
        proc_stall = 0;
        nxt_state = IN_CACHE;
        nxt_mem_read = 0;
        nxt_mem_write = 0;
        nxt_mem_addr = 0;
        case(state)
            IN_CACHE: begin
                if (hit) begin
                    nxt_state = IN_CACHE;
                    proc_stall = 0;
                    nxt_mem_read = 0;
                    nxt_mem_write = 0;
                    nxt_mem_addr = proc_addr[29:2];
                    if (proc_write) begin
                        nxt_dirty[index] = 1;
                        nxt_data[index][block_offset] = proc_wdata;
                    end
                end
                else if (proc_read | proc_write) begin
                    proc_stall = 1;
                    if (dirty[index]) begin
                        nxt_state = UPDATE_MEM;
                        nxt_mem_read = 0;
                        nxt_mem_write = 1;
                        nxt_mem_addr = {tag[index], index};
                    end
                    else begin
                        nxt_state = READ_MEM;
                        nxt_mem_read = 1;
                        nxt_mem_write = 0;
                        nxt_mem_addr = proc_addr[29:2];
                    end
                end
            end
            UPDATE_MEM: begin
                proc_stall = 1;
                if (mem_ready) begin
                    nxt_state = READ_MEM;
                    nxt_dirty[index] = 0;
                    nxt_mem_read = 1;
                    nxt_mem_write = 0;
                    nxt_mem_addr = proc_addr[29:2];
                end
                else begin
                    nxt_state = UPDATE_MEM;
                    nxt_mem_read = 0;
                    nxt_mem_write = 1;
                    nxt_mem_addr = mem_addr;
                end
            end
            READ_MEM: begin
                nxt_mem_addr = proc_addr[29:2];
                if (mem_ready) begin
                    proc_stall = 1;
                    nxt_mem_read = 0;
                    nxt_mem_write = 0;
                    nxt_valid[index] = 1;
                    nxt_tag[index] = data_tag;
                    nxt_data[index][0] = mem_rdata[31:0];
                    nxt_data[index][1] = mem_rdata[63:32];
                    nxt_data[index][2] = mem_rdata[95:64];
                    nxt_data[index][3] = mem_rdata[127:96];
                    if (proc_write) begin
                        nxt_state = WRITE_CACHE;
                    end
                    else begin
                        nxt_state = IN_CACHE;
                    end
                end
                else begin
                    nxt_state = READ_MEM;
                    proc_stall = 1;
                    nxt_mem_read = 1;
                    nxt_mem_write = 0;
                end
            end
            WRITE_CACHE: begin
                nxt_state = IN_CACHE;
                proc_stall = 1;
                nxt_mem_read = 0;
                nxt_mem_write = 0;
                nxt_mem_addr = proc_addr[29:2];
                nxt_dirty[index] = 1;
                nxt_data[index][block_offset] = proc_wdata;
            end
        endcase
    end


//==== sequential circuit =================================
    always@( posedge clk ) begin
        if( proc_reset ) begin
            state <= IN_CACHE;
            mem_read <= 0;
            mem_write <= 0;
            mem_addr <= 0;
            for (i = 0; i < 8; i = i + 1) valid[i] <= 0;
            for (i = 0; i < 8; i = i + 1) tag[i] <= 0;
            for (i = 0; i < 8; i = i + 1) dirty[i] <= 0;
            for (i = 0; i < 8; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    data[i][j] <= 0;
                end
            end
        end
        else begin
            state <= nxt_state;   
            mem_read <= nxt_mem_read;
            mem_write <= nxt_mem_write;
            mem_addr <= nxt_mem_addr;
            for (i = 0; i < 8; i = i + 1) valid[i] <= nxt_valid[i];
            for (i = 0; i < 8; i = i + 1) tag[i] <= nxt_tag[i];
            for (i = 0; i < 8; i = i + 1) dirty[i] <= nxt_dirty[i];
            for (i = 0; i < 8; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    data[i][j] <= nxt_data[i][j];
                end
            end
        end
    end

endmodule

module cache_not_used(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;
    

//==== Parameter ==========================================
    localparam IN_CACHE = 2'b00;
    localparam UPDATE_MEM = 2'b01;
    localparam READ_MEM = 2'b10;
    localparam WRITE_CACHE = 2'b11;

//==== wire/reg definition ================================
    wire [25:0] data_tag;
    wire [1:0] index;
    wire [1:0] block_offset;
    wire hit0, hit1, hit;
    wire set_choose_temp;
    reg proc_stall;
    reg mem_read, nxt_mem_read;
    reg mem_write, nxt_mem_write;
    reg [27:0] mem_addr, nxt_mem_addr;
    reg [1:0] state, nxt_state;
    reg [31:0] data [0:1][0:3][0:3];
    reg [31:0] nxt_data [0:1][0:3][0:3];
    reg [25:0] tag [0:1][0:3];
    reg [25:0] nxt_tag [0:1][0:3];
    reg valid [0:1][0:3];
    reg nxt_valid [0:1][0:3];
    reg dirty [0:1][0:3];
    reg nxt_dirty [0:1][0:3];
    reg set_choose, nxt_set_choose;
    reg set_use [0:3];
    reg nxt_set_use [0:3];
    
    integer i, j;

//==== combinational circuit ==============================
    assign data_tag = proc_addr[29:4];
    assign index = proc_addr[3:2];
    assign block_offset = proc_addr[1:0];
    assign hit0 = valid[0][index] & (tag[0][index] == data_tag);
    assign hit1 = valid[1][index] & (tag[1][index] == data_tag);
    assign hit = hit0 ^ hit1;
    assign proc_rdata = (hit0) ? data[0][index][block_offset] : data[1][index][block_offset];
    assign mem_wdata = {data[set_choose][index][3], data[set_choose][index][2], data[set_choose][index][1], data[set_choose][index][0]};
    assign set_choose_temp = ~set_use[index];

    always@(*) begin
        for (i = 0; i < 4; i = i + 1) nxt_valid[0][i] = valid[0][i];
        for (i = 0; i < 4; i = i + 1) nxt_valid[1][i] = valid[1][i];
        for (i = 0; i < 4; i = i + 1) nxt_tag[0][i] = tag[0][i];
        for (i = 0; i < 4; i = i + 1) nxt_tag[1][i] = tag[1][i];
        for (i = 0; i < 4; i = i + 1) nxt_dirty[0][i] = dirty[0][i];
        for (i = 0; i < 4; i = i + 1) nxt_dirty[1][i] = dirty[1][i];
        for (i = 0; i < 4; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                nxt_data[0][i][j] = data[0][i][j];
            end
        end
        for (i = 0; i < 4; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                nxt_data[1][i][j] = data[1][i][j];
            end
        end
        nxt_set_choose = set_choose;
        for (i = 0; i < 4; i = i + 1) nxt_set_use[i] = set_use[i];
        proc_stall = 0;
        nxt_state = IN_CACHE;
        nxt_mem_read = 0;
        nxt_mem_write = 0;
        nxt_mem_addr = 0;
        case(state)
            IN_CACHE: begin
                if (hit) begin
                    nxt_state = IN_CACHE;
                    proc_stall = 0;
                    nxt_mem_read = 0;
                    nxt_mem_write = 0;
                    nxt_mem_addr = proc_addr[29:2];
                    if (proc_write) begin
                        if (hit0) nxt_dirty[0][index] = 1;
                        else nxt_dirty[1][index] = 1;
                        if (hit0) nxt_data[0][index][block_offset] = proc_wdata;
                        else nxt_data[1][index][block_offset] = proc_wdata;
                    end
                end
                else if (proc_read | proc_write) begin
                    proc_stall = 1;
                    nxt_set_choose = set_choose_temp;
                    if (dirty[set_choose_temp][index]) begin
                        nxt_state = UPDATE_MEM;
                        nxt_mem_read = 0;
                        nxt_mem_write = 1;
                        nxt_mem_addr = {tag[set_choose_temp][index], index};
                    end
                    else begin
                        nxt_state = READ_MEM;
                        nxt_mem_read = 1;
                        nxt_mem_write = 0;
                        nxt_mem_addr = proc_addr[29:2];
                    end
                end
            end
            UPDATE_MEM: begin
                proc_stall = 1;
                if (mem_ready) begin
                    nxt_state = READ_MEM;
                    nxt_dirty[set_choose][index] = 0;
                    nxt_mem_read = 1;
                    nxt_mem_write = 0;
                    nxt_mem_addr = proc_addr[29:2];
                end
                else begin
                    nxt_state = UPDATE_MEM;
                    nxt_mem_read = 0;
                    nxt_mem_write = 1;
                    nxt_mem_addr = mem_addr;
                end
            end
            READ_MEM: begin
                nxt_mem_addr = proc_addr[29:2];
                if (mem_ready) begin
                    proc_stall = 1;
                    nxt_mem_read = 0;
                    nxt_mem_write = 0;
                    nxt_valid[set_choose][index] = 1;
                    nxt_tag[set_choose][index] = data_tag;
                    nxt_data[set_choose][index][0] = mem_rdata[31:0];
                    nxt_data[set_choose][index][1] = mem_rdata[63:32];
                    nxt_data[set_choose][index][2] = mem_rdata[95:64];
                    nxt_data[set_choose][index][3] = mem_rdata[127:96];
                    nxt_set_use[index] = set_choose;
                    if (proc_write) begin
                        nxt_state = WRITE_CACHE;
                    end
                    else begin
                        nxt_state = IN_CACHE;
                    end
                end
                else begin
                    nxt_state = READ_MEM;
                    proc_stall = 1;
                    nxt_mem_read = 1;
                    nxt_mem_write = 0;
                end
            end
            WRITE_CACHE: begin
                nxt_state = IN_CACHE;
                proc_stall = 1;
                nxt_mem_read = 0;
                nxt_mem_write = 0;
                nxt_mem_addr = proc_addr[29:2];
                nxt_dirty[set_choose][index] = 1;
                nxt_data[set_choose][index][block_offset] = proc_wdata;
            end
        endcase
    end

//==== sequential circuit =================================
always@( posedge clk ) begin
    if( proc_reset ) begin
            state <= IN_CACHE;
            mem_read <= 0;
            mem_write <= 0;
            mem_addr <= 0;
            for (i = 0; i < 4; i = i + 1) valid[0][i] <= 0;
            for (i = 0; i < 4; i = i + 1) tag[0][i] <= 0;
            for (i = 0; i < 4; i = i + 1) dirty[0][i] <= 0;
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    data[0][i][j] <= 0;
                end
            end
            for (i = 0; i < 4; i = i + 1) valid[1][i] <= 0;
            for (i = 0; i < 4; i = i + 1) tag[1][i] <= 0;
            for (i = 0; i < 4; i = i + 1) dirty[1][i] <= 0;
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    data[1][i][j] <= 0;
                end
            end
            set_choose <= 0;
            for (i = 0; i < 4; i = i + 1) set_use[i] <= 1;
    end
    else begin
            state <= nxt_state;   
            mem_read <= nxt_mem_read;
            mem_write <= nxt_mem_write;
            mem_addr <= nxt_mem_addr;
            for (i = 0; i < 4; i = i + 1) valid[0][i] <= nxt_valid[0][i];
            for (i = 0; i < 4; i = i + 1) tag[0][i] <= nxt_tag[0][i];
            for (i = 0; i < 4; i = i + 1) dirty[0][i] <= nxt_dirty[0][i];
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    data[0][i][j] <= nxt_data[0][i][j];
                end
            end
            for (i = 0; i < 4; i = i + 1) valid[1][i] <= nxt_valid[1][i];
            for (i = 0; i < 4; i = i + 1) tag[1][i] <= nxt_tag[1][i];
            for (i = 0; i < 4; i = i + 1) dirty[1][i] <= nxt_dirty[1][i];
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    data[1][i][j] <= nxt_data[1][i][j];
                end
            end
            set_choose <= nxt_set_choose;
            for (i = 0; i < 4; i = i + 1) set_use[i] <= nxt_set_use[i];
    end
end

endmodule

module cache_read_only(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;
    

//==== Parameter ==========================================
    localparam IN_CACHE = 2'b00;
    localparam UPDATE_MEM = 2'b01;
    localparam READ_MEM = 2'b10;

//==== wire/reg definition ================================
    wire [25:0] data_tag;
    wire [1:0] index;
    wire [1:0] block_offset;
    wire hit0, hit1, hit;
    wire set_choose_temp;
    reg proc_stall;
    reg mem_read, nxt_mem_read;
    //reg mem_write, nxt_mem_write;
    reg [27:0] mem_addr, nxt_mem_addr;
    reg [1:0] state, nxt_state;
    reg [31:0] data [0:1][0:3][0:3];
    reg [31:0] nxt_data [0:1][0:3][0:3];
    reg [25:0] tag [0:1][0:3];
    reg [25:0] nxt_tag [0:1][0:3];
    reg valid [0:1][0:3];
    reg nxt_valid [0:1][0:3];
    reg set_choose, nxt_set_choose;
    reg set_use [0:3];
    reg nxt_set_use [0:3];
    
    integer i, j;

//==== combinational circuit ==============================
    assign data_tag = proc_addr[29:4];
    assign index = proc_addr[3:2];
    assign block_offset = proc_addr[1:0];
    assign hit0 = valid[0][index] & (tag[0][index] == data_tag);
    assign hit1 = valid[1][index] & (tag[1][index] == data_tag);
    assign hit = hit0 ^ hit1;
    assign proc_rdata = (hit0) ? data[0][index][block_offset] : data[1][index][block_offset];
    assign mem_wdata = 0;
    assign mem_write = 0;
    assign set_choose_temp = ~set_use[index];

    always@(*) begin
        for (i = 0; i < 4; i = i + 1) nxt_valid[0][i] = valid[0][i];
        for (i = 0; i < 4; i = i + 1) nxt_valid[1][i] = valid[1][i];
        for (i = 0; i < 4; i = i + 1) nxt_tag[0][i] = tag[0][i];
        for (i = 0; i < 4; i = i + 1) nxt_tag[1][i] = tag[1][i];
        for (i = 0; i < 4; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                nxt_data[0][i][j] = data[0][i][j];
            end
        end
        for (i = 0; i < 4; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                nxt_data[1][i][j] = data[1][i][j];
            end
        end
        nxt_set_choose = set_choose;
        for (i = 0; i < 4; i = i + 1) nxt_set_use[i] = set_use[i];
        proc_stall = 0;
        nxt_state = IN_CACHE;
        nxt_mem_read = 0;
        nxt_mem_addr = 0;
        case(state)
            IN_CACHE: begin
                if (hit) begin
                    nxt_state = IN_CACHE;
                    proc_stall = 0;
                    nxt_mem_read = 0;
                    nxt_mem_addr = proc_addr[29:2];
                end
                else if (proc_read) begin
                    proc_stall = 1;
                    nxt_set_choose = set_choose_temp;
                    nxt_state = READ_MEM;
                    nxt_mem_read = 1;
                    nxt_mem_addr = proc_addr[29:2];
                end
            end
            READ_MEM: begin
                nxt_mem_addr = proc_addr[29:2];
                if (mem_ready) begin
                    proc_stall = 1;
                    nxt_mem_read = 0;
                    nxt_valid[set_choose][index] = 1;
                    nxt_tag[set_choose][index] = data_tag;
                    nxt_data[set_choose][index][0] = mem_rdata[31:0];
                    nxt_data[set_choose][index][1] = mem_rdata[63:32];
                    nxt_data[set_choose][index][2] = mem_rdata[95:64];
                    nxt_data[set_choose][index][3] = mem_rdata[127:96];
                    nxt_set_use[index] = set_choose;
                    nxt_state = IN_CACHE;
                end
                else begin
                    nxt_state = READ_MEM;
                    proc_stall = 1;
                    nxt_mem_read = 1;
                end
            end
        endcase
    end

//==== sequential circuit =================================
always@( posedge clk ) begin
    if( proc_reset ) begin
            state <= IN_CACHE;
            mem_read <= 0;
            mem_addr <= 0;
            for (i = 0; i < 4; i = i + 1) valid[0][i] <= 0;
            for (i = 0; i < 4; i = i + 1) tag[0][i] <= 0;
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    data[0][i][j] <= 0;
                end
            end
            for (i = 0; i < 4; i = i + 1) valid[1][i] <= 0;
            for (i = 0; i < 4; i = i + 1) tag[1][i] <= 0;
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    data[1][i][j] <= 0;
                end
            end
            set_choose <= 0;
            for (i = 0; i < 4; i = i + 1) set_use[i] <= 1;
    end
    else begin
            state <= nxt_state;   
            mem_read <= nxt_mem_read;
            mem_addr <= nxt_mem_addr;
            for (i = 0; i < 4; i = i + 1) valid[0][i] <= nxt_valid[0][i];
            for (i = 0; i < 4; i = i + 1) tag[0][i] <= nxt_tag[0][i];
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    data[0][i][j] <= nxt_data[0][i][j];
                end
            end
            for (i = 0; i < 4; i = i + 1) valid[1][i] <= nxt_valid[1][i];
            for (i = 0; i < 4; i = i + 1) tag[1][i] <= nxt_tag[1][i];
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    data[1][i][j] <= nxt_data[1][i][j];
                end
            end
            set_choose <= nxt_set_choose;
            for (i = 0; i < 4; i = i + 1) set_use[i] <= nxt_set_use[i];
    end
end

endmodule
