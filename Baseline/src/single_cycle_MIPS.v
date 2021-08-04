// Single Cycle MIPS
//=========================================================
// Input/Output Signals:
// positive-edge triggered         clk
// active low synchronous reset   rst_n
// instruction memory interface    IR_addr, IR
// output for testing purposes     RF_writedata  
//=========================================================
// Wire/Reg Specifications:
// control signals             MemToReg, MemRead, MemWrite, 
//                             RegDST, RegWrite, Branch, 
//                             Jump, ALUSrc, ALUOp
// ALU control signals         ALUctrl
// ALU input signals           ALUin1, ALUin2
// ALU output signals          ALUresult, ALUzero
// instruction specifications  r, j, jal, jr, lw, sw, beq
// sign-extended signal        SignExtend
// MUX output signals          MUX_RegDST, MUX_MemToReg, 
//                             MUX_Src, MUX_Branch, MUX_Jump
// registers input signals     Reg_R1, Reg_R2, Reg_W, WriteData 
// registers                   Register
// registers output signals    ReadData1, ReadData2
// data memory contral signals CEN, OEN, WEN
// data memory output signals  ReadDataMem
// program counter/address     PCin, PCnext, JumpAddr, BranchAddr
//=========================================================

module SingleCycle_CHIP( 
    clk,
    rst_n,
    IR_addr,
    IR,
    RF_writedata,
    ReadDataMem,
    CEN,
    WEN,
    A,
    ReadData2,
    OEN
);

//==== in/out declaration =================================
    //-------- processor ----------------------------------
    input         clk, rst_n;
    input  [31:0] IR;
    output [31:0] IR_addr, RF_writedata;
    //-------- data memory --------------------------------
    input  [31:0] ReadDataMem;  // read_data from memory
    output        CEN;  // chip_enable, 0 when you read/write data from/to memory
    output        WEN;  // write_enable, 0 when you write data into SRAM & 1 when you read data from SRAM
    output  [6:0] A;  // address
    output [31:0] ReadData2;  // write_data to memory
    output        OEN;  // output_enable, 0

//==== reg/wire declaration ===============================
    reg [31:0] PC;
    wire [31:0] PC_n, busX, aluY, ALUresult, PC_plus4, extended, branch_out;
    wire [4:0] write_reg;
    wire [2:0] alu_ctrl;
    wire [1:0] RegDst, MemtoReg, ALUOp;
    wire zero, RegWrite, Jump, Jr, Branch, ALUSrc;

//==== combinational part =================================
    assign IR_addr = PC;
    assign A = ALUresult[8:2];
    assign OEN = 0;

    alu alu0(.ctrl(alu_ctrl), .x(busX), .y(aluY), .zero(zero), .out(ALUresult));
    control control0(.op(IR[31:26]), .f5(IR[5]), .RegDst(RegDst), .Jump(Jump), .Jr(Jr), .Branch(Branch), .CEN(CEN), .MemtoReg(MemtoReg), .ALUOp(ALUOp), .WEN(WEN), .ALUSrc(ALUSrc), .RegWrite(RegWrite));
    alu_control alu_control0(.ALUOp(ALUOp), .Funct_field(IR[5:0]), .ALUOperation(alu_ctrl));
    PC_adder pc_adder0(.pc(IR_addr), .PC_plus4(PC_plus4));
    branch_mux bm0(.imm(extended), .PC_plus4(PC_plus4), .Branch(Branch), .Zero(zero), .out(branch_out));
    jump_mux jm0(.imm(IR[25:0]), .PC_plus4(PC_plus4), .branch_out(branch_out), .busX(busX), .Jump(Jump), .Jr(Jr), .out(PC_n));
    alu_mux am0(.x(extended), .y(ReadData2), .ctrl(ALUSrc), .out(aluY));
    WR_mux wm0(.x(IR[20:16]), .y(IR[15:11]), .ctrl(RegDst), .out(write_reg));
    busW_mux bum0(.x(ALUresult), .y(ReadDataMem), .PC_plus4(PC_plus4), .ctrl(MemtoReg), .out(RF_writedata));
    sign_extend sign0(.x(IR[15:0]), .out(extended));

//==== sequential part ====================================
    register_file reg_file0(.Clk(clk), .rst(rst_n), .WEN(RegWrite), .RW(write_reg), .busW(RF_writedata), .RX(IR[25:21]), .RY(IR[20:16]), .busX(busX), .busY(ReadData2));
    always@(posedge clk) begin
        if (!rst_n) begin
            PC <= 0;
        end
        else begin
            PC <= PC_n;
        end
    end

//=========================================================
endmodule

module alu(
    ctrl,
    x,
    y,
    zero,
    out 
);
    
    input  [2:0] ctrl;
    input  [31:0] x;
    input  [31:0] y;
    output       zero;
    output [31:0] out;
   
    reg [31:0] out;
    assign zero = !out;

    always@(*) begin
        case (ctrl)
            3'b000: out = x & y;
            3'b001: out = x | y;
            3'b010: out = x + y;
            3'b110: out = x - y;
            3'b111: out = (x < y) ? 1 : 0;
            default: out = 0;
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
    f5,
    RegDst,
    Jump,
    Jr,
    Branch,
    CEN,
    MemtoReg,
    ALUOp,
    WEN,
    ALUSrc,
    RegWrite
);

    input  [5:0] op;
    input  f5;
    output       Jump, Jr, Branch, CEN, WEN, ALUSrc, RegWrite;
    output [1:0] RegDst, MemtoReg, ALUOp;

    reg       Jump, Jr, CEN, WEN, ALUSrc, RegWrite;
    reg [1:0] RegDst, MemtoReg, ALUOp;

    assign Branch = ALUOp[0];
    always@(*) begin
        Jump = 0;
        Jr = 0;
        CEN = 1;
        WEN = 0;
        ALUSrc = 0;
        RegWrite = 0;
        RegDst = 0;
        MemtoReg = 0;
        ALUOp = 0;
        case (op) //beq is default
            6'b000000: begin //R-type
                RegDst = 1;
                RegWrite = f5;
                ALUOp = 2;
                Jr = ~f5;
            end
            6'b100011: begin //lw
                CEN = 0;
                WEN = 1;
                MemtoReg = 1;
                ALUSrc = 1;
                RegWrite = 1;
            end
            6'b101011: begin //sw
                CEN = 0;
                ALUSrc = 1;
            end
            6'b000100:  //beq
                ALUOp = 1;
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
    ALUOperation
);

    input  [1:0] ALUOp;
    input  [5:0] Funct_field;
    output [2:0] ALUOperation;

    reg [2:0] ALUOperation;
    always@(*) begin
        case (ALUOp)
            2'b00: ALUOperation = 2; //lw,sw
            2'b01: ALUOperation = 6; //beq
            2'b10: begin//R-type
                case (Funct_field[3:0])
                    4'b0000: ALUOperation = 2; //add
                    4'b0010: ALUOperation = 6; //subtract
                    4'b0100: ALUOperation = 0; //and
                    4'b0101: ALUOperation = 1; //or
                    4'b1010: ALUOperation = 7; //set on less than
                    default: ALUOperation = 0;
                endcase
            end
            default: ALUOperation = 0;
        endcase
    end

endmodule

module PC_adder(pc, PC_plus4);

    input  [31:0] pc;
    output [31:0] PC_plus4;

    assign PC_plus4 = pc + 4;

endmodule

module branch_mux(
    imm, 
    PC_plus4,
    Branch,
    Zero,
    out
);

    input  [31:0] imm, PC_plus4;
    input Branch, Zero;
    output [31:0] out;

    wire [31:0] address;
    wire ctrl;

    assign ctrl = Branch & Zero;
    assign address = imm << 2;
    assign out = (ctrl) ? (address + PC_plus4) : PC_plus4;

endmodule

module jump_mux(
    imm, 
    PC_plus4,
    branch_out,
    busX,
    Jump,
    Jr,
    out
);

    input  [25:0] imm;
    input  [31:0] PC_plus4, branch_out, busX;
    input Jump, Jr;
    output [31:0] out;

    wire [27:0] address;
    wire [31:0] jump_address;
    wire [31:0] net;

    assign address = imm << 2;
    assign jump_address = {PC_plus4[31:28], address};
    assign out = Jr ? busX : (Jump ? jump_address : branch_out);

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

    input  [31:0] x, y, PC_plus4;
    input  [1:0]  ctrl;
    output [31:0] out;

    assign out = (ctrl[1]) ? PC_plus4 : (ctrl[0] ? y : x);
endmodule

module sign_extend(x, out);

    input  [15:0] x;
    output [31:0] out;

    assign out = {{16{x[15]}}, x};

endmodule
