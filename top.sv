`define LEDR_BASE       32'hFF200000
`define HEX3_HEX0_BASE  32'hFF200020
`define HEX5_HEX4_BASE  32'hFF200030
`define SW_BASE         32'hFF200040

module top(
           input  logic        MAX10_CLK1_50, 
  			output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite,
           input  logic [9:0]  SW,
           input  logic [1:0]  KEY,
           output logic [9:0]  LEDR,
           output logic [7:0] HEX0, HEX1, HEX2, HEX3,
output logic [7:0] HEX4, HEX5          
           ); 	
    logic clk;
	 assign clk = MAX10_CLK1_50;
    logic reset;
    assign reset = ~KEY[0];

    logic [31:0] PC, Instr, ReadData;
    logic [31:0] Read_data_from_mux;

    // Instantiate processor and memories
    // Note: ALUResult from processor is the Address (DataAdr) for memory
    riscvsingle rvsingle(clk, reset, PC, Instr, MemWrite, DataAdr, WriteData, ReadData);

    imem imem (PC, Instr);
    dmem dmem (clk, MemWrite, DataAdr, WriteData, Read_data_from_mux);

    always_ff@(posedge clk)
    begin
        if (reset) begin
    LEDR <= 10'b0;
    HEX0 <= 8'hFF; HEX1 <= 8'hFF; HEX2 <= 8'hFF; HEX3 <= 8'hFF; // Turn off
    HEX4 <= 8'hFF; HEX5 <= 8'hFF;
end
        else if (MemWrite)
        begin
            case (DataAdr)
                `LEDR_BASE: LEDR           <= WriteData[9:0];
					 `HEX3_HEX0_BASE: 
					 begin
					  HEX0 <= WriteData[7:0];
					  HEX1 <= WriteData[15:8];
					  HEX2 <= WriteData[23:16];
					  HEX3 <= WriteData[31:24];
					end
`HEX5_HEX4_BASE: begin
    HEX4 <= WriteData[7:0];
    HEX5 <= WriteData[15:8];
end

            endcase
        end
    end

    always_comb
    begin
        if (DataAdr == `SW_BASE)
            ReadData = {22'b0, SW[9:0]};
        else
            ReadData = Read_data_from_mux;
    end

endmodule

// ------------------------------------------------------------
// RISC-V SINGLE CYCLE CPU
// ------------------------------------------------------------
module riscvsingle(input  logic        clk, reset,
                   output logic [31:0] PC,
                   input  logic [31:0] Instr,
                   output logic        MemWrite,
                   output logic [31:0] ALUResult, WriteData,
                   input  logic [31:0] ReadData);
    
    logic       ALUSrc, RegWrite, Jump, Zero, PCSrc;
    logic [1:0] ResultSrc, ImmSrc;
    logic [2:0] ALUControl;

    controller c(Instr[6:0], Instr[14:12], Instr[30], Zero, 
                 ResultSrc, MemWrite, PCSrc, ALUSrc, 
                 RegWrite, Jump, ImmSrc, ALUControl);

    datapath dp(clk, reset, ResultSrc, PCSrc, ALUSrc, RegWrite, 
                ImmSrc, ALUControl, Zero, PC, Instr, 
                ALUResult, WriteData, ReadData);
endmodule

// ------------------------------------------------------------
// CONTROLLER
// ------------------------------------------------------------
module controller(input  logic [6:0] op,
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
                  input  logic       Zero,
                  output logic [1:0] ResultSrc,
                  output logic       MemWrite,
                  output logic       PCSrc, ALUSrc,
                  output logic       RegWrite, Jump,
                  output logic [1:0] ImmSrc,
                  output logic [2:0] ALUControl);
    
    logic [1:0] ALUOp;
    logic       Branch;

    maindec md(op, ResultSrc, MemWrite, Branch, ALUSrc, RegWrite, Jump, ImmSrc, ALUOp);
    aludec  ad(op[5], funct3, funct7b5, ALUOp, ALUControl);

    assign PCSrc = (Branch & Zero) | Jump;
endmodule

// ------------------------------------------------------------
// MAIN DECODER
// ------------------------------------------------------------
module maindec(input  logic [6:0] op,
               output logic [1:0] ResultSrc,
               output logic       MemWrite,
               output logic       Branch, ALUSrc,
               output logic       RegWrite, Jump,
               output logic [1:0] ImmSrc,
               output logic [1:0] ALUOp);
               
    logic [10:0] controls;

    assign {RegWrite, ImmSrc, ALUSrc, MemWrite, ResultSrc, Branch, ALUOp, Jump} = controls;

    always_comb
        case (op)
            // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
            7'b0000011: controls = 11'b1_00_1_0_01_0_00_0; // lw
            7'b0100011: controls = 11'b0_01_1_1_00_0_00_0; // sw
            7'b0110011: controls = 11'b1_xx_0_0_00_0_10_0; // R-type
            7'b1100011: controls = 11'b0_10_0_0_00_1_01_0; // beq
            7'b0010011: controls = 11'b1_00_1_0_00_0_10_0; // I-type
            7'b1101111: controls = 11'b1_11_0_0_10_0_00_1; // jal
            default:    controls = 11'bx_xx_x_x_xx_x_xx_x; // invalid
        endcase
endmodule

// ------------------------------------------------------------
// ALU DECODER
// ------------------------------------------------------------
module aludec(input  logic       opb5,
              input  logic [2:0] funct3,
              input  logic       funct7b5,
              input  logic [1:0] ALUOp,
              output logic [2:0] ALUControl);
    
    logic RtypeSub;
    assign RtypeSub = funct7b5 & opb5;

    always_comb
        case (ALUOp)
            2'b00: ALUControl = 3'b000; // Addition (lw/sw)
            2'b01: ALUControl = 3'b001; // Subtraction (beq)
            default: case(funct3)
                3'b000: if (RtypeSub) ALUControl = 3'b001; // sub
                        else          ALUControl = 3'b000; // add
                3'b010: ALUControl = 3'b101; // slt
                3'b110: ALUControl = 3'b011; // or
                3'b111: ALUControl = 3'b010; // and
                default: ALUControl = 3'bxxx; 
            endcase
        endcase
endmodule

// ------------------------------------------------------------
// ALU
// ------------------------------------------------------------
module alu(input  logic [31:0] a, b,
           input  logic [2:0]  alucontrol,
           input  logic        funct7b5, 
           output logic [31:0] result,
           output logic        zero);

    logic [31:0] condinvb, sum;
    logic        v;        
    logic        isAddSub; 

    assign condinvb = alucontrol[0] ? ~b : b;
    assign sum = a + condinvb + alucontrol[0];
    assign isAddSub = ~alucontrol[2] & ~alucontrol[1] | ~alucontrol[1] & alucontrol[0];

    always_comb
        case (alucontrol)
            3'b000:  result = sum;       // add
            3'b001:  result = sum;       // subtract
            3'b010:  result = a & b;     // and
            3'b011:  result = a | b;     // or
            3'b100:  result = a ^ b;     // xor
            3'b101:  result = {31'b0, sum[31] ^ v};  // slt
            3'b110:  result = a << b[4:0];  // sll
            3'b111:  if (funct7b5) 
                        result = $signed(a) >>> b[4:0]; // SRA
                     else 
                        result = a >> b[4:0]; // SRL
            default: result = 32'bx;
        endcase
 
    assign zero = (result == 32'b0);
    assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
endmodule

// ------------------------------------------------------------
// DATAPATH
// ------------------------------------------------------------
module datapath(input  logic        clk, reset,
                input  logic [1:0]  ResultSrc,
                input  logic        PCSrc, ALUSrc,
                input  logic        RegWrite,
                input  logic [1:0]  ImmSrc,
                input  logic [2:0]  ALUControl,
                output logic        Zero,
                output logic [31:0] PC,
                input  logic [31:0] Instr,
                output logic [31:0] ALUResult, WriteData,
                input  logic [31:0] ReadData);

    // FIXED TYPO HERE: Changed PCNest to PCNext
    logic [31:0] PCNext, PCPlus4, PCTarget;
    logic [31:0] ImmExt;        
    logic [31:0] SrcA, SrcB;
    logic [31:0] Result;

    flopr #(32) pcreg(clk, reset, PCNext, PC);
    adder       pcadd4(PC, 32'd4, PCPlus4);
    adder       pcaddbranch(PC, ImmExt, PCTarget);
    mux2 #(32)  pcmux(PCPlus4, PCTarget, PCSrc, PCNext);

    regfile rf (
        .clk(clk), 
        .we3(RegWrite), 
        .a1(Instr[19:15]), 
        .a2(Instr[24:20]), 
        .a3(Instr[11:7]), 
        .wd3(Result), 
        .rd1(SrcA), 
        .rd2(WriteData) 
    ); 

    extend      ext(Instr[31:7], ImmSrc, ImmExt);

    mux2 #(32)  srcbmux(WriteData, ImmExt, ALUSrc, SrcB);
    alu         alu(SrcA, SrcB, ALUControl, Instr[30], ALUResult, Zero);
    mux3 #(32)  resultmux(ALUResult, ReadData, PCPlus4, ResultSrc, Result);
endmodule

// ------------------------------------------------------------
// HELPER MODULES
// ------------------------------------------------------------

module regfile(input  logic        clk,
               input  logic        we3,
               input  logic [4:0]  a1, a2, a3,
               input  logic [31:0] wd3,
               output logic [31:0] rd1, rd2);
    logic [31:0] rf[31:0];

    always_ff @(posedge clk)
        if (we3) rf[a3] <= wd3;
            
    assign rd1 = (a1 != 0) ? rf[a1] : 0;
    assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);
    assign y = a + b;
endmodule

module extend(input  logic [31:7] instr,
              input  logic [1:0]  immsrc,
              output logic [31:0] immext);
    always_comb
        case(immsrc)
            2'b00:   immext = {{20{instr[31]}}, instr[31:20]};
            2'b01:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};
            2'b10:   immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
            2'b11:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
            default: immext = 32'bx; 
        endcase
endmodule

module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d,
               output logic [WIDTH-1:0] q);
    always_ff @(posedge clk, posedge reset)
        if (reset) q <= 0;
        else       q <= d;
endmodule

module flopenr #(parameter WIDTH = 8)
                (input  logic             clk, reset, en,
                 input  logic [WIDTH-1:0] d,
                 output logic [WIDTH-1:0] q);
    always_ff @(posedge clk, posedge reset)
        if(reset)    q <= 0;
        else if (en) q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH - 1:0] d0, d1,
              input  logic             s,
              output logic [WIDTH - 1:0] y);
    assign y = s ? d1 : d0; 
endmodule 

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH - 1:0] d0, d1, d2,
              input  logic [1:0]       s,
              output logic [WIDTH - 1:0] y);
     assign y = s[1] ? d2 : (s[0] ? d1 : d0);
 endmodule

// ------------------------------------------------------------
// MEMORIES
// ------------------------------------------------------------
module imem(input  logic [31:0] a,
            output logic [31:0] rd);
    logic [31:0] RAM[63:0];
    initial
      $readmemh("imem.txt", RAM);
    assign rd = RAM[a[31:2]]; // word aligned
endmodule

module dmem(input  logic        clk, we,
            input  logic [31:0] a, wd,
            output logic [31:0] rd);
  
 
  
  
    logic [31:0] RAM[63:0];
    assign rd = RAM[a[31:2]]; // word-aligned
    always_ff @(posedge clk)
        if (we) RAM[a[31:2]] <=wd;
  
   initial
    $readmemh("dmem.txt", RAM);
endmodule
