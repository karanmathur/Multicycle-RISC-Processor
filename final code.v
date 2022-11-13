// N HARISHCHANDRA PRASAD - 2018A3PS0422P
// KARAN SINGH MATHUR - 2018A3PS0340P
// AARSHIBH SINGH - 2018A3PS0437P
// The file location for the instruction file is just "ins_mem.dat"
// The file location for the data memory is just "dat_mem.dat"
// time range should be 0-3us
// Data in dat_mem.dat and ins_mem.dat are in Little Endian Format
// Waveforms are extremely magnified in ModelSim

`timescale 1ns / 1ps

module tb_multi_cycle;
    
    reg clk;
    reg reset;
    
    multi_cycle uut(.clk(clk), .reset(reset));
    
    wire [15:0] reg1, reg2, reg3, reg4, reg5, reg6, reg7, reg8,
    reg9, reg10, reg11, reg12, reg13, reg14, reg15;
    
    assign reg1 = uut.I.rf.registers[4'd1];
    assign reg2 = uut.I.rf.registers[4'd2];
    assign reg3 = uut.I.rf.registers[4'd3];
    assign reg4 = uut.I.rf.registers[4'd4];
    assign reg5 = uut.I.rf.registers[4'd5];
    assign reg6 = uut.I.rf.registers[4'd6];
    assign reg7 = uut.I.rf.registers[4'd7];
    assign reg8 = uut.I.rf.registers[4'd8];
    assign reg9 = uut.I.rf.registers[4'd9];
    assign reg10 = uut.I.rf.registers[4'd10];
    assign reg11 = uut.I.rf.registers[4'd11];
    assign reg12 = uut.I.rf.registers[4'd12];
    assign reg13 = uut.I.rf.registers[4'd13];
    assign reg14 = uut.I.rf.registers[4'd14];
    assign reg15 = uut.I.rf.registers[4'd15];
    
    wire [15:0] mem_data, Address, IR, MDR, ALUout, PC;
    wire [1:0] PCSrc;
    wire MemRead, PCWrite;
    
    assign mem_data = uut.I.insMem.mem_data;
    assign Address = uut.I.insMem.Address;
    assign MemRead = uut.MemRead;
    assign IR = uut.IR;
    assign MDR = uut.I.mdrReg.MDR;
    assign ALUout = uut.ALUout;
    assign PC = uut.PC;
    assign PCSrc = uut.PCSrc;
    assign PCWrite = uut.PCWrite;
    
    always #5 clk = ~clk;
    
    initial
    begin
        clk = 1'b0;
        reset = 1'b1;
        #10
        reset=1'b0;
    end
    
    always@*
    begin
    $display("The values of the internal registers 1-15 are - ");
    $display("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", reg1, reg2, reg3, reg4, reg5, reg6, reg7, reg8,
    reg9, reg10, reg11, reg12, reg13, reg14, reg15);
    end
    
    always@*
    begin
    $display("Instruction Register (IR), PC (Program Counter),  ALUout - ");
    $display("%d, %d, %d ", IR, PC, ALUout);
    end
    
endmodule

//------------------------------------------------------------------------------------------------------------------------

`timescale 1ns / 1ps

module multi_cycle(
clk, reset
    );

input clk, reset;
wire [3:0] opcode, func_field;
wire PCWrite, PCWriteCond, MemReadI, MemRead, MemWrite, IRWrite, RegWrite, RegDst, MemToReg, Branch;
wire [1:0] ALUSrcA, Shiftop, PCSrc;
wire [2:0] ALUSrcB, ALUop;
wire [4:0] present_state, next_state;

controlunit cu(
opcode, func_field, clk, reset, present_state, next_state, 
PCWrite, PCWriteCond, MemReadI, MemRead, MemWrite, IRWrite, RegWrite, RegDst, MemToReg, Branch, 
ALUSrcA, Shiftop, PCSrc, 
ALUSrcB, ALUop
    );
    
wire [15:0] A,  B;
wire [3:0] IR74;
wire [15:0] Result;
wire Carryout, ZeroBit;
    
alu my_alu(
A, B, IR74, Shiftop, ALUop, Result, Carryout, ZeroBit
    );

wire [15:0] RegA, RegB, RegC, RegD, RegE;
wire [15:0] IR;
wire [15:0] Address;
reg [15:0] ALUout;

interconnect I(.A(RegA), .B(RegB), .C(RegC), .D(RegD), .E(RegE), 
.IR(IR), .clk(clk), .IRWrite(IRWrite), .Address(Address), .MemReadI(MemReadI), .MemRead(MemRead), .RegWrite(RegWrite), 
.RegDst(RegDst), .MemToReg(MemToReg), .ALUout(ALUout), .MemWrite(MemWrite));

always@(posedge clk)
begin
ALUout <= Result;
end

wire [15:0] PCSrc_muxout;
wire [15:0] PC;

assign opcode = IR[15:12];
assign func_field = IR[3:0];
assign IR74 = IR[7:4];
assign Address = PC;

PC_reg pc_r(.reset(reset), .PCSrc_muxout(PCSrc_muxout), .PCWrite(PCWrite), .PCWriteCond(PCWriteCond), 
.zero(ZeroBit), .Branch(Branch), .clk(clk) , .PC(PC));

wire [15:0] ALUSrcB_muxout, ALUSrcA_muxout;

ALUSrcB_mux aluBmux(.IR(IR), .B(RegB), .ALUSrcB(ALUSrcB), .ALUSrcB_muxout(ALUSrcB_muxout)); 
ALUSrcA_mux aluAmux(.PC(PC), .A(RegA), .C(RegC), .D(RegD), .ALUSrcA(ALUSrcA), .ALUSrcA_muxout(ALUSrcA_muxout));

assign A = ALUSrcA_muxout;
assign B = ALUSrcB_muxout;

PCSrc_mux pcmux(.ALUoutput(Result), .ALUout(ALUout), .C(RegC), .PCSrc(PCSrc), .PCSrc_muxout(PCSrc_muxout));

endmodule

//------------------------------------------------------------------------------------------------------------------------

`timescale 1ns / 1ps

module alu(
A, B, IR74, Shiftop, ALUop, Result, Carryout, ZeroBit
    );

input [15:0] A,  B;
input [3:0] IR74;
// amount of shift
input [1:0] Shiftop;
// tells whether to shift arithmetic left, logical left, logical right etc
input [2:0] ALUop;
output reg [15:0] Result;
output Carryout, ZeroBit;

wire Binvert;
wire [1:0] ALUop10;
assign Binvert = ALUop [2];
assign ALUop10 = ALUop [1:0];

wire [15:0] Result0, Result1, Result2, Result3;

assign Result0 = {~(A[15]&B[15]), ~(A[14]&B[14]), ~(A[13]&B[13]), ~(A[12]&B[12]), ~(A[11]&B[11]), 
~(A[10]&B[10]), ~(A[9]&B[9]), ~(A[8]&B[8]), ~(A[7]&B[7]), ~(A[6]&B[6]), ~(A[5]&B[5]), 
~(A[4]&B[4]), ~(A[3]&B[3]), ~(A[2]&B[2]), ~(A[1]&B[1]), ~(A[0]&B[0])};

assign Result1 = {(A[15]|B[15]), (A[14]|B[14]), (A[13]|B[13]), (A[12]|B[12]), (A[11]|B[11]), 
(A[10]|B[10]), (A[9]|B[9]), (A[8]|B[8]), (A[7]|B[7]), (A[6]|B[6]), (A[5]|B[5]), 
(A[4]|B[4]), (A[3]|B[3]), (A[2]|B[2]), (A[1]|B[1]), (A[0]|B[0])};

wire [15:0] B_src_adder;

assign B_src_adder = Binvert? ~B : B;

adder16bit FA16(.X(A), .Y(B_src_adder), .Carryin(Binvert), .Result2(Result2), .Carryout(Carryout));

nor (ZeroBit, Result2[15], Result2[14], Result2[13], Result2[12], Result2[11], Result2[10], Result2[9], 
Result2[8], Result2[7], Result2[6], Result2[5], Result2[4], Result2[3], Result2[2], 
Result2[1], Result2[0]);

shiftreg SR(A, Shiftop, IR74, Result3);

always@(*)
begin
case(ALUop10)
2'b00: Result = Result0;
2'b01: Result = Result1;
2'b10: Result = Result2;
2'b11: Result = Result3;
default: Result = 16'dz;
endcase
end

endmodule

`timescale 1ns / 1ps

module shiftreg(
A, Shiftop, IR74, Result3
    );

input [15:0] A;
input [1:0] Shiftop;
// if 01, logical left
// if 10, logical right
// if 11, arithmetic left
input [3:0] IR74;
// amount of shifr
output reg [15:0] Result3;

always@(*)
begin

case(Shiftop)

2'b01: 
begin
case(IR74)
4'd0: Result3 = A;
4'd1: Result3 = {A[14:0], 1'd0};
4'd2: Result3 = {A[13:0], 2'd0};
4'd3: Result3 = {A[12:0], 3'd0};
4'd4: Result3 = {A[11:0], 4'd0};
4'd5: Result3 = {A[10:0], 5'd0};
4'd6: Result3 = {A[9:0], 6'd0};
4'd7: Result3 = {A[8:0], 7'd0};
4'd8: Result3 = {A[7:0], 8'd0};
4'd9: Result3 = {A[6:0], 9'd0};
4'd10: Result3 = {A[5:0], 10'd0};
4'd11: Result3 = {A[4:0], 11'd0};
4'd12: Result3 = {A[3:0], 12'd0};
4'd13: Result3 = {A[2:0], 13'd0};
4'd14: Result3 = {A[1:0], 14'd0};
4'd15: Result3 = {A[0], 15'd0};
default: Result3 = 16'dz;
endcase
end

2'b10: 
begin
case(IR74)
4'd0: Result3 = A;
4'd1: Result3 = {1'd0, A[15:1]};
4'd2: Result3 = {2'd0, A[15:2]};
4'd3: Result3 = {3'd0, A[15:3]};
4'd4: Result3 = {4'd0, A[15:4]};
4'd5: Result3 = {5'd0, A[15:5]};
4'd6: Result3 = {6'd0, A[15:6]};
4'd7: Result3 = {7'd0, A[15:7]};
4'd8: Result3 = {8'd0, A[15:8]};
4'd9: Result3 = {9'd0, A[15:9]};
4'd10: Result3 = {10'd0, A[15:10]};
4'd11: Result3 = {11'd0, A[15:11]};
4'd12: Result3 = {12'd0, A[15:12]};
4'd13: Result3 = {13'd0, A[15:13]};
4'd14: Result3 = {14'd0, A[15:14]};
4'd15: Result3 = {15'd0, A[15]};
default: Result3 = 16'dz;
endcase
end

2'b11: 
begin
case(IR74)
4'd0: Result3 = A;
4'd1: Result3 = {A[15], A[15:1]};
4'd2: Result3 = {A[15], A[15], A[15:2]};
4'd3: Result3 = {A[15], A[15], A[15], A[15:3]};
4'd4: Result3 = {A[15], A[15], A[15], A[15], A[15:4]};
4'd5: Result3 = {A[15], A[15], A[15], A[15], A[15], A[15:5]};
4'd6: Result3 = {A[15], A[15], A[15], A[15], A[15], A[15], A[15:6]};
4'd7: Result3 = {A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15:7]};
4'd8: Result3 = {A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15:8]};
4'd9: Result3 = {A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15:9]};
4'd10: Result3 = {A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15:10]};
4'd11: Result3 = {A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15:11]};
4'd12: Result3 = {A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15:12]};
4'd13: Result3 = {A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15:13]};
4'd14: Result3 = {A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15:14]};
4'd15: Result3 = {A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15], A[15]};
default: Result3 = 16'dz;
endcase
end

default: Result3 = 16'dz;

endcase

end

endmodule

`timescale 1ns / 1ps

module adder16bit(
X, Y, Carryin, Result2, Carryout
    );

input Carryin;
input [15:0] X, Y;
output [15:0] Result2;
output Carryout;

wire [16:0] a;

assign a = {1'b0, X} + {1'b0, Y} + Carryin;

assign Carryout = a[16];
assign Result2 = a[15:0];

endmodule

//-------------------------------------------------------------------------------------------------------------------------------------

`timescale 1ns / 1ps

module adder16bit_tb;

wire [15:0] Result2;
wire Carryout;
reg [15:0] X, Y;
reg Carryin;

adder16bit x(X, Y, Carryin, Result2, Carryout);

initial begin

Carryin=1'b0;
X=16'd10000;
Y=16'd5000;
#5
Carryin = 1'b1;
#5
X=16'b1111_1111_1111_1111;
Y=16'b1111_1111_1111_1111;
#5
X=16'd0;
#5
Y=16'd0;
#5
Carryin=1'b0;
#5
Carryin=1'b1;
X=16'd8926;
Y=16'd65000;

end

endmodule

//----------------------------------------------------------------------------------------------------------------------------------------

`timescale 1ns / 1ps

module shiftreg_tb;

wire [15:0] Result3;
reg [15:0] A;
reg [3:0] IR74;
reg [1:0] Shiftop;

shiftreg x(A, Shiftop, IR74, Result3);

initial begin

Shiftop = 2'd1;
A= 16'b1011_0000_1010_1001;
IR74 = 4'd14;
#5
IR74 = 4'd3;
#5
IR74 = 4'd0;
#5
A= 16'b1000_1110_1110_1011;
IR74 = 4'd1;
#5
IR74 = 4'd6;
#5

Shiftop = 2'd3;
A= 16'b1111_0000_1010_1111;
IR74 = 4'd5;
#5
IR74 = 4'd0;
#5
A= 16'b0110_0010_1011_0101;
IR74 = 4'd9;
#5

Shiftop = 2'd2;
A= 16'b1111_0000_1010_1111;
IR74 = 4'd5;
#5
IR74 = 4'd0;
#5
A= 16'b0110_0010_1011_0101;
IR74 = 4'd9;
#5

Shiftop=2'd3;
A=16'b1000_0010_0110_0101;
IR74=4'd12;
#5

Shiftop=2'd1;
A=16'b1111_0010_1000_1010;
IR74=4'd3;
#5

Shiftop=2'd2;
A=16'b1110_1010_1000_1010;
IR74=4'd2;

end

endmodule

//----------------------------------------------------------------------------------------------------------------------------------

`timescale 1ns / 1ps

module alu_tb;

reg [15:0] A,  B;
reg [3:0] IR74;
reg [1:0] Shiftop;
reg [2:0] ALUop;
wire [15:0] Result;
wire Carryout, ZeroBit;

alu my_alu(A, B, IR74, Shiftop, ALUop, Result, Carryout, ZeroBit);

initial begin

ALUop = 3'b000;
A = 16'd0;
B = 16'd0;
#5
A = 16'b0011_0101_1010_1111;
B = 16'b1110_1100_0010_0110;
#5
A = 16'b1111_1111_1111_1111;
B = 16'b0100_1100_1011_1001;
#5

ALUop = 3'b001;
A = 16'd0;
B = 16'b1110_1101_1001_0011;
#5
A = 16'b1111_0000_0000_1010;
B = 16'b1110_1101_1001_0011;

ALUop = 3'b011;
Shiftop = 2'b11;
IR74 = 4'd8;
A = 16'b1110_1101_1001_0011;
#5
A = 16'b1111_0000_0000_1010;
IR74=4'd2;
Shiftop = 2'b10;
#5
Shiftop = 2'b01;
#5
IR74=4'd0;
#5

ALUop = 3'b010;
A = 16'd0;
B = 16'b1110_1101_1001_0011;
#5
B = 16'd0;
#5
A=16'd100;
B=16'd200;
#5
A=16'd60000;
B=16'd10000;
#5

ALUop = 3'b110;
A = 16'd500;
B = 16'd2;
#5
B = 16'd0;
#5
A=16'b1000_1111_0000_1011;
B=16'b0100_1011_1010_1001;
#5
A=16'd15432;
B=16'd15432;
#5

ALUop=3'b100;
A=16'b1111_0000_1111_0000;
B=16'b0011_0111_1000_1001;
#5
ALUop=3'b001;
#5
ALUop=3'd2;
#5
ALUop=3'd6;
#5
ALUop=3'd3;
Shiftop=2'd2;
IR74=4'd6;
#5
Shiftop=2'd1;
IR74=4'd1;

end

endmodule

//--------------------------------------------------------------------------------------------------------------------------------

`timescale 1ns / 1ps

module controlunit(
opcode, func_field, clk, reset, present_state, next_state, 
PCWrite, PCWriteCond, MemReadI, MemRead, MemWrite, IRWrite, RegWrite, RegDst, MemToReg, Branch,
ALUSrcA, Shiftop, PCSrc, 
ALUSrcB, ALUop
    );

input clk, reset;
input [3:0] opcode, func_field;
output reg PCWrite, PCWriteCond, MemReadI, MemRead, MemWrite, IRWrite, RegWrite, RegDst, MemToReg, Branch;
output reg [1:0] ALUSrcA, Shiftop, PCSrc;
output reg [2:0] ALUSrcB, ALUop;
output reg [4:0] present_state, next_state;

localparam state0 = 5'd0, state1=5'd1, state2 = 5'd2, state3 = 5'd3, 
state4 = 5'd4, state5 = 5'd5, state6=5'd6, state7=5'd7, state8=5'd8, state9=5'd9, state10=5'd10,
state11 = 5'd11, state12 = 5'd12, state13 = 5'd13, state14 = 5'd14, state15 = 5'd15, state16 = 5'd16, 
state17 = 5'd17, state18 = 5'd18, state19 = 5'd19, state20 = 5'd20, state21 = 5'd21, state22 = 5'd22;

always @(posedge clk)
begin
if(reset)
present_state <= state0;
else
present_state <= next_state;
end

always @(*)
// outputs
begin
case(present_state)
state0: begin MemReadI = 1'b1;
Branch=1'b0; 
IRWrite = 1'b1; 
ALUSrcA = 2'b00; 
ALUSrcB = 3'b001; 
ALUop = 3'b010; 
PCSrc = 2'b00; 
PCWrite = 1'b1; 
RegWrite = 1'b0; 
MemRead = 1'b0;
MemWrite = 1'b0; end
state1: begin MemReadI = 1'b0;
Branch=1'b0; 
PCWrite = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state2: begin ALUSrcA = 2'b01;
Branch=1'b0;
ALUSrcB = 3'b000;
ALUop = 3'b010;
MemReadI = 1'b0;
PCWrite = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state3: begin MemToReg = 1'b0;
Branch=1'b0;
RegDst = 1'b1;
RegWrite = 1'b1;
MemReadI = 1'b0;
PCWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state4: begin ALUSrcA = 2'b01;
Branch=1'b0;
ALUSrcB = 3'b000;
ALUop = 3'b110;
MemReadI = 1'b0;
PCWrite = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state5: begin ALUSrcA = 2'b01;
Branch=1'b0;
ALUSrcB = 3'b000;
ALUop = 3'b000;
MemReadI = 1'b0;
PCWrite = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state6: begin ALUSrcA = 2'b01;
Branch=1'b0;
ALUSrcB = 3'b000;
ALUop = 3'b001;
MemReadI = 1'b0;
PCWrite = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state7: begin ALUSrcA = 2'b10;
Branch=1'b0;
ALUSrcB = 3'b101;
ALUop = 3'b010;
MemReadI = 1'b0;
PCWrite = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state8: begin ALUSrcA = 2'b10;
Branch=1'b0;
ALUSrcB = 3'b100;
ALUop = 3'b010;
MemReadI = 1'b0;
PCWrite = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state9: begin ALUSrcA = 2'b10;
Branch=1'b0;
ALUSrcB = 3'b101;
ALUop = 3'b110;
MemReadI = 1'b0;
PCWrite = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state10: begin ALUSrcA = 2'b10;
Branch=1'b0;
ALUSrcB = 3'b100;
ALUop = 3'b110;
MemReadI = 1'b0;
PCWrite = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state11: begin ALUSrcA = 2'b10;
Branch=1'b0;
ALUSrcB = 3'b101;
ALUop = 3'b000;
MemReadI = 1'b0;
PCWrite = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state12: begin ALUSrcA = 2'b10;
Branch=1'b0;
ALUSrcB = 3'b101;
ALUop = 3'b001;
MemReadI = 1'b0;
PCWrite = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state13: begin ALUSrcA = 2'b10;
Branch=1'b0;
ALUop = 3'b011;
Shiftop = 2'b01;
MemReadI = 1'b0;
PCWrite = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state14: begin ALUSrcA = 2'b10;
Branch=1'b0;
ALUop = 3'b011;
Shiftop = 2'b10;
MemReadI = 1'b0;
PCWrite = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state15: begin ALUSrcA = 2'b10;
Branch=1'b0;
ALUop = 3'b011;
Shiftop = 2'b11;
MemReadI = 1'b0;
PCWrite = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state16: begin ALUSrcA = 2'b01;
Branch=1'b1;
ALUSrcB = 3'b000;
ALUop = 3'b110;
PCWrite = 1'b0;
PCWriteCond =1'b1;
PCSrc = 2'b10;
MemReadI = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state17: begin ALUSrcA = 2'b01;
Branch=1'b1;
ALUSrcB = 3'b000;
ALUop = 3'b110;
PCWrite = 1'b0;
PCWriteCond = 1'b0;
PCSrc = 2'b10;
MemReadI = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state18: begin ALUSrcA = 2'b00;
Branch=1'b0;
ALUSrcB = 3'b011;
ALUop = 3'b010;
PCSrc = 2'b00;
PCWrite = 1'b1;
MemReadI = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state19: begin ALUSrcA = 2'b11;
Branch=1'b0;
ALUSrcB = 3'b010;
ALUop = 3'b010;
PCWrite = 1'b0;
MemReadI = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state20: begin MemWrite = 1'b1;
Branch=1'b0;
PCWrite = 1'b0;
MemReadI = 1'b0;
RegWrite = 1'b0;
MemRead = 1'b0;
IRWrite = 1'b0; end
state21: begin MemRead = 1'b1;
Branch=1'b0;
PCWrite = 1'b0;
MemReadI = 1'b0;
RegWrite = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
state22: begin MemToReg = 1'b1;
Branch=1'b0;
RegDst = 1'b0;
RegWrite = 1'b1;
PCWrite = 1'b0;
MemReadI = 1'b0;
MemRead = 1'b0;
MemWrite = 1'b0;
IRWrite = 1'b0; end
default: PCWrite = 1'bz;
//doesn't matter what the default case does, just need one so latch isn't synthesised
endcase
end

always @(*)
// next_state calculation
begin
case(present_state)
state0: next_state = state1;
state1: begin case(opcode)
4'b1000: next_state = state2;
4'b1100: next_state = state4;
4'b1011: next_state = state5;
4'b1111: next_state = state6;
4'b1001: next_state = state7;
4'b1010: next_state = state8;
4'b1101: next_state = state9;
4'b1110: next_state = state10;
4'b0111: next_state = state11;
4'b0110: next_state = state12;
4'b0000: next_state = func_field[0]? (func_field[1]?state15:state13) : state14;
4'b0100: next_state = state16;
4'b0101: next_state = state17;
4'b0011: next_state = state18;
4'b0010: next_state = state19;
4'b0001: next_state = state19;
default: next_state = 4'bzzzz;
endcase
end
state2: next_state = state3;
state3: next_state = state0;
state4: next_state = state3;
state5: next_state = state3;
state6: next_state = state3;
state7: next_state = state3;
state8: next_state = state3;
state9: next_state = state3;
state10: next_state = state3;
state11: next_state = state3;
state12: next_state = state3;
state13: next_state = state3;
state14: next_state = state3;
state15: next_state = state3;
state16: next_state = state0;
state17: next_state = state0;
state18: next_state = state0;
state19: next_state = opcode[0]?state21:state20;
state20: next_state = state0;
state21: next_state = state22;
state22: next_state = state0;
default: next_state = 4'dz;
endcase

end

endmodule

//----------------------------------------------------------------------------------------------------------------------------

`timescale 1ns / 1ps

module controlunit_tb;

reg clk, reset;
reg [3:0] opcode, func_field;
wire PCWrite, PCWriteCond, MemReadI, MemRead, MemWrite, IRWrite, RegWrite, RegDst, MemToReg, Branch;
wire [1:0] ALUSrcA, Shiftop, PCSrc;
wire [2:0] ALUSrcB, ALUop;
wire [4:0] present_state, next_state;

controlunit cu(
opcode, func_field, clk, reset, present_state, next_state, 
PCWrite, PCWriteCond, MemReadI, MemRead, MemWrite, IRWrite, RegWrite, RegDst, MemToReg, Branch, 
ALUSrcA, Shiftop, PCSrc, 
ALUSrcB, ALUop);

always #5 clk=~clk;

initial begin
clk=1'b0;
reset=1'b1;
#10
// checking state transition for each opcode
opcode=4'b1000;
reset=1'b0;
#40
opcode=4'b1100;
#40
opcode=4'b1011;
#40
opcode=4'b1100;
#40
opcode=4'b1111;
#40
opcode=4'b1001;
#40
opcode=4'b1010;
#40
opcode=4'b1101;
#40
opcode=4'b1110;
#40
opcode=4'b0111;
#40
opcode=4'b0110;
#40
opcode=4'b0000;
func_field = 4'b0001;
#40
opcode=4'b0000;
func_field = 4'b0010;
#40
opcode=4'b0000;
func_field = 4'b0011;
#40
opcode=4'b0100;
#30
opcode=4'b0101;
#30
opcode=4'b0011;
#30
opcode=4'b0010;
#40
opcode=4'b0001;
#50;



end

endmodule

//-----------------------------------------------------------------------------------------------------------------------------

// memory not equal to 64kB
// registers other than r0 initialized
// instantiate interconnect, inputs and outputs:
// outputs: A, B, C, D, E, IR; inputs: clk, IRWrite, Address, MemReadI, MemRead, RegWrite, RegDst, MemToReg, ALUout;
// connect it between PC and ALU

//------------------------------------------------------------------------
// interconnect module (interconnects, muxes...)

module interconnect(A, B, C, D, E, IR, clk, IRWrite, Address, MemReadI, MemRead, RegWrite, RegDst, MemToReg, ALUout, MemWrite); // outputs: A, B, C, D, E, IR; inputs: clk, IRWrite, Address, MemReadI, MemRead, RegWrite, RegDst, MemToReg, ALUout;

    // direct connections
    input wire clk;
    output wire [15:0] A;
    wire [15:0] read_data1;
    output wire [15:0] B;
    wire [15:0] read_data2;
    output wire [15:0] C;
    wire [15:0] read_data3;
    output wire [15:0] D;
    wire [15:0] read_data4;
    output wire [15:0] E;
    wire [15:0] read_data5;
    output wire [15:0] IR;
    input wire IRWrite;
    wire [15:0] MDR;
    wire [15:0] mem_dataI;
    input wire [15:0] Address;
    input wire MemReadI;
    wire [15:0] mem_data;
    input wire MemRead;
    input wire MemWrite;
    
    // might involve muxes
    wire [3:0] read_reg1;
    wire [3:0] read_reg2;
    wire [3:0] read_reg3;
    wire [3:0] read_reg4;
    wire [3:0] read_reg5;
    wire [3:0] write_register;
    wire [15:0] write_data;
    input wire RegWrite;
    input wire RegDst;
    input wire MemToReg;
    input wire [15:0] ALUout;

    assign read_reg1 = IR[7:4];
    assign read_reg2 = IR[3:0];
    assign read_reg3 = IR[11:8];
    assign read_reg4 = {2'b10, IR[9:8]};
    assign read_reg5 = {2'b11, IR[11:10]};
    assign write_data = (MemToReg)? MDR : ALUout;
    assign write_register = (RegDst)? IR[11:8] : {2'b11, IR[11:10]};

    register_file rf(.read_data1(read_data1), .read_data2(read_data2), .read_data3(read_data3), .read_data4(read_data4), .read_data5(read_data5), .read_reg1(read_reg1), .read_reg2(read_reg2), .read_reg3(read_reg3), .read_reg4(read_reg4), .read_reg5(read_reg5), .write_register(write_register), .write_data(write_data), .RegWrite(RegWrite), .clk(clk));// (read_data1, read_data2, read_data3, read_data4, read_data5, read_reg1, read_reg2, read_reg3, read_reg4, read_reg5, write_register, write_data, RegWrite, clk)
    register_A rA(.A(A), .read_data1(read_data1), .clk(clk));    // (A, read_data1, clk)
    register_B rB(.B(B), .read_data2(read_data2), .clk(clk));    // (B, read_data2, clk)
    register_C rC(.C(C), .read_data3(read_data3), .clk(clk));    // (C, read_data3, clk)
    register_D rD(.D(D), .read_data4(read_data4), .clk(clk));    // (D, read_data4, clk)
    register_E rE(.E(E), .read_data5(read_data5), .clk(clk));    // (E, read_data5, clk)
    instruction_register insReg(.IR(IR), .IRWrite(IRWrite), .mem_data(mem_dataI), .clk(clk));  // (IR, IRWrite, mem_data, clk)
    MDR_reg mdrReg(.MDR(MDR), .mem_data(mem_data), .clk(clk));   // (MDR, mem_data, clk)
    instruction_memory insMem(.mem_data(mem_dataI), .Address(Address), .MemReadI(MemReadI));    // (mem_data, Address, MemReadI)
    data_memory dataMem(.mem_data(mem_data), .Address(ALUout), .MemRead(MemRead), .MemWrite(MemWrite), .write_data(E));  // (mem_data, Address, MemRead, MemWrite, write_data)

endmodule


//------------------------------------------------------------------------
// register file

module register_file(read_data1, read_data2, read_data3, read_data4, read_data5, read_reg1, read_reg2, read_reg3, read_reg4, read_reg5, write_register, write_data, RegWrite, clk);
    
    output [15:0] read_data1;
    output [15:0] read_data2;
    output [15:0] read_data3;
    output [15:0] read_data4;
    output [15:0] read_data5;
    input [3:0] read_reg1;
    input [3:0] read_reg2;
    input [3:0] read_reg3;
    input [3:0] read_reg4;
    input [3:0] read_reg5;
    input [3:0] write_register;
    input [15:0] write_data;
    input RegWrite;
    input clk;
    
    reg [15:0] registers [15:0];
    
    initial
    begin
        registers[4'd0] = 16'd0; // r0 = 0
        registers[4'd1] = 16'd8;
        registers[4'd2] = 16'd4;
        registers[4'd3] = 16'd2;
        registers[4'd4] = 16'd0; // r0 = 0
        registers[4'd5] = 16'd8;
        registers[4'd6] = 16'd4;
        registers[4'd7] = 16'd1000;
        registers[4'd8] = 16'd500;
        registers[4'd9] = 16'd0;
        registers[4'd10] = 16'd2;
        registers[4'd11] = 16'd0;
        registers[4'd12] = 16'd0;
        registers[4'd13] = 16'd0;
        registers[4'd14] = 16'd0;
        registers[4'd15] = 16'd0;
    end
    
    
    assign read_data1 = registers[read_reg1];
    assign read_data2 = registers[read_reg2];
    assign read_data3 = registers[read_reg3];
    assign read_data4 = registers[read_reg4];
    assign read_data5 = registers[read_reg5];
    
    always @ (posedge clk)
    begin
        if(RegWrite)
        begin
            registers[write_register] <= write_data;
        end
    end
    
endmodule

//------------------------------------------------------------------------
// registers (A, B, C, D & E)

module register_A(A, read_data1, clk);

    output reg [15:0] A;
    input [15:0] read_data1;
    input clk;
    
    always @ (posedge clk)
    begin
        A <= read_data1;
    end

endmodule

module register_B(B, read_data2, clk);

    output reg [15:0] B;
    input [15:0] read_data2;
    input clk;
    
    always @ (posedge clk)
    begin
        B <= read_data2;
    end

endmodule

module register_C(C, read_data3, clk);

    output reg [15:0] C;
    input [15:0] read_data3;
    input clk;
    
    always @ (posedge clk)
    begin
        C <= read_data3;
    end

endmodule

module register_D(D, read_data4, clk);

    output reg [15:0] D;
    input [15:0] read_data4;
    input clk;
    
    always @ (posedge clk)
    begin
        D <= read_data4;
    end

endmodule

module register_E(E, read_data5, clk);

    output reg [15:0] E;
    input [15:0] read_data5;
    input clk;
    
    always @ (posedge clk)
    begin
        E <= read_data5;
    end

endmodule


//------------------------------------------------------------------------
// instruction register

module instruction_register(IR, IRWrite, mem_data, clk);
    
    output reg [15:0] IR;
    input IRWrite;
    input [15:0] mem_data;
    input clk;
    
    always @ (posedge clk)
    begin
        if(IRWrite)
        begin
            IR <= mem_data;
        end
    end
    
endmodule


//------------------------------------------------------------------------
// MDR

module MDR_reg(MDR, mem_data, clk);

    output reg [15:0] MDR;
    input [15:0] mem_data;
    input clk;
    
    always @ (posedge clk)
    begin
        MDR <= mem_data;
    end

endmodule


//------------------------------------------------------------------------
// instruction memory

module instruction_memory(mem_data, Address, MemReadI);

    output reg [15:0] mem_data;
    input [15:0] Address;
    input MemReadI;
    
    reg [7:0] ins_mem [127:0];   // code wasn't running for 64kB
    wire [15:0] nAddress;  // next address
    
    assign nAddress = Address + 16'b1;
    
    initial
    begin
        $readmemb("ins_mem.dat", ins_mem);
    end
    
    always @ (*)
    begin
        if(MemReadI)
        begin
            mem_data <= {ins_mem[nAddress], ins_mem[Address]};
        end
    end

endmodule


//------------------------------------------------------------------------
// data memory

module data_memory(mem_data, Address, MemRead, MemWrite, write_data);

    output reg [15:0] mem_data;
    input [15:0] Address;
    input MemRead;
    input MemWrite;
    input [15:0] write_data;

    reg [7:0] dat_mem [11:0];   // code wasn't running for 64kB
    wire [15:0] nAddress;  // next address
    
    assign nAddress = Address + 16'b1;
    
    initial
    begin
        $readmemb("dat_mem.dat", dat_mem);
    end
    
    always @ (*)
    begin
        if(MemRead)
        begin
            mem_data <= {dat_mem[nAddress], dat_mem[Address]};
        end
    end
    
    always @ (*)
    begin
        if(MemWrite)
        begin
            {dat_mem[nAddress], dat_mem[Address]} = write_data;
            $writememb("dat_mem.dat", dat_mem);
        end
    end

endmodule

//-----------------------------------------------------------------------------------------------------------------------------


`timescale 1ns/1ns

module PC_reg(input reset, input [15:0] PCSrc_muxout, input PCWrite, PCWriteCond, zero, clk, Branch ,output reg [15:0] PC); 

wire PCWriteCntl; 

assign PCWriteCntl = (( (~(zero^PCWriteCond))&(Branch) )|PCWrite)&(~reset); 

always@(posedge clk) begin 

	if(PCWriteCntl == 1'b1) begin 

	PC <= PCSrc_muxout; 

	end
end

always@(posedge clk) begin

    if(reset) begin
    
    PC <= 16'b0;
    
    end
end

endmodule 

module ALUSrcB_mux(input [15:0] IR, B, input [2:0] ALUSrcB, output [15:0] ALUSrcB_muxout); 

wire [15:0] inp0,inp1,inp2,inp3,inp4,inp5;

assign inp4 = {8'd0,IR[7:0]}; 
assign inp5 = IR[7]?{8'd1,IR[7:0]}:{8'd0,IR[7:0]};
assign inp2 = inp5<<1'b1; 
assign inp0 = B;
assign inp1 = 16'd2; 
assign inp3 = IR[11]?{4'd1,IR[11:0]}:{4'd0,IR[11:0]};

assign ALUSrcB_muxout = ALUSrcB[2]?(ALUSrcB[0]?inp5:inp4):(ALUSrcB[1]?(ALUSrcB[0]?inp3:inp2):(ALUSrcB[0]?inp1:inp0));

endmodule 

module ALUSrcA_mux(input [15:0] PC, A, C, D, input [1:0] ALUSrcA, output [15:0] ALUSrcA_muxout);

wire [15:0] inp0,inp1,inp2,inp3; 

assign inp0 = PC;
assign inp1 = A;
assign inp2 = C; 
assign inp3 = D; 

assign ALUSrcA_muxout = ALUSrcA[1]?(ALUSrcA[0]?inp3:inp2):(ALUSrcA[0]?inp1:inp0);

endmodule 

module PCSrc_mux(input [15:0] ALUoutput, ALUout, C, input [1:0] PCSrc, output [15:0] PCSrc_muxout);

wire [15:0] inp0,inp1,inp2; 

assign inp0 = ALUoutput; 
assign inp1 = ALUout; 
assign inp2 = C;

assign PCSrc_muxout = PCSrc[1]?(PCSrc[0]?16'dz:inp2):(PCSrc[0]?inp1:inp0); 
 
endmodule

//-------------------------------------------------------------------------------------------------------------------------------

