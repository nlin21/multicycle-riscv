module top(input logic clk, reset,
			  output logic [31:0] WriteData, DataAdr,
			  output logic MemWrite);
			  
	logic [31:0] ReadData;
	
	riscv rvmulti(clk, reset, ReadData, DataAdr, MemWrite, WriteData);
	umem unifiedmem(clk, MemWrite, DataAdr, WriteData, ReadData);
	
endmodule

module riscv(input logic clk, reset,
				 input logic [31:0] ReadData,			// from umem
				 output logic [31:0] Adr,				// to umem
				 output logic MemWrite,					// from controller to umem
				 output logic [31:0] WriteData);		// to umem
	
	// Internal signals
	logic [1:0] ImmSrc, ALUSrcA, ALUSrcB, ResultSrc;
	logic AdrSrc, Zero;
	logic [2:0] ALUControl;
	logic IRWrite, PCWrite;
	logic RegWrite;
	logic [31:0] Instr;
	
	controller c(clk, reset, Instr[6:0], Instr[14:12], Instr[30],
					 Zero, ImmSrc, ALUSrcA, ALUSrcB, ResultSrc, AdrSrc,
					 ALUControl, IRWrite, PCWrite, RegWrite, MemWrite);
					 
	datapath dp(clk, reset, ResultSrc, ALUControl, ALUSrcA, ALUSrcB,
					ImmSrc, RegWrite, PCWrite, AdrSrc, IRWrite, ReadData,
					Instr, Zero, WriteData, Adr);
	
endmodule

module controller(input logic clk,
						input logic reset,
						input logic [6:0] op,
						input logic [2:0] funct3,
						input logic funct7b5,
						input logic zero,
						output logic [1:0] immsrc,
						output logic [1:0] alusrca, alusrcb,
						output logic [1:0] resultsrc,
						output logic adrsrc,
						output logic [2:0] alucontrol,
						output logic irwrite, pcwrite,
						output logic regwrite, memwrite);
	
	logic [1:0] ALUOp;
  logic branch, pcupdate;
	logic [3:0] state;
	
	mainfsm mm(clk, reset, op, branch, pcupdate, regwrite, memwrite, irwrite, resultsrc, alusrca, alusrcb, adrsrc, ALUOp, state);		
	aludec ad(op[5], funct3, funct7b5, ALUOp, alucontrol);
	instrdec id(op, immsrc);
	
	assign pcwrite = (zero && branch) || pcupdate;
	
endmodule

module mainfsm(input logic clk,
					input logic reset,
					input logic [6:0] op,
					output logic branch,
					output logic pcupdate,
					output logic regwrite,
					output logic memwrite,
					output logic irwrite,
					output logic [1:0] resultsrc,
					output logic [1:0] alusrca, alusrcb,
					output logic adrsrc,
					output logic [1:0] ALUOp,
					output logic [3:0] state);
	
	logic S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10;
	logic S0_next, S1_next, S2_next, S3_next, S4_next, S5_next, S6_next, S7_next, S8_next, S9_next, S10_next;
	
	
	assign S0_next  = reset || S4 || S5 || S7 || S10;
	assign S1_next  = ~reset && S0;
	assign S2_next  = ~reset && S1 && (op[6:0] == 7'b0000011 || op[6:0] == 7'b0100011);
	assign S3_next  = ~reset && S2 && (op[6:0] == 7'b0000011);
	assign S4_next  = ~reset && S3;
	assign S5_next  = ~reset && S2 && (op[6:0] == 7'b0100011);
	assign S6_next  = ~reset && S1 && (op[6:0] == 7'b0110011);
	assign S7_next  = ~reset && (S6 || S8 || S9);
	assign S8_next  = ~reset && S1 && (op[6:0] == 7'b0010011);
	assign S9_next  = ~reset && S1 && (op[6:0] == 7'b1101111);
	assign S10_next = ~reset && S1 && (op[6:0] == 7'b1100011);
	 
	always_ff@ (posedge clk)
		begin
			S0  <= S0_next;
			S1  <= S1_next;
			S2  <= S2_next;
			S3  <= S3_next;
			S4  <= S4_next;
			S5  <= S5_next;
			S6  <= S6_next;
			S7  <= S7_next;
			S8  <= S8_next;
			S9  <= S9_next;
			S10 <= S10_next;
		end
		
	always_comb begin
		if (S0) begin
			adrsrc    = 1'b0;
			irwrite   = 1'b1;
			alusrca   = 2'b00;
			alusrcb   = 2'b10;
			ALUOp     = 2'b00;
			resultsrc = 2'b10;
			pcupdate  = 1'b1;
			// set dont cares to 0
			branch    = 1'b0;
			regwrite  = 1'b0;
			memwrite  = 1'b0;
			state 	 = 4'b0000;
		end else if (S1) begin
			alusrca   = 2'b01;
			alusrcb   = 2'b01;
			ALUOp     = 2'b00;
			// set dont cares to 0
			branch    = 1'b0;
			pcupdate  = 1'b0;
			regwrite  = 1'b0;
			memwrite  = 1'b0;
			irwrite   = 1'b0;
			resultsrc = 2'b00;
			adrsrc	 = 1'b0;
			state 	 = 4'b0001;
		end else if (S2) begin
			alusrca   = 2'b10;
			alusrcb   = 2'b01;
			ALUOp     = 2'b00;
			// set dont cares to 0
			branch    = 1'b0;
			pcupdate  = 1'b0;
			regwrite  = 1'b0;
			memwrite  = 1'b0;
			irwrite   = 1'b0;
			resultsrc = 2'b00;
			adrsrc	 = 1'b0;
			state 	 = 4'b0010;
		end else if (S3) begin
			resultsrc = 2'b00;
			adrsrc 	 = 1'b1;
			// set dont cares to 0
			branch    = 1'b0;
			pcupdate  = 1'b0;
			regwrite  = 1'b0;
			memwrite  = 1'b0;
			irwrite   = 1'b0;
			alusrca   = 2'b00;
			alusrcb   = 2'b00;
			ALUOp     = 2'b00;
			state 	 = 4'b0011;
		end else if (S4) begin
			resultsrc = 2'b01;
			regwrite  = 1'b1;
			// set dont cares to 0
			branch    = 1'b0;
			pcupdate  = 1'b0;
			memwrite  = 1'b0;
			irwrite   = 1'b0;
			alusrca   = 2'b00;
			alusrcb   = 2'b00;
			ALUOp     = 2'b00;
			adrsrc	 = 1'b0;
			state 	 = 4'b0100;
		end else if (S5) begin
			resultsrc = 2'b00;
			adrsrc 	 = 1'b1;
			memwrite  = 1'b1;
			// set dont cares to 0
			branch	 = 1'b0;
			pcupdate  = 1'b0;
			regwrite  = 1'b0;
			irwrite	 = 1'b0;
			alusrca   = 2'b00;
			alusrcb   = 2'b00;
			ALUOp     = 2'b00;
			state 	 = 4'b0101;
		end else if (S6) begin
			alusrca   = 2'b10;
			alusrcb   = 2'b00;
			ALUOp     = 2'b10;
			// set dont cares to 0
			branch	 = 1'b0;
			pcupdate  = 1'b0;
			regwrite  = 1'b0;
			memwrite  = 1'b0;
			irwrite   = 1'b0;
			resultsrc = 2'b00;
			adrsrc 	 = 1'b0;
			state 	 = 4'b0110;
		end else if (S7) begin
			resultsrc = 2'b00;
			regwrite  = 1'b1;
			// set dont cares to 0
			branch	 = 1'b0;
			pcupdate  = 1'b0;
			memwrite  = 1'b0;
			irwrite	 = 1'b0;
			alusrca   = 2'b00;
			alusrcb   = 2'b00;
			adrsrc 	 = 1'b0;
			ALUOp     = 2'b00;
			state 	 = 4'b0111;
		end else if (S8) begin
			alusrca   = 2'b10;
			alusrcb   = 2'b01;
			ALUOp     = 2'b10;
			// set dont cares to 0
			branch	 = 1'b0;
			pcupdate  = 1'b0;
			regwrite	 = 1'b0;
			memwrite  = 1'b0;
			irwrite	 = 1'b0;
			resultsrc = 2'b00;
			adrsrc 	 = 1'b0;
			state 	 = 4'b1000;
		end else if (S9) begin
			alusrca   = 2'b01;
			alusrcb   = 2'b10;
			ALUOp     = 2'b00;
			resultsrc = 2'b00;
			pcupdate  = 1'b1;
			// set dont cares to 0
			branch	 = 1'b0;
			regwrite	 = 1'b0;
			memwrite  = 1'b0;
			irwrite	 = 1'b0;
			adrsrc 	 = 1'b0;
			state 	 = 4'b1001;
		end else begin
			alusrca   = 2'b10;
			alusrcb   = 2'b00;
			ALUOp     = 2'b01;
			resultsrc = 2'b00;
			branch    = 1'b1;
			// set dont cares to 0
			pcupdate  = 1'b0;
			regwrite	 = 1'b0;
			memwrite  = 1'b0;
			irwrite	 = 1'b0;
			adrsrc 	 = 1'b0;
			state 	 = 4'b1010;
		end
	end
	
endmodule	

module aludec(input logic opb5,
				  input logic [2:0] funct3,
				  input logic funct7b5,
				  input logic [1:0] ALUOp,
				  output logic [2:0] ALUControl);

	logic RtypeSub;
	assign RtypeSub = funct7b5 & opb5; // TRUE for R-type subtract instruction
	
	always_comb
		case(ALUOp)
			2'b00: ALUControl = 3'b000; // addition
			2'b01: ALUControl = 3'b001; // subtraction
			default: case(funct3) // R-type or I-type ALU
							3'b000: if (RtypeSub)
											ALUControl = 3'b001; // sub
									  else
											ALUControl = 3'b000; // add, addi
							3'b010: 		ALUControl = 3'b101; // slt, slti
							3'b110: 		ALUControl = 3'b011; // or, ori
							3'b111: 		ALUControl = 3'b010; // and, andi
							default: 	ALUControl = 3'b000; // ???
						endcase
		endcase
endmodule

module instrdec (input logic [6:0] op,
					  output logic [1:0] ImmSrc);
	always_comb
		case(op)
			7'b0110011: ImmSrc = 2'b00; // R-type
			7'b0010011: ImmSrc = 2'b00; // I-type ALU
			7'b0000011: ImmSrc = 2'b00; // lw
			7'b0100011: ImmSrc = 2'b01; // sw
			7'b1100011: ImmSrc = 2'b10; // beq
			7'b1101111: ImmSrc = 2'b11; // jal
			default: 	ImmSrc = 2'b00; // ???
		endcase
endmodule

module datapath(input logic clk, reset,
					 input logic [1:0] ResultSrc,				// from controller		
					 input logic [2:0] ALUControl,			// from controller
					 input logic [1:0] ALUSrcA, ALUSrcB,	// from controller
					 input logic [1:0] ImmSrc,					// from controller
					 input logic RegWrite,						// from controller
					 input logic PCWrite,						// from controller
					 input logic AdrSrc,							// from controller
					 input logic IRWrite,						// from controller
					 input logic [31:0] ReadData,				// from umem through riscv
					 output logic [31:0]	Instr,				// to controller
					 output logic Zero,							// to controller
					 output logic [31:0] WriteData,			// to umem
					 output logic [31:0] Adr);					// to umem
	
	// Internal signals
	logic [31:0] ALUResult, ALUOut, Result;				// Result = PCNext in Figure 1
	logic [31:0] PC, OldPC, ImmExt, RD1, RD2, A, SrcA, SrcB, Data;
	
	
	// Building blocks from left to right in Figure 1
	
	flopr_en #(32) PCNext_F(clk, reset, PCWrite, Result, PC);
	
	// Unified memory
	mux2 #(32) Adr_M(PC, Result, AdrSrc, Adr);
	flopr_en #(32) PC_F(clk, reset, IRWrite, PC, OldPC);
	flopr_en #(32) RD_F1(clk, reset, IRWrite, ReadData, Instr);
	flopr #(32) RD_F2(clk, reset, ReadData, Data);
	
	
	// Register File
	regfile RegisterFile(clk, RegWrite, Instr[19:15], Instr[24:20], Instr[11:7], Result, RD1, RD2);
	flopr #(32) RD1_F(clk, reset, RD1, A);		
	flopr #(32) RD2_F(clk, reset, RD2, WriteData);
	
	// Extend
	extend Extend(Instr[31:7], ImmSrc, ImmExt);
	
	mux3 #(32) SrcA_M(PC, OldPC, A, ALUSrcA, SrcA);
	mux3 #(32) SrcB_M(WriteData, ImmExt, 32'd4, ALUSrcB, SrcB);
	
	// ALU
	alu ALU(SrcA, SrcB, ALUControl, ALUResult, Zero);
	flopr #(32) ALUResult_F(clk, reset, ALUResult, ALUOut);
	
	mux3 #(32) ALUOut_M(ALUOut, Data, ALUResult, ResultSrc, Result);
	
	
endmodule					 

module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [ 4:0] a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  always_ff @(posedge clk)
    if (we3) rf[a3] <= wd3;	

  assign rd1 = (a1 != 0) ? rf[a1] : 0;
  assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule


module extend(input  logic [31:7] instr,
              input  logic [1:0]  immsrc,
              output logic [31:0] immext);
 
  always_comb
    case(immsrc) 
               // I-type 
      2'b00:   immext = {{20{instr[31]}}, instr[31:20]};
		
      2'b01:   // S-type (stores)
					immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};
               
      2'b10:   // B-type (branches)
					immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
               
      2'b11:   // J-type (jal)
					immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
					
      default: immext = 32'bx; // undefined
    endcase             
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);

  assign y = a + b;
endmodule

module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
	 
endmodule

module flopr_en #(parameter WIDTH = 8)
              (input  logic             clk, reset, en,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) 	q <= 0;
    else if (en)  q <= d;		// want to trigger on en
	 
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module umem(input logic clk, we,
				input logic [31:0] a, wd,
				output logic [31:0] rd);
				
	// Combined imem and dmem from lab 6
	
	logic [31:0] RAM[63:0];
	
	initial
      $readmemh("riscvtest.txt",RAM);

	assign rd = RAM[a[31:2]]; // word aligned
	
	always_ff @(posedge clk)
		if (we) RAM[a[31:2]] <= wd;
				
endmodule

module alu(input  logic [31:0] a, b,
           input  logic [2:0]  alucontrol,
           output logic [31:0] result,
           output logic        zero);

  logic [31:0] condinvb, sum;
  logic        v;              // overflow
  logic        isAddSub;       // true when is add or subtract operation

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                    ~alucontrol[1] & alucontrol[0];

  always_comb
    case (alucontrol)
      3'b000:  result = sum;                 // add
      3'b001:  result = sum;                 // subtract
      3'b010:  result = a & b;               // and
      3'b011:  result = a | b;       			// or
      3'b100:  result = a ^ b;       			// xor
      3'b101:  result = a < b? 32'b1: 32'b0; // slt
      3'b110:  result = a << b;					// sll
      3'b111:  result = a >>> b;       		// sra
      default: result = 32'bx;
    endcase

  assign zero = (result == 32'b0);
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
  
endmodule


