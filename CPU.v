module processor( input         clk, reset,
                  output [31:0] PC,
                  input  [31:0] instruction,
                  output        WE,
                  output [31:0] address_to_mem,
                  output [31:0] data_to_mem,
                  input  [31:0] data_from_mem
                );

    wire 		PC_Jal_J;
    wire 		PC_Jr;
    wire 		Branch;
    wire 		RegWrite;
    wire 		MemToReg;
    wire 		RegDst;
    wire 		ALUSrc;
    wire [3:0]  ALUControl;

    control_unit control_inst(instruction[31:26], 
    						  instruction[5:0], 
    						  instruction[10:6], 
    						  PC_Jal_J, 
    						  PC_Jr, 
    						  Branch, 
    						  RegWrite, 
    						  MemToReg, 
    						  WE, 
    						  RegDst, 
    						  ALUSrc, 
    						  ALUControl);

    data_path  execution_inst(clk, 
    					      reset, 
    					      instruction, 
    					      data_from_mem, 
    					      ALUControl, 
    					      PC_Jal_J, 
    					      PC_Jr, 
    					      Branch, 
    					      RegWrite, 
    					      MemToReg, 
    					      RegDst, 
    					      ALUSrc, 
    					      PC, 
    					      address_to_mem, 
    					      data_to_mem);
endmodule


//-------------------------------------------------------------------
//control unit for control path - ok
module control_unit (input [5:0] OpCode,
					 input [5:0] Funct,
					 input [4:0] Shamt,
					 output reg PC_Jal_J, PC_Jr, Branch, RegWrite, MemToReg, WE, RegDst, ALUSrc,
					 output reg [3:0] ALUControl);
always @(OpCode or Funct or Shamt) begin
	case(OpCode)
		6'b000010 :	begin		//j - ok
			{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b100000000000;
		end

		6'b000011 :	begin		//jal - ok
			{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b100100000000;
		end

		6'b000111 :	begin		//jr - ok
			{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b010000000000;
		end

		6'b000100 :	begin		//beq - ok
			{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b001000001100;
		end

		6'b100011 :	begin		//lw - ok
			{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000101100100;
		end

		6'b101011 :	begin		//sw - ok
			{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000010100100;
		end

		6'b001000 :	begin		//addi - ok
			{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000100100100;
		end

		6'b011111 :	begin		//.qb - ok
			if (Funct == 6'b010000) begin
				case (Shamt) 
					5'b00000 : begin 		//addu - ok
						{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000100010001;
					end

					5'b00100 : begin 		//addu_s - ok
						{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000100010011;
					end

					default : begin 	    //ERROR || nop - ok
						{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000000000000;
					end
				endcase					
			end else begin 		//ERROR || nop - ok
				{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000000000000;
			end
		end

		6'b000000 :	begin		//R - ok
			case (Funct) 
				6'b100000 : begin 		//add - ok
					{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000100000101;
				end

				6'b100010 : begin 		//sub - ok
					{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000100001101;
				end 

				6'b100100 : begin 		//and - ok
					{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000100000001;
				end

				6'b100101 : begin 		//or - ok
					{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000100000011;
				end

				6'b101010 : begin 		//slt - ok
					{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000100001111;
				end

				6'b000100 : begin 		//sllv - ok
					{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000100010101;
				end

				6'b000110 : begin 		//srlv - ok
					{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000100010111;
				end	
				
				6'b000111 : begin 		//srav - ok
					{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000100011001;
				end

				default : begin 		//ERROR || nop - ok
					{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000000000000;
				end	
			endcase
		end

		default : begin 	    //ERROR || nop - ok
			{PC_Jal_J, PC_Jr, Branch, RegWrite, WE, MemToReg, ALUSrc, ALUControl, RegDst} = 12'b000000000000;
		end
	endcase	
end
endmodule

//-------------------------------------------------------------------
//control unit for data path - ?
module data_path (input  		clk, reset,
				  input  [31:0] instruction,
				  input  [31:0] data_from_mem,
				  input  [3:0]  ALUControl,
				  input         PC_Jal_J, PC_Jr, Branch, RegWrite, MemToReg, RegDst, ALUSrc,
				  output [31:0] PC,
				  output [31:0] address_to_mem,
				  output [31:0] data_to_mem);
wire 		PC_Beq;
wire 		Zero;
wire [31:0] SrcA;
wire [31:0] SrcB;
wire [31:0] PC_Jal_J_v;
wire [31:0] PC_Beq_v;
wire [31:0] PCPlus4;
wire [31:0] PCNext;
wire [31:0] ExtData;
wire [31:0] ExtData4;
wire [31:0] Resault;
wire [31:0] WD3;
wire [4:0]  A3;
wire [4:0]  A3Dst;

alu 			alu32bit   (ALUControl, 			//ALUControl
							SrcA, 					//SrcA
							SrcB, 					//SrcB
							address_to_mem, 		//ALUResult
							Zero);					//Zero

file_register 	registers  (WD3, 					//WriteData_3
							instruction[25:21], 	//A1
							instruction[20:16], 	//A2
							A3, 					//A3
							RegWrite, 				//WriteEnable_3
							clk, 					//clk
							SrcA, 					//RegistrData_1
							data_to_mem);			//RegistrData_2

summator_32    PCsummator  (PC,									  	//Data_0
							32'b00000000000000000000000000000100, 	//Data_1	
							PCPlus4);								//ReturnData

summator_32   BEQsummator  (ExtData4,								//Data_0 
							PCPlus4, 								//Data_1
							PC_Beq_v);								//ReturnData

and_1x1        andBeqZero  (Zero, 			//Data_0
							Branch, 		//Data_1
							PC_Beq);    	//ReturnData

pc_jal_j  PC_Jal_J_vector  (PCPlus4[31:28],  		//PcLeft_4
							instruction[25:0],		//Imm_26 
							PC_Jal_J_v);			//Pc_Jal_J

sign_ext       signed_ext  (instruction[15:0], 		//imm_16
							ExtData);				//imm_32

left_shift_2  	     ls_2  (ExtData, 				//InputNum
							ExtData4);				//OutputNum

mux_5_5      mux_WriteDst  (RegDst,					//Control
							instruction[20:16],		//Data_0
							instruction[15:11],		//Data_1
							A3Dst);					//ReturnData

mux_5_5 mux_WriteJalOrDst  (PC_Jal_J,				//Control
							A3Dst,					//Data_0
							5'b11111,				//Data_1
							A3);					//ReturnData

mux_32_32 mux_WriteDataReg (PC_Jal_J,				//Control
							Resault,				//Data_0
							PCPlus4,				//Data_1
							WD3);					//ReturnData

mux_32_32      mux_ALUSrc  (ALUSrc,					//Control
							data_to_mem,			//Data_0
							ExtData,				//Data_1
							SrcB);					//ReturnData

mux_32_32    mux_MemToReg  (MemToReg,				//Control
							address_to_mem,			//Data_0
							data_from_mem,			//Data_1
							Resault);				//ReturnData

mux_PC		mux_PCounter   (PC_Jr,					//jr
							PC_Jal_J,				//jal_j
							PC_Beq,					//beq
							SrcA,					//jr_pc
							PC_Jal_J_v,				//jal_j_pc
							PC_Beq_v,				//beq_pc
							PCPlus4,				//pointer_pc
							PCNext);				//Ret_pc

PCounter   CounterControl  (PCNext,					//newPC
							clk,					//clk
							reset,					//reset
							PC);					//PC

endmodule

//-------------------------------------------------------------------
//alu controller - ok
module alu (input 	   [3:0]  ALUControl,
            input 	   [31:0] SrcA, SrcB,
            output reg [31:0] ALUResult,
            output reg 		  Zero );

	//SrcA == s
	//SrcA == t

    always @(ALUControl or SrcA or SrcB) begin
    	case(ALUControl) 
    		4'b0010 : begin 					//ADD - ok
    			ALUResult = SrcA + SrcB;
    		end
    		
    		4'b0110 : begin 					//SUB - ok
    			ALUResult = SrcA - SrcB;
    		end
    		
    		4'b0000 : begin 					//AND - ok
    			ALUResult = SrcA & SrcB;
    		end
    		
			4'b0001 : begin 					//OR - ok
    			ALUResult = SrcA | SrcB;
    		end
    		
    		4'b0111 : begin 					//SLT - ok
    			if (SrcA < SrcB) begin
    				ALUResult = 32'b00000000000000000000000000000001;
    			end else begin
    				ALUResult = 32'b00000000000000000000000000000000;
    			end
    		end
    		
    		4'b1000 : begin 					//ADDU.QB - ok
    			ALUResult[7:0]   = SrcA[7:0]   + SrcB[7:0];
    			ALUResult[15:8]  = SrcA[15:8]  + SrcB[15:8];
    			ALUResult[23:16] = SrcA[23:16] + SrcB[23:16];
    			ALUResult[31:24] = SrcA[31:24] + SrcB[31:24];
    		end

    		4'b1001 : begin 					//ADDU_S.QB - ok
				ALUResult[7:0] = SrcA[6:0] + SrcB[6:0];
				if ((ALUResult[7] && (SrcA[7] || SrcB[7])) || (!ALUResult[7] && SrcA[7] && SrcB[7])) begin
					ALUResult[7:0] = 8'b11111111;
				end else begin
					ALUResult[7] = ALUResult[7] + SrcA[7] + SrcB[7];
				end

				ALUResult[15:8] = SrcA[14:8] + SrcB[14:8];
				if ((ALUResult[15] && (SrcA[15] || SrcB[15])) || (!ALUResult[15] && SrcA[15] && SrcB[15])) begin
					ALUResult[15:8] = 8'b11111111;
				end else begin
					ALUResult[15] = ALUResult[15] + SrcA[15] + SrcB[15];
				end

				ALUResult[23:16] = SrcA[22:16] + SrcB[22:16];
				if ((ALUResult[23] && (SrcA[23] || SrcB[23])) || (!ALUResult[23] && SrcA[23] && SrcB[23])) begin
					ALUResult[23:16] = 8'b11111111;
				end else begin
					ALUResult[23] = ALUResult[23] + SrcA[23] + SrcB[23];
				end

				ALUResult[31:24] = SrcA[31:24] + SrcB[31:24];
				if ((ALUResult[31] && (SrcA[31] || SrcB[31])) || (!ALUResult[31] && SrcA[31] && SrcB[31])) begin
					ALUResult[31:24] = 8'b11111111;
				end else begin
					ALUResult[31] = ALUResult[31] + SrcA[31] + SrcB[31];
				end
    		end

    		4'b1010 : begin 					//SLLV - ok
    			ALUResult = SrcB << (SrcA[4:0]);	
    		end

    		4'b1011 : begin 					//SRLV - ok
				ALUResult = SrcB >> (SrcA[4:0]);
    		end

    		4'b1100 : begin 					//SRAV - ok
    			case(SrcA[4:0]) 
    			5'b00000 : ALUResult = SrcB;
    			5'b00001 : ALUResult = {{1{SrcB[31]}},  SrcB[31:1]};
    			5'b00010 : ALUResult = {{2{SrcB[31]}},  SrcB[31:2]};
    			5'b00011 : ALUResult = {{3{SrcB[31]}},  SrcB[31:3]};
    			5'b00100 : ALUResult = {{4{SrcB[31]}},  SrcB[31:4]};
    			5'b00101 : ALUResult = {{5{SrcB[31]}},  SrcB[31:5]};
    			5'b00110 : ALUResult = {{6{SrcB[31]}},  SrcB[31:6]};
    			5'b00111 : ALUResult = {{7{SrcB[31]}},  SrcB[31:7]};
    			5'b01000 : ALUResult = {{8{SrcB[31]}},  SrcB[31:8]};
    			5'b01001 : ALUResult = {{9{SrcB[31]}},  SrcB[31:9]};
    			5'b01010 : ALUResult = {{10{SrcB[31]}}, SrcB[31:10]};
    			5'b01011 : ALUResult = {{11{SrcB[31]}}, SrcB[31:11]};
    			5'b01100 : ALUResult = {{12{SrcB[31]}}, SrcB[31:12]};
    			5'b01101 : ALUResult = {{13{SrcB[31]}}, SrcB[31:13]};
    			5'b01110 : ALUResult = {{14{SrcB[31]}}, SrcB[31:14]};
    			5'b01111 : ALUResult = {{15{SrcB[31]}}, SrcB[31:15]};
    			5'b10000 : ALUResult = {{16{SrcB[31]}}, SrcB[31:16]};
    			5'b10001 : ALUResult = {{17{SrcB[31]}}, SrcB[31:17]};
    			5'b10010 : ALUResult = {{18{SrcB[31]}}, SrcB[31:18]};
    			5'b10011 : ALUResult = {{19{SrcB[31]}}, SrcB[31:19]};
    			5'b10100 : ALUResult = {{20{SrcB[31]}}, SrcB[31:20]};
    			5'b10101 : ALUResult = {{21{SrcB[31]}}, SrcB[31:21]};
    			5'b10110 : ALUResult = {{22{SrcB[31]}}, SrcB[31:22]};
    			5'b10111 : ALUResult = {{23{SrcB[31]}}, SrcB[31:23]};
    			5'b11000 : ALUResult = {{24{SrcB[31]}}, SrcB[31:24]};
    			5'b11001 : ALUResult = {{25{SrcB[31]}}, SrcB[31:25]};
    			5'b11010 : ALUResult = {{26{SrcB[31]}}, SrcB[31:26]};
    			5'b11011 : ALUResult = {{27{SrcB[31]}}, SrcB[31:27]};
    			5'b11100 : ALUResult = {{28{SrcB[31]}}, SrcB[31:28]};
    			5'b11101 : ALUResult = {{29{SrcB[31]}}, SrcB[31:29]};
    			5'b11110 : ALUResult = {{30{SrcB[31]}}, SrcB[31:30]};
    			5'b11111 : ALUResult = {{31{SrcB[31]}}, SrcB[31]};
    			default  : ALUResult = {{31{SrcB[31]}}, SrcB[31]};
    			endcase
    		end

    		default : begin
    			ALUResult[31:0] = 32'b00000000000000000000000000000000;
    		end
    	endcase
    end

    always @(ALUResult) begin
    	if (ALUResult == 0) begin
    		Zero = 1'b1;
    	end else begin
    		Zero = 1'b0;
    	end
    end
endmodule

//-------------------------------------------------------------------
//file register - ok
module file_register (input 	 [31:0] WriteData_3,
					  input 	 [4:0]  A1, A2, A3,
					  input 			WriteEnable_3, clk,		
					  output reg [31:0] RegistrData_1, RegistrData_2);

	reg [31:0] register[31:0];
	
	initial begin
		register[0]  = 32'h00000000;		// $0  - zero
		register[1]  = 32'h00000000;		// $1  - at
		register[2]  = 32'h00000000;		// $2  - v0 (function return)	
		register[3]  = 32'h00000000;		// $3  - v1 (function return)
		register[4]  = 32'h00000000;		// $4  - a0 (function arg)
		register[5]  = 32'h00000000;		// $5  - a1 (function arg)
		register[6]  = 32'h00000000;		// $6  - a2 (function arg)
		register[7]  = 32'h00000000;		// $7  - a3 (function arg)
		register[8]  = 32'h00000000;		// $8  - t0	(tmp)
		register[9]  = 32'h00000000;		// $9  - t1	(tmp)
		register[10] = 32'h00000000;		// $10 - t2	(tmp)
		register[11] = 32'h00000000;		// $11 - t3	(tmp)
		register[12] = 32'h00000000;		// $12 - t4	(tmp)
		register[13] = 32'h00000000;		// $13 - t5	(tmp)
		register[14] = 32'h00000000;		// $14 - t6	(tmp)
		register[15] = 32'h00000000;		// $15 - t7	(tmp)
		register[16] = 32'h00000000;		// $16 - s0 (saved tmp)
		register[17] = 32'h00000000;		// $17 - s1 (saved tmp)
		register[18] = 32'h00000000;		// $18 - s2 (saved tmp)
		register[19] = 32'h00000000;		// $19 - s3 (saved tmp)
		register[20] = 32'h00000000;		// $20 - s4 (saved tmp)
		register[21] = 32'h00000000;		// $21 - s5 (saved tmp)
		register[22] = 32'h00000000;		// $22 - s6 (saved tmp)
		register[23] = 32'h00000000;		// $23 - s7 (saved tmp)
		register[24] = 32'h00000000;		// $24 - t8	(tmp)
		register[25] = 32'h00000000;		// $25 - t9	(tmp)
		register[26] = 32'h00000000;		// $26 - k0 (reserved for kernel)
		register[27] = 32'h00000000;		// $27 - k1 (reserved for kernel)
		register[28] = 32'h00000000;		// $28 - gp (global pointer)
		register[29] = 32'h00004000;		// $29 - sp (stack pointer)
		register[30] = 32'h00000000;		// $30 - fp (frame pointer)
		register[31] = 32'h00000000;		// $31 - ra (return address)
	end

	always @(A1 or A2) begin
		RegistrData_1 = register[A1];
		RegistrData_2 = register[A2];
	end

	always @(posedge clk) begin
		if (WriteEnable_3 && A3 != 0) begin
			register[A3] = WriteData_3;
		end
	end
endmodule

//-------------------------------------------------------------------
//summator 32bit - ok
module summator_32 (input 	[31:0] Data_0, Data_1,
					output  [31:0] ReturnData);

	assign ReturnData = Data_1 + Data_0;
endmodule

//-------------------------------------------------------------------
//and 1bit & 1bit - ok
module and_1x1 (input 	   Data_0, Data_1, 
				output reg ReturnData);

	always @(Data_0 or Data_1) begin
		ReturnData = Data_1 & Data_0;
	end
endmodule

//-------------------------------------------------------------------
//calculate PC for jal - ok
module pc_jal_j (input 		[3:0]  PcLeft_4,
				 input 		[25:0] Imm_26,
				 output reg [31:0] Pc_Jal_J);
	
	always @(*) begin
		Pc_Jal_J = {PcLeft_4[3:0], Imm_26[25:0], {2{1'b0}}};
	end
endmodule

//-------------------------------------------------------------------
//signed ext - ok
module sign_ext (input 		[15:0] imm_16,
				 output reg [31:0] imm_32);

always @(imm_16) begin
	if (imm_16[15] == 1) begin
		imm_32 = 32'b11111111111111110000000000000000 + imm_16;
	end else begin
		imm_32 = 32'b00000000000000000000000000000000 + imm_16;
	end
end
endmodule

//-------------------------------------------------------------------
//<<2 - ok
module left_shift_2 (input 		[31:0] InputNum,
					 output reg [31:0] OutputNum);
	
	always @(InputNum) begin
		OutputNum = {InputNum[29:0], {2'b00}};
	end
endmodule

//-------------------------------------------------------------------
//multiplexer 5bit x 2 - ok
module mux_5_5 (input 			  Control,
				input 		[4:0] Data_0, Data_1,
				output reg  [4:0] ReturnData);

	always @(Data_0 or Data_1 or Control) begin
		if (Control == 0) begin
			ReturnData = Data_0;
		end else begin
			ReturnData = Data_1;
		end
	end
endmodule

//-------------------------------------------------------------------
//multiplexer 32bit x 2 - ok
module mux_32_32 (input 			Control,
				  input 	 [31:0] Data_0, Data_1,
				  output reg [31:0] ReturnData);

	always @(Control or Data_0 or Data_1) begin
		if (Control == 0) begin
			ReturnData = Data_0;	
		end else begin
			ReturnData = Data_1;
		end
	end
endmodule

//-------------------------------------------------------------------
//multiplexer for PC - ok 
module mux_PC (input 	   		 jr, jal_j, beq,
			   input 	  [31:0] jr_pc, jal_j_pc, beq_pc, pointer_pc, 
			   output reg [31:0] ret_pc);
	always @(*) begin
		if (jr == 1) begin
			ret_pc = jr_pc;
		end else if (jal_j == 1) begin
			ret_pc = jal_j_pc;
		end else if (beq == 1) begin
			ret_pc = beq_pc;
		end else begin
			ret_pc = pointer_pc;
		end
	end
endmodule

//-------------------------------------------------------------------
//PC module - ok
module PCounter (input 		[31:0] newPC,
				 input 			   clk, reset,
				 output reg [31:0] PC);

initial begin
	PC = 32'b00000000000000000000000000000000;
end

always @(posedge clk) begin
	if (reset) begin
		PC = 32'b00000000000000000000000000000000;
	end
	else begin
		PC = newPC;
	end
end
endmodule