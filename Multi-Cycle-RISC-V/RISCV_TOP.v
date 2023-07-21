module RISCV_TOP (
//in-out port
	//General Signals
	input wire CLK,
	input wire RSTn,

	//I-Memory Signals
	output wire I_MEM_CSN,
	input wire [31:0] I_MEM_DI,//input from IM
	output reg [11:0] I_MEM_ADDR,//in byte address

	//D-Memory Signals
	output wire D_MEM_CSN,
	input wire [31:0] D_MEM_DI,
	output wire [31:0] D_MEM_DOUT,
	output wire [11:0] D_MEM_ADDR,//in word address
	output wire D_MEM_WEN,
	output wire [3:0] D_MEM_BE,

	//RegFile Signals
	output wire RF_WE,
	output wire [4:0] RF_RA1,
	output wire [4:0] RF_RA2,
	output wire [4:0] RF_WA1,
	input wire [31:0] RF_RD1,
	input wire [31:0] RF_RD2,
	output wire [31:0] RF_WD,
	output wire HALT,
	output reg [31:0] NUM_INST,
	output wire [31:0] OUTPUT_PORT
	);
//

parameter	[2:0]	START = 3'd0,
					IF = 3'd1,
	 				ID = 3'd2,
	 				EXE = 3'd3,
	 				MEM = 3'd4,
	 				WB = 3'd5;
parameter	[6:0]  	OP_R = 7'b0110011, 	//ADD OR...
	          		OP_IC= 7'b0010011, //ADDI, ANDI...
	          		OP_L = 7'b0000011, 	//LW
	          		OP_S = 7'b0100011, 	//SW
					OP_B = 7'b1100011,	//BR
	          		OP_JAL = 7'b1101111,	//JAL
	          		OP_JALR = 7'b1100111;	//JALR
parameter	[3:0]	ALU_ADD = 4'b0000,
					ALU_SUB = 4'b0001,
					ALU_SLL = 4'b0010,
					ALU_SLT = 4'b0011,
					ALU_SLTU = 4'b0100,
					ALU_XOR = 4'b0101,
					ALU_SRL = 4'b0110,
					ALU_SRA = 4'b0111,
					ALU_OR = 4'b1000,
					ALU_AND = 4'b1001,
					ALU_BEQ = 4'b1010,
					ALU_BNE = 4'b1011,
					ALU_BLT = 4'b1100,
					ALU_BGE = 4'b1101,
					ALU_BLTU = 4'b1110,
					ALU_BGEU = 4'b1111;

//states
reg [2:0] state, next_state; 

// wire definitions, assign each wire to a reg to be used in behaviral
//I-memory 
	wire 	[31:0] pc = {{20{I_MEM_ADDR[11]}}, I_MEM_ADDR[11:0]};  //sign_extened pc
	reg		[31:0] inst;
	assign I_MEM_CSN = ~RSTn;
	assign D_MEM_CSN = ~RSTn;
//D-memory
	reg		[31:0]	dmemdout,dmemdi;
	reg		[31:0]	dmemaddr;
	reg		MemWriteN,dmemwen;
	assign 	D_MEM_DOUT = dmemdout;
	assign 	D_MEM_ADDR = dmemaddr;
	assign 	D_MEM_WEN = dmemwen; 
	assign D_MEM_BE = 4'b1111;
//RegFile 
	reg 	RegWrite;
	reg 	[4:0] 	rfra1,rfra2,rfwa1;
	reg 	[31:0]	rfrd1,rfrd2,rfwd;
	reg 	rfwe;
	reg 	MemToReg;
	assign	RF_WE = rfwe;
	assign 	RF_RA1 = rfra1;
	assign 	RF_RA2 = rfra2;
	assign 	RF_WA1 = rfwa1;
	assign 	RF_WD = rfwd;
	assign OUTPUT_PORT = rfwd;
//immediate field
	reg [31:0] imm;

//control signals 
	reg branch;
	reg jump;
	reg	 bcond;
	reg [6:0]opcode;
	reg [2:0]func3;
	reg ALU_src;
	reg [3:0] ALU_OP;
	reg [31:0] ALU_B;
	reg  [31:0] ALU_RESULT;
	reg wd_src; 
	reg JALR;
	always @(*) begin 
		branch = (opcode == OP_B)? 1:0; 
		ALU_B = ALU_src? RF_RD2: imm;
		jump = (opcode == OP_JAL)? 1:0;
		JALR = (opcode == OP_JALR)? 1:0;
	end
	
//HALT 
	assign HALT = (I_MEM_DI==32'h00008067&&RF_RD1 ==32'h0000000c)? 1:0;

//Initialize
	always @(negedge CLK) begin
		if(~RSTn) begin
			next_state <= START;
			NUM_INST <= 0;
		end
	end
	always @(negedge CLK) begin
		if(RSTn) begin
			if(state==START)begin
				I_MEM_ADDR <= 12'd0;
				next_state <= ID;
			end
		end
	end

//IF: set pc correctly 
	always @(negedge CLK) begin
		if(RSTn) begin
			if(state==IF)begin
				next_state <= ID;
				//I_MEM_ADDR 
				if(jump || (bcond&&branch)) I_MEM_ADDR <= ( pc + imm ) & 12'hffc;
				else if (JALR) I_MEM_ADDR <= ((ALU_RESULT)&32'hfffffffe)&12'hffc;
				else 
				I_MEM_ADDR <= (pc+12'd4) & 12'hffc; 
				rfwe = 1'b0;
				dmemwen = 1'b1;
			end		
		end
	end
//ID
	always @(negedge CLK) begin
		if(RSTn) begin
			if(state==ID)begin
				// store instruction 
				inst <= I_MEM_DI;
				//decode instruction tyep 
				opcode <= I_MEM_DI[6:0];
				//func3 <= I_MEM_DI[14:12];
				rfra1 <= I_MEM_DI [19:15];
				rfra2 <= I_MEM_DI [24:20];
				rfwa1 <= I_MEM_DI [11:7];

				//construct immediate field from instruction 
				case (I_MEM_DI[6:0]) 
					OP_IC: 	imm <= {{21{I_MEM_DI[31]}}, I_MEM_DI[30:20]};
					OP_L:	imm <= {{21{I_MEM_DI[31]}}, I_MEM_DI[30:20]};
					OP_S:	imm <= {{21{I_MEM_DI[31]}}, I_MEM_DI[30:25], I_MEM_DI[11:7]};
					OP_B: 	imm <= {{20{I_MEM_DI[31]}}, I_MEM_DI[7], I_MEM_DI[30:25], I_MEM_DI[11:8],1'b0};
					OP_JAL:	imm <= {{12{I_MEM_DI[31]}}, I_MEM_DI[19:12], I_MEM_DI[20], I_MEM_DI[30:21],1'b0};
					OP_JALR:	imm <= {{12{I_MEM_DI[31]}}, I_MEM_DI[19:12], I_MEM_DI[20], I_MEM_DI[30:21],1'b0};
				endcase
				//choose ALU_B
				if(I_MEM_DI[6:0] == OP_R||I_MEM_DI[6:0] == OP_B) begin
					ALU_src <= 1'b1;
				end
				else if(I_MEM_DI[6:0] == OP_L||I_MEM_DI[6:0] == OP_S||I_MEM_DI[6:0] == OP_JALR||I_MEM_DI[6:0]== OP_IC) begin
					ALU_src <= 1'b0;
				end
				//ALU_OP 
				if(I_MEM_DI[6:0] == OP_L||I_MEM_DI[6:0] == OP_S||opcode==OP_JALR) ALU_OP <= ALU_ADD;
				else if (I_MEM_DI[6:0] == OP_R) begin
					case(I_MEM_DI[14:12])
						3'b000: ALU_OP <= (I_MEM_DI[30] == 0)?ALU_ADD:ALU_SUB;
						3'b001: ALU_OP <= ALU_SLL; 
						3'b010: ALU_OP <= ALU_SLT; 
						3'b011: ALU_OP <= ALU_SLTU; 
						3'b100: ALU_OP <= ALU_XOR; 
						3'b101: ALU_OP <= (I_MEM_DI[30] == 0)? ALU_SRL : ALU_SRA; 
						3'b110: ALU_OP <= ALU_OR; 
						3'b111: ALU_OP <= ALU_AND; 					
					endcase
				end
				else if (I_MEM_DI[6:0] == OP_IC) begin

					case(I_MEM_DI[14:12])
						3'b000: ALU_OP <= ALU_ADD;
						3'b001: ALU_OP <= ALU_SLL; 
						3'b010: ALU_OP <= ALU_SLT; 
						3'b011: ALU_OP <= ALU_SLTU; 
						3'b100: ALU_OP <= ALU_XOR; 
						3'b101: ALU_OP <= (I_MEM_DI[30] == 0)? ALU_SRL : ALU_SRA; 
						3'b110: ALU_OP <= ALU_OR; 
						3'b111: ALU_OP <= ALU_AND; 					
					endcase
				end
				//B
				else if (I_MEM_DI[6:0] == OP_B) begin
					case(I_MEM_DI[14:12])
						3'b000: ALU_OP <= ALU_BEQ; 
						3'b001: ALU_OP <= ALU_BNE; 
						3'b100: ALU_OP <= ALU_BLT; 
						3'b101: ALU_OP <= ALU_BGE; 
						3'b110: ALU_OP <= ALU_BLTU; 
						3'b111: ALU_OP <= ALU_BGEU; 					
					endcase				
				end

				//control signals and next state 		
				if (I_MEM_DI[6:0] == OP_R||I_MEM_DI[6:0] == OP_IC||I_MEM_DI[6:0] == OP_JAL||I_MEM_DI[6:0] == OP_L||I_MEM_DI[6:0] == OP_JALR)	RegWrite <=1'b1;
				else	RegWrite <= 1'b0;
				if (I_MEM_DI[6:0] == OP_S)	MemWriteN <=1'b0;
				else	MemWriteN <= 1'b1;
				if (I_MEM_DI[6:0] == OP_L)	MemToReg <=1'b1;
				else	MemToReg <=1'b0;	

				//JAL exit
				if (I_MEM_DI[6:0] == OP_JAL) begin	
					next_state <= IF;
					rfwe <= 1'b1;
					rfwd <= pc + 4;
					NUM_INST <= NUM_INST +1;
				end
				else next_state <= EXE;			

			end			
		end
	end
//EXE
	always @(negedge CLK) begin
		if(RSTn) begin
			if(state==EXE)begin
				rfrd1 <= RF_RD1;
				rfrd2 <= RF_RD2;
				//ALU
				case(ALU_OP)
					ALU_ADD: ALU_RESULT <= RF_RD1 + ALU_B ;//RF_RD1 + ((ALU_src)? RF_RD2 : imm);
					ALU_SUB: ALU_RESULT <= RF_RD1 - ALU_B;
					ALU_SLL: ALU_RESULT <= RF_RD1 << ALU_B; 
					ALU_SLT: begin
						if(RF_RD1[31]==0 && ALU_B[31]==0)begin
							ALU_RESULT <= (RF_RD1 < ALU_B) ? 1:0;
						end
						else if (RF_RD1[31]==1 && ALU_B[31]==1) begin
							ALU_RESULT <= (RF_RD1 < ALU_B) ? 0:1;
						end
						else if (RF_RD1[31]==0 && ALU_B[31]==1) begin
							ALU_RESULT <= 0;
						end
						else if (RF_RD1[31]==1 && ALU_B[31]==0) begin
							ALU_RESULT <= 1;
						end				
					end
					ALU_SLTU: ALU_RESULT <= (RF_RD1 < ALU_B) ? 1:0;
					ALU_XOR: ALU_RESULT <= RF_RD1 ^ ALU_B;
					ALU_SRL: ALU_RESULT <= RF_RD1 >> ALU_B;
					ALU_SRA: ALU_RESULT <= RF_RD1 >>> ALU_B;
					ALU_OR: ALU_RESULT <= RF_RD1 | ALU_B;
					ALU_AND: ALU_RESULT <= RF_RD1 & ALU_B;
				endcase
				//B type ALU
				if(opcode == OP_B) begin
					case(ALU_OP)
						ALU_BEQ: bcond <= (RF_RD1==ALU_B) ? 1:0;
						ALU_BNE: bcond <= (RF_RD1==ALU_B) ? 0:1;
						ALU_BLT: begin
							if(RF_RD1[31]==0 && ALU_B[31]==0)begin
								bcond <= (RF_RD1 < ALU_B) ? 1:0;
								rfwd <= (RF_RD1 < ALU_B) ? 1:0;
							end
							else if (RF_RD1[31]==1 && ALU_B[31]==1) begin
								bcond <= (RF_RD1 < ALU_B) ? 0:1;
								rfwd <= (RF_RD1 < ALU_B) ? 0:1;
							end
							else if (RF_RD1[31]==0 && ALU_B[31]==1) begin
								bcond <= 0;
								rfwd<= 0;
							end
							else if (RF_RD1[31]==1 && ALU_B[31]==0) begin
								bcond <= 1;
								rfwd <= 1;
							end				
						end
						ALU_BGE: begin
							if(RF_RD1[31]==0 && ALU_B[31]==0)begin
								bcond <= (RF_RD1 < ALU_B) ? 0:1;
								rfwd <=  (RF_RD1 < ALU_B) ? 0:1;
							end
							else if (RF_RD1[31]==1 && ALU_B[31]==1) begin
								bcond <= (RF_RD1 < ALU_B) ? 1:0;
								rfwd <= (RF_RD1 < ALU_B) ? 1:0;
							end
							else if (RF_RD1[31]==0 && ALU_B[31]==1) begin
								bcond <= 1;
								rfwd <= 1;
							end
							else if (RF_RD1[31]==1 && ALU_B[31]==0) begin
								bcond <= 0;
								rfwd <= 0;
							end				
						end
						ALU_BLTU: begin
							bcond <= (RF_RD1 < ALU_B) ? 1:0;
							rfwd <= (RF_RD1 < ALU_B) ? 1:0;
						end
						ALU_BGEU:begin
							bcond <= (RF_RD1 < ALU_B) ? 0:1;
							rfwd <= (RF_RD1 < ALU_B) ? 0:1;
						end
					endcase 					
					next_state <= IF;
					NUM_INST<=NUM_INST+1;
					
				end
				//JALR exit
				else if(opcode == OP_JALR) begin
					next_state <= IF;
					rfwe <= RegWrite; 
					rfwd <= pc+4;
					dmemwen <= MemWriteN;
					NUM_INST <= NUM_INST +1;
				end
				else if(opcode == OP_R||opcode == OP_IC) begin
					next_state <= WB;
				end
				else if(opcode == OP_L||opcode == OP_S) begin
					next_state <= MEM;
					dmemaddr <= (RF_RD1 + ALU_B) & 16'hfffc;
				end				


			end
		end
	end
//MEM
	always @(negedge CLK) begin
		if(state==MEM)begin

			if(opcode == OP_L) begin
				dmemdi <= D_MEM_DI; 
				next_state <= WB; 
			end	
			//SW exit
			if(opcode == OP_S) begin
				rfwe <= RegWrite;
				dmemwen <= MemWriteN;
				dmemdout <= rfrd2; 
				rfwd <= ALU_RESULT; 	//only for output port, no effect
				NUM_INST = NUM_INST + 1;
				next_state <= IF;

			end	

		end			
	end
//WB
	always @(negedge CLK) begin
		if (RSTn) begin
			//L,R,Ic exit
			if(state==WB)begin
				next_state <= IF; 
				rfwe <= RegWrite;
				if(MemToReg) rfwd <=  dmemdi;
				else 	rfwd <= ALU_RESULT;
				NUM_INST <= NUM_INST + 1;
			end	
		end		
	end	

//state update
	always @(posedge CLK ) begin
		state <= next_state;
	end
endmodule //
