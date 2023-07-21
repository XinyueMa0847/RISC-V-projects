module RISCV_TOP(
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
	output wire HALT,                   // if set, terminate program
	output reg [31:0] NUM_INST,         // number of instruction completed
	output wire [31:0] OUTPUT_PORT      // equal RF_WD this port is used for test
	);

///////////////////////////////PARAMATERS////////////////////////////////////
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
///////////////////////////////PARAMATERS////////////////////////////////////

///////////////////////////////REGISTERS/////////////////////////////////////
// wire definitions, assign each wire to a reg to be used in behaviral
// Chip select 
	assign I_MEM_CSN = ~RSTn;
	assign D_MEM_CSN = ~RSTn; 
//D-memory
	reg		[31:0]	dmemdout,dmemdi;
	reg		[31:0]	dmemaddr;
	reg		MemWriteN,dmemwen;
	assign 	D_MEM_DOUT = dmemdout;
	assign 	D_MEM_ADDR = dmemaddr & 16'h3fff;
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

/* DATA HARZARDS */ 
	reg id_ex_in_DH_raw1;
	reg id_ex_in_DH_raw2;
	reg id_mem_in_DH_1;
	reg id_mem_in_DH_2;
	reg id_wb_in_DH_1;
	reg id_wb_in_DH_2;

	reg id_ex_out_DH_raw1;
	reg id_ex_out_DH_raw2;
	reg id_mem_out_DH_1;
	reg id_mem_out_DH_2;
	reg id_wb_out_DH_1;
	reg id_wb_out_DH_2;	

// BRANCH Conditions
	reg bcond;
	// weather a branch was taken 
	reg branch_at_ex_mem_in,branch_at_ex_mem_out;
	reg branch_at_mem_wb_in,branch_at_mem_wb_out;

/* Pipeline Registers */ 
	// the instruction I_MEM_DI is forwarded to every stage of that instruction  

	/* IF stage registers*/ 
	reg if_id_start;
	reg if_id_end;
	// INPUTS
	reg [31:0] if_id_in_I_MEM_DI; 	//input from insturction memeory
	reg [11:0] if_id_in_I_MEM_ADDR;	//instruction address
	// to ID 
	reg [31:0]	if_id_out_I_MEM_DI;
	reg [11:0]	if_id_out_I_MEM_ADDR;	

	/* ID stage registers */
	reg id_exe_start;
	reg id_exe_end;

	reg [31:0] 	id_ex_in_I_MEM_DI;
	reg [11:0]	id_ex_in_I_MEM_ADDR; 
	reg [31:0]	id_ex_in_ALU_A, id_ex_in_ALU_B;
	reg [31:0] 	id_ex_in_imm;
	reg [4:0]	id_ex_in_RF_WA1; //register to write need to passed to every stage
	reg [31:0]	id_ex_in_RF_RD1; 
	reg [31:0]	id_ex_in_RF_RD2; 
	// CONTROLS 
	reg 		id_ex_in_RegWrite, id_ex_in_ALU_src,id_ex_in_MemWriteN;
	reg [3:0]	id_ex_in_ALU_OP; 
	// to EXE
	reg [3:0] 	id_ex_out_ALU_OP;
	reg [31:0]	id_ex_out_I_MEM_DI;
	reg	[11:0]	id_ex_out_I_MEM_ADDR;
	reg [31:0]	id_ex_out_ALU_A,id_ex_out_ALU_B;
	reg [4:0]	id_ex_out_RF_WA1;
	reg [31:0]	id_ex_out_ALU_Result;
	reg [31:0] id_ex_out_RF_RD1;
	reg [31:0] id_ex_out_RF_RD2;
	reg id_ex_out_RegWrite;
	reg id_ex_out_ALU_src;
	reg id_ex_out_MemWriteN;
	reg [31:0]	id_ex_out_imm;

	/* EX stage registers */
	reg exe_mem_start;
	reg exe_mem_end;
	// INPUTS
	reg [31:0] 	ALU_A, ALU_B;
	// OUTPUTS 
	reg [31:0]	ex_mem_in_ALU_Result;
	reg [31:0]	ex_mem_in_I_MEM_DI;
	reg [11:0]	ex_mem_in_I_MEM_ADDR;
	reg	[4:0]	ex_mem_in_RF_WA1;
	reg [31:0]	ex_mem_in_RF_RD1; 
	reg [31:0]	ex_mem_in_RF_RD2; 
	// CONTROLS
	reg ex_mem_in_MemWriteN;
	reg ex_mem_in_RegWrite;
	// to MEM
	reg [31:0]	ex_mem_out_ALU_Result;
	reg [31:0]	ex_mem_out_I_MEM_DI;
	reg	[11:0]	ex_mem_out_I_MEM_ADDR;
	reg [31:0]	ex_mem_out_RF_RD1;
	reg [31:0]	ex_mem_out_RF_RD2;
	reg [3:0]	ex_mem_out_ALU_OP;
	reg [4:0]	ex_mem_out_RF_WA1;
	reg ex_mem_out_RegWrite,ex_mem_out_MemWriteN;

	// MEM stage registers 
	reg mem_wb_start;
	reg	mem_wb_end;
	//OUTPUTS 
	reg [31:0]	mem_wb_in_ALU_Result;
	reg [31:0]	mem_wb_in_I_MEM_DI;
	reg	[11:0]	mem_wb_in_I_MEM_ADDR;
	reg [4:0]	mem_wb_in_RF_WA1;
	reg [3:0]	mem_wb_in_ALU_OP;
	reg	[31:0]	mem_wb_in_D_MEM_DI;
	reg [31:0]	mem_wb_in_RF_RD1;
	reg [31:0]	mem_wb_in_RF_RD2;
	//CONTROL
	reg mem_wb_in_RegWrite;
	// to WB 
	reg [31:0]	mem_wb_out_ALU_Result;
	reg [31:0] 	mem_wb_out_I_MEM_DI;
	reg [11:0]	mem_wb_out_I_MEM_ADDR;
	reg [31:0]	mem_wb_out_D_MEM_DI;
	reg [31:0]  mem_wb_out_RF_RD1;
	reg [31:0]	mem_wb_out_RF_RD2;
	reg [4:0]	mem_wb_out_RF_WA1;
	reg	[3:0]	mem_wb_out_ALU_OP;
	reg mem_wb_out_RegWrite;

//Halt
	assign HALT = (ex_mem_out_I_MEM_DI== 32'h00008067 && ex_mem_out_ALU_Result == 32'h0000000c)?1'b1:1'b0;
///////////////////////////////REGISTERS/////////////////////////////////////


/////////////////////////////FOR DEBUGGING PURPOSES /////////////////////////
	// reg [31:0] if_num, id_num, ex_num,mem_num,wb_num; 
	// reg [31:0] cycle; 
	// initial begin
	// 	if_num <=0; 
	// 	id_num <=0;
	// 	ex_num <=0;
	// 	mem_num <=0;
	// 	wb_num <=0; 
	// 	cycle <=0;
	// end
	// always @(negedge CLK) begin 
	// 	if (RSTn) begin 
	// 		cycle =cycle+1;
	// 		if(if_id_start==1) if_num = if_num+1; 
	// 		if(id_exe_start==1) id_num = id_num+1; 
	// 		if(exe_mem_start==1) ex_num = ex_num+1; 
	// 		if(mem_wb_start==1) mem_num = mem_num+1; 
	// 		if(mem_wb_end == 1) wb_num = wb_num+1; 
			
	// 		$display("Cycle:#d", cycle);
	// 		$display("hazards: %d/%d/%d/%d", id_ex_out_DH_raw1,id_ex_out_DH_raw2,id_mem_out_DH_1 ,id_mem_out_DH_2);
	// 		$display("Instruction#%d, IF stage", if_num);
	// 		$display("Instruction#%d, ID stage", id_num); 
	// 		//if (ex_num==1) $display("Instruction 1 ALU_RESLT: %d", ex_mem_in_ALU_Result);
	// 		$display("Instruction#%d, EX stage", ex_num); 
	// 		$display("ALU_OP: 0x%0x", id_ex_out_ALU_OP);
	// 		$display("ALU_A: 0x%0x, ALU_B: 0x%0x, ALU_result:0x%0x", ALU_A, ALU_B,ex_mem_in_ALU_Result);
	// 		$display("Instruction#%d, MEM stage", mem_num);
	// 		//if (ex_num==1) $display("I1: mem_wb_in_alu_result: %d rfwd:%d", mem_wb_in_ALU_Result,rfwd); 
	// 		$display("Instruction#%d, WB stage", wb_num); 
	// 		$display("output port: 0x%0x", OUTPUT_PORT);
	// 	end		
	// end 
//////////////////////////////FOR DEBUG PURPOSES/////////////////////////////////////////


///////////////////////////////COMBINATIONAL LOGIC//////////////////////////////////////
// IF stage
	always @(*) begin
		if (RSTn) begin  
			if_id_start = 1;  		

			if_id_in_I_MEM_ADDR = I_MEM_ADDR;
			if_id_in_I_MEM_DI = I_MEM_DI;
		end
	end

// ID stage
	always @(*) begin
		if (RSTn) begin   
			id_exe_start = if_id_end;

			id_ex_in_I_MEM_DI = if_id_out_I_MEM_DI;
			id_ex_in_I_MEM_ADDR = if_id_out_I_MEM_ADDR;
			
			rfra1 = if_id_out_I_MEM_DI[19:15]; 
			rfra2 = if_id_out_I_MEM_DI[24:20];
			id_ex_in_RF_WA1  = if_id_out_I_MEM_DI[11:7];
			
		/* construct immediate field from instruction */
			case (id_ex_in_I_MEM_DI[6:0]) 
				OP_IC: 	id_ex_in_imm = {{21{if_id_out_I_MEM_DI[31]}}, if_id_out_I_MEM_DI[30:20]};
				OP_L:	id_ex_in_imm = {{21{if_id_out_I_MEM_DI[31]}}, if_id_out_I_MEM_DI[30:20]};
				OP_S:	id_ex_in_imm = {{21{if_id_out_I_MEM_DI[31]}}, if_id_out_I_MEM_DI[30:25], if_id_out_I_MEM_DI[11:7]};
				OP_B: 	id_ex_in_imm = {{20{if_id_out_I_MEM_DI[31]}}, if_id_out_I_MEM_DI[7], if_id_out_I_MEM_DI[30:25], if_id_out_I_MEM_DI[11:8],1'b0};
				default id_ex_in_imm =0;
			endcase		
			// ALU_src
			if (if_id_out_I_MEM_DI[6:0] == OP_IC ||if_id_out_I_MEM_DI[6:0] == OP_L || if_id_out_I_MEM_DI[6:0] == OP_S ) begin
				id_ex_in_ALU_src = 1;
			end
			else id_ex_in_ALU_src= 0;	

			// ALU_OP 
			case(if_id_out_I_MEM_DI[6:0])
				OP_R: begin 
					case(if_id_out_I_MEM_DI[14:12]) 
						3'b000: id_ex_in_ALU_OP = (if_id_out_I_MEM_DI[31:25]==7'b0000000)? ALU_ADD:ALU_SUB;
						3'b001: id_ex_in_ALU_OP = ALU_SLL;
						3'b010:	id_ex_in_ALU_OP = ALU_SLT;
						3'b011:	id_ex_in_ALU_OP = ALU_SLTU;
						3'b100: id_ex_in_ALU_OP = ALU_XOR;
						3'b101:	id_ex_in_ALU_OP = (if_id_out_I_MEM_DI[31:25]==7'b0000000)? ALU_SRL:ALU_SRA;
						3'b110:	id_ex_in_ALU_OP = ALU_OR;
						3'b111: id_ex_in_ALU_OP = ALU_AND;
					endcase
				end 
				OP_IC: begin 
					case(if_id_out_I_MEM_DI[14:12]) 
						3'b000: id_ex_in_ALU_OP = ALU_ADD;
						3'b001: id_ex_in_ALU_OP = ALU_SLL;
						3'b010:	id_ex_in_ALU_OP = ALU_SLT;
						3'b011:	id_ex_in_ALU_OP = ALU_SLTU;
						3'b100: id_ex_in_ALU_OP = ALU_XOR;
						3'b101:	id_ex_in_ALU_OP = (if_id_out_I_MEM_DI[31:25]==7'b0000000)? ALU_SRL:ALU_SRA;
						3'b110:	id_ex_in_ALU_OP = ALU_OR;
						3'b111: id_ex_in_ALU_OP = ALU_AND;
					endcase
				end 
				OP_L: begin 
					id_ex_in_ALU_OP = ALU_ADD;
				end 
				OP_S: begin 
					id_ex_in_ALU_OP = ALU_ADD;	
				end 
				OP_B: begin 
					case(if_id_out_I_MEM_DI[14:12])
						3'b000: id_ex_in_ALU_OP = ALU_BEQ;
						3'b001: id_ex_in_ALU_OP = ALU_BNE;
						3'b100: id_ex_in_ALU_OP = ALU_BLT;
						3'b101:	id_ex_in_ALU_OP = ALU_BGE;
						3'b110:	id_ex_in_ALU_OP = ALU_BLTU;
						3'b111: id_ex_in_ALU_OP = ALU_BGEU;
					endcase
				end
			endcase

			// Set ALU operands 
			id_ex_in_ALU_A = RF_RD1;
			id_ex_in_ALU_B =  id_ex_in_ALU_src? id_ex_in_imm: RF_RD2; 

			// save rs1 and rs2 for later use
			id_ex_in_RF_RD2 = RF_RD2; 
			id_ex_in_RF_RD1 = RF_RD1;

			// RegWrite
			if (if_id_out_I_MEM_DI[6:0] == OP_R ||if_id_out_I_MEM_DI[6:0] == OP_L || if_id_out_I_MEM_DI[6:0]== OP_IC|| if_id_out_I_MEM_DI[6:0] == OP_JAL || if_id_out_I_MEM_DI[6:0] == OP_JALR ) begin
				id_ex_in_RegWrite = 1;
			end
			else id_ex_in_RegWrite= 0;
			// MemWrite 
			id_ex_in_MemWriteN = (if_id_out_I_MEM_DI[6:0] == OP_S) ? 0:1;

	/* Hazard Detection */  
			// hazard in ex state 
			// read after write rs1 
			if (id_ex_out_RegWrite && if_id_out_I_MEM_DI[19:15]==id_ex_out_I_MEM_DI[11:7]) begin 
				id_ex_in_DH_raw1 = 1;
			end
			else id_ex_in_DH_raw1 = 0;
			// read after write rs2
			if (id_ex_out_RegWrite && if_id_out_I_MEM_DI[24:20]==id_ex_out_I_MEM_DI[11:7]) begin 
				id_ex_in_DH_raw2 = 1;
			end
			else id_ex_in_DH_raw2 = 0;			
			
			// hazard in mem state 
			// read after write rs1 
			if (ex_mem_out_RegWrite && if_id_out_I_MEM_DI[19:15]==ex_mem_out_I_MEM_DI[11:7]) begin 
				id_mem_in_DH_1 = 1;
			end
			else id_mem_in_DH_1 = 0;

			// read after write rs2 
			if (ex_mem_out_RegWrite && if_id_out_I_MEM_DI[24:20]==ex_mem_out_I_MEM_DI[11:7]) begin 
				id_mem_in_DH_2 = 1;
			end
			else id_mem_in_DH_2 = 0;	
		end
	end
	
	// EXE
	always @(*) begin
		if (RSTn) begin  
			exe_mem_start = id_exe_end; 

			// get ID pipeline registers and feed to EXE pipeline 
			ex_mem_in_I_MEM_DI  = id_ex_out_I_MEM_DI; 
			ex_mem_in_I_MEM_ADDR = id_ex_out_I_MEM_ADDR; 
			ex_mem_in_RF_WA1 = id_ex_out_RF_WA1;

			ex_mem_in_RegWrite = id_ex_out_RegWrite; 
			ex_mem_in_MemWriteN = id_ex_out_MemWriteN; 

			// save rs1, rs2
			ex_mem_in_RF_RD2 = id_ex_out_RF_RD2; 
			ex_mem_in_RF_RD1 = id_ex_out_RF_RD1; 

			// dealing with harzards 
			// 1.ALU_A: choose the ALU result of the previous instruction or data memory if load instruction
			if(id_ex_out_DH_raw1) begin 
				ALU_A = ex_mem_out_I_MEM_DI[6:0] == OP_L? D_MEM_DI:ex_mem_out_ALU_Result;	
			end
			else if (id_mem_out_DH_1) begin 
				ALU_A = mem_wb_out_I_MEM_DI[6:0] == OP_L? mem_wb_out_D_MEM_DI:mem_wb_out_ALU_Result;		
			end
			else ALU_A = id_ex_out_ALU_A;

			// 1.ALU_B : choose alu_b is rs2, choose previous alu result or data memory if load instruction
			if(id_ex_out_DH_raw2) begin 
				if (id_ex_out_ALU_src) ALU_B = id_ex_out_imm; 
				else ALU_B = (ex_mem_out_I_MEM_DI[6:0] == OP_L) ? D_MEM_DI: ex_mem_out_ALU_Result;
			end
			else if (id_mem_out_DH_2) begin 
				ALU_B = mem_wb_out_I_MEM_DI[6:0] == OP_L? mem_wb_out_D_MEM_DI:mem_wb_out_ALU_Result;		
			end
			else ALU_B = id_ex_out_ALU_B;		
			
			// rs2: rs2 might not have been updated, get the alu_result instead, or data memory if load instruction  
			if (id_ex_out_DH_raw2) begin 
				ex_mem_in_RF_RD2 = (ex_mem_out_I_MEM_DI[6:0]==OP_L)? D_MEM_DI:ex_mem_out_ALU_Result;
			end
			else if (id_mem_out_DH_2 )begin 
				ex_mem_in_RF_RD2 = (mem_wb_out_I_MEM_DI[6:0]==OP_L)? mem_wb_out_D_MEM_DI:mem_wb_out_ALU_Result; 
			end
			else ex_mem_in_RF_RD2 =  id_ex_out_RF_RD2;



			// ALU unit 
			case (id_ex_out_ALU_OP)
				ALU_ADD: ex_mem_in_ALU_Result = ALU_A + ALU_B;
				ALU_SUB: ex_mem_in_ALU_Result = ALU_A - ALU_B;
				ALU_SLL: ex_mem_in_ALU_Result = ALU_A << ALU_B[4:0];
				ALU_SLT: begin 
					if(ALU_A[31]==0 && ALU_B[31]==0)begin
						ex_mem_in_ALU_Result = (ALU_A < ALU_B) ? 1:0;
					end
					else if (ALU_A[31]==1 && ALU_B[31]==1) begin
						ex_mem_in_ALU_Result = (ALU_A < ALU_B) ? 0:1;
					end
					else if (ALU_A[31]==0 && ALU_B[31]==1) begin
						ex_mem_in_ALU_Result = 0;
					end
					else if (ALU_A[31]==1 && ALU_B[31]==0) begin
						ex_mem_in_ALU_Result = 1;
					end						
				end
				ALU_SLTU: 	ex_mem_in_ALU_Result = (ALU_A<ALU_B)? 1:0;
				ALU_XOR:	ex_mem_in_ALU_Result = ALU_A ^ ALU_B;
				ALU_SRL:	ex_mem_in_ALU_Result = ALU_A >> ALU_B[4:0];
				ALU_SRA:	ex_mem_in_ALU_Result = ALU_A >>> ALU_B;
				ALU_OR:		ex_mem_in_ALU_Result = ALU_A | ALU_B;
				ALU_AND:	ex_mem_in_ALU_Result = ALU_A & ALU_B;

				// ALU for branching
				ALU_BEQ: bcond = (ALU_A == ALU_B)? 1:0;  
				ALU_BNE: bcond = (ALU_A != ALU_B)? 1:0; 
				ALU_BLT: begin 
					if(ALU_A[31]==0 && ALU_B[31]==0)begin
						bcond = (ALU_A < ALU_B) ? 1:0;
					end
					else if (ALU_A[31]==1 && ALU_B[31]==1) begin
						bcond = (ALU_A < ALU_B) ? 0:1;
					end
					else if (ALU_A[31]==0 && ALU_B[31]==1) begin
						bcond = 0;
					end
					else if (ALU_A[31]==1 && ALU_B[31]==0) begin
						bcond = 1;
					end	
				end
				ALU_BGE: begin
					if(ALU_A[31]==0 && ALU_B[31]==0)begin
						bcond = (ALU_A < ALU_B) ? 0:1;
					end
					else if (ALU_A[31]==1 && ALU_B[31]==1) begin
						bcond = (ALU_A < ALU_B) ? 1:0;
					end
					else if (ALU_A[31]==0 && ALU_B[31]==1) begin
						bcond = 1;
					end
					else if (ALU_A[31]==1 && ALU_B[31]==0) begin
						bcond = 0;
					end	
				end
				ALU_BLTU: bcond = (ALU_A < ALU_B)? 1:0; 
				ALU_BGEU: bcond = (ALU_A > ALU_B)? 1:0; 
				default: begin
						ex_mem_in_ALU_Result = 0;
						bcond = 0;
				end
			endcase
			 
			// whether the branch was taken 
			branch_at_ex_mem_in = (bcond && (id_ex_out_I_MEM_DI[6:0]==OP_B)) ? 1:0;
		end
	end
	
// MEM 
	always @(*) begin
		if (RSTn) begin  
			mem_wb_start = exe_mem_end; 

			// get EXE pipeline registers and fill in MEM pipeline registers 
			mem_wb_in_ALU_Result = ex_mem_out_ALU_Result; 
			mem_wb_in_I_MEM_DI = ex_mem_out_I_MEM_DI; 
			mem_wb_in_I_MEM_ADDR = ex_mem_out_I_MEM_ADDR;
			mem_wb_in_D_MEM_DI = D_MEM_DI; 
			mem_wb_in_RegWrite = ex_mem_out_RegWrite; 
			mem_wb_in_RF_WA1 = ex_mem_out_RF_WA1; 
			//save rs1 rs2
			mem_wb_in_RF_RD2 = ex_mem_out_RF_RD2; 
			mem_wb_in_RF_RD1 = ex_mem_out_RF_RD1; 
			//save branch information 
			branch_at_mem_wb_in = branch_at_ex_mem_out; 

			// Load and Store Exit 
			if (mem_wb_in_I_MEM_DI[6:0]== OP_L || mem_wb_in_I_MEM_DI[6:0]== OP_S) begin
				dmemaddr = ex_mem_out_ALU_Result; 
				dmemwen = ex_mem_out_MemWriteN; 
				dmemdout = ex_mem_out_RF_RD2;
			end
		end		
	end

// WB
	always @(*) begin
		if (RSTn) begin  
			// Other instruction exit 
			//set write back value 
			case (mem_wb_out_I_MEM_DI[6:0])
				OP_L: 		rfwd = mem_wb_out_D_MEM_DI; 
				OP_JAL: 	rfwd = mem_wb_out_I_MEM_ADDR + 12'd4; 
				OP_B: 		rfwd = branch_at_mem_wb_out; 
				default: 	rfwd = mem_wb_out_ALU_Result;
			endcase			
			// setup write back	
			rfwa1 = mem_wb_out_I_MEM_DI[11:7]; 
			rfwe = mem_wb_out_RegWrite; 
		end
	end
///////////////////////////////COMBINATIONAL LOGIC//////////////////////////////////////

///////////////////////////////SEWQUENCIAL LOGIC//////////////////////////////////////
	initial begin
		NUM_INST <= 0;
	end
	
	//count number of instructions
	always @(negedge CLK) begin 
		if(mem_wb_end) begin 
			NUM_INST <= NUM_INST + 1;
		end
	end
// update PC, stall the pipeline if branch was taken or jumped 
	always @(posedge CLK) begin 
		if (~RSTn) begin 
			NUM_INST <= 0; 
			I_MEM_ADDR <= 0;
		end
		if (RSTn) begin
			// find next pc
			// if branch was taken, stall and move to branch address  
			if (branch_at_ex_mem_in) begin 
				I_MEM_ADDR 	<= ({{20{id_ex_out_I_MEM_ADDR}},id_ex_out_I_MEM_ADDR[11:0]} + id_ex_out_imm) & 12'hffc; 
			end
			// if JALR, stall and move to return address
			else if (id_ex_in_I_MEM_DI[6:0]==OP_JALR) begin 
				I_MEM_ADDR 	<= ((id_ex_in_ALU_A + 
					({{20{id_ex_in_I_MEM_DI}},id_ex_in_I_MEM_DI[31:20]}))&32'hfffffffe) & 12'hffc;  
			end
			// if JAL, stall and move to jump address 
			else if (if_id_in_I_MEM_DI[6:0]==OP_JAL) begin 
				I_MEM_ADDR <= ({ {20{I_MEM_ADDR[11]}} ,I_MEM_ADDR[11:0]} +
					 { {11{if_id_in_I_MEM_DI[31]}}, if_id_in_I_MEM_DI[31], if_id_in_I_MEM_DI[19:12]
					 ,if_id_in_I_MEM_DI[20],if_id_in_I_MEM_DI[30:21],1'b0}) & 12'hffc; 
			end
			//pc_step
			else I_MEM_ADDR <= (I_MEM_ADDR + 4) & 12'hffc;
		end
	end

// pipeline flow IF -> ID 
	always @(posedge CLK) begin 
		if (RSTn) begin
			// stall if branched 
			if ( id_ex_in_I_MEM_DI[6:0] == OP_JALR || branch_at_ex_mem_in) begin 
				if_id_end <= 0; 
				if_id_out_I_MEM_DI <= 0;
				if_id_out_I_MEM_ADDR <= 0;
			end
			// pipe flows otherwise 
			else begin 
				if_id_end <= if_id_start; 
				if_id_out_I_MEM_DI <= if_id_in_I_MEM_DI; 
				if_id_out_I_MEM_ADDR <= if_id_in_I_MEM_ADDR; 
			end
		end
	end

// pipeline flow ID -> EXE
	always @(posedge CLK) begin 
		if (RSTn) begin
			// if branched, stall the pipe line, reset data hazard as well 
			if (branch_at_ex_mem_in) begin 
				id_exe_end <= 0; 
				id_ex_out_ALU_OP 	<= 0; 
				id_ex_out_imm 		<= 0; 
				id_ex_out_I_MEM_DI 	<= 0;
				id_ex_out_I_MEM_ADDR <= 0; 
				id_ex_out_RegWrite 	<= 0;
				id_ex_out_RF_WA1 	<= 0;
				id_ex_out_ALU_src 	<= 0;
				id_ex_out_ALU_A 	<=0; 
				id_ex_out_ALU_B 	<=0;
				
				id_ex_out_DH_raw1 	<=0; 				
				id_mem_out_DH_1 	<=0; 
				id_wb_out_DH_1 		<=0; 
				id_ex_out_DH_raw2 	<=0;
				id_mem_out_DH_2 	<= 0; 
				id_wb_out_DH_2 		<= 0;

				id_ex_out_MemWriteN <=0;
				id_ex_out_RF_RD2 	<= 0; 
				id_ex_out_RF_RD1 	<= 0; 
			end
			// otherwise pipe flows ID->EXE
			else begin 
				id_exe_end 				<= id_exe_start; 
				id_ex_out_ALU_OP 		<= id_ex_in_ALU_OP; 
				id_ex_out_imm 			<= id_ex_in_imm; 
				id_ex_out_I_MEM_DI 		<= id_ex_in_I_MEM_DI;
				id_ex_out_I_MEM_ADDR 	<= id_ex_in_I_MEM_ADDR; 
				id_ex_out_RegWrite 		<= (branch_at_mem_wb_in)? 0:id_ex_in_RegWrite;
				id_ex_out_RF_WA1 		<= id_ex_in_RF_WA1;
				id_ex_out_ALU_src 		<= id_ex_in_ALU_src;
				id_ex_out_ALU_A 		<= id_ex_in_ALU_A; 
				id_ex_out_ALU_B 		<= id_ex_in_ALU_B;
				id_ex_out_MemWriteN <= (branch_at_mem_wb_in)? 1:id_ex_in_MemWriteN;
				id_ex_out_RF_RD2 	<= id_ex_in_RF_RD2; 
				id_ex_out_RF_RD1 	<= id_ex_in_RF_RD1; 

				id_ex_out_DH_raw1 	<= id_ex_in_DH_raw1; 	
				id_mem_out_DH_1 	<= id_mem_in_DH_1; 
				id_wb_out_DH_1 		<= id_wb_in_DH_1; 
				id_ex_out_DH_raw2 	<= id_ex_in_DH_raw2;
				id_mem_out_DH_2 	<= id_mem_in_DH_2; 
				id_wb_out_DH_2 		<= id_wb_in_DH_2;
			end
		end
	end
// pipeline flow EXE --> MEM	
	always @(posedge CLK) begin 
		if (RSTn) begin	
			// All hazard and branching should have been detected
			// pipe flows
			exe_mem_end				<= exe_mem_start;
			ex_mem_out_I_MEM_DI 	<= ex_mem_in_I_MEM_DI; 
			ex_mem_out_I_MEM_ADDR 	<= ex_mem_in_I_MEM_ADDR; 
			ex_mem_out_ALU_Result 	<= ex_mem_in_ALU_Result;
			ex_mem_out_RegWrite 	<= ex_mem_in_RegWrite; 
			ex_mem_out_RF_WA1 		<= ex_mem_in_RF_WA1; 
			ex_mem_out_MemWriteN 	<= ex_mem_in_MemWriteN;
			ex_mem_out_RF_RD2 		<= ex_mem_in_RF_RD2; 
			ex_mem_out_RF_RD1 		<= ex_mem_in_RF_RD1; 

			branch_at_ex_mem_out 	<= branch_at_ex_mem_in; 		
		end
	end
// pipeline flow MEM --> WB 
	always @(posedge CLK) begin 
		if (RSTn) begin
			// All hazard and branching should have been detected
			// pipe flows
			mem_wb_end <= mem_wb_start; 
			mem_wb_out_I_MEM_DI 	<= mem_wb_in_I_MEM_DI; 
			mem_wb_out_I_MEM_ADDR 	<= mem_wb_in_I_MEM_ADDR; 
			mem_wb_out_ALU_Result 	<= mem_wb_in_ALU_Result;
			mem_wb_out_RegWrite 	<= mem_wb_in_RegWrite; 
			mem_wb_out_RF_WA1 		<= mem_wb_in_RF_WA1; 
			mem_wb_out_D_MEM_DI 	<= mem_wb_in_D_MEM_DI; 
			mem_wb_out_RF_RD2 		<= mem_wb_in_RF_RD2; 
			mem_wb_out_RF_RD1 		<= mem_wb_in_RF_RD1; 

			branch_at_mem_wb_out 	<= branch_at_mem_wb_in; 
		end 
	end
///////////////////////////////SEWQUENCIAL LOGIC//////////////////////////////////////
endmodule //
