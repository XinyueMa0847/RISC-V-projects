module alu(A,B,OP,C,Bcond);

    input [31:0]A;
	input [31:0]B;
    input [3:0]OP;
	output [31:0]C;
    output Bcond;

    reg Bcond;
    reg c;
	reg [31:0] C;

	always @(*)
        
		case(OP)
		4'b0000: begin
			//ADD
            assign C = A + B; Bcond = 0; 
			
            end
		1:	begin
			//SUB
            assign C = A - B; Bcond = 0; 

			end
        2:	begin
            //SLT
            if(A[31] == 1 && B[31] == 0)begin
			    assign C = 32'b00001;
            end
            else if(A[31] == 0 && B[31] == 1)begin
                assign C = 32'b00000;
            end
            else if(A[31] == 0 && B[31] == 0)begin
                assign C = (A < B) ? 1 : 0;
            end
            else begin
                assign C = (A > B) ? 1 : 0;   
            end
            assign Bcond = 0; 
        
			end

		3:	begin
            //SLTU
			assign C = (A < B) ? 1 : 0; Bcond = 0; 		
			
			end

		4:	begin
            //SLL
			assign C = A << B; Bcond = 0; 

			end 

		5:	begin
            //SRL
			assign C = A >> B; Bcond = 0; 

			end

		6:	begin
            //SRA
			if(A[31] == 0)begin
            assign C = A >>> B; Bcond = 0; 
			end
			if(A[31] == 1)begin
			assign C = -((-A) >>> B); Bcond = 0;
			end
			
			end

		7:	begin
            //OR
			assign C = A | B; Bcond = 0; 		
			
			end
	
		8:	begin
            //XOR
			assign C = A ^ B; Bcond = 0; 

            end

		9:	begin
			//AND
            assign C = A & B; Bcond = 0; 

			end

		10:	begin
			//BEQ
            assign Bcond = (A == B) ? 1 : 0; C = 0;

			end

		11:	begin
			//BNE
            assign Bcond = (A != B) ? 1 : 0; C = 0;

            end

		12:	begin
			//BLT
            if(A[31] == 1 && B[31] == 0)begin
			    assign c = 1;
            end
            else if(A[31] == 0 && B[31] == 1)begin
                assign c = 0;
            end
            else if(A[31] == 0 && B[31] == 0)begin
                assign c = (A < B) ? 1 : 0;
            end
            else begin
                assign c = (A > B) ? 1 : 0;   
            end
            assign Bcond = c ? 1 : 0; C = 0;
			
            end
		
		13:	begin
			//BGE
            if(A[31] == 1 && A[31] == 0)begin
			    assign c = 1;
            end
            else if(A[31] == 0 && B[31] == 1)begin
                assign c = 0;
            end
            else if(A[31] == 0 && B[31] == 0)begin
                assign c = (A < B) ? 1 : 0;
            end
            else begin
                assign c = (A > B) ? 1 : 0;   
            end
            assign Bcond = (!c) ? 1 : 0; C = 0;

			end


		14:	begin
			//BGEU
            assign c = (A < B) ? 1 : 0;
            assign Bcond = c ? 1 : 0; C = 0;

			end
	
		15:	begin
            //BLTU
            assign c = (A < B) ? 1 : 0;
            assign Bcond = (!c) ? 1 : 0; C = 0;

			end
			
		endcase	
endmodule

module mux2 (
    input [31:0] A,
    input [31:0] B,
    input  sel,
    output reg [31:0] out
    ); 
    //reg [31:0] out;
    always @(*)
    begin
        if (sel) begin
            out = B;
        end
        else begin 
            out = A;
	end
    end
endmodule

module mux3 (
    input [31:0] A,
    input [31:0] B,
    input [31:0] C,
    input  [1:0] sel,
    output reg [31:0] out
    );

	always @(*) begin
		case(sel) 
			2'b00: begin 
				out = A;
			end
			2'b01: begin
				out = B;
			end
			
			2'b10: begin
				out = C;
			end
		endcase		
	end

endmodule

module mux7 (
    input [31:0] A,
    input [31:0] B,
    input [31:0] C,
    input [31:0] D,
    input [31:0] E,
    input [31:0] F,
    input [31:0] G,
    input  [2:0] sel,
    output reg [31:0] out
    ); 

	always @(*)
	begin
		case(sel) 
			3'b000: out = A;
			3'b011: out = B;
			3'b001: out = C;
			3'b100: out = D;
			3'b110: out = E;
            3'b111: out = F;
            3'b010: out = G;
		endcase
	end

endmodule

module imm_proc(
    input [31:0] inst,
    input [2:0] mode,
    output reg [31:0] imm
    );
	always @(*) begin
		case(mode) 
			// I-type
			3'b000:  imm = { {21 {inst[31]} }, inst[30:20] };
			// S-type
			3'b001:  imm = { {21 {inst[31]} }, inst[30:25], inst[11:8],inst[7]};
			//B-type 
			3'b010:  imm = { {20 {inst[31]} }, inst[7], inst[30:25], inst[11:8],1'b0};
			//U-type
			3'b011:  imm = { inst[31:12], {12{1'b0}}};
			//J-type
			3'b100:  imm = { {13{inst[31]}}, inst[19:12], inst[20], inst[30:21],1'b0};
			default: imm = {32{1'b0}};
		endcase
	end	
endmodule

module mem_out_proc(
    input [31:0] D_MEM_DI,
    input [3:0] BE,
    input sign_extend,
    output reg [31:0] P_Dout
    );
	always @(*) begin
		case({BE,sign_extend}) 
			
			//LB
			5'b00011: begin
				P_Dout = { {24 {D_MEM_DI[7]} }, D_MEM_DI[7:0] };
			end
			//LBU
			5'b00010: begin
				P_Dout = {{24{1'b0}},D_MEM_DI[7:0]};
			end
			//LH
			5'b00111:
				P_Dout = {{16{D_MEM_DI[15]}},D_MEM_DI[15:0]};
			//LHU
			5'b00110:
				P_Dout = { {16{1'b0}} ,D_MEM_DI[15:0] };
			//LW
			5'b11111: P_Dout = D_MEM_DI;
			default:  P_Dout = D_MEM_DI;
			
		endcase		
	end
endmodule

module control (
    input [31:0] inst,
    input RSTn,
    output reg I_MEM_CSN,
    output reg D_MEM_CSN,
	output reg REG_WRITE,		//connect to REG_FILE: WE
	output reg D_MEM_WEN,	//connect to D_MEM: WEN 0 when write is enabled
	output reg [3:0] D_MEM_BE,		//connect to D_MEM: BE
	output reg sign_extend, 	//sign_extend method for processing D_MEM_OUT
	output reg [2:0] IMM_MODE,	//connect to imm_processing :mode
	output reg BRANCH,			//AND with ALU: ZERO to connect to pc_select MUX2: sel
	output reg [3:0] ALU_ARITH, // Connect to alu: OP
	output reg ALU_SEL,		//connect to mux: sel, before ALU: B
	output reg ALU_PC_SEL,		//connect to mux: sel before ALU for pc: A
	output reg TAR_SEL,		//connect to pc_select MUX 2, choose the target address, doesn't decide whether branching takes place
	output reg [2:0] WD_SEL, 	//coonect to mux6 before REG_ED
    output reg JUMP, 
	output reg HALT_INST		//whether a halt instruction 0x00008067 was received 
    );

    always @(*) begin
        if (~RSTn) begin 
            I_MEM_CSN = 1'b1;
            D_MEM_CSN = 1'b1;
            REG_WRITE = 1'b0;
            D_MEM_WEN = 1'b1;
            D_MEM_BE = 4'b0000;
        end 
        else begin 
            if(inst == 32'h00008067) 
                HALT_INST = 1'b1;		
            else begin
                HALT_INST = 1'b0;
            end
            I_MEM_CSN = 1'b0;
            //opcode decoding 
            case(inst[6:0]) 
                //R-type:
                7'b0110011: begin
                    D_MEM_CSN = 1'b1;
                    REG_WRITE = 1'b1; 
                    D_MEM_WEN = 1'b1;
                    D_MEM_BE = 4'b1111; 	//by default, read all four bytes
                    sign_extend = 1'b1; 	//by default, sign-extend immediate
                    IMM_MODE = 3'b111; //immediate will be 0
                    BRANCH = 1'b0;
                    //ALU_ARITH
                    case({inst[31:25],inst[14:12]})
                        //ADD
                        10'b0000000000: ALU_ARITH = 4'b0000; 
                        //SUB 
                        10'b0100000000: ALU_ARITH = 4'b0001;
                        //SLT 
                        10'b0000000010: ALU_ARITH = 4'b0010;
                        //SLTU 
                        10'b0000000011: ALU_ARITH = 4'b0011;
                        //SLL
                        10'b0000000001: ALU_ARITH = 4'b0100;   
                        //SRL
                        10'b0000000101: ALU_ARITH = 4'b0101; 
                        //SRA
                        10'b0100000101: ALU_ARITH = 4'b0110; 
                        //OR
                        10'b0000000110: ALU_ARITH = 4'b0111;
                        //XOR
                        10'b0000000100: ALU_ARITH = 4'b1000;
                        //AND 
                        10'b0000000111: ALU_ARITH = 4'b1001;

                        //what to use as default? 
                        default: ALU_ARITH = 4'b0000;
                    endcase
                    ALU_SEL = 1'b1; 	//R-type, the second operand is rs2
                    ALU_PC_SEL = 1'b1; 		//select PC value (instead of rs1) 
                    TAR_SEL = 1'b0;		//Not relevent, by default choose immediate along
                    WD_SEL = 3'b000; 	//WD is ALU result
                    JUMP = 1'b0;

                end
                //I-type, LOAD:
                7'b0000011: begin
                    D_MEM_CSN = 1'b0;
                    REG_WRITE = 1'b1;
                    D_MEM_WEN = 1'b1; 
                    case(inst[14:12]) 
                        3'b000: {D_MEM_BE,sign_extend} = 5'b00011;		//LB 
                        3'b001: {D_MEM_BE,sign_extend} = 5'b00111;		//LH
                        3'b010: {D_MEM_BE,sign_extend} = 5'b11111;		//LW
                        3'b100: {D_MEM_BE,sign_extend} = 5'b00010;		//LBU
                        3'b101: {D_MEM_BE,sign_extend} = 5'b00110;		//LHU
                    endcase
                    IMM_MODE = 3'b000;
                    BRANCH=1'b0;
                    ALU_ARITH = 4'b0000;  //LD adds the immediate with rs1 
                    ALU_SEL = 1'b0; 		//LD instructions, second operand is an Immediate
                    ALU_PC_SEL = 1'b1; 	    //select pc
                    TAR_SEL = 1'b1;		//Not relevent, by default choose immediate along
                    WD_SEL = 3'b001; 	//WD is MEM_DOUT
                    JUMP = 1'b0;
                end

                //I-type: computational
                7'b0010011: begin
                    D_MEM_CSN = 1'b1;
                    REG_WRITE = 1'b1;
                    D_MEM_WEN = 1'b1;
                    D_MEM_BE = 4'b1111;
                    sign_extend = 1'b1;   
                    IMM_MODE = 3'b000;
                    BRANCH=1'b0;
                    //ALU-ARITH
                    case(inst[14:12])
                        3'b000: ALU_ARITH = 4'b0000; //ADDI
                        3'b001: ALU_ARITH = 4'b0100; //SLL 
                        3'b010: ALU_ARITH = 4'b0010; //SLTI
                        3'b011: ALU_ARITH = 4'b0011; //SLTIU
                        3'b100: ALU_ARITH = 4'b1000; //XORI 
                        3'b110: ALU_ARITH = 4'b0111; //ORI    
                        3'b111: ALU_ARITH = 4'b1001; //ANDI   
                        3'b101: begin 
                            if (inst[31:25]==7'b0000000) begin 
                                ALU_ARITH = 4'b0101; //SRLI    
                            end
                            else begin 
                                 ALU_ARITH = 4'b0110; //SRAI 
                            end 
                        end                  
                    endcase
                    ALU_SEL = 1'b0; //I-type arithmetic, second operand is an Immediate
                    ALU_PC_SEL = 1'b1; 	
                    TAR_SEL = 1'b0;		//Not relevent, by default choose immediate along
                    WD_SEL = 3'b000; 	//WD is ALU result
                    JUMP = 1'b0;
                end
                

                //B-type:
                7'b1100011: begin
                    D_MEM_CSN = 1'b1;
                    REG_WRITE = 1'b0; 
                    D_MEM_WEN = 1'b1; 
                    D_MEM_BE = 4'b1111;
                    sign_extend = 1'b1;
                    IMM_MODE = 3'b010;
                    BRANCH= 1'b1;
                    //ALU——ARITH
                    case(inst[14:12])

                        //BEQ
                        3'b000: ALU_ARITH = 4'b1010;
                        //BNE
                        3'b001: ALU_ARITH = 4'b1011;
                        //BLT
                        3'b100: ALU_ARITH = 4'b1100;
                        //BGE
                        3'b101: ALU_ARITH = 4'b1101;
                        //BGEU
                        3'b111: ALU_ARITH = 4'b1110;
                        //BLTU
                        3'b110: ALU_ARITH = 4'b1111;
                    endcase
                    ALU_SEL = 1'b1; 	//B-type, compare rs1, rs2; thus second operand is rs2
                    ALU_PC_SEL = 1'b1; 	 	//branch: PC+ sign_extended immediate
                    TAR_SEL = 1'b0;		//target is branch to PC+sign-extend immediate
                    WD_SEL = 3'b010; 	//not relevent, default is ALU output
                end

                //S-type:
                7'b0100011: begin
                    D_MEM_CSN = 1'b0;
                    REG_WRITE = 1'b0;
                    D_MEM_WEN = 1'b0;  
                    case(inst[14:12]) 
                        3'b000: {D_MEM_BE,sign_extend} = 5'b00011;		//SB 
                        3'b001: {D_MEM_BE,sign_extend} = 5'b00111;		//SH
                        3'b010: {D_MEM_BE,sign_extend} = 5'b11111;		//SW
                    endcase
                    IMM_MODE = 3'b001;
                    BRANCH= 1'b0;
                    ALU_ARITH = 4'b0000;  //Store adds the immediate with rs1
                    ALU_SEL = 1'b0; 	//adds rs1 with an immediate to form effective address, second operand is an immediate
                    ALU_PC_SEL = 1'b1; 	
                    TAR_SEL = 1'b0;		//Not relevent, by default choose immediate along
                    WD_SEL = 3'b000; 	//not relevent, default is ALU output
                    JUMP = 1'b0;
                end

                //U-type-LUI
                7'b0110111: begin
                    D_MEM_CSN = 1'b1;
                    REG_WRITE = 1'b1;
                    D_MEM_WEN = 1'b1;
                    D_MEM_BE = 4'b1111;
                    sign_extend = 1'b1;
                    IMM_MODE = 3'b011;
                    BRANCH= 1'b0;
                    ALU_ARITH = 4'b0000;  //LUI doesnt use ALU, set to 1 by default
                    ALU_SEL = 1'b1; 	//LUI doent use ALU, but ALU_SEL is set to 1 by default
                    ALU_PC_SEL = 1'b1; 	
                    TAR_SEL = 1'b0;		//Not relevent, by default choose immediate along
                    WD_SEL = 3'b100; 	//WD is processed immediate
                    JUMP = 1'b0;

                end

                //U-type-AUIPC
                7'b0010111: begin
                    D_MEM_CSN = 1'b1; 
                    REG_WRITE = 1'b1;
                    D_MEM_WEN = 1'b1;
                    D_MEM_BE = 4'b1111;
                    sign_extend = 1'b1;
                    IMM_MODE = 3'b011;
                    BRANCH= 1'b0;
                    ALU_ARITH = 4'b0000; //AUIPC gives operand to the ALU_PC, not the general ALU
                    ALU_SEL = 1'b1; 	//AUIPC doent use ALU, but ALU_SEL is set to 1 by default
                    ALU_PC_SEL = 1'b1; 	
                    TAR_SEL = 1'b0;		//Not relevent, by default choose immediate along
                    WD_SEL = 3'b110; 	//WD is pc+sign-extend immediate
                    JUMP = 1'b0; 
                end

                //J-type JAL 
                7'b1101111: begin
                    D_MEM_CSN = 1'b1;
                    REG_WRITE = 1'b1;
                    D_MEM_WEN = 1'b1; 
                    D_MEM_BE = 4'b1111;
                    sign_extend = 1'b1;
                    IMM_MODE = 3'b100;
                    BRANCH= 1'b0;
                    ALU_ARITH = 4'b0000; //JAL gives operand to the ALU_PC, not the general ALU
                    ALU_SEL = 1'b1; 	//set to 1 by default
                    ALU_PC_SEL = 1'b1; 	//choose pc, not rs1
                    TAR_SEL = 1'b0;		//target is PC + sign-extend
                    WD_SEL = 3'b011; 	//WD is PC+4
                    JUMP = 1'b1;
                end

                //I-type JALR
                7'b1100111: begin
                    D_MEM_CSN = 1'b1;
                    REG_WRITE = 1'b1;
                    D_MEM_WEN = 1'b1;
                    D_MEM_BE = 4'b1111;
                    sign_extend = 1'b1;
                    IMM_MODE = 3'b000;
                    BRANCH= 1'b0;
                    ALU_ARITH = 4'b0000; //JALR gives operand to the ALU_PC, not the general ALU
                    ALU_SEL = 1'b1; 	//set to 1 by default
                    ALU_PC_SEL = 1'b0; 		//target = (rs1 + sign-extend immediate	) & 0xfffffffe 
                    TAR_SEL = 1'b1;		//target = (rs1 + sign-extend immediate	) & 0xfffffffe
                    WD_SEL = 3'b011; 	//WD is PC+4
                    JUMP = 1'b1;
                end

            endcase
        end 
		
	end

endmodule

module RISCV_TOP (
	//General Signals
	input wire CLK,
	input wire RSTn,

	//I-Memory Signals
	/*Instruction Memory*/
	output wire I_MEM_CSN, //set to 0 when RSTn is 1
	input wire [31:0] I_MEM_DI,//input from IM
	output reg [11:0] I_MEM_ADDR,//in byte address

	//D-Memory Signals
	/*Data Memory*/
	output wire D_MEM_CSN, //set to 0 when RSTn is 1
	input wire [31:0] D_MEM_DI,
	output wire [31:0] D_MEM_DOUT,
	output wire [11:0] D_MEM_ADDR,//in word address
	output wire D_MEM_WEN,


	/*SB b0001, SHb0011, SW b1111, SW b1111,LB b0001, LH b0011, LW b1111*/
	output wire [3:0] D_MEM_BE, /* BE= byte-enabled*/

	//RegFile Signals
	output wire RF_WE, /*register write enable*/
	output wire [4:0] RF_RA1, //rs1
	output wire [4:0] RF_RA2, //rs2
	output wire [4:0] RF_WA1, //rd
	input wire [31:0] RF_RD1, //data from rs1
	input wire [31:0] RF_RD2, //data from rs2
	output wire [31:0] RF_WD, //data to write to rd

	/*The terminate condition is (RF_RD1 == 0x0000000c) when the received instruction is 0x00008067.*/
	output wire HALT,                   // if set, terminate program
	output reg [31:0] NUM_INST,         // number of instruction completed
	output wire [31:0] OUTPUT_PORT      // equal RF_WD this port is used for test
	);

    reg [31:0] pc;
    wire [31:0] next_pc;// = 32'b0; 
    wire [31:0] no_write = 32'b0;
    wire [31:0] pc_wire;
    wire [31:0] imm;
    wire [31:0] inst = I_MEM_DI;
    assign RF_RA1 = inst [19:15];
    assign RF_RA2 = inst [24:20];
    assign RF_WA1 = inst [11:7];
    assign pc_wire = pc; 


    wire [31:0] ALU_RESULT; 
	assign OUTPUT_PORT = RF_WD;
    assign D_MEM_ADDR = ALU_RESULT[13:2];
    assign D_MEM_DOUT = RF_RD2;
	initial begin
		NUM_INST <= 0;
        pc <= 0;
	end

	// Only allow for NUM_INST
	always @ (negedge CLK) begin
		if (RSTn) NUM_INST <= NUM_INST + 1;
	end
	always @ (posedge CLK) begin
		if(~RSTn) begin 
            pc = 32'b0;
            I_MEM_ADDR = pc[11:0];
        end
        else begin
            pc=next_pc;
            I_MEM_ADDR = pc[11:0];
        end

	end
    
    wire sign_extend,branch,ALU_B_SEL,TAR_SEL,HALT_INST,JUMP;
    reg HALT_RD;
    wire [2:0] IMM_MODE,WD_SEL;
    wire [3:0] ALU_OP; 
    always @(*) begin 
        if(RF_RD1==32'h0000000c) begin 
           HALT_RD = 1'b1;
        end
        else begin 
            HALT_RD = 1'b0;
        end
    end
    assign HALT = HALT_INST & HALT_RD;
    control control_unit(
        inst,
        RSTn,
        I_MEM_CSN,
        D_MEM_CSN,
        RF_WE,
        D_MEM_WEN,
        D_MEM_BE,
        sign_extend,
        IMM_MODE,
        branch,
        ALU_OP,
        ALU_B_SEL,
        ALU_PC_SEL, 
        TAR_SEL,
        WD_SEL,
        JUMP,
        HALT_INST

    );
    wire [31:0] ALU_B;
    imm_proc immediate(inst,IMM_MODE,imm);
    mux2 alu_b(
        imm,
        RF_RD2,
        ALU_B_SEL,
        ALU_B
    );

    wire bcond;
    alu compute_data(
        .A(RF_RD1),
        .B(ALU_B),
        .OP(ALU_OP),
        .C(ALU_RESULT),
        .Bcond(bcond)
    );
    wire [31:0] PC_ALU_A;
    mux2 sel_pc_or_rs1(
        RF_RD1,
        pc_wire,
        ALU_PC_SEL,
        PC_ALU_A
    );

    wire dummy_bcond; 
    wire [31:0]four = 32'd4;
    wire [31:0] pc_step;
    wire [31:0] pc_branch_addr;
    alu pc_add_imm(PC_ALU_A,imm,4'b0000,pc_branch_addr,dummy_bcond);
    alu pc_add_4(PC_ALU_A,four,4'b0000,pc_step,dummy_bcond);

    
    mux7 choose_wd(
        ALU_RESULT,
        pc_step,
        D_MEM_DI,
        imm,
        pc_branch_addr,
        no_write,
        {{31{1'b0}},bcond}, 
        WD_SEL,
        RF_WD
    );
    wire [31:0] JALR_branch_addr = pc_branch_addr & 32'hfffffffe;
    wire [31:0] target;
    mux2 target_mux(
        pc_branch_addr,
        JALR_branch_addr,
        TAR_SEL,
        target
    );
    wire pc_sel;
    assign pc_sel = ((branch & bcond)|JUMP);
    mux2 next_pc_mux(
        pc_step,
        target,
        pc_sel,
        next_pc
    );

endmodule