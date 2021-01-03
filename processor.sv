`include "proc_hdr.sv"
`include "rtllib.sv"
`define SIM  //tick commands are commands to the tools.  Tells the tools that it should go to these files and grab whats in there.  

//----------------------------------------------
`timescale 1ns/1ps

module tb_top ();
 
 
//----------------------------------------------
//Clock definition.  The true clock is eph0 which is passed to a latch with the kill logic.
//The kill logic terminates the clock passed to the processor, eph1, when a valid ebreak instruction
//reaches the alu_unit.   
 
	 localparam MAX_CLKS = 50;

	 //--clock gen
	 logic eph0; 
	 always 
			begin
					eph0  = 1'b1;
					#1; 
					eph0 = 1'b0; 
					#1; 
			end

	logic clk_kill;
	logic eph1;
		
	initial clk_kill = 1'b0;   
	rregs clockkillreg (clk_kill, 1'b1, (processor_kill & ~reset));  
	rregs clockrestert (clk_kill, 1'b0, reset);  										 
	assign eph1 = eph0 & ~clk_kill; 
		
		
			

		int random_num;
		logic start, reset, reset_r;
		initial begin
				reset  = 1;
				$display("Starting Proc Simulation");
				random_num = $random(1);
	 
				repeat(2) @(posedge eph1);
				#1 reset= '0;
		end
		rregs resetr (reset_r, reset , eph1);
		assign start = ~reset & (reset_r ^ reset);

//--params to UUT
		//input
		logic [31:0]      inst;		
		//--outputs
		logic         result_valid_e, branch;
		logic [4:0]   rd_e;            
		logic [31:0]  alu_result_e;
		logic [10:0]  branch_addr;
		
	
//processor is only capable of 32 bit instructions - not 16 or 64.  		
			 localparam NUM_INSTS = 24;
		 const logic[31:0] insts   [NUM_INSTS] = '{
			 32'h00100093, 32'h00108133, 32'h401101b3, 32'h0021e193,
			 32'h00219233, 32'h00125213, 32'h00427213, 32'h00124293,
			 32'h0080096f, 32'h4d108093, 32'h00129413, 32'h00244433,
			 32'h401403b3, 32'h12345fb7, 32'h678f8f93, 32'hff940213, 
			 32'h00821213, 
			 32'h0000A023, // store data 1d in memory address 0
			 32'h0000A583, //load data in memory address 0.  
			 32'h000a58393, //adds 10d to the value in reg 11 places in reg 7.
			 32'hfe121ee3, 32'hfff43513, 32'hfff43513, 32'h00100073
			};


  
//--load inst memeory-------
//Takes the NUM_INST vectors and writes sequential instructions (load_inst_mem_data) and associates them
//with an address (load_inst_mem_addr).  These values are passed to fetch "fetchmod" to be written to the 
//Harvard-architecture instruction memory.  When all instructions have been written to memory,
//flag load_inst_mem drops to 0.

  logic                load_inst_mem;    
	logic [PC_SZ_2B-1:0] load_inst_mem_addr;    
	logic [31:0]    		 load_inst_mem_data;    

  logic [$clog2(NUM_INSTS)-1:0] inst_ctr_r, inst_ctr_next;
  assign inst_ctr_next = (inst_ctr_r < NUM_INSTS-1 ) ? inst_ctr_r + 1'b1 : inst_ctr_r; 
  rregs #($clog2(NUM_INSTS)) ictrr  (inst_ctr_r, reset ? '0 : inst_ctr_next, eph1 );
  

  rregs limr (load_inst_mem, inst_ctr_r != NUM_INSTS-1, eph1); //NUM_INSTS-1
	
  assign load_inst_mem_addr = inst_ctr_r;
  assign load_inst_mem_data = insts[inst_ctr_r];



//--UUT---------------------------
	processor UUT (
		.eph1         				(eph1),
		.reset        				(reset),
		.start 								(start),

		.load_inst_mem        (load_inst_mem),    
		.load_inst_mem_addr   (load_inst_mem_addr),    
		.load_inst_mem_data   (load_inst_mem_data),    

		.result_valid_e  			(result_valid_e),
		.result_e        			(alu_result_e),

		.branch          			(branch),
		.processor_kill				(processor_kill)
	);

	endmodule:tb_top


	module processor (
		input  logic            			eph1,
		input  logic            			reset,
		input	 logic									start,

		input logic                		load_inst_mem,    
		input logic [PC_SZ_2B-1:0] 		load_inst_mem_addr,    
		input logic [R_SZ-1:0]     		load_inst_mem_data,    

		output logic            			result_valid_e,
		output logic [R_SZ-1:0] 			result_e,

		output logic 				   				branch,
		output logic 									processor_kill

	);


/////Start stage////
	logic										run, run_i, run_d, run_r, run_e;

/////Fetch stage////
  logic [R_SZ-1:0]     				inst; 
	logic [PC_SZ_2B-1:0]		pc_i, pc_inc;
	logic             			valid, inst_kill_logic;

/////Decode Stage////
	UOP                   	uop_d;
  logic [R_SZ-1:0]      	reg_r1_data, reg_r2_data; 
  logic [31:0]          	inst_d;
	logic [PC_SZ_2B-1:0]  	pc_d;
  logic                 	valid_d;
	logic				  					invalid_inst;

/////Register stage/////	
	//Register file substage
  UOP             				uop_r;
	logic [31:0] 		  			r1_data_r, r2_data_r, wr_data;
	logic	[PC_SZ_2B-1:0]		jalx_addr_1B;
	logic 				  				fwd_flag, fwd_flagr1, fwd_flagr2;
	logic            				valid_r;	

	//Load/Store unit substage
	logic [R_SZ-1:0] ld_st_addr_1B, ld_st_addr_4B, ld_out;
	logic [1:0] ld_st_addr_2MSB;




/////Execute/ALU_unit stage/////	
	UOP                   	uop_e;
	logic [R_SZ-1:0]      	r1_data_e, r2_data_e, alu_result_e;
	logic [PC_SZ_2B-1:0]  	branch_target_e;
	logic                 	branch_e, valid_e;
	

//-----------------------------------------------------
//--Start stage.  Sends signal to activate output pins.
//----------------------------------------------------- 
//The start stage is driven by the fall of the load_inst_mem flag from tb_top.  
//This will delay the start of actual computation by the amount of clock cycles
//equal to the number of instructions, but not delaying risks the execution 
//of a branch to a memory address that has not yet been written with an instruction.

	rregs irun 		(run_i, ~load_inst_mem  & ~start & ~reset , eph1);	
	rregs drun 		(run_d, run_i & ~start & ~reset , eph1);
	rregs rrun 		(run_r, run_d & ~start & ~reset , eph1);
	rregs erun 		(run_e, run_r & ~start & ~reset , eph1);


//-------------------------------------------
//--Inst fetch stage instantiation
//------------------------------------------- 

	fetch fetchmod ( 
		.pc_i									(pc_i),
		.pc_inc								(pc_inc),
		.inst_internal				(inst),
		
		.load_inst_mem        (load_inst_mem),    
		.load_inst_mem_addr   (load_inst_mem_addr),    
		.load_inst_mem_data   (load_inst_mem_data),    
				
		.start								(start),
		.uop_e								(uop_e),
		.alu_result_e					(alu_result_e),
		.r2_data_e						(r2_data_e),
		.branch_target_e			(branch_target_e),
		.reset								(reset),
		.branch_e							(branch_e),
		.eph1									(eph1)
	);




//-------------------------------------------
//--Decode stage 
//-------------------------------------------
//Instantiates module decode and takes registered data from the fetch stage.

	assign inst_kill_logic = branch_e; //taken branches result in a 3x clock delay.  

	assign valid = ((|inst[15:0]) | (~&inst)) & (~inst_kill_logic); 
		
		rregs #(PC_SZ_2B)      instid    (pc_d, 			pc_i,     eph1);	
		rregs #(R_SZ)     		 instdr    (inst_d, valid ? inst : 32'b11,     eph1);
		rregs #(1)       			 valdr     (valid_d,    valid,    eph1);

	decode decode (    
		.inst_d         	(inst_d),
		.pc_d           	(pc_d),
		.uop_d          	(uop_d),
		.invalid_inst   	(invalid_inst), 
		.valid_d					(valid_d),
		.run_d						(run_d),
		.inst_kill_logic	(inst_kill_logic)
	); 


//-------------------------------------------
//--Register stage  
//-------------------------------------------
//Contains the register file regfile and the load store module ldst.  
//both regfile and ldst contain internal registers, making their outputs synchronous with the next e stage.

		rregs #($bits(UOP)) uoprr  (uop_r,     (valid_d & ~inst_kill_logic) ? uop_d : '0,      eph1);
    rregs #(1)          valrr  (valid_r,   (valid_d & ~inst_kill_logic) ,    eph1);		  
	
	assign jalx_addr_1B = {uop_r.pc[PC_SZ_2B-2:0] , 1'b0}; //produces the storage address for use in JALX instructions, which is 
																												 //the 1B representation of the uop_e's pc + 2 (2B) (equal to uop_r's pc)
																												 //both JAL and JALR write one byte addresses.  


//Load store module instantiated as rf_1rw.  Unit is only capable of WORD reads and writes.  
//LB/SB and LH/SH instructions are valid but process as full words using the previous valid 4B 
//memory address specified by the register data + offset.    
//ldst both receives and exports data on every clock but only writes data to memory for store intsructions
//and ld_out only used if there is a load instruction in the r stage.  

	assign ld_st_addr_1B = r1_data_r + uop_r.immed_full;
	assign ld_st_addr_4B = ld_st_addr_1B>>2;


	rf_1rw  #(32,32) ldst (   
		.dout		(ld_out),	//data out. This is the memory in the loaded address.  
		
		.eph1		(eph1),   				
		.write	(uop_r.st_op),   
		.addr		(ld_st_addr_4B),     // a four byte address indexed in the 
		.din    (r2_data_r)   		
	);	


	
	rmuxd3 #(R_SZ) regdatawr (wr_data,												
													uop_e.ld_op 																					,ld_out, 
													(uop_e.jal | uop_e.jalr) & ~uop_e.ld_op & ~uop_e.st_op,(R_SZ)'(jalx_addr_1B), 										
													alu_result_e);
 
	
//regfile takes inputs from decode and alu_unit with an internal register.  
//The decode inputs (rd_enX/rd_rdX) create the value in the registers (reg_rX_data) as an output, 
//and the alu_unit inputs specify the 4Byte address (uop_e.wr_en/uop_e.rd) and data (wr_data) to be written to the register.  	 
 
	rf_2r1w_32x32 regfile (
		.clk          (eph1),
		.rden0_p      (uop_d.rd_en1),    
		.rdaddr0_p    (uop_d.r1),       
		.rddata0_p    (reg_r1_data),
																							
		.rden1_p      (uop_d.rd_en2 ),  
		.rdaddr1_p    (uop_d.r2),       
		.rddata1_p    (reg_r2_data),   

		.wren0_p      (uop_e.wr_en), 
		.wraddr0_p    (uop_e.rd),                   
		.wrdata0_p    (wr_data)                
	);	



//Forwarding aborts regfile's outputs if (case1) an instruction calls the same register that the previous instruction modified,   
//or (case2) if reg0 is one of the registers used. For case1, regfile's output is overwritten with the previous clock's alu_result_e.
//For case2, reg0's value in computation is always zero. 
		assign fwd_flagr1 = (uop_e.rd == uop_r.r1);
		
		assign fwd_flagr2 = (uop_e.rd == uop_r.r2);  

	rmuxd3 #(R_SZ) reg1sel (r1_data_r,
														(uop_r.r1 == 5'b0)									, 32'b0,
														(fwd_flagr1) & (uop_r.r1 != 5'b0)		, alu_result_e, 
														reg_r1_data
	);



	rmuxd3 #(R_SZ) reg2sel (r2_data_r,
														(uop_r.r2 == 5'b0)								, 32'b0,
														(fwd_flagr2) & (uop_r.r2 != 5'b0)	, alu_result_e, 
														reg_r2_data
	);


//-------------------------------------------
//--Exexcute stage ONLY with ALU now
//-------------------------------------------


		//Register passdown to alu stage including checks for branch kill logics (inst_kill_logic)
		rregs #($bits(UOP)) uoper  (uop_e, (valid_r & ~inst_kill_logic) ? uop_r : '0, eph1);
		rregs #(R_SZ)    r1dataer  (r1_data_e,  r1_data_r,  eph1);
		rregs #(R_SZ)    r2dataer  (r2_data_e,  r2_data_r,  eph1);
		rregs #(1)       valer     (valid_e,(valid_r & ~inst_kill_logic),    eph1);

	alu_unit alu_unit ( 
		.uop_e           (uop_e),
		.r1_data_e       (r1_data_e),
		.r2_data_e       (r2_data_e),
		.alu_result_e    (alu_result_e),
		.branch_e        (branch_e),
		.branch_target_e (branch_target_e),
		.run_e					 (run_e),
		.processor_kill	 (processor_kill)
	);

	assign result_valid_e = valid_e; 

	assign result_e =	alu_result_e;

			
	endmodule:processor



//---------------------------------------------
//begin stage module descriptions
//---------------------------------------------

	module fetch (
		output logic [R_SZ-1:0]				inst_internal,
		output logic [PC_SZ_2B-1:0]		pc_i,  
		output logic [PC_SZ_2B-1:0] 	pc_inc,

		input logic                		load_inst_mem,    
		input logic  [PC_SZ_2B-1:0]		load_inst_mem_addr,    
		input logic  [R_SZ-1:0]    		load_inst_mem_data, 
				

		input UOP 							 			uop_e,
		input logic		[R_SZ-1:0] 			alu_result_e,
		input logic		[R_SZ-1:0]			r2_data_e,
		input logic		[PC_SZ_2B-1:0] 	branch_target_e,
		input logic										reset,
		input logic										start,
		input logic 									branch_e,
		input logic										eph1
	);

		logic [R_SZ-1:0] rf1rw_out; 
		logic [PC_SZ_2B-1:0] pc_unlock, pc_feedback, pc_schedule_next, pc_sel_4B, branch_target_jalr, rf1rw_addr; 
		logic read_start;  
		
//By default the pc is in two byte increments and must select between the register value (JALR), branch target, or increment.
	assign branch_target_jalr = { branch_target_e[PC_SZ_2B-1], branch_target_e[PC_SZ_2B-1:1]}; //converts b_t_e to a two byte address for jalr instructions 	

	rmuxd3 #(PC_SZ_2B) fetchpc	 (pc_schedule_next,  
																(branch_e &  uop_e.jalr)		, branch_target_jalr, 
																(branch_e & ~uop_e.jalr)		, branch_target_e, //b_t_e is a one byte address but is used like a 2 byte address.
																pc_inc
	);

	rregs rdst (read_start , ~load_inst_mem , eph1); //This is a bandaid used to retard the start of memory read for one additional clock cycle
																									 //beyond load_inst_memory flag going down.  Without this 1 clk delay the memory output will
																									 //be read as the final loaded instruction's contents @ pc = 0
							
							
	assign pc_unlock = (read_start & ~(reset | start)) ? pc_schedule_next : (PC_SZ_2B)'(1'b0);  //selects the next instruction if done loading 
																																															//and a start/reset condition does not exist.  

	rregs #(PC_SZ_2B) pcfeedback (pc_feedback, pc_unlock,  eph1); //Register necessary to keep timing with pc memory delay.  

	assign pc_inc = pc_feedback + 2'b10;

	assign pc_i = pc_feedback;  											//The actual program counter which registers through the processor. 
	
		
//rf_1rw ramin can only accept four byte inputs so the two byte pc must be arith shifted right to accomodate.  
//The selected address is the program counter, unless the instructions are still being loaded at reset in which
//case it is load_inst_mem_addr. 

	assign pc_sel_4B = { pc_unlock[PC_SZ_2B-1] , pc_unlock[PC_SZ_2B-1:1] };

	assign rf1rw_addr = ~load_inst_mem ? pc_sel_4B : load_inst_mem_addr ;
 		  
	rf_1rw  #(32,32) ramin (   
		.dout		(rf1rw_out),	
		
		.eph1		(eph1),   			
		.write	(load_inst_mem),  
		.addr		(rf1rw_addr),    
		.din    (load_inst_mem_data)   	
		
	);	
	
	assign inst_internal = read_start ? rf1rw_out : 32'h3 ; //selects the memory output if done loading, otherwise output a no op.

endmodule: fetch

//The primary purpose of the decode stage is to create UOP uop_d; see proc_hdr for structural breakdown.
//invalid instructions, defined as instructions of all 1's or where the first 16 bits are zero, raise the invalid_inst flag.
//valid instructions which do not have an agreeing opcode, func3 code, and func7 code (for those with precise func7 codes) per
//page 130 of the RISC-V ISA raise no flags under UOP's structure and informally become no-op instructions.   
	module decode ( 
		input	logic [R_SZ-1:0]  	 	inst_d, 
		input	logic [PC_SZ_2B-1:0] 	pc_d, 
		input logic 								valid_d,
		input logic									run_d,
		input logic									inst_kill_logic,

		output UOP          		  	uop_d,
		output logic         		 		invalid_inst
	);

		INST32 											rv_inst;
		logic 	[R_SZ-1:0] 					immed_i32, immed_s32, immed_b32, immed_u32, immed_j32;  
		logic 	[20:0] 							sign_ext_32; 
		logic 	[4:0] 							opcode; 			
				 
		assign rv_inst = 						inst_d;   	
		
		assign opcode  = 						rv_inst.r.op2; 
																														
		assign uop_d.pc = 					pc_d;        
		
		assign uop_d.func3 = 				rv_inst.r.func3;      // primary function selector.  r selection arbitrary.
				
		assign uop_d.bit30op =			(	(opcode == ALU_OP)   & (uop_d.func3 == ADD_NN) | (uop_d.func3 == SRL_NN)  )
															& (	(~rv_inst.r.func7[6] & rv_inst.r.func7[5] & ~|rv_inst.r.func7[4:0])				);   // secondary ""
				
		assign uop_d.rd 	=					rv_inst.r.rd;         
				
		assign uop_d.r1 	= 				rv_inst.r.r1;         
				
		assign uop_d.r2	= 					rv_inst.r.r2;          
				
		assign uop_d.ld_op =				(opcode == LD_OP) & (rv_inst.i.func3[2:1] != 2'b11);  
				
		assign uop_d.st_op =				(opcode == ST_OP)	& (rv_inst.i.func3 != 3'b011) & (rv_inst.i.func3[2] != 1'b1); 
		
		 
 //The immediate section detects if an immediate instruction is used, and what the value of the immediate value is if it is used.
 //If an immediate instruction is not used (immed_op_type == 0), the immediate value is forced to zero for completeness but is 
 //otherwise unused.  
	
//Page 130 RISC-V ISA refers.
	assign uop_d.immed_op_type  =			((opcode == LUI_OP) 													 			   																				) 
																	| ((opcode == AUIPC_OP) 	 											 				  																			) 
																	| ((opcode == JAL_OP)																 	   																				)
																	| ((opcode == JALR_OP)	& (rv_inst.u.imm20[2:0] == ADD_NN)							   											)  				  
																	| ((opcode == ALUI_OP) 																   																				)  
																	| ((opcode == LD_OP) 		& (rv_inst.i.func3 != 3'b011)				&		(rv_inst.i.func3[2:1] != 2'b11)	) 
																	| ((opcode == ST_OP)		& (rv_inst.s.func3[1:0]!= 2'b11) 		& 	(rv_inst.s.func3[2] != 1'b1)		); 
																			
																						
//For use in immediate_full decodes: page 17 RISCV ISA fig 2.4  
	assign sign_ext_32[20:0]=	{21{rv_inst[31]}};	//Repeats MSB for use in immediate extensions below
		
	assign immed_i32  = 			{ sign_ext_32 , rv_inst.i.imm12[10:0]};
		
	assign immed_s32  = 			{ sign_ext_32, rv_inst.s.imm7[5:0] , rv_inst.s.imm4};
		
	assign immed_b32  = 			{ sign_ext_32[19:0], rv_inst.b.imm4[0] , rv_inst.b.imm7[5:0] , rv_inst.b.imm4[4:1] , 1'b0 };
			
	assign immed_u32  = 			{ rv_inst.u.imm20 , {12{1'b0}}};
		
	assign immed_j32  = 			{ sign_ext_32[11:0], rv_inst.j.imm20[7:0] , rv_inst.j.imm20[8], rv_inst.j.imm20[18:9] , 1'b0};
		 
//mux selects which immediate incoding variant is written to immed_full. 
//List is based on the RISC 32 base instruction formats.   
	rmuxd6 #(R_SZ) immeddata	(uop_d.immed_full,  
													( (opcode == LUI_OP)  |   (opcode == AUIPC_OP)										    										),immed_u32,
													( (opcode == JAL_OP)  																													      		),immed_j32,
													( (opcode == BR_OP)	  &   (rv_inst.b.func3[2:1] != 2'b01)							    								),immed_b32,							  
													( (opcode == ALUI_OP) |  	((opcode == JALR_OP) & (rv_inst.u.imm20[2:0] == ADD_NN)) //immed_i32 continued on next line			
													 |((opcode == LD_OP)	&		(rv_inst.i.func3 != 3'b011) & (rv_inst.i.func3[2:1] != 2'b11) ) ),immed_i32,
													( (opcode == ST_OP)	  &   (rv_inst.s.func3[1:0]!= 2'b11) & (rv_inst.s.func3[2] != 1'b1) 	),immed_s32,
																																																													R_SZ'(1'b0)//catchall zeroes 
	); 

//End immediate section 

	assign uop_d.add_op =	 	 (( (opcode == AUIPC_OP)													 	 	 	 	     																				)
													 |( (opcode == JAL_OP)															 	 	    																 					)
													 |( (opcode == JALR_OP)		& 	(rv_inst.j.imm20[2:0] == 3'b0)								     											)  
													 |( (opcode == ALUI_OP) 	& 	((rv_inst.i.func3 == ADD_NN) 		  			|  (rv_inst.i.func3 == SLT_NN)) )
													 |( (opcode == BR_OP)			& 	(rv_inst.b.func3[2:1] != 2'b01)							 	     											) 
													 |( (opcode == ALU_OP)  	& 	((uop_d.bit30op | (~|rv_inst.r.func7))	& (rv_inst.r.func3 == ADD_NN))	//continued on next line
															|((~|rv_inst.r.func7) & (rv_inst.r.func3[2:1] == 2'b01)) 																					)
	); 
	

	//subtract forms a subset of the adder.  
	assign uop_d.sub_op = (((opcode == ALU_OP)  &   	uop_d.bit30op 	& (rv_inst.r.func3 == ADD_NN))	 																		//SUB
													| (opcode==BR_OP)   														  																														//BR 
													|((opcode == ALU_OP ) & (~|rv_inst.r.func7)	& ((rv_inst.r.func3 == SLT_NN) | (rv_inst.r.func3 == SLTU_NN) ))  //SLT | SLTU
													|((opcode == ALUI_OP )& ((rv_inst.r.func3 == SLT_NN) | (rv_inst.r.func3 == SLTU_NN) ))												//SLTI| SLTIU 
	); 											 
								
		
	assign uop_d.br_op =	   ((opcode ==	BR_OP) 																			  )											
													|((opcode ==	JAL_OP) 																		  )
													|((opcode ==	JALR_OP  & 	(rv_inst.j.imm20[2:0] == 3'b000)) ); 	//branches only. Valid func3 codes for all except 2, 3.  
			
	assign uop_d.lui =	 	 (opcode == LUI_OP);
			
	assign uop_d.auipc = 	 (opcode == AUIPC_OP);
		 
	assign uop_d.jal = 	 	 (opcode == JAL_OP);
			
	assign uop_d.jalr =	 	 (opcode == JALR_OP)  & (rv_inst.j.imm20[2:0] == 3'b000);
			
	assign uop_d.fence =	 (opcode == FENCE_OP) & (rv_inst.i.func3 == 3'b000);
			
	//  The fence_i instruction is not implemented.
			
	assign uop_d.ecall = 		~|rv_inst.i.imm12[11:0] & ~|rv_inst.i.r1 & ~|rv_inst.i.func3 & ~|rv_inst.i.rd & (opcode == SPEC_OP);  //requires a 0 in imm7's LSB.  Zeroes elsewhere

	assign invalid_inst =		(~|rv_inst[15:0]) | &rv_inst ;  //The instruction is valid if it is not illegal, which is [15:0] = 0...0, or [MSB:0]=1...1

	assign uop_d.valid =	  valid_d & ~inst_kill_logic & run_d;		
	 
	assign uop_d.ebreak =		(rv_inst[R_SZ-1:7] == 25'h000_2000) & (opcode == SPEC_OP) & run_d;    //requires a 1 in imm7's LSBzerroes elsewhere	
		
///read and write enables.
///////////////////////////////////
//Resolves 1 for write, read1, read2  enable if the instruction is VALID, has a defined OPCODE with a matching func3 code.  
		
	assign uop_d.wr_en =			uop_d.valid & run_d &
														(( (opcode == LUI_OP) 													 	 	 	 	 		) 
														|( (opcode == AUIPC_OP) 													 	 	 			) 
														|( (opcode == JAL_OP)															 	 				)
														|( (opcode == JALR_OP)	& (rv_inst.j.imm20[2:0] == ADD_NN)	)  //func3 MUST be 0
														|( (opcode == LD_OP) 		& (rv_inst.i.func3[2:1] != 2'b11)		)  //all func3 codes valid except 6 and 7
														|( (opcode == ALUI_OP) 																	 		)
														|( (opcode == ALU_OP) 													 			 	 		)
														|( (opcode == FENCE_OP) & (rv_inst.i.func3 == ADD_NN)				)
	);					
									

									
	assign uop_d.rd_en1 =	 		(( (opcode == JALR_OP)	&	(rv_inst.j.imm20[2:0] == ADD_NN)							    							)  //func3 MUST be 0
														|( (opcode == BR_OP)		&	(rv_inst.b.func3[2:1] != 2'b01)							    								) 
														|( (opcode == LD_OP) 		&	(rv_inst.i.func3 != 3'b011)  & (rv_inst.i.func3[2:1] != 2'b11) 	)  //all func3 codes valid except 3 6 and 7
														|( (opcode == ST_OP)		& (rv_inst.s.func3 != 3'b011) & (rv_inst.s.func3[2] != 1'b1)    	)
														|( (opcode == ALUI_OP) 																	    															)
														|( (opcode == ALU_OP) 													 			 																		)
														|( (opcode == FENCE_OP) & (rv_inst.i.func3 == ADD_NN)								 											)
	);

																													 
	assign uop_d.rd_en2 =	 		( ((opcode == BR_OP)		& (rv_inst.b.func3[2:1] != 2'b01)							    						)  //all func3 codes valid except 2 and 3
														| ((opcode == ST_OP)		& (rv_inst.s.func3 != 3'b011) & (rv_inst.s.func3[2] != 1'b1)	) // func3 codes 0 1 2 valid.  (~3 & ~4+) 
														| ((opcode == ALU_OP) 												 																				)
	); 
						
///End read and write enables.
///////////////////////////////////	
		
endmodule:decode
//------------------------------------------------------

////////////////////////////////////////////////////////////////////////
///////////register file module definition is in rtllib.sv//////////////
////////////////////////////////////////////////////////////////////////

//------------------------------------------------------

	module alu_unit (
	//---------------------------
		input  logic                eph1,          
		input  UOP            	    uop_e,          
		input  logic [R_SZ-1:0]			r1_data_e,      
		input  logic [R_SZ-1:0]			r2_data_e,      
		input	 logic 								run_e,

		output logic [R_SZ-1:0]			alu_result_e,   
		output logic [PC_SZ_2B-1:0] branch_target_e, 
		output logic								branch_e,      
		output logic								processor_kill	
	);

		logic            						valid_r;
		logic [31:0] 		  					r1_data_r, r2_data_r, wr_data;
		logic 				  						fwd_flagr1, fwd_flagr2; 
///Arithmetic declarations
		logic  [R_SZ-1:0] oprnd1, oprnd2a, oprnd2, add_out, alu_out_m, alu_out_s;
		logic             jalx;
		logic 			  		cy;  
///Shift declarations
		logic signed	[R_SZ:0] 		shift_input;	
		logic 				[R_SZ:0]		shift_action;								
		logic 				[R_SZ-1:0]	right_shift_data,  left_shift_data,   
															r_l_shift_select,	 right_shift_result,
															left_shift_result, shift_output;	
		logic					[4:0]				shift_magnitude; 
//Branch declarations
		logic 				[R_SZ-1:0]	br_target_add, immed_full_2B; 
		logic 				 branchfunc3flag, BGES_logic;


//////////Operand Assignments////////////
/////////////////////////////////////////

	assign jalx = 				uop_e.jal | uop_e.jalr;
																						 
	assign oprnd1   = 		(jalx | uop_e.auipc)  ? R_SZ'(uop_e.pc) : r1_data_e;	
			
	rmuxd3 #(R_SZ)	op2m	(oprnd2a,
															 jalx																				, R_SZ'(2'h2),
															~jalx & (uop_e.auipc | uop_e.immed_op_type)	, uop_e.immed_full, 
																																						r2_data_e  
	); 
																 					 
	assign oprnd2 = 			uop_e.sub_op  ? ~oprnd2a  : oprnd2a; 
		
	assign {cy, add_out}= oprnd1 +  oprnd2 + uop_e.sub_op;       
		
	
//END Operand Assignments/////////////////
//////////////////////////////////////////


/////////////////////////////////
//////////shifts////////////////
//All shift operations are encoded as variations of shift-right-arithmetic.
//Shift left operations reverse the input vector, perform the number of shifts, then reverse again.
//Logical shifts force the "carry in" bit to zero, resulting in zeros shifting in from the MSB side.
//The excess shift "in" bit is truncated after the shift action has processed. 
	
	assign right_shift_data	=		r1_data_e;

	assign left_shift_data = 		{<< {r1_data_e}};

	rmuxdx2 #(5)	shiftmag 			(shift_magnitude, 																//the mux that logically outputs the magnitude(# of bits shifted)
																uop_e.immed_op_type,	uop_e.immed_full[4:0],  	//zero for immediate - only the smallest 5 bits (shift up to 31 spaces)
															 ~uop_e.immed_op_type,	r2_data_e[4:0]   												//one for R.  Holds the value of R2
	);
			
	assign r_l_shift_select= 		uop_e.func3[2] ? right_shift_data : left_shift_data;						//selects right or left shift based on func3 MSB

	assign shift_input = 				{uop_e.bit30op & r_l_shift_select[R_SZ-1], r_l_shift_select};		//concats the shift in bit.

	assign shift_action = 			shift_input >>> shift_magnitude;												//Right arith shifts r_l_shift_select shift_magnitude number of times.  
	 
	assign right_shift_result = shift_action[R_SZ-1:0];																					//Removes the MSB prior to the flip back if applicable.

	assign left_shift_result = 	{<<{right_shift_result}};
		
	assign shift_output  = 			uop_e.func3[2] ? right_shift_result : left_shift_result; 				//reflects the bits back again if performing left shift.
			
//////////END shifts/////////////
/////////////////////////////////
		
/////////////////////////////////
//////////ALU result selections//
//organized into muxes based on time of result arrival

	rmuxd4 #(R_SZ) aluslow (alu_out_s, // the slow path
														((uop_e.func3==ADD_NN)	| (uop_e.br_op))	,	add_out,
														uop_e.func3==SLT_NN         							,	R_SZ'(~add_out[R_SZ-1]), 
														uop_e.func3==SLTU_NN        							,	R_SZ'( ~cy),   
																																				R_SZ'(1'b0)
	);

	rmuxd6 #(R_SZ) alufast (alu_out_m,                                                                  
														~uop_e.lui & ((uop_e.func3==SLL_NN) | (uop_e.func3==SRL_NN)) 	, shift_output ,
														~uop_e.lui & (uop_e.func3==AND_NN)          	             	 	, oprnd1 &  oprnd2,
														~uop_e.lui & (uop_e.func3==OR_NN)       	                 	 	, oprnd1 |  oprnd2,  
														~uop_e.lui & (uop_e.func3==XOR_NN)  	                     		, oprnd1 ^  oprnd2,
														 uop_e.lui                        	                        	, uop_e.immed_full,
																																														R_SZ'(1'b0)
	);

	assign alu_result_e = uop_e.add_op ? alu_out_s: alu_out_m;  							

/////End ALU result selections////
//////////////////////////////////
	
/////////////////////////////////////
/////Branch selection/computation////
//The branch section determines a branch location and whether a branch condition exists.  
//If the a branch condition exists (branch_e == 1), mux fetchpc selects a branch address rather than increment.

	assign BGES_logic = 			(oprnd1[R_SZ-1]^oprnd2a[R_SZ-1]) ? ~oprnd1[R_SZ-1] : ~alu_result_e[R_SZ-1];	

	assign immed_full_2B = {uop_e.immed_full[R_SZ-1] ,uop_e.immed_full[R_SZ-1:1]};
			
	assign br_target_add = 		(uop_e.jalr ? r1_data_e : uop_e.pc ) + (uop_e.jalr ? uop_e.immed_full: immed_full_2B); //ONLY JALR reads one byte addresses.  Rest use two bytes.   
					
	assign branch_target_e = 	{br_target_add[PC_SZ_2B-1:1] , br_target_add[0] & ~uop_e.jalr};  //forces the last bit to zero for the JALR instructions
									 
													
	rmuxd7 #(1) branchflags 	(branchfunc3flag,					
															uop_e.func3 == BEQ_NN ,  ~|alu_result_e,      		//table IAW page 130 of RISC-V spec pg 130
															uop_e.func3 == BNE_NN ,   |alu_result_e,       		// "
															uop_e.func3 == BLT_NN ,   ~BGES_logic,						// "
															uop_e.func3 == BGE_NN ,    BGES_logic, 						// "
															uop_e.func3 == BLTU_NN,   ~cy, 										// The unsigned BGEU logic is equal to the carry out bit of the operands.  
															uop_e.func3 == BGEU_NN,    cy, 	
																													1'b0
	);					
																						 
	assign 	branch_e = 				uop_e.br_op & (branchfunc3flag | jalx) & run_e; 									 

	assign 	processor_kill = 	uop_e.valid & uop_e.ebreak & run_e;

//////////////////////////////////////
//END Branch selection/computation////
//////////////////////////////////////																						
														
endmodule:alu_unit









