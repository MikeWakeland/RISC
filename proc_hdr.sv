
localparam R_SZ       = 32;  // 32b data size
localparam PC_SZ_2B   = 11;  // arbitrary = 4KB inst ram

//------------------------------------------
// UOP -- this is the result of decoding an inst
//        it contains the control values needed to execute the inst   
//
typedef struct packed {
      logic                   valid;
      logic [PC_SZ_2B-1:0]     pc;        // addr (2B form) of inst

      //--parts of insts to be used/decoded later
      //  if a part is not used by inst, what goes here is DC, so can just be inst bits
      logic [2:0]             func3;      // primary function selector
      logic                   bit30op;    // secondary ""
      logic [4:0]             rd;         // the three regsiter address fields
      logic [4:0]             r1;         //
      logic [4:0]             r2;         //
      
      //--common decodeccontols across multiple insts
      logic                   wr_en;      // inst write a register with rd addr
      logic                   rd_en1;     // inst reads a regsiter with r1 addr
      logic                   rd_en2;     //   "                        r2
      
      logic                   ld_op;      // a memory load inst
      logic                   st_op;      // "        store
      logic                   add_op;     // any op needing add, selects adder result, see adder in i-unit for details of which ops use
      logic                   sub_op;     // "                 , turns adder into subtracter
      logic                   br_op;      // a conditional branch inst

      //--specific inst decode that do unique things
      logic                   lui;
      logic                   auipc;
      logic                   jal;
      logic                   jalr;
      logic                   fence;
      logic                   fence_i;
      logic                   ecall;
      logic                   ebreak;
 
      //--immediate data from inst, extended as appropriate
      logic                   immed_op_type;  // means use immed for oprnd
      logic [R_SZ-1:0]        immed_full;     // the immed data for this inst
	  

} UOP;

//---------------------------------------------
//--DECODE THINGS-----------------------------
//---------------------------------------------
    typedef union packed {
        logic [31:0]           raw;
        struct packed {
           logic signed [11:0] imm12;    
           logic        [4:0]  r1;       
           logic        [2:0]  func3;    
           logic        [4:0]  rd;       
           logic        [4:0]  op2;     // =00xxx
           logic        [1:0]  op1; 
        } i;                            // alui, ld
        struct packed {
           logic        [6:0]  func7;    
           logic        [4:0]  r2;       
           logic        [4:0]  r1;       
           logic        [2:0]  func3;    
           logic        [4:0]  rd;       
           logic        [4:0]  op2;    // =01xxx
           logic        [1:0]  op1; 
        } r;                           // alu
        struct packed {
           logic signed [6:0]  imm7;     
           logic        [4:0]  r2;       
           logic        [4:0]  r1;       
           logic        [2:0]  func3;    
           logic        [4:0]  imm4;     
           logic        [4:0]  op2;    // =01xxx
           logic        [1:0]  op1; 
        } s;                           // st
        struct packed {
           logic signed [6:0]  imm7;     
           logic        [4:0]  r2;       
           logic        [4:0]  r1;       
           logic        [2:0]  func3;    
           logic        [4:0]  imm4;     
           logic        [4:0]  op2;      // =11xxx
           logic        [1:0]  op1; 
        } b;                             // br
        struct packed {
           logic signed [19:0] imm20;     
           logic        [4:0]  rd;     
           logic        [4:0]  op2; 
           logic        [1:0]  op1; 
        } u;                         //
        struct packed {
           logic signed [19:0] imm20;     
           logic        [4:0]  rd;     
           logic        [4:0]  op2; 
           logic        [1:0]  op1; 
        } j;                             //
    } INST32;   // op1 = 2'b11 for all these

    //--op2 decodes
    localparam LD_OP        = {2'b00, 3'b000};     // 00 000
		localparam FENCE_OP     = {2'b00, 3'b011};     // 00 011    fence, fence.i
		localparam ALUI_OP      = {2'b00, 3'b100};     // 00 100
		localparam AUIPC_OP 		= {2'b00, 3'b101};	   // 00 101
    localparam ALUI64_OP    = {2'b00, 3'b110};     // 00 110    addiw, slliw. srliw, sraiw 
	
    localparam ST_OP        = {2'b01, 3'b000};     // 01 000
    localparam ALU_OP       = {2'b01, 3'b100};     // 01 100    includes M    (func7 used)
		localparam LUI_OP				= {2'b01, 3'b101};	   // 01 101  	LUI
    localparam ALU64_OP     = {2'b01, 3'b110};     // 01 110    addw, sllw, srlw, sraw (subw using b30)
	
    localparam BR_OP        = {2'b11, 3'b000};     // 11 000
		localparam JALR_OP 			= {2'b11, 3'b001};	   // 11 001
		localparam JAL_OP 			= {2'b11, 3'b011};	   // 11 011	
		localparam SPEC_OP      = {2'b11, 3'b100};     // 11 100    call, break, csr...



	

    //--bit30 qualifier for func7 (R) format (func7=32 or 0)
    `define SUB_vs_ADD    1
    `define SRA_vs_SRL    1

    //--I & R type func3 alu codes
    localparam  ADD_NN= 0;
    localparam  SLL_NN= 1;
    localparam  SLT_NN = 2;
    localparam  SLTU_NN=3;
    localparam  XOR_NN= 4;
    localparam  SRL_NN= 5;
    localparam  OR_NN = 6;
    localparam  AND_NN= 7;

    //--branch func3 codes
    localparam  BEQ_NN  =  0;
    localparam  BNE_NN   = 1;
    localparam  BLT_NN   = 4;
    localparam  BGE_NN   = 5;
    localparam  BLTU_NN  = 6;
    localparam  BGEU_NN  = 7;

    //--store (S type) func3 codes
    localparam  SB_NN    = 0;
    localparam  SH_NN    = 1;
    localparam  SW_NN    = 2;
    localparam  SD_NN    = 3;  // 64b

    //--load (I type) func3 codes
    localparam  LB_NN    = 0;
    localparam  LH_NN    = 1;
    localparam  LW_NN    = 2;
    localparam  LD_NN    = 3;  // 64b
    localparam  LBU_NN   = 4;
    localparam  LHU_NN   = 5;
    localparam  LWU_NN   = 6;  // 64b
	

    //--M type func3 codes
    localparam  MUL_NN   = 0;
    localparam  MULH_NN  = 1;
    localparam  MULHSU_NN= 2;
    localparam  MULHU_NN = 3;  
    localparam  DIV_NN   = 4;
    localparam  DIVU_NN  = 5;
    localparam  REM_NN   = 6;  
    localparam  REMU_NN  = 7;  



    //--csr func3 codes
    localparam  CSRRW_NN = 1;
    localparam  CSRRS_NN = 2;
    localparam  CSRRC_NN = 3;
    localparam  CSRRWI_NN= 5;  // uimm replaces r1 field for the "i" versions
    localparam  CSRRSI_NN= 6;
    localparam  CSRRCI_NN= 7;
	
    //--invalid func3 codes - to be used when a given function code does not exist for that opcode.  
	localparam 	INV_NN0 = 0;
	localparam 	INV_NN1 = 1;
	localparam 	INV_NN2 = 2;
	localparam 	INV_NN3 = 3;
	localparam 	INV_NN4 = 4;
	localparam 	INV_NN5 = 5;
	localparam 	INV_NN6 = 6;
	localparam 	INV_NN7 = 7;
	



