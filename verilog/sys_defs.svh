/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  sys_defs.svh                                        //
//                                                                     //
//  Description :  This file defines macros and data structures used   //
//                 throughout the processor.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`ifndef __SYS_DEFS_SVH__
`define __SYS_DEFS_SVH__

// all files should `include "sys_defs.svh" to at least define the timescale
`timescale 1ns/100ps

///////////////////////////////////
// ---- Starting Parameters ---- //
///////////////////////////////////

// some starting parameters that you should set
// this is *your* processor, you decide these values (try analyzing which is best!)


//`define DEBUG 1
// `define DEBUG_ROB '1
// `define DEBUG_MT '1
// `define DEBUG_FL '1
// `define DEBUG_RS '1

// superscalar width
`define N 3
`define CDB_SZ `N // This MUST match your superscalar width

`define PREDICTOR_EN 1
// `define ANALYTICS_EN 1

// sizes
`define ROB_SZ 32
`define RS_SZ 16
`define ARCH_REG_SZ 32
`define PHYS_REG_SZ_P6 32
`define PHYS_REG_SZ_R10K (32 + `ROB_SZ)

// sizes: these are directly correlated
`define BRANCH_HISTORY_REG_SZ 4
`define BRANCH_HISTORY_TABLE_SIZE 64 // 2^(BRANCH_HISTORY_REG_SZ)
`define BRANCH_TARGET_BUFFER_SIZE 64

// worry about these later
`define BRANCH_PRED_SZ 4
`define LSQ_SZ xx
`define SQ_SZ 16
`define LD_SZ 8
`define INST_BUFF_DEPTH 8

// functional units (you should decide if you want more or fewer types of FUs)
`define NUM_FU_ALU 4
`define NUM_FU_MULT 2
`define NUM_FU_LD 1
`define NUM_FU_STORE 1
`define NUM_FU_BR 1
`define NUM_FUS_CDB `NUM_FU_ALU + `NUM_FU_MULT + `LD_SZ + `NUM_FU_BR // 0 0 00 0000
`define NUM_FUS `NUM_FU_ALU + `NUM_FU_MULT + `NUM_FU_LD + `SQ_SZ + `NUM_FU_BR

// number of mult stages (2, 4) (you likely don't need 8)
`define MULT_STAGES 4

///////////////////////////////
// ---- Basic Constants ---- //
///////////////////////////////

// NOTE: the global CLOCK_PERIOD is defined in the Makefile

// useful boolean single-bit definitions
`define FALSE 1'h0
`define TRUE  1'h1

// word and register sizes
typedef logic [31:0] ADDR;
typedef logic [31:0] DATA;
typedef logic [4:0] REG_IDX;
typedef logic [$clog2(`PHYS_REG_SZ_R10K)-1:0] PHYS_REG_IDX;

// the zero register
// In RISC-V, any read of this register returns zero and any writes are thrown away
`define ZERO_REG 5'd0

// Basic NOP instruction. Allows pipline registers to clearly be reset with
// an instruction that does nothing instead of Zero which is really an ADDI x0, x0, 0
`define NOP 32'h00000013

//////////////////////////////////
// ---- Memory Definitions ---- //
//////////////////////////////////

// Cache mode removes the byte-level interface from memory, so it always returns
// a double word. The original processor won't work with this defined. Your new
// processor will have to account for this effect on mem.
// Notably, you can no longer write data without first reading.
// TODO: uncomment this line once you've implemented your cache
`define CACHE_MODE

// you are not allowed to change this definition for your final processor
// the project 3 processor has a massive boost in performance just from having no mem latency
// see if you can beat it's CPI in project 4 even with a 100ns latency!
//`define MEM_LATENCY_IN_CYCLES  0
`define MEM_LATENCY_IN_CYCLES (100.0/`CLOCK_PERIOD+0.49999)
// the 0.49999 is to force ceiling(100/period). The default behavior for
// float to integer conversion is rounding to nearest

// memory tags represent a unique id for outstanding mem transactions
// 0 is a sentinel value and is not a valid tag
`define NUM_MEM_TAGS 15
typedef logic [3:0] MEM_TAG;

// icache 
`define ICACHE_LINES 32
`define ICACHE_LINE_BITS $clog2(`ICACHE_LINES)

`define DCACHE_LINES 32
`define DCACHE_LINE_BITS $clog2(`DCACHE_LINES)
// Number of blocks to prefetch (not instructions - )
`define PREFETCH_DISTANCE 4

`define MEM_SIZE_IN_BYTES (64*1024)
`define MEM_64BIT_LINES   (`MEM_SIZE_IN_BYTES/8)

// A memory or cache block
typedef union packed {
    logic [7:0][7:0]  byte_level;
    logic [3:0][15:0] half_level;
    logic [1:0][31:0] word_level;
    logic      [63:0] dbbl_level;
} MEM_BLOCK;

typedef enum logic [1:0] {
    BYTE   = 2'h0,
    HALF   = 2'h1,
    WORD   = 2'h2,
    DOUBLE = 2'h3
} MEM_SIZE;

// Memory bus commands
typedef enum logic [1:0] {
    MEM_NONE   = 2'h0,
    MEM_LOAD   = 2'h1,
    MEM_STORE  = 2'h2
} MEM_COMMAND;

// icache tag struct
typedef struct packed {
    logic [12-`ICACHE_LINE_BITS:0] tags;
    logic                          valid;
    logic                          alloc;
} ICACHE_TAG;

typedef struct packed {
    logic [12-`DCACHE_LINE_BITS:0] tags;
    logic                          valid;
} DCACHE_TAG;

///////////////////////////////
// ---- Exception Codes ---- //
///////////////////////////////

/**
 * Exception codes for when something goes wrong in the processor.
 * Note that we use HALTED_ON_WFI to signify the end of computation.
 * It's original meaning is to 'Wait For an Interrupt', but we generally
 * ignore interrupts in 470
 *
 * This mostly follows the RISC-V Privileged spec
 * except a few add-ons for our infrastructure
 * The majority of them won't be used, but it's good to know what they are
 */

typedef enum logic [3:0] {
    INST_ADDR_MISALIGN  = 4'h0,
    INST_ACCESS_FAULT   = 4'h1,
    ILLEGAL_INST        = 4'h2,
    BREAKPOINT          = 4'h3,
    LOAD_ADDR_MISALIGN  = 4'h4,
    LOAD_ACCESS_FAULT   = 4'h5,
    STORE_ADDR_MISALIGN = 4'h6,
    STORE_ACCESS_FAULT  = 4'h7,
    ECALL_U_MODE        = 4'h8,
    ECALL_S_MODE        = 4'h9,
    NO_ERROR            = 4'ha, // a reserved code that we use to signal no errors
    ECALL_M_MODE        = 4'hb,
    INST_PAGE_FAULT     = 4'hc,
    LOAD_PAGE_FAULT     = 4'hd,
    HALTED_ON_WFI       = 4'he, // 'Wait For Interrupt'. In 470, signifies the end of computation
    STORE_PAGE_FAULT    = 4'hf
} EXCEPTION_CODE;

///////////////////////////////////
// ---- Instruction Typedef ---- //
///////////////////////////////////

// from the RISC-V ISA spec
typedef union packed {
    logic [31:0] inst;
    struct packed {
        logic [6:0] funct7;
        logic [4:0] rs2; // source register 2
        logic [4:0] rs1; // source register 1
        logic [2:0] funct3;
        logic [4:0] rd; // destination register
        logic [6:0] opcode;
    } r; // register-to-register instructions
    struct packed {
        logic [11:0] imm; // immediate value for calculating address
        logic [4:0]  rs1; // source register 1 (used as address base)
        logic [2:0]  funct3;
        logic [4:0]  rd;  // destination register
        logic [6:0]  opcode;
    } i; // immediate or load instructions
    struct packed {
        logic [6:0] off; // offset[11:5] for calculating address
        logic [4:0] rs2; // source register 2
        logic [4:0] rs1; // source register 1 (used as address base)
        logic [2:0] funct3;
        logic [4:0] set; // offset[4:0] for calculating address
        logic [6:0] opcode;
    } s; // store instructions
    struct packed {
        logic       of;  // offset[12]
        logic [5:0] s;   // offset[10:5]
        logic [4:0] rs2; // source register 2
        logic [4:0] rs1; // source register 1
        logic [2:0] funct3;
        logic [3:0] et;  // offset[4:1]
        logic       f;   // offset[11]
        logic [6:0] opcode;
    } b; // branch instructions
    struct packed {
        logic [19:0] imm; // immediate value
        logic [4:0]  rd; // destination register
        logic [6:0]  opcode;
    } u; // upper-immediate instructions
    struct packed {
        logic       of; // offset[20]
        logic [9:0] et; // offset[10:1]
        logic       s;  // offset[11]
        logic [7:0] f;  // offset[19:12]
        logic [4:0] rd; // destination register
        logic [6:0] opcode;
    } j;  // jump instructions

// extensions for other instruction types
`ifdef ATOMIC_EXT
    struct packed {
        logic [4:0] funct5;
        logic       aq;
        logic       rl;
        logic [4:0] rs2;
        logic [4:0] rs1;
        logic [2:0] funct3;
        logic [4:0] rd;
        logic [6:0] opcode;
    } a; // atomic instructions
`endif
`ifdef SYSTEM_EXT
    struct packed {
        logic [11:0] csr;
        logic [4:0]  rs1;
        logic [2:0]  funct3;
        logic [4:0]  rd;
        logic [6:0]  opcode;
    } sys; // system call instructions
`endif

} INST; // instruction typedef, this should cover all types of instructions

////////////////////////////////////////
// ---- Datapath Control Signals ---- //
////////////////////////////////////////

// ALU opA input mux selects
typedef enum logic [1:0] {
    OPA_IS_RS1  = 2'h0,
    OPA_IS_NPC  = 2'h1,
    OPA_IS_PC   = 2'h2,
    OPA_IS_ZERO = 2'h3
} ALU_OPA_SELECT;

// ALU opB input mux selects
typedef enum logic [3:0] {
    OPB_IS_RS2    = 4'h0,
    OPB_IS_I_IMM  = 4'h1,
    OPB_IS_S_IMM  = 4'h2,
    OPB_IS_B_IMM  = 4'h3,
    OPB_IS_U_IMM  = 4'h4,
    OPB_IS_J_IMM  = 4'h5
} ALU_OPB_SELECT;

// ALU function code
typedef enum logic [3:0] {
    ALU_ADD     = 4'h0,
    ALU_SUB     = 4'h1,
    ALU_SLT     = 4'h2,
    ALU_SLTU    = 4'h3,
    ALU_AND     = 4'h4,
    ALU_OR      = 4'h5,
    ALU_XOR     = 4'h6,
    ALU_SLL     = 4'h7,
    ALU_SRL     = 4'h8,
    ALU_SRA     = 4'h9
} ALU_FUNC;

// MULT funct3 code
// we don't include division or rem options
typedef enum logic [2:0] {
    M_MUL,
    M_MULH,
    M_MULHSU,
    M_MULHU
} MULT_FUNC;

////////////////////////////////
// ---- Datapath Packets ---- //
////////////////////////////////

/* P4 ADDED */

typedef enum logic [2:0] {
    ALU_INST,
    MULT_INST,
    LD_INST,
    STORE_INST,
    BR_INST
} FU_TYPE;

typedef enum logic [1:0] {
    NOTHING,
    CLEAR,
    SQUASH
} BR_TASK;

typedef enum logic [1:0] {
    STRONG_NT,
    NT,
    T,
    STRONG_T
} TWO_BIT_COUNTER;

typedef struct packed {
    PHYS_REG_IDX reg_idx;
    logic valid; // POST MILESTONE 2: DO WE NEED THIS
    logic ready;
} MAP_TABLE_PACKET;

typedef struct packed {
    PHYS_REG_IDX reg_idx;
    logic valid;
} FREE_LIST_PACKET;

typedef struct packed {
    REG_IDX reg_idx;
    PHYS_REG_IDX p_reg_idx;
    DATA reg_val;
    logic taken;
    logic valid;
} CDB_PACKET;

typedef struct packed {
    logic [`BRANCH_PRED_SZ-1:0] b_mask;
} BR_MASK;

typedef struct packed {
    logic valid;
    logic [`BRANCH_PRED_SZ-1:0] b_id;
    logic [`BRANCH_PRED_SZ-1:0] b_mask;
    MAP_TABLE_PACKET [`ARCH_REG_SZ-1:0] rec_mt;
    logic [$clog2(`ROB_SZ+1)-1:0] fl_head;
    logic [$clog2(`ROB_SZ)-1:0] rob_tail;
    logic [$clog2(`SQ_SZ)-1:0] sq_tail;
    logic [`BRANCH_HISTORY_REG_SZ-1:0] bhr;
    logic pred_taken;
} CHECKPOINT;

typedef enum logic [1:0] { 
    NONE = 0,
    WAITING_FOR_LOAD_DATA = 1,
    TRANSACTION_COMPLETE = 2
} MSHR_STATE;

typedef struct packed {
    MSHR_STATE state;

    ADDR addr;
    DATA data;

    MEM_TAG mem_tag;
    MEM_BLOCK mem_data;

    MEM_SIZE st_size;
    logic is_store;
} MSHR;

/* END */

/**
 * Packets are used to move many variables between modules with
 * just one datatype, but can be cumbersome in some circumstances.
 *
 * Define new ones in project 4 at your own discretion
 */

/**
 * IF_ID Packet:
 * Data exchanged from the IF to the ID stage
 */
typedef struct packed {
    INST  inst;
    ADDR  PC;
    ADDR  NPC; // PC + 4
    logic [`BRANCH_HISTORY_REG_SZ-1:0] bhr;
    logic valid;
    logic pred_taken;
} INST_PACKET;

/**
 * ID_EX Packet:
 * Data exchanged from the ID to the EX stage
 */
typedef struct packed {
    logic valid;
    INST inst;
    ADDR PC;
    ADDR NPC; // PC + 4
    logic [`BRANCH_HISTORY_REG_SZ-1:0] bhr;

    FU_TYPE fu_type;
    REG_IDX reg1;
    REG_IDX reg2;

    ALU_OPA_SELECT opa_select; // ALU opa mux select (ALU_OPA_xxx *)
    ALU_OPB_SELECT opb_select; // ALU opb mux select (ALU_OPB_xxx *)

    REG_IDX  dest_reg_idx;  // destination (writeback) register index
    ALU_FUNC alu_func;      // ALU function select (ALU_xxx *)
    logic    mult;          // Is inst a multiply instruction?
    logic    rd_mem;        // Does inst read memory?
    logic    wr_mem;        // Does inst write memory?
    logic    cond_branch;   // Is inst a conditional branch?
    logic    uncond_branch; // Is inst an unconditional branch?
    logic    halt;          // Is this a halt?
    logic    illegal;       // Is this instruction illegal?
    logic    csr_op;        // Is this a CSR operation? (we only used this as a cheap way to get return code)
    logic    taken;
    logic    pred_taken;

    logic [$clog2(`SQ_SZ)-1:0] sq_tail;
} DECODED_PACKET;

typedef struct packed {
    DECODED_PACKET decoded_vals;

    /* P4 ADDED STUFF */
    FREE_LIST_PACKET t;
    MAP_TABLE_PACKET t1;
    MAP_TABLE_PACKET t2;
    BR_MASK b_id;
    BR_MASK b_mask;

    logic ld_ready;
    /* END */
} RS_PACKET;

typedef struct packed {
    RS_PACKET decoded_vals;

    DATA rs1_value; // reg A value
    DATA rs2_value; // reg B value
} ISSUE_PACKET;

typedef enum logic [2:0] {
    EMPTY,
    READY_TO_QUERY,
    READY_TO_ISSUE, // waiting for DM to be ready
    WAITING_FOR_DATA, // DM transaction in flight
    DATA_READY // waiting for CDB to be ready
} LOAD_STATE;

typedef struct packed {
    RS_PACKET decoded_vals;

    DATA result;

    logic pred_correct;

    DATA target_addr;
    DATA rs2_value;
    LOAD_STATE ld_state;
} FU_PACKET;

/**
 * EX_MEM Packet:
 * Data exchanged from the EX to the MEM stage
 */
typedef struct packed {
    DATA alu_result;
    ADDR NPC;

    logic    take_branch; // Is this a taken branch?
    // Pass-through from decode stage
    DATA     rs2_value;
    logic    rd_mem;
    logic    wr_mem;
    REG_IDX  dest_reg_idx;
    logic    halt;
    logic    illegal;
    logic    csr_op;
    logic    rd_unsigned; // Whether proc2Dmem_data is signed or unsigned
    MEM_SIZE mem_size;
    logic    valid;
} EX_MEM_PACKET;

/**
 * MEM_WB Packet:
 * Data exchanged from the MEM to the WB stage
 *
 * Does not include data sent from the MEM stage to memory
 */
typedef struct packed {
    DATA    result;
    ADDR    NPC;
    REG_IDX dest_reg_idx; // writeback destination (ZERO_REG if no writeback)
    logic   take_branch;
    logic   halt;    // not used by wb stage
    logic   illegal; // not used by wb stage
    logic   valid;
} MEM_WB_PACKET;

/**
 * Commit Packet:
 * This is an output of the processor and used in the testbench for counting
 * committed instructions
 *
 * It also acts as a "WB_PACKET", and can be reused in the final project with
 * some slight changes
 */
typedef struct packed {
    ADDR    NPC;
    DATA    data;
    REG_IDX reg_idx;
    logic   halt;
    logic   illegal;
    logic   valid;
    `ifdef DEBUG
    PHYS_REG_IDX tag;
    `endif
} COMMIT_PACKET;

typedef struct packed {
    ADDR            PC;
    REG_IDX         dest_reg_idx;
    logic           halt;
    PHYS_REG_IDX    t;
    PHYS_REG_IDX    t_old; // look up t_old in arch map table to get arch reg and update to t on retire
    logic           complete;
    logic           valid;
    logic           is_branch;
    logic           taken;
    logic           pred_taken;

    logic           wr_mem;
    DATA            data;
} ROB_PACKET;

`endif // __SYS_DEFS_SVH__
