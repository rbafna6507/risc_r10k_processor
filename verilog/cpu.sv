/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  cpu.sv                                              //
//                                                                     //
//  Description :  Top-level module of the verisimple processor;       //
//                 This instantiates and connects the 5 stages of the  //
//                 Verisimple pipeline together.                       //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"


module cpu (
    input                                                                                       clock, // System clock
    input                                                                                       reset, // System reset

    input MEM_TAG   mem2proc_transaction_tag, // Memory tag for current transaction [current transaction]
    input MEM_BLOCK mem2proc_data,            // Data coming back from memory
    input MEM_TAG   mem2proc_data_tag,        // Tag for which finished transaction data is for

    output MEM_COMMAND proc2mem_command, // Command sent to memory
    output ADDR        proc2mem_addr,    // Address sent to memory
    output MEM_BLOCK   proc2mem_data,    // Data sent to memory
    output MEM_SIZE    proc2mem_size,    // Data size sent to memory

    // Note: these are assigned at the very bottom of the modulo
    output COMMIT_PACKET                [`N-1:0]                                                committed_insts,
    output ROB_PACKET                   [`N-1:0]                                                retired_insts,

    output logic                        [3:0]                                                   ib_open,
    output ADDR                                                                                 NPC

    `ifdef ANALYTICS_EN
    ,   output logic                                                                                pred_valid,
        output logic                                                                                pred_correct
    `endif

    `ifdef DEBUG
    ,   output logic                    [`BRANCH_HISTORY_REG_SZ-1:0]                            debug_bhr,
        output logic                    [$clog2(`N+1)-1:0]                                      debug_num_dispatched,
        output logic                    [$clog2(`N+1)-1:0]                                      debug_num_retired,

        output INST_PACKET              [`INST_BUFF_DEPTH-1:0]                                  debug_inst_buff_entries,
        output logic                    [$clog2(`INST_BUFF_DEPTH)-1:0]                          debug_inst_buff_head,
        output logic                    [$clog2(`INST_BUFF_DEPTH)-1:0]                          debug_inst_buff_tail,

        output DECODED_PACKET           [`N-1:0]                                                debug_dis_insts,
        output logic                    [$clog2(`N+1)-1:0]                                      debug_dispatch_limit,
        output logic                    [$clog2(`N+1)-1:0]                                      debug_num_store_dispatched,

        output FREE_LIST_PACKET         [`ROB_SZ-1:0]                                           debug_fl_entries,
        output logic                    [$clog2(`ROB_SZ)-1:0]                                   debug_fl_head,
        output logic                    [$clog2(`ROB_SZ)-1:0]                                   debug_fl_tail,

        output MAP_TABLE_PACKET         [`ARCH_REG_SZ-1:0]                                      debug_mt_entries,

        output RS_PACKET                [`RS_SZ-1:0]                                            debug_rs_entries,
        output logic                    [`RS_SZ-1:0]                                            debug_rs_open_spots,
        output logic                    [`RS_SZ-1:0]                                            debug_rs_other_sig,
        output logic                    [$clog2(`RS_SZ+1)-1:0]                                  debug_rs_open_entries,
        output logic                    [`RS_SZ-1:0]                                            debug_rs_all_issued_insts,
        output logic                    [`RS_SZ-1:0]                                            debug_all_issued_alu,
        output logic                    [`RS_SZ-1:0]                                            debug_all_issued_mult,
        output logic                    [`RS_SZ-1:0]                                            debug_all_issued_br,
        output logic                    [`RS_SZ-1:0]                                            debug_all_issued_st,
        output logic                    [`RS_SZ-1:0]                                            debug_all_issued_ld,
        output BR_MASK                                                                          debug_rs_br_mask,

        output ROB_PACKET               [`ROB_SZ-1:0]                                           debug_rob_entries,
        output logic                    [$clog2(`ROB_SZ)-1:0]                                   debug_rob_head,
        output logic                    [$clog2(`ROB_SZ)-1:0]                                   debug_rob_tail,
        output logic                    [$clog2(`ROB_SZ)-1:0]                                   debug_rob_num_entries,

        output CHECKPOINT               [`BRANCH_PRED_SZ-1:0]                                   debug_bs_entries,
        output logic                    [`BRANCH_PRED_SZ-1:0]                                   debug_bs_free_entries,
        output logic                    [`BRANCH_PRED_SZ-1:0]                                   debug_bs_stack_gnt,

        output CDB_PACKET               [`N-1:0]                                                debug_cdb_entries,
        output logic                    [`NUM_FUS_CDB-1:0]                                      debug_cdb_gnt,
        output logic                    [`N-1:0][`NUM_FUS_CDB-1:0]                              debug_cdb_gnt_bus,
        output logic                    [`NUM_FUS_CDB-1:0]                                      debug_cdb_fu_done,
        output logic                    [`NUM_FUS_CDB-1:0]                                      debug_cdb_stall_sig,

        output logic                    [`NUM_FU_ALU-1:0]                                       debug_alu_done,
        output logic                    [`NUM_FU_MULT-1:0]                                      debug_mult_done,
        output logic                    [`NUM_FU_MULT-1:0]                                      debug_mult_rd_en,

        output ISSUE_PACKET             [`NUM_FU_ALU-1:0]                                       debug_issued_alu_pack,
        output ISSUE_PACKET             [`NUM_FU_MULT-1:0]                                      debug_issued_mult_pack,
        output ISSUE_PACKET                                                                     debug_issued_br_pack,
        output ISSUE_PACKET             [`SQ_SZ-1:0]                                            debug_issued_st_pack,
        output ISSUE_PACKET                                                                     debug_issued_ld_pack,

        output logic                    [$clog2(`SQ_SZ)-1:0]                                    debug_sq_head,
        output logic                    [$clog2(`SQ_SZ)-1:0]                                    debug_sq_tail,
        output logic                    [$clog2(`N+1)-1:0]                                      debug_sq_open,

        output logic                                                                            debug_start_store,

        output FU_PACKET                [`SQ_SZ-1:0]                                            debug_sq_entries,
        output logic                    [$clog2(`SQ_SZ+1)-1:0]                                  debug_sq_num_entries,
        output logic                    [$clog2(`SQ_SZ+1)-1:0]                                  debug_sq_br_tail,

        output logic                    [`NUM_FU_LD-1:0]                                        debug_ld_rd_en,
        output FU_PACKET                [`LD_SZ-1:0]                                            debug_ld_entries,
        output logic                    [`LD_SZ-1:0]                                            debug_ld_open_spots,
        output logic                    [`LD_SZ-1:0]                                            debug_ld_ready_spots,
        output logic                    [`LD_SZ-1:0]                                            debug_ld_alloc_spot,
        output logic                    [`LD_SZ-1:0]                                            debug_ld_issued_entry,
        output logic                    [`LD_SZ-1:0]                                            debug_ld_broadcast_entry,
        output logic                                                                            debug_ld_full,
        output logic                    [`LD_SZ-1:0]                                            debug_ld_stall_sig,
        output logic                    [`LD_SZ-1:0]                                            debug_ld_squashed,

        output logic                                                                            debug_Dcache_ld_out,
        output ADDR                                                                             debug_Dcache_addr_out,
        output logic                                                                            debug_mshr2cache_wr,

        output MSHR                                                                             debug_mshr,
        output DCACHE_TAG               [`DCACHE_LINES-1:0]                                     debug_dcache_tags,
        output logic                    [`NUM_FU_ALU-1:0]                                       debug_alu_rd_en,
        output logic                    [`NUM_FU_ALU-1:0][`RS_SZ-1:0]                           debug_alu_issued_bus,

        output FU_PACKET                [`NUM_FU_ALU-1:0]                                       debug_alu_data,
        output FU_PACKET                [`NUM_FU_ALU-1:0]                                       debug_alu_next_data,
        output logic                                                                            debug_sq_full,

        output ADDR                                                                             debug_fetch_target,
        output logic                                                                            debug_fetch_arbiter_signal, 
        output BR_TASK                                                                          debug_fetch_br_task,  
        output logic                    [$clog2(`INST_BUFF_DEPTH+1)-1:0]                        debug_fetch_ibuff_open,

        output MEM_TAG                                                                          debug_fetch_mem_transaction_tag, 
        output MEM_TAG                                                                          debug_fetch_mem_data_tag, 
        output MEM_BLOCK                                                                        debug_fetch_mem_data, 

        output logic                                                                            debug_fetch_mem_en,
        output ADDR                                                                             debug_fetch_mem_addr_out, 

        output INST_PACKET              [3:0]                                                   debug_fetch_out_insts, 
        output logic                    [2:0]                                                   debug_fetch_out_num_insts,

        output ADDR                     [`NUM_MEM_TAGS:1]                                       debug_mshr_data,
        output logic                    [`NUM_MEM_TAGS:1]                                       debug_mshr_valid,
        output MEM_BLOCK                [`PREFETCH_DISTANCE-1:0]                                 debug_icache_data,
        output logic                    [`PREFETCH_DISTANCE-1:0]                                 debug_icache_valid,
        output ADDR                     [`PREFETCH_DISTANCE-1:0]                                debug_icache_raddr
    `endif
);

    //////////////////////////////////////////////////
    //                                              //
    //               pipeline wires                 //
    //                                              //
    //////////////////////////////////////////////////


    // output of fetch?
    logic fetch_bhr_wr_en;
    logic fetch_bhr_wr_taken;

    // output of bhr
    logic [`BRANCH_HISTORY_REG_SZ-1:0] out_bhr;

    // output of ib
    INST_PACKET [`N-1:0] ib_insts;


    // output of dispatch
    DECODED_PACKET [`N-1:0] dis_insts;
    logic [$clog2(`N+1)-1:0] num_dis;
    // dispatch helpers
    REG_IDX      [`N-1:0] dis_r1_idx;
    REG_IDX      [`N-1:0] dis_r2_idx;
    REG_IDX      [`N-1:0] dis_dest_reg_idx; // dest_regs that are getting mapped to a new phys_reg from free_list
    PHYS_REG_IDX [`N-1:0] dis_free_reg;  // comes from the free list
    logic        [`N-1:0] dis_incoming_valid;
    logic        [$clog2(`N+1)-1:0] num_store_dispatched; // # of dispatched isntructions are stores


    // output of RS
    logic        [$clog2(`N+1)-1:0]     rs_open;
    RS_PACKET    [`NUM_FU_ALU-1:0]      issued_alu;
    RS_PACKET    [`NUM_FU_MULT-1:0]     issued_mult;
    RS_PACKET    [`NUM_FU_LD-1:0]       issued_ld;
    RS_PACKET    [`SQ_SZ-1:0]           issued_store;
    RS_PACKET                           issued_br;

    MEM_COMMAND d_proc2mem_command;
    ADDR        d_proc2mem_addr, fetch_proc2mem_addr;

    // output of ROB
    logic [$clog2(`N+1)-1:0] rob_open, num_retired;
    ROB_PACKET [`N-1:0] retiring_data; // rob entry packet, but want register vals to update architectural map table + free list
    logic [$clog2(`ROB_SZ)-1:0] rob_tail;
    logic start_store;
    // commit helpers
    FREE_LIST_PACKET [`N-1:0] retiring_t_old;

    // output of SQ
    logic [$clog2(`SQ_SZ)-1:0] sq_head, sq_tail;
    logic [$clog2(`N+1)-1:0] sq_open;
    ADDR Dmem_st_addr, Dmem_ld_addr, Dmem_addr;
    DATA Dmem_store_data;
    MEM_SIZE Dmem_size;

    // output of MT
    PHYS_REG_IDX             [`N-1:0]             t_old_data;
    MAP_TABLE_PACKET         [`N-1:0]             r1_p_reg;
    MAP_TABLE_PACKET         [`N-1:0]             r2_p_reg;
    MAP_TABLE_PACKET         [`ARCH_REG_SZ-1:0]     out_mt; // CHECK: this size does not match up to branch stack in_mt


    // output of freelist
    FREE_LIST_PACKET [`N-1:0]                 fl_reg; // displayed available reg idxs, these are always output, and only updated based on rd_num
    logic            [$clog2(`ROB_SZ+1)-1:0]  fl_head_ptr;


    // output of cdb
    CDB_PACKET [`N-1:0] cdb_entries;
    logic [`NUM_FUS_CDB-1:0] cdb_stall_sig;
    // cdb helpers
    REG_IDX         [`N-1:0] cdb_reg_idx;
    PHYS_REG_IDX    [`N-1:0] cdb_p_reg_idx;
    logic           [`N-1:0] cdb_valid;
    DATA            [`N-1:0] cdb_wr_data;


    // output of br stack
    CHECKPOINT  cp_out;
    logic br_full;
    BR_MASK assigned_b_id, assigned_b_mask;


    // output of regfile
    DATA  [`NUM_FUS-1:0] reg_data_1, reg_data_2;


    // out of issue
    logic          [`NUM_FU_ALU-1:0]        alu_rd_en;
    logic          [`NUM_FU_MULT-1:0]       mult_rd_en;
    logic          [`NUM_FU_LD-1:0]         ld_rd_en;
    logic          [`SQ_SZ-1:0]             st_rd_en;
    logic                                   br_rd_en;

    ISSUE_PACKET   [`NUM_FU_ALU-1:0]        issued_alu_pack;
    ISSUE_PACKET   [`NUM_FU_MULT-1:0]       issued_mult_pack;
    ISSUE_PACKET   [`NUM_FU_LD-1:0]         issued_ld_pack;
    ISSUE_PACKET   [`SQ_SZ-1:0]             issued_st_pack;
    ISSUE_PACKET                            issued_br_pack;

    PHYS_REG_IDX   [`NUM_FUS-1:0]           reg_idx_1, reg_idx_2;


    // output of alu
    FU_PACKET [`NUM_FU_ALU-1:0] alu_fu_out;
    logic     [`NUM_FU_ALU-1:0] alu_done;


    // output of mult
    FU_PACKET [`NUM_FU_MULT-1:0] mult_fu_out;
    logic     [`NUM_FU_MULT-1:0] mult_done;
    logic     [`NUM_FU_MULT-1:0] mult_busy;

    logic sq_full;

    logic ld_full, start_load;
    FU_PACKET [`LD_SZ-1:0] ld_fu_out;
    logic [`LD_SZ-1:0] ld_done;
    // assign ld_done = '0;
    // assign ld_fu_out = '0;


    // output of branch fu
    FU_PACKET br_fu_out;
    BR_TASK   br_task;
    logic     br_done;
    logic     br_taken;

    // output of dcache
    MEM_BLOCK Dcache_data_out; // this is for cache hit on a load inst (miss data will come from mshr)
    logic     Dcache_hit_out; // When valid is high
    ADDR      Dcache_addr_out;
    logic     Dcache_ld_out; 
    // helpers for dcache
    logic is_store;
    MEM_SIZE st_size;
    DATA in_data;
    ADDR proc2Dcache_addr;

    // output of mshr
    ADDR        mshr2cache_addr;
    DATA        mshr2cache_data;
    MEM_BLOCK   mshr2cache_mem_block;
    MEM_SIZE    mshr2cache_st_size;
    logic       mshr2cache_is_store;
    logic       mshr2cache_wr;
    logic       mshr_stall;
    logic       mshr_squash;
    logic       valid_mem_inst;


    `ifdef DEBUG
        assign debug_bhr = out_bhr;
        assign debug_dis_insts = dis_insts;
        assign debug_num_dispatched = num_dis;
        assign debug_num_retired = num_retired;
        assign debug_cdb_entries = cdb_entries;
        assign debug_cdb_stall_sig = cdb_stall_sig;
        assign debug_alu_done = alu_done;
        assign debug_mult_done = mult_done;
        assign debug_mult_rd_en = mult_rd_en;
        assign debug_issued_alu_pack = issued_alu_pack;
        assign debug_issued_mult_pack = issued_mult_pack;
        assign debug_issued_br_pack = issued_br_pack;
        assign debug_issued_st_pack = issued_st_pack;
        assign debug_issued_ld_pack = issued_ld_pack;

        assign debug_sq_head = sq_head;
        assign debug_sq_tail = sq_tail;
        assign debug_sq_open = sq_open;

        assign debug_num_store_dispatched = num_store_dispatched;
        assign debug_start_store = start_store;

        assign debug_ld_rd_en = ld_rd_en;
        assign debug_ld_full = ld_full;

        assign debug_Dcache_addr_out = Dcache_addr_out;
        assign debug_Dcache_ld_out = Dcache_ld_out;
        assign debug_mshr2cache_wr = mshr2cache_wr;

        assign debug_alu_rd_en = alu_rd_en;
        assign debug_sq_full = sq_full;
        assign debug_sq_br_tail = cp_out.sq_tail;
    `endif

    //////////////////////////////////////////////////
    //                                              //
    //          john and rohan trying               //
    //                                              //
    //////////////////////////////////////////////////

    logic fetch_mem_en;
    //assign proc2mem_command = fetch_mem_en & ~reset; // TODO replace with arbiter
    assign proc2mem_command = d_proc2mem_command == MEM_NONE ? (fetch_mem_en ? MEM_LOAD : MEM_NONE) : d_proc2mem_command;
    assign proc2mem_addr = d_proc2mem_command == MEM_NONE ? (fetch_mem_en ? fetch_proc2mem_addr : '0) : d_proc2mem_addr;

    logic [2:0] num_input;
    INST_PACKET [3:0] in_insts;

    bhr goop (
        .clock(clock),
        .reset(reset),

        .wr_en(fetch_bhr_wr_en),
        .taken(fetch_bhr_wr_taken),

        .br_task(br_task),
        .br_checkpoint_bhr(cp_out.bhr),
        .br_pred_taken(cp_out.pred_taken),

        .out_bhr(out_bhr)
    );


    fetch rufus (
        .clock(clock),
        .reset(reset),

        .target(br_fu_out.target_addr),
        .arbiter_signal(d_proc2mem_command == MEM_NONE), 
        .br_task(br_task),
        .ibuff_open(ib_open),
        .mem_transaction_tag(mem2proc_transaction_tag),
        .mem_data_tag(mem2proc_data_tag),
        .mem_data(mem2proc_data),

        .rd_bhr(out_bhr),

        .pred_wr_en(br_task == SQUASH & br_taken),
        .pred_wr_taken(br_taken),
        .pred_wr_target(br_fu_out.target_addr),
        .pred_wr_pc(br_fu_out.decoded_vals.decoded_vals.PC),
        .pred_wr_bhr(br_fu_out.decoded_vals.decoded_vals.bhr),

        .mem_en(fetch_mem_en),
        .mem_addr_out(fetch_proc2mem_addr),

        .bhr_wr_en(fetch_bhr_wr_en),
        .bhr_wr_taken(fetch_bhr_wr_taken),

        .out_insts(in_insts),
        .out_num_insts(num_input),
        .NPC_out(NPC)
        `ifdef DEBUG
        ,   .debug_mshr_data(debug_mshr_data),
            .debug_mshr_valid(debug_mshr_valid),
            .Icache_data_out(debug_icache_data),
            .Icache_valid_out(debug_icache_valid),
            .debug_icache_raddr(debug_icache_raddr)
        `endif
    );

    `ifdef DEBUG
        assign debug_fetch_target               = NPC;
        assign debug_fetch_arbiter_signal       = 1'b1;
        assign debug_fetch_br_task              = br_task;
        assign debug_fetch_ibuff_open           = ib_open;

        assign debug_fetch_mem_transaction_tag  = mem2proc_transaction_tag;
        assign debug_fetch_mem_data_tag         = mem2proc_data_tag;
        assign debug_fetch_mem_data             = mem2proc_data;

        assign debug_fetch_mem_en               = fetch_mem_en;
        assign debug_fetch_mem_addr_out         = proc2mem_addr;

        assign debug_fetch_out_insts            = in_insts;
        assign debug_fetch_out_num_insts        = num_input;
    `endif


    inst_buffer buffet (
        .clock(clock),
        .reset(reset),

        .in_insts(in_insts),
        .num_dispatch(num_dis),
        .num_accept(num_input),
        .br_en(br_done & ~br_fu_out.pred_correct), 

        .dispatched_insts(ib_insts),
        .open_entries(ib_open)

        `ifdef DEBUG
        ,   .debug_entries(debug_inst_buff_entries),
            .debug_head(debug_inst_buff_head),
            .debug_tail(debug_inst_buff_tail)
        `endif
    );

    dispatch disbitch (
        .clock(clock),
        .reset(reset),
        .rob_open(rob_open),
        .rs_open(rs_open),
        .insts(ib_insts),
        .sq_tail_in(sq_tail),
        .bs_full(br_full),
        .sq_full(sq_full),

        .num_dispatch(num_dis),
        .num_store_dispatched(num_store_dispatched),
        .out_insts(dis_insts)

        `ifdef DEBUG
        ,   .debug_dispatch_limit(debug_dispatch_limit)
        `endif
    );

    freelist flo_from_progressive (
        .clock(clock),
        .reset(reset),

        .rd_num(num_dis),  // number of regs to take off of the free list
        .wr_num(num_retired),  // number of regs to add back to the free list
        .wr_reg(retiring_t_old),  // reg idxs to add to free list
        .br_en(br_done & ~br_fu_out.pred_correct),  // enable signal for EBR
        .head_ptr_in(cp_out.fl_head),  // free list copy for EBR

        .rd_reg(fl_reg),
        .head_ptr(fl_head_ptr)

        `ifdef DEBUG
        ,   .debug_entries(debug_fl_entries),
            .debug_head(debug_fl_head),
            .debug_tail(debug_fl_tail)
        `endif
    );

    map_table im_the_map (
        .clock(clock),
        .reset(reset),

        .r1_idx(dis_r1_idx),
        .r2_idx(dis_r2_idx),
        .dest_reg_idx(dis_dest_reg_idx), // dest_regs that are getting mapped to a new phys_reg from free_list
        .free_reg(dis_free_reg),  // comes from the free list
        .incoming_valid(dis_incoming_valid), // inputs to expect

        .ready_reg_idx(cdb_reg_idx), // readys from CDB - arch reg
        .ready_phys_idx(cdb_p_reg_idx), // corresponding phys reg
        .ready_valid(cdb_valid), // one hot encoded inputs to expect

        .in_mt_en(br_done & ~br_fu_out.pred_correct),
        .in_mt(cp_out.rec_mt),//cp.rec_mt),

        .t_old_data(t_old_data), //?
        .r1_p_reg(r1_p_reg),
        .r2_p_reg(r2_p_reg),
        .out_mt(out_mt)

        `ifdef DEBUG
        ,   .debug_entries(debug_mt_entries)
        `endif
    );

    logic [`NUM_FU_ALU-1:0] fu_alu_busy;
    assign fu_alu_busy = cdb_stall_sig[`NUM_FU_ALU-1:0] | alu_rd_en;

    logic [`NUM_FU_MULT-1:0] fu_mult_busy;
    assign fu_mult_busy = cdb_stall_sig[`NUM_FU_ALU+`NUM_FU_MULT-1:`NUM_FU_ALU] | mult_busy;

    rs rasam (
        .clock(clock),
        .reset(reset),

        .rs_in(dis_insts),
        .t_in(fl_reg),
        .t1_in(r1_p_reg),
        .t2_in(r2_p_reg),
        .b_id(assigned_b_id),
        .b_mask(assigned_b_mask),

        .cdb_in(cdb_entries),

        .sq_head_in(sq_head),
        .start_store(start_store),

    // ebr logic
        .rem_b_id(br_fu_out.decoded_vals.b_id),
        .br_task(br_task),

        // busy bits from FUs to mark when available to issue
        .fu_alu_busy(fu_alu_busy),
        .fu_mult_busy(fu_mult_busy),
        .fu_ld_busy(ld_full),
        .fu_br_busy(1'b0),

        .num_accept(num_dis),

        // output packets directly to FUs (they all are pipelined)
        .issued_alu(issued_alu),
        .issued_mult(issued_mult),
        .issued_ld(issued_ld),
        .issued_store(issued_store),
        .issued_br(issued_br),

        .open_entries(rs_open)

        `ifdef DEBUG
        ,   .debug_entries(debug_rs_entries),
            .debug_open_spots(debug_rs_open_spots),
            .debug_other_sig(debug_rs_other_sig),
            .debug_open_entries(debug_rs_open_entries),
            .debug_all_issued_insts(debug_rs_all_issued_insts),
            .debug_all_issued_alu(debug_all_issued_alu),
            .debug_all_issued_mult(debug_all_issued_mult),
            .debug_all_issued_br(debug_all_issued_br),
            .debug_all_issued_ld(debug_all_issued_ld),
            .debug_all_issued_st(debug_all_issued_st),
            .debug_b_mask(debug_rs_br_mask),
            .debug_alu_issued_bus(debug_alu_issued_bus)
        `endif
    );

    rob robert (
        .clock(clock),
        .reset(reset),

        .wr_data(dis_insts),
        .complete_taken(br_taken),
        .t(dis_free_reg),
        .t_old(t_old_data),

        .complete_t(cdb_p_reg_idx), // comes from the CDB
        .store_complete_t(issued_st_pack),
        .num_accept(num_dis), // input signal from min block, dependent on open_entries 
        .br_tail(cp_out.rob_tail),
        .br_en(br_done & ~br_fu_out.pred_correct),
        .dm_stalled(mshr_stall),
        .cdb_wr_data(cdb_wr_data),

        .retiring_data(retiring_data), // rob entry packet, but want register vals to update architectural map table + free list
        .open_entries(rob_open), // number of open entires AFTER retirement
        .num_retired(num_retired),
        .out_tail(rob_tail),
        .start_store(start_store)

        `ifdef DEBUG
        ,   .debug_entries(debug_rob_entries),
            .debug_head(debug_rob_head),
            .debug_tail(debug_rob_tail),
            .debug_num_entries(debug_rob_num_entries)
        `endif
    );
    assign retired_insts = retiring_data;

    cdb cbd (
        .clock(clock),
        .reset(reset),

        .fu_done({br_done, ld_done, mult_done, alu_done}),
        .wr_data({br_fu_out, ld_fu_out, mult_fu_out, alu_fu_out}),
        .taken(br_taken),

        .rem_br_task(br_task),
        .rem_b_id(br_fu_out.decoded_vals.b_id),

        .entries(cdb_entries),
        .stall_sig(cdb_stall_sig)

        `ifdef DEBUG
        ,   .debug_cdb_gnt(debug_cdb_gnt),
            .debug_cdb_gnt_bus(debug_cdb_gnt_bus)
        `endif
    );

    `ifdef DEBUG
        assign debug_cdb_fu_done = {br_done, ld_done, mult_done, alu_done};
    `endif

    br_stack pancake (
        .clock(clock),
        .reset(reset),

        .dis_inst(dis_insts[0]),
        .branch_t(dis_free_reg[0]),
        .in_mt(out_mt),
        .in_fl_head(fl_head_ptr),
        .in_rob_tail(rob_tail),
        .in_sq_tail(sq_tail),

        .cdb_in(cdb_entries),

        .br_task(br_task), // not defined here. in main sysdefs
        .rem_b_id(br_fu_out.decoded_vals.b_id), // b_id to remove

        .assigned_b_id(assigned_b_id),
        .assigned_b_mask(assigned_b_mask),
        .cp_out(cp_out),
        .full(br_full)

        `ifdef DEBUG
        ,   .debug_entries(debug_bs_entries),
            .debug_free_entries(debug_bs_free_entries),
            .debug_stack_gnt(debug_bs_stack_gnt)
        `endif
    );

    regfile reggie (
        .clock(clock), // system clock
        .reset(reset),

        .read_idx_1(reg_idx_1),
        .read_idx_2(reg_idx_2),
        .write_idx(cdb_p_reg_idx),
        .write_en(cdb_valid),
        .write_data(cdb_wr_data),

        .read_out_1(reg_data_1),
        .read_out_2(reg_data_2)
    );

    //////////////////////////////////////////////////
    //                                              //
    //                   dispatch                   //
    //                                              //
    //////////////////////////////////////////////////

    always_comb begin
        for (int i = 0; i < `N; i++) begin
            dis_r1_idx[i] = dis_insts[i].reg1;
            dis_r2_idx[i] = dis_insts[i].reg2;
            dis_dest_reg_idx[i] = dis_insts[i].dest_reg_idx; // dest_regs that are getting mapped to a new phys_reg from free_list
            dis_free_reg[i] = fl_reg[i].reg_idx;  // comes from the free list
            dis_incoming_valid[i] = dis_insts[i].valid;
        end
    end

    //////////////////////////////////////////////////
    //                                              //
    //                    issue                     //
    //                                              //
    //////////////////////////////////////////////////

    issue anup (
        .clock(clock),
        .reset(reset),

        .reg_data_1(reg_data_1),
        .reg_data_2(reg_data_2),

        .issued_alu(issued_alu),
        .issued_mult(issued_mult),
        .issued_ld(issued_ld),
        .issued_st(issued_store),
        .issued_br(issued_br),

        .alu_rd_en(alu_rd_en),
        .mult_rd_en(mult_rd_en),
        .ld_rd_en(ld_rd_en),
        .st_rd_en(st_rd_en),
        .br_rd_en(br_rd_en),

        .issued_alu_pack(issued_alu_pack),
        .issued_mult_pack(issued_mult_pack),
        .issued_ld_pack(issued_ld_pack),
        .issued_st_pack(issued_st_pack),
        .issued_br_pack(issued_br_pack),

        .reg_idx_1(reg_idx_1),
        .reg_idx_2(reg_idx_2)
    );

    //////////////////////////////////////////////////
    //                                              //
    //                  execution                   //
    //                                              //
    //////////////////////////////////////////////////

    generate
        for (genvar i = 0; i < `NUM_FU_ALU; i++) begin
            alu what_the (
                .clock(clock),
                .reset(reset),
                .is_pack(issued_alu_pack[i]),
                .stall(cdb_stall_sig[i]),
                .rd_in(alu_rd_en[i]),

                .rem_br_task(br_task),
                .rem_b_id(br_fu_out.decoded_vals.b_id),

                .fu_pack(alu_fu_out[i]),
                .data_ready(alu_done[i])

                `ifdef DEBUG
                ,   .debug_data(debug_alu_data[i]),
                    .debug_next_data(debug_alu_next_data[i])
                `endif
            );
        end
    endgenerate

    generate
        for (genvar i = 0; i < `NUM_FU_MULT; i++) begin
            mult what_the_fck (
                .clock(clock),
                .reset(reset),
                .is_pack(issued_mult_pack[i]),
                .stall(cdb_stall_sig[`NUM_FU_ALU + i]),
                .rd_in(mult_rd_en[i]),

                .rem_br_task(br_task),
                .rem_b_id(br_fu_out.decoded_vals.b_id),

                .fu_pack(mult_fu_out[i]),
                .data_ready(mult_done[i]),
                .busy(mult_busy[i])
            );
        end
    endgenerate

    branch_fu what_the_duck (
        .clock(clock),
        .reset(reset),
        .is_pack(issued_br_pack),
        .rd_en(br_rd_en),

        .rem_br_task(br_task),
        .rem_b_id(br_fu_out.decoded_vals.b_id),

        .fu_pack(br_fu_out),
        .br_task(br_task),
        .data_ready(br_done),
        .br_taken(br_taken)
    );

    load_fu loud (
        .clock(clock),
        .reset(reset),

        .is_pack(issued_ld_pack),
        .rd_en(ld_rd_en),

        .Dmem_data_ready(Dcache_ld_out),
        .Dmem_base_addr(Dcache_addr_out),
        .Dmem_load_data(Dcache_data_out),

        .Dcache_hit(Dcache_hit_out),
        .mshr2cache_wr(mshr2cache_wr),

        .rem_br_task(br_task),
        .rem_b_id(br_fu_out.decoded_vals.b_id),

        .dm_stalled(mshr_stall),
        .start_store(start_store),

        .cdb_stall(cdb_stall_sig[`NUM_FU_ALU+`NUM_FU_MULT+`LD_SZ-1:`NUM_FU_ALU+`NUM_FU_MULT]),
        .full(ld_full),

        .dm_squash(mshr_squash),
        .start_load(start_load),
        .Dmem_addr(Dmem_ld_addr),

        .fu_pack(ld_fu_out),
        .data_ready(ld_done)

        `ifdef DEBUG
        ,   .debug_entries(debug_ld_entries),
            .debug_open_spots(debug_ld_open_spots),
            .debug_ready_spots(debug_ld_ready_spots),
            .debug_alloc_spot(debug_ld_alloc_spot),
            .debug_issued_entry(debug_ld_issued_entry),
            .debug_broadcast_entry(debug_ld_broadcast_entry),
            .debug_ld_stall_sig(debug_ld_stall_sig),
            .debug_ld_squashed(debug_ld_squashed)
        `endif
    );

    sq storey (
        .clock(clock),
        .reset(reset),

        .num_store_dispatched(num_store_dispatched),

        .is_pack(issued_st_pack),
        .rd_en(st_rd_en),

        .start_store(start_store),

        .br_en(br_done & ~br_fu_out.pred_correct),
        .br_tail(cp_out.sq_tail),

        .open_entries(sq_open),

        .Dmem_addr(Dmem_st_addr),
        .Dmem_store_data(Dmem_store_data),
        .Dmem_size(Dmem_size),

        .sq_head(sq_head),
        .sq_tail(sq_tail),
        .sq_full(sq_full)

        `ifdef DEBUG
        ,   .debug_entries(debug_sq_entries),
            .debug_num_entries(debug_sq_num_entries)
        `endif
    );

    //////////////////////////////////////////////////
    //                                              //
    //                 data memory                  //
    //                                              //
    //////////////////////////////////////////////////

    assign valid_mem_inst = start_store || start_load;
    assign Dmem_addr = start_store ? Dmem_st_addr : Dmem_ld_addr;
    assign is_store = mshr2cache_wr ? mshr2cache_is_store : start_store;
    assign st_size = mshr2cache_wr ? mshr2cache_st_size : Dmem_size;
    assign in_data = mshr2cache_wr ? mshr2cache_data : Dmem_store_data;
    assign proc2Dcache_addr = mshr2cache_wr ? mshr2cache_addr : Dmem_addr;
    assign d_proc2mem_addr = Dcache_addr_out;
    assign proc2mem_data = Dcache_data_out;

    mshr miss_human_resources (
        .clock(clock),
        .reset(reset),

        .valid(valid_mem_inst),
        .in_addr(Dmem_addr),
        .in_data(Dmem_store_data),
        .st_size(Dmem_size),
        .is_store(start_store),
        .mshr_squash(mshr_squash),

        // From Dcache
        .Dcache_hit(Dcache_hit_out),

        // From memory
        .mem2proc_transaction_tag(mem2proc_transaction_tag), // Should be zero unless there is a response
        .mem2proc_data_tag(mem2proc_data_tag),
        .mem2proc_data(mem2proc_data),

        // To memory
        .proc2mem_command(d_proc2mem_command),

        // To cache
        .mshr2cache_addr(mshr2cache_addr),
        .mshr2cache_data(mshr2cache_data),
        .mshr2cache_mem_block(mshr2cache_mem_block),
        .mshr2cache_st_size(mshr2cache_st_size),
        .mshr2cache_is_store(mshr2cache_is_store),
        .mshr2cache_wr(mshr2cache_wr),

        // To load and store units
        .stall(mshr_stall)

        `ifdef DEBUG
        ,   .debug_mshr(debug_mshr)
        `endif
    );

    dcache cash_me_outside (
        .clock(clock),
        .reset(reset),

        .proc2Dcache_addr(proc2Dcache_addr),

        .is_store(is_store),
        .st_size(st_size),
        .in_data(in_data),

        .mshr2Dcache_wr(mshr2cache_wr),
        .mshr2Dcache_mem_block(mshr2cache_mem_block),

        // To load unit stage
        .Dcache_ld_out(Dcache_ld_out),
        .Dcache_data_out(Dcache_data_out), // this is for cache hit on a load inst 
        .Dcache_hit_out(Dcache_hit_out), // When valid is high
        .Dcache_addr_out(Dcache_addr_out)  // addr goes to the load unit for a load inst, and mem for a store inst

        `ifdef DEBUG
        ,   .debug_dcache_tags(debug_dcache_tags)
        `endif
    );


    //////////////////////////////////////////////////
    //                                              //
    //               complete/commit                //
    //                                              //
    //////////////////////////////////////////////////

    always_comb begin
        cdb_reg_idx = '0;
        cdb_p_reg_idx = '0;
        cdb_valid = '0;
        cdb_wr_data = '0;
        for (int i = 0; i < `N; i++) begin
            cdb_reg_idx[i]   = cdb_entries[i].reg_idx;
            cdb_p_reg_idx[i] = cdb_entries[i].p_reg_idx;
            cdb_valid[i]     = cdb_entries[i].valid;
            cdb_wr_data[i]   = cdb_entries[i].reg_val;
        end
    end

    always_comb begin
        retiring_t_old = '0;

        for (int i = 0; i < `N; i++) begin
            retiring_t_old[i].valid = retiring_data[i].valid;
            retiring_t_old[i].reg_idx = retiring_data[i].t_old;
        end
    end

    //////////////////////////////////////////////////
    //                                              //
    //               pipeline outputs               //
    //                                              //
    //////////////////////////////////////////////////

    // Output the committed instruction to the testbench for counting
    always_comb begin
        committed_insts = '0;
        for (int i = 0; i < `N; i++) begin
            committed_insts[i].NPC = retiring_data[i].PC + 4; // TODO make this correct
            committed_insts[i].data = retiring_data[i].data;
            committed_insts[i].reg_idx = retiring_data[i].dest_reg_idx;
            committed_insts[i].halt = retiring_data[i].halt;
            committed_insts[i].illegal = '0; //TODO
            committed_insts[i].valid = (i < num_retired) ? 1 : 0;
            `ifdef DEBUG
                committed_insts[i].tag = retiring_data[i].t;
            `endif
            `ifndef DC
            if (committed_insts[i].halt) begin
                $write("SETTING HALT BIT FOR INST %h\n", retiring_data[i].PC);
            end
            `endif
        end
    end

    // Statistics & Analytics
    `ifdef ANALYTICS_EN
        assign pred_valid = br_task != NOTHING;
        assign pred_correct = br_task == CLEAR;
    `endif

    // DEBUG OUTPUTS
    `ifdef DEBUG
        `ifndef DC
            int cycle = 0;
            always @(posedge clock) begin
                $display("\n====================== CPU ======================");
                $display("@@@ Cycle %0d @@@", cycle);
                $display("Time: %0t", $time);
                cycle++;

                // if (cycle == 107) begin
                //     $finish;
                // end
            end
        `endif
    `endif

endmodule // pipeline
