/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  cpu_test.sv                                         //
//                                                                     //
//  Description :  Testbench module for the VeriSimpleV processor.     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"

// P4 TODO: Add your own debugging framework. Basic printing of data structures
//          is an absolute necessity for the project. You can use C functions 
//          like in test/pipeline_print.c or just do everything in verilog.
//          Be careful about running out of space on CAEN printing lots of state
//          for longer programs (alexnet, outer_product, etc.)

// These link to the pipeline_print.c file in this directory, and are used below to print
// detailed output to the pipeline_output_file, initialized by open_pipeline_output_file()
import "DPI-C" function string decode_inst(int inst);
//import "DPI-C" function void open_pipeline_output_file(string file_name);
//import "DPI-C" function void print_header();
//import "DPI-C" function void print_cycles(int clock_count);
//import "DPI-C" function void print_stage(int inst, int npc, int valid_inst);
//import "DPI-C" function void print_reg(int wb_data, int wb_idx, int wb_en);
//import "DPI-C" function void print_membus(int proc2mem_command, int proc2mem_addr,
//                                          int proc2mem_data_hi, int proc2mem_data_lo);
//import "DPI-C" function void close_pipeline_output_file();


`define TB_MAX_CYCLES 1_000_000


module testbench;
    // string inputs for loading memory and output files
    // run like: cd build && ./simv +MEMORY=../programs/mem/<my_program>.mem +OUTPUT=../output/<my_program>
    // this testbench will generate 4 output files based on the output
    // named OUTPUT.{out cpi, wb, ppln} for the memory, cpi, writeback, and pipeline outputs.
    string program_memory_file, output_name;
    string out_outfile, cpi_outfile, writeback_outfile, writeback_t_outfile;//, pipeline_outfile;
    int out_fileno, cpi_fileno, wb_fileno, wbt_fileno; // verilog uses integer file handles with $fopen and $fclose


    // variables used in the testbench
    logic        clock;
    logic        reset;
    logic [31:0] clock_count; // also used for terminating infinite loops
    logic [31:0] instr_count;

    MEM_COMMAND proc2mem_command;
    ADDR        proc2mem_addr;
    MEM_BLOCK   proc2mem_data;
    MEM_TAG     mem2proc_transaction_tag;
    MEM_BLOCK   mem2proc_data;
    MEM_TAG     mem2proc_data_tag;
    MEM_SIZE    proc2mem_size;


    INST_PACKET   [7:0] in_insts;
    logic         [3:0] num_input;

    logic         [3:0] ib_open;
    ADDR                    NPC;

    `ifdef ANALYTICS_EN
        int             num_branches, num_branches_correct; 
    `endif

    COMMIT_PACKET [`N-1:0] committed_insts;

    ROB_PACKET [`N-1:0] retired_insts;

    // DECODED_PACKET [`N-1:0] dis_insts;

    EXCEPTION_CODE error_status = NO_ERROR;
    logic [63:0] unified_memory [`MEM_64BIT_LINES-1:0];

    `ifdef DEBUG
        logic                   [`BRANCH_HISTORY_REG_SZ-1:0]                                debug_bhr;

        logic                   [$clog2(`N+1)-1:0]                                          debug_num_dispatched;
        DECODED_PACKET          [`N-1:0]                                                    debug_dis_insts;
        logic                   [$clog2(`N+1)-1:0]                                          debug_num_retired;

        logic                   [$clog2(`N+1)-1:0]                                          debug_dispatch_limit;
        logic                   [$clog2(`N+1)-1:0]                                          debug_num_store_dispatched;

        INST_PACKET             [`INST_BUFF_DEPTH-1:0]                                      debug_inst_buff_entries;
        logic                   [$clog2(`INST_BUFF_DEPTH)-1:0]                              debug_inst_buff_head;
        logic                   [$clog2(`INST_BUFF_DEPTH)-1:0]                              debug_inst_buff_tail;

        FREE_LIST_PACKET        [`ROB_SZ-1:0]                                               debug_fl_entries;
        logic                   [$clog2(`ROB_SZ)-1:0]                                       debug_fl_head;
        logic                   [$clog2(`ROB_SZ)-1:0]                                       debug_fl_tail;

        MAP_TABLE_PACKET        [`ARCH_REG_SZ-1:0]                                          debug_mt_entries;

        RS_PACKET               [`RS_SZ-1:0]                                                debug_rs_entries;
        logic                   [`RS_SZ-1:0]                                                debug_rs_open_spots;
        logic                   [`RS_SZ-1:0]                                                debug_rs_other_sig;
        logic                   [$clog2(`RS_SZ+1)-1:0]                                      debug_rs_open_entries;
        logic                   [`RS_SZ-1:0]                                                debug_rs_all_issued_insts;
        logic                   [`RS_SZ-1:0]                                                debug_all_issued_alu;
        logic                   [`RS_SZ-1:0]                                                debug_all_issued_mult;
        logic                   [`RS_SZ-1:0]                                                debug_all_issued_br;
        logic                   [`RS_SZ-1:0]                                                debug_all_issued_ld;
        logic                   [`RS_SZ-1:0]                                                debug_all_issued_st;
        BR_MASK                                                                             debug_rs_br_mask;

        ROB_PACKET              [`ROB_SZ-1:0]                                               debug_rob_entries;
        logic                   [$clog2(`ROB_SZ)-1:0]                                       debug_rob_head;
        logic                   [$clog2(`ROB_SZ)-1:0]                                       debug_rob_tail;
        logic                   [$clog2(`ROB_SZ)-1:0]                                       debug_rob_num_entries;

        CHECKPOINT              [`BRANCH_PRED_SZ-1:0]                                       debug_bs_entries;
        logic                   [`BRANCH_PRED_SZ-1:0]                                       debug_bs_free_entries;
        logic                   [`BRANCH_PRED_SZ-1:0]                                       debug_bs_stack_gnt;

        CDB_PACKET              [`N-1:0]                                                    debug_cdb_entries;
        logic                   [`NUM_FUS_CDB-1:0]                                          debug_cdb_gnt;
        logic                   [`N-1:0][`NUM_FUS_CDB-1:0]                                  debug_cdb_gnt_bus;
        logic                   [`NUM_FUS_CDB-1:0]                                          debug_cdb_fu_done;
        logic                   [`NUM_FUS_CDB-1:0]                                          debug_cdb_stall_sig;

        logic                   [`NUM_FU_ALU-1:0]                                           debug_alu_done;
        logic                   [`NUM_FU_MULT-1:0]                                          debug_mult_done;
        logic                   [`NUM_FU_MULT-1:0]                                          debug_mult_rd_en;

        ISSUE_PACKET            [`NUM_FU_ALU-1:0]                                           debug_issued_alu_pack;
        ISSUE_PACKET            [`NUM_FU_MULT-1:0]                                          debug_issued_mult_pack;
        ISSUE_PACKET                                                                        debug_issued_br_pack;
        ISSUE_PACKET            [`SQ_SZ-1:0]                                                debug_issued_st_pack;
        ISSUE_PACKET                                                                        debug_issued_ld_pack;

        logic                   [$clog2(`SQ_SZ)-1:0]                                        debug_sq_head;
        logic                   [$clog2(`SQ_SZ)-1:0]                                        debug_sq_tail;
        logic                   [$clog2(`N+1)-1:0]                                          debug_sq_open;

        logic                                                                               debug_start_store;

        FU_PACKET               [`SQ_SZ-1:0]                                                debug_sq_entries;
        logic                   [$clog2(`SQ_SZ+1)-1:0]                                      debug_sq_num_entries;

        logic                   [`NUM_FU_LD-1:0]                                            debug_ld_rd_en;
        FU_PACKET               [`LD_SZ-1:0]                                                debug_ld_entries;
        logic                   [`LD_SZ-1:0]                                                debug_ld_open_spots;
        logic                   [`LD_SZ-1:0]                                                debug_ld_ready_spots;
        logic                   [`LD_SZ-1:0]                                                debug_ld_alloc_spot;
        logic                   [`LD_SZ-1:0]                                                debug_ld_issued_entry;
        logic                   [`LD_SZ-1:0]                                                debug_ld_broadcast_entry;
        logic                                                                               debug_ld_full;
        logic                   [`LD_SZ-1:0]                                                debug_ld_stall_sig;
        logic                   [`LD_SZ-1:0]                                                debug_ld_squashed;

        MSHR                                                                                debug_mshr;
        DCACHE_TAG               [`DCACHE_LINES-1:0]                                        debug_dcache_tags;

        logic                                                                               debug_Dcache_ld_out;
        ADDR                                                                                debug_Dcache_addr_out;
        logic                                                                               debug_mshr2cache_wr;
        logic                   [`NUM_FU_ALU-1:0]                                           debug_alu_rd_en;
        logic                   [`NUM_FU_ALU-1:0][`RS_SZ-1:0]                               debug_alu_issued_bus;

        FU_PACKET               [`NUM_FU_ALU-1:0]                                           debug_alu_data;
        FU_PACKET               [`NUM_FU_ALU-1:0]                                           debug_alu_next_data;
        logic                                                                               debug_sq_full;
        logic                   [$clog2(`SQ_SZ+1)-1:0]                                      debug_sq_br_tail;

        ADDR                                                                                debug_fetch_target;
        logic                                                                               debug_fetch_arbiter_signal; 
        BR_TASK                                                                             debug_fetch_br_task;
        logic                    [$clog2(`INST_BUFF_DEPTH+1)-1:0]                           debug_fetch_ibuff_open;

        MEM_TAG                                                                             debug_fetch_mem_transaction_tag; 
        MEM_TAG                                                                             debug_fetch_mem_data_tag;
        MEM_BLOCK                                                                           debug_fetch_mem_data;

        logic                                                                               debug_fetch_mem_en;
        ADDR                                                                                debug_fetch_mem_addr_out; 

        INST_PACKET              [3:0]                                                      debug_fetch_out_insts;
        logic                    [2:0]                                                      debug_fetch_out_num_insts;

        ADDR                    [`NUM_MEM_TAGS:1]                                           debug_mshr_data;
        logic                   [`NUM_MEM_TAGS:1]                                           debug_mshr_valid;
        MEM_BLOCK                [`PREFETCH_DISTANCE-1:0]                                   debug_icache_data;
        logic                    [`PREFETCH_DISTANCE-1:0]                                   debug_icache_valid;
        ADDR                    [`PREFETCH_DISTANCE-1:0]                                    debug_icache_raddr;
    
    `endif


    // Instantiate the Pipeline
    cpu dut(
        .clock(clock),
        .reset(reset),
        .mem2proc_transaction_tag(mem2proc_transaction_tag),
        .mem2proc_data(mem2proc_data),
        .mem2proc_data_tag(mem2proc_data_tag),
        .proc2mem_command(proc2mem_command),
        .proc2mem_addr(proc2mem_addr),
        .proc2mem_data(proc2mem_data),
        .proc2mem_size(proc2mem_size),
        .committed_insts(committed_insts),
        .retired_insts(retired_insts),
        .ib_open(ib_open),
        .NPC(NPC)
        `ifdef ANALYTICS_EN
        ,   .pred_valid(pred_valid),
            .pred_correct(pred_correct)
        `endif

        `ifdef DEBUG
        ,   .debug_bhr(debug_bhr),

            .debug_num_dispatched(debug_num_dispatched),
            .debug_num_retired(debug_num_retired),

            .debug_inst_buff_entries(debug_inst_buff_entries),
            .debug_inst_buff_head(debug_inst_buff_head),
            .debug_inst_buff_tail(debug_inst_buff_tail),

            .debug_dis_insts(debug_dis_insts),
            .debug_dispatch_limit(debug_dispatch_limit),
            .debug_num_store_dispatched(debug_num_store_dispatched),

            .debug_fl_entries(debug_fl_entries),
            .debug_fl_head(debug_fl_head),
            .debug_fl_tail(debug_fl_tail),

            .debug_mt_entries(debug_mt_entries),

            .debug_rs_entries(debug_rs_entries),
            .debug_rs_open_spots(debug_rs_open_spots),
            .debug_rs_other_sig(debug_rs_other_sig),
            .debug_rs_open_entries(debug_rs_open_entries),
            .debug_rs_all_issued_insts(debug_rs_all_issued_insts),
            .debug_rs_br_mask(debug_rs_br_mask),

            .debug_all_issued_alu(debug_all_issued_alu),
            .debug_all_issued_mult(debug_all_issued_mult),
            .debug_all_issued_br(debug_all_issued_br),
            .debug_all_issued_ld(debug_all_issued_ld),
            .debug_all_issued_st(debug_all_issued_st),

            .debug_rob_entries(debug_rob_entries),
            .debug_rob_head(debug_rob_head),
            .debug_rob_tail(debug_rob_tail),
            .debug_rob_num_entries(debug_rob_num_entries),

            .debug_bs_entries(debug_bs_entries),
            .debug_bs_free_entries(debug_bs_free_entries),
            .debug_bs_stack_gnt(debug_bs_stack_gnt),

            .debug_cdb_stall_sig(debug_cdb_stall_sig),
            .debug_cdb_entries(debug_cdb_entries),
            .debug_cdb_gnt(debug_cdb_gnt),
            .debug_cdb_gnt_bus(debug_cdb_gnt_bus),
            .debug_cdb_fu_done(debug_cdb_fu_done),

            .debug_alu_done(debug_alu_done),
            .debug_mult_done(debug_mult_done),
            .debug_mult_rd_en(debug_mult_rd_en),

            .debug_issued_alu_pack(debug_issued_alu_pack),
            .debug_issued_mult_pack(debug_issued_mult_pack),
            .debug_issued_br_pack(debug_issued_br_pack),
            .debug_issued_st_pack(debug_issued_st_pack),
            .debug_issued_ld_pack(debug_issued_ld_pack),

            .debug_sq_head(debug_sq_head),
            .debug_sq_tail(debug_sq_tail),
            .debug_sq_open(debug_sq_open),

            .debug_start_store(debug_start_store),

            .debug_sq_entries(debug_sq_entries),
            .debug_sq_num_entries(debug_sq_num_entries),

            .debug_ld_rd_en(debug_ld_rd_en),
            .debug_ld_entries(debug_ld_entries),
            .debug_ld_open_spots(debug_ld_open_spots),
            .debug_ld_ready_spots(debug_ld_ready_spots),
            .debug_ld_alloc_spot(debug_ld_alloc_spot),
            .debug_ld_issued_entry(debug_ld_issued_entry),
            .debug_ld_broadcast_entry(debug_ld_broadcast_entry),
            .debug_ld_full(debug_ld_full),
            .debug_ld_stall_sig(debug_ld_stall_sig),
            .debug_ld_squashed(debug_ld_squashed),

            .debug_mshr(debug_mshr),
            .debug_dcache_tags(debug_dcache_tags),

            .debug_Dcache_ld_out(debug_Dcache_ld_out),
            .debug_Dcache_addr_out(debug_Dcache_addr_out),
            .debug_mshr2cache_wr(debug_mshr2cache_wr),

            .debug_alu_rd_en(debug_alu_rd_en),
            .debug_alu_issued_bus(debug_alu_issued_bus),

            .debug_alu_data(debug_alu_data),
            .debug_alu_next_data(debug_alu_next_data),
            .debug_sq_full(debug_sq_full),
            .debug_sq_br_tail(debug_sq_br_tail),


            .debug_fetch_target(debug_fetch_target),
            .debug_fetch_arbiter_signal(debug_fetch_arbiter_signal),
            .debug_fetch_br_task(debug_fetch_br_task),
            .debug_fetch_ibuff_open(debug_fetch_ibuff_open),

            .debug_fetch_mem_transaction_tag(debug_fetch_mem_transaction_tag),
            .debug_fetch_mem_data_tag(debug_fetch_mem_data_tag),
            .debug_fetch_mem_data(debug_fetch_mem_data),

            .debug_fetch_mem_en(debug_fetch_mem_en),
            .debug_fetch_mem_addr_out(debug_fetch_mem_addr_out),

            .debug_fetch_out_insts(debug_fetch_out_insts),
            .debug_fetch_out_num_insts(debug_fetch_out_num_insts),

            .debug_mshr_data(debug_mshr_data),
            .debug_mshr_valid(debug_mshr_valid),
            .debug_icache_data(debug_icache_data),
            .debug_icache_valid(debug_icache_valid),
            .debug_icache_raddr(debug_icache_raddr)
    
        `endif
    );

    // Instantiate the Data Memory
    mem memory (
        // Inputs
        .clock            (clock),
        .proc2mem_command (proc2mem_command),
        .proc2mem_addr    (proc2mem_addr),
        .proc2mem_data    (proc2mem_data),
`ifndef CACHE_MODE
        .proc2mem_size    (proc2mem_size),
`endif

        // Outputs
        .mem2proc_transaction_tag (mem2proc_transaction_tag),
        .mem2proc_data            (mem2proc_data),
        .mem2proc_data_tag        (mem2proc_data_tag)
    );

    // Generate System Clock
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    initial begin
        $display("\n---- Starting CPU Testbench ----\n");

        // set paramterized strings, see comment at start of module
        if ($value$plusargs("MEMORY=%s", program_memory_file)) begin
            $display("Using memory file  : %s", program_memory_file);
        end else begin
            $display("Did not receive '+MEMORY=' argument. Exiting.\n");
            $finish;
        end
        if ($value$plusargs("OUTPUT=%s", output_name)) begin
            $display("Using output files : %s.{out, cpi, wb, ppln}", output_name);
            out_outfile       = {output_name,".out"}; // this is how you concatenate strings in verilog
            cpi_outfile       = {output_name,".cpi"};
            writeback_outfile = {output_name,".wb"};
            writeback_t_outfile = {output_name, ".wbt"};
            //pipeline_outfile  = {output_name,".ppln"};
        end else begin
            $display("\nDid not receive '+OUTPUT=' argument. Exiting.\n");
            $finish;
        end

        clock = 1'b0;
        reset = 1'b0;

        $display("\n  %16t : Asserting Reset", $realtime);
        reset = 1'b1;

        @(posedge clock);
        @(posedge clock);

        $display("  %16t : Loading Unified Memory", $realtime);
        // load the compiled program's hex data into the memory module
        $readmemh(program_memory_file, memory.unified_memory);
        @(posedge clock);
        @(posedge clock);
        #1; // This reset is at an odd time to avoid the pos & neg clock edges
        $display("  %16t : Deasserting Reset", $realtime);
        reset = 1'b0;

        wb_fileno = $fopen(writeback_outfile);
        wbt_fileno = $fopen(writeback_t_outfile);
        $fdisplay(wb_fileno, "Register writeback output (hexadecimal)");
        $fdisplay(wbt_fileno, "Register writeback + tag output (hexadecimal)");

        // Open pipeline output file AFTER throwing the reset otherwise the reset state is displayed
        // open_pipeline_output_file(pipeline_outfile);
        // print_header();

        out_fileno = $fopen(out_outfile);

        $display("  %16t : Running Processor", $realtime);
    end

    ADDR current;
    MEM_BLOCK block;
    always @(negedge clock) begin
        if (reset) begin
            // Count the number of cycles and number of instructions committed
            clock_count = 0;
            instr_count = 0;
            `ifdef ANALYTICS_EN
                num_branches = 0;
                num_branches_correct = 0;
            `endif
        end else begin
            #2; // wait a short time to avoid a clock edge
            clock_count = clock_count + 1;

            if (clock_count % 10000 == 0) begin
                $display("  %16t : %d cycles", $realtime, clock_count);
            end
            //if(clock_count > 15000) begin
                dump_state();
            //end
            `ifdef ANALYTICS_EN
                for (int n = 0; n < `N; n++) begin
                    if (retired_insts[n].is_branch) begin
                        num_branches++;
                        if (retired_insts[n].pred_taken == retired_insts[n].taken) begin
                            num_branches_correct++;
                            $display("PREDICTED CORRECTLY BRANCH %h", retired_insts[n].PC);
                        end else begin
                            $display("MISPREDICTED BRANCH %h", retired_insts[n].PC);
                        end
                    end
                end
            `endif

            // print the pipeline debug outputs via c code to the pipeline output file
            // print_cycles(clock_count - 1);
            // print_stage(if_inst_dbg,     if_NPC_dbg,     {31'b0,if_valid_dbg});
            // print_stage(if_id_inst_dbg,  if_id_NPC_dbg,  {31'b0,if_id_valid_dbg});   
            // print_stage(id_ex_inst_dbg,  id_ex_NPC_dbg,  {31'b0,id_ex_valid_dbg});
            // print_stage(ex_mem_inst_dbg, ex_mem_NPC_dbg, {31'b0,ex_mem_valid_dbg});
            // print_stage(mem_wb_inst_dbg, mem_wb_NPC_dbg, {31'b0,mem_wb_valid_dbg});
            // print_reg(committed_insts[0].data, {27'b0,committed_insts[0].reg_idx},
            //           {31'b0,committed_insts[0].valid});
            // print_membus({30'b0,proc2mem_command}, proc2mem_addr[31:0],
            //              proc2mem_data[63:32], proc2mem_data[31:0]);

            // num_input = 0;
            // for (int i = 0; i < ib_open; i++) begin
            //     current = NPC + i * 4;

            //     block = unified_memory[current[31:3]];
            //     in_insts[i].inst = block.word_level[current[2]];

            //     if (in_insts[i].inst) begin
            //         in_insts[i].valid = 1;
            //         in_insts[i].PC = current;
            //         in_insts[i].NPC = current + 4;
            //         in_insts[i].pred_taken = 0;
            //         num_input++;
            //     end else begin
            //         in_insts[i].valid = 0;
            //     end

            //     // $display("index: %0d, inst: %0h, pc: %0d", i, block.word_level[current[2]], current);

            //     // if (in_insts[i].inst == 32'h10500073) begin
            //     //     $display("halting...");
            //     //     error_status = NO_ERROR;
            //     //     #200 $finish;
            //     // end
            // end

            //print_custom_data();

            output_reg_writeback_and_maybe_halt();

            // stop the processor
            if (error_status != NO_ERROR) begin

                $display("  %16t : Processor Finished", $realtime);

                // close the writeback and pipeline output files
                // close_pipeline_output_file();
                $fclose(wb_fileno);

                `ifdef ANALYTICS_EN
                    output_analytics();
                `endif

                $fclose(wbt_fileno);

                // display the final memory and status
                show_final_mem_and_status(error_status);
                // output the final CPI
                output_cpi_file();


                $display("\n---- Finished CPU Testbench ----\n");

                #100 $finish;
            end
        end // if(reset)
    end


    // Task to output register writeback data and potentially halt the processor.
    task output_reg_writeback_and_maybe_halt;
        ADDR pc;
        DATA inst;
        MEM_BLOCK block;
        PHYS_REG_IDX tag;
        for (int n = 0; n < `N; ++n) begin
            if (committed_insts[n].valid) begin
                // update the count for every committed instruction
                instr_count = instr_count + 1;

                pc = committed_insts[n].NPC - 4;
                block = memory.unified_memory[pc[31:3]];
                inst = block.word_level[pc[2]];
                // print the committed instructions to the writeback output file
                if (committed_insts[n].reg_idx == `ZERO_REG) begin
                    $fdisplay(wb_fileno, "PC %4x:%-8s| ---", pc, decode_inst(inst));
                    `ifdef DEBUG
                        $fdisplay(wbt_fileno, "PC %4x:%-8s| --- (%02d) #%02d", pc, decode_inst(inst), committed_insts[n].tag, clock_count);
                    `endif
                end else begin
                    $fdisplay(wb_fileno, "PC %4x:%-8s| r%02d=%-8x",
                              pc,
                              decode_inst(inst),
                              committed_insts[n].reg_idx,
                              committed_insts[n].data);
                    `ifdef DEBUG
                        $fdisplay(wbt_fileno, "PC %4x:%-8s| r%02d=%-8x (%02d) #%02d",
                                pc,
                                decode_inst(inst),
                                committed_insts[n].reg_idx,
                                committed_insts[n].data,
                                committed_insts[n].tag,
                                clock_count
                                );
                    `endif
                end

                // exit if we have an illegal instruction or a halt
                if (committed_insts[n].illegal) begin
                    error_status = ILLEGAL_INST;
                    break;
                end else if(committed_insts[n].halt) begin
                    error_status = HALTED_ON_WFI;
                    break;
                end
            end // if valid
        end
    endtask // task output_reg_writeback_and_maybe_halt


    // Task to output the final CPI and # of elapsed clock edges
    task output_cpi_file;
        real cpi;
        begin
            cpi = $itor(clock_count) / instr_count; // must convert int to real
            cpi_fileno = $fopen(cpi_outfile);
            $fdisplay(cpi_fileno, "Cycles: %0d", clock_count);
            $fdisplay(cpi_fileno, "Insts: %0d", instr_count);
            $fdisplay(cpi_fileno, "CPI: %f", cpi);
            $fdisplay(cpi_fileno, "Time %4.2f", clock_count * `CLOCK_PERIOD);
            $fclose(cpi_fileno);
        end
    endtask // task output_cpi_file

    task output_analytics;
        `ifdef ANALYTICS_EN
            $fdisplay(wbt_fileno, "\nFinal Branch Prediction Accuracy: %0d / %0d\n", num_branches_correct, num_branches);
        `endif
    endtask


    // Show contents of Unified Memory in both hex and decimal
    // Also output the final processor status
    task show_final_mem_and_status;
        input EXCEPTION_CODE final_status;
        int showing_data;
        begin
            $fdisplay(wbt_fileno, "Final branch prediction accuracy:\n");
        end
        begin
            $fdisplay(out_fileno, "\nFinal memory state and exit status:\n");
            $fdisplay(out_fileno, "@@@ Unified Memory contents hex on left, decimal on right: ");
            $fdisplay(out_fileno, "@@@");
            showing_data = 0;
            for (int k = 0; k <= `MEM_64BIT_LINES - 1; k = k+1) begin
                if (memory.unified_memory[k] != 0) begin
                    $fdisplay(out_fileno, "@@@ mem[%5d] = %x : %0d", k*8, memory.unified_memory[k],
                                                             memory.unified_memory[k]);
                    showing_data = 1;
                end else if (showing_data != 0) begin
                    $fdisplay(out_fileno, "@@@");
                    showing_data = 0;
                end
            end
            $fdisplay(out_fileno, "@@@");

            case (final_status)
                LOAD_ACCESS_FAULT: $fdisplay(out_fileno, "@@@ System halted on memory error");
                HALTED_ON_WFI:     $fdisplay(out_fileno, "@@@ System halted on WFI instruction");
                ILLEGAL_INST:      $fdisplay(out_fileno, "@@@ System halted on illegal instruction");
                default:           $fdisplay(out_fileno, "@@@ System halted on unknown error code %x", final_status);
            endcase
            $fdisplay(out_fileno, "@@@");
            $fclose(out_fileno);
        end
    endtask // task show_final_mem_and_status



//     // OPTIONAL: Print our your data here
//     // It will go to the $program.log file
//     task print_custom_data;
//         //$display("%3d: YOUR DATA HERE", 
//         //    clock_count-1
//         //);
//     endtask

    // DEBUGGER

    `ifdef DEBUG
    // print fetch/prefetcher
    function void print_fetch();
        $display("FETCH");

        $display("                   target                        |                          mem_data");
        $display("  %h   |   %b   |", 
                debug_fetch_target,
                debug_fetch_mem_data
            );

        $display("|   mem_en   |  mem_addr_out   | out_num_insts |");
        $display(" %d |      %h      |  %d |",
                debug_fetch_mem_en,
                debug_fetch_mem_addr_out,
                debug_fetch_out_num_insts
            );

        $display("mem_transaction_tag    |    mem_data_tag    |                      mem_data                     |");
        $display("  %b   |   %b   | %b  | %b", 
                debug_fetch_mem_transaction_tag,
                debug_fetch_mem_data_tag,
                debug_fetch_mem_data,
                debug_fetch_ibuff_open
            );

        $display("MSHR:");
        $display("| Tag | Valid |");
        for (int i = 1; i <= `NUM_MEM_TAGS; i++) begin
            $display("|%4h | %d |", debug_mshr_data[i], debug_mshr_valid[i]);
        end
        $display("");

        $display("ICACHE:");
        $display("| Data | Valid | Addr |");
        for (int i = 0; i < 4; i++) begin
            $display("| %h | %d | %h |", debug_icache_data[i], debug_icache_valid[i], debug_icache_raddr[i]);
        end
        $display("");
        
        
        // for (int i = 0; i < `INST_BUFF_DEPTH; i++) begin
        //     $display("  %b   |   %b   | %d |      %b      |  %d |", 
        //         i, 
        //         debug_inst_buff_entries[i].valid, 
        //         debug_inst_buff_entries[i].inst, 
        //         debug_inst_buff_entries[i].PC, 
        //         debug_inst_buff_entries[i].NPC, 
        //         debug_inst_buff_entries[i].pred_taken ? "t" : "nt"
        //     );
        // end
    endfunction

    // inst buff
    function void print_inst_buff();
        $display("Instruction Buffer");
        $display("#\t| valid |   inst     |   PC   |  NPC   | pred   |");
        for (int i = 0; i < `INST_BUFF_DEPTH; i++) begin
            $display("%02d\t|   %d   | %x\t|  %05x |  %05x |   %s   |", 
                i, 
                debug_inst_buff_entries[i].valid, 
                debug_inst_buff_entries[i].inst, 
                debug_inst_buff_entries[i].PC, 
                debug_inst_buff_entries[i].NPC, 
                debug_inst_buff_entries[i].pred_taken ? "t" : "nt"
            );
        end
    endfunction

    // dispatch
    function void print_dispatch();
        $display("\nDispatch (Limit: %02d) (SQ Open: %02d) (SQ Tail: %02d) (num_store_dis: %02d)", debug_dispatch_limit, debug_sq_open, debug_sq_tail, debug_num_store_dispatched);
        $display("#\t| valid |    inst    |   PC   |   NPC  |");
        for (int i = 0; i < `N; i++) begin
            $write("%02d\t|   %d   | %08x   | %05x  | %05x  |\n", 
                i, 
                debug_dis_insts[i].valid, 
                debug_dis_insts[i].inst, 
                debug_dis_insts[i].PC, 
                debug_dis_insts[i].NPC
            );
        end
    endfunction

    // rob
    function void print_rob();
        $display("\nReorder Buffer (ROB) (%02d) (Start Store: %b)", debug_rob_num_entries, debug_start_store);
        $display("Status | #  |     PC     |  dest_reg   | halt | complete |    t   | t_old  |");
        for (int i = 0; i < `ROB_SZ; i++) begin
            string status = "";
            if (i == debug_rob_tail && i== debug_rob_head)
                status = "HT"; 
            else if (i == debug_rob_head) 
                status = "HEAD"; 
            else if (i == debug_rob_tail)
                status = "TAIL"; 
            else
                status = ""; 

            $display("%-6s | %02d |   %05x    |  %02d         |  %d   |    %d     |   %02d   |   %02d   |", 
                    status, 
                    i, 
                    //debug_rob_entries[i].valid, 
                    debug_rob_entries[i].PC, 
                    debug_rob_entries[i].dest_reg_idx, 
                    debug_rob_entries[i].halt, 
                    debug_rob_entries[i].complete, 
                    debug_rob_entries[i].t, 
                    debug_rob_entries[i].t_old);
        end
    endfunction


    function void print_sq();
        $display("\nStore Queue Full: %0b (%02d) (Execute Store: %b) (BR Tail: %02d - %02d)", debug_sq_full, debug_sq_num_entries, debug_start_store, debug_sq_br_tail, debug_sq_tail);
        $display("Status | #  |    PC   | Target Addr ");
        for(int i=0;i<`SQ_SZ;i++) begin
            string status = "";
            if (i == debug_sq_tail && i== debug_sq_head)
                status = "HT"; 
            else if (i == debug_sq_head) 
                status = "HEAD"; 
            else if (i == debug_sq_tail)
                status = "TAIL"; 
            else
                status = "";
            $display("%-6s | %02d |  %05x  |  %05x ", status, i, debug_sq_entries[i].decoded_vals.decoded_vals.PC, debug_sq_entries[i].target_addr);
        end
    endfunction

    function void print_ld();
        $display("\nLoad Unit (Full: %b) (Rd_en: %b) (Dcache_ld_out: %b) (mshr2cache_wr: %b)", debug_ld_full, debug_ld_rd_en, debug_Dcache_ld_out, debug_mshr2cache_wr);
        $display("CDB Stall: %b", debug_cdb_stall_sig[`NUM_FU_ALU+`NUM_FU_MULT+`LD_SZ-1:`NUM_FU_ALU+`NUM_FU_MULT]);
        $display("Dcache Addr Out: 0h%05x", debug_Dcache_addr_out);
        $display("#  |    PC   |Target Addr| Result | State | Open? | Ready? | Alloc? | Issued? | BCast? | Stalled? | Squashed? | b_mask");
        for(int i=0;i<`LD_SZ;i++) begin
            $display("%02d |  %05x  |   %05x   | %05x  |   %d   |   %b   |   %b    |   %b    |    %b    |   %b    |  %b       |    %b      |  %b", i, debug_ld_entries[i].decoded_vals.decoded_vals.PC, debug_ld_entries[i].target_addr, debug_ld_entries[i].result, debug_ld_entries[i].ld_state, debug_ld_open_spots[i], debug_ld_ready_spots[i], debug_ld_alloc_spot[i], debug_ld_issued_entry[i], debug_ld_broadcast_entry[i], debug_ld_stall_sig[i], debug_ld_squashed[i], debug_ld_entries[i].decoded_vals.b_mask);
        end
    endfunction

    // rs
    function void print_rs();
        $display("\nReservation Station (SQ Head: %02d) (B Mask: %b)", debug_sq_head, debug_rs_br_mask);
        $display("ALU Busy: %b", debug_cdb_stall_sig[`NUM_FU_ALU-1:0]);
        $display("ALU Issued Bus");
        for(int i=`NUM_FU_ALU-1;i>=0;i--) begin
            $display("%02d: %b", i, debug_alu_issued_bus[i]);
        end
        $display("#  | valid |  PC   |  NPC  |fu_type| t |t1 |t2 |b_id|b_mask| sq tail|alu issued|mult issued|br issued|ld issued|st issued|ld ready");
        for (int i = `RS_SZ-1; i >= 0; i--) begin
            $display("%02d |  %d    | %05x | %05x |  %02d   | %02d|%02d%s|%02d%s|%04b| %04b |  %05d |    %d     |     %d     |     %d   |     %d   |     %d   |     %d   |", 
                        i,
                        debug_rs_entries[i].decoded_vals.valid,
                        debug_rs_entries[i].decoded_vals.PC,
                        debug_rs_entries[i].decoded_vals.NPC,
                        debug_rs_entries[i].decoded_vals.fu_type,
                        debug_rs_entries[i].t.reg_idx,
                        debug_rs_entries[i].t1.reg_idx,
                        (debug_rs_entries[i].t1.ready) ? "+" : " ",
                        debug_rs_entries[i].t2.reg_idx,
                        (debug_rs_entries[i].t2.ready) ? "+" : " ",
                        debug_rs_entries[i].b_id,
                        debug_rs_entries[i].b_mask,
                        debug_rs_entries[i].decoded_vals.sq_tail,
                        debug_all_issued_alu[i],
                        debug_all_issued_mult[i],
                        debug_all_issued_br[i],
                        debug_all_issued_ld[i],
                        debug_all_issued_st[i],
                        debug_rs_entries[i].ld_ready
                        );
        end
    endfunction

    // map table
    // TODO: need to add debug_mt_entries to the map table test
    function void print_map_table();
        $display("\nMap Table");
        $display("reg_idx| p_reg_idx | ready |valid |");
        for (int i = 0; i < `ARCH_REG_SZ; i++) begin
            $display("%02d\t|   %04d    |   %1d   |   %1d  |", 
                i, 
                debug_mt_entries[i].reg_idx, 
                debug_mt_entries[i].ready, 
                debug_mt_entries[i].valid);
        end
    endfunction

    // freelist
    function void print_freelist();
        $display("\nFree List");
        $display("Status | #  | reg_idx | valid |");
        for (int i = 0; i < `ROB_SZ; i++) begin
            string pos; 
            pos = "";
            if (i == debug_fl_head && i == debug_fl_tail)
                pos = "HT"; 
            else if (i == debug_fl_head)
                pos = "HEAD";
            else if (i == debug_fl_tail)
                pos = "TAIL";

            $display("%-6s | %02d |  %04d   |   %1d   |", 
                pos, 
                i, 
                debug_fl_entries[i].reg_idx, 
                debug_fl_entries[i].valid);
        end
    endfunction

    // issue
    function void print_issue();
        $display("\nIssue Module");
        $display("ALU packets");
        $display("#  | valid |    inst    |     PC      |     NPC     |   rs1_value    |   rs2_value    | b_mask |");
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            $display("%02d |  %d    |  %08x  |  0h%08x |  0h%08x |  %08x      |  %08x      |  %b  |", 
                    i,
                    debug_issued_alu_pack[i].decoded_vals.decoded_vals.valid,
                    debug_issued_alu_pack[i].decoded_vals.decoded_vals.inst,
                    debug_issued_alu_pack[i].decoded_vals.decoded_vals.PC,
                    debug_issued_alu_pack[i].decoded_vals.decoded_vals.NPC,
                    debug_issued_alu_pack[i].rs1_value,
                    debug_issued_alu_pack[i].rs2_value,
                    debug_issued_alu_pack[i].decoded_vals.b_mask
                    );
        end

        $display("MULT packets");
        $display("#  | valid |    inst    |     PC      |     NPC     |   rs1_value    |   rs2_value    |");
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            $display("%02d |  %d    |  %08x  |  0h%08x |  0h%08x |  %08x      |  %08x      |", 
                    i,
                    debug_issued_mult_pack[i].decoded_vals.decoded_vals.valid,
                    debug_issued_mult_pack[i].decoded_vals.decoded_vals.inst,
                    debug_issued_mult_pack[i].decoded_vals.decoded_vals.PC,
                    debug_issued_mult_pack[i].decoded_vals.decoded_vals.NPC,
                    debug_issued_mult_pack[i].rs1_value,
                    debug_issued_mult_pack[i].rs2_value);
        end
        $display("BR packets");
        $display("#  | valid |    inst    |     PC      |     NPC     |   rs1_value    |   rs2_value    | b_id | b_mask");
        $display("%02d |  %d    |  %08x  |  0h%08x |  0h%08x |  %08x      |  %08x      | %b | %b   |", 
                    0,
                    debug_issued_br_pack.decoded_vals.decoded_vals.valid,
                    debug_issued_br_pack.decoded_vals.decoded_vals.inst,
                    debug_issued_br_pack.decoded_vals.decoded_vals.PC,
                    debug_issued_br_pack.decoded_vals.decoded_vals.NPC,
                    debug_issued_br_pack.rs1_value,
                    debug_issued_br_pack.rs2_value,
                    debug_issued_br_pack.decoded_vals.b_id,
                    debug_issued_br_pack.decoded_vals.b_mask
                    );

        $display("ST packets");
        $display("#  | valid |    inst    |     PC      |     NPC     |   rs1_value    |   rs2_value    |");
        for (int i = 0; i < `SQ_SZ; i++) begin
            $display("%02d |  %d    |  %08x  |  0h%08x |  0h%08x |  %08x      |  %08x      |", 
                    i,
                    debug_issued_st_pack[i].decoded_vals.decoded_vals.valid,
                    debug_issued_st_pack[i].decoded_vals.decoded_vals.inst,
                    debug_issued_st_pack[i].decoded_vals.decoded_vals.PC,
                    debug_issued_st_pack[i].decoded_vals.decoded_vals.NPC,
                    debug_issued_st_pack[i].rs1_value,
                    debug_issued_st_pack[i].rs2_value);
        end
        $display("LD packets");
        $display("#  | valid |    inst    |     PC      |     NPC     |   rs1_value    |   rs2_value    |");
            $display("%02d |  %d    |  %08x  |  0h%08x |  0h%08x |  %08x      |  %08x      |", 
                    0,
                    debug_issued_ld_pack.decoded_vals.decoded_vals.valid,
                    debug_issued_ld_pack.decoded_vals.decoded_vals.inst,
                    debug_issued_ld_pack.decoded_vals.decoded_vals.PC,
                    debug_issued_ld_pack.decoded_vals.decoded_vals.NPC,
                    debug_issued_ld_pack.rs1_value,
                    debug_issued_ld_pack.rs2_value);
    endfunction

    function void print_alu_data();
        $display("\nALU State");
        $display("#  | valid | PC | result");
        for(int i=`NUM_FU_ALU-1;i>=0;i--) begin
            $display("%02d | %b  | %x  | %x", i, debug_alu_data[i].decoded_vals.decoded_vals.valid, debug_alu_data[i].decoded_vals.decoded_vals.PC, debug_alu_data[i].result);
            $display("%02d | %b  | %x  | %x", i, debug_alu_next_data[i].decoded_vals.decoded_vals.valid, debug_alu_next_data[i].decoded_vals.decoded_vals.PC, debug_alu_next_data[i].result);
            $display();
        end
    endfunction

    // branch stack
    function void print_br_stack();
        $display("\nBranch Stack");
        $display("#  | valid |   b_id   |  b_mask  | fl_head|rob_tail|");

        // Print the state of each entry in the branch stack
        for (int i = 0; i < `BRANCH_PRED_SZ; i++) begin
            $display("%02d |   %d   |   %04b   |   %04b   |   %02d   |   %02d   |", 
                i, 
                debug_bs_entries[i].valid, 
                debug_bs_entries[i].b_id, 
                debug_bs_entries[i].b_mask, 
                debug_bs_entries[i].fl_head, 
                debug_bs_entries[i].rob_tail
            );
        end
    endfunction

    // cdb
    function void print_cdb();
        $display("\nCDB, gnt: %b", debug_cdb_gnt);
        $display("FU DONE: %b", debug_cdb_fu_done);
        $display("CDB Stall Sig %b", debug_cdb_stall_sig);
        $display("ALU RD en %b", debug_alu_rd_en);
        $display("#  |   valid |  reg_idx | p_reg_idx |   reg_val   |");

        for (int i = 0; i < `N; i++) begin
            $display("%02d |    %d    |    %02d    |     %02d    |  %08x   |", 
                i, 
                debug_cdb_entries[i].valid, 
                debug_cdb_entries[i].reg_idx, 
                debug_cdb_entries[i].p_reg_idx, 
                debug_cdb_entries[i].reg_val
            );
        end
    endfunction

    // mshr
    function void print_mshr();
        $display("\nMSHR");
        $display("state                 | addr           | data           | mem_tag  | store size | is_store |");
        $display("%-12s | 0h%08x     | 0h%08x     | %02d       | %02d         | %d",
            debug_mshr.state.name(),
            debug_mshr.addr,
            debug_mshr.data,
            debug_mshr.mem_tag,
            debug_mshr.st_size,
            debug_mshr.is_store
        );
    endfunction

    // bhr
    function void print_bhr();
        $display("\nBranch History Register");
        $display("<-Youngest   oldest->");
        for (int i = 0; i < `BRANCH_HISTORY_REG_SZ; i++) begin
            $write("|%0d", debug_bhr[i]);
        end
        $display("|");
    endfunction

    // dcache
    // function void print_dcache();
    //     $display("\nDcache");
    //     $display("Index |   Valid   |      Tag       |      Data       |");

    //     for (int i = 0; i < `DCACHE_LINES; i++) begin
    //         $display("%4d  |     %b     |  0h%08x   | %64h",
    //             i,
    //             debug_dcache_tags[i].valid,
    //             debug_dcache_tags[i].tags,
    //             // accessing the actual data
    //         );
    //     end
    // endfunction
    `endif

    function void dump_state();
        $display("--------------");
        $display("Clock #%02d, NPC: %x", clock_count, NPC);
        `ifdef DEBUG
        $display("num_dispatched: %02d , num_issued: %02d, num_retired: %02d", debug_num_dispatched, $countones(debug_rs_all_issued_insts), debug_num_retired);
        `endif
        $display("mem2proc TTag: %02d, DTag: %02d, Data_in: 0h%x", mem2proc_transaction_tag, mem2proc_data_tag, mem2proc_data);
        $display("CMD: %s (%b), Addr: 0h%05x, Data_out: 0h%x", proc2mem_command.name(), proc2mem_command, proc2mem_addr, proc2mem_data);
        `ifdef DEBUG
        $display("CDB Stall Sig %b", debug_cdb_stall_sig);
        $display("PCs Retired");
        for(int i=0;i<debug_num_retired;i++) begin
            $display("%02d: 0h%05x", i, retired_insts[i].PC);
        end
        $display("\n");
        print_fetch();
        print_inst_buff();
        print_dispatch();
        print_sq();
        print_ld();
        print_mshr();
        print_rob();
        $display("N is ", `N);
        $display("\nALU Data Ready: %b", debug_alu_done);
        $display();
        $display("MULT Rd EN: %b", debug_mult_rd_en);
        $display("MULT Data Ready: %b", debug_mult_done);
        // $display("MULT Data Ready: %b", dut.mult_done);
        // $display("FU DONE: %b", dut.cbd.fu_done);

        print_rs();
        print_map_table();
        print_freelist();
        print_bhr();
        print_br_stack();
        print_cdb();
        print_alu_data();
        print_issue();
        $display("\n");
        `endif
    endfunction


endmodule // module testbench
