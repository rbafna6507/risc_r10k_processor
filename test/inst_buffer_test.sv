/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  inst_buffer_test.sv                                 //
//                                                                     //
//  Description :  Testbench module for the N-way ROB module           //
//                                                                     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"
`include "ISA.svh"

module inst_buffer_tb();

    parameter DEPTH = 8;
    parameter N = 3; 
    localparam LOG_DEPTH = $clog2(DEPTH);

    logic                                  clock;
    logic                                  reset;

    INST_PACKET      [DEPTH-1:0]           in_insts;
    logic            [$clog2(N+1)-1:0]     num_dispatch;
    logic            [$clog2(DEPTH+1)-1:0] num_accept;

    INST_PACKET      [N-1:0]               dispatched_insts;
    logic            [$clog2(DEPTH+1)-1:0] open_entries;

    `ifdef DEBUG
        INST_PACKET [DEPTH-1:0]     debug_entries;
        logic            [LOG_DEPTH-1:0] debug_head;
        logic            [LOG_DEPTH-1:0] debug_tail;
    `endif
    
    INST_PACKET inst_buffer_model[$:DEPTH];
    int instruction = 4'b1000;
    int pc = 8'b00001000; 
    int npc = 8'b00001001; 
    logic [$clog2(DEPTH)-1:0] k = 0;

    inst_buffer #(
        .DEPTH(DEPTH),
        .N(N)
    ) dut (
        .clock(clock),
        .reset(reset),
        .in_insts(in_insts),
        .num_dispatch(num_dispatch),
        .num_accept(num_accept),

        .dispatched_insts(dispatched_insts),
        .open_entries(open_entries)

        `ifdef DEBUG
        , .debug_entries(debug_entries),
            .debug_head(debug_head),
            .debug_tail(debug_tail)
        `endif
    );

    always begin
        #(`CLOCK_PERIOD / 2.0);
        clock = ~clock;
    end

    // Test sequence
    initial begin
        $display("\nStart Testbench");

        clock = 0;
        reset = 1;
        @(negedge clock);

        reset = 0;

        // population input instructions with random instructions
        for (int i = 0; i < DEPTH; i++) begin
            in_insts[i] = {instruction, pc, npc, 1};
            instruction = instruction + 2;
            pc = pc + 2;
        end

        // for my viewership
        print_input_inst();

        // ------------------------------ Test 1 ------------------------------ //
        $display("\nTest 1: Put Instructions in the Buffa");
        
        num_accept = 4;
        num_dispatch = 0;
        add_inst();
        @(negedge clock);

        num_accept = 0;
        @(negedge clock);

        check_dispatched_entries();
        check_open_entries(4);
        $display("PASSED TEST 1");

        // ------------------------------ Test 2 ------------------------------ //
        $display("\nTest 2: Update Head and Tail based on Num Dispatch");

        num_dispatch = 2;
        remove_inst();
        @(negedge clock);

        num_dispatch = 0;
        @(negedge clock);

        check_dispatched_entries();
        check_open_entries(6);
        $display("PASSED TEST 2");

        // ------------------------------ Test 3 ------------------------------ //
        $display("\nTest 3: Update Head and Tail based on Num Dispatch and Num Accept");

        num_dispatch = 2;
        remove_inst();
        num_accept = 2;
        add_inst();
        @(negedge clock);

        num_accept = 0;
        num_dispatch = 0;
        @(negedge clock);

        check_dispatched_entries();
        check_open_entries(6);
        $display("PASSED TEST 3");

        $finish;
    end
    
    int cycle_number = 0;
    always @(posedge clock) begin     
        #(`CLOCK_PERIOD * 0.2);
        print_entries();
        print_model();
        print_dispatched_instructions();
        print_open_entries();
        check_entries();
        $display("@@@ FINISHED CYCLE NUMBER: %0d @@@ \n", cycle_number);
        cycle_number++;
    end
    
    // functions

    function void add_inst();
        for (int i = 0; i < num_accept; i++) begin
            inst_buffer_model.push_back(in_insts[i]);
        end
    endfunction

    function void remove_inst();
        for (int i = 0; i < num_dispatch; i++) begin
            inst_buffer_model.pop_front();
        end
    endfunction

    function void reset_model();
        for (int i = 0; i < DEPTH; i++) begin
            inst_buffer_model.pop_front();
        end
    endfunction

    // printing

    function void print_input_inst();
        $display("\Input Instructions");
        for (int i = 0; i < DEPTH; i++) begin
            $display("\nindex: %d", i);
            $display("inst:   %d", in_insts[i].inst);
            $display("pc:     %d", in_insts[i].PC);
        end
    endfunction

    function void print_model();
        $display("\nInst Buff Model");
        for (int i = 0; i < DEPTH; i++) begin
            $display("\nindex: %d", i);
            $display("inst:   %d", inst_buffer_model[i].inst);
            $display("pc:     %d", inst_buffer_model[i].PC);
        end
    endfunction

    function void print_entries();
        $display("\nEntries");
        for (int i = 0; i < DEPTH; i++) begin
            $display("\nindex: %d", i);
            $display("inst:   %d", debug_entries[i].inst);
            $display("pc:     %d", debug_entries[i].PC);
        end
        $display("\nhead %0d", debug_head);
        $display("tail %0d", debug_tail);
        $display("reset %0d", reset);
    endfunction

    function void print_dispatched_instructions();
        $display("\nDispatched Instructions");
        for (int i = 0; i < N; i++) begin
            $display("\nindex: %0d", i);
            $display("inst:   %0d", dispatched_insts[i].inst);
            $display("pc:     %0d", dispatched_insts[i].PC);
        end
    endfunction

    function void print_open_entries();
        $display("\nOpen Entries: %0d", open_entries);
    endfunction

    // checks

    function void check_open_entries(int open);
        if (open != open_entries) begin
            $error("@@@ FAILED @@@");
            $error("Check open entry: expected %0d, but got %0d", open, open_entries);
            $finish;
        end
    endfunction

    function void check_entries();
        k = debug_head;
        for (int i = 0; i < DEPTH; i++) begin
            if (k == debug_tail && debug_head != 0 && debug_tail != 0) begin // how to account for case where we are just beginning and head and tail are 0
                break;
            end

            if (inst_buffer_model[i].inst != debug_entries[k].inst) begin
                $error("@@@ FAILED @@@");
                $error("Check entry error: expected %0d, but got %0d", inst_buffer_model[i].inst, debug_entries[k].inst);
                $finish;
            end

            k = (k + 1) % DEPTH;
        end  
    endfunction

    function void check_dispatched_entries();
        for (int i = 0; i < inst_buffer_model.size(); i++) begin
            if (inst_buffer_model[i].inst != dispatched_insts[i].inst) begin
                $error("@@@ FAILED @@@");
                $error("Check dispatched entry error: expected %0d, but got %0d", inst_buffer_model[i].inst, dispatched_insts[i].inst);
                $finish;
            end
        end
    endfunction

endmodule