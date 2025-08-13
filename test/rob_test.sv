/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rob_test.sv                                         //
//                                                                     //
//  Description :  Testbench module for the N-way ROB module           //
//                                                                     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"
`include "ISA.svh"

typedef struct packed {
    PHYS_REG_IDX    t;
    PHYS_REG_IDX    t_old; // look up t_old in arch map table to get arch reg and update to t on retire
    logic           complete;
    logic           valid;
} TEST_ROB_PACKET;

module ROB_tb();

    parameter DEPTH = `ROB_SZ;
    parameter N = `N;
    localparam LOG_DEPTH = $clog2(DEPTH);

    logic                                                                           clock;
    logic                                                                           reset;
    DECODED_PACKET                              [N-1:0]                             wr_data;
    PHYS_REG_IDX                                [N-1:0]                             t;
    PHYS_REG_IDX                                [N-1:0]                             t_old;
    PHYS_REG_IDX                                [N-1:0]                             complete_t;
    PHYS_REG_IDX                                                                    br_complete_t;
    logic                                       [$clog2(N+1)-1:0]                   num_accept;
    logic                                       [$clog2(DEPTH)-1:0]                 br_tail;
    logic                                                                           br_en;
    ROB_PACKET                                  [N-1:0]                             retiring_data;
    logic                                       [$clog2(N+1)-1:0]                   open_entries;
    logic                                       [$clog2(N+1)-1:0]                   num_retired;
    logic                                       [$clog2(DEPTH)-1:0]                 out_tail;

    `ifdef DEBUG
        ROB_PACKET                              [DEPTH-1:0]                         debug_entries;
        logic                                   [LOG_DEPTH-1:0]                     debug_head;
        logic                                   [LOG_DEPTH-1:0]                     debug_tail;
    `endif

    TEST_ROB_PACKET rob_model [$:(DEPTH)];
    TEST_ROB_PACKET inst_buf [$:((DEPTH)*2)];
    PHYS_REG_IDX complete_queue [$:(DEPTH)];


    rob #(
        .DEPTH(DEPTH),
        .N(N))
    dut (
        .clock(clock),
        .reset(reset),
        .wr_data(wr_data),
        .t(t),
        .t_old(t_old),
        .complete_t(complete_t),
        .br_complete_t(br_complete_t),
        .num_accept(num_accept),
        .br_tail(br_tail),
        .br_en(br_en),

        .retiring_data(retiring_data),
        .open_entries(open_entries),
        .num_retired(num_retired),
        .out_tail(out_tail)

        `ifdef DEBUG
        , .debug_entries(debug_entries),
        .debug_head(debug_head),
        .debug_tail(debug_tail)
        `endif
    );

    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    // Variable for Test 5
    int tmp_tail;
    logic auto_test;

    initial begin
        $display("\nStart Testbench");

        br_complete_t = 0;
        clock = 0;
        reset = 1;
        auto_test = 1;
        clear_inputs();

        @(negedge clock);
        @(negedge clock);
        reset = 0;

        // ------------------------------ Test 1 ------------------------------ //
        $display("\nTest 1: Write and Read 1 Entry with a 1 cycle wait");
        generate_instructions(1);

        $display("Write 1 value");
        add_entries(1);
        @(negedge clock);
        clear_inputs();

        $display("Wait one cycle");
        @(negedge clock);

        $display("Mark instruction complete");
        set_complete(1); // set here
        @(negedge clock);
        clear_inputs();

        $display("Retire instruction");
        // posedge 
        // fail here
        @(negedge clock);

        @(negedge clock);
        assert_empty();
        $display("PASSED TEST 1");

        // ------------------------------ Test 2 ------------------------------ //
        $display("\nTest 2: Insert DEPTH Entries, then complete 8 in order");
        generate_instructions(DEPTH);

        $display("\nInsert DEPTH instructions");
        while (inst_buf.size() > 0) begin
            add_entries(N);
            @(negedge clock);
            clear_inputs();
        end

        $display("Set all instructions to complete");
        while (rob_model.size() > 0) begin
            set_complete(N);
            @(negedge clock);
            clear_inputs();
        end

        @(negedge clock);
        assert_empty();
        $display("PASSED TEST 2");

        // ------------------------------ Test 3 ------------------------------ //
        $display("\nTest 3: Insert DEPTH entries, then complete them in backwards order");
        generate_instructions(DEPTH);

        $display("\nInsert DEPTH instructions");
        while (inst_buf.size() > 0) begin
            add_entries(N);
            @(negedge clock);
            clear_inputs();
        end

        $display("\nSet all instructions to complete, but backwards");
        while (complete_queue.size() > 0) begin
            for (int i = 0; i < N; i++) begin
                if (complete_queue.size() > 0) begin
                    complete_t[i] = complete_queue.pop_back();
                end else begin
                    break;
                end
            end
            @(negedge clock);
            clear_inputs();
        end

        $display("\nWaiting for ROB to flush");
        while (rob_model.size() > 0) begin
            @(negedge clock);
        end

        @(negedge clock);
        assert_empty();
        $display("PASSED TEST 3");

        // ------------------------------ Test 4 ------------------------------ //
        $display("\nTest 4: Insert DEPTH entries, then complete every other, then every other");
        generate_instructions(DEPTH);

        $display("\nInsert DEPTH instructions");
        while (inst_buf.size() > 0) begin
            add_entries(N);
            @(negedge clock);
            clear_inputs();
        end

        $display("\nSet every other instruction to complete");
        for (int i = 0; i < DEPTH;) begin
            for (int j = 0; j < N; j++) begin
                if (i < DEPTH) begin
                    complete_t[j] = complete_queue[i];
                    i += 2;
                end else begin
                    break;
                end
            end
            @(negedge clock);
            clear_inputs();
        end

        $display("\nSet every other, other instruction to complete");
        for (int i = 1; i < DEPTH;) begin
            for (int j = 0; j < N; j++) begin
                if (i < DEPTH) begin
                    complete_t[j] = complete_queue[i];
                    i += 2;
                end else begin
                    break;
                end
            end
            @(negedge clock);
            clear_inputs();
        end

        $display("\nWaiting for ROB to flush");
        while (rob_model.size() > 0) begin
            @(negedge clock);
        end

        // Empty complete queue, as test is complete
        while (complete_queue.size() > 0) begin
            complete_queue.pop_front();
        end

        @(negedge clock);
        assert_empty();
        $display("PASSED TEST 4");

        // ------------------------------ Test 5 ------------------------------ //
        $display("\nTest 5: EBR, adding instructions then resetting tail again");
        generate_instructions(N);

        tmp_tail = out_tail - 1;

        $display("Write N values");
        add_entries(N);
        @(negedge clock);
        clear_inputs();

        $display("Wait one cycle");
        @(negedge clock);

        $display("Reset Tail");
        br_en = 1;
        br_tail = tmp_tail;
        auto_test = 0; // turn off rob_model check
        @(negedge clock);
        clear_inputs();

        $display("Assert tail is reset");
        assert_empty();
        
        @(negedge clock);
        assert_empty();
        $display("PASSED TEST 5");



        $display("@@@ PASSED ALL TESTS @@@");
        $finish;
    end


    // Correctness Verification
    always @(posedge clock) begin
        #(`CLOCK_PERIOD * 0.2);
        if (~reset && auto_test) begin
            check_retired_entries();
            check_open_entries();
        end
    end

    // Helper function to clear inputs to ROB
    function void clear_inputs();
        num_accept = 0;
        wr_data = '0;
        t = '0;
        t_old = '0;
        complete_t = '0;
        br_tail = '0;
        br_en = 0;
    endfunction

    TEST_ROB_PACKET pack;
    // Helper function that adds entries to rob_model, writes them to wr_data, sets num_accept, also adds tag to complete queue
    function void add_entries(int num);
        if (num > N) begin
            $error("@@@ FAILED @@@");
            $error("Test Error: tried to add %0d entries, but N=%0d", num, N);
            $finish;
        end
        num_accept = num < inst_buf.size() ? num : inst_buf.size();
        for (int i = 0; i < num_accept; i++) begin
            pack = inst_buf.pop_front();
            t[i] = pack.t;
            t_old[i] = pack.t_old;
            wr_data[i].valid = 1;
            rob_model.push_back(pack);
            complete_queue.push_back(pack.t);
        end
    endfunction

    // Generates N instructions and adds them to the instruction buffer, complete tags go from 1-DEPTH
    function void generate_instructions(int num);
        integer i;
        logic [6:0] op;
        for (i = 0; i < num; i++) begin
            op = i[6:0];
            inst_buf.push_back('{t: ((i % DEPTH)+1), t_old: 5'b0, complete: 1'b0, valid: 1'b1});
        end
    endfunction

    // Helper Function to set complete_t, pops num completes off of the queue
    function void set_complete(int num);
        if (num > N) begin
            $error("@@@ FAILED @@@");
            $error("Test Error: tried to complete %0d entries, but N=%0d", num, N);
            $finish;
        end
        for (int i = 0; i < (num < complete_queue.size() ? num : complete_queue.size()); i++) begin
            complete_t[i] = complete_queue.pop_front();
        end
    endfunction
    

    // Open Entries Validation
    function void check_open_entries();
        int model_open_entries;
        model_open_entries = (N < (DEPTH - rob_model.size())) ? N : (DEPTH - rob_model.size());
        if (open_entries != model_open_entries) begin
            $error("@@@ FAILED @@@");
            $error("Open entries error: expected %d, but got %d", model_open_entries, open_entries);
            $finish;
        end
    endfunction

    // Retired Entries Validation
    function void check_retired_entries();
        TEST_ROB_PACKET inst;
        for (int i = 0; i < num_retired; i++) begin
            inst = rob_model.pop_front();
            if (inst.t !== retiring_data[i].t) begin
                $error("@@@ FAILED @@@");
                $error("Retirement data error: t expected (%0d), but got %0d!", inst.t, retiring_data[i].t);
                $finish;
            end
            if (inst.t_old !== retiring_data[i].t_old) begin
                $error("@@@ FAILED @@@");
                $error("Retirement data error: t_old expected (%0d), but got %0d!", inst.t_old, retiring_data[i].t_old);
                $finish;
            end
            if (~retiring_data[i].complete) begin
                $error("@@@ FAILED @@@");
                $error("Retirement data error: instruction not marked complete");
                $finish;
            end
        end
    endfunction

    // Ensure ROB is empty
    function void assert_empty();
        if (open_entries !== N || debug_head != debug_tail) begin
            $error("@@@ FAILED @@@");
            $error("Open entries error: expected %0d, but got %0d", N, open_entries);
            $error("Expected tail=%0d, but got tail=%0d", tmp_tail, debug_tail);
           $finish;
        end
    endfunction

    // Monitoring Statements
    int cycle_number = 0;
    always @(posedge clock) begin
        $display("------------------------------------------------------------");
        $display("@@@ Cycle Number: %0d @@@", cycle_number);
        $display("   Time: %0t", $time);
        $display("   Reset: %0d\n", reset);
        $display("   Open Entries: %0d", open_entries);
        $display("   Retired Entries: %0d\n", num_retired);

        $display("   Write Data:");
        for (int i = 0; i < num_accept; i++) begin
            $display("      wr_data[%0d]: t=%0d, t_old=%0d, valid=%0b",
                i, t[i], t_old[i], wr_data[i].valid
            );
        end
        $display("");

        $display("   Complete Data:");
        for (int i = 0; i < N; i++) begin
            if (complete_t[i] != 0) begin
                $display("      complete_t[%0d]: t=%0d", i, complete_t[i]);
            end
        end
        $display("");

        $display("   Retiring Data:");
        for (int i = 0; i < num_retired; i++) begin
            $display("      retiring_data[%0d]: t=%0d, t_old=%0d, complete=%0b, valid=%0b",
                i, retiring_data[i].t, retiring_data[i].t_old,
                retiring_data[i].complete, retiring_data[i].valid
            );
        end
        $display("");

        `ifdef DEBUG
            $display("   Debug Information:");
            $display("      Head: %0d", debug_head);
            $display("      Tail: %0d", debug_tail);
            $display("      Entries: ");
            for (int j = 0; j < DEPTH; j++) begin
                $display("         debug_entries[%0d]: t=%0d, t_old=%0d, complete=%0b, valid=%0b",
                    j, debug_entries[j].t, debug_entries[j].t_old,
                    debug_entries[j].complete, debug_entries[j].valid
                );
            end
            $display("");
        `endif

        cycle_number++;
    end

endmodule