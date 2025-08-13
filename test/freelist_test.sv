/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  freelist_test.sv                                    //
//                                                                     //
//  Description :  Testbench module for the free list                  //
//                                                                     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"
`include "ISA.svh"

module freelist_tb();

    parameter DEPTH = `ROB_SZ;
    parameter N = 2;
    localparam LOG_DEPTH = $clog2(DEPTH);

    logic                                               clock;
    logic                                               reset;
    logic                       [$clog2(N+1)-1:0]       rd_num;  // number of regs to take off of the free list
    logic                       [$clog2(N+1)-1:0]       wr_num;  // number of regs to add back to the free list
    FREE_LIST_PACKET            [N-1:0]                 wr_reg;  // reg idxs to add to free list
    logic                                               br_en;  // enable signal for EBR
    logic                       [$clog2(DEPTH+1)-1:0]   head_ptr_in;


    FREE_LIST_PACKET            [N-1:0]                 rd_reg;   // displayed available reg idxs, these are always output, and only updated based on rd_num
    //logic                   [$clog2(DEPTH+1)-1:0]   num_avail; // broadcasting number of regs available
    logic                       [$clog2(DEPTH+1)-1:0]   head_ptr;

    `ifdef DEBUG
        FREE_LIST_PACKET        [DEPTH-1:0]             debug_entries;   // free list to output
        logic                   [LOG_DEPTH-1:0]         debug_head;
        logic                   [LOG_DEPTH-1:0]         debug_tail;
    `endif 

    // queue declaration for free_list model
    FREE_LIST_PACKET free_list_model [$:(DEPTH)];
    FREE_LIST_PACKET free_list_model_copy [$:(DEPTH)];

    // other variables
    FREE_LIST_PACKET entry_one;
    FREE_LIST_PACKET entry_random;
    FREE_LIST_PACKET  [2:0] pr_list;
    logic [$clog2(DEPTH)-1:0] k = 0;

    freelist #(
        .DEPTH(DEPTH),
        .N(N)
    )
    dut (
        .clock(clock),
        .reset(reset),
        .rd_num(rd_num),  
        .wr_num(wr_num),  
        .wr_reg(wr_reg),  
        .br_en(br_en),  
        .head_ptr_in(head_ptr_in),  

        .rd_reg(rd_reg),   
        //.num_avail(num_avail),
        .head_ptr(head_ptr)

        `ifdef DEBUG
        ,   .debug_entries(debug_entries),   
            .debug_head(debug_head),
            .debug_tail(debug_tail)
        `endif
    );

    always begin 
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    initial begin
        $display("\nStart Testbench");

        clock = 0;
        reset = 1;
        rd_num = 0;
        wr_num = 0;
        br_en = 0;

        @(negedge clock);
        @(negedge clock);
        reset = 0;
        
        // ------------------------------ Test 1 ------------------------------ //
        $display("\nTest 1: Check Free List is Full");
        
        reset_free_list();
        @(negedge clock);
        
        clear_inputs();
        @(posedge clock);
        @(negedge clock);

        $display("PASSED TEST 1");

        // ------------------------------ Test 2 ------------------------------ //
        $display("\nTest 2: Check One Entry Popped");
        
        reset_free_list();
        @(negedge clock);
        
        pop_n_from_free_list(1);
        @(negedge clock);
        
        clear_inputs();
        @(posedge clock);
        @(negedge clock);

        $display("PASSED TEST 2");

        // ------------------------------ Test 3 ------------------------------ //
        $display("\nTest 3: Check One Entry Written Back");
        
        entry_one = '{reg_idx: 32, valid: 1};
        entry_random = '{reg_idx: 33, valid: 1};
        pr_list[0] = entry_one;
        pr_list[1] = entry_random;

        add_prs_to_free_list(pr_list, 1);
        @(negedge clock);
        
        clear_inputs();
        @(posedge clock);
        @(negedge clock);

        $display("PASSED TEST 3");

        // ------------------------------ Test 4 ------------------------------ //
        $display("\nTest 4: Check One Entry Read and One Entry Written at same time");
        
        reset_free_list();
        reset = 1;
        @(negedge clock);
        reset = 0;

        //free_list_model.pop_front();
        pop_n_from_free_list(DEPTH);
        add_prs_to_free_list(pr_list, 1);
        pop_n_from_free_list(1);
        @(negedge clock);

        clear_inputs();
        @(negedge clock);
    
        $display("PASSED TEST 4");

        // some thoughts to myself bc i am dumb
        // write comes before read, bc we will not have the situation where we want to read something off and then write it back right after while it is full
        // that don't make sense- instead, if it's empty i want to write first so i can read
        // so in order to test writing and reading at the same time, start empty and then try writing and reading at the same time :D

        // ------------------------------ Test 5 ------------------------------ //
        $display("\nTest 5: Empty Out Free List and Add One Back ");

        reset_free_list();
        reset = 1;
        @(negedge clock);
        reset = 0;

        pop_n_from_free_list(DEPTH);
        add_prs_to_free_list(pr_list, 1);

        @(negedge clock);

        clear_inputs();
        @(negedge clock);
    
        $display("PASSED TEST 5");

        // ------------------------------ Test 6 ------------------------------ //
        $display("\nTest 6: EBR Mispredict Testing");

    
        reset_free_list();
        reset = 1;
        @(negedge clock);
        reset = 0;

        // POP DOESN'T WORK WITH POPPING 4

        pop_n_from_free_list(3);
        @(negedge clock);

        // tail 0
        // head 3

        clear_inputs();
        @(negedge clock);

        free_list_model_copy = free_list_model;

        pop_n_from_free_list(3);
        @(negedge clock);

        // tail 0
        // head 6

        clear_inputs();
        @(negedge clock);
    
        head_ptr_in = 3;
        br_en = 1;
        free_list_model = free_list_model_copy;
        @(negedge clock);

        // head 3
        // tail 0
        $display("PASSED TEST 6");

        // ------------------------------ Test 7 ------------------------------ //
        $display("\nTest 7: Add zero reg to FL");

        reset_free_list();
        clear_inputs();
        reset = 1;
        @(negedge clock);
        reset = 0;

        pop_n_from_free_list(1);
        pr_list[0] = {reg_idx:0, valid: 1};
        add_prs_to_free_list(pr_list, 1);

        @(negedge clock);

        clear_inputs();
        @(negedge clock);
    
        $display("PASSED TEST 7");

        $finish;

    end

    int cycle_number = 0;
    // Correctness Verification
    always @(posedge clock) begin
        #(`CLOCK_PERIOD * 0.2);
        print_model();
        print_free_list();
        $display("rd_num: %d", rd_num);
        $display("wr_num: %d\n", wr_num);
        check_entries();
        $display("@@@ FINISHED CYCLE NUMBER: %0d @@@ \n", cycle_number);
        cycle_number++;
    end

    // pop_from_free_list function
        // if want to pop all, set N to size
    function void pop_n_from_free_list(int num_pop);
        rd_num = num_pop;
        for (int i = 0; i < num_pop; i++) begin
            free_list_model.pop_front();
        end
    endfunction

    // add_to_free_list function
    function void add_prs_to_free_list(FREE_LIST_PACKET [N-1:0] pr, int num_write);
        wr_reg = pr;
        wr_num = num_write;
        for (int i = 0; i < num_write; i++) begin
            if(pr[i].reg_idx != 0) begin
                free_list_model.push_back(pr[i]);
            end
        end
    endfunction

    // reset free_list function, add all regs + valid bits
    function void reset_free_list();
        for (int i = 0; i < DEPTH; i++) begin
            free_list_model[i].reg_idx = i + `ARCH_REG_SZ;
            free_list_model[i].valid = 1;
        end
    endfunction

    // compare free list model to actual free list
    function void check_entries();
        $display("checking entries");
        k = debug_head;
        for (int i = 0; i < DEPTH; i++) begin
            if (k == debug_tail && debug_head != 0 && debug_tail != 0) begin // how to account for case where we are just beginning and head and tail are 0
                break;
            end

            `ifdef DEBUG
                if (free_list_model[i].reg_idx != debug_entries[k].reg_idx) begin
                    $error("Check entry error: expected %0d, but got %0d", free_list_model[i].reg_idx, debug_entries[k].reg_idx);
                    $finish;
                end
            `endif

            k = (k + 1) % DEPTH;
        end  
    endfunction

    function void print_model();
        $display("\nFree List Model");
        for (int i = 0; i < DEPTH; i++) begin
            $display("model[%0d]: %0d", i, free_list_model[i].reg_idx);
        end
    endfunction

    function void print_free_list();
    `ifdef DEBUG
        $display("\nActual Free List");
        for (int i = 0; i < DEPTH; i++) begin
            $display("free_list[%0d]: %0d", i, debug_entries[i].reg_idx);
        end
        $display("\nhead %0d", debug_head);
        $display("tail %0d", debug_tail);
        $display("reset %0d", reset);
    `endif
    endfunction

    function void clear_inputs();
        rd_num = 0;
        wr_num = 0;
        br_en = 0;
        reset = 0;
    endfunction

endmodule