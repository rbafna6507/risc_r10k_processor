/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  br_stack_test.sv                                    //
//                                                                     //
//  Description :  Testbench module for the br_stack                   //
//                                                                     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////
// for noop shouldn't assign b_id should be 0s
`include "sys_defs.svh"
`include "ISA.svh"

module br_stack_tb();

    parameter DEPTH = `BRANCH_PRED_SZ;
    parameter N = 3;
    localparam LOG_DEPTH = $clog2(DEPTH);

    logic                                                       clock;
    logic                                                       reset;
    DECODED_PACKET                                              dis_inst; 
    MAP_TABLE_PACKET        [`ARCH_REG_SZ-1:0]                  in_mt;
    logic                   [$clog2(`ROB_SZ+1)-1:0]             in_fl_head;
    logic                   [$clog2(`ROB_SZ)-1:0]               in_rob_tail;

    CDB_PACKET              [N-1:0]                             cdb_in;
    BR_TASK                                                     br_task;
    BR_MASK                                                     rem_b_id;

    BR_MASK                                                     assigned_b_id;
    CHECKPOINT                                                  cp_out;
    logic                                                       full;

    `ifdef DEBUG
        CHECKPOINT [DEPTH-1:0] debug_entries;
        logic [DEPTH-1:0] debug_free_entries;
        logic [DEPTH-1:0] debug_stack_gnt;
    `endif

    CHECKPOINT [DEPTH-1:0] model_entries;
   
    br_stack #(
        .DEPTH(DEPTH),
        .N(N)
    )
    dut (
        .clock(clock),
        .reset(reset),
        .dis_inst(dis_inst),  
        .in_fl_head(in_fl_head),
        .in_mt(in_mt),  
        .in_rob_tail(in_rob_tail),  
        .cdb_in(cdb_in),  
        .br_task(br_task),   
        .rem_b_id(rem_b_id),   
        
        .assigned_b_id(assigned_b_id),
        .cp_out(cp_out),
        .full(full)

        `ifdef DEBUG
        ,   .debug_entries(debug_entries),
            .debug_free_entries(debug_free_entries),
            .debug_stack_gnt(debug_stack_gnt)
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
        clear_inputs();

        @(negedge clock);
        @(negedge clock);
        reset = 0;

        
        // ------------------------------ Test 1 ------------------------------ //
        clear_inputs();
        $display("\nTest 1: Test Checkpoint Coming In\n");
        // send in checkpoint and check all the outputs are correct
        
        @(negedge clock);  

        in_mt[0] = {32'd13, 1'b1, 1'b0};
        in_mt[1] = {32'd14, 1'b1, 1'b0}; 
        in_mt[2] = {32'd15, 1'b1, 1'b0};   

        in_fl_head = 5'b00001;
        in_rob_tail = 6'b000100;

        dis_inst.PC = 0;
        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[3].valid = 1;
        model_entries[3].b_id = 4'b1000;
        model_entries[3].b_mask = 4'b1000;
        model_entries[3].rec_mt = in_mt;
        model_entries[3].fl_head = in_fl_head;
        model_entries[3].rob_tail = in_rob_tail;

        @(negedge clock);  
        //print_entries();
        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock);  

        $display("PASSED TEST 1");

        // ------------------------------ Test 2 ------------------------------ //
        $display("\nTest 2: Squash Branch, Check Dependent Checkpoints\n");
        
        // if you squash a branch that came in, 
        // it should get rid of all the dependent checkpoints

        // 2nd checkpoint in

        in_mt[0] = {32'd13, 1'b1, 1'b0};
        in_mt[1] = {32'd14, 1'b1, 1'b0}; 
        in_mt[2] = {32'd15, 1'b1, 1'b0};  

        in_fl_head = 5'b00100;
        in_rob_tail = 6'b000110; 

        dis_inst.PC = 1;
        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[2].valid = 1;
        model_entries[2].b_id = 4'b0100;
        model_entries[2].b_mask = 4'b1100;
        model_entries[2].rec_mt = in_mt;
        model_entries[2].fl_head = in_fl_head;
        model_entries[2].rob_tail = in_rob_tail;

        @(negedge clock);  
        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock);  

        // 3rd checkpoint in

        in_mt[0] = {32'd13, 1'b1, 1'b0};
        in_mt[1] = {32'd14, 1'b1, 1'b0}; 
        in_mt[2] = {32'd15, 1'b1, 1'b0};  

        in_fl_head = 5'b01010;
        in_rob_tail = 6'b000001; 

        dis_inst.PC = 1;
        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[1].valid = 1;
        model_entries[1].b_id = 4'b0010;
        model_entries[1].b_mask = 4'b1110;
        model_entries[1].rec_mt = in_mt;
        model_entries[1].fl_head = in_fl_head;
        model_entries[1].rob_tail = in_rob_tail;

        @(negedge clock);  
        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock); 

        // 4th checkpoint in
        in_mt[0] = {32'd13, 1'b1, 1'b0};
        in_mt[1] = {32'd14, 1'b1, 1'b0}; 
        in_mt[2] = {32'd15, 1'b1, 1'b0};  

        in_fl_head = 5'b00010;
        in_rob_tail = 6'b000010; 

        dis_inst.PC = 1;
        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[0].valid = 1;
        model_entries[0].b_id = 4'b0001;
        model_entries[0].b_mask = 4'b1111;
        model_entries[0].rec_mt = in_mt;
        model_entries[0].fl_head = in_fl_head;
        model_entries[0].rob_tail = in_rob_tail;

        @(negedge clock);  
        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock); 

        // squashing third branch, second and first branch should go too

        rem_b_id = 4'b0100;
        br_task = SQUASH;

        model_entries[0] = '0;
        model_entries[1] = '0;
        model_entries[2] = '0;

        @(negedge clock);

        rem_b_id = '0;
        br_task = '0;

        @(negedge clock);

        $display("PASSED TEST 2");

        // ------------------------------ Test 3 ------------------------------ //
         $display("\nTest 3: Clear Checkpoint, Check Bits in other Checkpoints\n");
        
        // if you clear one of the checkpoints, it should get rid of the 
        // corresponding bits in all of the masks of the other checkpoints

        // add in 3 checkpoints with different branch_ids but one is  
        // clear the second one

        // 2nd checkpoint in

        in_mt[0] = {32'd13, 1'b1, 1'b0};
        in_mt[1] = {32'd14, 1'b1, 1'b0}; 
        in_mt[2] = {32'd15, 1'b1, 1'b0};  
      
        in_fl_head = 5'b00100;
        in_rob_tail = 6'b000110; 

        dis_inst.PC = 1;
        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[2].valid = 1;
        model_entries[2].b_id = 4'b0100;
        model_entries[2].b_mask = 4'b1100;
        model_entries[2].rec_mt = in_mt;
        model_entries[2].fl_head = in_fl_head;
        model_entries[2].rob_tail = in_rob_tail;

        @(negedge clock);  

        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock);  

        // 3rd checkpoint in

        in_mt[0] = {32'd13, 1'b1, 1'b0};
        in_mt[1] = {32'd14, 1'b1, 1'b0}; 
        in_mt[2] = {32'd15, 1'b1, 1'b0};  

        in_fl_head = 5'b01010;
        in_rob_tail = 6'b000001; 

        dis_inst.PC = 1;
        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[1].valid = 1;
        model_entries[1].b_id = 4'b0010;
        model_entries[1].b_mask = 4'b1110;
        model_entries[1].rec_mt = in_mt;
        model_entries[1].fl_head = in_fl_head;
        model_entries[1].rob_tail = in_rob_tail;

        @(negedge clock);  
        print_entries();
        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock); 

        // 4th checkpoint in

        in_mt[0] = {32'd13, 1'b1, 1'b0};
        in_mt[1] = {32'd14, 1'b1, 1'b0}; 
        in_mt[2] = {32'd15, 1'b1, 1'b0};  

        in_fl_head = 5'b00010;
        in_rob_tail = 6'b000010; 

        dis_inst.PC = 1;
        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[0].valid = 1;
        model_entries[0].b_id = 4'b0001;
        model_entries[0].b_mask = 4'b1111;
        model_entries[0].rec_mt = in_mt;
        model_entries[0].fl_head = in_fl_head;
        model_entries[0].rob_tail = in_rob_tail;

        @(negedge clock);  
        //print_entries();
        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock); 

        rem_b_id = 4'b0010;
        br_task = CLEAR;

        model_entries[1] = '0; 
        model_entries[0].b_mask = 4'b1101;

        @(negedge clock);

        rem_b_id = '0;
        br_task = '0;

        $display("PASSED TEST 3");

        // ------------------------------ Test 5 ------------------------------ //
         $display("\nTest 5: Squash and Take in New Checkpoint\n");
        // squash and try to take in a new checkpoint

        clock = 0;
        reset = 1;
        clear_inputs();

        @(negedge clock);
        reset = 0;

        // 1st checkpoint in
        in_mt[0] = {32'd13, 1'b1, 1'b0};
        in_mt[1] = {32'd14, 1'b1, 1'b0}; 
        in_mt[2] = {32'd15, 1'b1, 1'b0};   

        in_fl_head = 5'b00001;
        in_rob_tail = 6'b000100;

        dis_inst.PC = 0;
        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[3].valid = 1;
        model_entries[3].b_id = 4'b1000;
        model_entries[3].b_mask = 4'b1000;
        model_entries[3].rec_mt = in_mt;
        model_entries[3].fl_head = in_fl_head;
        model_entries[3].rob_tail = in_rob_tail;

        @(negedge clock);  
        //print_entries();
        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock);  

        // 2nd checkpoint in

        in_mt[0] = {32'd13, 1'b1, 1'b0};
        in_mt[1] = {32'd14, 1'b1, 1'b0}; 
        in_mt[2] = {32'd15, 1'b1, 1'b0};  

        in_fl_head = 5'b00100;
        in_rob_tail = 6'b000110; 

        dis_inst.PC = 1;
        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[2].valid = 1;
        model_entries[2].b_id = 4'b0100;
        model_entries[2].b_mask = 4'b1100;
        model_entries[2].rec_mt = in_mt;
        model_entries[2].fl_head = in_fl_head;
        model_entries[2].rob_tail = in_rob_tail;

        @(negedge clock);  

        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock);  

        // 3rd checkpoint in

        in_mt[0] = {32'd13, 1'b1, 1'b0};
        in_mt[1] = {32'd14, 1'b1, 1'b0}; 
        in_mt[2] = {32'd15, 1'b1, 1'b0};  

        in_fl_head = 5'b01010;
        in_rob_tail = 6'b000001; 

        dis_inst.PC = 1;
        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[1].valid = 1;
        model_entries[1].b_id = 4'b0010;
        model_entries[1].b_mask = 4'b1110;
        model_entries[1].rec_mt = in_mt;
        model_entries[1].fl_head = in_fl_head;
        model_entries[1].rob_tail = in_rob_tail;

        @(negedge clock);  

        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock); 

        // 4th checkpoint in

        in_mt[0] = {32'd13, 1'b1, 1'b0};
        in_mt[1] = {32'd14, 1'b1, 1'b0}; 
        in_mt[2] = {32'd15, 1'b1, 1'b0};  

        in_fl_head = 5'b00010;
        in_rob_tail = 6'b000010; 

        dis_inst.PC = 1;
        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[0].valid = 1;
        model_entries[0].b_id = 4'b0001;
        model_entries[0].b_mask = 4'b1111;
        model_entries[0].rec_mt = in_mt;
        model_entries[0].fl_head = in_fl_head;
        model_entries[0].rob_tail = in_rob_tail;

        @(negedge clock);  

        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock); 
      
        rem_b_id = 4'b0010;
        br_task = SQUASH;

        model_entries[1] = '0;
        model_entries[0] = '0;

        @(negedge clock);  

        in_mt[0] = {32'd13, 1'b1, 1'b0};
        in_mt[1] = {32'd16, 1'b1, 1'b0}; 
        in_mt[2] = {32'd15, 1'b1, 1'b0};  

        in_fl_head = 5'b01010;
        in_rob_tail = 6'b010010; 

        dis_inst.PC = 3;

        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[1].valid = 1;
        model_entries[1].b_id = 4'b0010;
        model_entries[1].b_mask = 4'b1110;
        model_entries[1].rec_mt = in_mt;
        model_entries[1].fl_head = in_fl_head;
        model_entries[1].rob_tail = in_rob_tail;
        
        @(negedge clock); 

        rem_b_id = '0;
        br_task = '0;
        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock); 

        $display("PASSED TEST 5");

        // ------------------------------ Test 6 ------------------------------ //
         $display("\nTest 6: Clear Checkpoint, Add in a New One\n");
        // when you clear a checkpoint and add in a new one,
        //  want to make sure the bit mask is correct
        
        clock = 0;
        reset = 1;
        clear_inputs();

        @(negedge clock);
        reset = 0;

        // 1st checkpoint in
        in_mt[0] = {32'd13, 1'b1, 1'b0};
        in_mt[1] = {32'd14, 1'b1, 1'b0}; 
        in_mt[2] = {32'd15, 1'b1, 1'b0};   

        in_fl_head = 5'b00001;
        in_rob_tail = 6'b000100;

        dis_inst.PC = 0;
        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[3].valid = 1;
        model_entries[3].b_id = 4'b1000;
        model_entries[3].b_mask = 4'b1000;
        model_entries[3].rec_mt = in_mt;
        model_entries[3].fl_head = in_fl_head;
        model_entries[3].rob_tail = in_rob_tail;

        @(negedge clock);  

        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock);  

        // 2nd checkpoint in

        in_mt[0] = {32'd13, 1'b1, 1'b0};
        in_mt[1] = {32'd14, 1'b1, 1'b0}; 
        in_mt[2] = {32'd15, 1'b1, 1'b0};  

        in_fl_head = 5'b00100;
        in_rob_tail = 6'b000110; 

        dis_inst.PC = 1;
        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[2].valid = 1;
        model_entries[2].b_id = 4'b0100;
        model_entries[2].b_mask = 4'b1100;
        model_entries[2].rec_mt = in_mt;
        model_entries[2].fl_head = in_fl_head;
        model_entries[2].rob_tail = in_rob_tail;

        @(negedge clock);  

        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock);  

        // 3rd checkpoint in
        in_mt[0] = {32'd13, 1'b1, 1'b0};
        in_mt[1] = {32'd14, 1'b1, 1'b0}; 
        in_mt[2] = {32'd15, 1'b1, 1'b0};  

        in_fl_head = 5'b01010;
        in_rob_tail = 6'b000001; 

        dis_inst.PC = 1;
        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[1].valid = 1;
        model_entries[1].b_id = 4'b0010;
        model_entries[1].b_mask = 4'b1110;
        model_entries[1].rec_mt = in_mt;
        model_entries[1].fl_head = in_fl_head;
        model_entries[1].rob_tail = in_rob_tail;

        @(negedge clock);  

        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock); 

        // 4th checkpoint in
        in_mt[0] = {32'd13, 1'b1, 1'b0};
        in_mt[1] = {32'd14, 1'b1, 1'b0}; 
        in_mt[2] = {32'd15, 1'b1, 1'b0};  

        in_fl_head = 5'b00010;
        in_rob_tail = 6'b000010; 

        dis_inst.PC = 1;
        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[0].valid = 1;
        model_entries[0].b_id = 4'b0001;
        model_entries[0].b_mask = 4'b1111;
        model_entries[0].rec_mt = in_mt;
        model_entries[0].fl_head = in_fl_head;
        model_entries[0].rob_tail = in_rob_tail;

        @(negedge clock);  

        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock); 
      
        rem_b_id = 4'b0010;
        br_task = CLEAR;

        model_entries[1] = '0;
        model_entries[0].b_mask = 4'b1101;

        @(negedge clock);  

        in_mt[0] = {32'd13, 1'b1, 1'b1};
        in_mt[1] = {32'd16, 1'b1, 1'b1}; 
        in_mt[2] = {32'd15, 1'b1, 1'b1};  

        in_fl_head = 5'b01010;
        in_rob_tail = 6'b010010; 

        dis_inst.PC = 3;

        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[1].valid = 1;
        model_entries[1].b_id = 4'b0010;
        model_entries[1].b_mask = 4'b1111;
        model_entries[1].rec_mt = in_mt;
        model_entries[1].fl_head = in_fl_head;
        model_entries[1].rob_tail = in_rob_tail;

        @(negedge clock); 

        rem_b_id = '0;
        br_task = '0;
        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock); 

        $display("PASSED TEST 6");

        // ------------------------------ Test 4 ------------------------------ //
        $display("\nTest 4: CDB Outputs Register\n");
        // when cdb outputs a register that's ready, maptable in 
        // checkpoint should also update

        clock = 0;
        reset = 1;
        clear_inputs();

        @(negedge clock);
        reset = 0;

        //in_mt[0] = {32'd14, 1'b0, 1'b0};
        in_mt[1] = {32'd13, 1'b0, 1'b0}; 
        in_mt[2] = {32'd16, 1'b0, 1'b0}; 
        in_mt[3] = {32'd17, 1'b0, 1'b0}; 

        in_mt[0].ready = 0; 

        in_fl_head = 5'b01010;
        in_rob_tail = 6'b010010; 

        dis_inst.PC = 3;

        dis_inst.uncond_branch = 1;
        dis_inst.valid = 1;

        model_entries[3].valid = 1;
        model_entries[3].b_id = 4'b1000;
        model_entries[3].b_mask = 4'b1000;
        model_entries[3].rec_mt = in_mt;
        model_entries[3].fl_head = in_fl_head;
        model_entries[3].rob_tail = in_rob_tail;

        @(negedge clock);

        $display("map table");
        for (int i = 1; i < 4; i++) begin
            $display("i: %0d, reg_idx: %0d, valid: %0d, ready: %0d", i, debug_entries[3].rec_mt[i].reg_idx, debug_entries[3].rec_mt[i].valid, debug_entries[3].rec_mt[i].ready);
        end

        print_cdb();

        dis_inst.uncond_branch = 0;
        dis_inst.valid = 0;

        @(negedge clock);

        cdb_in[0].reg_idx = 32'd2;
        cdb_in[0].p_reg_idx = 32'd16;
        cdb_in[0].reg_val = 4;
        cdb_in[0].valid = 1'b1;

        cdb_in[1].reg_idx = 32'd5;
        cdb_in[1].p_reg_idx = 32'd19;
        cdb_in[1].reg_val = 6;
        cdb_in[1].valid = 1'b1;

        @(negedge clock);

        print_cdb();

        $display("map table again");
        for (int i = 1; i < 4; i++) begin
            $display("i: %0d, reg_idx: %0d, valid: %0d, ready: %0d", i, debug_entries[3].rec_mt[i].reg_idx, debug_entries[3].rec_mt[i].valid, debug_entries[3].rec_mt[i].ready);
        end


        $display("@@@ PASSED ALL TESTS @@@");
        $finish;
    end

    int cycle_number = 0;
    // Correctness Verification
    always @(posedge clock) begin
        #(`CLOCK_PERIOD * 0.2);
        print_model_entries();
        print_entries();
        check_entries();
        $display("\n@@@ FINISHED CYCLE NUMBER: %0d @@@ \n", cycle_number);
        cycle_number++;
    end

// updating

function void clear_inputs();
    dis_inst = 0;
    in_mt = '0;  
    in_rob_tail = 0;
    in_fl_head = 0;
    cdb_in = '0;
    br_task = 0;
    rem_b_id = 0;
    model_entries = '0;
endfunction

function void check_entries();
    for (int i = 0; i < DEPTH; i++) begin
        if (model_entries[i].b_id != debug_entries[i].b_id) begin
            $error("@@@ FAILED @@@");
            $error("Check b_id error: expected %0d, but got %0d", model_entries[i].b_id, debug_entries[i].b_id);
            $finish;
        end
        if (model_entries[i].b_mask != debug_entries[i].b_mask) begin
            $error("@@@ FAILED @@@");
            $error("Check b_mask error: expected %0d, but got %0d", model_entries[i].b_mask, debug_entries[i].b_mask);
            $finish;
        end
        if (model_entries[i].fl_head != debug_entries[i].fl_head) begin
            $error("@@@ FAILED @@@");
            $error("Check fl_head error: expected %0d, but got %0d", model_entries[i].fl_head, debug_entries[i].fl_head);
            $finish;
        end
        if (model_entries[i].rob_tail != debug_entries[i].rob_tail) begin
            $error("@@@ FAILED @@@");
            $error("Check rob_tail error: expected %0d, but got %0d", model_entries[i].rob_tail, debug_entries[i].rob_tail);
            $finish;
        end
    end
endfunction


// function void add_checkpoint(MAP_TABLE_PACKET [`ARCH_REG_SZ-1:0] test_in_mt, logic [$clog2(`ROB_SZ+1)-1:0] test_in_fl_head, logic [$clog2(`PHYS_REG_SZ_R10K)-1:0] test_in_rob_tail, DECODED_PACKET dis_inst_temp);
//     //stack_gnt = data.b_id;
//     in_mt = test_in_mt;
//     in_fl_head = test_in_fl_head;
//     in_rob_tail = test_in_rob_tail;
//     dis_inst = dis_inst_temp;
// endfunction
    
// function void set_task(BR_TASK tasky);
//     br_task = tasky;
// endfunction

// // checking

// function void check_free_entries(logic free);
//     if (debug_free_entries != free) begin
//         $error("@@@ FAILED @@@");
//         $error("Check free entry error: expected %0d, but got %0d", free, debug_free_entries);
//         $finish;
//     end
// endfunction

// function void check_entries();
//     for (int i = 0; i < DEPTH; i++) begin
//         if (model_entries[i].b_id != debug_entries[i].b_id) begin
//             $error("@@@ FAILED @@@");
//             $error("Check entry error: expected %0d, but got %0d", model_entries[i].b_id, debug_entries[i].b_id);
//             $finish;
//         end
//         if (model_entries[i].b_mask != debug_entries[i].b_mask) begin
//             $error("@@@ FAILED @@@");
//             $error("Check entry error: expected %0d, but got %0d", model_entries[i].b_mask, debug_entries[i].b_mask);
//             $finish;
//         end
//         if (model_entries[i].fl_head != debug_entries[i].fl_head) begin
//             $error("@@@ FAILED @@@");
//             $error("Check entry error: expected %0d, but got %0d", model_entries[i].fl_head, debug_entries[i].fl_head);
//             $finish;
//         end
//         if (model_entries[i].rob_tail != debug_entries[i].rob_tail) begin
//             $error("@@@ FAILED @@@");
//             $error("Check entry error: expected %0d, but got %0d", model_entries[i].rob_tail, debug_entries[i].rob_tail);
//             $finish;
//         end
//     end
// endfunction

// printing

function void print_entries();
    $display("\nEntries\n");
    for (int i = 0; i < DEPTH; i++) begin
        $display("index: %0d, b_id: %b, b_mask: %b, fl_head: %0d, rob_tail: %0d", i, debug_entries[i].b_id, debug_entries[i].b_mask, debug_entries[i].fl_head, debug_entries[i].rob_tail);
    end
endfunction

function void print_cdb();
    $display("\nCDB In\n");
    for (int i = 0; i < N; i++) begin
        $display("index: %0d, reg_idx: %b, p_reg_idx: %b, reg_val: %0d, valid: %0d", i, cdb_in[i].reg_idx, cdb_in[i].p_reg_idx, cdb_in[i].reg_val, cdb_in[i].valid);
    end
endfunction

function void print_model_entries();
    $display("\nModel Entries\n");
    for (int i = 0; i < DEPTH; i++) begin
        $display("index: %0d, b_id: %0b, b_mask: %0b, fl_head: %0d, rob_tail: %0d", i, model_entries[i].b_id, model_entries[i].b_mask, model_entries[i].fl_head, model_entries[i].rob_tail);
    end
endfunction

// function void print_free_entries();
//     $display("\nFree Entries: %0d", debug_free_entries);
// endfunction

// function void print_br_task();
//     $display("\nBR Task: %0d", br_task);
// endfunction

// // can use to print out cp_out
// function void print_checkpoint(CHECKPOINT data);
//     $display("\nCheckpoints\n");
//     $display("b_id: %0d, b_mask: %0d, rec_PC: %0d, fl_head: %0d, rob_tail: %0d\n", data.b_id, data.b_mask, data.rec_PC, data.fl_head, data.rob_tail);
// endfunction

// function void print_stack_gnt();
//     $display("\nStack Grant");
//     for (int i = 0; i < DEPTH; i++) begin
//         $display("%0d ", debug_stack_gnt[i]);
//     end
// endfunction

endmodule


// if you squash the first branch that came in, it should get rid of all the checkpoints
// if you clear one of the checkpoints, it should get rid of the corresponding bits in all of the masks of the other checkpoints
// when cdb outputs a register that's updated, recover maptable in checkpoint should also update
// squash and try to take in a new checkpoint
// when you clear a checkpoint and add in a new one, want to make sure the bit mask is correct


