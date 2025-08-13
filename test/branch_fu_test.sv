/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  branch_fu_test.sv                                   //
//                                                                     //
//  Description :  Testbench module for the branch_fu                  //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"
`include "ISA.svh"

module branch_fu_tb();

    logic clock;
    logic reset;
    ISSUE_PACKET is_pack;     
    logic rd_en;   

    FU_PACKET fu_pack;        
    BR_TASK br_task;          
    logic data_ready;  

    
    `ifdef DEBUG
        ADDR debug_branch_target;
    `endif      

    branch_fu dut (
        .clock(clock),
        .reset(reset),
        .is_pack(is_pack),
        .rd_en(rd_en),
        .fu_pack(fu_pack),
        .br_task(br_task),
        .data_ready(data_ready)

        `ifdef DEBUG
        , .debug_branch_target(debug_branch_target)
        `endif 
    );

    // Clock generation
    always begin
        #(`CLOCK_PERIOD / 2.0);
        clock = ~clock;
    end

    // Initialize the test
    initial begin
        $display("\nStart Testbench");

        clock = 0;
        reset = 1;
        rd_en = 0;
        clear_inputs();

        @(negedge clock);

        reset = 0;

        // ------------------------------ Test 1 ------------------------------ //
        $display("\nTest 1: Branch Predicted Correctly, Check Clear and Result");

        is_pack = '0;
        is_pack.decoded_vals.decoded_vals.inst.b.funct3 = 3'b000;
        is_pack.rs1_value = 10;
        is_pack.rs2_value = 10;

        rd_en = 1; 

        is_pack.decoded_vals.decoded_vals.pred_taken = 1;

        @(negedge clock);

        rd_en = 0;       

        assert(fu_pack.result == debug_branch_target);
        assert(br_task == CLEAR);

        $display("PASSED TEST 1");

        // ------------------------------ Test 2 ------------------------------ //
        $display("\nTest 2: Branch Predicted Taken Incorrectly, Check Squash and Result");

        reset = 1;
        clear_inputs();

        @(negedge clock);

        reset = 0;

        is_pack = '0;
        is_pack.decoded_vals.decoded_vals.inst.b.funct3 = 3'b000;
        is_pack.decoded_vals.decoded_vals.NPC = 4;
        is_pack.rs1_value = 15;
        is_pack.rs2_value = 10;

        rd_en = 1; 

        is_pack.decoded_vals.decoded_vals.pred_taken = 1;

        @(negedge clock);

        rd_en = 0;       

        assert(fu_pack.result == is_pack.decoded_vals.decoded_vals.NPC);
        assert(br_task == SQUASH);

        $display("PASSED TEST 2");

         // ------------------------------ Test 3 ------------------------------ //
        $display("\nTest 3: Branch Predicted Not Taken Incorrectly, Check Squash and Result");

        reset = 1;
        clear_inputs();

        @(negedge clock);

        reset = 0;

        is_pack = '0;
        is_pack.decoded_vals.decoded_vals.inst.b.funct3 = 3'b000;
        is_pack.rs1_value = 10;
        is_pack.rs2_value = 10;

        rd_en = 1; 

        is_pack.decoded_vals.decoded_vals.pred_taken = 0;

        @(negedge clock);

        rd_en = 0;       

        assert(fu_pack.result == debug_branch_target);
        assert(br_task == SQUASH);

        $display("PASSED TEST 3");

        $finish;
    end

    // Function to clear inputs before each test
    function void clear_inputs();
        is_pack = '0;
        rd_en = 0;
    endfunction


endmodule

// if it's predicted correctly it should send out a clear
// if it's predicted incorrectly, it should be a squash 
    // -> if we say it's taken, it would be nontaken to NPC
    // -> if we say it's not taken, it would be taken to branch target