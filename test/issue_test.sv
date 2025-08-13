/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  issue_test.sv                                       //
//                                                                     //
//  Description :  Testbench module for the issue stage                //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"
`include "ISA.svh"

module issue_tb();
    // Clock generation
    logic                               clock;
    logic                               reset;

    // Input signals
    DATA            [`NUM_FUS-1:0]      reg_data_1;
    DATA            [`NUM_FUS-1:0]      reg_data_2;

    RS_PACKET       [`NUM_FU_ALU-1:0]   issued_alu;
    RS_PACKET       [`NUM_FU_MULT-1:0]  issued_mult;
    RS_PACKET       [`NUM_FU_LD-1:0]    issued_ld;
    RS_PACKET       [`NUM_FU_STORE-1:0] issued_st;
    RS_PACKET                           issued_br;

    // Output signals
    logic          [`NUM_FU_ALU-1:0]    alu_rd_en;
    logic          [`NUM_FU_MULT-1:0]   mult_rd_en;
    logic          [`NUM_FU_LD-1:0]     ld_rd_en;
    logic          [`NUM_FU_STORE-1:0]  st_rd_en;
    logic                               br_rd_en;

    ISSUE_PACKET   [`NUM_FU_ALU-1:0]    issued_alu_pack;
    ISSUE_PACKET   [`NUM_FU_MULT-1:0]   issued_mult_pack;
    ISSUE_PACKET   [`NUM_FU_LD-1:0]     issued_ld_pack;
    ISSUE_PACKET   [`NUM_FU_STORE-1:0]  issued_st_pack;
    ISSUE_PACKET                        issued_br_pack;

    PHYS_REG_IDX   [`NUM_FUS-1:0]       reg_idx_1;
    PHYS_REG_IDX   [`NUM_FUS-1:0]       reg_idx_2;

    // Expected values for verification
    logic          [`NUM_FU_ALU-1:0]    expected_alu_rd_en;
    PHYS_REG_IDX   [`NUM_FU_ALU-1:0]    expected_alu_reg_idx_1;
    PHYS_REG_IDX   [`NUM_FU_ALU-1:0]    expected_alu_reg_idx_2;

    // Base DECODED_PACKET for initializing instructions
    DECODED_PACKET base_decoded_packet;

    // Instantiate the DUT
    issue dut (
        .clock(clock),
        .reset(reset),
        .reg_data_1(reg_data_1),
        .reg_data_2(reg_data_2),
        .issued_alu(issued_alu),
        .issued_mult(issued_mult),
        .issued_ld(issued_ld),
        .issued_st(issued_st),
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

    // Clock generation
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    // Initialize test inputs
    task init_test_inputs();
        // Initialize base decoded packet
        base_decoded_packet = '{
            valid: 1'b0,
            inst: '0,
            PC: '0,
            NPC: 'd4,
            fu_type: ALU_INST,
            reg1: '0,
            reg2: '0,
            opa_select: OPA_IS_RS1,
            opb_select: OPB_IS_RS2,
            dest_reg_idx: '0,
            alu_func: ALU_ADD,
            mult: 1'b0,
            rd_mem: 1'b0,
            wr_mem: 1'b0,
            cond_branch: 1'b0,
            uncond_branch: 1'b0,
            halt: 1'b0,
            illegal: 1'b0,
            csr_op: 1'b0,
            pred_taken: 1'b0
        };

        // Initialize all input signals
        reg_data_1 = '0;
        reg_data_2 = '0;

        // Initialize all RS_PACKETs
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            issued_alu[i].decoded_vals = base_decoded_packet;
            issued_alu[i].t = '{reg_idx: '0, valid: 1'b0};
            issued_alu[i].t1 = '{reg_idx: '0, ready: 1'b0, valid: 1'b0};
            issued_alu[i].t2 = '{reg_idx: '0, ready: 1'b0, valid: 1'b0};
            issued_alu[i].b_mask = '0;
            issued_alu[i].b_id = '0;
        end

        // Similarly initialize mult, ld, st packets
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            issued_mult[i].decoded_vals = base_decoded_packet;
            issued_mult[i].t = '{reg_idx: '0, valid: 1'b0};
            issued_mult[i].t1 = '{reg_idx: '0, ready: 1'b0, valid: 1'b0};
            issued_mult[i].t2 = '{reg_idx: '0, ready: 1'b0, valid: 1'b0};
            issued_mult[i].b_mask = '0;
            issued_mult[i].b_id = '0;
        end

        // Initialize branch packet
        issued_br.decoded_vals = base_decoded_packet;
        issued_br.t = '{reg_idx: '0, valid: 1'b0};
        issued_br.t1 = '{reg_idx: '0, ready: 1'b0, valid: 1'b0};
        issued_br.t2 = '{reg_idx: '0, ready: 1'b0, valid: 1'b0};
        issued_br.b_mask = '0;
        issued_br.b_id = '0;

        issued_st = '0;
        issued_ld = '0;
    endtask

    // Test sequence
    initial begin
        $display("\nStart Issue Module Testbench");

        // Initialize signals
        clock = 0;
        reset = 1;
        init_test_inputs();

        // Wait for 2 clock cycles with reset
        @(negedge clock);
        @(negedge clock);
        reset = 0;

        // ------------------------------ Test 1 ------------------------------ //
        $display("\nTest 1: Basic ALU Issue");
        
        // Setup ALU instruction issue
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            // Set up decoded_vals
            issued_alu[i].decoded_vals = base_decoded_packet;
            issued_alu[i].decoded_vals.valid = 1'b1;
            issued_alu[i].decoded_vals.fu_type = ALU_INST;
            
            // Set up register indices and values
            issued_alu[i].t1.reg_idx = i;
            issued_alu[i].t1.valid = 1'b1;
            issued_alu[i].t1.ready = 1'b1;
            
            issued_alu[i].t2.reg_idx = i + 1;
            issued_alu[i].t2.valid = 1'b1;
            issued_alu[i].t2.ready = 1'b1;

            reg_data_1[i] = i * 2;
            reg_data_2[i] = i * 2 + 1;
            
            expected_alu_rd_en[i] = 1'b1;
            expected_alu_reg_idx_1[i] = i;
            expected_alu_reg_idx_2[i] = i + 1;

            $display("meep: %d", expected_alu_reg_idx_2[i]);

        end
        @(negedge clock);
        @(negedge clock);
        check_alu_outputs();
        $display("@@@ PASSED TEST 1");

        // ------------------------------ Test 2 ------------------------------ //
        $display("\nTest 2: Reset Behavior");
        
        reset = 1;
        @(negedge clock);
        check_reset_state();
        reset = 0;
        $display("@@@ PASSED TEST 2");

        // ------------------------------ Test 3 ------------------------------ //
        $display("\nTest 3: Multiple Valid/Invalid Instructions");
        
        init_test_inputs();
        
        // Set up mixed valid/invalid instructions
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            issued_alu[i].decoded_vals = base_decoded_packet;
            issued_alu[i].decoded_vals.valid = (i % 2 == 0); // Alternate valid/invalid
            issued_alu[i].decoded_vals.fu_type = ALU_INST;
            issued_alu[i].t1.reg_idx = i;
            issued_alu[i].t2.reg_idx = i + 1;
            expected_alu_rd_en[i] = (i % 2 == 0);
        end

        @(negedge clock);
        check_alu_outputs();
        $display("@@@ PASSED TEST 3");

        $display("\n@@@ PASSED ALL TESTS @@@");
        $finish;
    end

    // Helper tasks for verification
    task check_alu_outputs();
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            if (alu_rd_en[i] !== expected_alu_rd_en[i]) begin
                $error("ALU read enable mismatch at index %0d. Expected: %0b, Got: %0b",
                       i, expected_alu_rd_en[i], alu_rd_en[i]);
                $finish;
            end
            if (expected_alu_rd_en[i]) begin
                $display("issued_alu[%d].t1.reg_idx: %d\n", i, issued_alu[i].t1.reg_idx);
                $display("issued_alu[%d].t2.reg_idx: %d\n\n", i,  issued_alu[i].t2.reg_idx);
                $display("expected alu reg 2: %d\n", expected_alu_reg_idx_2[i]);
                // TODO: figure out why these (reg_idx_1 and 2) are equal to X
                $display("reg 1 idx: %b", reg_idx_1);
                $display("reg 2 idx: %b", reg_idx_2);
                if (reg_idx_1[i] !== expected_alu_reg_idx_1[i]) begin
                    $error("ALU reg_idx_1 mismatch at index %0d, %d. Expected: %0d, Got: %0d",
                           i, (`NUM_FUS-`NUM_FU_ALU)+i, expected_alu_reg_idx_1[i], reg_idx_1[i]);
                    $finish;
                end
                if (reg_idx_2[i] !== expected_alu_reg_idx_2[i]) begin
                    $error("ALU reg_idx_2 mismatch at index %0d, %d. Expected: %0d, Got: %0d",
                           i, (`NUM_FUS-`NUM_FU_ALU)+i, expected_alu_reg_idx_2[i], reg_idx_2[i]);
                    $finish;
                end
            end
        end
    endtask

    task check_reset_state();
        if (alu_rd_en !== '0 || mult_rd_en !== '0 || ld_rd_en !== '0 ||
            st_rd_en !== '0 || br_rd_en !== '0) begin
            $error("Reset state mismatch - not all read enables are 0");
            $finish;
        end
    endtask

    // Cycle counter for debug
    int cycle_number = 0;
    always @(posedge clock) begin
        #(`CLOCK_PERIOD * 0.2);
        $display("@@@ Completed cycle %0d @@@", cycle_number);
        cycle_number++;
    end

endmodule