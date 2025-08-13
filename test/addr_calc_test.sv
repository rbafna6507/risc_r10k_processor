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

module addr_calc_tb();
    logic               clock; 
    logic               reset;
    ISSUE_PACKET        is_pack;
    logic               stall;
    logic               rd_in;

    FU_PACKET           fu_pack;
    logic               data_ready;

    DECODED_PACKET      disp_pack;
    RS_PACKET           rs_pack;
    DATA                expected;

    // // queue declaration for free_list model
    // FREE_LIST_PACKET free_list_model [$:(DEPTH)];
    // FREE_LIST_PACKET free_list_model_copy [$:(DEPTH)];

    // // other variables
    // FREE_LIST_PACKET entry_one;
    // FREE_LIST_PACKET entry_random;
    // FREE_LIST_PACKET  [2:0] pr_list;
    // logic [$clog2(DEPTH)-1:0] k = 0;

    addr_calc dut (
        .clock(clock),
        .reset(reset),
        .is_pack(is_pack),
        .stall(stall),
        .rd_in(rd_in),
        .fu_out(fu_pack),
        .data_ready(data_ready)
    );

    always begin 
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    initial begin
        $display("\nStart Testbench");

        clock = 0;
        reset = 1;
        rd_in = 0;
        stall = 0;
        disp_pack = '{
            inst: 0,
            PC: 10,
            NPC: 1,
            opa_select: OPA_IS_RS1,
            opb_select: OPB_IS_RS2,
            dest_reg_idx: '0,
            alu_func: ALU_ADD,
            mult: '0,
            rd_mem: '0,
            wr_mem: '0,
            cond_branch: '0,
            uncond_branch: '0,
            halt: '0,
            illegal: '0,
            csr_op: '0,
            fu_type: '0,
            pred_taken: '0,
            valid: 1,
            reg1: 1,
            reg2: 1
        };
        rs_pack = '{
            decoded_vals: disp_pack,
            t: '{reg_idx: 1, valid: 1},
            t1: '{reg_idx: 2, ready: 0, valid: 1},
            t2: '{reg_idx: 2, ready: 0, valid: 1},
            b_mask: '0,
            b_id: '0
        };
        is_pack = '{
            decoded_vals: rs_pack,
            rs1_value: 0,
            rs2_value: 0
        };

        @(negedge clock);
        @(negedge clock);
        reset = 0;
        
        // ------------------------------ Test 1 ------------------------------ //
        $display("\nTest 1: Check JAL");

        disp_pack.opa_select = OPA_IS_PC;
        disp_pack.opb_select = OPB_IS_J_IMM;
        disp_pack.inst[30:21] = 9'b1;
        rs_pack.decoded_vals = disp_pack;
        is_pack.decoded_vals = rs_pack;
        rd_in = 1;

        expected = 12;
        @(negedge clock);
        
        disp_pack.inst[30:21] = 9'b1000;
        rs_pack.decoded_vals = disp_pack;
        is_pack.decoded_vals = rs_pack;
        expected = 26;
        @(negedge clock);

        $display("PASSED TEST 1");

        // ------------------------------ Test 2 ------------------------------ //
        $display("\nTest 1: Check JALR");

        disp_pack.opa_select = OPA_IS_RS1;
        disp_pack.opb_select = OPB_IS_I_IMM;
        is_pack.rs1_value = 3;
        disp_pack.inst = 0;
        disp_pack.inst[30:20] = 10'b10;
        rs_pack.decoded_vals = disp_pack;
        is_pack.decoded_vals = rs_pack;
        rd_in = 1;

        expected = 5;
        @(negedge clock);
        
        is_pack.rs1_value = 8;
        disp_pack.inst = 0;
        disp_pack.inst[30:20] = 10'b10000;
        rs_pack.decoded_vals = disp_pack;
        is_pack.decoded_vals = rs_pack;
        rd_in = 1;

        expected = 24;
        @(negedge clock);

        is_pack.rs1_value = -8;
        disp_pack.inst = 0;
        disp_pack.inst[30:20] = 10'b10000;
        rs_pack.decoded_vals = disp_pack;
        is_pack.decoded_vals = rs_pack;
        rd_in = 1;

        expected = 8;
        @(negedge clock);

        $display("PASSED TEST 2");

        // ------------------------------ Test 3 ------------------------------ //
        $display("\nTest 3: Check basic LW");
        
        disp_pack.opa_select = OPA_IS_RS1;
        disp_pack.opb_select = OPB_IS_I_IMM;
        is_pack.rs1_value = 3;
        disp_pack.inst[30:20] = 10'b10;
        rs_pack.decoded_vals = disp_pack;
        is_pack.decoded_vals = rs_pack;
        rd_in = 1;

        expected = 5;
        @(negedge clock);

        is_pack.rs1_value = -3;
        disp_pack.inst[30:20] = 10'b10;
        rs_pack.decoded_vals = disp_pack;
        is_pack.decoded_vals = rs_pack;
        rd_in = 1;

        expected = -1;
        @(negedge clock);

        $display("PASSED TEST 3");

        // ------------------------------ Test 4 ------------------------------ //
        $display("\nTest 4: Check basic SW");
        
        disp_pack.opa_select = OPA_IS_RS1;
        disp_pack.opb_select = OPB_IS_S_IMM;
        is_pack.rs1_value = 3;
        disp_pack.inst[11:7] = 4'b10;
        rs_pack.decoded_vals = disp_pack;
        is_pack.decoded_vals = rs_pack;
        rd_in = 1;

        expected = 5;
        @(negedge clock);

        is_pack.rs1_value = -3;
        disp_pack.inst[11:7] = 4'b10;
        rs_pack.decoded_vals = disp_pack;
        is_pack.decoded_vals = rs_pack;
        rd_in = 1;

        expected = -1;
        @(negedge clock);

        $display("PASSED TEST 4");

        $finish;
    end

    int cycle_number = 0;
    // Correctness Verification
    always @(posedge clock) begin
        #(`CLOCK_PERIOD * 0.2);
        $display("expected: %d", expected);
        $display("result: %d\n", fu_pack.result);
        check_expected();
        $display("@@@ FINISHED CYCLE NUMBER: %0d @@@ \n", cycle_number);
        cycle_number++;
    end

    function void check_expected();
        if (expected != fu_pack.result) begin
            $error("@@@ FAILED @@@");
            $error("Check entry error: expected %0d, but got %0d", expected, fu_pack.result);
            $finish;
        end
    endfunction

endmodule