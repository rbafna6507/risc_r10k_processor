/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  decoder_test.sv                                     //
//                                                                     //
//  Description :  Testbench module for the decoder                    //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"
`include "ISA.svh"

module decoder_tb();
    INST_PACKET         inst;
    
    FU_TYPE            fu_type;
    ALU_OPA_SELECT     opa_select;
    ALU_OPB_SELECT     opb_select;
    logic              has_dest;
    ALU_FUNC           alu_func;
    logic              mult;
    logic              rd_mem;
    logic              wr_mem;
    logic              cond_branch;
    logic              uncond_branch;
    logic              csr_op;
    logic              halt;
    logic              illegal;

    FU_TYPE            expected_fu_type;
    ALU_OPA_SELECT     expected_opa_select;
    ALU_OPB_SELECT     expected_opb_select;
    logic              expected_has_dest;
    ALU_FUNC           expected_alu_func;
    logic              expected_mult;
    logic              expected_rd_mem;
    logic              expected_wr_mem;
    logic              expected_cond_branch;
    logic              expected_uncond_branch;
    logic              expected_csr_op;
    logic              expected_halt;
    logic              expected_illegal;

    decoder dut (
        .inst(inst),
        .fu_type(fu_type),
        .opa_select(opa_select),
        .opb_select(opb_select),
        .has_dest(has_dest),
        .alu_func(alu_func),
        .mult(mult),
        .rd_mem(rd_mem),
        .wr_mem(wr_mem),
        .cond_branch(cond_branch),
        .uncond_branch(uncond_branch),
        .csr_op(csr_op),
        .halt(halt),
        .illegal(illegal)
    );

    // Test stimulus
    initial begin
        $display("\nStart Testbench");

        inst = '0;
        set_expected_defaults();

        // ------------------------------ Test 1 ------------------------------ //
        $display("\nTest 1: Check invalid instruction (valid = 0)");
        
        // nop
        inst.valid = 1'b0;
        inst.inst = `RV32_ADD;  
        #1 check_expected();

        $display("@@@ PASSED TEST 1");

        // ------------------------------ Test 2 ------------------------------ //
        $display("\nTest 2: Check basic arithmetic instructions");
        
        inst.valid = 1'b1;
        
        // test ADD, SUB, ADDI
        inst.inst = `RV32_ADD;
        expected_has_dest = `TRUE;
        expected_alu_func = ALU_ADD;
        expected_fu_type = ALU_INST;
        #1 check_expected();

        inst.inst = `RV32_SUB;
        expected_alu_func = ALU_SUB;
        #1 check_expected();

        inst.inst = `RV32_ADDI;
        expected_alu_func = ALU_ADD;
        expected_opb_select = OPB_IS_I_IMM;
        #1 check_expected();

        $display("@@@ PASSED TEST 2");

        // ------------------------------ Test 3 ------------------------------ //
        $display("\nTest 3: Check memory operations");

        inst.inst = `RV32_LW;
        expected_has_dest = `TRUE;
        expected_opb_select = OPB_IS_I_IMM;
        expected_rd_mem = `TRUE;
        expected_fu_type = LD_INST;
        #1 check_expected();

        inst.inst = `RV32_SW;
        expected_has_dest = `FALSE;
        expected_rd_mem = `FALSE;
        expected_wr_mem = `TRUE;
        expected_opb_select = OPB_IS_S_IMM;
        expected_fu_type = STORE_INST;
        #1 check_expected();

        $display("@@@ PASSED TEST 3");

        // ------------------------------ Test 4 ------------------------------ //
        $display("\nTest 4: Check branch instructions");

        inst.inst = `RV32_BEQ;
        expected_wr_mem = `FALSE;
        expected_opa_select = OPA_IS_PC;
        expected_opb_select = OPB_IS_B_IMM;
        expected_cond_branch = `TRUE;
        expected_fu_type = BR_INST;
        #1 check_expected();

        inst.inst = `RV32_JAL;
        expected_has_dest = `TRUE;
        expected_cond_branch = `FALSE;
        expected_uncond_branch = `TRUE;
        expected_opa_select = OPA_IS_PC;
        expected_opb_select = OPB_IS_J_IMM;
        expected_fu_type = BR_INST;
        #1 check_expected();

        $display("@@@ PASSED TEST 4");

        // ------------------------------ Test 5 ------------------------------ //
        $display("\nTest 5: Check multiplication instructions");

        inst.inst = `RV32_MUL;
        expected_uncond_branch = `FALSE;
        expected_has_dest = `TRUE;
        expected_mult = `TRUE;
        expected_opa_select = OPA_IS_RS1;
        expected_opb_select = OPB_IS_RS2;
        expected_fu_type = MULT_INST;
        #1 check_expected();

        $display("@@@ PASSED TEST 5");

        // ------------------------------ Test 6 ------------------------------ //
        $display("\nTest 6: Check illegal instructions");

        // test instruction of all 1s
        inst.inst = '1;  
        set_expected_defaults();
        expected_illegal = `TRUE;
        #1 check_expected();

        $display("@@@ PASSED TEST 6");
        
        $display("\n@@@ PASSED ALL TESTS @@@");
        $finish;
    end

    function void set_expected_defaults();
        expected_fu_type       = ALU_INST;
        expected_opa_select    = OPA_IS_RS1;
        expected_opb_select    = OPB_IS_RS2;
        expected_alu_func      = ALU_ADD;
        expected_has_dest      = `FALSE;
        expected_csr_op        = `FALSE;
        expected_mult          = `FALSE;
        expected_rd_mem        = `FALSE;
        expected_wr_mem        = `FALSE;
        expected_cond_branch   = `FALSE;
        expected_uncond_branch = `FALSE;
        expected_halt          = `FALSE;
        expected_illegal       = `FALSE;
    endfunction

    function void check_expected();
        string error_msg = "";
        
        if (fu_type !== expected_fu_type)
            error_msg = $sformatf("%s\nfu_type: expected %0d, got %0d", error_msg, expected_fu_type, fu_type);
        if (opa_select !== expected_opa_select)
            error_msg = $sformatf("%s\nopa_select: expected %0d, got %0d", error_msg, expected_opa_select, opa_select);
        if (opb_select !== expected_opb_select)
            error_msg = $sformatf("%s\nopb_select: expected %0d, got %0d", error_msg, expected_opb_select, opb_select);
        if (has_dest !== expected_has_dest)
            error_msg = $sformatf("%s\nhas_dest: expected %0d, got %0d", error_msg, expected_has_dest, has_dest);
        if (alu_func !== expected_alu_func)
            error_msg = $sformatf("%s\nalu_func: expected %0d, got %0d", error_msg, expected_alu_func, alu_func);
        if (mult !== expected_mult)
            error_msg = $sformatf("%s\nmult: expected %0d, got %0d", error_msg, expected_mult, mult);
        if (rd_mem !== expected_rd_mem)
            error_msg = $sformatf("%s\nrd_mem: expected %0d, got %0d", error_msg, expected_rd_mem, rd_mem);
        if (wr_mem !== expected_wr_mem)
            error_msg = $sformatf("%s\nwr_mem: expected %0d, got %0d", error_msg, expected_wr_mem, wr_mem);
        if (cond_branch !== expected_cond_branch)
            error_msg = $sformatf("%s\ncond_branch: expected %0d, got %0d", error_msg, expected_cond_branch, cond_branch);
        if (uncond_branch !== expected_uncond_branch)
            error_msg = $sformatf("%s\nuncond_branch: expected %0d, got %0d", error_msg, expected_uncond_branch, uncond_branch);
        if (csr_op !== expected_csr_op)
            error_msg = $sformatf("%s\ncsr_op: expected %0d, got %0d", error_msg, expected_csr_op, csr_op);
        if (halt !== expected_halt)
            error_msg = $sformatf("%s\nhalt: expected %0d, got %0d", error_msg, expected_halt, halt);
        if (illegal !== expected_illegal)
            error_msg = $sformatf("%s\nillegal: expected %0d, got %0d", error_msg, expected_illegal, illegal);
            
        if (error_msg != "") begin
            $display("@@@ Failed");
            $display(error_msg);
            $finish;
        end
    endfunction

endmodule