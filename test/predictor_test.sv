/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  alu_test.sv                                         //
//                                                                     //
//  Description :  Testbench module for the alu FU                     //
//                                                                     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"
`include "ISA.svh"

module predictor_tb();

    parameter BHR_DEPTH = `BRANCH_HISTORY_REG_SZ;
    parameter BHT_DEPTH = `BRANCH_HISTORY_TABLE_SIZE;
    parameter PREFETCH_DISTANCE = `PREFETCH_DISTANCE;
    parameter PREFETCH_INSTS = `PREFETCH_DISTANCE*2;

    logic               clock; 
    logic               reset;

    ADDR                              rd_pc; // pc of current branch
    logic     [BHR_DEPTH-1:0]         rd_bhr; // current branch history register

    logic                             wr_en; // enabled when a branch gets resolved to update predictor
    logic                             wr_taken; // true if resolved branch is taken
    ADDR                              wr_target; // target address of a branch
    ADDR                              wr_pc; // pc of the branch instruction
    logic     [BHR_DEPTH-1:0]         wr_bhr; // branch history register of this instruction when predicted

    logic    [PREFETCH_INSTS-1:0] pred_taken; // true if predictor predicts branch is taken
    ADDR     [PREFETCH_INSTS-1:0] pred_target; // predicted target address

    predictor #(
        .BHR_DEPTH(`BRANCH_HISTORY_REG_SZ),
        .BHT_DEPTH(`BRANCH_HISTORY_TABLE_SIZE)
    ) dut (
        .clock(clock),
        .reset(reset),
        .rd_pc(rd_pc),
        .rd_bhr(rd_bhr),
        .wr_en(wr_en),
        .wr_taken(wr_taken),
        .wr_target(wr_target),
        .wr_pc(wr_pc),
        .wr_bhr(wr_bhr),
        .pred_taken(pred_taken),
        .pred_target(pred_target)
    );

    always begin 
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    initial begin
        $display("\nStart Testbench");

        clock = 0;
        reset = 1;

        @(negedge clock);
        @(negedge clock);
        clear_inputs();
        reset = 0;

        // ------------------------------ Test 1 ------------------------------ //
        $display("\nTest 1: The test");

        rd_pc = 24;
        rd_bhr = 7;

        wr_en = 1;
        wr_pc = 24;
        wr_bhr = 7;
        wr_taken = 1;
        wr_target = 53;

        @(negedge clock);
        @(negedge clock);
        @(negedge clock);
        @(negedge clock);
        @(negedge clock);
        @(negedge clock);



        $display("@@@ PASSED ALL TESTS");
        $finish;
    end

    function void clear_inputs();
        rd_pc = '0;
        rd_bhr = '0;
        wr_en = '0;
        wr_taken = '0;
        wr_target = '0;
        wr_pc = '0;
        wr_bhr = '0;
    endfunction

    int cycle_number = 0;
    always @(posedge clock) begin
        #(`CLOCK_PERIOD * 0.2);
        if (~reset) begin
            $display("Cycle: %0d", cycle_number);
            for (int i = 0; i < PREFETCH_INSTS; i++) begin
                $display("Prediction i=%0d: taken=%0d, target=%0d", i, pred_taken[i], pred_target[i]);
            end
        end
        cycle_number++;
    end

endmodule