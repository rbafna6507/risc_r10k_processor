/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  regfile_test.sv                                     //
//                                                                     //
//  Description :  no                                                  //
//                                                                     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"
`include "ISA.svh"


module regfile_tb();

    parameter DEPTH = `PHYS_REG_SZ_R10K;
    parameter WIDTH = 32;
    parameter N = 3;
    parameter NUM_FU = `NUM_FU_ALU + `NUM_FU_MULT + `NUM_FU_STORE + `NUM_FU_LD + `NUM_FU_BR;

    logic                             clock;
    logic                             reset;
    PHYS_REG_IDX     [NUM_FU-1:0]     read_idx_1, read_idx_2;

    PHYS_REG_IDX     [N-1:0]     write_idx;
    logic            [N-1:0]     write_en;
    DATA             [N-1:0]     write_data;

    DATA             [NUM_FU-1:0]     read_out_1, read_out_2;

    DATA             [DEPTH-1:0]        regfile_model;

    regfile #(
        .DEPTH(DEPTH),
        .NUM_FU(NUM_FU),
        .N(N)
    ) dut (
        .clock(clock),
        .reset(reset),
        .read_idx_1(read_idx_1),
        .read_idx_2(read_idx_2),
        .write_idx(write_idx),
        .write_en(write_en),
        .write_data(write_data),
        .read_out_1(read_out_1),
        .read_out_2(read_out_2)
    );

    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    always @(posedge clock) begin
        #(`CLOCK_PERIOD * 0.2);
        if (~reset) begin
            check_output();
        end
        clear_inputs();
    end

    always @(posedge clock) begin
        if (reset) begin
            regfile_model = '0;
        end
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
        $display("Test 1: Read 0th reg");
        @(negedge clock);

        $display("PASSED TEST 1");

        // ------------------------------ Test 2 ------------------------------ //
        $display("Test 1: Write and read 1 value from reg");
        @(negedge clock);

        write(1, 42);

        @(negedge clock);

        read_idx_1[0] = 1;
        read_idx_2[0] = 1;

        @(negedge clock);

        $display("PASSED TEST 2");

        // ------------------------------ Test 3 ------------------------------ //
        $display("Test 3: Write and read N values from reg");
        @(negedge clock);

        for (int i = 1; i < N; i++) begin
            write(i, i*2);
        end

        @(negedge clock);
        for (int i = 1; i < N; i++) begin
            read_idx_1[i] = i;
            read_idx_2[i] = i;
        end

        @(negedge clock);

        $display("PASSED TEST 3");

        // ------------------------------ Test 4 ------------------------------ //
        $display("Test 4: Read same values stored in last test");

        @(negedge clock);
        for (int i = 1; i < N; i++) begin
            read_idx_1[i] = i;
            read_idx_2[i] = i;
        end

        @(negedge clock);

        $display("PASSED TEST 4");


        $display("@@@ PASSED ALL TESTS @@@");

        // ------------------------------ Test 5 ------------------------------ //
        $display("Test 5: Read and Write values in same cycles");

        @(negedge clock);
        for (int i = 1; i < N; i++) begin
            read_idx_1[i] = i;
            read_idx_2[i] = i-1;
            write(i, 50-i);
        end

        @(negedge clock);

        for (int i = 1; i < N; i++) begin
            read_idx_1[i] = i;
        end

        @(negedge clock);

        $display("PASSED TEST 5");


        $display("@@@ PASSED ALL TESTS @@@");
        $finish;
    end


    function void write(int idx, int data);
        for (int i = 0; i < N; i++) begin
            if (write_en[i] === 0) begin
                write_en[i] = 1;
                write_idx[i] = idx;
                write_data[i] = data;
                regfile_model[idx] = data;
                break;
            end
        end
    endfunction

    // Helper function to clear inputs to ROB
    function void clear_inputs();
        read_idx_1 = '0;
        read_idx_2 = '0;
        write_idx = '0;
        write_en = '0;
        write_data = '0;
    endfunction

    function void check_output();
        for (int i = 0; i < NUM_FU; i++) begin
            if (regfile_model[read_idx_1[i]] !== read_out_1[i]) begin
                $error("Mismatch in data with model");
                $error("Expected reg[%0d] = %0d", read_idx_1[i], regfile_model[read_idx_1[i]]);
                $error("     Got reg[%0d] = %0d", read_idx_1[i], read_out_1[i]);
                $finish;
            end
            if (regfile_model[read_idx_2[i]] !== read_out_2[i]) begin
                $error("Mismatch in data with model");
                $error("Expected reg[%0d] = %0d", read_idx_2[i], regfile_model[read_idx_2[i]]);
                $error("     Got reg[%0d] = %0d", read_idx_2[i], read_out_2[i]);
                $finish;
            end
        end
    endfunction


    int cycle_number = 0;
    always @(posedge clock) begin
        if (`DEBUG) begin
            $display("@@@ Cycle %2d @@@", cycle_number);
            $display("Time: %0t", $time);
            $display("Reset: %0d", reset);
        end
        cycle_number++;
    end

endmodule