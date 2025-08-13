
`include "sys_defs.svh"

// Compute the correct mult output similar to project 3
module correct_mult (
    input DATA rs1,
    input DATA rs2,
    MULT_FUNC   func,

    output DATA result
);

    logic signed [63:0] signed_mul, mixed_mul;
    logic        [63:0] unsigned_mul;

    assign signed_mul = signed'(rs1) * signed'(rs2);
    assign unsigned_mul = rs1 * rs2;
    // Verilog only does signed multiplication if both arguments are signed :/
    assign mixed_mul = signed'(rs1) * signed'({1'b0, rs2});

    always_comb begin
        case (func)
            M_MUL:    result = signed_mul[31:0];
            M_MULH:   result = signed_mul[63:32];
            M_MULHU:  result = unsigned_mul[63:32];
            M_MULHSU: result = mixed_mul[63:32];
            default:  result = 0;
        endcase
    end

endmodule // correct_mult


module testbench;

    DATA r1, r2, correct_r, prev_r, mul_r;
    MULT_FUNC f;

    logic clock;
    logic reset;
    logic failed;
    ISSUE_PACKET is_pack;
    logic stall;
    logic rd_in;
    FU_PACKET fu_pack;
    logic data_ready;

    assign mul_r = fu_pack.result;


    string fmt;

    mult dut(
        .clock(clock),
        .reset(reset),
        .is_pack(is_pack),
        .stall(stall),
        .rd_in(rd_in),
        .fu_pack(fu_pack),
        .data_ready(data_ready)
    );

    correct_mult not_dut(
        .rs1(r1),
        .rs2(r2),
        .func(f),
        .result(correct_r)
    );


    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end


    task wait_until_done;
        forever begin : wait_loop
            if (data_ready) begin
                disable wait_until_done;
            end
            @(negedge clock);
        end
    endtask

    int cycles;

    always @(posedge clock) begin
        if (rd_in) begin
            cycles = 0;
        end else begin
            cycles++;
        end
    end

    task test;
        input MULT_FUNC func;
        input DATA reg_1, reg_2;
        begin
            @(negedge clock);
            rd_in = 1;
            r1 = reg_1;
            r2 = reg_2;
            f = func;
            is_pack = '0;
            is_pack.decoded_vals.decoded_vals.valid = 1;
            is_pack.rs1_value = reg_1;
            is_pack.rs2_value = reg_2;
            is_pack.decoded_vals.decoded_vals.mult = 1;
            is_pack.decoded_vals.decoded_vals.inst.r.funct3 = func;
            cycles = 0;
            @(negedge clock);
            rd_in = 0;
            is_pack = '0;
            wait_until_done();
            $display(fmt, f.name(), r1, r2, correct_r, mul_r);
            if (correct_r != mul_r) begin
                $display("NOT EQUAL");
                failed = 1;
            end
            if (cycles !== `MULT_STAGES - 1) begin
                $display("WRONG NUM CYCLES: %0d", cycles);
                failed = 1;
            end
            @(negedge clock);
        end
    endtask

    task test_stall;
        input MULT_FUNC func;
        input DATA reg_1, reg_2;
        integer i;
        begin
            @(negedge clock);
            rd_in = 1;
            r1 = reg_1;
            r2 = reg_2;
            f = func;
            is_pack = '0;
            is_pack.decoded_vals.decoded_vals.valid = 1;
            is_pack.rs1_value = reg_1;
            is_pack.rs2_value = reg_2;
            is_pack.decoded_vals.decoded_vals.mult = 1;
            is_pack.decoded_vals.decoded_vals.inst.r.funct3 = func;
            cycles = 0;
            @(negedge clock);
            rd_in = 0;
            is_pack = '0;
            begin
                for (i = 0; i < `MULT_STAGES - 1; i = i + 1) begin
                    @(negedge clock);
                end
            end
            prev_r = mul_r;
            stall = 1;
            @(negedge clock);
            stall = 0;
            wait_until_done();
            if (prev_r != mul_r) begin
                $display("NOT EQUAL");
                failed = 1;
            end
            if (cycles !== `MULT_STAGES) begin
                $display("WRONG NUM CYCLES: %0d", cycles);
                failed = 1;
            end
            $display(fmt, f.name(), r1, r2, prev_r, mul_r);
            @(negedge clock);
        end
    endtask


    initial begin
        clock = 0;
        reset = 1;
        failed = 0;
        stall = 0;
        @(negedge clock);
        @(negedge clock);
        reset = 0;
        @(negedge clock);

        fmt = "%-8s | %3d * %3d = correct: %3d | mul: %3d";
        test(M_MUL, 0, 0);
        test(M_MUL, 1, 0);
        test(M_MUL, 0, 1);
        test(M_MUL, 3, 4);
        test(M_MUL, 2, 15);
        test(M_MUL, 15, 2);
        test(M_MUL, 30, 30);

        fmt = "%-8s | %h * %h = correct: %h | mul: %h";
        $display("");
        test(M_MUL,    32'hff12_3456, 32'hfffff888);
        test(M_MULH,   32'hff12_3456, 32'hfffff888);
        test(M_MULHU,  32'hff12_3456, 32'hfffff888);
        test(M_MULHSU, 32'hff12_3456, 32'hfffff888);

        fmt = "%-8s | %d * %d = correct: %d | mul: %d";
        $display("");
        test(M_MUL,    32'h3 << 30, 4);
        test(M_MULH,   32'h3 << 30, 4);
        test(M_MULHU,  32'h3 << 30, 4);
        test(M_MULHSU, 32'h3 << 30, 4);
        test(M_MUL,    4, 32'h3 << 30);
        test(M_MULH,   4, 32'h3 << 30);
        test(M_MULHU,  4, 32'h3 << 30);
        test(M_MULHSU, 4, 32'h3 << 30);

        fmt = "%-8s | %d * %d = correct: %d | mul: %d";
        $display(""); repeat (10) test(M_MUL,    $random, $random);
        $display(""); repeat (10) test(M_MULH,   $random, $random);
        $display(""); repeat (10) test(M_MULHU,  $random, $random);
        $display(""); repeat (10) test(M_MULHSU, $random, $random);

        // stall testing

        fmt = "%-8s | %d * %d = correct: %d | mul: %d";
        $display(""); repeat (10) test_stall(M_MUL,    $random, $random);
        $display(""); repeat (10) test_stall(M_MULH,   $random, $random);
        $display(""); repeat (10) test_stall(M_MULHU,  $random, $random);
        $display(""); repeat (10) test_stall(M_MULHSU, $random, $random);

        if (failed)
            $display("@@@ Failed\n");
        else
            $display("@@@ Passed\n");

        $finish;
    end

endmodule
