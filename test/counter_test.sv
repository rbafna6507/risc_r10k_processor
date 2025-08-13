

`include "sys_defs.svh"
`include "ISA.svh"


module counter_tb();

    logic                       clock;
    logic                       reset;
    logic                       wr_en;
    logic                       taken;
    logic                       pred;

    counter dut (
        .clock(clock),
        .reset(reset),
        .wr_en(wr_en),
        .taken(taken),
        .pred(pred)
    );

    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    initial begin
        $display("\nStart Testbench");
        $finish;
    end

endmodule