`include "sys_defs.svh"
`include "ISA.svh"

module mult_stage (
    input clock, reset, start,
    input [63:0] prev_sum, mplier, mcand,
    input MULT_FUNC func,
    input stall, 

    output logic [63:0] product_sum, next_mplier, next_mcand,
    output MULT_FUNC, next_func,
    output logic done
);

    parameter SHIFT = 64/`MULT_STAGES;

    logic [63:0] partial_product, shifted_mplier, shifted_mcand;

    assign partial_product = mplier[SHIFT-1:0] * mcand;

    assign shifted_mplier = {SHIFT'('b0), mplier[63:SHIFT]};
    assign shifted_mcand = {mcand[63-SHIFT:0], SHIFT'('b0)};

    always_ff @(posedge clock) begin
        if(stall) begin
            product_sum <= product_sum;
            next_mplier <= next_mplier;
            next_mcand <= next_mcand;
            next_func <= next_func;
        end else begin
            product_sum <= prev_sum + partial_product;
            next_mplier <= shifted_mplier;
            next_mcand  <= shifted_mcand;
            next_func <= func;
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            done <= 1'b0;
        end else begin
            done <= stall ? done : start;
        end
    end

endmodule

module mult_impl (
    input clock, reset,
    input [63:0] mcand, mplier,
    input start,
    input stall,

    output [63:0] product,
    output [`MULT_STAGES-1:0] stall_stages,
    output done
);

    logic [`MULT_STAGES-2:0] internal_dones;
    logic [(64*(`MULT_STAGES-1))-1:0] internal_product_sums, internal_mcands, internal_mpliers;

    logic [63:0] mcand_out, mplier_out; // unused, just for wiring

    logic [`MULT_STAGES-1:0] stall_stages;

    always_comb begin
        stall_stages = '0
        if(stall) begin
            stall_stages[`MULT_STAGES-1] = '1;
            for(int i=`MULT_STAGES-2;i>=0;i++) begin
                if(internal_dones[i] && stall_stages[i-1]) begin
                    stall_stages[i] = '1;
                end
            end
        end
    end

    // instantiate an array of mult_stage modules
    // this uses concatenation syntax for internal wiring, see lab 2 slides
    mult_stage mstage [`MULT_STAGES-1:0] (
        .clock (clock),
        .reset (reset),
        .stall (stall_stages),
        .start       ({internal_dones,        start}), // forward prev done as next start
        .prev_sum    ({internal_product_sums, 64'h0}), // start the sum at 0
        .mplier      ({internal_mpliers,      mplier}),
        .mcand       ({internal_mcands,       mcand}),

        .product_sum ({product,    internal_product_sums}),
        .next_mplier ({mplier_out, internal_mpliers}),
        .next_mcand  ({mcand_out,  internal_mcands}),
        .done        ({done,       internal_dones}) // done when the final stage is done
    );

endmodule


module mult(
    input clock,
    input reset,

    input ISSUE_PACKET is_pack,

    input BR_TASK rem_br_task,
    input BR_MASK rem_b_id,

    input logic stall,
    input logic rd_in,

    output FU_PACKET fu_pack,
    output logic data_ready,
    output logic full
);

    FU_PACKET entries, next_entries;

    logic [`MULT_STAGES-1:0] stall_stages;
    mult_impl multer(
        .clock(clock),
        .reset(reset),

    )

endmodule
