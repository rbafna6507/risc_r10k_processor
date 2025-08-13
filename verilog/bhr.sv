`include "sys_defs.svh"
`include "ISA.svh"

module bhr #(
    parameter DEPTH = `BRANCH_HISTORY_REG_SZ
)
(
    input                           clock, 
    input                           reset,

    input logic                     wr_en,
    input logic                     taken,

    input BR_TASK                   br_task,
    input logic [`BRANCH_HISTORY_REG_SZ-1:0] br_checkpoint_bhr,
    input logic                     br_pred_taken,

    output logic    [DEPTH-1:0]     out_bhr
);
    logic [DEPTH-1:0] state, next_state;
    logic [DEPTH-1:0] br_state;

    assign br_state = {br_checkpoint_bhr[DEPTH-2:0], ~br_pred_taken};

    assign out_bhr = br_task == SQUASH ? br_state : state;

    always_comb begin
        next_state = br_task == SQUASH ? br_state : state;
        next_state = wr_en ? {next_state[DEPTH-2:0], taken}: next_state;
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            state <= '0;
        end else begin
            state <= next_state;
        end
    end

endmodule