`include "sys_defs.svh"
`include "ISA.svh"
//`include "btb.sv"
//`include "counter.sv"

module predictor #(
    parameter BHR_DEPTH = `BRANCH_HISTORY_REG_SZ,
    parameter BHT_DEPTH = `BRANCH_HISTORY_TABLE_SIZE,
    parameter PREFETCH_DISTANCE = `PREFETCH_DISTANCE,
    parameter PREFETCH_INSTS = `PREFETCH_DISTANCE*2
)
(
    input                                   clock,
    input                                   reset,

    input ADDR                              rd_pc, // pc of current branch
    input logic     [BHR_DEPTH-1:0]         rd_bhr, // current branch history register

    input logic                             wr_en, // enabled when a branch gets resolved to update predictor
    input logic                             wr_taken, // true if resolved branch is taken
    input ADDR                              wr_target, // target address of a branch
    input ADDR                              wr_pc, // pc of the branch instruction
    input logic     [BHR_DEPTH-1:0]         wr_bhr, // branch history register of this instruction when predicted

    output logic    [PREFETCH_INSTS-1:0] pred_taken, // true if predictor predicts branch is taken
    output logic    [PREFETCH_INSTS-1:0] pred_is_branch,
    output ADDR     [PREFETCH_INSTS-1:0] pred_target // predicted target address
);
    logic [PREFETCH_INSTS-1:0]   is_branch;
    ADDR  [PREFETCH_INSTS-1:0]   rd_pcs;

    assign pred_is_branch = is_branch;

    logic [PREFETCH_INSTS-1:0]   [BHR_DEPTH-1:0] rd_index;
    logic [BHR_DEPTH-1:0] wr_index;

    always_comb begin
        rd_pcs = '0;
        for (int i = 0; i < PREFETCH_INSTS; i++) begin
            rd_pcs[i] = rd_pc + (i*4);
        end
    end

    always_comb begin
        rd_index = '0;
        for (int i = 0; i < PREFETCH_INSTS; i++) begin
            rd_index[i] = rd_pcs[i][BHR_DEPTH+1:2] ^ rd_bhr;
        end
    end
    assign wr_index = wr_pc[BHR_DEPTH+1:2] ^ wr_bhr;

    logic [BHT_DEPTH-1:0] bht_taken;
    logic [BHT_DEPTH-1:0] bht_wr_en;
    logic [BHT_DEPTH-1:0] bht_pred;
    
    generate
        genvar i;
        for (i = 0; i < BHT_DEPTH; i++) begin
            counter i_bht (
                .clock(clock),
                .reset(reset),
                .taken(bht_taken[i]),
                .wr_en(bht_wr_en[i]),
                .pred(bht_pred[i])
            );
        end
    endgenerate

    btb bibibop (
        .clock(clock),
        .reset(reset),
        .rd_pc(rd_pcs),
        .wr_en(wr_en),
        .wr_pc(wr_pc),
        .wr_target(wr_target),
        .is_branch(is_branch),
        .pred_target(pred_target)
    );

    always_comb begin
        pred_taken = '0;
        for (int i = 0; i < PREFETCH_INSTS; i++) begin
            pred_taken[i] = is_branch[i] ? bht_pred[rd_index[i]] : 0;
        end
    end

    always_comb begin
        bht_taken = '0;
        bht_wr_en = '0;
        if (wr_en) begin
            bht_taken[wr_index] = wr_taken;
            bht_wr_en[wr_index] = 1;
        end
    end

endmodule