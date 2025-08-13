`include "sys_defs.svh"
`include "ISA.svh"

module btb #(
    parameter DEPTH = `BRANCH_TARGET_BUFFER_SIZE,
    parameter PREFETCH_DISTANCE = `PREFETCH_DISTANCE,
    parameter PREFETCH_INSTS = `PREFETCH_DISTANCE*2
)
(
    input                                           clock, 
    input                                           reset,

    input ADDR      [PREFETCH_INSTS-1:0]            rd_pc,

    input logic                                     wr_en,
    input ADDR                                      wr_pc,
    input ADDR                                      wr_target,

    output logic    [PREFETCH_INSTS-1:0]         is_branch,
    output ADDR     [PREFETCH_INSTS-1:0]         pred_target
);
    localparam LOG_DEPTH = $clog2(DEPTH);

    ADDR [DEPTH-1:0] btb, next_btb;
    logic [DEPTH-1:0] btb_valid, next_btb_valid;
    logic [DEPTH-1:0][13-LOG_DEPTH:0] btb_tags, next_btb_tags;
    logic [13-LOG_DEPTH:0] wr_tag, rd_tag;
    logic [LOG_DEPTH-1:0] wr_index, rd_index;

    assign {wr_tag, wr_index} = wr_pc[15:2];

    always_comb begin
        pred_target = '0;
        is_branch = '0;
        for (int i = 0; i < PREFETCH_INSTS; i++) begin
            {rd_tag, rd_index} = rd_pc[i][15:2];
            if (btb_valid[rd_index] & btb_tags[rd_index] == rd_tag) begin
                pred_target[i] = btb[rd_index];
                is_branch[i] = 1;
            end
        end
    end

    always_comb begin
        next_btb = btb;
        next_btb_valid = btb_valid;
        next_btb_tags = btb_tags;
        if (wr_en) begin
            next_btb[wr_index] = wr_target;
            next_btb_tags[wr_index] = wr_tag;
            next_btb_valid[wr_index] = 1;
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            btb         <= '0;
            btb_tags    <= '0;
            btb_valid   <= '0;
        end else begin
            btb         <= next_btb;
            btb_tags    <= next_btb_tags;
            btb_valid   <= next_btb_valid;
        end
    end

endmodule