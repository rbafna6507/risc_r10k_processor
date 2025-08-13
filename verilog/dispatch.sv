`include "sys_defs.svh"

// if you have instructions that don't include a branch, choose rob, rs, valid instruction min
// if you have a branch, that is the first one list of instructions, go up until min of next branch, rob, rs, or valid
// if you have a branch, that's in the middle, go up until that branch and stop

// 00610463 - branch instruction

module dispatch #(
    parameter N = `N
)(
    input                                               clock,
    input                                               reset,
    input logic             [$clog2(N+1)-1:0]           rob_open,
    input logic             [$clog2(N+1)-1:0]           rs_open,
    input logic             [$clog2(N+1)-1:0]           sq_open,
    input logic             [$clog2(`SQ_SZ)-1:0]        sq_tail_in,
    input INST_PACKET       [N-1:0]                     insts,
    input logic                                         bs_full,
    input logic                                         sq_full,

    output logic            [$clog2(N+1)-1:0]           num_dispatch,
    output logic            [$clog2(N+1)-1:0]           num_store_dispatched,
    output DECODED_PACKET   [N-1:0]                     out_insts

    `ifdef DEBUG
    , output logic [$clog2(N+1)-1:0] debug_num_valid_inst,
      output logic [$clog2(N+1)-1:0] debug_dispatch_limit
    `endif
);

    logic [$clog2(N+1)-1:0] num_rob_rs;
    logic [$clog2(N+1)-1:0] num_valid_inst;
    logic [$clog2(N+1)-1:0] limit;

    assign num_rob_rs = rob_open < rs_open ? rob_open : rs_open;
    assign limit = num_valid_inst < num_rob_rs ? num_valid_inst : num_rob_rs;

    `ifdef DEBUG
        assign debug_num_valid_inst = num_valid_inst;
        assign debug_dispatch_limit = limit;
    `endif

    DECODED_PACKET [N-1:0] decoded_insts;

    decode #(
        .N(N)
    ) decode (
        .clock(clock),           // system clock
        .reset(reset),           // system reset
        .insts(insts),

        .id_packet(decoded_insts)
    );


    always_comb begin
        // debugging purposes
        num_valid_inst = 0;
        for (int i = 0; i < N; i++) begin
            if (insts[i].valid) begin
                num_valid_inst++;
            end
        end
    end


    always_comb begin
        num_dispatch = '0;
        out_insts = '0;
        num_store_dispatched = '0;

        for (int i = 0; i < N; i++) begin
            if (decoded_insts[i].valid && i < limit) begin
                if(decoded_insts[i].illegal) begin
                    break;
                end
                if ((decoded_insts[i].uncond_branch || decoded_insts[i].cond_branch)) begin
                    if (!(i ==0 && !bs_full)) begin
                        break;
                    end
                    out_insts[i] = decoded_insts[i];
                end else if (decoded_insts[i].wr_mem || decoded_insts[i].rd_mem) begin
                    if(decoded_insts[i].wr_mem) begin
                        if(sq_full) begin
                            break;
                        end
                        out_insts[i] = decoded_insts[i];
                        out_insts[i].sq_tail = (sq_tail_in + num_store_dispatched) % `SQ_SZ;
                        num_store_dispatched = num_store_dispatched + 1;
                    end else begin
                        out_insts[i] = decoded_insts[i];
                        out_insts[i].sq_tail = (sq_tail_in + num_store_dispatched) % `SQ_SZ;
                    end

                end else begin
                    out_insts[i] = decoded_insts[i];
                end
                num_dispatch++;
            end
        end
    end


    // `ifdef DEBUG
    //     `ifndef DC
    //         always @(posedge clock) begin
    //             $display("      DISPATCH");
    //             $display("---------------------");
    //             $display(" valid  |\tinst ");
    //             for (int i = 0; i < num_dispatch; i++) begin
    //                 $display("\t%0d\t|\t%0h\t", out_insts[i].valid, out_insts[i].inst);
    //             end
    //             $display("num_dispatch: %0d", num_dispatch);
    //         end
    //     `endif
    // `endif

endmodule