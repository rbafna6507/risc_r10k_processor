`include "sys_defs.svh"
`ifndef _PSEL
`define _PSEL
`include "psel_gen.sv"
`endif

// typedef struct packed {
//     REG_IDX reg_idx;
//     PHYS_REG_IDX p_reg_idx;
//     DATA reg_val;
//     logic valid;
// } CDB_PACKET;

// typedef struct packed {
//     REG_IDX reg_idx;    decoded_packet.dest_reg_idx
//     PHYS_REG_IDX p_reg_idx; rs_packet.t.reg_idx
//     DATA reg_val; result
//     logic valid; decoded_packet.valid
// } FU_PACKET;


module cdb #(
    parameter N = `N,
    parameter NUM_FU = `NUM_FUS_CDB
)
(
    input logic                                         clock,
    input logic                                         reset,

    input logic         [NUM_FU-1:0]                    fu_done,
    input FU_PACKET     [NUM_FU-1:0]                    wr_data,
    input logic                                         taken,

    input BR_TASK                                       rem_br_task,
    input BR_MASK                                       rem_b_id,

    output CDB_PACKET   [N-1:0]                         entries,
    output logic        [NUM_FU-1:0]                    stall_sig

    `ifdef DEBUG
    ,   output logic    [NUM_FU-1:0]                    debug_cdb_gnt,
        output logic    [N-1:0][NUM_FU-1:0]             debug_cdb_gnt_bus
    `endif
);

    logic [NUM_FU-1:0] cdb_gnt;
    logic [N-1:0][NUM_FU-1:0] cdb_gnt_bus;
    CDB_PACKET [N-1:0] next_entries;

    assign stall_sig = cdb_gnt ^ fu_done;

    psel_gen #(
        .WIDTH(NUM_FU),
        .REQS(N)) 
    cdb_arb (
        .req(fu_done), // why is fu_done? is this not how many request signals we allow?
        .gnt(cdb_gnt),
        .gnt_bus(cdb_gnt_bus),
        .empty()
    );

    FU_PACKET [N-1:0] out_entries;
    wor FU_PACKET [N-1:0] selected_packets; // need a little more explanation on this

    generate
        genvar i, j;
        for (i = 0; i < N; i = i + 1) begin
            for (j = 0; j < NUM_FU; j = j + 1) begin
                assign selected_packets[i] = cdb_gnt_bus[i][j] ? wr_data[j] : '0;
            end
        end
    endgenerate

    always_comb begin
        next_entries = '0;
        out_entries = selected_packets;

        // Branch clear or branch squash
        if (rem_br_task == SQUASH) begin
            for (int i = 0; i < N; i++) begin
                if (out_entries[i].decoded_vals.b_id != rem_b_id && out_entries[i].decoded_vals.b_mask & rem_b_id) begin
                    out_entries[i] = '0;
                end
            end
        end 

        if (rem_br_task == CLEAR) begin
            for (int i = 0; i < N; i++) begin
                if (out_entries[i].decoded_vals.b_mask & rem_b_id) begin
                    out_entries[i].decoded_vals.b_mask &= ~rem_b_id;
                end
            end
        end

        for (int i = 0; i < N; i++) begin
            next_entries[i].reg_idx = out_entries[i].decoded_vals.decoded_vals.dest_reg_idx;
            next_entries[i].p_reg_idx = out_entries[i].decoded_vals.t.reg_idx;
            next_entries[i].reg_val = out_entries[i].result;
            next_entries[i].valid = out_entries[i].decoded_vals.decoded_vals.valid;
        end

        next_entries[0].taken = taken;

        `ifdef DEBUG
            debug_cdb_gnt = cdb_gnt;
            debug_cdb_gnt_bus = cdb_gnt_bus;
        `endif
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            entries <= '0;
        end else begin
            entries <= next_entries;
        end
    end

    `ifdef DEBUG
        `ifndef DC
            always @(posedge clock) begin
                $display("stall sig: %0b fu_done: %0b", stall_sig, fu_done );
            end
        `endif
    `endif

endmodule