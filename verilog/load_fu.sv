`include "sys_defs.svh"
/*
ASSUMPTIONS
    - RS will only issue 1 load at a time
*/

module load_fu #(
    parameter DEPTH=`LD_SZ
)(
    input                                                           clock,
    input                                                           reset,

    input ISSUE_PACKET                                              is_pack,
    input logic                                                     rd_en,

    input logic                                                     Dmem_data_ready,
    input ADDR                                                      Dmem_base_addr,
    input MEM_BLOCK                                                 Dmem_load_data,

    input logic                                                     mshr2cache_wr,
    input logic                                                     Dcache_hit,

    input BR_TASK                                                   rem_br_task,
    input BR_MASK                                                   rem_b_id,

    input logic                                                     dm_stalled,
    input logic                                                     start_store,

    input logic                         [DEPTH-1:0]                 cdb_stall,

    output logic                                                    full,
 
    output logic                                                    dm_squash,
    output logic                                                    start_load,
    output ADDR                                                     Dmem_addr,

    output FU_PACKET                    [DEPTH-1:0]                 fu_pack,
    output logic                        [DEPTH-1:0]                 data_ready

    `ifdef DEBUG
    ,   output FU_PACKET                [DEPTH-1:0]                 debug_entries,
        output logic                    [DEPTH-1:0]                 debug_open_spots,
        output logic                    [DEPTH-1:0]                 debug_ready_spots,
        output logic                    [DEPTH-1:0]                 debug_alloc_spot,
        output logic                    [DEPTH-1:0]                 debug_issued_entry,
        output logic                    [DEPTH-1:0]                 debug_broadcast_entry,
        output logic                    [DEPTH-1:0]                 debug_ld_stall_sig,
        output logic                    [DEPTH-1:0]                 debug_ld_squashed
    `endif
);

    FU_PACKET [DEPTH-1:0] entries, next_entries;
    logic [DEPTH-1:0] open_spots, next_open_spots;
    logic [DEPTH-1:0] query_spots, next_query_spots;
    logic [DEPTH-1:0] ready_spots, next_ready_spots;
    logic [DEPTH-1:0] data_ready_spots, next_data_ready_spots;
    logic [DEPTH-1:0] squashed_spots;

    DATA addr_result;
    basic_adder addr_calc(
        .is_pack(is_pack),
        .result(addr_result)
    );

    // psel to read in new packet
    logic [DEPTH-1:0] alloc_req, alloc_spot;
    assign alloc_req = open_spots | squashed_spots;
    psel_gen #(
        .WIDTH(DEPTH),
        .REQS(1)
    ) allocator(
        .req(open_spots),
        .gnt(alloc_spot),
        .gnt_bus(),
        .empty()
    );

    // psel to start transaction
    logic [DEPTH-1:0] query_req, query_entry;
    assign query_req = query_spots & ~squashed_spots;
    psel_gen #(
        .WIDTH(DEPTH),
        .REQS(1)
    ) queryier(
        .req(query_req),
        .gnt(query_entry),
        .gnt_bus(),
        .empty()
    );

    // psel to start transaction
    logic [DEPTH-1:0] issued_req, issued_entry;
    assign issued_req = ready_spots & ~squashed_spots;
    psel_gen #(
        .WIDTH(DEPTH),
        .REQS(1)
    ) issuer(
        .req(issued_req),
        .gnt(issued_entry),
        .gnt_bus(),
        .empty()
    );

    logic [DEPTH-1:0] broadcast_req, broadcasted_entry;
    assign broadcast_req = data_ready_spots & ~squashed_spots;
    psel_gen #(
        .WIDTH(DEPTH),
        .REQS(DEPTH)
    ) broadcaster (
        .req(broadcast_req),
        .gnt(broadcasted_entry),
        .gnt_bus(),
        .empty()
    );

    assign full = next_open_spots == '0;

    `ifdef DEBUG
        assign debug_entries = entries;
        assign debug_open_spots = open_spots;
        assign debug_ready_spots = ready_spots;
        assign debug_alloc_spot = rd_en ? alloc_spot : '0;
        assign debug_issued_entry = (!dm_stalled && !start_store) ? issued_entry : '0;
        assign debug_broadcast_entry = broadcasted_entry;
        assign debug_ld_stall_sig = cdb_stall;
        assign debug_ld_squashed = squashed_spots;
    `endif

    always_comb begin
        next_entries = entries;
        next_open_spots = open_spots;
        next_query_spots = query_spots;
        next_ready_spots = ready_spots;
        next_data_ready_spots = data_ready_spots;

        squashed_spots = '0;

        start_load = '0;
        Dmem_addr = '0;
        dm_squash = '0;

        fu_pack = '0;
        data_ready = '0;

        for(int i=0;i<DEPTH;i++) begin
            // Read in new issued packet
            if(rd_en && alloc_spot[i]) begin
                next_entries[i] = '{
                    decoded_vals: is_pack.decoded_vals,
                    target_addr: {16'b0, addr_result[15:0]},
                    result: 0,
                    ld_state: READY_TO_QUERY,
                    rs2_value: is_pack.rs2_value,
                    pred_correct: 0
                };
                next_open_spots[i] = '0;

                if(is_pack.decoded_vals.decoded_vals.dest_reg_idx == 0) begin
                    next_entries[i].ld_state = DATA_READY;
                    next_query_spots[i] = '0;
                    next_ready_spots[i] = '0;
                    next_data_ready_spots[i] = '1;
                end else begin
                    next_query_spots[i] = '1;
                    next_ready_spots[i] = '0;
                    next_data_ready_spots[i] = '0;
                end
            end

            // EBR
            if((next_entries[i].decoded_vals.b_mask & rem_b_id) != '0) begin
                if(rem_br_task == SQUASH) begin
                    next_entries[i] = '0;

                    squashed_spots[i] = '1;

                    dm_squash = next_entries[i].ld_state == WAITING_FOR_DATA;

                    next_open_spots[i] = '1;
                    next_query_spots[i] = '0;
                    next_ready_spots[i] = '0;
                    next_data_ready_spots[i] = '0;
                end
                if(rem_br_task == CLEAR) begin
                    next_entries[i].decoded_vals.b_mask ^= rem_b_id;

                    next_open_spots[i] = '0;
                end
            end

            if(!dm_stalled && !start_store && issued_entry[i]) begin
                start_load = 1;
                Dmem_addr = {next_entries[i].target_addr[31:3], 3'b0};

                next_entries[i].ld_state = WAITING_FOR_DATA;

                next_open_spots[i] = '0;
                next_query_spots[i] = '0;
                next_ready_spots[i] = '0;
                next_data_ready_spots[i] = '0;
            end

            if(!start_store && !mshr2cache_wr && query_entry[i]) begin
                next_entries[i].ld_state = READY_TO_ISSUE;
                next_open_spots[i] = '0;
                next_query_spots[i] = '0;
                next_ready_spots[i] = '1;
                next_data_ready_spots[i] = '0;

                if(dm_stalled) begin
                    Dmem_addr = {next_entries[i].target_addr[31:3], 3'b0};
                end
            end

            // Memory transaction completed
            if(Dmem_data_ready && Dmem_base_addr[15:3] == next_entries[i].target_addr[15:3] && (next_entries[i].ld_state == READY_TO_ISSUE  || next_entries[i].ld_state == READY_TO_QUERY || next_entries[i].ld_state == WAITING_FOR_DATA)) begin
                // aligned result
                case(MEM_SIZE'(next_entries[i].decoded_vals.decoded_vals.inst.r.funct3[1:0]))
                    BYTE: begin
                        next_entries[i].result = {24'b0, Dmem_load_data.byte_level[next_entries[i].target_addr[2:0]]};
                        if(!next_entries[i].decoded_vals.decoded_vals.inst.r.funct3[2]) begin
                            next_entries[i].result[31:8] = {(24){next_entries[i].result[7]}};
                        end
                    end
                    HALF: begin 
                        next_entries[i].result = {16'b0, Dmem_load_data.half_level[next_entries[i].target_addr[2:1]]};
                        if(!next_entries[i].decoded_vals.decoded_vals.inst.r.funct3[2]) begin
                            next_entries[i].result[31:16] = {(16){next_entries[i].result[15]}};
                        end
                    end
                    WORD: next_entries[i].result = Dmem_load_data.word_level[next_entries[i].target_addr[2]];
                    DOUBLE: next_entries[i].result = Dmem_load_data.dbbl_level;
                    default: next_entries[i].result = 64'hbadddada;
                endcase

                next_entries[i].ld_state = DATA_READY;

                next_open_spots[i] = '0;
                next_ready_spots[i] = '0;
                next_data_ready_spots[i] = '1;
            end

            if(broadcasted_entry[i]) begin
                fu_pack[i] = next_entries[i];
                data_ready[i] = '1;

                next_open_spots[i] = '0;
                next_ready_spots[i] = '0;
                next_data_ready_spots[i] = '1;
            end

            // CDB Accepted Packet
            if(!cdb_stall[i] && broadcasted_entry[i]) begin
                next_entries[i] = '0;

                next_open_spots[i] = '1;
                next_ready_spots[i] = '0;
                next_data_ready_spots[i] = '0;
            end
        end
    end

    always_ff @(posedge clock) begin
        if(reset) begin
            entries <= '0;
            open_spots <= '1;
            query_spots <= '0;
            ready_spots <= '0;
            data_ready_spots <= '0;
        end else begin
            entries <= next_entries;
            open_spots <= next_open_spots;
            query_spots <= next_query_spots;
            ready_spots <= next_ready_spots;
            data_ready_spots <= next_data_ready_spots;
        end
    end
endmodule;