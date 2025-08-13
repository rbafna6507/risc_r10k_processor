// N-way ROB 

// Input: wr_data, N array of packets with [0] being the oldest, [N-1] being the youngest

`include "sys_defs.svh"

module rob #(
    parameter DEPTH = `ROB_SZ,
    parameter N = `N
)
(
    input                                                               clock, 
    input                                                               reset,

    input DECODED_PACKET                        [N-1:0]                 wr_data,
    input logic                                                         complete_taken,
    input PHYS_REG_IDX                          [N-1:0]                 t,
    input PHYS_REG_IDX                          [N-1:0]                 t_old,

    input PHYS_REG_IDX                          [N-1:0]                 complete_t, // comes from the CDB
    input ISSUE_PACKET                          [`SQ_SZ-1:0]            store_complete_t,
    input                                       [$clog2(N+1)-1:0]       num_accept, // input signal from min block, dependent on open_entries 
    input logic                                 [$clog2(DEPTH)-1:0]     br_tail,
    input logic                                                         br_en,
    input logic                                                         dm_stalled,

    input  DATA                                 [N-1:0]                 cdb_wr_data,

    output ROB_PACKET                           [N-1:0]                 retiring_data, // rob entry packet, but want register vals to update architectural map table + free list
    output logic                                [$clog2(N+1)-1:0]       open_entries, // number of open entires AFTER retirement
    output logic                                [$clog2(N+1)-1:0]       num_retired,
    output logic                                [$clog2(DEPTH)-1:0]     out_tail,
    output logic                                                        start_store


    `ifdef DEBUG
    ,   output ROB_PACKET                       [DEPTH-1:0]             debug_entries,
        output logic                            [$clog2(DEPTH)-1:0]     debug_head,
        output logic                            [$clog2(DEPTH)-1:0]     debug_tail,
        output logic                            [$clog2(DEPTH)-1:0]     debug_num_entries
    `endif
);
    localparam LOG_DEPTH = $clog2(DEPTH);

    //typedef enum logic [1:0] {EMPTY, LOAD, FULL} STATE;

    logic [LOG_DEPTH-1:0] head, next_head;
    logic [LOG_DEPTH-1:0] tail, next_tail;
    logic [LOG_DEPTH:0] num_entries, next_num_entries;

    ROB_PACKET [DEPTH-1:0] entries, next_entries;

    assign out_tail = tail + 1; 

    // use head and tail because this updates between clock cycles, so will update to correct value
    // with head and tail on posedge
    // keeping the original version alongside simplified comb logic
    // assign num_entries = (tail >= head) ? (tail - head) : (DEPTH - head + tail);
    assign open_entries = (DEPTH - num_entries + num_retired > N) ? N : DEPTH - num_entries + num_retired;
    // DONE
    // output (up to N) completed entries
    always_comb begin
        next_head = head;
        retiring_data = '0;
        num_retired = '0;
        next_num_entries = num_entries;
        next_entries = entries;
        start_store = '0;

        // Dependent for-loop to retire instructions. 
        // We must retire instructions first in order to accept the highest # of incoming instructions
        for (int i = 0; i < N; ++i) begin
            if ((num_entries == DEPTH || ((head+i) % DEPTH) != tail) && entries[(head+i) % DEPTH].complete) begin
                if(entries[(head + i) % DEPTH].wr_mem) begin
                    if(dm_stalled || start_store == 1) begin
                        break;
                    end
                    start_store = '1;
                end
                if (entries[(head + i) % DEPTH].halt && (dm_stalled || start_store)) begin
                    break;
                end
                retiring_data[i] = entries[(head+i) % DEPTH];
                next_entries[(head+i) % DEPTH] = '0;
                next_head = (((head+i) % DEPTH) + 1) % DEPTH;
                num_retired++;

                if(retiring_data[i].halt) begin
                    break;
                end
            end else begin
                break;
            end
        end

        for(int k=0;k < DEPTH;k++) begin
            for(int j=0;j<N;j++) begin
                if(entries[k].t == complete_t[j]) begin
                    next_entries[k].complete = '1;
                    next_entries[k].data = cdb_wr_data[j];
                    if (j == 0) begin
                        next_entries[k].taken = complete_taken;
                    end
                end
            end
            for(int i=0;i<`SQ_SZ;i++) begin
                if(entries[k].t == store_complete_t[i].decoded_vals.t.reg_idx) begin
                    next_entries[k].complete = '1;
                end
            end
        end

        // These statements are dependent on updated num_accept
        next_tail = (br_en) ? (br_tail) % DEPTH : (tail + num_accept) % DEPTH; // next_tail points to one past the youngest inst
        // needed for loop to squash valid bits
        // if (br_en) begin
        //     for (int i = 0; i < DEPTH; i++) begin
        //         if ((tail < br_tail) & (i > tail | i <= br_tail)) begin
        //             next_entries[i] = '0;
        //         end else if ((tail > br_tail) & (i < tail | i >= br_tail)) begin
        //             next_entries[i] = '0;
        //         end
        //     end
        // end
        // TODO: verify that commenting this out is ok, and just iterating through next entries is ok to count num_entries
        // next_num_entries += num_accept;

        for(int j = 0;j < N; ++j) begin
            if(j < num_accept) begin
                next_entries[(tail+j) % DEPTH].PC = wr_data[j].PC;
                next_entries[(tail+j) % DEPTH].dest_reg_idx = wr_data[j].dest_reg_idx;
                next_entries[(tail+j) % DEPTH].halt = wr_data[j].halt;
                //next_entries[(tail+j) % DEPTH].valid = wr_data[j].valid;
                next_entries[(tail+j) % DEPTH].complete = 0;
                next_entries[(tail+j) % DEPTH].t = t[j];
                next_entries[(tail+j) % DEPTH].t_old = (wr_data[j].dest_reg_idx == '0) ? t[j] : t_old[j];

                next_entries[(tail+j) % DEPTH].wr_mem = wr_data[j].wr_mem;

                next_entries[(tail+j) % DEPTH].is_branch = wr_data[j].cond_branch | wr_data[j].uncond_branch;
                next_entries[(tail+j) % DEPTH].taken = wr_data[j].taken;
                next_entries[(tail+j) % DEPTH].pred_taken = wr_data[j].pred_taken;

                `ifdef DEBUG
                    next_entries[(tail+j) % DEPTH].data = '0;
                `endif
            end
        end

        // two assumptions:
        // - branch is the first instruction in the the dispatched instruction window
        // - only one branch per dispatched instruction window
        // TODO: would be nice to get another pair of eyes on this to update num_entries (entries in use)
        // next_num_entries = 0;
        // for (int i = 0; i < DEPTH; i++) begin
        //     if (next_entries[i].valid) begin
        //         next_num_entries++;
        //     end
        // end

        if (br_en) begin
            if (br_tail <= tail) begin
                next_num_entries = num_entries - num_retired - (tail - br_tail);
            end else begin
                next_num_entries = num_entries - num_retired - (DEPTH - (br_tail - tail));
            end
        end else begin
            next_num_entries = num_entries - num_retired + num_accept;
        end

        `ifdef DEBUG
            debug_entries = entries;
            debug_head = head;
            debug_tail = tail;
            debug_num_entries = num_entries;
        `endif
    end

    // Incoming insts from dispatch (up to min(N, open_entries))
    // advance tail, num_entries += num_accept

    // update state
    always_ff @(posedge clock) begin
        if (reset) begin
            num_entries <= '0;
            head <= '0;
            tail <= '0;
            entries <= '0;
        end else begin
            num_entries <= next_num_entries;
            head <= next_head;
            tail <= next_tail;
            entries <= next_entries;
        end

        `ifndef DC
            $display("rob num entries: %0d", num_entries);
        `endif
    end

    // // DEBUG OUTPUT
    // `ifdef DEBUG
    //     `ifndef DC
    //         always @(posedge clock) begin
    //             $display("====================== ROB ======================");
    //             $display("Entries: ");
    //             $display("dest_reg_idx\tt\tt_old\tcomplete\tvalid");
    //             for (int j = 0; j < DEPTH; j++) begin
    //                 $display("\t%4d\t%1d\t%5d\t%8d\t%5d",
    //                     entries[j].dest_reg_idx, entries[j].t, entries[j].t_old,
    //                     entries[j].complete, entries[j].valid
    //                 );
    //             end
    //             $display("num entries in use: %d\n", next_num_entries);
    //             $display("");
    //         end
    //     `endif
    // `endif

endmodule