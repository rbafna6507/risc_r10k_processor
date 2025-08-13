`include "sys_defs.svh"

module inst_buffer #(
    parameter DEPTH = `INST_BUFF_DEPTH,
    parameter N = `N
)
(
    input                                           clock,
    input                                           reset,

    input INST_PACKET    [3:0]                      in_insts,
    input logic          [$clog2(N+1)-1:0]          num_dispatch,
    input logic          [2:0]                      num_accept,
    input logic                                     br_en, // only happens when branch squashes
    
    output INST_PACKET   [N-1:0]                    dispatched_insts,
    output               [$clog2(DEPTH+1)-1:0]      open_entries

    `ifdef DEBUG
    ,   INST_PACKET      [DEPTH-1:0]                debug_entries,
        logic            [$clog2(DEPTH)-1:0]        debug_head,
        logic            [$clog2(DEPTH)-1:0]        debug_tail
    `endif

);
    localparam LOG_DEPTH = $clog2(DEPTH);

    logic [LOG_DEPTH-1:0] head, next_head;
    logic [LOG_DEPTH-1:0] tail, next_tail;
    logic [LOG_DEPTH:0] num_entries, next_num_entries;

    INST_PACKET [DEPTH-1:0] entries, next_entries;

    assign open_entries = DEPTH - num_entries + num_dispatch;

    always_comb begin
        dispatched_insts = '0;
        if (~br_en) begin
            for (int i = 0; i < N; ++i) begin
                dispatched_insts[i] = entries[(head+i) % DEPTH];
            end
        end
    end

    always_comb begin
        next_head = (head + num_dispatch) % DEPTH;
        next_num_entries = num_entries - num_dispatch + num_accept;
        next_entries = entries;

        next_tail = (tail + num_accept) % DEPTH;
        
        // head - next_head
        // br_tail - tail
        for (int j = 0; j < N; ++j) begin
            if (j < num_dispatch) begin
                next_entries[(head + j) % DEPTH] = '0;
            end
        end

        for(int j=0;j < 4; ++j) begin
            if(j < num_accept) begin
                next_entries[(tail+j) % DEPTH] = in_insts[j];
            end
        end

        `ifdef DEBUG
            debug_entries = entries;
            debug_head = head;
            debug_tail = tail;
        `endif
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            num_entries <= '0;
            head <= '0;
            tail <= '0;
            entries <= '0;
        end else if (br_en) begin
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
    end

    // `ifdef DEBUG
    //     `ifndef DC
    //         always @(posedge clock) begin
    //             $display("      INST BUFF      ");
    //             $display("---------------------");
    //             $display(" valid  |\tinst ");
    //             for (int i = 0; i < num_entries; i++) begin
    //                 $display("\t%0d\t|\t%0h\t", entries[i].valid, entries[i].inst);
    //             end
    //             $display("num_entries: %0d, num_dispatch: %0d, open_entries: %0d, num_accept: %0d", num_entries, num_dispatch, open_entries, num_accept);
    //         end
    //     `endif
    // `endif

endmodule