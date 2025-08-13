`include "sys_defs.svh"

/*
ASSUMPTIONS
    - 
    - Only 1 store is retired at a time
    - Blocking: DM interface outputs a store-in-progress signal which will block store from retiring

WORKFLOW
    DISPATCH
        - Loads/Stores record current SQ Tail (+ offset) in RS_PACKET
        - Stores are issued to the SQ entry saved during dispatch
        - Loads are issued when SQ HEAD passes saved TAIL

    D$ MISS
        - Issue comes in, addr calced, placed in entries next cycle
        - ROB retires store, SQ executes store, DM outputs HIGH store-in-progress
            - ROB will not retire stores if store-in-progress
        - DM completes transaction, DM sets store_complete to HIGH
            - SQ sets FU_Pack and data_ready HIGH next cycle

    D$ HIT
        - Issue
        - ROB retires, SQ executes, DM outputs HIGH store_complete

*/

module sq #(
    parameter DEPTH=`SQ_SZ,
    parameter N=`N
)
(
    input                                                               clock,
    input                                                               reset,

    // Dispatch, allocate entry, increments tail pointer
    input                               [$clog2(N+1)-1:0]               num_store_dispatched,

    // Issue, append store to queue
    input ISSUE_PACKET                  [DEPTH-1:0]                     is_pack,
    input logic                         [DEPTH-1:0]                     rd_en,

    // Set HIGH by ROB when store is retired, send store to memory
    input logic                                                         start_store,

    input logic                                                         br_en,
    input logic                         [$clog2(DEPTH)-1:0]             br_tail,

    output logic                        [$clog2(N+1)-1:0]               open_entries,

    output ADDR                                                         Dmem_addr,
    output DATA                                                         Dmem_store_data,
    output MEM_SIZE                                                     Dmem_size,

    output logic                        [$clog2(DEPTH)-1:0]             sq_head,
    output logic                        [$clog2(DEPTH)-1:0]             sq_tail,
    output logic                                                        sq_full

    `ifdef DEBUG
    ,   output FU_PACKET                [DEPTH-1:0]                     debug_entries,
        output logic                    [$clog2(DEPTH+1)-1:0]           debug_num_entries
    `endif
);


    FU_PACKET [DEPTH-1:0] entries, next_entries;
    logic [$clog2(DEPTH)-1:0] head, next_head;
    logic [$clog2(DEPTH)-1:0] tail, next_tail; // tail points to one-past last SQ entry

    logic [$clog2(DEPTH+1)-1:0] num_entries, next_num_entries; // keeps tracks of # of ALLOCATED entries (dispatched but not issued)

    DATA [DEPTH-1:0] addr_result;

    generate
        genvar i;
        for(i=0;i<DEPTH;i++) begin
            basic_adder addr_calcer(
                .is_pack(is_pack[i]),
                .result(addr_result[i])
            );
        end
    endgenerate

    assign open_entries = ((DEPTH - num_entries + (start_store ? 1 : 0)) > N ? N : (DEPTH - num_entries + (start_store ? 1 : 0)))-1;
    assign sq_full = (DEPTH - num_entries) <= N;
    assign sq_head = next_head;
    assign sq_tail = tail;

    `ifdef DEBUG
        assign debug_entries = entries;
        assign debug_num_entries = num_entries;
    `endif


    always_comb begin
        next_entries = entries;
        next_head = head;
        next_tail = br_en ? br_tail : tail;
        next_num_entries = num_entries - (br_en ? (br_tail <= tail ? (tail - br_tail) : (DEPTH - br_tail + tail)) : 0);

        Dmem_addr = '0;
        Dmem_store_data = '0;
        Dmem_size = '0;

        next_tail = (next_tail + num_store_dispatched) % DEPTH;
        next_num_entries += num_store_dispatched;

        for(int i=0;i<DEPTH;i++) begin
            if(rd_en[i]) begin
                next_entries[is_pack[i].decoded_vals.decoded_vals.sq_tail] = '{decoded_vals: is_pack[i].decoded_vals, target_addr: addr_result[i], rs2_value: is_pack[i].rs2_value, pred_correct: 0, ld_state: 0, result: 0};
                // bruh
            end
        end

        if(start_store) begin
            next_num_entries--;

            Dmem_addr = next_entries[head].target_addr;
            Dmem_store_data = next_entries[head].rs2_value;
            Dmem_size = MEM_SIZE'(next_entries[head].decoded_vals.decoded_vals.inst.r.funct3[1:0]);

            next_entries[head] = '0;

            next_head = (head + 1) % DEPTH;
        end
    end


    always_ff @(posedge clock) begin
        if(reset) begin
            head <= 0;
            tail <= 0;
            entries <= 0;
            num_entries <= 0;
        end else begin
            head <= next_head;
            tail <= next_tail;
            entries <= next_entries;
            num_entries <= next_num_entries;
        end
    end

    `ifdef DEBUG
        `ifndef DC
            always @(posedge clock) begin #2;
                $display("====== STORE QUEUE ======");
                $display("num_entries: %02d nsd: %02d", next_num_entries, num_store_dispatched);
            end
        `endif
    `endif
endmodule