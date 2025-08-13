/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  icache.sv                                           //
//                                                                     //
//  Description :  The instruction cache module that reroutes memory   //
//                 accesses to decrease misses.                        //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"

/**
 * A quick overview of the cache and memory:
 *
 * We've increased the memory latency from 1 cycle to 100ns. which will be
 * multiple cycles for any reasonable processor. Thus, memory can have multiple
 * transactions pending and coordinates them via memory tags (different meaning
 * than cache tags) which represent a transaction it's working on. Memory tags
 * are 4 bits long since 15 mem accesses can be live at one time, and only one
 * access happens per cycle.
 *
 * On a request, memory responds with the tag it will use for that transaction.
 * Then, ceiling(100ns/clock period) cycles later, it will return the data with
 * the corresponding tag. The 0 tag is a sentinel value and unused. It would be
 * very difficult to push your clock period past 100ns/15=6.66ns, so 15 tags is
 * sufficient.
 *
 * This cache coordinates those memory tags to speed up fetching reused data.
 *
 * Note that this cache is blocking, and will wait on one memory request before
 * sending another (unless the input address changes, in which case it abandons
 * that request). Implementing a non-blocking cache can count towards simple
 * feature points, but will require careful management of memory tags.
 */

module icache #(
    parameter PREFETCH_DISTANCE = `PREFETCH_DISTANCE
)
(
    input clock,
    input reset,

    // From memory
    //input MEM_TAG   Imem2proc_transaction_tag, // Should be zero unless there is a response
    //input MEM_BLOCK Imem2proc_data,
    //input MEM_TAG   Imem2proc_data_tag,

    // From fetch stage
    input ADDR alloc_addr,
    input logic alloc_en,
    input ADDR [PREFETCH_DISTANCE-1:0] proc2Icache_addr, // read addr
    input logic write_en,
    input ADDR write_addr,
    input MEM_BLOCK write_data,
    input BR_TASK br_task,
    // To memory
    //output MEM_COMMAND proc2Imem_command,
    //output ADDR        proc2Imem_addr,

    output MEM_BLOCK [PREFETCH_DISTANCE-1:0]  Icache_data_out, // Data is mem[proc2Icache_addr]
    output logic     [PREFETCH_DISTANCE-1:0]  Icache_valid_out, // When valid is high
    output logic     [PREFETCH_DISTANCE-1:0]  Icache_alloc_out // When valid is high
);

    // Note: cache tags, not memory tags
    logic [12-`ICACHE_LINE_BITS:0] write_tag, alloc_tag;
    logic [`ICACHE_LINE_BITS-1:0] write_index, alloc_index;
    //logic                          got_mem_data;

    logic [PREFETCH_DISTANCE-1:0] [12-`ICACHE_LINE_BITS:0] current_tag;
    logic [PREFETCH_DISTANCE-1:0] [`ICACHE_LINE_BITS-1:0] current_index;

    logic [PREFETCH_DISTANCE-1:0] [`ICACHE_LINE_BITS-1:0] raddr; // TODO

    always_comb begin
        raddr = '0;
        for (int i = 0; i < PREFETCH_DISTANCE; i++) begin
            raddr[i] = proc2Icache_addr[i][`ICACHE_LINE_BITS+2:3];
        end
    end

    // ---- Cache data ---- //

    ICACHE_TAG [`ICACHE_LINES-1:0] icache_tags;

    memDP #(
        .WIDTH     (64),
        .DEPTH     (`ICACHE_LINES),
        .READ_PORTS(PREFETCH_DISTANCE),
        .BYPASS_EN (0))
    icache_mem (
        .clock(clock),
        .reset(reset),
        .re   ('1),
        .raddr(raddr),
        .rdata(Icache_data_out), // TODO SUS?
        .we   (write_en),
        .waddr(write_index),
        .wdata(write_data)
    );
    

    // ---- Addresses and final outputs ---- //

    // 02e4 = 0000 0010| 1110 0| 000

    // 02e8 = 0000 0010| 1110 1| 000
    // 03e8 = 0000 0011| 1110 1| 000

    always_comb begin
        current_tag = '0;
        current_index = '0;
        for (int i = 0; i < PREFETCH_DISTANCE; i++) begin
            {current_tag[i], current_index[i]} = proc2Icache_addr[i][15:3];
        end
    end

    assign {write_tag, write_index} = write_addr[15:3];
    assign {alloc_tag, alloc_index} = alloc_addr[15:3];

    //
    always_comb begin
        Icache_valid_out = '0;
        $display("ICACHE TAGS:");
        $display("| Tags | Current tag | i | valid |");
        for (int i = 0; i < PREFETCH_DISTANCE; i++) begin
            //Icache_valid_out[i] = icache_tags[(current_index+i)% `ICACHE_LINES].valid && (icache_tags[(current_index+i)% `ICACHE_LINES].tags) == (current_tag+i); 
            Icache_valid_out[i] = icache_tags[current_index[i]].valid && ((icache_tags[current_index[i]].tags) == current_tag[i]); 
            Icache_alloc_out[i] = icache_tags[current_index[i]].alloc;
            $write("| %h | %h | %d | %b |\n", icache_tags[current_index[i]].tags, current_tag[i], i, Icache_valid_out[i]);
        end
        $display("");
    end
    // 
    /*
    assign Icache_valid_out =  icache_tags[current_index].valid &&
                              (icache_tags[current_index].tags == current_tag);*/

    // ---- Main cache logic ---- //

    //MEM_TAG current_mem_tag; // The current memory tag we might be waiting on
    //logic miss_outstanding; // Whether a miss has received its response tag to wait on

    //logic changed_addr;
    //logic update_mem_tag;
    //logic unanswered_miss;

    //assign got_mem_data = (current_mem_tag == Imem2proc_data_tag) && (current_mem_tag != 0);

    //assign changed_addr = (current_index != last_index) || (current_tag != last_tag);

    // Set mem tag to zero if we changed_addr, and keep resetting while there is
    // a miss_outstanding. Then set to zero when we got_mem_data.
    // (this relies on Imem2proc_transaction_tag being zero when there is no request)
    // assign update_mem_tag = changed_addr || miss_outstanding || got_mem_data;

    // If we have a new miss or still waiting for the response tag, we might
    // need to wait for the response tag because dcache has priority over icache
    // assign unanswered_miss = changed_addr ? !Icache_valid_out :
                                        // miss_outstanding && (Imem2proc_transaction_tag == 0);

    // Keep sending memory requests until we receive a response tag or change addresses
    // assign proc2Imem_command = (miss_outstanding && !changed_addr) ? MEM_LOAD : MEM_NONE;
    // assign proc2Imem_addr    = {proc2Icache_addr[31:3],3'b0};

    // ---- Cache state registers ---- //

    always_ff @(posedge clock) begin
        //$display("ICACHE: \nicache_write_addr: %b \ncurrent_index: %b \nicache_valid_tags: %b \nicache_tags: %b \ncurrent_tag: %b", proc2Icache_addr, current_index, icache_tags[current_index].valid, icache_tags[current_index].tags, current_tag);
        if (reset) begin
            //last_index       <= -1; // These are -1 to get ball rolling when
            //last_tag         <= -1; // reset goes low because addr "changes"
            //current_mem_tag  <= '0;
            //miss_outstanding <= '0;
            icache_tags      <= '0; // Set all cache tags and valid bits to 0
        end else begin
            //last_index       <= current_index;
            //last_tag         <= current_tag;
            //miss_outstanding <= unanswered_miss;
            // if (update_mem_tag) begin
            //     current_mem_tag <= Imem2proc_transaction_tag;
            // end/*
/*
            if (br_task == SQUASH) begin
                for (int i = 0; i < `ICACHE_LINES; i++) begin
                    icache_tags[i].alloc <= 1'b0;
                end
            end else begin*/
                if (write_en) begin // If data, meaning tag matches
                    $write("ICACHE WRITING %h to idx %d for addr %h (tag: %b)\n", write_data, write_index, write_addr, write_tag);
                    icache_tags[write_index].tags  <= write_tag;
                    icache_tags[write_index].valid <= 1'b1;
                    icache_tags[write_index].alloc <= 1'b0;
                end
                if (alloc_en) begin
                    icache_tags[alloc_index].alloc <= 1'b1;
                end
            //end
        end
        `ifdef DEBUG
            //$write("raddr: %b\n", raddr[PREFETCH_DISTANCE-1:0]);
            for (int i = 0; i < `ICACHE_LINES; i++) begin
                //$write("ICache tag: %d %b %b %b\n", i, icache_tags[i].tags, icache_tags[i].valid, icache_tags[i].alloc);
            end
            for (int i = 0; i < PREFETCH_DISTANCE; i++) begin
                //$write("ICache data out: %d %b %b %b %b\n", i, raddr[i], Icache_data_out[i], Icache_valid_out[i], Icache_alloc_out[i]);
            end
        `endif
        
    end

endmodule // icache
