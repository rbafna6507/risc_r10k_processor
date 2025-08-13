`include "verilog/sys_defs.svh"

module mshr (
    input               clock,
    input               reset,

    input   logic       valid,
    input   ADDR        in_addr,
    input   DATA        in_data,
    input   MEM_SIZE    st_size,
    input   logic       is_store,
    input   logic       mshr_squash,

    // From Dcache
    input   logic       Dcache_hit,

    // From memory
    input   MEM_TAG     mem2proc_transaction_tag, // Should be zero unless there is a response
    input   MEM_TAG     mem2proc_data_tag,
    input   MEM_BLOCK   mem2proc_data,

    // To memory
    output  MEM_COMMAND proc2mem_command,

    // To cache
    output  ADDR        mshr2cache_addr,
    output  DATA        mshr2cache_data,
    output  MEM_BLOCK   mshr2cache_mem_block,
    output  MEM_SIZE    mshr2cache_st_size,
    output  logic       mshr2cache_is_store,
    output  logic       mshr2cache_wr,

    // To load and store units
    output  logic       stall

    `ifdef DEBUG
    , output MSHR       debug_mshr
    `endif
);
 
    MSHR mshr, next_mshr;

    assign stall = (mshr.state != NONE);

    `ifdef DEBUG
        `ifndef DC
            always @(posedge clock) begin #5;
                $display("--- mshr inputs ---");
                $display("valid: %b", valid);
                $display("in_addr: %x", in_addr);
                $display("in_data: %x", in_data);
                $display("st_size: %s", st_size.name());
                $display("is_store: %d", is_store);

                $display("Dcache_hit: %d", Dcache_hit);
                $display("mem2proc_transaction_tag: %d", mem2proc_transaction_tag);
                $display("mem2proc_data_tag: %d", mem2proc_data_tag);

                $display("--- mshr outputs ---");
                $display("proc2mem_command: %s", proc2mem_command.name());
                $display("mshr2cache_addr: %x", mshr2cache_addr);
                $display("mshr2cache_data: %x", mshr2cache_data);
                $display("mshr2cache_st_size: %s", mshr2cache_st_size.name());
                $display("mshr2cache_is_store: %d", mshr2cache_is_store);
                $display("mshr2cache_wr: %d", mshr2cache_wr);

                $display("data returned from memory: %x", mem2proc_data);


                $display("mshr state: %s", mshr.state.name());
                $display("mshr stall: %b", stall);
            end
        `endif
    `endif

    //assign mshr2cache_wr = mshr.state == WAITING_FOR_LOAD_DATA;// && mshr.mem_tag !=0 && mem2proc_data_tag == mshr.mem_tag;

    always_comb begin
        next_mshr = mshr;
        mshr2cache_st_size = '0;
        mshr2cache_is_store = '0;
        mshr2cache_addr = '0;
        mshr2cache_data = '0;
        mshr2cache_mem_block = '0;

        proc2mem_command = MEM_NONE;
        mshr2cache_wr = 0;


        if (valid && mshr.state == NONE) begin
            if (Dcache_hit && is_store) begin
                proc2mem_command = MEM_STORE;
            end
            if (!Dcache_hit) begin // make request directly to cashay and see what she says
                proc2mem_command = MEM_LOAD;

                next_mshr.state = WAITING_FOR_LOAD_DATA;
                next_mshr.addr = in_addr;
                next_mshr.data = in_data;
                next_mshr.mem_tag = mem2proc_transaction_tag;
                next_mshr.is_store = is_store;
                next_mshr.st_size = st_size;
            end
        end
        if(mshr.state == WAITING_FOR_LOAD_DATA) begin
            if(mem2proc_data_tag == mshr.mem_tag && mshr.mem_tag !=0) begin
                next_mshr.state = TRANSACTION_COMPLETE;
                next_mshr.mem_data = mem2proc_data;
            end
        end
        if (mshr.state == TRANSACTION_COMPLETE) begin
            next_mshr = '0;
            mshr2cache_wr = 1;
            mshr2cache_addr = mshr.addr;
            mshr2cache_mem_block = mshr.mem_data;

            if (mshr.is_store) begin
                proc2mem_command = MEM_STORE;
                mshr2cache_st_size = mshr.st_size;
                mshr2cache_is_store = mshr.is_store;
                mshr2cache_data = mshr.data;
            end
        end
        if(mshr_squash) begin
            next_mshr = '0;
        end

        `ifdef DEBUG
            debug_mshr = mshr;
        `endif
    end


    always_ff @(posedge clock) begin
        if (reset) begin
            mshr <= '0;
        end else begin
            mshr <= next_mshr;
        end
    end

endmodule