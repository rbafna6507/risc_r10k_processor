// dcache name: cashay

`include "verilog/sys_defs.svh"

module dcache (
    input clock,
    input reset,

    input ADDR      proc2Dcache_addr,

    input logic     is_store,
    input MEM_SIZE  st_size,
    input DATA      in_data,

    input logic     mshr2Dcache_wr,
    input MEM_BLOCK mshr2Dcache_mem_block,

    // To load unit stage
    output logic     Dcache_ld_out,
    output MEM_BLOCK Dcache_data_out, // this is for load inst and store inst
    output logic     Dcache_hit_out, // When valid is high
    output ADDR      Dcache_addr_out  // addr goes to the load unit for a load inst, and mem for a store inst

    `ifdef DEBUG
    ,   output DCACHE_TAG [`DCACHE_LINES-1:0] debug_dcache_tags
    `endif
);

    // Note: cache tags, not memory tags
    logic [12-`DCACHE_LINE_BITS:0] current_tag;
    logic [`DCACHE_LINE_BITS -1:0] current_index;
    logic                          got_mem_data;
    MEM_BLOCK                      wr_cache_data;
    MEM_BLOCK                      rd_cache_data;


    // ---- Cache data ---- //

    DCACHE_TAG [`DCACHE_LINES-1:0] dcache_tags;

    memDP #(
        .WIDTH     ($bits(MEM_BLOCK)),
        .DEPTH     (`DCACHE_LINES),
        .READ_PORTS(1),
        .BYPASS_EN (0))
    dcache_mem (
        .clock(clock),
        .reset(reset),
        .re   (1'b1),
        .raddr(current_index),
        .rdata(rd_cache_data),
        .we   (mshr2Dcache_wr || (Dcache_hit_out && is_store)),
        .waddr(current_index),
        .wdata(wr_cache_data)
    );

    `ifdef DEBUG
        `ifndef DC
            always @(negedge clock) begin #5;
                $display("---- d cache inputs ----");
                $display("  proc2Dcache_addr: %x, tag: %b, idx: %b", proc2Dcache_addr, current_tag, current_index);
                $display("  is_store: %b, st_size: %s, in_data: %x", is_store, st_size.name(), in_data);
                $display("  mshr2Dcache_wr: %b, mshr2Dcache_mem_block: %x, rd_cache_data: %x", mshr2Dcache_wr, mshr2Dcache_mem_block, rd_cache_data);
                $display("---- d cache outputs ----");
                $display("  Dcache_ld_out: %b, Dcache_data_out: %x", Dcache_ld_out, Dcache_data_out);
                $display("  Dcache_hit_out: %b, Dcache_addr_out: %x", Dcache_hit_out, Dcache_addr_out);
            end
        `endif
    `endif

    // ---- Addresses and final outputs ---- //

    assign {current_tag, current_index} = proc2Dcache_addr[15:3];

    assign Dcache_hit_out =  dcache_tags[current_index].valid &&
                              (dcache_tags[current_index].tags == current_tag);

    assign Dcache_ld_out = (Dcache_hit_out && !is_store) || (mshr2Dcache_wr && !is_store); 
    assign Dcache_addr_out = {proc2Dcache_addr[31:3], 3'b0};

    always_comb begin
        wr_cache_data = (mshr2Dcache_wr) ? mshr2Dcache_mem_block : rd_cache_data;
        Dcache_data_out = '0;
        if ((Dcache_hit_out || mshr2Dcache_wr)) begin
            if(is_store) begin
                if (st_size == BYTE) begin
                    wr_cache_data.byte_level[proc2Dcache_addr[2:0]] = in_data;
                end
                if (st_size == HALF) begin
                    wr_cache_data.half_level[proc2Dcache_addr[2:1]] = in_data;
                end
                if (st_size == WORD) begin
                    wr_cache_data.word_level[proc2Dcache_addr[2]] = in_data;
                end
            end
            Dcache_data_out = wr_cache_data;
        end
    end

    // ---- Cache state registers ---- //

    always_ff @(posedge clock) begin
        if (reset) begin
            dcache_tags      <= '0; // Set all cache tags and valid bits to 0
        end else begin
            if (mshr2Dcache_wr) begin
                dcache_tags[current_index].tags  <= current_tag;
                dcache_tags[current_index].valid <= 1'b1;
            end
        end
    end

endmodule