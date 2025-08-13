// SystemVerilog Assertions (SVA) for use with our ROB module
// This file is included by the testbench to separate our main module checking code
// SVA are relatively new to 470, feel free to use them in the final project if you like

`ifndef ROB_SVA_SVH
`define ROB_SVA_SVH

module ROB_sva 
  #(
    parameter DEPTH = `PHYS_REG_SZ_R10K,
    parameter N = `N
  )(
    input                           clock, 
    input                           reset,
    input ROB_ENTRY_PACKET          [N-1:0] wr_data, 
    input PHYS_REG_IDX              [N-1:0] complete_t,
    input                           [$clog2(N+1)-1:0] num_accept,
    
    output ROB_ENTRY_PACKET         [N-1:0] retiring_data,
    output logic                    [$clog2(DEPTH+1)-1:0] open_entries,
    output logic                    [$clog2(N+1)-1:0] num_retired
);
    localparam LOG_DEPTH = $clog2(DEPTH);

    PHYS_REG_IDX [DEPTH-1:0]    rob_dest_regs;  // keep track of entries independently
    logic [DEPTH-1:0]           rob_ready_bits; // keep track of complete insts in rob (one-hot)
    logic [$clog2(DEPTH+1)-1:0] tail, next_tail; // internal, for modifying rob_dest_regs
    logic [$clog2(DEPTH+1)-1:0] num_entries_c; // keep track of entries independently
    logic [$clog2(N+1)-1:0] num_retired_c; // keep track

    // keep internal registers and tail updated
    always_comb begin
        next_tail = tail;
        for (int i = 0; i < N; ++i) begin
            if (i == num_accept | next_tail == DEPTH - 1) begin
                break;
            end
            rob_dest_regs[next_tail] = complete_t[i];
            next_tail += 1;
        end
    end

    // keep internal ready bits updated
    always_comb begin
        for (int i = 0; i < N; ++i) begin
            for (int j = 0; j < DEPTH; ++j) begin
                if (complete_t[i] == 0) begin
                    break;
                end
                if (complete_t[i] == rob_dest_regs[j]) begin
                    rob_ready_bits[j] = 1;
                end
            end
        end
    end

    // keep num_retired_c updated (number to pop)
    always_comb begin
        num_retired_c = N;
        for (int i = 0; i < N; ++i) begin
            if (rob_ready_bits[i] == 0) begin
                num_retired_c = i;
                break;
            end
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            rob_dest_regs <= 0;
            rob_ready_bits <= 0;
            num_entries_c <= 0;
            tail <= 0;
        end else begin
            rob_dest_regs <= rob_dest_regs >> num_retired_c;
            rob_ready_bits <= rob_ready_bits >> num_retired_c;
            num_entries_c <= num_entries_c + num_accept - num_retired_c;
            tail <= next_tail;
        end
    end

    task exit_on_error;
        begin
            $display("\n\033[31m@@@ Failed at time %4d\033[0m\n", $time);
            $finish;
        end
    endtask

    clocking cb @(posedge clock);
        // rd_valid asserted if and only if rd_en=1 and there is valid data
        property num_entries_correct;
            num_entries_c;
        endproperty

        // wr_valid asserted if and only if wr_en=1 and buffer not full
        property num_retired_correct;
            num_valid_c;
        endproperty

        // Check that data written in comes out after proper number of reads
        // NOTE: this property isn't used in verification as it runs slowly
        //      However, feel free to reference as an example of a more
        //      complex assertion
        property write_read_correctly;
            logic [WIDTH-1:0] data_in;
            int               idx;
            (wr_valid, data_in=wr_data, idx=(rd_count+entries)) // value is written
            ##[1:$] (rd_valid && rd_count == idx) // wait for previous entries to be read
            |-> rd_data === data_in;              // ensure correct value out
        endproperty

        property rd_valid_live;
            rd_en |-> s_eventually rd_valid;
        endproperty

        property wr_valid_live;
            wr_en |-> s_eventually wr_valid;
        endproperty

    endclocking

    // Assert properties
    ValidRd:    assert property(cb.rd_valid_correct)     else exit_on_error;
    ValidWr:    assert property(cb.wr_valid_correct)     else exit_on_error;
    ValidFull:  assert property(cb.full_correct)         else exit_on_error;
    ValidAFull: assert property(cb.almost_full_correct)  else exit_on_error;

    // Liveness checks
    RdValidLiveness: assert property(cb.rd_valid_live)   else exit_on_error;
    WrValidLiveness: assert property(cb.wr_valid_live)   else exit_on_error;

    // This assertion is large and slow for formal verification, 
    // but it works for a testbench
    DataOutErr: assert property(cb.write_read_correctly) else exit_on_error;

    genvar i;
    generate 
        for (i = 0; i < WIDTH; i++) begin
            cov_bit_i:  cover property(@(posedge clock) wr_data[i]);
        end
    endgenerate
    

endmodule

`endif // FIFO_SVA_SVH