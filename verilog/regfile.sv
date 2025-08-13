/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  regfile.sv                                          //
//                                                                     //
//  Description :  This module creates the Regfile used by the ID and  //
//                 WB Stages of the Pipeline.                          //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"
//`include "memDP.sv"

module regfile #(
    parameter DEPTH = `PHYS_REG_SZ_R10K,
    parameter NUM_FU = `NUM_FUS,
    parameter N = `N
)(
    input         clock, // system clock
    input         reset,
    // note: no system reset, register values must be written before they can be read
    input  PHYS_REG_IDX     [NUM_FU-1:0]       read_idx_1, read_idx_2, 
    input  PHYS_REG_IDX     [N-1:0]            write_idx,
    input                   [N-1:0]            write_en,
    input  DATA             [N-1:0]            write_data,

    output DATA             [NUM_FU-1:0]       read_out_1, read_out_2
);

    DATA [DEPTH-1:0] data, next_data; 

    always_comb begin
        next_data = data;

        for(int i=0;i<N;i++) begin
            if(write_en[i] && write_idx[i]) begin
                next_data[write_idx[i]] = write_data[i];
            end
        end

        for(int i=0;i<NUM_FU;i++) begin
            read_out_1[i] = next_data[read_idx_1[i]];
            read_out_2[i] = next_data[read_idx_2[i]];
        end
    end

    always_ff @(posedge clock) begin
        if(reset) begin
            data <= 0;
        end else begin
            data <= next_data;
        end
    end

    // `ifdef DEBUG
    //     `ifndef DC
    //         always @(posedge clock) begin
    //             $display("--------------- REGFILE ---------------");

    //             $display("Inputs:");

    //             $display("read_idx_1:");
    //             for (int i = 0; i < NUM_FU; i++) begin
    //                 $write("| %2d", read_idx_1[i]);
    //             end
    //             $display("");

    //             $display("read_idx_2:");
    //             for (int i = 0; i < NUM_FU; i++) begin
    //                 $write("| %2d", read_idx_2[i]);
    //             end
    //             $display("");

    //             $display("write_en:");
    //             for (int i = 0; i < N; i++) begin
    //                 $write("| %2d", write_en[i]);
    //             end
    //             $display("");

    //             $display("write_idx:");
    //             for (int i = 0; i < N; i++) begin
    //                 $write("| %2d", write_idx[i]);
    //             end
    //             $display("");

    //             $display("write_data:");
    //             for (int i = 0; i < N; i++) begin
    //                 $write("| %2d", write_data[i]);
    //             end
    //             $display("\n");

    //             $display("Outputs:");

    //             $display("read_out_1:");
    //             for (int i = 0; i < NUM_FU; i++) begin
    //                 $write("| %2d", read_out_1[i]);
    //             end
    //             $display("");

    //             $display("read_out_2:");
    //             for (int i = 0; i < NUM_FU; i++) begin
    //                 $write("| %2d", read_out_2[i]);
    //             end
    //             $display("\n");
    //         end
    //     `endif
    // `endif

endmodule // regfile
