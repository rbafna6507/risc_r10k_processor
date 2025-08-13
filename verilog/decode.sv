/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_id.sv                                         //
//                                                                     //
//  Description :  instruction decode (ID) stage of the pipeline;      //
//                 decode the instruction fetch register operands, and //
//                 compute immediate operand (if applicable)           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"
`include "ISA.svh"

module decode #(
    parameter N = `N
)(
    input                         clock,           // system clock
    input                         reset,           // system reset
    input  INST_PACKET    [N-1:0] insts,

    output DECODED_PACKET [N-1:0] id_packet
);

    logic [N-1:0] has_dest_reg;
    always_comb begin
        for (int i = 0; i < N; i++) begin
            id_packet[i].inst = insts[i].inst;
            id_packet[i].PC   = insts[i].PC;
            id_packet[i].NPC  = insts[i].NPC;
            id_packet[i].bhr  = insts[i].bhr;
            id_packet[i].valid = insts[i].valid;
            id_packet[i].reg1 = (id_packet[i].opa_select == OPA_IS_RS1 && !id_packet[i].halt || id_packet[i].cond_branch || id_packet[i].wr_mem) ? insts[i].inst.r.rs1 : 0;
            id_packet[i].reg2 = (id_packet[i].opb_select == OPB_IS_RS2 && !id_packet[i].halt || id_packet[i].cond_branch || id_packet[i].wr_mem) ?  insts[i].inst.r.rs2 : 0;
            id_packet[i].dest_reg_idx = (has_dest_reg[i]) ? insts[i].inst.r.rd : `ZERO_REG;
            id_packet[i].pred_taken = insts[i].pred_taken;
            id_packet[i].taken = 0; // will be set on resolve
            id_packet[i].sq_tail = 0;
            //$write("DECODE PC %d %d", insts[i].PC, id_packet[i].dest_reg_idx);
        end
    end



    // Instantiate the instruction decoder
    generate
        for (genvar i = 0; i < N; i++) begin : decode_gen
            decoder decoder_0 (
                // Inputs
                .inst  (insts[i]),

                // Outputs
                .fu_type       (id_packet[i].fu_type),
                .opa_select    (id_packet[i].opa_select),
                .opb_select    (id_packet[i].opb_select),
                .alu_func      (id_packet[i].alu_func),
                .has_dest      (has_dest_reg[i]),
                .mult          (id_packet[i].mult),
                .rd_mem        (id_packet[i].rd_mem),
                .wr_mem        (id_packet[i].wr_mem),
                .cond_branch   (id_packet[i].cond_branch),
                .uncond_branch (id_packet[i].uncond_branch),
                .csr_op        (id_packet[i].csr_op),
                .halt          (id_packet[i].halt),
                .illegal       (id_packet[i].illegal)
            );
        end
    endgenerate

endmodule // stage_id
