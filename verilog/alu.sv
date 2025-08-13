`include "sys_defs.svh"
`include "ISA.svh"

module alu_impl (
    input DATA     opa,
    input DATA     opb,
    input ALU_FUNC alu_func,

    output DATA result
);

    always_comb begin
        case (alu_func)
            ALU_ADD:  result = opa + opb;
            ALU_SUB:  result = opa - opb;
            ALU_AND:  result = opa & opb;
            ALU_SLT:  result = signed'(opa) < signed'(opb);
            ALU_SLTU: result = opa < opb;
            ALU_OR:   result = opa | opb;
            ALU_XOR:  result = opa ^ opb;
            ALU_SRL:  result = opa >> opb[4:0];
            ALU_SLL:  result = opa << opb[4:0];
            ALU_SRA:  result = signed'(opa) >>> opb[4:0]; // arithmetic from logical shift
            // here to prevent latches:
            default:  result = 32'hfacebeec;
        endcase
    end

endmodule // alu


module alu(
    input clock,
    input reset,
    input ISSUE_PACKET is_pack,
    input logic stall,
    input logic rd_in,

    input BR_TASK       rem_br_task,
    input BR_MASK       rem_b_id,

    output FU_PACKET    fu_pack,
    output logic        data_ready

    `ifdef DEBUG
    ,   output FU_PACKET debug_data,
        output FU_PACKET debug_next_data
    `endif
);

    FU_PACKET data, next_data;


    `ifdef DEBUG
        assign debug_data = data;
        assign debug_next_data = next_data;
    `endif

    DATA result, opa, opb;
    alu_impl impl(
        .opa(opa),
        .opb(opb),
        .alu_func(is_pack.decoded_vals.decoded_vals.alu_func),
        .result(result)
    );
    // ALU opA mux
    always_comb begin
        case (is_pack.decoded_vals.decoded_vals.opa_select)
            OPA_IS_RS1:  opa = is_pack.rs1_value;
            OPA_IS_NPC:  opa = is_pack.decoded_vals.decoded_vals.NPC;
            OPA_IS_PC:   opa = is_pack.decoded_vals.decoded_vals.PC;
            OPA_IS_ZERO: opa = 0;
            default:     opa = 32'hdeadface; // dead face
        endcase
    end

    // ALU opB mux
    always_comb begin
        case (is_pack.decoded_vals.decoded_vals.opb_select)
            OPB_IS_RS2:   opb = is_pack.rs2_value;
            OPB_IS_I_IMM: opb = `RV32_signext_Iimm(is_pack.decoded_vals.decoded_vals.inst);
            OPB_IS_S_IMM: opb = `RV32_signext_Simm(is_pack.decoded_vals.decoded_vals.inst);
            OPB_IS_B_IMM: opb = `RV32_signext_Bimm(is_pack.decoded_vals.decoded_vals.inst);
            OPB_IS_U_IMM: opb = `RV32_signext_Uimm(is_pack.decoded_vals.decoded_vals.inst);
            OPB_IS_J_IMM: opb = `RV32_signext_Jimm(is_pack.decoded_vals.decoded_vals.inst);
            default:      opb = 32'hfacefeed; // face feed
        endcase
    end

    assign fu_pack = data;
    assign data_ready = data != '0;

    always_comb begin
        next_data = stall ? data : '0;

        if(!stall && rd_in) begin
            next_data = '{
                result: result,
                decoded_vals: is_pack.decoded_vals,
                pred_correct: 0,
                rs2_value: 0,
                ld_state: 0,
                target_addr: 0
            };
        end

        if((next_data.decoded_vals.b_mask & rem_b_id) != '0) begin
            if(rem_br_task == SQUASH) begin
                next_data = '0;
            end
            if(rem_br_task == CLEAR) begin
                next_data.decoded_vals.b_mask = next_data.decoded_vals.b_mask ^ rem_b_id;
            end
        end
    end

    always_ff @(posedge clock) begin
        if(reset) begin
            data <= '0;
        end else begin
            data <= next_data;
        end
    end
endmodule
