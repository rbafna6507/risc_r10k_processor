`include "sys_defs.svh"
`include "ISA.svh"

module issue #(
    parameter NUM_FU = `NUM_FUS
)(
    input                                          clock,
    input                                          reset,

    input  DATA           [NUM_FU-1:0]             reg_data_1,
    input  DATA           [NUM_FU-1:0]             reg_data_2,

    input RS_PACKET       [`NUM_FU_ALU-1:0]        issued_alu,
    input RS_PACKET       [`NUM_FU_MULT-1:0]       issued_mult,
    input RS_PACKET       [`NUM_FU_LD-1:0]         issued_ld,
    input RS_PACKET       [`SQ_SZ-1:0]             issued_st,
    input RS_PACKET                                issued_br,

    output logic          [`NUM_FU_ALU-1:0]        alu_rd_en,
    output logic          [`NUM_FU_MULT-1:0]       mult_rd_en,
    output logic          [`NUM_FU_LD-1:0]         ld_rd_en,
    output logic          [`SQ_SZ-1:0]             st_rd_en,
    output logic                                   br_rd_en,

    output ISSUE_PACKET   [`NUM_FU_ALU-1:0]        issued_alu_pack,
    output ISSUE_PACKET   [`NUM_FU_MULT-1:0]       issued_mult_pack,
    output ISSUE_PACKET   [`NUM_FU_LD-1:0]         issued_ld_pack,
    output ISSUE_PACKET   [`SQ_SZ-1:0]             issued_st_pack,
    output ISSUE_PACKET                            issued_br_pack,

    output PHYS_REG_IDX   [NUM_FU-1:0]             reg_idx_1,
    output PHYS_REG_IDX   [NUM_FU-1:0]             reg_idx_2
);

    logic        [`NUM_FU_ALU-1:0]      alu_rd_en_vals;
    PHYS_REG_IDX [`NUM_FU_ALU-1:0]      alu_reg_1, alu_reg_2;
    ISSUE_PACKET [`NUM_FU_ALU-1:0]      issued_alu_pack_temp;

    logic        [`NUM_FU_MULT-1:0]     mult_rd_en_vals;
    PHYS_REG_IDX [`NUM_FU_MULT-1:0]     mult_reg_1, mult_reg_2;
    ISSUE_PACKET [`NUM_FU_MULT-1:0]     issued_mult_pack_temp;

    logic        [`NUM_FU_LD-1:0]       ld_rd_en_vals;
    PHYS_REG_IDX [`NUM_FU_LD-1:0]       ld_reg_1, ld_reg_2;
    ISSUE_PACKET [`NUM_FU_LD-1:0]       issued_ld_pack_temp;

    logic        [`SQ_SZ-1:0]           st_rd_en_vals;
    PHYS_REG_IDX [`SQ_SZ-1:0]           st_reg_1, st_reg_2;
    ISSUE_PACKET [`SQ_SZ-1:0]           issued_st_pack_temp;

    logic                               br_rd_en_vals;
    PHYS_REG_IDX                        br_reg_1, br_reg_2;
    ISSUE_PACKET                        issued_br_pack_temp;

    // ----- ALU -----

    // alu issuing signals
    always_comb begin
        alu_rd_en_vals = '0;
        for (int i = 0; i <`NUM_FU_ALU; i++) begin
            alu_rd_en_vals[i] = issued_alu[i].decoded_vals.valid;
            alu_reg_1[i] = issued_alu[i].t1.reg_idx;
            alu_reg_2[i] = issued_alu[i].t2.reg_idx;
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            alu_rd_en       <= 0;
            issued_alu_pack <= '0;
        end else begin
            alu_rd_en       <= alu_rd_en_vals;
            issued_alu_pack <= issued_alu_pack_temp;
        end 
    end

    // ----- MULT -----

    // mult issuing signals
    always_comb begin
        mult_rd_en_vals = '0;
        for (int i = 0; i <`NUM_FU_MULT; i++) begin
            mult_rd_en_vals[i] = issued_mult[i].decoded_vals.valid;
            mult_reg_1[i] = issued_mult[i].t1.reg_idx;
            mult_reg_2[i] = issued_mult[i].t2.reg_idx;
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            mult_rd_en          <= 0;
            issued_mult_pack    <= '0;
        end else begin
            mult_rd_en          <= mult_rd_en_vals;
            issued_mult_pack    <= issued_mult_pack_temp;
        end 
    end

    // ----- LD -----

    // load issuing signals
    always_comb begin
        ld_rd_en_vals = '0;
        for (int i = 0; i <`NUM_FU_LD; i++) begin
            ld_rd_en_vals[i] = issued_ld[i].decoded_vals.valid;
            ld_reg_1[i] = issued_ld[i].t1.reg_idx;
            ld_reg_2[i] = issued_ld[i].t2.reg_idx;
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            ld_rd_en        <= 0;
            issued_ld_pack  <= '0;
        end else begin
            ld_rd_en        <= ld_rd_en_vals;
            issued_ld_pack  <= issued_ld_pack_temp;
        end 
    end

    // ----- STORE -----

    // store issuing signals
    always_comb begin
        st_rd_en_vals = '0;
        for (int i = 0; i <`SQ_SZ; i++) begin
            st_rd_en_vals[i] = issued_st[i].decoded_vals.valid;
            st_reg_1[i] = issued_st[i].t1.reg_idx;
            st_reg_2[i] = issued_st[i].t2.reg_idx;
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            st_rd_en        <= 0;
            issued_st_pack  <= '0;
        end else begin
            st_rd_en        <= st_rd_en_vals;
            issued_st_pack  <= issued_st_pack_temp;
        end 
    end

    // ----- BRANCH -----

    // branch issuing signals
    always_comb begin
        br_rd_en_vals = issued_br.decoded_vals.valid;
        br_reg_1 = issued_br.t1.reg_idx;
        br_reg_2 = issued_br.t2.reg_idx;
    end

    always_ff @(posedge clock) begin
        `ifndef DC
            $display("issue register br inst: %0x, PC: %0d", issued_br_pack.decoded_vals.decoded_vals.inst, issued_br_pack.decoded_vals.decoded_vals.PC);
            //$display("br_task: %0s, rem_b_id: %0b, current br b_mask: %0b", br_task.name(), rem_b_id, issued_br.b_mask);
        `endif

        if (reset) begin
            br_rd_en        <= 0;
            issued_br_pack  <= '0;
        end else begin
            br_rd_en        <= br_rd_en_vals;
            issued_br_pack  <= issued_br_pack_temp;
        end 
    end

    // ---- REGFILE INPUT ----

    // assign reg_idx_1 = {alu_reg_1, mult_reg_1, ld_reg_1, st_reg_1, br_reg_1};
    assign reg_idx_1 = {br_reg_1, st_reg_1, ld_reg_1, mult_reg_1, alu_reg_1};
    assign reg_idx_2 = {br_reg_2, st_reg_2, ld_reg_2, mult_reg_2, alu_reg_2};
    // assign reg_idx_2 = {alu_reg_2, mult_reg_2, ld_reg_2, st_reg_2, br_reg_2};

    // ---- ISSUE PACKET ----

    always_comb begin
        issued_alu_pack_temp  = 0;
        issued_mult_pack_temp = 0;
        issued_ld_pack_temp   = 0;
        issued_st_pack_temp   = 0;
        issued_br_pack_temp   = 0;

        // ALU
        for (int a = 0; a < `NUM_FU_ALU; a++) begin
            issued_alu_pack_temp[a].decoded_vals = issued_alu[a];
            issued_alu_pack_temp[a].rs1_value = reg_data_1[a];
            issued_alu_pack_temp[a].rs2_value = reg_data_2[a];
        end

        // MULT
        for (int m = 0; m < `NUM_FU_MULT; m++) begin
            issued_mult_pack_temp[m].decoded_vals = issued_mult[m];
            issued_mult_pack_temp[m].rs1_value = reg_data_1[(`NUM_FU_ALU) + m]; 
            issued_mult_pack_temp[m].rs2_value = reg_data_2[(`NUM_FU_ALU) + m]; 
        end

        // LD
        for (int l = 0; l < `NUM_FU_LD; l++) begin
            issued_ld_pack_temp[l].decoded_vals = issued_ld[l];
            issued_ld_pack_temp[l].rs1_value = reg_data_1[(`NUM_FU_ALU + `NUM_FU_MULT) + l];
            issued_ld_pack_temp[l].rs2_value =reg_data_2[(`NUM_FU_ALU + `NUM_FU_MULT) + l];
        end

        // STORE
        for (int s = 0; s < `SQ_SZ; s++) begin
            issued_st_pack_temp[s].decoded_vals = issued_st[s];
            issued_st_pack_temp[s].rs1_value = reg_data_1[(`NUM_FU_ALU + `NUM_FU_MULT + `NUM_FU_LD) + s];
            issued_st_pack_temp[s].rs2_value = reg_data_2[(`NUM_FU_ALU + `NUM_FU_MULT + `NUM_FU_LD) + s];
        end

        // BR
        issued_br_pack_temp.decoded_vals = issued_br;
        issued_br_pack_temp.rs1_value = reg_data_1[`NUM_FU_ALU + `NUM_FU_MULT + `NUM_FU_LD + `SQ_SZ];
        issued_br_pack_temp.rs2_value = reg_data_2[`NUM_FU_ALU + `NUM_FU_MULT + `NUM_FU_LD + `SQ_SZ];
    end


endmodule