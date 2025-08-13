module rs_psel #(
    parameter DEPTH,
    parameter NUM_FU
)
(
    input logic             [DEPTH-1:0]                         inst_req, // which insts can be issued for this FU
    input logic             [NUM_FU-1:0]                        fu_req, // which FUs (for this op) are ready
    output logic            [$clog2(NUM_FU+1)-1:0]              num_issued,
    output logic            [NUM_FU-1:0][DEPTH-1:0]             fu_issued_insts, // issued insts w.r.t current FU
    output logic            [DEPTH-1:0]                         all_issued_insts  // issued insts w.r.t all RS entires

    `ifdef DEBUG
    ,   output logic        [NUM_FU-1:0][NUM_FU-1:0]            debug_fu_gnt_bus,
        output logic        [NUM_FU-1:0][DEPTH-1:0]             debug_inst_gnt_bus
    `endif
);

    logic [NUM_FU-1:0][DEPTH-1:0]  inst_gnt_bus; // one-hot
    logic [NUM_FU-1:0]             fu_gnt;
    logic [NUM_FU-1:0][NUM_FU-1:0] fu_gnt_bus;

    psel_gen #(
        .WIDTH(DEPTH),
        .REQS(NUM_FU))
    inst_psel (
        .req(inst_req),
        .gnt(),
        .gnt_bus(inst_gnt_bus),
        .empty()
    );

    psel_gen #(
        .WIDTH(NUM_FU),
        .REQS(NUM_FU))
    fu_psel (
        .req(fu_req),
        .gnt(fu_gnt),
        .gnt_bus(fu_gnt_bus),
        .empty()
    );

    `ifdef DEBUG
        assign debug_fu_gnt_bus = fu_gnt_bus;
        assign debug_inst_gnt_bus = inst_gnt_bus;
    `endif

    always_comb begin
        all_issued_insts = '0;
        fu_issued_insts = '0;
        for(int i=0;i<NUM_FU;i++) begin
            for(int j=0;j<NUM_FU;j++) begin
                fu_issued_insts[j] |= fu_gnt_bus[i][j] ? inst_gnt_bus[i] : 0;
            end
            if(fu_gnt_bus[i] && inst_gnt_bus[i]) begin
                all_issued_insts |= inst_gnt_bus[i];
            end
        end
    end

    always_comb begin
        num_issued = '0;

        for(int i=0;i<DEPTH;i++) begin
            if(all_issued_insts[i]) begin
                num_issued++;
            end
        end
    end

endmodule