/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  cdb_test.sv                                         //
//                                                                     //
//  Description :  Testbench module for the CDB                        //
//                                                                     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"
`include "ISA.svh"

module cdb_tb ();
    // THINGS ARE ONLY CHEKCKED WHEN N = 4
    parameter N = `N;
    parameter NUM_FU = `NUM_FU_ALU + `NUM_FU_MULT + `NUM_FU_LD + `NUM_FU_STORE;

    logic                       clock;
    logic                       reset;

    logic         [NUM_FU-1:0]  fu_done;
    FU_PACKET     [NUM_FU-1:0]  wr_data;

    CDB_PACKET    [N-1:0]       entries;
    logic         [NUM_FU-1:0]  stall_sig;

    `ifdef DEBUG
        logic [NUM_FU-1:0] debug_cdb_gnt;
        logic [N-1:0][NUM_FU-1:0] debug_cdb_gnt_bus;
    `endif 

    // local params
    CDB_PACKET [N-1:0] model_entries;
    logic [NUM_FU-1:0] fu_input;
    FU_PACKET [NUM_FU-1:0] wr_data_in;
    DECODED_PACKET disp_pack;
    RS_PACKET rs_pack;

    cdb #(
        .N(N),
        .NUM_FU(NUM_FU)
    )
    dut (
        .clock(clock),
        .reset(reset),

        .fu_done(fu_done),
        .wr_data(wr_data),

        .entries(entries),
        .stall_sig(stall_sig)

        `ifdef DEBUG
        ,   .debug_cdb_gnt(debug_cdb_gnt),
            .debug_cdb_gnt_bus(debug_cdb_gnt_bus)
        `endif 
    );

    always begin 
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    initial begin
        $display("\nStart Testbench");

        clock = 0;
        reset = 1;
        model_entries = '0;
        setup_fu_done();
        fu_done = fu_input;
        wr_data = wr_data_in;
        print_all();
        model_entries[0].reg_val = 0;
        model_entries[1].reg_val = 0;
        model_entries[2].reg_val = 0;
        model_entries[3].reg_val = 0;

        // ------------------------------ Test 1 ------------------------------ //
        $display("\nTest 1: Broadcast One FU Value\n");

        update_FU_done(1, 1);
        model_entries[0].reg_val = wr_data[1].result;
        
        @(negedge clock);
        update_FU_done(1, 0);

        $display("DONE TEST 1, CHECK MANUALLY");

        // ------------------------------ Test 2 ------------------------------ //
        $display("\nTest 2: Multiple FUs Broadcast\n");

        update_FU_done(5, 1);
        update_FU_done(11, 1);

        model_entries[0].reg_val = wr_data[11].result;
        model_entries[1].reg_val = wr_data[5].result;

        @(negedge clock);

        update_FU_done(5, 0);
        update_FU_done(11, 0);

        $display("DONE TEST 2, CHECK MANUALLY");

        // ------------------------------ Test 3 ------------------------------ //
        $display("\nTest 3: FU Request More than N\n");

        update_FU_done(5, 1);
        update_FU_done(11, 1);
        update_FU_done(2, 1);
        update_FU_done(9, 1); // this one shouldn't get picked when N = 4
        update_FU_done(10, 1);
        model_entries[0].reg_val = wr_data[11].result;
        model_entries[1].reg_val = wr_data[2].result;
        model_entries[2].reg_val = wr_data[10].result;
        model_entries[3].reg_val = wr_data[5].result;

        @(negedge clock);

        update_FU_done(5, 0);
        update_FU_done(11, 0);
        update_FU_done(2, 0);
        update_FU_done(9, 0);
        update_FU_done(10, 0);

        $display("DONE TEST 3, CHECK MANUALLY \n");

        $finish;

    end

    int cycle_number = 0;
    always @(posedge clock) begin
        #(`CLOCK_PERIOD * 0.2);
        print_all();
        if (N == 4) begin
            check_entries();
        end
        $display("\n@@@ FINISHED CYCLE NUMBER: %0d @@@ \n", cycle_number);
        cycle_number++;
    end

    function void setup_fu_done();
    
        int regidx = 1;
        int pregidx = 14;
        int regvalue = 0;

        disp_pack = '{
            inst: '0,
            PC: '0,
            NPC: 1,
            opa_select: OPA_IS_RS1,
            opb_select: OPB_IS_RS2,
            dest_reg_idx: regidx,
            alu_func: ALU_ADD,
            mult: '0,
            rd_mem: '0,
            wr_mem: '0,
            cond_branch: '0,
            uncond_branch: '0,
            halt: '0,
            illegal: '0,
            csr_op: '0,
            fu_type: '0,
            pred_taken: '0,
            valid: 1,
            reg1: 1,
            reg2: 1
        };

        rs_pack = '{
            decoded_vals: disp_pack,
            t: '{reg_idx: pregidx, valid: 1},
            t1: '{reg_idx: 2, ready: 0, valid: 1},
            t2: '{reg_idx: 2, ready: 0, valid: 1},
            b_mask: '0,
            b_id: '0
        };

        for (int i = 0; i < NUM_FU; i++) begin
            disp_pack.dest_reg_idx = regidx;
            disp_pack.valid = 0;
            rs_pack.decoded_vals = disp_pack;
            rs_pack.t.reg_idx = pregidx;
            wr_data_in[i].decoded_vals = rs_pack;
            wr_data_in[i].result = regvalue;
            
            $display("regvalue: %d", regvalue);
            $display("wr_data result: %d\n", wr_data[i].result);
            // wr_data_in[i] = '{decoded_vals.decoded_vals.dest_reg_idx: regidx, decoded_vals.t.reg_idx: pregidx, result: regvalue, decoded_vals.decoded_vals.valid: 0};
            regidx++;
            pregidx++;
            regvalue += 5;
        end

        for (int i = 0; i < NUM_FU; i++) begin
            fu_input[i] = 0;
        end
    endfunction

    function print_wr_data();
        $display("\Write Data:\n");
        $write("index: ");
        for (int i = 0; i < NUM_FU; i++) begin
            $write("%0d    ", i);
        end
        $write("\nreg:   ");
        for (int i = 0; i < NUM_FU; i++) begin
            $write("%0d    ", wr_data[i].decoded_vals.decoded_vals.dest_reg_idx);
        end
        $write("\npreg:  ");
        for (int i = 0; i < NUM_FU; i++) begin
            $write("%0d   ", wr_data[i].decoded_vals.t.reg_idx);
        end
        $write("\nval:   ");
        for (int i = 0; i < NUM_FU; i++) begin
            if (i < 2) begin
                $write("%0d    ", wr_data[i].result);
            end
            else begin 
                $write("%0d   ", wr_data[i].result);
            end
        end
        $write("\n");
    endfunction

    function print_fu_done();
        $display("\nFU Done:\n");
        $write("index: ");
        for (int i = 0; i < NUM_FU; i++) begin
            $write("%0d    ", i);
        end
        $write("\nsignal:   ");
        for (int i = 0; i < NUM_FU; i++) begin
            $write("%0d    ", fu_done[i]);
        end
        $write("\n");
    endfunction

    function print_cdb_grant();
        $display("\nCDB Grant:\n");
        $write("index: ");
        for (int i = 0; i < NUM_FU; i++) begin
            $write("%0d    ", i);
        end
        $write("\nsignal:   ");
        for (int i = 0; i < NUM_FU; i++) begin
            $write("%0d    ", debug_cdb_gnt[i]);
        end
        $write("\n");
    endfunction

    function print_stall_sig();
        $display("\nStall Signal:\n");
        $write("index: ");
        for (int i = 0; i < NUM_FU; i++) begin
            $write("%0d    ", i);
        end
        $write("\nsignal:   ");
        for (int i = 0; i < NUM_FU; i++) begin
            $write("%0d    ", stall_sig[i]);
        end
        $write("\n");
    endfunction

    function print_all();
        print_wr_data();
        print_fu_done();
        print_cdb_grant();
        print_stall_sig();
        print_model_entries();
        print_entries();
    endfunction

    function print_entries();
        $display("\nEntries:\n");
        $write("index:   ");
        for (int i = 0; i < N; i++) begin
            $write("%0d    ", i);
        end
        $write("\nentries:   ");
        for (int i = 0; i < N; i++) begin
            $write("%0d    ", entries[i].reg_val);
        end
        $write("\n");
    endfunction

    function print_model_entries();
        $display("\nModel Entries:\n");
        $write("index:   ");
        for (int i = 0; i < N; i++) begin
            $write("%0d    ", i);
        end
        $write("\nentries:   ");
        for (int i = 0; i < N; i++) begin
            $write("%0d    ", model_entries[i].reg_val);
        end
        $write("\n");
    endfunction

    function check_entries();
        //$display("\nchecking entries\n");
        for (int i = 0; i < N; i++) begin
            if (model_entries[i].reg_val != entries[i].reg_val) begin
                $error("@@@ FAILED @@@");
                $error("Check entry error: expected %0d, but got %0d", model_entries[i].reg_val, entries[i].reg_val);
                $finish;
            end
        end
    endfunction

    function update_FU_done(int i, int signal);
        fu_done[i] = signal;
    endfunction

endmodule
   