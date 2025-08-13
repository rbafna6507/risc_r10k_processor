/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rs_test.sv                                          //
//                                                                     //
//  Description :  Testbench module for the N-way RS module            //
//                                                                     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

/*
    ASSUMPTIONS:
        - Infinite ROB
        - Infinite free list
*/

`include "sys_defs.svh"
`include "ISA.svh"

// comment out for less output
`define MONITOR 1

module RS_tb();

    parameter DEPTH = `RS_SZ;
    parameter N = `N;
    localparam LOG_DEPTH = $clog2(DEPTH);

    logic                                                                            clock;
    logic                                                                            reset;
    DECODED_PACKET              [N-1:0]                                              rs_in;
    FREE_LIST_PACKET            [N-1:0]                                              t_in;
    MAP_TABLE_PACKET            [N-1:0]                                              t1_in;
    MAP_TABLE_PACKET            [N-1:0]                                              t2_in;
    BR_MASK                                                                          b_id;

    CDB_PACKET                  [N-1:0]                                              cdb_in;
    logic                       [$clog2(N+1)-1:0]                                    num_accept;
    BR_MASK                                                                          rem_b_id;
    BR_TASK                                                                          br_task;

    logic                       [`NUM_FU_ALU-1:0]                                    fu_alu_busy;
    logic                       [`NUM_FU_MULT-1:0]                                   fu_mult_busy;
    logic                       [`NUM_FU_LD-1:0]                                     fu_ld_busy;
    logic                       [`NUM_FU_STORE-1:0]                                  fu_store_busy;
    logic                       [`NUM_FU_BR-1:0]                                     fu_br_busy;

    RS_PACKET                   [`NUM_FU_ALU-1:0]                                    issued_alu;
    RS_PACKET                   [`NUM_FU_MULT-1:0]                                   issued_mult;
    RS_PACKET                   [`NUM_FU_LD-1:0]                                     issued_ld;
    RS_PACKET                   [`NUM_FU_STORE-1:0]                                  issued_store;
    RS_PACKET                   [`NUM_FU_BR-1:0]                                     issued_br;

    logic                       [$clog2(N+1)-1:0]                                    open_entries;

`ifdef DEBUG
    RS_PACKET                   [DEPTH-1:0]                                          debug_entries;
    logic                       [DEPTH-1:0]                                          debug_open_spots;
    logic                       [DEPTH-1:0]                                          debug_other_sig;
    logic                       [N-1:0][DEPTH-1:0]                                   debug_dis_entries_bus;
    logic                       [$clog2(DEPTH+1)-1:0]                                debug_open_entries;
    logic                       [DEPTH-1:0]                                          debug_all_issued_insts;

    logic                       [`NUM_FU_ALU-1:0][DEPTH-1:0]                         debug_alu_issued_bus;
    logic                       [DEPTH-1:0]                                          debug_alu_req;
    logic                       [`NUM_FU_ALU-1:0][`NUM_FU_ALU-1:0]                   debug_alu_fu_gnt_bus;
    logic                       [`NUM_FU_ALU-1:0][DEPTH-1:0]                         debug_alu_inst_gnt_bus;

    logic                       [`NUM_FU_MULT-1:0][DEPTH-1:0]                        debug_mult_issued_bus;
    logic                       [DEPTH-1:0]                                          debug_mult_req;
    logic                       [`NUM_FU_MULT-1:0][`NUM_FU_MULT-1:0]                 debug_mult_fu_gnt_bus;
    logic                       [`NUM_FU_MULT-1:0][DEPTH-1:0]                        debug_mult_inst_gnt_bus;

    logic                       [`NUM_FU_BR-1:0][DEPTH-1:0]                          debug_br_issued_bus;
    logic                       [DEPTH-1:0]                                          debug_br_req;
    logic                       [`NUM_FU_BR-1:0][`NUM_FU_BR-1:0]                     debug_br_fu_gnt_bus;
    logic                       [`NUM_FU_BR-1:0][DEPTH-1:0]                          debug_br_inst_gnt_bus;

    logic                       [DEPTH-1:0]                                          debug_all_issued_alu;
    logic                       [DEPTH-1:0]                                          debug_all_issued_mult;
`endif

    RS_PACKET model_rs[$:(DEPTH)] = '{DEPTH{0}};
    RS_PACKET decoded_inst_buffer[$:(DEPTH*2)];

    RS_PACKET issued_alu_buffer[$:(`NUM_FU_ALU)] = '{`NUM_FU_ALU{0}};
    RS_PACKET issued_mult_buffer[$:(`NUM_FU_MULT)] = '{`NUM_FU_MULT{0}};
    RS_PACKET issued_ld_buffer[$:(`NUM_FU_LD)] = '{`NUM_FU_LD{0}};
    RS_PACKET issued_store_buffer[$:(`NUM_FU_STORE)] = '{`NUM_FU_STORE{0}};
    RS_PACKET issued_br_buffer[$:(`NUM_FU_BR)] = '{`NUM_FU_BR{0}};

    rs #(
        .DEPTH(DEPTH),
        .N(N))
    dut (
        .clock(clock),
        .reset(reset),
        .rs_in(rs_in),
        .t_in(t_in),
        .t1_in(t1_in),
        .t2_in(t2_in),
        .b_id(b_id),
        .cdb_in(cdb_in),
        .num_accept(num_accept),
        .rem_b_id(rem_b_id),
        .br_task(br_task),
        .fu_alu_busy(fu_alu_busy),
        .fu_mult_busy(fu_mult_busy),
        .fu_ld_busy(fu_ld_busy),
        .fu_store_busy(fu_store_busy),
        .fu_br_busy(fu_br_busy),
        .issued_alu(issued_alu),
        .issued_mult(issued_mult),
        .issued_ld(issued_ld),
        .issued_store(issued_store),
        .issued_br(issued_br),
        .open_entries(open_entries)
        `ifdef DEBUG
        ,   .debug_entries(debug_entries),
            .debug_open_spots(debug_open_spots),
            .debug_other_sig(debug_other_sig),
            .debug_dis_entries_bus(debug_dis_entries_bus),
            .debug_open_entries(debug_open_entries),
            .debug_all_issued_insts(debug_all_issued_insts),

            .debug_alu_issued_bus(debug_alu_issued_bus),
            .debug_alu_req(debug_alu_req),
            .debug_alu_fu_gnt_bus(debug_alu_fu_gnt_bus),
            .debug_alu_inst_gnt_bus(debug_alu_inst_gnt_bus),

            .debug_mult_issued_bus(debug_mult_issued_bus),
            .debug_mult_req(debug_mult_req),
            .debug_mult_fu_gnt_bus(debug_mult_fu_gnt_bus),
            .debug_mult_inst_gnt_bus(debug_mult_inst_gnt_bus),

            .debug_br_issued_bus(debug_br_issued_bus),
            .debug_br_req(debug_br_req),
            .debug_br_fu_gnt_bus(debug_br_fu_gnt_bus),
            .debug_br_inst_gnt_bus(debug_br_inst_gnt_bus)
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
        clear_signals();
        @(negedge clock);
        reset = 0;


        // ------------------------------ Test 1 ------------------------------ //
        $display("\nTest 1: Basic Dispatch/Issue N ALU Instructions");
        generate_ops(N, ALU_INST);
        @(negedge clock); // dispatch
        fu_alu_busy = 0;
        @(negedge clock); // issue

        reset = 1;
        clear_signals();
        @(negedge clock); // verify issued
        $display("TEST 1: PASSED");

        // ------------------------------ Test 2 ------------------------------ //
        $display("\nTest 2: Basic Dispatch/Issue N MULT Instructions");
        reset = 0;
        generate_ops(N, MULT_INST);
        @(negedge clock); // dispatch
        fu_mult_busy = 0;
        @(negedge clock); // issue

        reset = 1;
        clear_signals();
        @(negedge clock); // verify issued
        $display("TEST 2: PASSED");

        // ------------------------------ Test 3 ------------------------------ //
        $display("\nTest 3: Basic Dispatch/Stall N ALU Instructions");
        reset = 0;
        generate_ops(N, ALU_INST);
        @(negedge clock); // dispatch
        fu_alu_busy = '1;
        @(negedge clock); // issue
        @(negedge clock);

        reset = 1;
        clear_signals();
        @(negedge clock);
        $display("TEST 3: PASSED");

        // ------------------------------ Test 4 ------------------------------ //
        $display("\nTest 4: Basic Dispatch/Stall N MULT Instructions");
        reset = 0;
        generate_ops(N, MULT_INST);
        @(negedge clock); // dispatch
        fu_mult_busy = '1;
        @(negedge clock); // issue
        @(negedge clock)

        reset = 1;
        clear_signals();
        @(negedge clock); // verify issued
        $display("TEST 4: PASSED");

        // ------------------------------ Test 5 ------------------------------ //
        $display("\nTest 5: 2N Dispatch/Stall ALU Instructions");
        reset = 0;
        generate_ops(2*N, ALU_INST);
        @(negedge clock); // dispatch N
        fu_alu_busy = '1;
        @(negedge clock); // issue 0, dispatch N
        @(negedge clock); // verify 0, 6 entries
        @(negedge clock); 

        reset = 1;
        clear_signals();
        @(negedge clock); // verify issued
        $display("TEST 5: PASSED");

        // ------------------------------ Test 6 ------------------------------ //
        $display("\nTest 6: Fill RS");
        reset = 0;
        generate_ops(DEPTH+1, ALU_INST);
        @(negedge clock); // dispatch N
        fu_alu_busy = '1;
        for(int i=0;i<(DEPTH/N)+1;i++) begin
            @(negedge clock); // issue 0, dispatch N
        end
        @(negedge clock); 

        reset = 1;
        clear_signals();
        @(negedge clock); // verify issued
        $display("TEST 6: PASSED");

        // ------------------------------ Test 7 ------------------------------ //
        $display("\nTest 7: Unready Registers");
        reset = 0;
        generate_ops(N, ALU_INST, 0);
        @(negedge clock); // dispatch N
        fu_alu_busy = '0;
        @(negedge clock); // issue 0, dispatch N
        @(negedge clock); 

        reset = 1;
        clear_signals();
        @(negedge clock); // verify issued
        $display("TEST 7: PASSED");

        // ------------------------------ Test 8 ------------------------------ //
        $display("\nTest 8: Unready Registers & CDB");
        reset = 0;
        generate_ops(N, ALU_INST, 0);
        @(negedge clock); // dispatch N
        fu_alu_busy = '0;
        @(negedge clock); // issue 0, dispatch N

        reset = 1;
        clear_signals();
        @(negedge clock); // verify issued
        $display("TEST 8: PASSED");

        // ------------------------------ Test 9 ------------------------------ //
        $display("\nTest 9: Dispatch BR and Clear dependent insts b_ids");
        reset = 0;
        generate_ops(1, BR_INST, 1, 4'b0001);
        generate_ops(N-1, ALU_INST, 1, 4'b0001);
        @(negedge clock); // dispatch 1
        fu_alu_busy = '1;
        @(negedge clock); // issue 1 BR, dispatch N-1
        fu_alu_busy = '0;
        @(negedge clock);
        rem_b_id = 4'b0001;
        br_task = CLEAR;
        @(negedge clock);

        reset = 1;
        clear_signals();
        @(negedge clock); // verify issued
        $display("TEST 9: PASSED");

        $display("@@@ PASSED ALL TESTS @@@");
        $finish;
    end


    int cycle_number = 0;
    // Correctness Verification
    always @(negedge clock) begin
        #(`CLOCK_PERIOD * 0.2);
        `ifdef MONITOR
            $display("------------------------------------------------------------");
            $display("@@@ Cycle Number: %0d @@@", cycle_number);
            $display("   Time: %0t", $time);
            $display("   Reset: %0d\n", reset);

            print_issue_signal();
        `endif 
        model_rs_check_comb(); // verify open_entries + issued packets
        cycle_number++;
    end

    always @(posedge clock) begin
        #(`CLOCK_PERIOD * 0.2);
        `ifdef MONITOR
            rs_print();
        `endif
        model_rs_check_seq(); // verify entries + open_spots + num_open_entries

        model_rs_update();
        dispatch();
    end

    // Helper function to clear inputs to RS
    function void clear_signals();
        num_accept = 0;
        rs_in = 0;
        t_in = 0;
        t1_in = 0;
        t2_in = 0;
        b_id = 0;
        cdb_in = 0;
        rem_b_id = 0;
        br_task = 0;
        fu_alu_busy = 0;
        fu_mult_busy = 0;
        fu_ld_busy = 0;
        fu_store_busy = 0;
        fu_br_busy = 0;

        decoded_inst_buffer = {};
    endfunction

    function void model_rs_insert(RS_PACKET in, int lsb);
        if(lsb) begin
            for(int i=0;i<DEPTH;i++) begin
                if(!model_rs[i].decoded_vals.valid) begin
                    `ifdef MONITOR
                        $display("dispatching [(%b) (%02d)] to %d", in.decoded_vals.valid, in.t.reg_idx, i);
                    `endif
                    model_rs[i] = in;
                    return;
                end
            end
        end else begin
            for(int i=DEPTH-1;i>=0;i--) begin
                if(!model_rs[i].decoded_vals.valid) begin
                    `ifdef MONITOR
                        $display("dispatching [(%b) (%02d)] to %d", in.decoded_vals.valid, in.t.reg_idx, i);
                    `endif
                    model_rs[i] = in;
                    return;
                end
            end
        end
    endfunction

    function RS_PACKET model_rs_pop(int lsb, FU_TYPE fu_type);
        RS_PACKET res;
        res = 0;
        if(lsb) begin
            for(int i=0;i<DEPTH;i++) begin
                if(model_rs[i].decoded_vals.fu_type == fu_type && model_rs[i].decoded_vals.valid && model_rs[i].t1.ready && model_rs[i].t2.ready) begin
                    res = model_rs[i];
                    model_rs_delete(i);
                    break;
                end
            end
        end else begin
            for(int i=DEPTH-1;i>=0;i--) begin
                if(model_rs[i].decoded_vals.fu_type == fu_type && model_rs[i].decoded_vals.valid && model_rs[i].t1.ready && model_rs[i].t2.ready) begin
                    res = model_rs[i];
                    model_rs_delete(i);
                    break;
                end
            end
        end
        return res;
    endfunction

    function void model_rs_delete(int idx);
        model_rs[idx] = '0;
    endfunction

    function int model_rs_count();
        int count;
        count = 0;
        for(int i=0;i<DEPTH;i++) begin
            if(model_rs[i].decoded_vals.valid) begin
                count++;
            end
        end
        return count;
    endfunction

    function int model_rs_open_entries();
        return DEPTH-model_rs_count();
    endfunction

    function void model_rs_set_ready(PHYS_REG_IDX idx);
        for(int i=0;i<DEPTH;i++) begin
            if(model_rs[i].decoded_vals.valid) begin
                if(model_rs[i].t1.reg_idx == idx) begin
                    model_rs[i].t1.ready = '1;
                end
                if(model_rs[i].t2.reg_idx == idx) begin
                    model_rs[i].t2.ready = '1;
                end
            end
        end
    endfunction

    function int rs_equal(RS_PACKET gt, RS_PACKET val);
        if((rem_b_id & gt.b_mask) != 0) begin
            if(br_task == CLEAR) begin
                gt.b_mask = 0;
            end
            if(br_task == SQUASH) begin
                gt = 0;
            end
        end
        return gt.decoded_vals.valid == val.decoded_vals.valid && gt.t == val.t && gt.t1 == val.t1 && gt.t2 == val.t2 && gt.b_mask == val.b_mask && gt.decoded_vals.fu_type == val.decoded_vals.fu_type;
    endfunction

    function void model_rs_check_comb();
        if(reset) begin
            return;
        end
        for(int i=0;i<`NUM_FU_ALU;i++) begin
            if(!rs_equal(issued_alu_buffer[i], issued_alu[i])) begin
                $display("@@@ FAILED");
                $display("ALU ISSUE PACKET MISMATCH AT i=%0d", i);
                $display("  expected v=%0b b_id=%4b b_mask=%4b", issued_alu_buffer[i].decoded_vals.valid, issued_alu_buffer[i].b_id, issued_alu_buffer[i].b_mask);
                $display("   but got v=%0b b_id=%4b b_mask=%4b", issued_alu[i].decoded_vals.valid, issued_alu[i].b_id, issued_alu[i].b_mask);
                $finish;
            end
        end
        for(int i=0;i<`NUM_FU_MULT;i++) begin
            if(!rs_equal(issued_mult_buffer[i], issued_mult[i])) begin
                $display("@@@ FAILED");
                $display("MULT ISSUE PACKET MISMATCH AT i=%d v=%0b b=%4b", i, issued_mult_buffer[i].decoded_vals.valid, issued_mult[i].b_mask);
                $finish;
            end
        end
        for(int i=0;i<`NUM_FU_BR;i++) begin
            if(!rs_equal(issued_br_buffer[i], issued_br[i])) begin
                $display("@@@ FAILED");
                $display("BR ISSUE PACKET MISMATCH AT i=%d v=%0b b=%4b", i, issued_br_buffer[i].decoded_vals.valid, issued_br[i].b_mask);
                $finish;
            end
        end 
    endfunction

    function void model_rs_check_seq();
        int rs_entries, capped_entries;

        if(reset) begin
            return;
        end

        rs_entries = model_rs_open_entries();
        capped_entries = rs_entries > N ? N : rs_entries;

        if(open_entries != capped_entries) begin
            $display("@@@ FAILED");
            $display("OPEN ENTRIES (CAPPED) MISMATCH! %02d %02d %02d", open_entries, capped_entries, rs_entries);
            $finish;
        end

        `ifdef DEBUG
            if(debug_open_entries != rs_entries) begin
                $display("@@@ FAILED");
                $display("OPEN ENTRIES MISMATCH! %02d %02d", debug_open_entries, rs_entries);
                $finish;
            end
        `endif 
    endfunction

    function void model_rs_update();
        RS_PACKET issued_packet;
        int fu_issued_idx;

        int fu_alu_ready, num_alu_ready, num_alu_issued;
        int fu_mult_ready, num_mult_ready, num_mult_issued;
        int fu_br_ready, num_br_ready, num_br_issued;

        if(reset) begin
            model_rs = '{DEPTH{0}};
            issued_alu_buffer = '{`NUM_FU_ALU{0}};
            issued_mult_buffer = '{`NUM_FU_MULT{0}};
            issued_ld_buffer = '{`NUM_FU_LD{0}};
            issued_store_buffer = '{`NUM_FU_STORE{0}};
            issued_br_buffer = '{`NUM_FU_BR{0}};
            return;
        end

        for(int i=0;i<N;i++) begin
            if(cdb_in[i].valid) begin
                model_rs_set_ready(cdb_in[i].reg_idx);
            end
        end

        for(int i=0;i<DEPTH;i++) begin
            if((model_rs[i].b_mask & rem_b_id) != 0) begin
                if(br_task == CLEAR) begin
                    model_rs[i].b_mask ^= rem_b_id;
                    model_rs[i].b_id = 0;
                end else if (br_task == SQUASH) begin
                    model_rs_delete(i);
                end
            end
        end

        num_alu_issued = 0;
        fu_alu_ready = ~fu_alu_busy;
        num_alu_ready = `NUM_FU_ALU - $countones(fu_alu_busy);
        issued_alu_buffer = '{`NUM_FU_ALU{0}};

        num_mult_issued = 0;
        fu_mult_ready = ~fu_mult_busy;
        num_mult_ready = `NUM_FU_MULT - $countones(fu_mult_busy);
        issued_mult_buffer = '{`NUM_FU_MULT{0}};

        num_br_issued = 0;
        fu_br_ready = ~fu_br_busy;
        num_br_ready = `NUM_FU_BR - $countones(fu_br_busy);
        issued_br_buffer = '{`NUM_FU_BR{0}};

        while(num_alu_ready > 0) begin
            issued_packet = model_rs_pop(num_alu_issued % 2, ALU_INST);

            if(!issued_packet.decoded_vals.valid) begin
                break;
            end

            fu_issued_idx = num_alu_issued % 2 ? lsb(fu_alu_ready, `NUM_FU_ALU) : msb(fu_alu_ready, `NUM_FU_ALU);

            fu_alu_ready[fu_issued_idx] = 0;
            issued_alu_buffer[fu_issued_idx] = issued_packet;

            num_alu_ready--;
            num_alu_issued++;
        end

        while(num_mult_ready > 0) begin
            issued_packet = model_rs_pop(num_mult_issued % 2, MULT_INST);

            if(!issued_packet.decoded_vals.valid) begin
                break;
            end

            fu_issued_idx = num_mult_issued % 2 ? lsb(fu_mult_ready, `NUM_FU_MULT) : msb(fu_mult_ready, `NUM_FU_MULT);

            fu_mult_ready[fu_issued_idx] = 0;
            issued_mult_buffer[fu_issued_idx] = issued_packet;

            num_mult_ready--;
            num_mult_issued++;
        end

        while(num_br_ready > 0) begin
            issued_packet = model_rs_pop(num_br_issued % 2, BR_INST);

            if(!issued_packet.decoded_vals.valid) begin
                break;
            end

            fu_issued_idx = num_br_issued % 2 ? lsb(fu_br_ready, `NUM_FU_BR) : msb(fu_br_ready, `NUM_FU_BR);

            fu_br_ready[fu_issued_idx] = 0;
            issued_br_buffer[fu_issued_idx] = issued_packet;

            num_br_ready--;
            num_br_issued++;
        end
    endfunction

    function void generate_ops(int num, FU_TYPE fu_type, int is_ready = 1, logic [`BRANCH_PRED_SZ-1:0] b_id = 0);
        RS_PACKET inst = '0;
        for(int i=0;i<num;i++) begin
            inst.t  = '{reg_idx: i, valid: 1};
            inst.t1 = '{reg_idx: 0, valid: 1, ready: is_ready};
            inst.t2 = '{reg_idx: 0, valid: 1, ready: is_ready};
            inst.b_id = b_id;

            inst.decoded_vals = 0;
            inst.decoded_vals.valid = 1;
            inst.decoded_vals.fu_type = fu_type;

            decoded_inst_buffer.push_back(inst);
        end
    endfunction

    function void dispatch();
        RS_PACKET packet;
        int num_insts_avail, num_rs_avail;

        rs_in = 0;
        t_in = 0;
        t1_in = 0;
        t2_in = 0;
        b_id = 0;
        num_rs_avail = model_rs_open_entries();
        num_insts_avail = decoded_inst_buffer.size();

        num_accept = N < num_insts_avail 
                        ? N < num_insts_avail ? N : num_insts_avail
                        : num_insts_avail < num_rs_avail ?  num_insts_avail : num_rs_avail;

        for(int i=0;i<num_accept;i++) begin
            packet = decoded_inst_buffer.pop_front();
            rs_in[i] = packet.decoded_vals;
            t_in[i] = packet.t;
            t1_in[i] = packet.t1;
            t2_in[i] = packet.t2;
            b_id[i] = packet.b_mask;

            model_rs_insert(packet, i % 2);

            if(packet.decoded_vals.fu_type == BR_INST) begin
                num_accept = i+1;
                return;
            end
        end
    endfunction

    function print_fu_issued(int num_fu, FU_TYPE fu_type);
        $write("\nModel RS Issued Signal [%01d]", fu_type);

        case(fu_type)
            ALU_INST: begin
                $write(" [%b]", fu_alu_busy);
            end
            MULT_INST: begin
                $write(" [%b]", fu_mult_busy);
            end
            LD_INST: begin
                $write(" [%b]", fu_ld_busy);
            end
            STORE_INST: begin
                $write(" [%b]", fu_store_busy);
            end
            BR_INST: begin
                $write(" [%b]\t", fu_br_busy);
            end
        endcase

        $write("\t\t\t\t\t\t\t\t\t\t");
        $write("RS Issued Signal [%01d]\n", fu_type);
        $write("#\t| valid |dest_idx|\tt1\t|\tt2\t|  b_mask   |fu_type|");
        $write("\t\t\t\t");
        $write("#\t| valid |dest_idx|\tt1\t|\tt2\t|  b_mask   |fu_type|");
        $write("\n");


        for(int i=num_fu-1;i>=0;i--) begin
            case(fu_type)
                ALU_INST: begin
                    $write("%02d\t|\t%01d\t|\t%02d\t |\t%02d\t|\t%02d\t|\t%04b\t|\t%01d\t|", i, issued_alu_buffer[i].decoded_vals.valid, issued_alu_buffer[i].t.reg_idx, issued_alu_buffer[i].t1.reg_idx, issued_alu_buffer[i].t2.reg_idx, issued_alu_buffer[i].b_mask, issued_alu_buffer[i].decoded_vals.fu_type);
                    $write("\t\t\t\t");
                    $write("\t\t\t\t");
                    $write("%02d\t|\t%01d\t|\t%02d\t |\t%02d\t|\t%02d\t|\t%04b\t|\t%01d\t|", i, issued_alu[i].decoded_vals.valid, issued_alu[i].t.reg_idx, issued_alu[i].t1.reg_idx, issued_alu[i].t2.reg_idx, issued_alu[i].b_mask, issued_alu[i].decoded_vals.fu_type);
                end
                MULT_INST: begin
                    $write("%02d\t|\t%01d\t|\t%02d\t |\t%02d\t|\t%02d\t|\t%04b\t|\t%01d\t|", i, issued_mult_buffer[i].decoded_vals.valid, issued_mult_buffer[i].t.reg_idx, issued_mult_buffer[i].t1.reg_idx, issued_mult_buffer[i].t2.reg_idx, issued_mult_buffer[i].b_mask, issued_mult_buffer[i].decoded_vals.fu_type);
                    $write("\t\t\t\t");
                    $write("\t\t\t\t");
                    $write("%02d\t|\t%01d\t|\t%02d\t |\t%02d\t|\t%02d\t|\t%04b\t|\t%01d\t|", i, issued_mult[i].decoded_vals.valid, issued_mult[i].t.reg_idx, issued_mult[i].t1.reg_idx, issued_mult[i].t2.reg_idx, issued_mult[i].b_mask, issued_mult[i].decoded_vals.fu_type);
                end
                BR_INST: begin
                    $write("%02d\t|\t%01d\t|\t%02d\t |\t%02d\t|\t%02d\t|\t%04b\t|\t%01d\t|", i, issued_br_buffer[i].decoded_vals.valid, issued_br_buffer[i].t.reg_idx, issued_br_buffer[i].t1.reg_idx, issued_br_buffer[i].t2.reg_idx, issued_br_buffer[i].b_mask, issued_br_buffer[i].decoded_vals.fu_type);
                    $write("\t\t\t\t");
                    $write("\t\t\t\t");
                    $write("%02d\t|\t%01d\t|\t%02d\t |\t%02d\t|\t%02d\t|\t%04b\t|\t%01d\t|", i, issued_br[i].decoded_vals.valid, issued_br[i].t.reg_idx, issued_br[i].t1.reg_idx, issued_br[i].t2.reg_idx, issued_br[i].b_mask, issued_br[i].decoded_vals.fu_type);

                end
            endcase
            $write("\n");
        end
        $write("\n");
    endfunction

    function print_issue_signal();
        print_fu_issued(`NUM_FU_ALU, ALU_INST);
        print_fu_issued(`NUM_FU_MULT, MULT_INST);
        print_fu_issued(`NUM_FU_BR, BR_INST);

        `ifdef DEBUG
            $write("\n");
            $write("ALU Issued Bus [%b]", debug_alu_req);
            $write("\t\t\t\t");
            $write("MULT Issued Bus [%b]", debug_mult_req);
            $write("\t\t\t\t");
            $write("BR Issued Bus [%b]", debug_br_req);
            $write("\n");
            for(int i=`NUM_FU_ALU-1;i>=0;i--) begin
                $write("%02d %b %b %b", i, debug_alu_issued_bus[i], debug_alu_fu_gnt_bus[i], debug_alu_inst_gnt_bus[i]);
                $write("\t\t");
                $write("%02d %b %b %b", i, debug_mult_issued_bus[i], debug_mult_fu_gnt_bus[i], debug_mult_inst_gnt_bus[i]);
                $write("\t\t");
                $write("%02d %b %b %b", i, debug_br_issued_bus[i], debug_br_fu_gnt_bus[i], debug_br_inst_gnt_bus[i]);
                $write("\n");
            end
            $write("\n");
        `endif
    endfunction

    function void rs_print();
        $write("Model RS Entries (%02d)", model_rs_open_entries());
        `ifdef DEBUG
            $write("\t\t\t\t\t\t\t\t\t\t\t\t\tRS Entries (open_entries: %02d [%02d]) (open_spots: %b) (all_issued: %b)  (other_sig: %b)", open_entries, debug_open_entries, debug_open_spots, debug_all_issued_insts, debug_other_sig);
        `endif
        $write("\n");

        $write("#\t| valid |dest_idx|\tt1\t\t|\tt2\t\t|  b_mask   |fu_type|");
        `ifdef DEBUG
            $write("\t\t");
            $write("#\t| valid |dest_idx|\tt1\t\t|\tt2\t\t|  b_mask   |fu_type|");
        `endif
        $write("\n");

        for(int i=DEPTH-1;i>=0;i--) begin
            //          idx       valid    dest    t1        t2     bmask      op
            $write("%02d\t|\t%01d\t|\t%02d\t |\t%02d[%b]\t|\t%02d[%b]\t|\t%04b\t|\t%01d\t|", i, model_rs[i].decoded_vals.valid, model_rs[i].t.reg_idx, model_rs[i].t1.reg_idx, model_rs[i].t1.valid, model_rs[i].t2.reg_idx, model_rs[i].t2.valid, model_rs[i].b_mask, model_rs[i].decoded_vals.fu_type);
            `ifdef DEBUG
                $write("\t\t\t\t\t\t");
                $write("%02d\t|\t%01d\t|\t%02d\t |\t%02d[%b]\t|\t%02d[%b]\t|\t%04b\t|\t%01d\t|", i, debug_entries[i].decoded_vals.valid, debug_entries[i].t.reg_idx, debug_entries[i].t1.reg_idx, debug_entries[i].t1.valid, debug_entries[i].t2.reg_idx, debug_entries[i].t2.valid, debug_entries[i].b_mask, debug_entries[i].decoded_vals.fu_type);
            `endif
            $write("\n");
        end

        `ifdef DEBUG
            $display("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tDispatching Entries Bus");
            for(int i=0;i<N;i++) begin
                $write("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t");
                $write("[%b (%02d)] [(%b) (%02d)]", debug_dis_entries_bus[i], msb(debug_dis_entries_bus[i], DEPTH), rs_in[i].valid, t_in[i].reg_idx);
                $write("\n");
            end
            $display();
        `endif
    endfunction

    function int msb(int value, int width);
        for(int i=width-1;i>=0;i--) begin
            if(value[i]) begin
                return i;
            end
        end
    endfunction

    function int lsb(int value, int width);
        for(int i=0;i<width;i++) begin
            if(value[i]) begin
                return i;
            end
        end
    endfunction

    

endmodule
