/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rob_queue_test.sv                                   //
//                                                                     //
//  Description :  Testbench module for the N-way ROB module           //
//                                                                     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"
`include "ISA.svh"

module ROB_queue_tb();

  // Parameters
  parameter DEPTH = 8;
  parameter WIDTH = 32;
  parameter N = 1;
  localparam LOG_DEPTH = $clog2(DEPTH);

  // Signals
  logic                     clock;
  logic                     reset;
  ROB_ENTRY_PACKET          [N-1:0] wr_data;
  PHYS_REG_IDX              [N-1:0] complete_t;
  logic                     [$clog2(N+1)-1:0] num_accept;
  ROB_ENTRY_PACKET          [N-1:0] retiring_data;
  logic                     [$clog2(DEPTH+1)-1:0] open_entries;
  logic                     [$clog2(N+1)-1:0] num_retired;

  // Debugging
  `ifdef DEBUG
    ROB_ENTRY_PACKET [DEPTH-1:0] entry_data;
    logic [LOG_DEPTH-1:0] debug_head;
    logic [LOG_DEPTH-1:0] debug_tail;
  `endif

  // Instantiate the ROB
  ROB #(
    .DEPTH(DEPTH),
    .N(N)
  ) rob_inst (

    // inputs
    .clock(clock),
    .reset(reset),
    .wr_data(wr_data),
    .complete_t(complete_t),
    .num_accept(num_accept),

    // outputs
    .retiring_data(retiring_data),
    .open_entries(open_entries),
    .num_retired(num_retired)

    // debugging
    `ifdef DEBUG
    , .debug_entries(entry_data),
    .debug_head(debug_head),
    .debug_tail(debug_tail)
    `endif
  );

  // Queue Instantiations
  ROB_ENTRY_PACKET rob_model [$:(DEPTH - 1)];
  ROB_ENTRY_PACKET inst_buff [$:(DEPTH*2)-1];

  // Generate System Clock
  always begin
    #(`CLOCK_PERIOD/2.0);
      clock = ~clock;
  end

  always @(posedge clock) begin
    #(`CLOCK_PERIOD * 0.2);
  end
  
  initial begin
    clock = 0;
    reset = 1;
    reset();
    @(posedge clock)

    // initially reset the rob
    #5 reset = 0;

    //*** TESTBENCHES ***//

    // TESTBENCH 1

    // add N instructions, when the ROB is empty, complete ALL => exepct AL retired
    fetch_entries();
    add_entries(N);
    @(negedge clock);

    check_open_entries();
    check_retired_entries();

    set_complete(N);


    reset();
    @(posedge clock)

    #5 reset = 0;

  /*
    // add N instructions, when the ROB is full
    for (int i = 0; i < (DEPTH / N); i++) begin
      add_entries(N);
    end

    reset();
    @(posedge clock)

    #5 reset = 0;

    // add N instructions, when the ROB has stuff but not full

    reset();
    @(posedge clock)

    #5 reset = 0;
    // add less than N instructions, when the ROB is empty
    
    reset();
    @(posedge clock)

    #5 reset = 0;
    // add less than N instructions, when the ROB is full
    
    reset();
    @(posedge clock)

    #5 reset = 0;
    // add less than N instructions, when the ROB has stuff but not full

    reset();
    @(posedge clock)

    #5 reset = 0; */

  end 

  always @(posedge clock) begin
    $display("Time=%0t", $time);
    $display("Reset=%0d", reset);
    $display("open_entries=%0d", open_entries);
    $display("number of entries retired=%0d", num_retired);
    `ifdef DEBUG
      $display("entries: ");
      for (int j = 0; j < N; j++) begin
        $display("entry_data[%0d]:  op_code=%0d, t=%0d, t_old=%0d, complete=%0b, valid=%0b",
                j, entry_data[j].op_code, entry_data[j].t, entry_data[j].t_old,
                entry_data[j].complete, entry_data[j].valid);
      end
      $display("head=%0d", debug_head);
      $display("tail=%0d", debug_tail);
    `endif

    $display("retiring data: ");
    for (int i = 0; i < N; i++) begin
      $display("retiring_data[%0d]: op_code=%0d, t=%0d, t_old=%0d, complete=%0b, valid=%0b",
               i, retiring_data[i].op_code, retiring_data[i].t, retiring_data[i].t_old,
               retiring_data[i].complete, retiring_data[i].valid);
    end
    $display("----------------------");
  end

  function void fetch_entries() 
    for(int i=0;i<DEPTH*2;i++) begin
      ROB_ENTRY_PACKET inst = '{op_code: i[6:0], t: (i+1) % DEPTH, t_old: i % DEPTH, complete: 1'b0, valid: 1'b1 };
      inst_buff.push_back(inst);
    end
  endfunction

  function void reset()
    inst_buff.delete();
    rob_model.delete();
    wr_data = '0;
    complete_t = '0;
    num_accept = '0;
    reset = '1;
  endfunction

  function void add_entries(int num_add);
    for (int i = 0; i < num_add; i++) begin

        ROB_ENTRY_PACKET inst = inst_buff.pop_front();
        rob_model.push_back(inst);

        wr_data[i] = inst;

        inst.pop_front();
    end
  endfunction
  
  function void check_open_entries();
    int expected = (DEPTH - rob_model.size());
    
    if (open_entries != expected) begin
      $error("@@@ FAILED");
      $error("Open entries error: expected %0d, but got %0d", expected, open_entries);
      $finish;
    end
  endfunction

  function void set_complete(PHYS_REG_IDX [N-1:0] complete_idx);
    complete_t = complete_idx;

    for(int k=0; k < rob_model.size(); ++k) begin
      if(rob_model[k].t == complete_idx[j]) begin
          rob_model[k].complete = 'b1;
      end
    end
  endfunction

  function void check_retired_entries();
    int expected = 0;
    for (int i = 0; i < rob_model.size(); i++) begin
        if (rob_model[i].complete) begin
            expected++;
        end else begin 
            break;
        end
    end

    if(num_retired != expected) begin
      $error("@@@ FAILED");
      $error("Retirement error: expected (%0d) retires, but got %0d!", expected, num_retired);
      $finish;
    end
  endfunction

endmodule