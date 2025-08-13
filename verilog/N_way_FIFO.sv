// Simple FIFO with parametrizable depth and width

module FIFO #(
    parameter DEPTH = 16,
    parameter WIDTH = 32,
    parameter ALERT_DEPTH = 3,
    parameter N = 1
) (
    input                           clock, 
    input                           reset,
    input                           wr_num,
    input                           rd_num,
    input        [N-1:0][WIDTH-1:0] wr_data,
    output logic                    wr_valid,
    output logic                    rd_valid,
    output logic [N-1:0][WIDTH-1:0] rd_data,
    output logic                    almost_full,
    output logic                    full
);
    localparam LOG_DEPTH = $clog2(DEPTH);

    typedef enum logic [1:0] {EMPTY, LOAD, FULL} STATE;

    logic [N-1:0][WIDTH-1:0] head_val;

    logic [DEPTH-1:0] valid, next_valid;

    logic [LOG_DEPTH-1:0] head, next_head;
    logic [LOG_DEPTH-1:0] tail, next_tail;
    logic [LOG_DEPTH-1:0] num_entries;

    STATE state, next_state;

    logic empty;

    // LAB5 TODO: Make the FIFO, see other TODOs below
    // Some things you will need to do:
    // - Define the sizes for your head (read) and tail (write) pointer
    // - Increment the pointers if needed (hint: try using the modulo operator: '%')
    // - Write to the tail when wr_en == 1 and the fifo isn't full
    // - Read from the head when rd_en == 1 and the fifo isn't empty
    
    memDP #(
        .WIDTH     (WIDTH),
        .DEPTH     (DEPTH),
        .PORTS     (N),
        .BYPASS_EN (0))
    fifo_mem (
        .clock  (clock),
        .reset  (reset),
        .re     (rd_valid),  // [READ_PORTS-1:0] Read enable
        .raddr  (head),  // [READ_PORTS-1:0][$clog2(DEPTH)-1:0] Read address
        .rdata  (head_val),  // [READ_PORTS-1:0][WIDTH-1:0]Read data
        .we     (wr_valid),  // Write enable
        .waddr  (tail),  // [$clog2(DEPTH)-1:0] Write address
        .wdata  (wr_data)   // [WIDTH-1:0] Write data
    );

    // LAB5 TODO: Use one of three ways to track if full/empty:
    //  1. (easiest) Keep a count of the number of entries
    //  2. (medium)  Use a valid bit for each entry
    //  3. (hardest) Use head == tail and keep a state of empty vs. full in always_ff
    //               This is hardest because you also need to track almost_full

    always_comb begin
        next_head = head;
        next_tail = tail;
        next_valid = valid;

        // handle pointer incrementing and valid vits
        if (rd_valid) begin
            next_valid &= ~(1 << head);
            next_head = (head + 1) % DEPTH;
        end
        if (wr_valid) begin
            next_valid |= (1 << tail);
            next_tail = (tail + 1) % DEPTH;
        end

        // handle state transitions
        if (&next_valid) begin
            next_state = FULL;
        end else if (~|next_valid) begin
            next_state = EMPTY;
        end else begin
            next_state = LOAD;
        end
    end


    // LAB5 TODO: Output read data from the head combinationally
    //            (doing this correctly in always_ff is somewhat difficult)
    assign rd_data = rd_en ? head_val : '0;

    assign full = (state == FULL);
    assign empty = (state == EMPTY);

    assign num_entries = (tail >= head) ? (tail - head) : (DEPTH - head + tail);
    assign almost_full = (DEPTH - num_entries == ALERT_DEPTH);

    assign rd_valid = rd_en & ~empty;
    assign wr_valid = wr_en & (~full | rd_valid);

    always_ff @(posedge clock) begin
        if (reset) begin
            state <= EMPTY;
            head <= '0;
            tail <= '0;
            valid <= '0;
        end else begin
            state <= next_state;
            head <= next_head;
            tail <= next_tail;
            valid <= next_valid;
        end
    end

endmodule
