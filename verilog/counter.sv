`include "sys_defs.svh"
`include "ISA.svh"

module counter(
    input               clock, 
    input               reset,

    input logic         taken,
    input logic         wr_en,

    output logic        pred
);
    
    TWO_BIT_COUNTER state, next_state;

    always_comb begin
        case (state)
            STRONG_NT:  pred = 0;
            NT:         pred = 0;
            T:          pred = 1;
            STRONG_T:   pred = 1;
            default:    pred = 0;
        endcase
    end

    always_comb begin
        next_state = state;

        if (wr_en) begin
            case (state)
                STRONG_NT:  next_state = taken ? NT : STRONG_NT;
                NT:         next_state = taken ? T : STRONG_NT;
                T:          next_state = taken ? STRONG_T : NT;
                STRONG_T:   next_state = taken ? STRONG_T : T;
                default:    next_state = STRONG_NT;
            endcase
        end
    end


    always_ff @(posedge clock) begin
        if (reset) begin
            state <= STRONG_NT;
        end else begin
            state <= next_state;
        end
    end

endmodule