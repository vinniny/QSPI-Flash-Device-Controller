module fifo #(
    parameter ADDR_WIDTH = 8,
    parameter DEPTH = 1 << ADDR_WIDTH
) (
    input  wire        clk,
    input  wire        reset,
    input  wire        wr_en,
    input  wire [31:0] data_in,
    output reg         full,
    input  wire        rd_en,
    output reg  [31:0] data_out,
    output reg         empty
);

    reg [31:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH-1:0] wptr;
    reg [ADDR_WIDTH-1:0] rptr;
    reg [ADDR_WIDTH:0]   count;
    reg [ADDR_WIDTH:0]   next_count;

    always @(posedge clk) begin
        if (reset) begin
            wptr      <= {ADDR_WIDTH{1'b0}};
            rptr      <= {ADDR_WIDTH{1'b0}};
            count     <= {(ADDR_WIDTH+1){1'b0}};
            full      <= 1'b0;
            empty     <= 1'b1;
            data_out  <= 32'd0;
        end else begin
            next_count = count;
            case ({wr_en & ~full, rd_en & ~empty})
                2'b10: begin
                    mem[wptr] <= data_in;
                    wptr      <= wptr + 1'b1;
                    next_count = count + 1'b1;
                end
                2'b01: begin
                    data_out  <= mem[rptr];
                    rptr      <= rptr + 1'b1;
                    next_count = count - 1'b1;
                end
                2'b11: begin
                    mem[wptr] <= data_in;
                    wptr      <= wptr + 1'b1;
                    data_out  <= mem[rptr];
                    rptr      <= rptr + 1'b1;
                end
                default: begin
                end
            endcase
            count <= next_count;
            full  <= (next_count == DEPTH);
            empty <= (next_count == 0);
        end
    end
endmodule
