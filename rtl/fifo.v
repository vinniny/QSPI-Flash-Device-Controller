module fifo (
    input wire clk,
    input wire rst_n,
    input wire wr_en,        // Write enable signal
    input wire [31:0] data_in,  // Data to be written
    output reg full,         // Full flag
    input wire rd_en,        // Read enable signal
    output reg [31:0] data_out, // Data read from FIFO
    output reg empty         // Empty flag
);

    // FIFO storage
    reg [31:0] mem [0:255];  // FIFO size: 256 entries (32-bit words)
    reg [7:0] write_ptr;     // Write pointer
    reg [7:0] read_ptr;      // Read pointer

    // FIFO write operation
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            write_ptr <= 8'd0;
            read_ptr  <= 8'd0;
            full      <= 1'b0;
            empty     <= 1'b1;
            data_out  <= 32'd0;
        end else begin
            // Write operation
            if (wr_en && !full) begin
                mem[write_ptr] <= data_in;
                write_ptr <= write_ptr + 1;
            end

            // Read operation
            if (rd_en && !empty) begin
                data_out <= mem[read_ptr];
                read_ptr  <= read_ptr + 1;
            end

            // Update full and empty flags
            full  <= (write_ptr == (read_ptr - 1)) ? 1'b1 : 1'b0;
            empty <= (write_ptr == read_ptr) ? 1'b1 : 1'b0;
        end
    end
endmodule
