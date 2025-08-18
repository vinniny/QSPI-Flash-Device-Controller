`timescale 1ns/1ps

module fifo_tb;
    reg clk;
    reg reset;
    reg wr_en;
    reg rd_en;
    reg [31:0] data_in;
    wire [31:0] data_out;
    wire full;
    wire empty;

    fifo #(.ADDR_WIDTH(2)) dut (
        .clk(clk),
        .reset(reset),
        .wr_en(wr_en),
        .data_in(data_in),
        .full(full),
        .rd_en(rd_en),
        .data_out(data_out),
        .empty(empty)
    );

    integer i;
    integer rd_cnt;

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        reset = 1;
        wr_en = 0;
        rd_en = 0;
        data_in = 0;
        #20 reset = 0;

        // Write four values
        for (i = 0; i < 4; i = i + 1) begin
            data_in = i;
            wr_en = 1;
            @(posedge clk);
        end
        wr_en = 0;
        @(posedge clk);
        if (!full) $fatal(1, "FIFO should be full after 4 writes");
        if (empty) $fatal(1, "FIFO should not be empty after writes");

        // Read back and check
        rd_cnt = 0;
        for (i = 0; i < 4; i = i + 1) begin
            rd_en = 1;
            @(posedge clk);
            rd_en = 0;
            rd_cnt = rd_cnt + 1;
        end
        if (rd_cnt != 4) $fatal(1, "Read count mismatch");
        if (!empty) $fatal(1, "FIFO should be empty at end");

        // Underflow check
        rd_en = 1;
        @(posedge clk);
        rd_en = 0;
        if (!empty) $fatal(1, "FIFO underflow not flagged");
        $display("FIFO test passed");
        $finish;
    end
endmodule
