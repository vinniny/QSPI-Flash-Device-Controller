`timescale 1ns/1ps

module fifo_tb;
    reg clk;
    reg rst_n;
    reg wr_en;
    reg rd_en;
    reg [31:0] data_in;
    wire [31:0] data_out;
    wire full;
    wire empty;

    fifo dut (
        .clk(clk),
        .rst_n(rst_n),
        .wr_en(wr_en),
        .data_in(data_in),
        .full(full),
        .rd_en(rd_en),
        .data_out(data_out),
        .empty(empty)
    );

    reg [31:0] exp [0:3];
    integer i;

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        rst_n = 0;
        wr_en = 0;
        rd_en = 0;
        data_in = 0;
        #20 rst_n = 1;

        // Write four values
        for (i = 0; i < 4; i = i + 1) begin
            data_in = i;
            exp[i] = i;
            wr_en = 1;
            @(posedge clk);
        end
        wr_en = 0;
        // Full/empty sanity after writes
        if (full) $fatal(1, "FIFO should not be full after 4 writes");
        if (empty) $fatal(1, "FIFO should not be empty after writes");

        // Read back and check
        for (i = 0; i < 4; i = i + 1) begin
            rd_en = 1;
            @(posedge clk);
            rd_en = 0;
            @(posedge clk);
            if (data_out !== exp[i]) begin
                $fatal(1, "FIFO data mismatch at %0d: got %h exp %h", i, data_out, exp[i]);
            end
        end
        if (!empty) $fatal(1, "FIFO should be empty at end");
        $display("FIFO test passed");
        $finish;
    end
endmodule
