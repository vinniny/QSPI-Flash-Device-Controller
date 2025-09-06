`timescale 1ns/1ps

module fifo_tx_tb;
    localparam WIDTH = 32;
    localparam DEPTH = 4;

    reg                    clk;
    reg                    resetn;
    reg                    wr_en;
    reg  [WIDTH-1:0]       wr_data;
    reg                    rd_en;
    wire [WIDTH-1:0]       rd_data;
    wire                   full;
    wire                   empty;
    wire [7:0] level;

    integer i;
    reg [WIDTH-1:0] expd;

    fifo_tx #(.WIDTH(WIDTH), .DEPTH(DEPTH)) dut (
        .clk       (clk),
        .resetn    (resetn),
        .wr_en_i   (wr_en),
        .wr_data_i (wr_data),
        .rd_en_i   (rd_en),
        .rd_data_o (rd_data),
        .full_o    (full),
        .empty_o   (empty),
        .level_o   (level)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        $dumpfile("fifo_tx_tb.vcd");
        $dumpvars(0, fifo_tx_tb);
        resetn = 0;
        wr_en  = 0;
        rd_en  = 0;
        wr_data = 0;
        expd = 0;
        repeat (3) @(posedge clk);
        resetn = 1;

        // Write DEPTH entries
        for (i = 0; i < DEPTH; i = i + 1) begin
            @(posedge clk);
            wr_data <= i;
            wr_en   <= 1;
        end
        @(posedge clk);
        wr_en <= 0;
        @(posedge clk);

        if (!full) $fatal(1, "fifo_tx: full should be high after %0d writes", DEPTH);
        if (empty) $fatal(1, "fifo_tx: empty should be low after writes");

        // Read back and check order
        for (i = 0; i < DEPTH; i = i + 1) begin
            @(posedge clk);
            rd_en <= 1;
            @(posedge clk);
            rd_en <= 0;
            // data is presented in the cycle of read; check after a small delay
            @(negedge clk);
            if (rd_data !== expd) $fatal(1, "fifo_tx: data mismatch exp=%h got=%h at %0d", expd, rd_data, i);
            expd = expd + 1;
        end

        @(posedge clk);
        if (!empty) $fatal(1, "fifo_tx: empty should be high at end");

        // Extra read should not change empty
        rd_en <= 1; @(posedge clk); rd_en <= 0;
        if (!empty) $fatal(1, "fifo_tx: underflow not handled (empty should remain high)");

        $display("FIFO TX test passed");
        $finish;
    end
endmodule
