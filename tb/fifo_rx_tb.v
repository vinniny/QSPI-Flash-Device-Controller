`timescale 1ns/1ps

module fifo_rx_tb;
    localparam WIDTH = 32;
    localparam DEPTH = 4;

    reg                    clk;
    reg                    resetn;
    reg                    wr_en;      // from FSM
    reg  [WIDTH-1:0]       wr_data;
    reg                    rd_en;      // to CSR/DMA
    wire [WIDTH-1:0]       rd_data;
    wire                   full;
    wire                   empty;
    wire [7:0] level;

    integer i;
    reg [WIDTH-1:0] expd;

    fifo_rx #(.WIDTH(WIDTH), .DEPTH(DEPTH)) dut (
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
        $dumpfile("fifo_rx_tb.vcd");
        $dumpvars(0, fifo_rx_tb);
        resetn = 0;
        wr_en  = 0;
        rd_en  = 0;
        wr_data = 0;
        expd = 0;
        repeat (3) @(posedge clk);
        resetn = 1;

        // Push DEPTH entries from "FSM"
        for (i = 0; i < DEPTH; i = i + 1) begin
            @(posedge clk);
            wr_data <= 32'h1000 + i;
            wr_en   <= 1;
        end
        @(posedge clk);
        wr_en <= 0;
        @(posedge clk);

        if (!full) $fatal(1, "fifo_rx: full should be high after %0d writes", DEPTH);
        if (empty) $fatal(1, "fifo_rx: empty should be low after writes");

        // Pop and check order
        for (i = 0; i < DEPTH; i = i + 1) begin
            @(posedge clk);
            rd_en <= 1;
            @(posedge clk);
            rd_en <= 0;
            @(negedge clk);
            if (rd_data !== (32'h1000 + expd)) $fatal(1, "fifo_rx: data mismatch");
            expd = expd + 1;
        end

        @(posedge clk);
        if (!empty) $fatal(1, "fifo_rx: empty should be high at end");

        // Extra read keeps empty asserted
        rd_en <= 1; @(posedge clk); rd_en <= 0;
        if (!empty) $fatal(1, "fifo_rx: underflow not handled (empty should remain high)");

        $display("FIFO RX test passed");
        $finish;
    end

    // Global timeout to prevent stalls
    initial begin
        #1_000_000; // 1 ms cutoff
        $display("[fifo_rx_tb] Global timeout reached — finishing.");
        $finish;
    end
endmodule
