`timescale 1ns/1ps

module axi_write_block_tb;
    reg clk;
    reg rst_n;
    reg start;
    reg [31:0] addr;
    reg [15:0] transfer_size;
    wire [31:0] awaddr;
    wire awvalid;
    reg awready;
    wire [31:0] wdata;
    wire wvalid;
    wire [3:0] wstrb;
    reg wready;
    reg bvalid;
    wire bready;
    reg [31:0] data_in;
    reg empty;
    wire rd_en;
    wire busy;
    wire done;

    axi_write_block dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .addr(addr),
        .transfer_size(transfer_size),
        .awaddr(awaddr),
        .awvalid(awvalid),
        .awready(awready),
        .wdata(wdata),
        .wvalid(wvalid),
        .wstrb(wstrb),
        .wready(wready),
        .bvalid(bvalid),
        .bready(bready),
        .data_in(data_in),
        .empty(empty),
        .rd_en(rd_en),
        .busy(busy),
        .done(done)
    );

    reg [31:0] fifo_mem [0:1];
    reg [31:0] captured [0:1];
    integer idx;
    integer write_cnt;

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        rst_n = 0;
        start = 0;
        addr = 0;
        transfer_size = 16'd8;
        awready = 1;
        wready = 1;
        bvalid = 1;
        data_in = 0;
        empty = 0;
        idx = 0;
        write_cnt = 0;
        fifo_mem[0] = 32'hDEADBEEF;
        fifo_mem[1] = 32'h12345678;
        #20 rst_n = 1;
        start = 1;
        @(posedge clk);
        start = 0;
    end

    always @(posedge clk) begin
        if (rd_en) begin
            data_in <= fifo_mem[idx];
            idx <= idx + 1;
            empty <= (idx + 1 >= 2);
        end
        if (wvalid && wready) begin
            captured[write_cnt] <= wdata;
            write_cnt <= write_cnt + 1;
        end
    end

    // Check address once at accept
    always @(posedge clk) if (awvalid && awready) begin
        if (awaddr !== addr) $fatal(1, "Unexpected AWADDR %h", awaddr);
    end

    initial begin
        // Wait for both words to be written
        wait (write_cnt == 2);
        // Give one cycle for B to handshake if needed
        @(posedge clk);
        if (captured[0] !== fifo_mem[0] || captured[1] !== fifo_mem[1]) begin
            $fatal(1, "AXI write data mismatch: got %h %h exp %h %h",
                   captured[0], captured[1], fifo_mem[0], fifo_mem[1]);
        end
        // Check done
        if (!done) begin
            // Allow the block to raise done in the next few cycles
            repeat (4) @(posedge clk);
            if (!done) $fatal(1, "AXI write block never asserted done");
        end
        $display("AXI write block test passed");
        $finish;
    end
endmodule
