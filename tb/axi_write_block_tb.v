`timescale 1ns/1ps

module axi_write_block_tb;
    reg clk;
    reg reset;
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
        .reset(reset),
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

    reg [31:0] fifo_mem [0:3];
    reg [31:0] captured [0:3];
    integer idx;
    integer write_cnt;

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        reset = 1;
        start = 0;
        addr = 0;
        transfer_size = 16'd16;
        awready = 1;
        wready = 1;
        bvalid = 1;
        data_in = 0;
        empty = 0;
        idx = 0;
        write_cnt = 0;
        fifo_mem[0] = 32'hDEADBEEF;
        fifo_mem[1] = 32'h12345678;
        fifo_mem[2] = 32'hA5A5A5A5;
        fifo_mem[3] = 32'h00000000;
        #20 reset = 0;
        start = 1;
        @(posedge clk);
        start = 0;
    end

    always @(posedge clk) begin
        if (rd_en) begin
            data_in <= fifo_mem[idx];
            idx <= idx + 1;
            empty <= (idx + 1 >= 4);
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
        // Wait for all words to be written
        wait (write_cnt == 4);
        // Give one cycle for B to handshake if needed
        @(posedge clk);
        if (captured[0] !== fifo_mem[0] || captured[1] !== fifo_mem[1] ||
            captured[2] !== fifo_mem[2] || captured[3] !== fifo_mem[3]) begin
            $fatal(1, "AXI write data mismatch");
        end
        if (!done) begin
            repeat (4) @(posedge clk);
            if (!done) $fatal(1, "AXI write block never asserted done");
        end
        if (busy) $fatal(1, "Busy stuck high at end");
        $display("AXI write block test passed");
        $finish;
    end
endmodule
