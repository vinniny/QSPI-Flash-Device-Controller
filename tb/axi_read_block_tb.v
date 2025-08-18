`timescale 1ns/1ps

module axi_read_block_tb;
    reg clk;
    reg reset;
    reg start;
    reg [31:0] addr;
    reg [15:0] transfer_size;
    wire [31:0] araddr;
    wire arvalid;
    reg arready;
    reg rvalid;
    reg [31:0] rdata;
    wire rready;
    wire [31:0] data_out;
    wire wr_en;
    reg full;
    wire busy;
    wire done;

    axi_read_block dut (
        .clk(clk),
        .reset(reset),
        .start(start),
        .addr(addr),
        .transfer_size(transfer_size),
        .araddr(araddr),
        .arvalid(arvalid),
        .arready(arready),
        .rvalid(rvalid),
        .rdata(rdata),
        .rready(rready),
        .data_out(data_out),
        .wr_en(wr_en),
        .full(full),
        .busy(busy),
        .done(done)
    );

    integer count;

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        clk = 0;
        reset = 1;
        start = 0;
        addr = 0;
        transfer_size = 16'd16;
        arready = 0;
        rvalid = 0;
        rdata = 0;
        full = 0;
        count = 0;
        #20 reset = 0;
        @(posedge clk);
        start <= 1;
        @(posedge clk);
        start <= 0;
    end

    always @(posedge clk) begin
        if (arvalid) begin
            arready <= 1;
            rdata   <= araddr;
            rvalid  <= 1;
        end else begin
            arready <= 0;
            if (rready && rvalid) rvalid <= 0;
        end

        if (wr_en) begin
            if (data_out !== (addr + count * 4))
                $fatal(1, "Unexpected data %h", data_out);
            count <= count + 1;
        end

        if (count == 2) full <= 1; else if (wr_en) full <= 0;

        if (done) begin
            if (count != 4) $fatal(1, "Wrong word count");
            if (busy) $fatal(1, "Busy stuck high at done");
            $display("AXI read block test passed");
            $finish;
        end
    end
endmodule
