`timescale 1ns/1ps

module tb_axi4_ram_slave;
  reg         clk;
  reg         resetn;

  // AXI4-Lite signals
  reg  [31:0] awaddr;
  reg         awvalid;
  wire        awready;
  reg  [31:0] wdata;
  reg  [3:0]  wstrb;
  reg         wvalid;
  wire        wready;
  wire [1:0]  bresp;
  wire        bvalid;
  reg         bready;
  reg  [31:0] araddr;
  reg         arvalid;
  wire        arready;
  wire [31:0] rdata;
  wire [1:0]  rresp;
  wire        rvalid;
  reg         rready;

  axi4_ram_slave #(.ADDR_WIDTH(32), .MEM_WORDS(256)) dut (
    .clk(clk), .resetn(resetn),
    .awaddr(awaddr), .awvalid(awvalid), .awready(awready),
    .wdata(wdata), .wstrb(wstrb), .wvalid(wvalid), .wready(wready),
    .bresp(bresp), .bvalid(bvalid), .bready(bready),
    .araddr(araddr), .arvalid(arvalid), .arready(arready),
    .rdata(rdata), .rresp(rresp), .rvalid(rvalid), .rready(rready)
  );

  initial clk = 0;
  always #5 clk = ~clk;

  task axi_write(input [31:0] addr, input [31:0] data, input [3:0] strb);
  begin
    awaddr  <= addr; awvalid <= 1;
    wdata   <= data; wstrb  <= strb; wvalid <= 1;
    bready  <= 1;
    @(posedge clk);
    while(!(awready)) @(posedge clk);
    awvalid <= 0;
    while(!(wready)) @(posedge clk);
    wvalid <= 0;
    while(!(bvalid)) @(posedge clk);
    @(posedge clk);
    bready <= 0;
  end
  endtask

  task axi_read(input [31:0] addr, output [31:0] data);
  begin
    araddr <= addr; arvalid <= 1; rready <= 1;
    @(posedge clk);
    while(!(arready)) @(posedge clk);
    arvalid <= 0;
    while(!(rvalid)) @(posedge clk);
    data = rdata;
    rready <= 0;
  end
  endtask

  reg [31:0] tmp;

initial begin
    $dumpfile("axi4_ram_slave_tb.vcd");
    $dumpvars(0, tb_axi4_ram_slave);
    resetn = 0;
    awaddr=0; awvalid=0; wdata=0; wstrb=0; wvalid=0; bready=0;
    araddr=0; arvalid=0; rready=0;
    repeat (4) @(posedge clk);
    resetn = 1;

    // Write and read back
    axi_write(32'h0000_0000, 32'hDEADBEEF, 4'hF);
    axi_write(32'h0000_0004, 32'h12345678, 4'hF);
    axi_write(32'h0000_0008, 32'hCAFEBABE, 4'hF);

    axi_read(32'h0, tmp);       if (tmp !== 32'hDEADBEEF) $fatal(1, "Readback mismatch 0");
    axi_read(32'h4, tmp);       if (tmp !== 32'h12345678) $fatal(1, "Readback mismatch 4");
    axi_read(32'h8, tmp);       if (tmp !== 32'hCAFEBABE) $fatal(1, "Readback mismatch 8");

    // Byte strobe test
    axi_write(32'h0000_0000, 32'h0000_00AA, 4'b0001); // update byte0
    axi_read(32'h0, tmp);       if (tmp !== 32'hDEADBEAA) $fatal(1, "WSTRB[0] failed");
    axi_write(32'h0000_0000, 32'h00BB_0000, 4'b0100); // update byte2
    axi_read(32'h0, tmp);       if (tmp !== 32'hDEBB_BEAA) $fatal(1, "WSTRB[2] failed");

    $display("AXI4 RAM slave test passed");
    $finish;
  end

  // Global timeout to prevent stalls
  initial begin
    #1_000_000; // 1 ms cutoff
    $display("[tb_axi4_ram_slave] Global timeout reached — finishing.");
    $finish;
  end
endmodule
