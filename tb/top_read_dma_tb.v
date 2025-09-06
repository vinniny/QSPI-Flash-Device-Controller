`timescale 1ns/1ps

module top_read_dma_tb;
  reg clk; reg resetn;
  // APB
  reg        psel; reg penable; reg pwrite; reg [11:0] paddr; reg [31:0] pwdata; reg [3:0] pstrb;
  wire [31:0] prdata; wire pready; wire pslverr;

  // AXI master (DMA)
  wire [31:0] m_awaddr; wire m_awvalid; wire m_awready;
  wire [31:0] m_wdata;  wire m_wvalid;  wire [3:0] m_wstrb; wire m_wready;
  wire [1:0]  m_bresp;  wire m_bvalid;  wire m_bready;
  wire [31:0] m_araddr; wire m_arvalid; wire m_arready;
  wire [31:0] m_rdata;  wire [1:0] m_rresp; wire m_rvalid; wire m_rready;

  // XIP ignored
  wire [31:0] s_awaddr = 32'h0; wire s_awvalid = 1'b0; wire s_awready;
  wire [31:0] s_wdata = 32'h0;  wire [3:0] s_wstrb = 4'h0; wire s_wvalid = 1'b0; wire s_wready;
  wire [1:0]  s_bresp; wire s_bvalid; wire s_bready = 1'b0;
  wire [31:0] s_araddr; wire s_arvalid; wire s_arready;
  wire [31:0] s_rdata;  wire [1:0] s_rresp; wire s_rvalid; wire s_rready;

  wire sclk; wire cs_n; wire [3:0] io; wire irq;

  qspi_controller dut (
    .clk(clk), .resetn(resetn),
    .psel(psel), .penable(penable), .pwrite(pwrite), .paddr(paddr), .pwdata(pwdata), .pstrb(pstrb),
    .prdata(prdata), .pready(pready), .pslverr(pslverr),
    .m_axi_awaddr(m_awaddr), .m_axi_awvalid(m_awvalid), .m_axi_awready(m_awready),
    .m_axi_wdata(m_wdata), .m_axi_wvalid(m_wvalid), .m_axi_wstrb(m_wstrb), .m_axi_wready(m_wready),
    .m_axi_bvalid(m_bvalid), .m_axi_bresp(m_bresp), .m_axi_bready(m_bready),
    .m_axi_araddr(m_araddr), .m_axi_arvalid(m_arvalid), .m_axi_arready(m_arready),
    .m_axi_rdata(m_rdata), .m_axi_rvalid(m_rvalid), .m_axi_rresp(m_rresp), .m_axi_rready(m_rready),
    .s_axi_awaddr(s_awaddr), .s_axi_awvalid(s_awvalid), .s_axi_awready(s_awready),
    .s_axi_wdata(s_wdata), .s_axi_wstrb(s_wstrb), .s_axi_wvalid(s_wvalid), .s_axi_wready(s_wready),
    .s_axi_bresp(s_bresp), .s_axi_bvalid(s_bvalid), .s_axi_bready(s_bready),
    .s_axi_araddr(s_araddr), .s_axi_arvalid(s_arvalid), .s_axi_arready(s_arready),
    .s_axi_rdata(s_rdata), .s_axi_rresp(s_rresp), .s_axi_rvalid(s_rvalid), .s_axi_rready(s_rready),
    .sclk(sclk), .cs_n(cs_n), .io(io), .irq(irq)
  );

  axi4_ram_slave mem (
    .clk(clk), .resetn(resetn),
    .awaddr(m_awaddr), .awvalid(m_awvalid), .awready(m_awready),
    .wdata(m_wdata), .wstrb(m_wstrb), .wvalid(m_wvalid), .wready(m_wready),
    .bresp(m_bresp), .bvalid(m_bvalid), .bready(m_bready),
    .araddr(m_araddr), .arvalid(m_arvalid), .arready(m_arready),
    .rdata(m_rdata), .rresp(m_rresp), .rvalid(m_rvalid), .rready(m_rready)
  );

  // Full Macronix flash model (Mode 0)
  MX25L6436F flash (
    .SCLK(sclk), .CS(cs_n), .SI(io[0]), .SO(io[1]), .WP(io[2]), .SIO3(io[3])
  );

  initial clk=0; always #5 clk=~clk;
  initial begin
    $dumpfile("top_read_dma_tb.vcd");
    $dumpvars(1, top_read_dma_tb);
    #2_000_000 $dumpoff; // limit VCD size
  end

  // Global timeout safety
  initial begin
    #3_500_000; // 3.5 ms cutoff
    $display("[top_read_dma_tb] Global timeout reached — finishing.");
    $finish;
  end

  task apb_write(input [11:0] addr, input [31:0] data);
  begin @(posedge clk); psel<=1;penable<=0;pwrite<=1;paddr<=addr;pwdata<=data;pstrb<=4'hF; @(posedge clk); penable<=1; @(posedge clk);
        psel<=0;penable<=0;pwrite<=0;paddr<=0;pwdata<=0;pstrb<=0; end endtask
  
  // CSR addrs
  localparam CTRL=12'h004, CLKDIV=12'h014, CS_CTRL=12'h018, CMD_CFG=12'h024, CMD_OP=12'h028,
             CMD_ADDR=12'h02C, CMD_LEN=12'h030, DMA_CFG=12'h038, DMA_ADDR=12'h03C, DMA_LEN=12'h040;

  integer i; reg [31:0] d;
  initial begin
    resetn=0; psel=0; penable=0; pwrite=0; paddr=0; pwdata=0; pstrb=0; d=0; repeat(10) @(posedge clk); resetn=1;
    // MXIC power-up tVSL (~800us)
    #900_000;
    // Mode 0 (CPOL=0, CPHA=0), clkdiv, cs auto
    apb_write(CTRL, 32'h0000_0001);
    apb_write(CLKDIV, 32'h0000_0004);
    apb_write(CS_CTRL, 32'h0000_0001);
    // DMA: dir=1 (RX->mem), burst=1, incr=1, addr=0, len=4
    apb_write(DMA_CFG, 32'h0000_0031);
    apb_write(DMA_ADDR,32'h0000_0000);
    apb_write(DMA_LEN, 32'h0000_0004);
    // CMD 0x03 read 4B from 0
    apb_write(CMD_CFG, 32'h0000_0040);
    apb_write(CMD_OP,  32'h0000_0003);
    apb_write(CMD_ADDR,32'h0000_0000);
    apb_write(CMD_LEN, 32'h0000_0004);
    // Enable DMA and trigger (Mode 0)
    apb_write(CTRL, 32'h0000_0041);
    apb_write(CTRL, 32'h0000_0141);
    // wait some cycles
    repeat(2000) @(posedge clk);
    if (mem.mem[0] !== 32'hFFFF_FFFF) $fatal(1, "DMA read mismatch: %h", mem.mem[0]);
    $display("Top read DMA test passed");
    $finish;
  end
endmodule
