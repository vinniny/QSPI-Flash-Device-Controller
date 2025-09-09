`timescale 1ns/1ps

// irq_dma_tb - Integration test for IRQs from real CMD and DMA events.
// - Enables CMD_DONE and DMA_DONE interrupts in CSR
// - Issues JEDEC ID read (0x9F) and checks IRQ + INT_STAT[0]
// - Issues DMA fast read (0x0B) for 16 bytes, checks IRQ + INT_STAT[1]

module irq_dma_tb;
  reg clk; reg resetn;

  // APB
  reg        psel, penable, pwrite; reg [11:0] paddr; reg [31:0] pwdata; reg [3:0] pstrb;
  wire [31:0] prdata; wire pready; wire pslverr;

  // AXI master (DMA)
  wire [31:0] m_awaddr; wire m_awvalid; wire m_awready;
  wire [31:0] m_wdata;  wire m_wvalid; wire [3:0] m_wstrb; wire m_wready;
  wire [1:0]  m_bresp;  wire m_bvalid; wire m_bready;
  wire [31:0] m_araddr; wire m_arvalid; wire m_arready;
  wire [31:0] m_rdata;  wire [1:0] m_rresp; wire m_rvalid; wire m_rready;

  // XIP slave (unused)
  wire [31:0] s_awaddr=32'h0; wire s_awvalid=1'b0; wire s_awready;
  wire [31:0] s_wdata =32'h0; wire [3:0] s_wstrb=4'h0; wire s_wvalid=1'b0; wire s_wready;
  wire [1:0]  s_bresp; wire s_bvalid; wire s_bready=1'b0;
  wire [31:0] s_araddr=32'h0; wire s_arvalid=1'b0; wire s_arready;
  wire [31:0] s_rdata; wire [1:0] s_rresp; wire s_rvalid; wire s_rready=1'b0;

  // QSPI pads
  wire sclk; wire cs_n; wire [3:0] io;

  wire irq;

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

  qspi_device flash (
    .qspi_sclk(sclk), .qspi_cs_n(cs_n), .qspi_io0(io[0]), .qspi_io1(io[1]), .qspi_io2(io[2]), .qspi_io3(io[3])
  );

  // Clock
  initial clk=0; always #5 clk=~clk;

  // VCD + timeout
  initial begin
    $dumpfile("irq_dma_tb.vcd");
    $dumpvars(1, irq_dma_tb);
    #30_000_000 $display("[irq_dma_tb] Global timeout"); $finish;
  end

  // APB helpers
  task apb_write(input [11:0] a, input [31:0] d);
  begin
    @(posedge clk); psel<=1; penable<=0; pwrite<=1; paddr<=a; pwdata<=d; pstrb<=4'hF;
    @(posedge clk); penable<=1; @(posedge clk);
    psel<=0; penable<=0; pwrite<=0; paddr<=0; pwdata<=0; pstrb<=0;
  end endtask
  task apb_read(input [11:0] a, output [31:0] d);
  begin
    @(posedge clk); psel<=1; penable<=0; pwrite<=0; paddr<=a; pstrb<=0;
    @(posedge clk); penable<=1; @(posedge clk); d=prdata;
    psel<=0; penable<=0; paddr<=0;
  end endtask
  task pop_rx(output [31:0] d);
    reg [31:0] tmp; begin apb_read(12'h048,tmp); apb_read(12'h048,d); end
  endtask

  // CSR addresses
  localparam CTRL    = 12'h004;
  localparam STATUS  = 12'h008;
  localparam INT_EN  = 12'h00C;
  localparam INT_STAT= 12'h010;
  localparam CLK_DIV = 12'h014;
  localparam CS_CTRL = 12'h018;
  localparam CMD_CFG = 12'h024;
  localparam CMD_OP  = 12'h028;
  localparam CMD_ADDR= 12'h02C;
  localparam CMD_LEN = 12'h030;
  localparam DMA_CFG = 12'h038;
  localparam DMA_ADDR= 12'h03C;
  localparam DMA_LEN = 12'h040;
  localparam FIFO_STAT=12'h04C;

  // High-level helpers
  task ctrl_enable(); begin apb_write(CTRL, 32'h0000_0001); end endtask
  task ctrl_trigger(); begin apb_write(CTRL, 32'h0000_0101); end endtask
  task ctrl_dma_enable(); begin apb_write(CTRL, 32'h0000_0041); end endtask
  task cfg_cmd(input [7:0] op, input [1:0] ab, input [3:0] dmy, input [31:0] addr, input [31:0] len, input is_wr);
    reg [31:0] cfg; reg [31:0] o;
  begin
    cfg = {19'd0, is_wr, dmy[3:0], ab[1:0], 2'b00, 2'b00};
    o   = {8'd0, 8'h00, op};
    apb_write(CMD_CFG, cfg); apb_write(CMD_OP,o); apb_write(CMD_ADDR, addr); apb_write(CMD_LEN, len);
  end endtask
  task cfg_dma(input [3:0] burst, input dir_read, input [31:0] addr, input [31:0] len);
    reg [31:0] d; begin d={26'd0,1'b1,dir_read,burst[3:0]}; apb_write(DMA_CFG,d); apb_write(DMA_ADDR,addr); apb_write(DMA_LEN,len); end
  endtask

  integer i; reg [31:0] d;

  initial begin
    // reset
    psel=0; penable=0; pwrite=0; paddr=0; pwdata=0; pstrb=0; resetn=0; repeat(10) @(posedge clk); resetn=1;
    // allow power-up time
    #900_000;
    // basic setup
    ctrl_enable(); apb_write(CLK_DIV, 32'h7); apb_write(CS_CTRL, 32'h0000_0019);
    // enable CMD_DONE and DMA_DONE interrupts
    apb_write(INT_EN, 32'h0000_0003);

    // 1) JEDEC ID read -> expect IRQ and INT_STAT[0]
    cfg_cmd(8'h9F, 2'b00, 4'd0, 32'h0, 32'd4, 1'b0); ctrl_trigger();
    // Wait for RX avail to guarantee cmd finished
    begin : wait_rx_id
      for (i=0;i<2000;i=i+1) begin apb_read(FIFO_STAT,d); if (d[7:4]!=0) disable wait_rx_id; @(posedge clk); end
    end
    pop_rx(d);
    if (!irq) $fatal(1, "IRQ not asserted after CMD_DONE");
    apb_read(INT_STAT,d); if (d[0]!==1'b1) $fatal(1, "INT_STAT[0] not set");
    apb_write(INT_STAT, 32'h1); // W1C
    apb_read(INT_STAT,d); if (d[0]!==1'b0) $fatal(1, "INT_STAT[0] not cleared");

    // 2) DMA fast read, 16 bytes -> expect IRQ and INT_STAT[1]
    cfg_dma(4'd4, 1'b1, 32'h0000_0000, 32'd16);
    cfg_cmd(8'h0B, 2'b01, 4'd8, 32'h0, 32'd16, 1'b0);
    // Combined DMA_EN + TRIGGER in single CTRL write
    apb_write(CTRL, 32'h0000_0141);
    // Wait for DMA_DONE interrupt status to latch
    begin : wait_dma
      for (i=0;i<400000;i=i+1) begin
        apb_read(INT_STAT,d); if (d[1]==1'b1) disable wait_dma; @(posedge clk);
      end
    end
    if (d[1]!==1'b1) $fatal(1, "DMA_DONE not observed");
    if (!irq) $fatal(1, "IRQ not asserted after DMA_DONE");
    apb_write(INT_STAT, 32'h2); // W1C
    apb_read(INT_STAT,d); if (d[1]!==1'b0) $fatal(1, "INT_STAT[1] not cleared");

    $display("irq_dma_tb: passed");
    $finish;
  end
endmodule
