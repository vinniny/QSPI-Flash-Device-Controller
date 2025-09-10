`timescale 1ns/1ps

// cmd_dma_burst_tb - Command-mode multiword DMA read burst (fast read 0x0B)
module cmd_dma_burst_tb;
  // Clock/reset
  reg clk; reg resetn;

  // APB
  reg        psel; reg penable; reg pwrite; reg [11:0] paddr; reg [31:0] pwdata; reg [3:0] pstrb;
  wire [31:0] prdata; wire pready; wire pslverr;

  // AXI master (DMA)
  wire [31:0] m_awaddr; wire m_awvalid; wire m_awready; wire [31:0] m_wdata; wire m_wvalid; wire [3:0] m_wstrb; wire m_wready; wire [1:0] m_bresp; wire m_bvalid; wire m_bready;
  wire [31:0] m_araddr; wire m_arvalid; wire m_arready; wire [31:0] m_rdata; wire [1:0] m_rresp; wire m_rvalid; wire m_rready;

  // XIP AXI slave (unused)
  wire [31:0] s_awaddr = 32'h0; wire s_awvalid = 1'b0; wire s_awready; wire [31:0] s_wdata = 32'h0; wire [3:0] s_wstrb = 4'h0; wire s_wvalid = 1'b0; wire s_wready; wire [1:0] s_bresp; wire s_bvalid; wire s_bready = 1'b0;
  wire [31:0] s_araddr; wire s_arvalid; wire s_arready; wire [31:0] s_rdata; wire [1:0] s_rresp; wire s_rvalid; wire s_rready;

  // QSPI pads
  wire sclk; wire cs_n; wire [3:0] io;

  wire irq;

  // DUT
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

  // AXI4-Lite RAM for DMA
  axi4_ram_slave mem (
    .clk(clk), .resetn(resetn),
    .awaddr(m_awaddr), .awvalid(m_awvalid), .awready(m_awready),
    .wdata(m_wdata), .wstrb(m_wstrb), .wvalid(m_wvalid), .wready(m_wready),
    .bresp(m_bresp), .bvalid(m_bvalid), .bready(m_bready),
    .araddr(m_araddr), .arvalid(m_arvalid), .arready(m_arready),
    .rdata(m_rdata), .rresp(m_rresp), .rvalid(m_rvalid), .rready(m_rready)
  );

  // Flash model
  qspi_device #(.CS_HIGH_MIN_NS(0)) flash (
    .qspi_sclk(sclk), .qspi_cs_n(cs_n), .qspi_io0(io[0]), .qspi_io1(io[1]), .qspi_io2(io[2]), .qspi_io3(io[3])
  );

  // Clock
  initial clk = 0; always #5 clk = ~clk;

  // VCD + timeout
  initial begin
    $dumpfile("cmd_dma_burst_tb.vcd");
    $dumpvars(1, cmd_dma_burst_tb);
    #60_000_000 $display("[cmd_dma_burst_tb] Global timeout reached — finishing.");
    $finish;
  end

  // APB helpers
  task apb_write(input [11:0] addr, input [31:0] data);
  begin
    @(posedge clk);
    psel <= 1; penable <= 0; pwrite <= 1; paddr <= addr; pwdata <= data; pstrb <= 4'hF;
    @(posedge clk); penable <= 1; @(posedge clk);
    psel <= 0; penable <= 0; pwrite <= 0; paddr <= 0; pwdata <= 0; pstrb <= 4'h0;
  end
  endtask

  task apb_read(input [11:0] addr, output [31:0] data);
  begin
    @(posedge clk);
    psel <= 1; penable <= 0; pwrite <= 0; paddr <= addr; pstrb <= 4'h0;
    @(posedge clk); penable <= 1;
    @(posedge clk); data = prdata;
    psel <= 0; penable <= 0; paddr <= 0;
  end
  endtask

  // CSR addresses
  localparam CTRL      = 12'h004;
  localparam STATUS    = 12'h008;
  localparam CS_CTRL   = 12'h018;
  localparam CMD_CFG   = 12'h024;
  localparam CMD_OP    = 12'h028;
  localparam CMD_ADDR  = 12'h02C;
  localparam CMD_LEN   = 12'h030;
  localparam CMD_DMY   = 12'h034;
  localparam DMA_CFG   = 12'h038;
  localparam DMA_ADDR  = 12'h03C;
  localparam DMA_LEN   = 12'h040;
  localparam FIFO_RX   = 12'h048;

  // Helpers
  task ctrl_enable();       begin apb_write(CTRL, 32'h0000_0001); end endtask
  task ctrl_set_mode0();    begin apb_write(CTRL, 32'h0000_0001); end endtask
  task ctrl_trigger();      begin apb_write(CTRL, 32'h0000_0101); end endtask
  task ctrl_dma_enable();   begin apb_write(CTRL, 32'h0000_0041); end endtask
  task set_cs_auto();       begin apb_write(CS_CTRL, 32'h0000_0001); end endtask

  task cfg_cmd(
    input [1:0] lanes_cmd, input [1:0] lanes_addr, input [1:0] lanes_data,
    input [1:0] addr_bytes, input [3:0] dummies, input is_write,
    input [7:0] opcode, input [7:0] mode, input [31:0] addr, input [31:0] len
  );
    reg [31:0] cfg, op;
  begin
    cfg = {19'd0, is_write, dummies[3:0], addr_bytes[1:0], lanes_data[1:0], lanes_addr[1:0], lanes_cmd[1:0]};
    op  = {8'd0, mode, opcode};
    apb_write(CMD_CFG, cfg);
    apb_write(CMD_OP,  op);
    apb_write(CMD_ADDR, addr);
    apb_write(CMD_LEN,  len);
  end
  endtask

  task cfg_dma(input [3:0] burst_words, input dir_read_to_mem, input incr, input [31:0] addr, input [31:0] len);
    reg [31:0] d;
  begin
    d = {26'd0, incr, dir_read_to_mem, burst_words[3:0]};
    apb_write(DMA_CFG, d);
    apb_write(DMA_ADDR, addr);
    apb_write(DMA_LEN,  len);
  end
  endtask

  // Wait until controller busy bit clears
  task wait_idle();
    reg [31:0] stat; integer t;
  begin
    for (t=0;t<5000;t=t+1) begin
      apb_read(STATUS, stat);
      if (!stat[3]) disable wait_idle;
      @(posedge clk);
    end
  end endtask

  // Poll flash WIP using RDSR (0x05) until it clears; drain RX FIFO words
  task wait_flash_ready();
    reg [31:0] stat, fstat, sreg; integer t;
  begin
    begin : wip_poll
      for (t = 0; t < 200000; t = t + 1) begin
        cfg_cmd(2'b00,2'b00,2'b00, 2'b00, 4'd0, 1'b0, 8'h05, 8'h00, 32'h0, 32'd1);
        ctrl_trigger();
        // wait RX to have data
        begin : wait_rx
          integer u;
          for (u = 0; u < 80; u = u + 1) begin
            apb_read(12'h04C, fstat); if (fstat[7:4] != 4'd0) disable wait_rx; @(posedge clk);
          end
        end
        // Pop RX word via CSR timing (1-cycle latency)
        @(posedge clk); psel<=1; penable<=0; pwrite<=0; paddr<=FIFO_RX; pstrb<=0;
        @(posedge clk); penable<=1; @(posedge clk); sreg<=prdata; psel<=0; penable<=0; paddr<=0;
        if (!sreg[8]) disable wip_poll; // WIP bit cleared
        @(posedge clk);
      end
    end
    // ensure controller idle
    wait_idle();
  end endtask

  integer i;
  initial begin
    // Reset
    clk=0; resetn=0; psel=0; penable=0; pwrite=0; paddr=0; pwdata=0; pstrb=0;
    repeat(10) @(posedge clk);
    resetn=1;
    // Power-up wait (~0.9 ms) to mimic tVSL
    #900_000;

    // Basic setup
    ctrl_enable();
    // slow down SCLK for model stability
    apb_write(12'h014, 32'h00000007); // CLK_DIV
    ctrl_set_mode0();
    set_cs_auto();
    // Extra CS delay for robustness
    apb_write(CS_CTRL, 32'h0000_0019); // cs_auto=1, cs_delay=3

    // Ensure sector erased at 0x000000
    cfg_cmd(2'b00,2'b00,2'b00, 2'b00, 4'd0, 1'b0, 8'h06, 8'h00, 32'h0, 32'd0); // WREN
    ctrl_trigger();
    wait_idle();
    repeat (200) @(posedge clk);
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd0, 1'b0, 8'h20, 8'h00, 32'h0, 32'd0); // SE 4KB
    ctrl_trigger();
    wait_flash_ready();

    // Multiword DMA read: 64 bytes from flash @0x000000 to mem[0..15]
    cfg_dma(4'd8, 1'b1, 1'b1, 32'h0000_0000, 32'd64);
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd8, 1'b0, 8'h0B, 8'h00, 32'h0000_0000, 32'd64);
    ctrl_trigger();
    ctrl_dma_enable();
    // Allow time, then wait idle
    repeat (4000) @(posedge clk);
    wait_idle();

    // Verify memory contents are 0xFFFF_FFFF in first 16 words
    for (i=0; i<16; i=i+1) begin
      if (mem.mem[i] !== 32'hFFFF_FFFF) begin
        $fatal(1, "DMA burst read mismatch at mem[%0d] = %08h", i, mem.mem[i]);
      end
    end

    $display("cmd_dma_burst_tb: PASS (test passed)");
    $finish;
  end
endmodule

