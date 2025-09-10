`timescale 1ns/1ps

// cmd_dma_erase_burst_tb - Program 64B via DMA, erase sector, then DMA-read back 64B and verify 0xFFFF_FFFF
module cmd_dma_erase_burst_tb;
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
    $dumpfile("cmd_dma_erase_burst_tb.vcd");
    $dumpvars(1, cmd_dma_erase_burst_tb);
    #60_000_000 $display("[cmd_dma_erase_burst_tb] Global timeout reached — finishing.");
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
  localparam CLK_DIV   = 12'h014;
  localparam CS_CTRL   = 12'h018;
  localparam CMD_CFG   = 12'h024;
  localparam CMD_OP    = 12'h028;
  localparam CMD_ADDR  = 12'h02C;
  localparam CMD_LEN   = 12'h030;
  localparam DMA_CFG   = 12'h038;
  localparam DMA_ADDR  = 12'h03C;
  localparam DMA_LEN   = 12'h040;
  localparam FIFO_RX   = 12'h048;
  localparam FIFO_STAT = 12'h04C;

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

  // Simple RX FIFO pop (CSR one-cycle latency)
  task pop_rx(output [31:0] d);
  begin
    @(posedge clk); psel<=1; penable<=0; pwrite<=0; paddr<=FIFO_RX; pstrb<=0;
    @(posedge clk); penable<=1; @(posedge clk); d<=prdata; psel<=0; penable<=0; paddr<=0;
  end endtask

  // Wait controller idle
  task wait_idle();
    reg [31:0] stat; integer t;
  begin for (t=0;t<5000;t=t+1) begin apb_read(STATUS, stat); if (!stat[3]) disable wait_idle; @(posedge clk); end end endtask

  // Poll flash WIP until clear
  task wait_flash_ready();
    reg [31:0] fstat, sreg; integer t;
  begin
    begin : wip_poll
      for (t=0;t<200000;t=t+1) begin
        cfg_cmd(2'b00,2'b00,2'b00, 2'b00, 4'd0, 1'b0, 8'h05, 8'h00, 32'h0, 32'd1);
        ctrl_trigger();
        begin : wait_rx
          integer u; for (u=0;u<80;u=u+1) begin apb_read(FIFO_STAT, fstat); if (fstat[7:4]!=4'd0) disable wait_rx; @(posedge clk); end end
        pop_rx(sreg);
        if (!sreg[8]) disable wip_poll; // WIP cleared
        @(posedge clk);
      end
    end
    wait_idle();
  end endtask

  // Drain RX FIFO (pop via CSR) until empty
  task drain_rx();
    reg [31:0] fstat, dummy; integer k;
  begin
    for (k=0;k<64;k=k+1) begin
      apb_read(FIFO_STAT, fstat);
      if (fstat[7:4]==4'd0) disable drain_rx;
      pop_rx(dummy);
    end
  end endtask

  integer i;
  initial begin
    // Reset and power-up wait
    clk=0; resetn=0; psel=0; penable=0; pwrite=0; paddr=0; pwdata=0; pstrb=0;
    repeat(10) @(posedge clk); resetn=1; #900_000;

    // Basic setup
    ctrl_enable();
    apb_write(CLK_DIV, 32'h0000_0007); // slow SCLK
    ctrl_set_mode0();
    set_cs_auto();
    apb_write(CS_CTRL, 32'h0000_0019); // cs_auto=1, cs_delay=3

    // Prepare source data in AXI RAM at base 0x0000_0040 (mem[16])
    for (i=0;i<16;i=i+1) begin
      mem.mem[16+i] = 32'h55AA_0000 + i;
    end

    // WREN and verify via RDSR if needed
    cfg_cmd(2'b00,2'b00,2'b00, 2'b00, 4'd0, 1'b0, 8'h06, 8'h00, 32'h0, 32'd0);
    ctrl_trigger(); wait_idle(); repeat (200) @(posedge clk);

    // DMA PROGRAM 64B to flash @0x0
    cfg_dma(4'd8, 1'b0, 1'b1, 32'h0000_0040, 32'd64);
    ctrl_dma_enable(); repeat (50) @(posedge clk);
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd0, 1'b1, 8'h02, 8'h00, 32'h0000_0000, 32'd64);
    ctrl_trigger();
    wait_flash_ready();

    // WREN before ERASE and verify WEL via RDSR; retry if needed
    cfg_cmd(2'b00,2'b00,2'b00, 2'b00, 4'd0, 1'b0, 8'h06, 8'h00, 32'h0, 32'd0);
    ctrl_trigger(); wait_idle(); repeat (200) @(posedge clk);
    begin : ensure_wel_erase
      integer t; reg [31:0] dword; reg wel_ok; wel_ok = 1'b0;
      for (t = 0; t < 400; t = t + 1) begin
        cfg_cmd(2'b00,2'b00,2'b00, 2'b00, 4'd0, 1'b0, 8'h05, 8'h00, 32'h0, 32'd1);
        ctrl_trigger();
        begin : wait_rx_wel_e
          reg [31:0] fstat; integer u; for (u=0;u<80;u=u+1) begin apb_read(FIFO_STAT, fstat); if (fstat[7:4]!=4'd0) disable wait_rx_wel_e; @(posedge clk); end end
        pop_rx(dword);
        if (dword[9]) begin
          wait_idle();
          wel_ok = 1'b1; disable ensure_wel_erase;
        end
        if (t==200) begin
          cfg_cmd(2'b00,2'b00,2'b00, 2'b00, 4'd0, 1'b0, 8'h06, 8'h00, 32'h0, 32'd0);
          ctrl_trigger(); wait_idle(); repeat (400) @(posedge clk);
        end
        @(posedge clk);
      end
      if (!wel_ok) $display("[WARN] WEL not observed before ERASE; proceeding");
    end
    repeat (80) @(posedge clk); #500;

    // Sector erase @0x0
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd0, 1'b1, 8'h20, 8'h00, 32'h0000_0000, 32'd0);
    ctrl_trigger();
    wait_flash_ready();

    // DMA READ back 64B from @0x0 to mem[64..79]
    // Ensure RX FIFO is empty after status polling
    drain_rx();
    cfg_dma(4'd8, 1'b1, 1'b1, 32'h0000_0100, 32'd64);
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd8, 1'b0, 8'h0B, 8'h00, 32'h0000_0000, 32'd64);
    ctrl_trigger();
    ctrl_dma_enable();
    repeat (4000) @(posedge clk);
    wait_idle();

    // Verify mem[64..79] are 0xFFFF_FFFF
    for (i=0;i<16;i=i+1) begin
      if (mem.mem[64+i] !== 32'hFFFF_FFFF) $fatal(1, "Erase readback mismatch mem[%0d]=%08h", 64+i, mem.mem[64+i]);
    end

    $display("cmd_dma_erase_burst_tb: PASS (test passed)");
    $finish;
  end
endmodule
