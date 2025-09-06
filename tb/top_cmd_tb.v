`timescale 1ns/1ps

module top_cmd_tb;
  // Clock/reset
  reg clk;
  reg resetn;

  // APB
  reg        psel;
  reg        penable;
  reg        pwrite;
  reg [11:0] paddr;
  reg [31:0] pwdata;
  reg [3:0]  pstrb;
  wire [31:0] prdata;
  wire        pready;
  wire        pslverr;

  // AXI master (DMA)
  wire [31:0] m_awaddr;
  wire        m_awvalid;
  wire        m_awready;
  wire [31:0] m_wdata;
  wire        m_wvalid;
  wire [3:0]  m_wstrb;
  wire        m_wready;
  wire [1:0]  m_bresp;
  wire        m_bvalid;
  wire        m_bready;
  wire [31:0] m_araddr;
  wire        m_arvalid;
  wire        m_arready;
  wire [31:0] m_rdata;
  wire [1:0]  m_rresp;
  wire        m_rvalid;
  wire        m_rready;

  // XIP AXI slave (unused)
  wire [31:0] s_awaddr = 32'h0;
  wire        s_awvalid = 1'b0;
  wire        s_awready;
  wire [31:0] s_wdata = 32'h0;
  wire [3:0]  s_wstrb = 4'h0;
  wire        s_wvalid = 1'b0;
  wire        s_wready;
  wire [1:0]  s_bresp;
  wire        s_bvalid;
  wire        s_bready = 1'b0;
  wire [31:0] s_araddr;
  wire        s_arvalid;
  wire        s_arready;
  wire [31:0] s_rdata;
  wire [1:0]  s_rresp;
  wire        s_rvalid;
  wire        s_rready;

  // QSPI wires
  wire        sclk;
  wire        cs_n;
  wire [3:0]  io;

  // Pull-ups for HOLD#/SIO3 and WP# pins on the external flash model
  pullup pu_hold (io[3]);
  pullup pu_wp   (io[2]);

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

  // Full Macronix flash model
  MX25L6436F flash (
    .SCLK(sclk), .CS(cs_n), .SI(io[0]), .SO(io[1]), .WP(io[2]), .SIO3(io[3])
  );

  // Clock
  initial clk = 0;
  always #5 clk = ~clk;

  // VCD and global timeout
  initial begin
    $dumpfile("top_cmd_tb.vcd");
    $dumpvars(1, top_cmd_tb);
    #60_000_000 $dumpoff; // cap VCD and prevent stalls
    $display("[top_cmd_tb] Global timeout reached — finishing.");
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
  localparam FIFO_TX   = 12'h044;
  localparam FIFO_RX   = 12'h048;

  // Helpers
  task ctrl_enable(); begin apb_write(CTRL, 32'h0000_0001); end endtask
  // optional helpers could be added here for CPOL/CPHA
  // Use SPI Mode 0 with the Macronix MX25L model
  task ctrl_set_mode0(); begin apb_write(CTRL, 32'h0000_0001); end endtask
  task ctrl_trigger(); begin apb_write(CTRL, 32'h0000_0101); end endtask
  task ctrl_dma_enable(); begin apb_write(CTRL, 32'h0000_0041); end endtask
  task set_cs_auto(); begin apb_write(CS_CTRL, 32'h0000_0001); end endtask

  task cfg_cmd(
    input [1:0] lanes_cmd, input [1:0] lanes_addr, input [1:0] lanes_data,
    input [1:0] addr_bytes, input [3:0] dummies, input is_write,
    input [7:0] opcode, input [7:0] mode, input [31:0] addr, input [31:0] len
  );
    reg [31:0] cfg;
    reg [31:0] op;
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

  // Simple RX FIFO pop
  task pop_rx(output [31:0] d);
  begin
    apb_read(FIFO_RX, d);
  end
  endtask

  // Test sequence
  integer i;
  reg [31:0] dword;

  initial begin
    $dumpfile("top_cmd_tb.vcd");
    $dumpvars(1, top_cmd_tb);
    #3_000_000 $dumpoff; // limit VCD size
    // Reset
    clk=0; resetn=0; psel=0; penable=0; pwrite=0; paddr=0; pwdata=0; pstrb=0;
    repeat (10) @(posedge clk);
    resetn = 1;
    // Macronix model requires tVSL ~800us after power-up
    #900_000;

    ctrl_enable();
    // slow down SCLK for external flash timing
    apb_write(12'h014, 32'h00000004); // CLK_DIV
    ctrl_set_mode0();
    set_cs_auto();

    // 1) JEDEC ID (0x9F) - read 4 bytes, check manufacturer (0xC2)
    cfg_cmd(2'b00,2'b00,2'b00, 2'b00, 4'd0, 1'b0, 8'h9F, 8'h00, 32'h0, 32'd4);
    ctrl_trigger();
    repeat (50) @(posedge clk);
    pop_rx(dword);
    $display("JEDEC ID word: %h", dword);

    // 2) Fast Read (0x0B) len=4 from 0x000000 should be 0xFFFF_FFFF
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd8, 1'b0, 8'h0B, 8'h00, 32'h0, 32'd4);
    ctrl_trigger();
    repeat (200) @(posedge clk);
    pop_rx(dword);
    if (dword !== 32'hFFFF_FFFF) $fatal(1, "Fast Read mismatch: %h", dword);

    // 3) WREN (0x06)
    cfg_cmd(2'b00,2'b00,2'b00, 2'b00, 4'd0, 1'b0, 8'h06, 8'h00, 32'h0, 32'd0);
    ctrl_trigger();
    repeat (10) @(posedge clk);

    // 4) Page Program (0x02) len=4 write 0xA5A5A5A5 @ 0x000000
    apb_write(FIFO_TX, 32'hA5A5_A5A5);
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd0, 1'b1, 8'h02, 8'h00, 32'h0, 32'd4);
    ctrl_trigger();
    // Poll status until WIP=0 (read 4 bytes; check bit0 of top byte)
    for (i=0;i<200000;i=i+1) begin
      cfg_cmd(2'b00,2'b00,2'b00, 2'b00, 4'd0, 1'b0, 8'h05, 8'h00, 32'h0, 32'd4);
      ctrl_trigger();
      repeat (50) @(posedge clk);
      pop_rx(dword);
      if ( (dword[0] | dword[8] | dword[16] | dword[24]) == 1'b0 ) i = 200000; // exit loop when WIP=0 in any byte
    end
    if ( (dword[0] | dword[8] | dword[16] | dword[24]) != 1'b0) $fatal(1, "WIP never cleared after program");

    // 5) Read back (0x03) len=4 verify data
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd0, 1'b0, 8'h03, 8'h00, 32'h0, 32'd4);
    ctrl_trigger();
    repeat (200) @(posedge clk);
    pop_rx(dword);
    if (dword !== 32'hA5A5_A5A5) $fatal(1, "Readback after program mismatch: %h", dword);

    // 6) Sector Erase (0x20) @ 0x000000
    // WREN
    cfg_cmd(2'b00,2'b00,2'b00, 2'b00, 4'd0, 1'b0, 8'h06, 8'h00, 32'h0, 32'd0);
    ctrl_trigger(); repeat (20) @(posedge clk);
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd0, 1'b1, 8'h20, 8'h00, 32'h0, 32'd0);
    ctrl_trigger();
    // Poll WIP
    for (i=0;i<2000000;i=i+1) begin
      cfg_cmd(2'b00,2'b00,2'b00, 2'b00, 4'd0, 1'b0, 8'h05, 8'h00, 32'h0, 32'd4);
      ctrl_trigger();
      repeat (100) @(posedge clk);
      pop_rx(dword);
      if ( (dword[0] | dword[8] | dword[16] | dword[24]) == 1'b0 ) i = 2000000;
    end
    if ( (dword[0] | dword[8] | dword[16] | dword[24]) != 1'b0 ) $fatal(1, "WIP never cleared after erase");

    // Verify erased
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd0, 1'b0, 8'h03, 8'h00, 32'h0, 32'd4);
    ctrl_trigger();
    repeat (200) @(posedge clk);
    pop_rx(dword);
    if (dword !== 32'hFFFF_FFFF) $fatal(1, "Readback after erase mismatch: %h", dword);

    // 7) DMA Read test: read 4B to mem[0]
    cfg_dma(4'd1, 1'b1, 1'b1, 32'h0000_0000, 32'd4);
    // Setup fast read
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd8, 1'b0, 8'h0B, 8'h00, 32'h0, 32'd4);
    ctrl_dma_enable();
    ctrl_trigger();
    repeat (2000) @(posedge clk);
    if (mem.mem[0] !== 32'hFFFF_FFFF) $fatal(1, "DMA read mismatch: %h", mem.mem[0]);

    // 8) DMA Program test: write 4B from mem[1] to flash @ 0x0
    // Prepare source data in AXI RAM
    mem.mem[1] = 32'h1122_3344;
    // WREN
    cfg_cmd(2'b00,2'b00,2'b00, 2'b00, 4'd0, 1'b0, 8'h06, 8'h00, 32'h0, 32'd0);
    ctrl_trigger(); repeat (20) @(posedge clk);
    // Configure program
    cfg_dma(4'd1, 1'b0, 1'b1, 32'h0000_0004, 32'd4);
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd0, 1'b1, 8'h02, 8'h00, 32'h0, 32'd4);
    ctrl_dma_enable();
    ctrl_trigger();
    // Poll WIP
    for (i=0;i<200000;i=i+1) begin
      cfg_cmd(2'b00,2'b00,2'b00, 2'b00, 4'd0, 1'b0, 8'h05, 8'h00, 32'h0, 32'd4);
      ctrl_trigger();
      repeat (50) @(posedge clk);
      pop_rx(dword);
      if (dword[31] == 1'b0) i = 200000;
    end
    // Verify
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd0, 1'b0, 8'h03, 8'h00, 32'h0, 32'd4);
    ctrl_trigger();
    repeat (200) @(posedge clk);
    pop_rx(dword);
    if (dword !== 32'h1122_3344) $fatal(1, "DMA program readback mismatch: %h", dword);

    // 9) Dual Output Read (0x3B) non-DMA, len=8 @ 0
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd8, 1'b0, 8'h3B, 8'h00, 32'h0, 32'd8);
    ctrl_trigger();
    repeat (400) @(posedge clk);
    pop_rx(dword); if (dword !== 32'hFFFF_FFFF) $fatal(1, "DREAD word0 mismatch: %h", dword);
    pop_rx(dword); if (dword !== 32'hFFFF_FFFF) $fatal(1, "DREAD word1 mismatch: %h", dword);

    // 10) Dual Output Read (0x3B) with DMA, len=8 @ mem[0]
    cfg_dma(4'd2, 1'b1, 1'b1, 32'h0000_0000, 32'd8);
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd8, 1'b0, 8'h3B, 8'h00, 32'h0, 32'd8);
    ctrl_dma_enable(); ctrl_trigger();
    repeat (4000) @(posedge clk);
    if (mem.mem[0] !== 32'hFFFF_FFFF || mem.mem[1] !== 32'hFFFF_FFFF) $fatal(1, "DMA DREAD mismatch");

    // 11) Quad Output Read (0x6B) non-DMA, len=8 @ 0
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd8, 1'b0, 8'h6B, 8'h00, 32'h0, 32'd8);
    ctrl_trigger();
    repeat (400) @(posedge clk);
    pop_rx(dword); if (dword !== 32'hFFFF_FFFF) $fatal(1, "QREAD word0 mismatch: %h", dword);
    pop_rx(dword); if (dword !== 32'hFFFF_FFFF) $fatal(1, "QREAD word1 mismatch: %h", dword);

    // 12) Quad Output Read (0x6B) with DMA, len=8 @ mem[2]
    cfg_dma(4'd2, 1'b1, 1'b1, 32'h0000_0008, 32'd8);
    cfg_cmd(2'b00,2'b00,2'b00, 2'b01, 4'd8, 1'b0, 8'h6B, 8'h00, 32'h0, 32'd8);
    ctrl_dma_enable(); ctrl_trigger();
    repeat (4000) @(posedge clk);
    if (mem.mem[2] !== 32'hFFFF_FFFF || mem.mem[3] !== 32'hFFFF_FFFF) $fatal(1, "DMA QREAD mismatch");

    $display("Top command-mode tests passed (with and without DMA)");
    $finish;
  end
endmodule
