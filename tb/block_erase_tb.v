`timescale 1ns/1ps

// block_erase_tb - Exercise 64KB block erase (0xD8) at address 0
module block_erase_tb;
  // Clock/reset
  reg clk; reg resetn;

  // APB
  reg        psel; reg penable; reg pwrite; reg [11:0] paddr; reg [31:0] pwdata; reg [3:0] pstrb;
  wire [31:0] prdata; wire pready; wire pslverr;

  // AXI master (DMA) - unused but must be connected
  wire [31:0] m_awaddr; wire m_awvalid; wire m_awready;
  wire [31:0] m_wdata;  wire m_wvalid; wire [3:0] m_wstrb; wire m_wready;
  wire [1:0]  m_bresp;  wire m_bvalid; wire m_bready;
  wire [31:0] m_araddr; wire m_arvalid; wire m_arready;
  wire [31:0] m_rdata;  wire [1:0] m_rresp;  wire m_rvalid; wire m_rready;

  // QSPI pads
  wire        sclk; wire cs_n; wire [3:0] io;

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
    .s_axi_awaddr(32'h0), .s_axi_awvalid(1'b0), .s_axi_awready(),
    .s_axi_wdata(32'h0), .s_axi_wstrb(4'h0), .s_axi_wvalid(1'b0), .s_axi_wready(),
    .s_axi_bresp(), .s_axi_bvalid(), .s_axi_bready(1'b0),
    .s_axi_araddr(), .s_axi_arvalid(), .s_axi_arready(),
    .s_axi_rdata(), .s_axi_rresp(), .s_axi_rvalid(), .s_axi_rready(1'b0),
    .sclk(sclk), .cs_n(cs_n), .io(io), .irq(irq)
  );

  // AXI RAM (dummy, unused)
  axi4_ram_slave mem (
    .clk(clk), .resetn(resetn),
    .awaddr(m_awaddr), .awvalid(m_awvalid), .awready(m_awready),
    .wdata(m_wdata), .wstrb(m_wstrb), .wvalid(m_wvalid), .wready(m_wready),
    .bresp(m_bresp), .bvalid(m_bvalid), .bready(m_bready),
    .araddr(m_araddr), .arvalid(m_arvalid), .arready(m_arready),
    .rdata(m_rdata), .rresp(m_rresp), .rvalid(m_rvalid), .rready(m_rready)
  );

  // QSPI device model
  qspi_device flash (
    .qspi_sclk(sclk), .qspi_cs_n(cs_n), .qspi_io0(io[0]), .qspi_io1(io[1]), .qspi_io2(io[2]), .qspi_io3(io[3])
  );

  // Clock
  initial clk=0; always #5 clk=~clk;

  // VCD and timeout
  initial begin $dumpfile("block_erase_tb.vcd"); $dumpvars(1, block_erase_tb); end
  initial begin #60_000_000; $display("[block_erase_tb] Global timeout reached — finishing."); $finish; end

  // CSR addresses
  localparam CTRL=12'h004, STATUS=12'h008, CLK_DIV=12'h014, CS_CTRL=12'h018,
             CMD_CFG=12'h024, CMD_OP=12'h028, CMD_ADDR=12'h02C, CMD_LEN=12'h030,
             FIFO_RX=12'h048, FIFO_STAT=12'h04C;

  // Helpers
  task apb_write(input [11:0] addr, input [31:0] data);
  begin @(posedge clk); psel<=1; penable<=0; pwrite<=1; paddr<=addr; pwdata<=data; pstrb<=4'hF;
        @(posedge clk); penable<=1; @(posedge clk);
        psel<=0; penable<=0; pwrite<=0; paddr<=0; pwdata<=0; pstrb<=0; end endtask
  task apb_read(input [11:0] addr, output [31:0] data);
  begin @(posedge clk); psel<=1; penable<=0; pwrite<=0; paddr<=addr; pstrb<=4'h0;
        @(posedge clk); penable<=1; @(posedge clk); data=prdata;
        psel<=0; penable<=0; paddr<=0; pstrb<=0; end endtask
  task ctrl_enable(); begin apb_write(CTRL, 32'h0000_0001); end endtask
  task ctrl_trigger(); begin apb_write(CTRL, 32'h0000_0101); end endtask
  task wait_cmd_done();
    reg [31:0] s;
  begin
    // wait busy assert
    repeat (2000) begin apb_read(STATUS,s); if (s[3]) disable wait_cmd_done; @(posedge clk); end
    // wait busy deassert
    repeat (2000) begin apb_read(STATUS,s); if (!s[3]) disable wait_cmd_done; @(posedge clk); end
  end endtask
  task pop_rx(output [31:0] d);
  begin
    @(posedge clk); psel<=1; penable<=0; pwrite<=0; paddr<=FIFO_RX; pstrb<=4'h0;
    @(posedge clk); penable<=1;
    @(posedge clk); // allow CSR to capture FIFO pop
    @(posedge clk); d=prdata;
    psel<=0; penable<=0; paddr<=0;
  end endtask

  integer i; reg [31:0] d;
  initial begin
    // Reset
    psel=0; penable=0; pwrite=0; paddr=0; pwdata=0; pstrb=0; resetn=0;
    repeat (8) @(posedge clk); resetn=1;
    ctrl_enable();
    apb_write(CLK_DIV, 32'h0000_0007); // slow down SCLK
    apb_write(CS_CTRL, 32'h0000_0019); // cs_auto=1, delay

    // 1) WREN
    apb_write(CMD_CFG, 32'h0000_0000);
    apb_write(CMD_OP,  32'h0000_0006);
    apb_write(CMD_LEN, 32'h0000_0000);
    ctrl_trigger(); wait_cmd_done(); repeat(400) @(posedge clk);

    // 2) Block Erase 0xD8 @0
    apb_write(CMD_CFG, 32'h0000_0040); // addr_bytes=01 (3B)
    apb_write(CMD_OP,  32'h0000_00D8);
    apb_write(CMD_ADDR,32'h0000_0000);
    apb_write(CMD_LEN, 32'h0000_0000);
    ctrl_trigger();
    // Poll WIP via RDSR until clear
    begin : wip_poll
      for (i=0; i<200000; i=i+1) begin
        apb_write(CMD_CFG, 32'h0000_0000);
        apb_write(CMD_OP,  32'h0000_0005);
        apb_write(CMD_LEN, 32'h0000_0001);
        ctrl_trigger();
        // wait for word
        begin : wait_rx
          integer u; reg [31:0] f;
          for (u=0; u<80; u=u+1) begin apb_read(FIFO_STAT,f); if (f[7:4]!=4'd0) disable wait_rx; @(posedge clk); end
        end
        pop_rx(d);
        if (!d[8]) disable wip_poll;
        @(posedge clk);
      end
    end

    // 3) Small idle then Read 0x03 @0, expect 0xFFFF_FFFF
    repeat (50) @(posedge clk);
    apb_write(CMD_CFG, 32'h0000_0040); // addr_bytes=01 (3B)
    apb_write(CMD_OP,  32'h0000_0003);
    apb_write(CMD_ADDR,32'h0000_0000);
    apb_write(CMD_LEN, 32'h0000_0004);
    ctrl_trigger();
    begin : wait_rx2
      for (i=0; i<4000; i=i+1) begin apb_read(FIFO_STAT,d); if (d[7:4]!=4'd0) disable wait_rx2; @(posedge clk); end
    end
    pop_rx(d);
    if (d !== 32'hFFFF_FFFF) $fatal(1, "Block erase readback mismatch: %08h", d);
    $display("block_erase_tb: PASS");
    $finish;
  end
endmodule
