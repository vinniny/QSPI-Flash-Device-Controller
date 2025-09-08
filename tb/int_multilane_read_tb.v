`timescale 1ns/1ps

// Integration: Multi-lane reads, with and without DMA
// Uses simplified qspi_device which returns 0xFF data for reads.
module int_multilane_read_tb;
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

  // QSPI pads
  wire sclk; wire cs_n; wire [3:0] io;

  // CSR <-> CE wires
  wire        enable_w, xip_en_w, quad_en_w, cpol_w, cpha_w, cmd_start_w, dma_en_w, mode_en_w;
  wire [2:0]  clk_div_w; wire cs_auto_w;
  wire [1:0]  cmd_lanes_w, addr_lanes_w, data_lanes_w, addr_bytes_w; wire [3:0] dummy_cycles_w;
  wire        is_write_w; wire [7:0] opcode_w, mode_bits_w; wire [31:0] cmd_addr_w, cmd_len_w; wire [7:0] extra_dummy_w;
  wire        cmd_trigger_clr_w;

  // FIFOs
  wire [31:0] fifo_tx_data_csr_w; wire fifo_tx_we_csr_w; wire [31:0] fifo_tx_rd_data_w; wire fifo_tx_rd_en_w;
  wire fifo_tx_full_w, fifo_tx_empty_w; wire [5:0] fifo_tx_level_w;
  wire fifo_rx_re_csr_w, fifo_rx_re_dma_w; wire [31:0] fifo_rx_rd_data_w; wire fifo_rx_full_w, fifo_rx_empty_w; wire [5:0] fifo_rx_level_w;

  // CE <-> FSM
  wire fsm_start_w, fsm_done_w; wire fsm_tx_ren_w; wire [31:0] fsm_rx_data_w; wire fsm_rx_wen_w;

  // DMA
  wire dma_done_set_w; wire dma_busy_w; wire dma_axi_err_w;

  // CSR
  csr u_csr (
    .pclk(clk), .presetn(resetn),
    .psel(psel), .penable(penable), .pwrite(pwrite), .paddr(paddr), .pwdata(pwdata), .pstrb(pstrb),
    .prdata(prdata), .pready(pready), .pslverr(pslverr),
    .enable_o(enable_w), .xip_en_o(xip_en_w), .quad_en_o(quad_en_w), .cpol_o(cpol_w), .cpha_o(cpha_w), .lsb_first_o(),
    .cmd_start_o(cmd_start_w), .dma_en_o(dma_en_w), .hold_en_o(), .wp_en_o(), .cmd_trigger_clr_i(cmd_trigger_clr_w),
    .clk_div_o(clk_div_w), .cs_auto_o(cs_auto_w), .cs_level_o(), .cs_delay_o(),
    .xip_addr_bytes_o(), .xip_data_lanes_o(), .xip_dummy_cycles_o(), .xip_cont_read_o(), .xip_mode_en_o(), .xip_write_en_o(),
    .xip_read_op_o(), .xip_mode_bits_o(), .xip_write_op_o(),
    .cmd_lanes_o(cmd_lanes_w), .addr_lanes_o(addr_lanes_w), .data_lanes_o(data_lanes_w), .addr_bytes_o(addr_bytes_w),
    .mode_en_cfg_o(mode_en_w), .dummy_cycles_o(dummy_cycles_w), .is_write_o(is_write_w), .opcode_o(opcode_w), .mode_bits_o(mode_bits_w),
    .cmd_addr_o(cmd_addr_w), .cmd_len_o(cmd_len_w), .extra_dummy_o(extra_dummy_w),
    .burst_size_o(), .dma_dir_o(), .incr_addr_o(), .dma_addr_o(), .dma_len_o(),
    .fifo_tx_data_o(fifo_tx_data_csr_w), .fifo_tx_we_o(fifo_tx_we_csr_w),
    .fifo_rx_data_i(fifo_rx_rd_data_w), .fifo_rx_re_o(fifo_rx_re_csr_w),
    .int_en_o(),
    .cmd_done_set_i(fsm_done_w), .dma_done_set_i(dma_done_set_w), .err_set_i(dma_axi_err_w), .fifo_tx_empty_set_i(1'b0), .fifo_rx_full_set_i(1'b0),
    .busy_i(dma_busy_w), .xip_active_i(1'b0), .cmd_done_i(1'b0), .dma_done_i(1'b0),
    .tx_level_i(fifo_tx_level_w[3:0]), .rx_level_i(fifo_rx_level_w[3:0]), .tx_empty_i(fifo_tx_empty_w), .rx_full_i(fifo_rx_full_w),
    .timeout_i(1'b0), .overrun_i(1'b0), .underrun_i(1'b0), .axi_err_i(dma_axi_err_w), .irq()
  );

  // CE
  cmd_engine #(.ADDR_WIDTH(32)) u_ce (
    .clk(clk), .resetn(resetn), .cmd_start_i(cmd_start_w), .cmd_trigger_clr_o(cmd_trigger_clr_w), .cmd_done_set_o(), .busy_o(),
    .cmd_lanes_i(cmd_lanes_w), .addr_lanes_i(addr_lanes_w), .data_lanes_i(data_lanes_w), .addr_bytes_i(addr_bytes_w),
    .mode_en_i(mode_en_w), .dummy_cycles_i(dummy_cycles_w), .extra_dummy_i(extra_dummy_w), .is_write_i(is_write_w),
    .opcode_i(opcode_w), .mode_bits_i(mode_bits_w), .cmd_addr_i(cmd_addr_w), .cmd_len_i(cmd_len_w),
    .quad_en_i(quad_en_w), .cs_auto_i(cs_auto_w), .xip_cont_read_i(1'b0), .clk_div_i(clk_div_w), .cpol_i(cpol_w), .cpha_i(cpha_w),
    .start_o(fsm_start_w), .done_i(fsm_done_w),
    .cmd_lanes_o(), .addr_lanes_o(), .data_lanes_o(), .addr_bytes_o(), .mode_en_o(), .dummy_cycles_o(), .dir_o(),
    .quad_en_o(), .cs_auto_o(), .xip_cont_read_o(), .opcode_o(), .mode_bits_o(), .addr_o(), .len_o(), .clk_div_o(), .cpol_o(), .cpha_o()
  );

  // FIFOs
  fifo_tx #(.WIDTH(32), .DEPTH(16)) u_ftx (
    .clk(clk), .resetn(resetn),
    .wr_en_i(fifo_tx_we_csr_w), .wr_data_i(fifo_tx_data_csr_w),
    .rd_en_i(fsm_tx_ren_w), .rd_data_o(fifo_tx_rd_data_w),
    .full_o(fifo_tx_full_w), .empty_o(fifo_tx_empty_w), .level_o(fifo_tx_level_w)
  );
  fifo_rx #(.WIDTH(32), .DEPTH(16)) u_frx (
    .clk(clk), .resetn(resetn),
    .wr_en_i(fsm_rx_wen_w), .wr_data_i(fsm_rx_data_w),
    .rd_en_i(fifo_rx_re_csr_w | fifo_rx_re_dma_w), .rd_data_o(fifo_rx_rd_data_w),
    .full_o(fifo_rx_full_w), .empty_o(fifo_rx_empty_w), .level_o(fifo_rx_level_w)
  );

  // FSM
  qspi_fsm u_fsm (
    .clk(clk), .resetn(resetn), .start(fsm_start_w), .done(fsm_done_w),
    .cmd_lanes_sel(cmd_lanes_w), .addr_lanes_sel(addr_lanes_w), .data_lanes_sel(data_lanes_w), .addr_bytes_sel(addr_bytes_w),
    .mode_en(1'b0), .dummy_cycles(dummy_cycles_w), .dir(1'b1), .quad_en(1'b0), .cs_auto(1'b1), .xip_cont_read(1'b0),
    .cmd_opcode(opcode_w), .mode_bits(8'h00), .addr(cmd_addr_w), .len_bytes(cmd_len_w),
    .clk_div({29'd0, clk_div_w}), .cpol(cpol_w), .cpha(cpha_w),
    .tx_data_fifo(fifo_tx_rd_data_w), .tx_empty(fifo_tx_empty_w), .tx_ren(fsm_tx_ren_w),
    .rx_data_fifo(fsm_rx_data_w), .rx_wen(fsm_rx_wen_w), .rx_full(fifo_rx_full_w),
    .sclk(sclk), .cs_n(cs_n), .io0(io[0]), .io1(io[1]), .io2(io[2]), .io3(io[3])
  );

  // DMA and AXI RAM
  dma_engine #(.ADDR_WIDTH(32), .TX_FIFO_DEPTH(16), .LEVEL_WIDTH(6)) u_dma (
    .clk(clk), .resetn(resetn), .dma_en_i(dma_en_w), .dma_dir_i(1'b1), .burst_size_i(4'd2), .incr_addr_i(1'b1),
    .dma_addr_i(32'h0000_0000), .dma_len_i(32'h0000_0008),
    .tx_level_i(fifo_tx_level_w), .fifo_tx_data_o(/*unused*/ ), .fifo_tx_we_o(/*unused*/),
    .rx_level_i(fifo_rx_level_w), .fifo_rx_data_i(fifo_rx_rd_data_w), .fifo_rx_re_o(fifo_rx_re_dma_w),
    .dma_done_set_o(dma_done_set_w), .axi_err_o(dma_axi_err_w), .busy_o(dma_busy_w),
    .awaddr_o(m_awaddr), .awvalid_o(m_awvalid), .awready_i(m_awready),
    .wdata_o(m_wdata), .wvalid_o(m_wvalid), .wstrb_o(m_wstrb), .wready_i(m_wready),
    .bvalid_i(m_bvalid), .bresp_i(m_bresp), .bready_o(m_bready),
    .araddr_o(m_araddr), .arvalid_o(m_arvalid), .arready_i(m_arready),
    .rdata_i(m_rdata), .rvalid_i(m_rvalid), .rresp_i(m_rresp), .rready_o(m_rready)
  );

  axi4_ram_slave u_ram (
    .clk(clk), .resetn(resetn),
    .awaddr(m_awaddr), .awvalid(m_awvalid), .awready(m_awready),
    .wdata(m_wdata), .wstrb(m_wstrb), .wvalid(m_wvalid), .wready(m_wready),
    .bresp(m_bresp), .bvalid(m_bvalid), .bready(m_bready),
    .araddr(m_araddr), .arvalid(m_arvalid), .arready(m_arready),
    .rdata(m_rdata), .rresp(m_rresp), .rvalid(m_rvalid), .rready(m_rready)
  );

  // Simple device
  qspi_device dev (
    .qspi_sclk(sclk), .qspi_cs_n(cs_n), .qspi_io0(io[0]), .qspi_io1(io[1]), .qspi_io2(io[2]), .qspi_io3(io[3])
  );

  // Clock/reset
  initial clk=0; always #5 clk=~clk; initial resetn=0;
  initial begin $dumpfile("int_multilane_read_tb.vcd"); $dumpvars(0, int_multilane_read_tb); end

  // APB helpers
  task apb_write(input [11:0] addr, input [31:0] data);
  begin @(posedge clk); psel<=1;penable<=0;pwrite<=1;paddr<=addr;pwdata<=data;pstrb<=4'hF; @(posedge clk); penable<=1; @(posedge clk);
        psel<=0;penable<=0;pwrite<=0;paddr<=0;pwdata<=0;pstrb<=0; end endtask
  task apb_read(input [11:0] addr, output [31:0] data);
  begin @(posedge clk); psel<=1;penable<=0;pwrite<=0;paddr<=addr; @(posedge clk); penable<=1; @(posedge clk); data=prdata; psel<=0; penable<=0; paddr<=0; end endtask

  // CSR addrs
  localparam CTRL=12'h004, CLKDIV=12'h014, CS_CTRL=12'h018, CMD_CFG=12'h024, CMD_OP=12'h028, CMD_ADDR=12'h02C, CMD_LEN=12'h030, FIFO_RX=12'h048;

  integer i; reg [31:0] d0, d1;
  initial begin
    // reset and bring-up
    psel=0;penable=0;pwrite=0;paddr=0;pwdata=0;pstrb=0; repeat(10) @(posedge clk); resetn=1;
    apb_write(CTRL,   32'h0000_0001); // enable, mode0
    apb_write(CLKDIV, 32'h0000_0004); // slow SCLK
    apb_write(CS_CTRL,32'h0000_0001); // CS auto

    // 1) Dual Output Read (0x3B) non-DMA, len=8 at 0x0
    apb_write(CMD_CFG, {19'd0,1'b0,4'd8,2'b01,2'b00,2'b00}); // lanes=1-1-1 (FSM decodes 3B to dual on data), addr_bytes=01
    apb_write(CMD_OP,  32'h0000_003B);
    apb_write(CMD_ADDR,32'h0000_0000);
    apb_write(CMD_LEN, 32'h0000_0008);
    apb_write(CTRL,    32'h0000_0101); // trigger
    repeat (400) @(posedge clk);
    apb_read(FIFO_RX, d0); apb_read(FIFO_RX, d1);
    if (d0 !== 32'hFFFF_FFFF || d1 !== 32'hFFFF_FFFF)
      $fatal(1, "DREAD mismatch: %h %h", d0, d1);

    // 2) Quad Output Read (0x6B) with DMA, len=8 to mem[2*4]
    apb_write(12'h038, 32'h0000_0032); // DMA_CFG: burst=2, dir=1, incr=1
    apb_write(12'h03C, 32'h0000_0000); // DMA_ADDR
    apb_write(12'h040, 32'h0000_0008); // DMA_LEN
    apb_write(CMD_CFG, {19'd0,1'b0,4'd8,2'b01,2'b00,2'b00});
    apb_write(CMD_OP,  32'h0000_006B);
    apb_write(CMD_ADDR,32'h0000_0000);
    apb_write(CMD_LEN, 32'h0000_0008);
    apb_write(CTRL,    32'h0000_0141); // enable DMA + trigger
    for (i=0;i<4000 && !dma_done_set_w; i=i+1) @(posedge clk);
    if (!dma_done_set_w) $fatal(1, "DMA not done");
    // settle
    repeat (10) @(posedge clk);
    if (u_ram.mem[0] !== 32'hFFFF_FFFF || u_ram.mem[1] !== 32'hFFFF_FFFF)
      $fatal(1, "DMA QREAD mismatch: %h %h", u_ram.mem[0], u_ram.mem[1]);

    $display("Multi-lane read tests passed (PIO + DMA)");
    $finish;
  end

  // Global timeout guard
  initial begin
    #5_000_000; // 5 ms cutoff
    $display("[int_multilane_read_tb] Global timeout reached — finishing.");
    $finish;
  end
endmodule
