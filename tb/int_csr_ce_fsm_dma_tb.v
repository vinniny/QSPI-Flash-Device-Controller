`timescale 1ns/1ps

// Integration step 3: CSR + CE + FIFOs + FSM + DMA + AXI RAM
// Performs a 0x0B fast read (8 dummy cycles) of 4 bytes with DMA into AXI RAM[0]
module int_csr_ce_fsm_dma_tb;
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

  // CSR outputs/minimal wires
  wire        enable_w, xip_en_w, quad_en_w, cpol_w, cpha_w, cmd_start_w, dma_en_w, mode_en_w;
  wire [2:0]  clk_div_w; wire cs_auto_w;
  wire [1:0]  cmd_lanes_w, addr_lanes_w, data_lanes_w, addr_bytes_w; wire [3:0] dummy_cycles_w;
  wire        is_write_w; wire [7:0] opcode_w, mode_bits_w; wire [31:0] cmd_addr_w, cmd_len_w; wire [7:0] extra_dummy_w;
  wire [3:0]  burst_size_w; wire dma_dir_w; wire incr_addr_w; wire [31:0] dma_addr_w; wire [31:0] dma_len_w;
  wire        cmd_trigger_clr_w;

  // FIFOs
  wire [31:0] fifo_tx_data_csr_w; wire fifo_tx_we_csr_w; wire [31:0] fifo_tx_rd_data_w; wire fifo_tx_rd_en_w;
  wire fifo_tx_full_w, fifo_tx_empty_w; wire [4:0] fifo_tx_level_w;
  wire fifo_rx_re_csr_w, fifo_rx_re_dma_w; wire [31:0] fifo_rx_rd_data_w; wire fifo_rx_full_w, fifo_rx_empty_w; wire [4:0] fifo_rx_level_w;

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
    .burst_size_o(burst_size_w), .dma_dir_o(dma_dir_w), .incr_addr_o(incr_addr_w), .dma_addr_o(dma_addr_w), .dma_len_o(dma_len_w),
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

  // DMA engine and AXI RAM
  dma_engine #(.ADDR_WIDTH(32), .TX_FIFO_DEPTH(16), .LEVEL_WIDTH(5)) u_dma (
    .clk(clk), .resetn(resetn), .dma_en_i(dma_en_w), .dma_dir_i(dma_dir_w), .burst_size_i(burst_size_w), .incr_addr_i(incr_addr_w),
    .dma_addr_i(dma_addr_w), .dma_len_i(dma_len_w),
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

  // Clock/VCD
  initial clk=0; always #5 clk=~clk; initial resetn=0;
  initial begin $dumpfile("int_csr_ce_fsm_dma_tb.vcd"); $dumpvars(0, int_csr_ce_fsm_dma_tb); end

  // APB helpers
  task apb_write(input [11:0] addr, input [31:0] data);
  begin @(posedge clk); psel<=1;penable<=0;pwrite<=1;paddr<=addr;pwdata<=data;pstrb<=4'hF; @(posedge clk); penable<=1; @(posedge clk);
        psel<=0;penable<=0;pwrite<=0;paddr<=0;pwdata<=0;pstrb<=0; end endtask

  // CSR addrs
  localparam CTRL=12'h004, CLKDIV=12'h014, CS_CTRL=12'h018, CMD_CFG=12'h024, CMD_OP=12'h028, CMD_ADDR=12'h02C, CMD_LEN=12'h030,
             DMA_CFG=12'h038, DMA_ADDR=12'h03C, DMA_LEN=12'h040;

  integer i;
  initial begin
    psel=0;penable=0;pwrite=0;paddr=0;pwdata=0;pstrb=0; repeat(10) @(posedge clk); resetn=1;
    // Mode0, slow, CS auto
    apb_write(CTRL,   32'h0000_0001);
    apb_write(CLKDIV, 32'h0000_0004);
    apb_write(CS_CTRL,32'h0000_0001);
    // DMA: dir=1 (RX->mem), burst=1, incr=1, addr=0, len=4
    apb_write(DMA_CFG, 32'h0000_0031);
    apb_write(DMA_ADDR,32'h0000_0000);
    apb_write(DMA_LEN, 32'h0000_0004);
    // Fast Read (0x0B) with 8 dummy cycles
    apb_write(CMD_CFG, 32'h0000_1840); // 8 dummy (bits 11:8 = 8), addr_bytes=01
    apb_write(CMD_OP,  32'h0000_000B);
    apb_write(CMD_ADDR,32'h0000_0000);
    apb_write(CMD_LEN, 32'h0000_0004);
    // Enable DMA and trigger in the same write to avoid busy gating
    apb_write(CTRL, 32'h0000_0141);
    // wait for DMA done
    for (i=0;i<20000 && !dma_done_set_w; i=i+1) @(posedge clk);
    if (!dma_done_set_w) $fatal(1, "DMA did not complete");
    // Allow memory write response to settle
    repeat(10) @(posedge clk);
    if (u_ram.mem[0] !== 32'h7FFF_FFFF && u_ram.mem[0] !== 32'hFFFF_FFFF)
      $fatal(1, "DMA read mismatch: %h", u_ram.mem[0]);
    $display("CSR+CE+FSM+DMA integration test passed");
    $finish;
  end

  // Global timeout to prevent stalls
  initial begin
    #1_000_000; // 1 ms cutoff
    $display("[int_csr_ce_fsm_dma_tb] Global timeout reached — finishing.");
    $finish;
  end
endmodule
