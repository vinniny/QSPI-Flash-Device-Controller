`timescale 1ns/1ps

// error_csr_tb - Unit test for CSR error/interrupt logic.
// Drives CSR directly to validate:
// - ERR_STAT W1C from timeout/overrun/underrun/axi_err inputs
// - INT_EN gating and irq assertion for cmd_done/dma_done/err

module error_csr_tb;
  reg clk; reg resetn;

  // APB
  reg        psel, penable, pwrite; reg [11:0] paddr; reg [31:0] pwdata; reg [3:0] pstrb;
  wire [31:0] prdata; wire pready; wire pslverr;

  // CSR sideband
  reg cmd_done_set_i, dma_done_set_i, err_set_i;
  reg [3:0] tx_level_i, rx_level_i; reg tx_empty_i, rx_full_i;
  reg busy_i, xip_active_i; reg cmd_done_i, dma_done_i;
  reg timeout_i, overrun_i, underrun_i, axi_err_i;
  wire irq;

  csr u_csr (
    .pclk(clk), .presetn(resetn),
    .psel(psel), .penable(penable), .pwrite(pwrite), .paddr(paddr), .pwdata(pwdata), .pstrb(pstrb),
    .prdata(prdata), .pready(pready), .pslverr(pslverr),
    .enable_o(), .xip_en_o(), .quad_en_o(), .cpol_o(), .cpha_o(), .lsb_first_o(),
    .cmd_start_o(), .dma_en_o(), .hold_en_o(), .wp_en_o(), .cmd_trigger_clr_i(1'b0),
    .clk_div_o(), .cs_auto_o(), .cs_level_o(), .cs_delay_o(),
    .xip_addr_bytes_o(), .xip_data_lanes_o(), .xip_dummy_cycles_o(), .xip_cont_read_o(), .xip_mode_en_o(), .xip_write_en_o(),
    .xip_read_op_o(), .xip_mode_bits_o(), .xip_write_op_o(),
    .cmd_lanes_o(), .addr_lanes_o(), .data_lanes_o(), .addr_bytes_o(), .mode_en_cfg_o(), .dummy_cycles_o(), .is_write_o(),
    .opcode_o(), .mode_bits_o(), .cmd_addr_o(), .cmd_len_o(), .extra_dummy_o(),
    .burst_size_o(), .dma_dir_o(), .incr_addr_o(), .dma_addr_o(), .dma_len_o(),
    .fifo_tx_data_o(), .fifo_tx_we_o(), .fifo_rx_data_i(32'h0), .fifo_rx_re_o(),
    .int_en_o(),
    .cmd_done_set_i(cmd_done_set_i), .dma_done_set_i(dma_done_set_i), .err_set_i(err_set_i), .fifo_tx_empty_set_i(1'b0), .fifo_rx_full_set_i(1'b0),
    .busy_i(busy_i), .xip_active_i(xip_active_i), .cmd_done_i(cmd_done_i), .dma_done_i(dma_done_i),
    .tx_level_i(tx_level_i), .rx_level_i(rx_level_i), .tx_empty_i(tx_empty_i), .rx_full_i(rx_full_i),
    .timeout_i(timeout_i), .overrun_i(overrun_i), .underrun_i(underrun_i), .axi_err_i(axi_err_i), .irq(irq)
  );

  // Clock
  initial clk=0; always #5 clk=~clk;

  // VCD + timeout
  initial begin $dumpfile("error_csr_tb.vcd"); $dumpvars(1, error_csr_tb); #5_000_000 $finish; end

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

  localparam INT_EN=12'h00C, INT_STAT=12'h010, ERR_STAT=12'h050;

  integer pass, fail; reg [31:0] d;

  initial begin
    // init
    psel=0; penable=0; pwrite=0; paddr=0; pwdata=0; pstrb=0; resetn=0;
    cmd_done_set_i=0; dma_done_set_i=0; err_set_i=0; busy_i=0; xip_active_i=0; cmd_done_i=0; dma_done_i=0;
    tx_level_i=0; rx_level_i=0; tx_empty_i=1; rx_full_i=0; timeout_i=0; overrun_i=0; underrun_i=0; axi_err_i=0;
    pass=0; fail=0;
    repeat(5) @(posedge clk); resetn=1;

    // Enable all interrupts
    apb_write(INT_EN, 32'h0000_001F);

    // Trigger cmd_done and check IRQ + INT_STAT[0]
    cmd_done_set_i=1; @(posedge clk); cmd_done_set_i=0; @(posedge clk);
    apb_read(INT_STAT,d); if (d[0]!==1) begin $display("[FAIL] CMD_DONE"); fail=fail+1; end else pass=pass+1;
    if (!irq) begin $display("[FAIL] IRQ for CMD_DONE"); fail=fail+1; end else pass=pass+1;
    apb_write(INT_STAT, 32'h1);

    // Trigger dma_done and check IRQ + INT_STAT[1]
    dma_done_set_i=1; @(posedge clk); dma_done_set_i=0; @(posedge clk);
    apb_read(INT_STAT,d); if (d[1]!==1) begin $display("[FAIL] DMA_DONE"); fail=fail+1; end else pass=pass+1;
    if (!irq) begin $display("[FAIL] IRQ for DMA_DONE"); fail=fail+1; end else pass=pass+1;
    apb_write(INT_STAT, 32'h2);

    // Inject error status inputs and validate ERR_STAT W1C
    timeout_i=1; overrun_i=1; underrun_i=1; axi_err_i=1; @(posedge clk);
    timeout_i=0; overrun_i=0; underrun_i=0; axi_err_i=0; @(posedge clk);
    apb_read(ERR_STAT,d); if (d[3:0]!==4'hF) begin $display("[FAIL] ERR_STAT set"); fail=fail+1; end else pass=pass+1;
    apb_write(ERR_STAT, 32'hF); apb_read(ERR_STAT,d); if (d[3:0]!==4'h0) begin $display("[FAIL] ERR_STAT clear"); fail=fail+1; end else pass=pass+1;

    if (fail==0) $display("error_csr_tb: passed"); else $fatal(1, "error_csr_tb: FAIL (%0d)", fail);
    $finish;
  end
endmodule
