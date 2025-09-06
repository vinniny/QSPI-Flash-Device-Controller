// qspi_controller.v - Top-level QSPI controller integrating CSR, engines and IO
//
// Provides APB slave for CSR access, AXI master for DMA, AXI slave for XIP,
// and QSPI pad connections. Modes are mutually exclusive between Command
// (with optional DMA) and XIP. Aggregates busy and interrupt signals.

module qspi_controller #(
  parameter integer ADDR_WIDTH      = 32,
  parameter integer APB_ADDR_WIDTH  = 12,
  parameter integer APB_WINDOW_LSB  = 12,
  parameter integer HAS_PSTRB       = 0,
  parameter integer HAS_WP          = 0,
  parameter integer FIFO_DEPTH      = 16
) (
  // Clock and reset
  input  wire                     clk,
  input  wire                     resetn,

  // APB slave interface
  input  wire                     psel,
  input  wire                     penable,
  input  wire                     pwrite,
  input  wire [APB_ADDR_WIDTH-1:0] paddr,
  input  wire [31:0]              pwdata,
  input  wire [3:0]               pstrb,
  output wire [31:0]              prdata,
  output wire                     pready,
  output wire                     pslverr,

  // AXI4-Lite master interface for DMA engine
  output wire [ADDR_WIDTH-1:0]    m_axi_awaddr,
  output wire                     m_axi_awvalid,
  input  wire                     m_axi_awready,
  output wire [31:0]              m_axi_wdata,
  output wire                     m_axi_wvalid,
  output wire [3:0]               m_axi_wstrb,
  input  wire                     m_axi_wready,
  input  wire                     m_axi_bvalid,
  input  wire [1:0]               m_axi_bresp,
  output wire                     m_axi_bready,
  output wire [ADDR_WIDTH-1:0]    m_axi_araddr,
  output wire                     m_axi_arvalid,
  input  wire                     m_axi_arready,
  input  wire [31:0]              m_axi_rdata,
  input  wire                     m_axi_rvalid,
  input  wire [1:0]               m_axi_rresp,
  output wire                     m_axi_rready,

  // AXI4-Lite slave interface for XIP engine
  input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr,
  input  wire                     s_axi_awvalid,
  output wire                     s_axi_awready,
  input  wire [31:0]              s_axi_wdata,
  input  wire [3:0]               s_axi_wstrb,
  input  wire                     s_axi_wvalid,
  output wire                     s_axi_wready,
  output wire [1:0]               s_axi_bresp,
  output wire                     s_axi_bvalid,
  input  wire                     s_axi_bready,
  input  wire [ADDR_WIDTH-1:0]    s_axi_araddr,
  input  wire                     s_axi_arvalid,
  output wire                     s_axi_arready,
  output wire [31:0]              s_axi_rdata,
  output wire [1:0]               s_axi_rresp,
  output wire                     s_axi_rvalid,
  input  wire                     s_axi_rready,

  // QSPI pads
  output wire                     sclk,
  output wire                     cs_n,
  inout  wire [3:0]               io,

  // Interrupt output
  output wire                     irq
);

  // ---------------------------------------------------------
  // Optional debug instrumentation (guarded)
  // Define QSPI_DEBUG at compile time to enable prints
  // Example: iverilog -D QSPI_DEBUG ...
  // ---------------------------------------------------------
`ifdef QSPI_DEBUG
  reg cmd_start_q, fsm_start_q, fsm_done_q, fsm_rx_wen_q;
  reg [LEVEL_WIDTH-1:0] fifo_rx_level_q;
  always @(posedge clk) begin
    if (!resetn) begin
      cmd_start_q     <= 1'b0;
      fsm_start_q     <= 1'b0;
      fsm_done_q      <= 1'b0;
      fsm_rx_wen_q    <= 1'b0;
      fifo_rx_level_q <= {LEVEL_WIDTH{1'b0}};
    end else begin
      if (cmd_start_pulse && !cmd_start_q)
        $display("[QSPI_CTL] cmd_start: op=%02h addr=%08h len=%0d dma_en=%0d @%0t",
                 opcode_w, cmd_addr_w, cmd_len_w, dma_en_w, $time);
      if (fsm_start_w && !fsm_start_q)
        $display("[QSPI_CTL] fsm_start: op=%02h addr=%08h len=%0d clk_div=%0d @%0t",
                 fsm_opcode_w, fsm_addr_w, fsm_len_w, fsm_clk_div_w, $time);
      if (fsm_done_w && !fsm_done_q)
        $display("[QSPI_CTL] fsm_done @%0t", $time);
      if (fsm_rx_wen_w && !fsm_rx_wen_q)
        $display("[QSPI_CTL] first RX: data=%08h level=%0d io1=%b @%0t",
                 fsm_rx_data_w, fifo_rx_level_w, io[1], $time);
      if (fifo_rx_level_w != fifo_rx_level_q)
        $display("[QSPI_CTL] fifo_rx_level %0d -> %0d @%0t",
                 fifo_rx_level_q, fifo_rx_level_w, $time);

      cmd_start_q     <= cmd_start_pulse;
      fsm_start_q     <= fsm_start_w;
      fsm_done_q      <= fsm_done_w;
      fsm_rx_wen_q    <= fsm_rx_wen_w;
      fifo_rx_level_q <= fifo_rx_level_w;
    end
  end
`endif

  // ---------------------------------------------------------
  // Local parameters and wires
  // ---------------------------------------------------------
  localparam integer LEVEL_WIDTH = $clog2(FIFO_DEPTH+1);

  // CSR <-> internal wires
  wire        enable_w;
  wire        xip_en_w;
  wire        quad_en_w;
  wire        cpol_w;
  wire        cpha_w;
  wire        cmd_start_w;
  wire        dma_en_w;
  wire        mode_en_w;
  wire        hold_en_w;
  wire        wp_en_w;
  wire [2:0]  clk_div_w;
  wire        cs_auto_w;
  wire [1:0]  cs_level_w;
  wire [1:0]  cs_delay_w;

  // XIP configuration wires
  wire [1:0]  xip_addr_bytes_w;
  wire [1:0]  xip_data_lanes_w;
  wire [3:0]  xip_dummy_cycles_w;
  wire        xip_cont_read_w;
  wire        xip_mode_en_w;
  wire        xip_write_en_w;
  wire [7:0]  xip_read_op_w;
  wire [7:0]  xip_mode_bits_w;
  wire [7:0]  xip_write_op_w;

  // Command configuration wires
  wire [1:0]  cmd_lanes_w;
  wire [1:0]  addr_lanes_w;
  wire [1:0]  data_lanes_w;
  wire [1:0]  addr_bytes_w;
  wire        mode_en_cfg_w;
  wire [3:0]  dummy_cycles_w;
  wire        is_write_w;
  wire [7:0]  opcode_w;
  wire [7:0]  mode_bits_w;
  wire [ADDR_WIDTH-1:0] cmd_addr_w;
  wire [31:0] cmd_len_w;
  wire [7:0]  extra_dummy_w;

  // DMA configuration wires
  wire [3:0]  burst_size_w;
  wire        dma_dir_w;
  wire        incr_addr_w;
  wire [ADDR_WIDTH-1:0] dma_addr_w;
  wire [31:0] dma_len_w;

  // FIFO interfaces
  wire [31:0] fifo_tx_data_csr_w;
  wire        fifo_tx_we_csr_w;
  wire [31:0] fifo_tx_data_dma_w;
  wire        fifo_tx_we_dma_w;
  wire [31:0] fifo_tx_rd_data_w;
  wire        fifo_tx_rd_en_w;
  wire        fifo_tx_full_w;
  wire        fifo_tx_empty_w;
  wire [LEVEL_WIDTH-1:0] fifo_tx_level_w;

  wire        fifo_rx_re_csr_w;
  wire        fifo_rx_re_dma_w;
  wire [31:0] fifo_rx_rd_data_w;
  wire        fifo_rx_full_w;
  wire        fifo_rx_empty_w;
  wire [LEVEL_WIDTH-1:0] fifo_rx_level_w;

  // Command engine signals
  wire        cmd_start_pulse;
  wire        cmd_trigger_clr_w;
  wire        cmd_done_set_w;
  wire        cmd_busy_w;
  wire        fsm_start_from_cmd;

  // DMA engine signals
  wire        dma_done_set_w;
  wire        dma_busy_w;
  wire        dma_axi_err_w;

  // XIP engine signals
  wire        xip_start_w;
  wire        xip_busy_w;
  wire        xip_active_w;
  wire        xip_fifo_rx_re_w;
  wire [31:0] xip_tx_data_w;
  wire        xip_tx_empty_w;

  // QSPI FSM connections
  wire        fsm_start_w;
  wire        fsm_done_w;
  wire [1:0]  fsm_cmd_lanes_w;
  wire [1:0]  fsm_addr_lanes_w;
  wire [1:0]  fsm_data_lanes_w;
  wire [1:0]  fsm_addr_bytes_w;
  wire        fsm_mode_en_w;
  wire [3:0]  fsm_dummy_cycles_w;
  wire        fsm_dir_w;
  wire        fsm_quad_en_w;
  wire        fsm_cs_auto_w;
  wire        fsm_xip_cont_w;
  wire [7:0]  fsm_opcode_w;
  wire [7:0]  fsm_mode_bits_w;
  wire [ADDR_WIDTH-1:0] fsm_addr_w;
  wire [31:0] fsm_len_w;
  wire [31:0] fsm_clk_div_w;
  wire        fsm_cpol_w;
  wire        fsm_cpha_w;
  wire [31:0] fsm_tx_data_w;
  wire        fsm_tx_empty_w;
  wire        fsm_tx_ren_w;
  wire [31:0] fsm_rx_data_w;
  wire        fsm_rx_wen_w;
  wire        sclk_int;
  wire        cs_n_int;

  // ---------------------------------------------------------
  // CSR instance
  // ---------------------------------------------------------
  csr #(
    .APB_ADDR_WIDTH (APB_ADDR_WIDTH),
    .APB_WINDOW_LSB (APB_WINDOW_LSB),
    .HAS_PSTRB      (HAS_PSTRB),
    .HAS_WP         (HAS_WP)
  ) u_csr (
    .pclk               (clk),
    .presetn            (resetn),
    .psel               (psel),
    .penable            (penable),
    .pwrite             (pwrite),
    .paddr              (paddr),
    .pwdata             (pwdata),
    .pstrb              (pstrb),
    .prdata             (prdata),
    .pready             (pready),
    .pslverr            (pslverr),
    .enable_o           (enable_w),
    .xip_en_o           (xip_en_w),
    .quad_en_o          (quad_en_w),
    .cpol_o             (cpol_w),
    .cpha_o             (cpha_w),
    .lsb_first_o        (),
    .cmd_start_o        (cmd_start_w),
    .dma_en_o           (dma_en_w),
    .mode_en_o          (mode_en_w),
    .hold_en_o          (hold_en_w),
    .wp_en_o            (wp_en_w),
    .cmd_trigger_clr_i  (cmd_trigger_clr_w),
    .clk_div_o          (clk_div_w),
    .cs_auto_o          (cs_auto_w),
    .cs_level_o         (cs_level_w),
    .cs_delay_o         (cs_delay_w),
    .xip_addr_bytes_o   (xip_addr_bytes_w),
    .xip_data_lanes_o   (xip_data_lanes_w),
    .xip_dummy_cycles_o (xip_dummy_cycles_w),
    .xip_cont_read_o    (xip_cont_read_w),
    .xip_mode_en_o      (xip_mode_en_w),
    .xip_write_en_o     (xip_write_en_w),
    .xip_read_op_o      (xip_read_op_w),
    .xip_mode_bits_o    (xip_mode_bits_w),
    .xip_write_op_o     (xip_write_op_w),
    .cmd_lanes_o        (cmd_lanes_w),
    .addr_lanes_o       (addr_lanes_w),
    .data_lanes_o       (data_lanes_w),
    .addr_bytes_o       (addr_bytes_w),
    .mode_en_cfg_o      (mode_en_cfg_w),
    .dummy_cycles_o     (dummy_cycles_w),
    .is_write_o         (is_write_w),
    .opcode_o           (opcode_w),
    .mode_bits_o        (mode_bits_w),
    .cmd_addr_o         (cmd_addr_w),
    .cmd_len_o          (cmd_len_w),
    .extra_dummy_o      (extra_dummy_w),
    .burst_size_o       (burst_size_w),
    .dma_dir_o          (dma_dir_w),
    .incr_addr_o        (incr_addr_w),
    .dma_addr_o         (dma_addr_w),
    .dma_len_o          (dma_len_w),
    .fifo_tx_data_o     (fifo_tx_data_csr_w),
    .fifo_tx_we_o       (fifo_tx_we_csr_w),
    .fifo_rx_data_i     (fifo_rx_rd_data_w),
    .fifo_rx_re_o       (fifo_rx_re_csr_w),
    .int_en_o           (),
    .cmd_done_set_i     (cmd_done_set_w),
    .dma_done_set_i     (dma_done_set_w),
    .err_set_i          (1'b0),
    .fifo_tx_empty_set_i(1'b0),
    .fifo_rx_full_set_i (1'b0),
    .busy_i             (cmd_busy_w | dma_busy_w | xip_busy_w),
    .xip_active_i       (xip_active_w),
    .cmd_done_i         (1'b0),
    .dma_done_i         (1'b0),
    .tx_level_i         (fifo_tx_level_w[3:0]),
    .rx_level_i         (fifo_rx_level_w[3:0]),
    .tx_empty_i         (fifo_tx_empty_w),
    .rx_full_i          (fifo_rx_full_w),
    .timeout_i          (1'b0),
    .overrun_i          (1'b0),
    .underrun_i         (1'b0),
    .axi_err_i          (dma_axi_err_w),
    .irq                (irq)
  );

  // ---------------------------------------------------------
  // TX FIFO write mux and instance
  // ---------------------------------------------------------
  wire fifo_tx_we_w   = fifo_tx_we_dma_w | fifo_tx_we_csr_w;
  wire [31:0] fifo_tx_data_w = fifo_tx_we_dma_w ? fifo_tx_data_dma_w : fifo_tx_data_csr_w;

  fifo_tx #(
    .WIDTH (32),
    .DEPTH (FIFO_DEPTH)
  ) u_fifo_tx (
    .clk       (clk),
    .resetn    (resetn),
    .wr_en_i   (fifo_tx_we_w),
    .wr_data_i (fifo_tx_data_w),
    .rd_en_i   (fifo_tx_rd_en_w),
    .rd_data_o (fifo_tx_rd_data_w),
    .full_o    (fifo_tx_full_w),
    .empty_o   (fifo_tx_empty_w),
    .level_o   (fifo_tx_level_w)
  );

  // ---------------------------------------------------------
  // RX FIFO read mux and instance
  // ---------------------------------------------------------
  fifo_rx #(
    .WIDTH (32),
    .DEPTH (FIFO_DEPTH)
  ) u_fifo_rx (
    .clk       (clk),
    .resetn    (resetn),
    .wr_en_i   (fsm_rx_wen_w),
    .wr_data_i (fsm_rx_data_w),
    .rd_en_i   (fifo_rx_re_csr_w | fifo_rx_re_dma_w | xip_fifo_rx_re_w),
    .rd_data_o (fifo_rx_rd_data_w),
    .full_o    (fifo_rx_full_w),
    .empty_o   (fifo_rx_empty_w),
    .level_o   (fifo_rx_level_w)
  );

  // ---------------------------------------------------------
  // Command engine instance
  // ---------------------------------------------------------
  assign cmd_start_pulse = cmd_start_w & ~xip_en_w & ~xip_busy_w;

  cmd_engine #(
    .ADDR_WIDTH (ADDR_WIDTH)
  ) u_cmd_engine (
    .clk              (clk),
    .resetn           (resetn),
    .cmd_start_i      (cmd_start_pulse),
    .cmd_trigger_clr_o(cmd_trigger_clr_w),
    .cmd_done_set_o   (cmd_done_set_w),
    .busy_o           (cmd_busy_w),
    .cmd_lanes_i      (cmd_lanes_w),
    .addr_lanes_i     (addr_lanes_w),
    .data_lanes_i     (data_lanes_w),
    .addr_bytes_i     (addr_bytes_w),
    .mode_en_i        (mode_en_cfg_w),
    .dummy_cycles_i   (dummy_cycles_w),
    .extra_dummy_i    (extra_dummy_w),
    .is_write_i       (is_write_w),
    .opcode_i         (opcode_w),
    .mode_bits_i      (mode_bits_w),
    .cmd_addr_i       (cmd_addr_w),
    .cmd_len_i        (cmd_len_w),
    .quad_en_i        (quad_en_w),
    .cs_auto_i        (cs_auto_w),
    .xip_cont_read_i  (xip_cont_read_w),
    .clk_div_i        (clk_div_w),
    .cpol_i           (cpol_w),
    .cpha_i           (cpha_w),
    .start_o          (fsm_start_from_cmd),
    .done_i           (fsm_done_w),
    .cmd_lanes_o      (),
    .addr_lanes_o     (),
    .data_lanes_o     (),
    .addr_bytes_o     (),
    .mode_en_o        (),
    .dummy_cycles_o   (),
    .dir_o            (),
    .quad_en_o        (),
    .cs_auto_o        (),
    .xip_cont_read_o  (),
    .opcode_o         (),
    .mode_bits_o      (),
    .addr_o           (),
    .len_o            (),
    .clk_div_o        (),
    .cpol_o           (),
    .cpha_o           ()
  );

  // ---------------------------------------------------------
  // DMA engine instance
  // ---------------------------------------------------------
  dma_engine #(
    .ADDR_WIDTH    (ADDR_WIDTH),
    .TX_FIFO_DEPTH (FIFO_DEPTH),
    .LEVEL_WIDTH   (LEVEL_WIDTH)
  ) u_dma_engine (
    .clk            (clk),
    .resetn         (resetn),
    .dma_en_i       (dma_en_w & ~xip_en_w),
    .dma_dir_i      (dma_dir_w),
    .burst_size_i   (burst_size_w),
    .incr_addr_i    (incr_addr_w),
    .dma_addr_i     (dma_addr_w),
    .dma_len_i      (dma_len_w),
    .tx_level_i     (fifo_tx_level_w),
    .fifo_tx_data_o (fifo_tx_data_dma_w),
    .fifo_tx_we_o   (fifo_tx_we_dma_w),
    .rx_level_i     (fifo_rx_level_w),
    .fifo_rx_data_i (fifo_rx_rd_data_w),
    .fifo_rx_re_o   (fifo_rx_re_dma_w),
    .dma_done_set_o (dma_done_set_w),
    .axi_err_o      (dma_axi_err_w),
    .busy_o         (dma_busy_w),
    .awaddr_o       (m_axi_awaddr),
    .awvalid_o      (m_axi_awvalid),
    .awready_i      (m_axi_awready),
    .wdata_o        (m_axi_wdata),
    .wvalid_o       (m_axi_wvalid),
    .wstrb_o        (m_axi_wstrb),
    .wready_i       (m_axi_wready),
    .bvalid_i       (m_axi_bvalid),
    .bresp_i        (m_axi_bresp),
    .bready_o       (m_axi_bready),
    .araddr_o       (m_axi_araddr),
    .arvalid_o      (m_axi_arvalid),
    .arready_i      (m_axi_arready),
    .rdata_i        (m_axi_rdata),
    .rvalid_i       (m_axi_rvalid),
    .rresp_i        (m_axi_rresp),
    .rready_o       (m_axi_rready)
  );

  // ---------------------------------------------------------
  // XIP engine instance
  // ---------------------------------------------------------
  xip_engine #(
    .ADDR_WIDTH (ADDR_WIDTH)
  ) u_xip_engine (
    .clk              (clk),
    .resetn           (resetn),
    .xip_en_i         (xip_en_w),
    .xip_addr_bytes_i (xip_addr_bytes_w),
    .xip_data_lanes_i (xip_data_lanes_w),
    .xip_dummy_cycles_i(xip_dummy_cycles_w),
    .xip_cont_read_i  (xip_cont_read_w),
    .xip_mode_en_i    (xip_mode_en_w),
    .xip_write_en_i   (xip_write_en_w),
    .xip_read_op_i    (xip_read_op_w),
    .xip_mode_bits_i  (xip_mode_bits_w),
    .xip_write_op_i   (xip_write_op_w),
    .clk_div_i        (clk_div_w),
    .cpol_i           (cpol_w),
    .cpha_i           (cpha_w),
    .quad_en_i        (quad_en_w),
    .cs_auto_i        (cs_auto_w),
    .cmd_busy_i       (cmd_busy_w),
    .awaddr_i         (s_axi_awaddr),
    .awvalid_i        (s_axi_awvalid),
    .awready_o        (s_axi_awready),
    .wdata_i          (s_axi_wdata),
    .wstrb_i          (s_axi_wstrb),
    .wvalid_i         (s_axi_wvalid),
    .wready_o         (s_axi_wready),
    .bresp_o          (s_axi_bresp),
    .bvalid_o         (s_axi_bvalid),
    .bready_i         (s_axi_bready),
    .araddr_i         (s_axi_araddr),
    .arvalid_i        (s_axi_arvalid),
    .arready_o        (s_axi_arready),
    .rdata_o          (s_axi_rdata),
    .rresp_o          (s_axi_rresp),
    .rvalid_o         (s_axi_rvalid),
    .rready_i         (s_axi_rready),
    .fifo_rx_data_i   (fifo_rx_rd_data_w),
    .fifo_rx_re_o     (xip_fifo_rx_re_w),
    .start_o          (xip_start_w),
    .done_i           (fsm_done_w),
    .tx_ren_i         (fsm_tx_ren_w),
    .tx_data_o        (xip_tx_data_w),
    .tx_empty_o       (xip_tx_empty_w),
    .cmd_lanes_o      (),
    .addr_lanes_o     (),
    .data_lanes_o     (),
    .addr_bytes_o     (),
    .mode_en_o        (),
    .dummy_cycles_o   (),
    .dir_o            (),
    .quad_en_o        (),
    .cs_auto_o        (),
    .xip_cont_read_o  (),
    .opcode_o         (),
    .mode_bits_o      (),
    .addr_o           (),
    .len_o            (),
    .clk_div_o        (),
    .cpol_o           (),
    .cpha_o           (),
    .busy_o           (xip_busy_w),
    .xip_active_o     (xip_active_w)
  );

  // ---------------------------------------------------------
  // QSPI FSM instance and signal muxing
  // ---------------------------------------------------------
  assign fsm_start_w      = fsm_start_from_cmd | xip_start_w;
  assign fsm_cmd_lanes_w  = xip_busy_w ? 2'b00 : cmd_lanes_w;
  assign fsm_addr_lanes_w = xip_busy_w ? 2'b00 : addr_lanes_w;
  assign fsm_data_lanes_w = xip_busy_w ? xip_data_lanes_w : data_lanes_w;
  assign fsm_addr_bytes_w = xip_busy_w ? xip_addr_bytes_w : addr_bytes_w;
  assign fsm_mode_en_w    = xip_busy_w ? xip_mode_en_w : mode_en_cfg_w;
  assign fsm_dummy_cycles_w = xip_busy_w ? xip_dummy_cycles_w : dummy_cycles_w;
  assign fsm_dir_w        = xip_busy_w ? 1'b1 : ~is_write_w;
  assign fsm_quad_en_w    = quad_en_w;
  assign fsm_cs_auto_w    = cs_auto_w;
  assign fsm_xip_cont_w   = xip_busy_w ? xip_cont_read_w : xip_cont_read_w;
  assign fsm_opcode_w     = xip_busy_w ? xip_read_op_w : opcode_w;
  assign fsm_mode_bits_w  = xip_busy_w ? xip_mode_bits_w : mode_bits_w;
  assign fsm_addr_w       = xip_busy_w ? {ADDR_WIDTH{1'b0}} : cmd_addr_w;
  assign fsm_len_w        = xip_busy_w ? 32'd4 : cmd_len_w;
  assign fsm_clk_div_w    = {29'd0, clk_div_w};
  assign fsm_cpol_w       = cpol_w;
  assign fsm_cpha_w       = cpha_w;
  assign fsm_tx_data_w    = xip_busy_w ? xip_tx_data_w : fifo_tx_rd_data_w;
  assign fsm_tx_empty_w   = xip_busy_w ? xip_tx_empty_w : fifo_tx_empty_w;

  assign fifo_tx_rd_en_w  = fsm_tx_ren_w & ~xip_busy_w;

  qspi_fsm #(
    .ADDR_WIDTH (ADDR_WIDTH)
  ) u_qspi_fsm (
    .clk           (clk),
    .resetn        (resetn),
    .start         (fsm_start_w),
    .done          (fsm_done_w),
    .cmd_lanes_sel (fsm_cmd_lanes_w),
    .addr_lanes_sel(fsm_addr_lanes_w),
    .data_lanes_sel(fsm_data_lanes_w),
    .addr_bytes_sel(fsm_addr_bytes_w),
    .mode_en       (fsm_mode_en_w),
    .dummy_cycles  (fsm_dummy_cycles_w),
    .dir           (fsm_dir_w),
    .quad_en       (fsm_quad_en_w),
    .cs_auto       (fsm_cs_auto_w),
    .xip_cont_read (fsm_xip_cont_w),
    .cmd_opcode    (fsm_opcode_w),
    .mode_bits     (fsm_mode_bits_w),
    .addr          (fsm_addr_w),
    .len_bytes     (fsm_len_w),
    .clk_div       (fsm_clk_div_w),
    .cpol          (fsm_cpol_w),
    .cpha          (fsm_cpha_w),
    .tx_data_fifo  (fsm_tx_data_w),
    .tx_empty      (fsm_tx_empty_w),
    .tx_ren        (fsm_tx_ren_w),
    .rx_data_fifo  (fsm_rx_data_w),
    .rx_wen        (fsm_rx_wen_w),
    .rx_full       (fifo_rx_full_w),
    .sclk          (sclk_int),
    .cs_n          (cs_n_int),
    .io0           (io[0]),
    .io1           (io[1]),
    .io2           (io[2]),
    .io3           (io[3])
  );

  // ---------------------------------------------------------
  // QSPI IO instance (pass-through)
  // ---------------------------------------------------------
  qspi_io u_qspi_io (
    .clk          (clk),
    .resetn       (resetn),
    .sclk_i       (sclk_int),
    .cs_n_i       (cs_n_int),
    .io_oe_i      (4'b0000),
    .out_bits_i   (4'b0000),
    .sclk_o       (sclk),
    .cs_n_o       (cs_n),
    .input_bits_o (),
    .io           (io)
  );

endmodule
