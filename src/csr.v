// csr.v - APB control/status registers for QSPI controller
// Implements zero wait-state APB3/4 slave. Register map stops at 0x050.
// CTRL[8] is CMD_TRIGGER (write-one-to-start) which generates a one-cycle
// cmd_start_o pulse and is cleared by cmd_trigger_clr_i from the command
// engine. Trigger is ignored when CTRL.enable=0, XIP_EN=1 or busy_i=1.
// CMD_TRIGGER when busy_i generates PSLVERR. XIP_EN writes while busy or
// DMA enabled are ignored. CMD and XIP modes are mutually exclusive.
// All registers use synchronous active-low reset.

module csr #(
  parameter integer APB_ADDR_WIDTH = 12,
  parameter integer APB_WINDOW_LSB = 12,  // 4KB window
  parameter integer HAS_PSTRB      = 0,   // 0: ignore pstrb
  parameter integer HAS_WP         = 0    // 1: CTRL[10] writable
)(
  // APB interface
  input  wire                     pclk,
  input  wire                     presetn,
  input  wire                     psel,
  input  wire                     penable,
  input  wire                     pwrite,
  input  wire [APB_ADDR_WIDTH-1:0] paddr,
  input  wire [31:0]              pwdata,
  input  wire [3:0]               pstrb,
  output reg  [31:0]              prdata,
  output wire                     pready,
  output reg                      pslverr,

  // CTRL outputs
  output wire enable_o,
  output wire xip_en_o,
  output wire quad_en_o,
  output wire cpol_o,
  output wire cpha_o,
  output wire lsb_first_o,
  output wire cmd_start_o,
  output wire dma_en_o,
  output wire hold_en_o,
  output wire wp_en_o,

  // clear from command engine
  input  wire cmd_trigger_clr_i,

  // Clock & CS
  output wire [2:0] clk_div_o,
  output wire       cs_auto_o,
  output wire [1:0] cs_level_o,
  output wire [1:0] cs_delay_o,

  // XIP configuration
  output wire [1:0] xip_addr_bytes_o,
  output wire [1:0] xip_data_lanes_o,
  output wire [3:0] xip_dummy_cycles_o,
  output wire       xip_cont_read_o,
  output wire       xip_mode_en_o,
  output wire       xip_write_en_o,
  output wire [7:0] xip_read_op_o,
  output wire [7:0] xip_mode_bits_o,
  output wire [7:0] xip_write_op_o,

  // Command configuration
  output wire [1:0] cmd_lanes_o,
  output wire [1:0] addr_lanes_o,
  output wire [1:0] data_lanes_o,
  output wire [1:0] addr_bytes_o,
  output wire       mode_en_cfg_o,
  output wire [3:0] dummy_cycles_o,
  output wire       is_write_o,
  output wire [7:0] opcode_o,
  output wire [7:0] mode_bits_o,
  output wire [31:0] cmd_addr_o,
  output wire [31:0] cmd_len_o,
  output wire [7:0] extra_dummy_o,

  // DMA configuration
  output wire [3:0]  burst_size_o,
  output wire        dma_dir_o,
  output wire        incr_addr_o,
  output wire [31:0] dma_addr_o,
  output wire [31:0] dma_len_o,

  // FIFO windows
  output wire [31:0] fifo_tx_data_o,
  output wire        fifo_tx_we_o,
  input  wire [31:0] fifo_rx_data_i,
  output wire        fifo_rx_re_o,

  // Interrupts
  output wire [4:0] int_en_o,
  input  wire cmd_done_set_i,
  input  wire dma_done_set_i,
  input  wire err_set_i,
  input  wire fifo_tx_empty_set_i,
  input  wire fifo_rx_full_set_i,

  // Status inputs
  input  wire busy_i,
  input  wire xip_active_i,
  input  wire cmd_done_i,
  input  wire dma_done_i,
  input  wire [3:0] tx_level_i,
  input  wire [3:0] rx_level_i,
  input  wire tx_empty_i,
  input  wire rx_full_i,
  input  wire timeout_i,
  input  wire overrun_i,
  input  wire underrun_i,
  input  wire axi_err_i,
  output wire irq
);

  // ---------------------------------------------------------
  // APB basic signals
  wire setup_phase  = psel & ~penable;
  wire access_phase = psel & penable;
  wire write_phase  = access_phase & pwrite;
  wire read_phase   = access_phase & ~pwrite;
  assign pready = 1'b1;                       // zero wait-state
  wire [3:0] pstrb_eff = (HAS_PSTRB!=0) ? pstrb : 4'b1111;

  // address window (map stops at 0x050)
  localparam integer WIN = APB_WINDOW_LSB;
  wire [WIN-1:0] a = paddr[WIN-1:0];

  localparam [WIN-1:0] ID_ADDR        = 'h000;
  localparam [WIN-1:0] CTRL_ADDR      = 'h004;
  localparam [WIN-1:0] STATUS_ADDR    = 'h008;
  localparam [WIN-1:0] INT_EN_ADDR    = 'h00C;
  localparam [WIN-1:0] INT_STAT_ADDR  = 'h010; // W1C
  localparam [WIN-1:0] CLK_DIV_ADDR   = 'h014;
  localparam [WIN-1:0] CS_CTRL_ADDR   = 'h018;
  localparam [WIN-1:0] XIP_CFG_ADDR   = 'h01C;
  localparam [WIN-1:0] XIP_CMD_ADDR   = 'h020;
  localparam [WIN-1:0] CMD_CFG_ADDR   = 'h024;
  localparam [WIN-1:0] CMD_OP_ADDR    = 'h028;
  localparam [WIN-1:0] CMD_ADDR_ADDR  = 'h02C;
  localparam [WIN-1:0] CMD_LEN_ADDR   = 'h030;
  localparam [WIN-1:0] CMD_DUMMY_ADDR = 'h034;
  localparam [WIN-1:0] DMA_CFG_ADDR   = 'h038;
  localparam [WIN-1:0] DMA_DST_ADDR   = 'h03C;
  localparam [WIN-1:0] DMA_LEN_ADDR   = 'h040;
  localparam [WIN-1:0] FIFO_TX_ADDR   = 'h044;
  localparam [WIN-1:0] FIFO_RX_ADDR   = 'h048;
  localparam [WIN-1:0] FIFO_STAT_ADDR = 'h04C;
  localparam [WIN-1:0] ERR_STAT_ADDR  = 'h050;

  // ---------------------------------------------------------
  // Registers
  reg [31:0] ctrl_reg, int_en_reg, int_stat_reg;
  reg [31:0] clk_div_reg, cs_ctrl_reg;
  reg [31:0] xip_cfg_reg, xip_cmd_reg;
  reg [31:0] cmd_cfg_reg, cmd_op_reg, cmd_addr_reg, cmd_len_reg, cmd_dummy_reg;
  reg [31:0] dma_cfg_reg, dma_addr_reg, dma_len_reg;
  reg [31:0] err_stat_reg;

  // fixed values
  localparam [31:0] ID_VALUE = 32'h1A00_1081;

  // status latches for cmd/dma done
  reg cmd_done_latched, dma_done_latched;

  // ---------------------------------------------------------
  // Address decode
  reg valid_addr, ro_addr;
  always @(*) begin
    valid_addr = 1'b0;
    ro_addr    = 1'b0;
    case (a)
      ID_ADDR        : begin valid_addr=1'b1; ro_addr=1'b1; end
      CTRL_ADDR      : begin valid_addr=1'b1; end
      STATUS_ADDR    : begin valid_addr=1'b1; ro_addr=1'b1; end
      INT_EN_ADDR    : begin valid_addr=1'b1; end
      INT_STAT_ADDR  : begin valid_addr=1'b1; end
      CLK_DIV_ADDR   : begin valid_addr=1'b1; end
      CS_CTRL_ADDR   : begin valid_addr=1'b1; end
      XIP_CFG_ADDR   : begin valid_addr=1'b1; end
      XIP_CMD_ADDR   : begin valid_addr=1'b1; end
      CMD_CFG_ADDR   : begin valid_addr=1'b1; end
      CMD_OP_ADDR    : begin valid_addr=1'b1; end
      CMD_ADDR_ADDR  : begin valid_addr=1'b1; end
      CMD_LEN_ADDR   : begin valid_addr=1'b1; end
      CMD_DUMMY_ADDR : begin valid_addr=1'b1; end
      DMA_CFG_ADDR   : begin valid_addr=1'b1; end
      DMA_DST_ADDR   : begin valid_addr=1'b1; end
      DMA_LEN_ADDR   : begin valid_addr=1'b1; end
      FIFO_TX_ADDR   : begin valid_addr=1'b1; end
      FIFO_RX_ADDR   : begin valid_addr=1'b1; ro_addr=1'b1; end
      FIFO_STAT_ADDR : begin valid_addr=1'b1; ro_addr=1'b1; end
      ERR_STAT_ADDR  : begin valid_addr=1'b1; end
      default        : begin valid_addr=1'b0; ro_addr=1'b0; end
    endcase
  end

  // ---------------------------------------------------------
  // PSLVERR generation
  always @(*) begin
    pslverr = 1'b0;
    if (write_phase) begin
      if (!valid_addr || ro_addr)
        pslverr = 1'b1;
      else if ((a==CTRL_ADDR) && pstrb_eff[1] && pwdata[8] && busy_i)
        pslverr = 1'b1; // busy lockout
    end
  end

  // global write enable
  wire wr_ok = write_phase & valid_addr & ~ro_addr;

  // FIFO side effects
  assign fifo_tx_we_o   = wr_ok & (a==FIFO_TX_ADDR);
  assign fifo_tx_data_o = pwdata;

  // Generate a pop pulse on the first access to FIFO_RX and suppress
  // additional pops until a different register is touched.
  reg fifo_rx_pop_seen;
  always @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      fifo_rx_pop_seen <= 1'b0;
    end else begin
      if (read_phase && valid_addr && (a==FIFO_RX_ADDR))
        fifo_rx_pop_seen <= 1'b1;
      else if (psel && (a!=FIFO_RX_ADDR))
        fifo_rx_pop_seen <= 1'b0;
    end
  end
  assign fifo_rx_re_o = (read_phase && valid_addr && (a==FIFO_RX_ADDR) && !fifo_rx_pop_seen);

  // ---------------------------------------------------------
  // Write masks
  // CTRL write mask: exclude CMD_TRIGGER (bit8) and reserve bit7 (no mode_en in CTRL)
  wire [31:0] CTRL_WMASK = (HAS_WP!=0) ? 32'h0000_067F : 32'h0000_027F;
  wire [31:0] CLKDIV_WMASK = 32'h0000_000F;
  wire [31:0] CSCTRL_WMASK = 32'h0000_001F;
  wire [31:0] XIPCFG_WMASK = 32'h0000_3FFF;
  wire [31:0] XIPCMD_WMASK = 32'h00FF_FFFF;
  wire [31:0] CMDCFG_WMASK = 32'h0000_3FFF; // include bit[13] for mode_en
  wire [31:0] CMDOP_WMASK  = 32'h0000_FFFF;
  wire [31:0] CMDDMY_WMASK = 32'h0000_00FF;
  wire [31:0] DMACFG_WMASK = 32'h0000_003F;

  // helper for byte-masked writes
  function [31:0] apply_strb;
    input [31:0] cur;
    input [31:0] data;
    begin
      apply_strb = {
        pstrb_eff[3] ? data[31:24] : cur[31:24],
        pstrb_eff[2] ? data[23:16] : cur[23:16],
        pstrb_eff[1] ? data[15:8]  : cur[15:8],
        pstrb_eff[0] ? data[7:0]   : cur[7:0]
      };
    end
  endfunction

  // ---------------------------------------------------------
  // CMD_TRIGGER handling (W1S)
  wire cmd_trig_wr = wr_ok & (a==CTRL_ADDR) & pstrb_eff[1] & pwdata[8];
  // evaluate enable/xip_en as they will be after this write to allow
  // combined enable+trigger transactions
  wire ctrl_enable_n = (wr_ok && (a==CTRL_ADDR) && pstrb_eff[0]) ? pwdata[0] : ctrl_reg[0];
  wire ctrl_xip_n    = (wr_ok && (a==CTRL_ADDR) && pstrb_eff[0]) ? pwdata[1] : ctrl_reg[1];
  wire cmd_trig_ok   = cmd_trig_wr & ctrl_enable_n & ~ctrl_xip_n & ~busy_i;
  reg  cmd_trig_q;
  always @(posedge pclk or negedge presetn) begin
    if (!presetn) cmd_trig_q <= 1'b0;
    else if (cmd_trigger_clr_i) cmd_trig_q <= 1'b0;
    else if (cmd_trig_ok) cmd_trig_q <= 1'b1;
  end
  assign cmd_start_o = cmd_trig_q; // one-cycle pulse when CE acks

  // ---------------------------------------------------------
  // FIFO_RX APB read: generate pop and return data with 1-cycle latency
  // This matches synchronous FIFO behavior where data appears after rd_en.
  // ---------------------------------------------------------
  reg [31:0] fifo_rx_data_q;
  reg        fifo_rx_re_q;
  always @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      fifo_rx_data_q <= 32'h0;
      fifo_rx_re_q   <= 1'b0;
    end else begin
      fifo_rx_re_q   <= fifo_rx_re_o;           // delay pop by one cycle
      if (fifo_rx_re_q)
        fifo_rx_data_q <= fifo_rx_data_i;       // capture popped data
    end
  end

  // ---------------------------------------------------------
  // Register writes
  always @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      ctrl_reg      <= 32'h0;
      int_en_reg    <= 32'h0;
      int_stat_reg  <= 32'h0;
      clk_div_reg   <= 32'h0;
      cs_ctrl_reg   <= 32'h0000_0001; // default CS auto=1
      // Default XIP: 1-1-1 fast read (0x0B), 3-byte addr, 8 dummy cycles
      // xip_cfg_reg bits: [1:0] addr_bytes, [3:2] data_lanes, [7:4] dummy, [8] cont_read, [9] mode_en
      xip_cfg_reg   <= 32'h0000_0081; // addr_bytes=01 (3B), data_lanes=00 (1), dummy=8, cont_read=0, mode_en=0
      xip_cmd_reg   <= 32'h0000_000B; // read opcode=0x0B
      cmd_cfg_reg   <= 32'h0;
      cmd_op_reg    <= 32'h0;
      cmd_addr_reg  <= 32'h0;
      cmd_len_reg   <= 32'h0;
      cmd_dummy_reg <= 32'h0;
      dma_cfg_reg   <= 32'h0;
      dma_addr_reg  <= 32'h0;
      dma_len_reg   <= 32'h0;
      err_stat_reg  <= 32'h0;
      cmd_done_latched <= 1'b0;
      dma_done_latched <= 1'b0;
    end else begin
      // CTRL write with masks
      if (wr_ok && (a==CTRL_ADDR)) begin
        reg [31:0] next_ctrl;
        next_ctrl = apply_strb(ctrl_reg, pwdata);
        next_ctrl = (next_ctrl & CTRL_WMASK) | (ctrl_reg & ~CTRL_WMASK);
        // Ignore XIP_EN if busy or if DMA_EN is being set in this write.
        // Allow enabling XIP in the same write that clears DMA_EN.
        if (busy_i || next_ctrl[6])
          next_ctrl[1] = ctrl_reg[1];
        ctrl_reg <= next_ctrl;
      end

      // INT_EN (bits [4:0] in byte0)
      if (wr_ok && (a==INT_EN_ADDR)) begin
        int_en_reg[4:0] <= pstrb_eff[0] ? pwdata[4:0] : int_en_reg[4:0];
      end

      if (wr_ok && (a==CLK_DIV_ADDR))
        clk_div_reg <= apply_strb(clk_div_reg, pwdata) & CLKDIV_WMASK;
      if (wr_ok && (a==CS_CTRL_ADDR))
        cs_ctrl_reg <= apply_strb(cs_ctrl_reg, pwdata) & CSCTRL_WMASK;
      if (wr_ok && (a==XIP_CFG_ADDR))
        xip_cfg_reg <= apply_strb(xip_cfg_reg, pwdata) & XIPCFG_WMASK;
      if (wr_ok && (a==XIP_CMD_ADDR))
        xip_cmd_reg <= apply_strb(xip_cmd_reg, pwdata) & XIPCMD_WMASK;
      if (wr_ok && (a==CMD_CFG_ADDR))
        cmd_cfg_reg <= apply_strb(cmd_cfg_reg, pwdata) & CMDCFG_WMASK;
      if (wr_ok && (a==CMD_OP_ADDR))
        cmd_op_reg <= apply_strb(cmd_op_reg, pwdata) & CMDOP_WMASK;
      if (wr_ok && (a==CMD_ADDR_ADDR))
        cmd_addr_reg <= apply_strb(cmd_addr_reg, pwdata);
      if (wr_ok && (a==CMD_LEN_ADDR))
        cmd_len_reg <= apply_strb(cmd_len_reg, pwdata);
      if (wr_ok && (a==CMD_DUMMY_ADDR))
        cmd_dummy_reg <= apply_strb(cmd_dummy_reg, pwdata) & CMDDMY_WMASK;
      if (wr_ok && (a==DMA_CFG_ADDR))
        dma_cfg_reg <= apply_strb(dma_cfg_reg, pwdata) & DMACFG_WMASK;
      if (wr_ok && (a==DMA_DST_ADDR))
        dma_addr_reg <= apply_strb(dma_addr_reg, pwdata);
      if (wr_ok && (a==DMA_LEN_ADDR))
        dma_len_reg <= apply_strb(dma_len_reg, pwdata);

      // INT_STAT W1C
      if (wr_ok && (a==INT_STAT_ADDR))
        int_stat_reg <= int_stat_reg & ~pwdata;

      // ERR_STAT W1C
      if (wr_ok && (a==ERR_STAT_ADDR))
        err_stat_reg <= err_stat_reg & ~pwdata;

      // latch status/interrupt sources
      if (cmd_done_set_i) begin
        int_stat_reg[0] <= 1'b1;
        cmd_done_latched <= 1'b1;
      end
      if (dma_done_set_i) begin
        int_stat_reg[1] <= 1'b1;
        dma_done_latched <= 1'b1;
      end
      if (err_set_i) int_stat_reg[2] <= 1'b1;
      if (fifo_tx_empty_set_i) int_stat_reg[3] <= 1'b1;
      if (fifo_rx_full_set_i) int_stat_reg[4] <= 1'b1;

      // error status bits
      if (timeout_i)  err_stat_reg[0] <= 1'b1;
      if (overrun_i)  err_stat_reg[1] <= 1'b1;
      if (underrun_i) err_stat_reg[2] <= 1'b1;
      if (axi_err_i)  err_stat_reg[3] <= 1'b1;

      // status W1C clears
      if (wr_ok && (a==STATUS_ADDR) && pstrb_eff[0]) begin
        if (pwdata[0]) cmd_done_latched <= 1'b0;
        if (pwdata[1]) dma_done_latched <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------
  // Read mux
  always @(*) begin
    prdata = 32'h0;
    if (read_phase && valid_addr) begin
      case (a)
        ID_ADDR        : prdata = ID_VALUE;
        CTRL_ADDR      : prdata = ctrl_reg;
        STATUS_ADDR    : prdata = {20'd0, rx_level_i, tx_level_i, busy_i, xip_active_i, cmd_done_latched, dma_done_latched};
        INT_EN_ADDR    : prdata = int_en_reg;
        INT_STAT_ADDR  : prdata = int_stat_reg;
        CLK_DIV_ADDR   : prdata = clk_div_reg;
        CS_CTRL_ADDR   : prdata = cs_ctrl_reg;
        XIP_CFG_ADDR   : prdata = xip_cfg_reg;
        XIP_CMD_ADDR   : prdata = xip_cmd_reg;
        CMD_CFG_ADDR   : prdata = cmd_cfg_reg;
        CMD_OP_ADDR    : prdata = cmd_op_reg;
        CMD_ADDR_ADDR  : prdata = cmd_addr_reg;
        CMD_LEN_ADDR   : prdata = cmd_len_reg;
        CMD_DUMMY_ADDR : prdata = cmd_dummy_reg;
        DMA_CFG_ADDR   : prdata = dma_cfg_reg;
        DMA_DST_ADDR   : prdata = dma_addr_reg;
        DMA_LEN_ADDR   : prdata = dma_len_reg;
        FIFO_RX_ADDR   : prdata = fifo_rx_data_q;
        FIFO_STAT_ADDR : prdata = {22'd0, rx_full_i, tx_empty_i, rx_level_i, tx_level_i};
        ERR_STAT_ADDR  : prdata = err_stat_reg;
        default        : prdata = 32'h0;
      endcase
    end
  end

  // ---------------------------------------------------------
  // Field mapping
  assign enable_o      = ctrl_reg[0];
  assign xip_en_o      = ctrl_reg[1];
  assign quad_en_o     = ctrl_reg[2]; // set only after flash QE set
  assign cpol_o        = ctrl_reg[3];
  assign cpha_o        = ctrl_reg[4];
  assign lsb_first_o   = ctrl_reg[5];
  assign dma_en_o      = ctrl_reg[6];
  assign hold_en_o     = ctrl_reg[9];
  assign wp_en_o       = (HAS_WP!=0) ? ctrl_reg[10] : 1'b0;

  assign clk_div_o     = clk_div_reg[2:0];
  assign cs_auto_o     = cs_ctrl_reg[0];
  assign cs_level_o    = cs_ctrl_reg[2:1];
  assign cs_delay_o    = cs_ctrl_reg[4:3];

  assign xip_addr_bytes_o  = xip_cfg_reg[1:0];
  assign xip_data_lanes_o  = xip_cfg_reg[3:2];
  assign xip_dummy_cycles_o= xip_cfg_reg[7:4];
  assign xip_cont_read_o   = xip_cfg_reg[8];
  assign xip_mode_en_o     = xip_cfg_reg[9];
  assign xip_write_en_o    = xip_cfg_reg[10];
  assign xip_read_op_o     = xip_cmd_reg[7:0];
  assign xip_write_op_o    = xip_cmd_reg[15:8];
  assign xip_mode_bits_o   = xip_cmd_reg[23:16];

  assign cmd_lanes_o   = cmd_cfg_reg[1:0];
  assign addr_lanes_o  = cmd_cfg_reg[3:2];
  assign data_lanes_o  = cmd_cfg_reg[5:4];
  assign addr_bytes_o  = cmd_cfg_reg[7:6];
  // Mode enable for command path is sourced from CMD_CFG[13] per spec
  assign mode_en_cfg_o = cmd_cfg_reg[13];
  assign dummy_cycles_o= cmd_cfg_reg[11:8];
  assign is_write_o    = cmd_cfg_reg[12];
  assign opcode_o      = cmd_op_reg[7:0];
  assign mode_bits_o   = cmd_op_reg[15:8];
  assign cmd_addr_o    = cmd_addr_reg;
  assign cmd_len_o     = cmd_len_reg;
  assign extra_dummy_o = cmd_dummy_reg[7:0];

  assign burst_size_o  = dma_cfg_reg[3:0];
  assign dma_dir_o     = dma_cfg_reg[4];
  assign incr_addr_o   = dma_cfg_reg[5];
  assign dma_addr_o    = dma_addr_reg;
  assign dma_len_o     = dma_len_reg;

  assign int_en_o      = int_en_reg[4:0];
  assign irq           = |(int_en_reg[4:0] & int_stat_reg[4:0]);

endmodule
