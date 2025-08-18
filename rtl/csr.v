// csr.v — QSPI CSR (docx-aligned)
// - Two-phase APB timing
// - W1C INT_STAT with rising-edge qualifiers
// - CTRL[8] CMD_TRIGGER is W1S on byte1 and cleared by CE via cmd_trigger_clr_i
// - Reserved-bit write masks
// - RO/invalid-write -> PSLVERR; unmapped reads return 0 (no error)
// - Optional APB4 byte strobes via HAS_PSTRB (default off; acts as 4'b1111)
// - Configurable APB decode window via APB_WINDOW_LSB (default 4KB)
// - INT_EN byte-lane fix: bits [4:0] use pstrb_eff[0]
// - Optional WP bit via HAS_WP (default 0 = not writeable / reads 0)
module csr #(
  parameter integer APB_ADDR_WIDTH = 12,
  parameter integer APB_WINDOW_LSB = 12, // 4KB window
  parameter integer HAS_PSTRB = 0, // 0: ignore pstrb; 1: honor pstrb
  parameter integer HAS_WP = 0 // 0: CTRL[10] not writeable/0; 1: writeable
)(
  // APB
  input wire pclk,
  input wire reset,
  input wire psel,
  input wire penable,
  input wire pwrite,
  input wire [APB_ADDR_WIDTH-1:0] paddr,
  input wire [31:0] pwdata,
  input wire [3:0] pstrb, // tie 4'b1111 if HAS_PSTRB=0
  output reg [31:0] prdata,
  output wire pready,
  output reg pslverr,
  // CTRL
  output wire enable_o,
  output wire xip_en_o,
  output wire quad_en_o,
  output wire cpol_o,
  output wire cpha_o,
  output wire lsb_first_o,
  output wire cmd_trigger_o, // W1S; cleared by CE
  output wire dma_en_o,
  output wire mode_en_o,
  output wire hold_en_o,
  output wire wp_en_o, // optional (HAS_WP)
  // CE clear for CMD_TRIGGER
  input wire cmd_trigger_clr_i,
  // Clock & CS
  output wire [2:0] clk_div_o,
  output wire cs_auto_o,
  output wire [1:0] cs_level_o,
  output wire [1:0] cs_delay_o,
  // XIP
  output wire [1:0] xip_addr_bytes_o,
  output wire [1:0] xip_data_lanes_o,
  output wire [3:0] xip_dummy_cycles_o,
  output wire xip_cont_read_o,
  output wire xip_mode_en_o,
  output wire xip_write_en_o,
  output wire [7:0] xip_read_op_o,
  output wire [7:0] xip_mode_bits_o,
  output wire [7:0] xip_write_op_o,
  // Command mode
  output wire [1:0] cmd_lanes_o,
  output wire [1:0] addr_lanes_o,
  output wire [1:0] data_lanes_o,
  output wire [1:0] addr_bytes_o,
  output wire mode_en_cfg_o, // tied to CTRL.mode_en
  output wire [3:0] dummy_cycles_o,
  output wire is_write_o, // 0=READ, 1=WRITE
  output wire [7:0] opcode_o,
  output wire [7:0] mode_bits_o,
  output wire [31:0] cmd_addr_o,
  output wire [31:0] cmd_len_o,
  output wire [7:0] extra_dummy_o,
  // DMA
  output wire [3:0] burst_size_o,
  output wire dma_dir_o,
  output wire incr_addr_o,
  output wire [31:0] dma_addr_o,
  output wire [31:0] dma_len_o,
  // FIFO windows
  output wire [31:0] fifo_tx_data_o,
  output wire fifo_tx_we_o,
  input wire [31:0] fifo_rx_data_i,
  output wire fifo_rx_re_o,
  // INT
  output wire [4:0] int_en_o,
  input wire cmd_done_set_i,
  input wire dma_done_set_i,
  input wire err_set_i,
  input wire fifo_tx_empty_set_i,
  input wire fifo_rx_full_set_i,
  // STATUS live
  input wire busy_i,
  input wire xip_active_i,
  input wire cmd_done_i,
  input wire dma_done_i,
  input wire [3:0] tx_level_i,
  input wire [3:0] rx_level_i,
  input wire tx_empty_i,
  input wire rx_full_i,
  input wire timeout_i,
  input wire overrun_i,
  input wire underrun_i,
  input wire axi_err_i,
  output wire irq
);
  // ---------------- Address window & constants -------------
  localparam integer WIN = APB_WINDOW_LSB;
  wire [WIN-1:0] a = paddr[WIN-1:0];
  // WIN-wide addresses (map stops at 0x050)
  localparam [WIN-1:0] ID_ADDR = 'h000;
  localparam [WIN-1:0] CTRL_ADDR = 'h004;
  localparam [WIN-1:0] STATUS_ADDR = 'h008;
  localparam [WIN-1:0] INT_EN_ADDR = 'h00C;
  localparam [WIN-1:0] INT_STAT_ADDR = 'h010; // W1C
  localparam [WIN-1:0] CLK_DIV_ADDR = 'h014;
  localparam [WIN-1:0] CS_CTRL_ADDR = 'h018;
  localparam [WIN-1:0] XIP_CFG_ADDR = 'h01C;
  localparam [WIN-1:0] XIP_CMD_ADDR = 'h020;
  localparam [WIN-1:0] CMD_CFG_ADDR = 'h024;
  localparam [WIN-1:0] CMD_OP_ADDR = 'h028;
  localparam [WIN-1:0] CMD_ADDR_ADDR = 'h02C;
  localparam [WIN-1:0] CMD_LEN_ADDR = 'h030;
  localparam [WIN-1:0] CMD_DUMMY_ADDR = 'h034;
  localparam [WIN-1:0] DMA_CFG_ADDR = 'h038;
  localparam [WIN-1:0] DMA_DST_ADDR = 'h03C; // DRAM destination
  localparam [WIN-1:0] DMA_LEN_ADDR = 'h040;
  localparam [WIN-1:0] FIFO_TX_ADDR = 'h044; // write
  localparam [WIN-1:0] FIFO_RX_ADDR = 'h048; // read
  localparam [WIN-1:0] FIFO_STAT_ADDR = 'h04C;
  localparam [WIN-1:0] ERR_STAT_ADDR = 'h050;
  // ---------------- Registers ------------------------------
  reg [31:0] ctrl_reg, int_en_reg, int_stat_reg;
  reg [31:0] clk_div_reg, cs_ctrl_reg;
  reg [31:0] xip_cfg_reg, xip_cmd_reg;
  reg [31:0] cmd_cfg_reg, cmd_op_reg, cmd_addr_reg, cmd_len_reg, cmd_dummy_reg;
  reg [31:0] dma_cfg_reg, dma_addr_reg, dma_len_reg;
  // Fixed IP ID (per spec)
  localparam [31:0] ID_VALUE = 32'h1A001081;
  // Read-only composites
  wire [31:0] id_reg = ID_VALUE;
  wire [31:0] status_reg = {26'd0, busy_i, xip_active_i, cmd_done_i, dma_done_i, 2'b00};
  wire [31:0] fifo_stat_reg = {22'd0, rx_full_i, tx_empty_i, rx_level_i[3:0], tx_level_i[3:0]};
  wire [31:0] err_stat_reg = {28'd0, axi_err_i, underrun_i, overrun_i, timeout_i};
  // ---------------- APB timing -----------------------------
  wire setup_phase = psel & ~penable;
  wire access_phase = psel & penable;
  wire write_phase = access_phase & pwrite;
  wire read_phase = access_phase & ~pwrite;
  assign pready = 1'b1;
  // Effective strobes (when HAS_PSTRB=0, behaves as full-word writes)
  wire [3:0] pstrb_eff = (HAS_PSTRB!=0) ? pstrb : 4'b1111;
  // Valid/RO decode
  reg valid_addr;
  reg ro_addr;
  always @(*) begin
    valid_addr = 1'b0;
    ro_addr = 1'b0;
    if (access_phase) begin
      case (a)
        ID_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b1; end
        CTRL_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b0; end
        STATUS_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b1; end
        INT_EN_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b0; end
        INT_STAT_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b0; end // W1C on write
        CLK_DIV_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b0; end
        CS_CTRL_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b0; end
        XIP_CFG_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b0; end
        XIP_CMD_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b0; end
        CMD_CFG_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b0; end
        CMD_OP_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b0; end
        CMD_ADDR_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b0; end
        CMD_LEN_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b0; end
        CMD_DUMMY_ADDR: begin valid_addr = 1'b1; ro_addr = 1'b0; end
        DMA_CFG_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b0; end
        DMA_DST_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b0; end
        DMA_LEN_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b0; end
        FIFO_TX_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b0; end // WO side-effect
        FIFO_RX_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b1; end
        FIFO_STAT_ADDR: begin valid_addr = 1'b1; ro_addr = 1'b1; end
        ERR_STAT_ADDR : begin valid_addr = 1'b1; ro_addr = 1'b1; end
        default : begin valid_addr = 1'b0; ro_addr = 1'b0; end
      endcase
    end
  end
  // PSLVERR: assert on invalid WRITE or WRITE to RO
  always @(*) begin
    if (access_phase && pwrite && (!valid_addr || ro_addr))
      pslverr = 1'b1;
    else
      pslverr = 1'b0;
  end
  // Global write gate (valid, and not RO)
  wire wr_ok = write_phase & valid_addr & ~ro_addr;
  // FIFO side-effects
  assign fifo_tx_we_o = wr_ok & (a == FIFO_TX_ADDR);
  assign fifo_tx_data_o = pwdata;
  assign fifo_rx_re_o = read_phase & valid_addr & (a == FIFO_RX_ADDR);
  // ---------------- Write masks ----------------------------
  // Effective CTRL mask (tighten if HAS_WP==0)
  wire [31:0] CTRL_WMASK_EFF = (HAS_WP != 0) ? 32'h0000_03FF : 32'h0000_01FF;
  // NOTE: masked bits read as 0; if you need write-any/read-same, remove masks
  wire [31:0] CTRL_WMASK = 32'h0000_03FF; // nominal (used for readback checks)
  wire [31:0] CLKDIV_WMASK = 32'h0000_000F;
  wire [31:0] CSCTRL_WMASK = 32'h0000_000F;
  wire [31:0] XIPCFG_WMASK = 32'h0000_3FFF;
  wire [31:0] XIPCMD_WMASK = 32'h00FF_FFFF;
  wire [31:0] CMDCFG_WMASK = 32'h0000_1FFF;
  wire [31:0] CMDOP_WMASK = 32'h0000_FFFF;
  wire [31:0] CMDDMY_WMASK = 32'h0000_00FF;
  wire [31:0] DMACFG_WMASK = 32'h0000_003F;
  // Pre-masked data per target reg
  // For CTRL: ensure bit[8] (trigger) is forced 0 into the stored ctrl_reg
  wire [31:0] CTRL_PMW_base = pwdata & CTRL_WMASK_EFF;
  wire [31:0] CTRL_PMW = { CTRL_PMW_base[31:9], 1'b0, CTRL_PMW_base[7:0] }; // clear bit8
  wire [31:0] CLKDIV_PMW = pwdata & CLKDIV_WMASK;
  wire [31:0] CSCTRL_PMW = pwdata & CSCTRL_WMASK;
  wire [31:0] XIPCFG_PMW = pwdata & XIPCFG_WMASK;
  wire [31:0] XIPCMD_PMW = pwdata & XIPCMD_WMASK;
  wire [31:0] CMDCFG_PMW = pwdata & CMDCFG_WMASK;
  wire [31:0] CMDOP_PMW = pwdata & CMDOP_WMASK;
  wire [31:0] CMDDMY_PMW = pwdata & CMDDMY_WMASK;
  wire [31:0] DMACFG_PMW = pwdata & DMACFG_WMASK;
  // Byte-strobe merged write data (no functions)
  wire [31:0] CTRL_WDATA = { pstrb_eff[3] ? CTRL_PMW[31:24] : ctrl_reg[31:24],
                               pstrb_eff[2] ? CTRL_PMW[23:16] : ctrl_reg[23:16],
                               pstrb_eff[1] ? CTRL_PMW[15:8] : ctrl_reg[15:8],
                               pstrb_eff[0] ? CTRL_PMW[7:0] : ctrl_reg[7:0] };
  wire [31:0] CLKDIV_WDATA = { pstrb_eff[3] ? CLKDIV_PMW[31:24] : clk_div_reg[31:24],
                               pstrb_eff[2] ? CLKDIV_PMW[23:16] : clk_div_reg[23:16],
                               pstrb_eff[1] ? CLKDIV_PMW[15:8] : clk_div_reg[15:8],
                               pstrb_eff[0] ? CLKDIV_PMW[7:0] : clk_div_reg[7:0] };
  wire [31:0] CSCTRL_WDATA = { pstrb_eff[3] ? CSCTRL_PMW[31:24] : cs_ctrl_reg[31:24],
                               pstrb_eff[2] ? CSCTRL_PMW[23:16] : cs_ctrl_reg[23:16],
                               pstrb_eff[1] ? CSCTRL_PMW[15:8] : cs_ctrl_reg[15:8],
                               pstrb_eff[0] ? CSCTRL_PMW[7:0] : cs_ctrl_reg[7:0] };
  wire [31:0] XIPCFG_WDATA = { pstrb_eff[3] ? XIPCFG_PMW[31:24] : xip_cfg_reg[31:24],
                               pstrb_eff[2] ? XIPCFG_PMW[23:16] : xip_cfg_reg[23:16],
                               pstrb_eff[1] ? XIPCFG_PMW[15:8] : xip_cfg_reg[15:8],
                               pstrb_eff[0] ? XIPCFG_PMW[7:0] : xip_cfg_reg[7:0] };
  wire [31:0] XIPCMD_WDATA = { pstrb_eff[3] ? XIPCMD_PMW[31:24] : xip_cmd_reg[31:24],
                               pstrb_eff[2] ? XIPCMD_PMW[23:16] : xip_cmd_reg[23:16],
                               pstrb_eff[1] ? XIPCMD_PMW[15:8] : xip_cmd_reg[15:8],
                               pstrb_eff[0] ? XIPCMD_PMW[7:0] : xip_cmd_reg[7:0] };
  wire [31:0] CMDCFG_WDATA = { pstrb_eff[3] ? CMDCFG_PMW[31:24] : cmd_cfg_reg[31:24],
                               pstrb_eff[2] ? CMDCFG_PMW[23:16] : cmd_cfg_reg[23:16],
                               pstrb_eff[1] ? CMDCFG_PMW[15:8] : cmd_cfg_reg[15:8],
                               pstrb_eff[0] ? CMDCFG_PMW[7:0] : cmd_cfg_reg[7:0] };
  wire [31:0] CMDOP_WDATA = { pstrb_eff[3] ? CMDOP_PMW[31:24] : cmd_op_reg[31:24],
                               pstrb_eff[2] ? CMDOP_PMW[23:16] : cmd_op_reg[23:16],
                               pstrb_eff[1] ? CMDOP_PMW[15:8] : cmd_op_reg[15:8],
                               pstrb_eff[0] ? CMDOP_PMW[7:0] : cmd_op_reg[7:0] };
  wire [31:0] CMDDMY_WDATA = { pstrb_eff[3] ? CMDDMY_PMW[31:24] : cmd_dummy_reg[31:24],
                               pstrb_eff[2] ? CMDDMY_PMW[23:16] : cmd_dummy_reg[23:16],
                               pstrb_eff[1] ? CMDDMY_PMW[15:8] : cmd_dummy_reg[15:8],
                               pstrb_eff[0] ? CMDDMY_PMW[7:0] : cmd_dummy_reg[7:0] };
  wire [31:0] DMACFG_WDATA = { pstrb_eff[3] ? DMACFG_PMW[31:24] : dma_cfg_reg[31:24],
                               pstrb_eff[2] ? DMACFG_PMW[23:16] : dma_cfg_reg[23:16],
                               pstrb_eff[1] ? DMACFG_PMW[15:8] : dma_cfg_reg[15:8],
                               pstrb_eff[0] ? DMACFG_PMW[7:0] : dma_cfg_reg[7:0] };
  wire [31:0] CMDADDR_WDATA= { pstrb_eff[3] ? pwdata[31:24] : cmd_addr_reg[31:24],
                               pstrb_eff[2] ? pwdata[23:16] : cmd_addr_reg[23:16],
                               pstrb_eff[1] ? pwdata[15:8] : cmd_addr_reg[15:8],
                               pstrb_eff[0] ? pwdata[7:0] : cmd_addr_reg[7:0] };
  wire [31:0] CMDLEN_WDATA = { pstrb_eff[3] ? pwdata[31:24] : cmd_len_reg[31:24],
                               pstrb_eff[2] ? pwdata[23:16] : cmd_len_reg[23:16],
                               pstrb_eff[1] ? pwdata[15:8] : cmd_len_reg[15:8],
                               pstrb_eff[0] ? pwdata[7:0] : cmd_len_reg[7:0] };
  wire [31:0] DMAADDR_WDATA= { pstrb_eff[3] ? pwdata[31:24] : dma_addr_reg[31:24],
                               pstrb_eff[2] ? pwdata[23:16] : dma_addr_reg[23:16],
                               pstrb_eff[1] ? pwdata[15:8] : dma_addr_reg[15:8],
                               pstrb_eff[0] ? pwdata[7:0] : dma_addr_reg[7:0] };
  wire [31:0] DMALEN_WDATA = { pstrb_eff[3] ? pwdata[31:24] : dma_len_reg[31:24],
                               pstrb_eff[2] ? pwdata[23:16] : dma_len_reg[23:16],
                               pstrb_eff[1] ? pwdata[15:8] : dma_len_reg[15:8],
                               pstrb_eff[0] ? pwdata[7:0] : dma_len_reg[7:0] };
  // ---------------- CMD_TRIGGER (W1S + CE clear) -----------
  wire cmd_trig_set = write_phase &
                      (a == CTRL_ADDR) &
                      pstrb_eff[1] & // byte1 must be enabled
                      pwdata[8] & // bit8 set
                      valid_addr & ~ro_addr; // valid write to CTRL
  reg cmd_trig_q;
  always @(posedge pclk) begin
    if (reset) cmd_trig_q <= 1'b0;
    else if (cmd_trigger_clr_i) cmd_trig_q <= 1'b0;
    else if (cmd_trig_set) cmd_trig_q <= 1'b1;
  end
  assign cmd_trigger_o = cmd_trig_q;
  // ---------------- Register writes ------------------------
  always @(posedge pclk) begin
    if (reset) begin
      ctrl_reg <= 32'h0;
      int_en_reg <= 32'h0;
      int_stat_reg <= 32'h0;
      clk_div_reg <= 32'h0;
      cs_ctrl_reg <= 32'h0000_0001; // auto CS=1 as a friendly default
      xip_cfg_reg <= 32'h0;
      xip_cmd_reg <= 32'h0;
      cmd_cfg_reg <= 32'h0;
      cmd_op_reg <= 32'h0;
      cmd_addr_reg <= 32'h0;
      cmd_len_reg <= 32'h0;
      cmd_dummy_reg <= 32'h0;
      dma_cfg_reg <= 32'h0;
      dma_addr_reg <= 32'h0;
      dma_len_reg <= 32'h0;
    end else begin
      if (wr_ok && (a==CTRL_ADDR)) ctrl_reg <= CTRL_WDATA;
      // INT_EN write: bits [4:0] are in byte0 -> use pstrb_eff[0] for all
      if (wr_ok && (a==INT_EN_ADDR)) begin
        int_en_reg <= {27'd0,
                       (pstrb_eff[0] ? pwdata[4] : int_en_reg[4]),
                       (pstrb_eff[0] ? pwdata[3] : int_en_reg[3]),
                       (pstrb_eff[0] ? pwdata[2] : int_en_reg[2]),
                       (pstrb_eff[0] ? pwdata[1] : int_en_reg[1]),
                       (pstrb_eff[0] ? pwdata[0] : int_en_reg[0])};
      end
      if (wr_ok && (a==CLK_DIV_ADDR)) clk_div_reg <= CLKDIV_WDATA;
      if (wr_ok && (a==CS_CTRL_ADDR)) cs_ctrl_reg <= CSCTRL_WDATA;
      if (wr_ok && (a==XIP_CFG_ADDR)) xip_cfg_reg <= XIPCFG_WDATA;
      if (wr_ok && (a==XIP_CMD_ADDR)) xip_cmd_reg <= XIPCMD_WDATA;
      if (wr_ok && (a==CMD_CFG_ADDR)) cmd_cfg_reg <= CMDCFG_WDATA;
      if (wr_ok && (a==CMD_OP_ADDR)) cmd_op_reg <= CMDOP_WDATA;
      if (wr_ok && (a==CMD_ADDR_ADDR)) cmd_addr_reg <= CMDADDR_WDATA;
      if (wr_ok && (a==CMD_LEN_ADDR)) cmd_len_reg <= CMDLEN_WDATA;
      if (wr_ok && (a==CMD_DUMMY_ADDR)) cmd_dummy_reg <= CMDDMY_WDATA;
      if (wr_ok && (a==DMA_CFG_ADDR)) dma_cfg_reg <= DMACFG_WDATA;
      if (wr_ok && (a==DMA_DST_ADDR)) dma_addr_reg <= DMAADDR_WDATA;
      if (wr_ok && (a==DMA_LEN_ADDR)) dma_len_reg <= DMALEN_WDATA;
      // INT W1C
      if (wr_ok && (a==INT_STAT_ADDR)) int_stat_reg <= int_stat_reg & ~pwdata;
      // INT setters (rising edges below)
      if (cmd_done_rise) int_stat_reg[0] <= 1'b1;
      if (dma_done_rise) int_stat_reg[1] <= 1'b1;
      if (err_rise) int_stat_reg[2] <= 1'b1;
      if (txe_rise) int_stat_reg[3] <= 1'b1;
      if (rxf_rise) int_stat_reg[4] <= 1'b1;
    end
  end
  // ---------------- INT edge qualification ----------------
  reg cmd_done_d, dma_done_d, err_d, txe_d, rxf_d;
  always @(posedge pclk) begin
    if (reset) begin
      cmd_done_d <= 1'b0; dma_done_d <= 1'b0; err_d <= 1'b0; txe_d <= 1'b0; rxf_d <= 1'b0;
    end else begin
      cmd_done_d <= cmd_done_set_i;
      dma_done_d <= dma_done_set_i;
      err_d <= err_set_i;
      txe_d <= fifo_tx_empty_set_i;
      rxf_d <= fifo_rx_full_set_i;
    end
  end
  wire cmd_done_rise = cmd_done_set_i & ~cmd_done_d;
  wire dma_done_rise = dma_done_set_i & ~dma_done_d;
  wire err_rise = err_set_i & ~err_d;
  wire txe_rise = fifo_tx_empty_set_i & ~txe_d;
  wire rxf_rise = fifo_rx_full_set_i & ~rxf_d;
  // ---------------- Read mux -------------------------------
  always @(*) begin
    prdata = 32'h0; // unmapped reads return 0
    if (read_phase & valid_addr) begin
      case (a)
        ID_ADDR : prdata = id_reg;
        CTRL_ADDR : prdata = ctrl_reg;
        STATUS_ADDR : prdata = status_reg;
        INT_EN_ADDR : prdata = int_en_reg;
        INT_STAT_ADDR : prdata = int_stat_reg;
        CLK_DIV_ADDR : prdata = clk_div_reg;
        CS_CTRL_ADDR : prdata = cs_ctrl_reg;
        XIP_CFG_ADDR : prdata = xip_cfg_reg;
        XIP_CMD_ADDR : prdata = xip_cmd_reg;
        CMD_CFG_ADDR : prdata = cmd_cfg_reg;
        CMD_OP_ADDR : prdata = cmd_op_reg;
        CMD_ADDR_ADDR : prdata = cmd_addr_reg;
        CMD_LEN_ADDR : prdata = cmd_len_reg;
        CMD_DUMMY_ADDR: prdata = cmd_dummy_reg;
        DMA_CFG_ADDR : prdata = dma_cfg_reg;
        DMA_DST_ADDR : prdata = dma_addr_reg;
        DMA_LEN_ADDR : prdata = dma_len_reg;
        FIFO_STAT_ADDR: prdata = fifo_stat_reg;
        ERR_STAT_ADDR : prdata = err_stat_reg;
        FIFO_RX_ADDR : prdata = fifo_rx_data_i;
        default : prdata = 32'h0;
      endcase
    end
  end
  // ---------------- Field mapping --------------------------
  // CTRL (bit8 is trigger only; not stored as '1' in ctrl_reg)
  assign enable_o = ctrl_reg[0];
  assign xip_en_o = ctrl_reg[1];
  assign quad_en_o = ctrl_reg[2];
  assign cpol_o = ctrl_reg[3];
  assign cpha_o = ctrl_reg[4];
  assign lsb_first_o = ctrl_reg[5];
  assign dma_en_o = ctrl_reg[6];
  assign mode_en_o = ctrl_reg[7];
  // bit[8] is trigger pulse only (see cmd_trigger_o)
  assign hold_en_o = ctrl_reg[9];
  assign wp_en_o = (HAS_WP!=0) ? ctrl_reg[10] : 1'b0;
  // Clock & CS
  assign clk_div_o = clk_div_reg[2:0];
  assign cs_auto_o = cs_ctrl_reg[0];
  assign cs_level_o = cs_ctrl_reg[2:1];
  assign cs_delay_o = cs_ctrl_reg[4:3];
  // XIP
  assign xip_addr_bytes_o = xip_cfg_reg[1:0];
  assign xip_data_lanes_o = xip_cfg_reg[3:2];
  assign xip_dummy_cycles_o = xip_cfg_reg[7:4];
  assign xip_cont_read_o = xip_cfg_reg[8];
  assign xip_mode_en_o = xip_cfg_reg[9];
  assign xip_write_en_o = xip_cfg_reg[10];
  assign xip_read_op_o = xip_cmd_reg[7:0];
  assign xip_write_op_o = xip_cmd_reg[15:8];
  assign xip_mode_bits_o = xip_cmd_reg[23:16];
  // Command mode
  assign cmd_lanes_o = cmd_cfg_reg[1:0];
  assign addr_lanes_o = cmd_cfg_reg[3:2];
  assign data_lanes_o = cmd_cfg_reg[5:4];
  assign addr_bytes_o = cmd_cfg_reg[7:6];
  assign mode_en_cfg_o = mode_en_o; // tied to CTRL.mode_en
  assign dummy_cycles_o = cmd_cfg_reg[11:8];
  assign is_write_o = cmd_cfg_reg[12];
  assign opcode_o = cmd_op_reg[7:0];
  assign mode_bits_o = cmd_op_reg[15:8];
  assign cmd_addr_o = cmd_addr_reg;
  assign cmd_len_o = cmd_len_reg;
  assign extra_dummy_o = cmd_dummy_reg[7:0];
  // DMA
  assign burst_size_o = dma_cfg_reg[3:0];
  assign dma_dir_o = dma_cfg_reg[4];
  assign incr_addr_o = dma_cfg_reg[5];
  assign dma_addr_o = dma_addr_reg;
  assign dma_len_o = dma_len_reg;
  // INT
  assign int_en_o = int_en_reg[4:0];
  assign irq = |(int_en_reg[4:0] & int_stat_reg[4:0]);
endmodule