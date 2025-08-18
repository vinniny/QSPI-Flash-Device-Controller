/*
 * cmd_engine.v - Command engine for QSPI controller
 *
 * Latches command configuration from CSR when cmd_start_i is asserted,
 * drives qspi_fsm.v start pulse and configuration, and generates
 * completion/trigger clear flags back to the CSR.
 */

module cmd_engine #(
  parameter ADDR_WIDTH = 32
)(
  input  wire                     clk,
  input  wire                     resetn,

  // CSR control
  input  wire                     cmd_start_i,
  output reg                      cmd_trigger_clr_o,
  output reg                      cmd_done_set_o,
  output wire                     busy_o,

  // CSR configuration
  input  wire [1:0]               cmd_lanes_i,
  input  wire [1:0]               addr_lanes_i,
  input  wire [1:0]               data_lanes_i,
  input  wire [1:0]               addr_bytes_i,
  input  wire                     mode_en_i,
  input  wire [3:0]               dummy_cycles_i,
  input  wire [7:0]               extra_dummy_i,
  input  wire                     is_write_i,
  input  wire [7:0]               opcode_i,
  input  wire [7:0]               mode_bits_i,
  input  wire [ADDR_WIDTH-1:0]    cmd_addr_i,
  input  wire [31:0]              cmd_len_i,
  input  wire                     quad_en_i,
  input  wire                     cs_auto_i,
  input  wire                     xip_cont_read_i,
  input  wire [2:0]               clk_div_i,
  input  wire                     cpol_i,
  input  wire                     cpha_i,

  // QSPI FSM interface
  output reg                      start_o,
  input  wire                     done_i,
  output wire [1:0]               cmd_lanes_o,
  output wire [1:0]               addr_lanes_o,
  output wire [1:0]               data_lanes_o,
  output wire [1:0]               addr_bytes_o,
  output wire                     mode_en_o,
  output wire [3:0]               dummy_cycles_o,
  output wire                     dir_o,
  output wire                     quad_en_o,
  output wire                     cs_auto_o,
  output wire                     xip_cont_read_o,
  output wire [7:0]               opcode_o,
  output wire [7:0]               mode_bits_o,
  output wire [ADDR_WIDTH-1:0]    addr_o,
  output wire [31:0]              len_o,
  output wire [31:0]              clk_div_o,
  output wire                     cpol_o,
  output wire                     cpha_o
);

  // ------------------------------------------------------------
  // Internal registers holding latched command fields
  // ------------------------------------------------------------
  reg        busy_r;
  reg [1:0]  cmd_lanes_r, addr_lanes_r, data_lanes_r, addr_bytes_r;
  reg        mode_en_r;
  reg [3:0]  dummy_cycles_r;
  reg        is_write_r;
  reg [7:0]  opcode_r, mode_bits_r;
  reg [ADDR_WIDTH-1:0] addr_r;
  reg [31:0] len_r;
  reg        quad_en_r, cs_auto_r, xip_cont_read_r;
  reg [31:0] clk_div_r;
  reg        cpol_r, cpha_r;

  assign busy_o = busy_r;

  // ------------------------------------------------------------
  // Dummy cycle combine and saturation (limited to 4 bits)
  // ------------------------------------------------------------
  wire [4:0] dummy_sum = {1'b0, dummy_cycles_i} + {1'b0, extra_dummy_i[3:0]};
  wire [3:0] dummy_eff = dummy_sum[4] ? 4'hF : dummy_sum[3:0];

  // ------------------------------------------------------------
  // Command capture and status generation
  // ------------------------------------------------------------
  always @(posedge clk) begin
    if (!resetn) begin
      busy_r           <= 1'b0;
      start_o          <= 1'b0;
      cmd_trigger_clr_o <= 1'b0;
      cmd_done_set_o   <= 1'b0;
      cmd_lanes_r      <= 2'b00;
      addr_lanes_r     <= 2'b00;
      data_lanes_r     <= 2'b00;
      addr_bytes_r     <= 2'b00;
      mode_en_r        <= 1'b0;
      dummy_cycles_r   <= 4'd0;
      is_write_r       <= 1'b0;
      opcode_r         <= 8'd0;
      mode_bits_r      <= 8'd0;
      addr_r           <= {ADDR_WIDTH{1'b0}};
      len_r            <= 32'd0;
      quad_en_r        <= 1'b0;
      cs_auto_r        <= 1'b0;
      xip_cont_read_r  <= 1'b0;
      clk_div_r        <= 32'd0;
      cpol_r           <= 1'b0;
      cpha_r           <= 1'b0;
    end else begin
      start_o          <= 1'b0;
      cmd_trigger_clr_o <= 1'b0;
      cmd_done_set_o   <= 1'b0;

      if (!busy_r && cmd_start_i) begin
        busy_r          <= 1'b1;
        start_o         <= 1'b1;
        cmd_trigger_clr_o <= 1'b1;

        cmd_lanes_r     <= cmd_lanes_i;
        addr_lanes_r    <= addr_lanes_i;
        data_lanes_r    <= data_lanes_i;
        addr_bytes_r    <= addr_bytes_i;
        mode_en_r       <= mode_en_i;
        dummy_cycles_r  <= dummy_eff;
        is_write_r      <= is_write_i;
        opcode_r        <= opcode_i;
        mode_bits_r     <= mode_bits_i;
        addr_r          <= cmd_addr_i;
        len_r           <= cmd_len_i;
        quad_en_r       <= quad_en_i;
        cs_auto_r       <= cs_auto_i;
        xip_cont_read_r <= xip_cont_read_i;
        clk_div_r       <= {29'd0, clk_div_i};
        cpol_r          <= cpol_i;
        cpha_r          <= cpha_i;
      end else if (busy_r && done_i) begin
        busy_r         <= 1'b0;
        cmd_done_set_o <= 1'b1;
      end
    end
  end

  // ------------------------------------------------------------
  // Outputs to QSPI FSM
  // ------------------------------------------------------------
  assign cmd_lanes_o      = cmd_lanes_r;
  assign addr_lanes_o     = addr_lanes_r;
  assign data_lanes_o     = data_lanes_r;
  assign addr_bytes_o     = addr_bytes_r;
  assign mode_en_o        = mode_en_r;
  assign dummy_cycles_o   = dummy_cycles_r;
  assign dir_o            = ~is_write_r; // 0:write 1:read
  assign quad_en_o        = quad_en_r;
  assign cs_auto_o        = cs_auto_r;
  assign xip_cont_read_o  = xip_cont_read_r;
  assign opcode_o         = opcode_r;
  assign mode_bits_o      = mode_bits_r;
  assign addr_o           = addr_r;
  assign len_o            = len_r;
  assign clk_div_o        = clk_div_r;
  assign cpol_o           = cpol_r;
  assign cpha_o           = cpha_r;

endmodule
