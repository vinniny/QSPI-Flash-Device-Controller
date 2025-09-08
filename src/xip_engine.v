/*
 * xip_engine.v - Execute-In-Place AXI4-Lite slave
 *
 * Converts AXI read (and optional write) transactions into QSPI flash
 * fetches using qspi_fsm.v. Read configuration is supplied from the
 * CSR block and latched per-transaction. Transactions are only
 * accepted when XIP mode is enabled and the command engine is idle.
 *
 * On an AXI read, the engine triggers the QSPI FSM to perform a flash
 * read of four bytes starting at the requested address. Returned data
 * is pulled from the RX FIFO and presented on the AXI read channel.
 *
 * Writes are optionally supported when xip_write_en_i is high. A single
 * 32-bit word is written using the write opcode from CSR. The write data
 * is supplied directly to the FSM without buffering.
 */

module xip_engine #(
  parameter ADDR_WIDTH = 32
) (
  input  wire                     clk,
  input  wire                     resetn,

  // CSR configuration
  input  wire                     xip_en_i,
  input  wire [1:0]               xip_addr_bytes_i,
  input  wire [1:0]               xip_data_lanes_i,
  input  wire [3:0]               xip_dummy_cycles_i,
  input  wire                     xip_cont_read_i,
  input  wire                     xip_mode_en_i,
  input  wire                     xip_write_en_i,
  input  wire [7:0]               xip_read_op_i,
  input  wire [7:0]               xip_mode_bits_i,
  input  wire [7:0]               xip_write_op_i,
  input  wire [2:0]               clk_div_i,
  input  wire                     cpol_i,
  input  wire                     cpha_i,
  input  wire                     quad_en_i,
  input  wire                     cs_auto_i,
  input  wire                     cmd_busy_i,

  // AXI4-Lite slave interface
  // Write address channel
  input  wire [ADDR_WIDTH-1:0]    awaddr_i,
  input  wire                     awvalid_i,
  output reg                      awready_o,
  // Write data channel
  input  wire [31:0]              wdata_i,
  input  wire [3:0]               wstrb_i,
  input  wire                     wvalid_i,
  output reg                      wready_o,
  // Write response channel
  output reg [1:0]                bresp_o,
  output reg                      bvalid_o,
  input  wire                     bready_i,
  // Read address channel
  input  wire [ADDR_WIDTH-1:0]    araddr_i,
  input  wire                     arvalid_i,
  output reg                      arready_o,
  // Read data channel
  output reg [31:0]               rdata_o,
  output reg [1:0]                rresp_o,
  output reg                      rvalid_o,
  input  wire                     rready_i,

  // RX FIFO interface
  input  wire [31:0]              fifo_rx_data_i,
  output reg                      fifo_rx_re_o,

  // QSPI FSM interface
  output reg                      start_o,
  input  wire                     done_i,
  input  wire                     tx_ren_i,
  output reg [31:0]               tx_data_o,
  output reg                      tx_empty_o,
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
  output wire                     cpha_o,

  // Status outputs
  output wire                     busy_o,
  output reg                      xip_active_o
);

  // ------------------------------------------------------------
  // Internal registers
  // ------------------------------------------------------------
  reg [ADDR_WIDTH-1:0] addr_r;
  reg [31:0]           wdata_r;
  reg                  is_write_r;
  reg [7:0]            opcode_r, mode_bits_r;
  reg [1:0]            addr_bytes_r, data_lanes_r;
  reg [3:0]            dummy_cycles_r;
  reg                  mode_en_r, xip_cont_read_r;
  reg [31:0]           clk_div_r;
  reg                  cpol_r, cpha_r;
  reg                  quad_en_r, cs_auto_r;
  reg                  busy_r;

  assign busy_o = busy_r;
  assign cmd_lanes_o     = 2'b00;          // command always single lane
  assign addr_lanes_o    = 2'b00;          // address single lane
  assign data_lanes_o    = data_lanes_r;
  assign addr_bytes_o    = addr_bytes_r;
  assign mode_en_o       = mode_en_r;
  assign dummy_cycles_o  = dummy_cycles_r;
  assign dir_o           = ~is_write_r;    // 1=read, 0=write
  assign quad_en_o       = quad_en_r;
  assign cs_auto_o       = cs_auto_r;
  assign xip_cont_read_o = xip_cont_read_r;
  assign opcode_o        = opcode_r;
  assign mode_bits_o     = mode_bits_r;
  assign addr_o          = addr_r;
  assign len_o           = 32'd4;
  assign clk_div_o       = clk_div_r;
  assign cpol_o          = cpol_r;
  assign cpha_o          = cpha_r;

  // ------------------------------------------------------------
  // State machine
  // ------------------------------------------------------------
  localparam S_IDLE     = 3'd0,
             S_RD_WAIT  = 3'd1,
             S_RD_POP   = 3'd2,
             S_RD_RESP  = 3'd3,
             S_WR_WAIT  = 3'd4,
             S_WR_RESP  = 3'd5,
             S_RD_CAP   = 3'd6;

  reg [2:0] state;

  // Combinational ready signals
  wire ar_ready_w = (state == S_IDLE) && xip_en_i && !cmd_busy_i;
  wire aw_ready_w = (state == S_IDLE) && xip_en_i && xip_write_en_i && !cmd_busy_i;
  wire w_ready_w  = aw_ready_w;

  // ------------------------------------------------------------
  // Sequential logic
  // ------------------------------------------------------------
  always @(posedge clk) begin
    if (!resetn) begin
      state        <= S_IDLE;
      busy_r       <= 1'b0;
      start_o      <= 1'b0;
      fifo_rx_re_o <= 1'b0;
      arready_o    <= 1'b0;
      awready_o    <= 1'b0;
      wready_o     <= 1'b0;
      rvalid_o     <= 1'b0;
      bvalid_o     <= 1'b0;
      rdata_o      <= 32'd0;
      bresp_o      <= 2'b00;
      rresp_o      <= 2'b00;
      tx_data_o    <= 32'd0;
      tx_empty_o   <= 1'b1;
      xip_active_o <= 1'b0;
    end else begin
      // default outputs
      start_o      <= 1'b0;
      fifo_rx_re_o <= 1'b0;
      rvalid_o     <= (state == S_RD_RESP);
      bvalid_o     <= (state == S_WR_RESP);
      arready_o    <= ar_ready_w;
      awready_o    <= aw_ready_w;
      wready_o     <= w_ready_w;
      xip_active_o <= (state != S_IDLE);

      case (state)
        S_IDLE: begin
          busy_r     <= 1'b0;
          tx_empty_o <= 1'b1;
          if (arvalid_i && ar_ready_w) begin
`ifdef XIP_DEBUG
            $display("[XIP] %0t AR addr=%08h start read", $time, araddr_i);
`endif
            // latch configuration for read
            addr_r          <= araddr_i;
            opcode_r        <= xip_read_op_i;
            mode_bits_r     <= xip_mode_bits_i;
            addr_bytes_r    <= xip_addr_bytes_i;
            data_lanes_r    <= xip_data_lanes_i;
            dummy_cycles_r  <= xip_dummy_cycles_i;
            mode_en_r       <= xip_mode_en_i;
            xip_cont_read_r <= xip_cont_read_i;
            clk_div_r       <= {29'd0, clk_div_i};
            cpol_r          <= cpol_i;
            cpha_r          <= cpha_i;
            quad_en_r       <= quad_en_i;
            cs_auto_r       <= cs_auto_i;
            is_write_r      <= 1'b0;
            busy_r          <= 1'b1;
            start_o         <= 1'b1;
            state           <= S_RD_WAIT;
          end else if (awvalid_i && wvalid_i && aw_ready_w && w_ready_w) begin
`ifdef XIP_DEBUG
            $display("[XIP] %0t AW/W start write addr=%08h data=%08h", $time, awaddr_i, wdata_i);
`endif
            // latch configuration for write
            addr_r          <= awaddr_i;
            wdata_r         <= wdata_i;
            opcode_r        <= xip_write_op_i;
            mode_bits_r     <= 8'd0;
            addr_bytes_r    <= xip_addr_bytes_i;
            data_lanes_r    <= xip_data_lanes_i;
            dummy_cycles_r  <= 4'd0;
            mode_en_r       <= 1'b0;
            xip_cont_read_r <= 1'b0;
            clk_div_r       <= {29'd0, clk_div_i};
            cpol_r          <= cpol_i;
            cpha_r          <= cpha_i;
            quad_en_r       <= quad_en_i;
            cs_auto_r       <= cs_auto_i;
            is_write_r      <= 1'b1;
            busy_r          <= 1'b1;
            start_o         <= 1'b1;
            tx_data_o       <= wdata_i;
            tx_empty_o      <= 1'b0;
            state           <= S_WR_WAIT;
          end
        end

        S_RD_WAIT: begin
          if (done_i) begin
`ifdef XIP_DEBUG
            $display("[XIP] %0t FSM done; popping RX", $time);
`endif
            state <= S_RD_POP;
          end
        end

        S_RD_POP: begin
          fifo_rx_re_o <= 1'b1;       // pop one word
          state        <= S_RD_CAP;
        end

        S_RD_CAP: begin
          rdata_o <= fifo_rx_data_i;  // capture after pop
          state   <= S_RD_RESP;
        end

        S_RD_RESP: begin
          
`ifdef XIP_DEBUG
          if (!rvalid_o) $display("[XIP] %0t RDATA=%08h (valid)", $time, fifo_rx_data_i);
`endif
          if (rvalid_o && rready_i) begin
            busy_r   <= 1'b0;
            state    <= S_IDLE;
          end
        end

        S_WR_WAIT: begin
          if (tx_ren_i)
            tx_empty_o <= 1'b1;
          if (done_i) begin
            bresp_o <= 2'b00;         // OKAY
            state   <= S_WR_RESP;
          end
        end

        S_WR_RESP: begin
          if (bvalid_o && bready_i) begin
            busy_r   <= 1'b0;
            state    <= S_IDLE;
          end
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule
