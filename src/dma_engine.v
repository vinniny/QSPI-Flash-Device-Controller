/*
 * dma_engine.v - AXI4-Lite DMA engine
 *
 * Moves data between system memory and TX/RX FIFOs. Direction is
 * selected by dma_dir_i: 0=read from memory to TX FIFO (flash write),
 * 1=write from RX FIFO to memory (flash read).
 *
 * Transfers are performed in bursts using the helper modules
 * axi_read_block and axi_write_block. Burst length and address
 * increment behavior are configurable. The engine waits on FIFO levels
 * to avoid overruns or underruns and raises dma_done_set_o when all
 * bytes have been transferred. AXI errors set axi_err_o.
 */

module dma_engine #(
  parameter integer ADDR_WIDTH    = 32,
  parameter integer TX_FIFO_DEPTH = 16,
  parameter integer LEVEL_WIDTH   = 5
)(
  input  wire                    clk,
  input  wire                    resetn,

  // CSR control/configuration
  input  wire                    dma_en_i,
  input  wire                    dma_dir_i,
  input  wire [3:0]              burst_size_i,
  input  wire                    incr_addr_i,
  input  wire [ADDR_WIDTH-1:0]   dma_addr_i,
  input  wire [31:0]             dma_len_i,

  // FIFO interfaces
  input  wire [LEVEL_WIDTH-1:0]  tx_level_i,
  output wire [31:0]            fifo_tx_data_o,
  output wire                   fifo_tx_we_o,
  input  wire [LEVEL_WIDTH-1:0]  rx_level_i,
  input  wire [31:0]            fifo_rx_data_i,
  output wire                   fifo_rx_re_o,

  // Status outputs
  output reg                    dma_done_set_o,
  output reg                    axi_err_o,
  output wire                   busy_o,

  // AXI4-Lite master interface
  // Write address channel
  output wire [ADDR_WIDTH-1:0]  awaddr_o,
  output wire                   awvalid_o,
  input  wire                   awready_i,
  // Write data channel
  output wire [31:0]            wdata_o,
  output wire                   wvalid_o,
  output wire [3:0]             wstrb_o,
  input  wire                   wready_i,
  // Write response channel
  input  wire                   bvalid_i,
  input  wire [1:0]             bresp_i,
  output wire                   bready_o,
  // Read address channel
  output wire [ADDR_WIDTH-1:0]  araddr_o,
  output wire                   arvalid_o,
  input  wire                   arready_i,
  // Read data channel
  input  wire [31:0]            rdata_i,
  input  wire                   rvalid_i,
  input  wire [1:0]             rresp_i,
  output wire                   rready_o
);

  // ------------------------------------------------------------
  // Derived constants and FIFO status
  // ------------------------------------------------------------
  localparam [LEVEL_WIDTH-1:0] TX_DEPTH_LEVEL = TX_FIFO_DEPTH[LEVEL_WIDTH-1:0];

  wire tx_full  = (tx_level_i == TX_DEPTH_LEVEL);
  wire rx_empty = (rx_level_i == {LEVEL_WIDTH{1'b0}});

  // ------------------------------------------------------------
  // DMA configuration registers
  // ------------------------------------------------------------
  reg                     dir_r;
  reg                     incr_addr_r;
  reg [3:0]               burst_size_r;
  reg [ADDR_WIDTH-1:0]    addr_r;
  reg [31:0]              rem_bytes_r;
  reg                     busy_r;
  reg [31:0]              burst_len_r;

  assign busy_o = busy_r;

  // detect rising edge on dma_en_i
  reg dma_en_d;
  wire start_pulse = dma_en_i & ~dma_en_d;
  always @(posedge clk) begin
    if (!resetn)
      dma_en_d <= 1'b0;
    else
      dma_en_d <= dma_en_i;
  end

  // ------------------------------------------------------------
  // Burst computation based on remaining bytes and burst_size
  // ------------------------------------------------------------
  wire [31:0] rem_words_w   = rem_bytes_r >> 2;
  wire [3:0]  burst_words_w = (burst_size_r == 4'd0) ? 4'd1 : burst_size_r;
  wire        rem_lt_burst  = (rem_words_w < burst_words_w);
  wire [3:0]  beats_w       = rem_lt_burst ? rem_words_w[3:0] : burst_words_w;
  wire [31:0] len_w         = {28'd0, beats_w} << 2;
  wire [LEVEL_WIDTH-1:0] beats_level = {{(LEVEL_WIDTH-4){1'b0}}, beats_w};
  wire tx_space_ok = (tx_level_i <= (TX_DEPTH_LEVEL - beats_level));
  wire rx_data_ok  = (rx_level_i >= beats_level);

  // ------------------------------------------------------------
  // AXI helper instances
  // ------------------------------------------------------------
  reg rd_start, wr_start;
  wire rd_done, wr_done;
  wire [ADDR_WIDTH-1:0] araddr_w, awaddr_w;
  wire arvalid_w, rready_w;
  wire awvalid_w, wvalid_w, bready_w;
  wire [31:0] data_out_w, wdata_w;
  wire [3:0]  wstrb_w;
  wire        wr_en_w, rd_en_w;

  axi_read_block u_axi_read_block (
    .clk          (clk),
    .reset        (~resetn),
    .start        (rd_start),
    .addr         (addr_r),
    .transfer_size(burst_len_r[15:0]),
    .araddr       (araddr_w),
    .arvalid      (arvalid_w),
    .arready      (arready_i),
    .rvalid       (rvalid_i),
    .rdata        (rdata_i),
    .rready       (rready_w),
    .data_out     (data_out_w),
    .wr_en        (wr_en_w),
    .full         (tx_full),
    .busy         (),
    .done         (rd_done)
  );

  axi_write_block u_axi_write_block (
    .clk          (clk),
    .reset        (~resetn),
    .start        (wr_start),
    .addr         (addr_r),
    .transfer_size(burst_len_r[15:0]),
    .awaddr       (awaddr_w),
    .awvalid      (awvalid_w),
    .awready      (awready_i),
    .wdata        (wdata_w),
    .wvalid       (wvalid_w),
    .wstrb        (wstrb_w),
    .wready       (wready_i),
    .bvalid       (bvalid_i),
    .bready       (bready_w),
    .data_in      (fifo_rx_data_i),
    .empty        (rx_empty),
    .rd_en        (rd_en_w),
    .busy         (),
    .done         (wr_done)
  );

  assign araddr_o       = araddr_w;
  assign arvalid_o      = arvalid_w;
  assign rready_o       = rready_w;
  assign fifo_tx_data_o = data_out_w;
  assign fifo_tx_we_o   = wr_en_w;

  assign awaddr_o       = awaddr_w;
  assign awvalid_o      = awvalid_w;
  assign wdata_o        = wdata_w;
  assign wvalid_o       = wvalid_w;
  assign wstrb_o        = wstrb_w;
  assign bready_o       = bready_w;
  assign fifo_rx_re_o   = rd_en_w;

  // ------------------------------------------------------------
  // DMA state machine
  // ------------------------------------------------------------
  localparam S_IDLE    = 3'd0,
             S_WAIT_RD = 3'd1,
             S_RUN_RD  = 3'd2,
             S_WAIT_WR = 3'd3,
             S_RUN_WR  = 3'd4,
             S_DONE    = 3'd5;

  reg [2:0] state;

  always @(posedge clk) begin
    if (!resetn) begin
      state           <= S_IDLE;
      dma_done_set_o  <= 1'b0;
      axi_err_o       <= 1'b0;
      rd_start        <= 1'b0;
      wr_start        <= 1'b0;
      busy_r          <= 1'b0;
      dir_r           <= 1'b0;
      incr_addr_r     <= 1'b0;
      burst_size_r    <= 4'd0;
      addr_r          <= {ADDR_WIDTH{1'b0}};
      rem_bytes_r     <= 32'd0;
      burst_len_r     <= 32'd0;
    end else begin
      dma_done_set_o <= 1'b0;
      rd_start       <= 1'b0;
      wr_start       <= 1'b0;

      case (state)
        S_IDLE: begin
          axi_err_o <= 1'b0;
          if (start_pulse) begin
            dir_r        <= dma_dir_i;
            incr_addr_r  <= incr_addr_i;
            burst_size_r <= burst_size_i;
            addr_r       <= dma_addr_i;
            rem_bytes_r  <= dma_len_i;
            busy_r       <= 1'b1;
            state        <= dma_dir_i ? S_WAIT_WR : S_WAIT_RD;
          end
        end

        S_WAIT_RD: begin
          if (rem_bytes_r == 0) begin
            state <= S_DONE;
          end else if (tx_space_ok) begin
            burst_len_r <= len_w;
            rd_start    <= 1'b1;
            state       <= S_RUN_RD;
          end
        end

        S_RUN_RD: begin
          if (rvalid_i && rready_o && rresp_i[1])
            axi_err_o <= 1'b1;
          if (rd_done) begin
            if (incr_addr_r)
              addr_r <= addr_r + burst_len_r;
            if (axi_err_o || (rem_bytes_r <= burst_len_r)) begin
              rem_bytes_r <= 32'd0;
              state       <= S_DONE;
            end else begin
              rem_bytes_r <= rem_bytes_r - burst_len_r;
              state       <= S_WAIT_RD;
            end
          end
        end

        S_WAIT_WR: begin
          if (rem_bytes_r == 0) begin
            state <= S_DONE;
          end else if (rx_data_ok) begin
            burst_len_r <= len_w;
            wr_start    <= 1'b1;
            state       <= S_RUN_WR;
          end
        end

        S_RUN_WR: begin
          if (bvalid_i && bready_o && bresp_i[1])
            axi_err_o <= 1'b1;
          if (wr_done) begin
            if (incr_addr_r)
              addr_r <= addr_r + burst_len_r;
            if (axi_err_o || (rem_bytes_r <= burst_len_r)) begin
              rem_bytes_r <= 32'd0;
              state       <= S_DONE;
            end else begin
              rem_bytes_r <= rem_bytes_r - burst_len_r;
              state       <= S_WAIT_WR;
            end
          end
        end

        S_DONE: begin
          dma_done_set_o <= 1'b1;
          busy_r        <= 1'b0;
          state         <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule

