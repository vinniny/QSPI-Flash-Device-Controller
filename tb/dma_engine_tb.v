`timescale 1ns/1ps

module dma_engine_tb;
  // Clock/reset
  reg clk;
  reg resetn;

  // DMA config
  reg        dma_en;
  reg        dma_dir;     // 0: mem->TX FIFO, 1: RX FIFO->mem
  reg  [3:0] burst;
  reg        incr;
  reg  [31:0] dma_addr;
  reg  [31:0] dma_len;

  // FIFO side
  reg  [4:0] tx_level;
  wire [31:0] tx_data;
  wire        tx_we;
  reg  [4:0] rx_level;
  reg  [31:0] rx_data;
  wire        rx_re;

  // AXI master wires
  wire [31:0] awaddr; wire awvalid; wire awready;
  wire [31:0] wdata;  wire wvalid;  wire [3:0] wstrb; wire wready;
  wire        bvalid; wire [1:0] bresp; wire bready;
  wire [31:0] araddr; wire arvalid; wire arready;
  wire [31:0] rdata;  wire rvalid;  wire [1:0] rresp; wire rready;

  wire dma_done;
  wire axi_err;
  wire busy;

  dma_engine #(.ADDR_WIDTH(32), .TX_FIFO_DEPTH(8), .LEVEL_WIDTH(5)) dut (
    .clk(clk), .resetn(resetn),
    .dma_en_i(dma_en), .dma_dir_i(dma_dir), .burst_size_i(burst), .incr_addr_i(incr),
    .dma_addr_i(dma_addr), .dma_len_i(dma_len),
    .tx_level_i(tx_level), .fifo_tx_data_o(tx_data), .fifo_tx_we_o(tx_we),
    .rx_level_i(rx_level), .fifo_rx_data_i(rx_data), .fifo_rx_re_o(rx_re),
    .dma_done_set_o(dma_done), .axi_err_o(axi_err), .busy_o(busy),
    .awaddr_o(awaddr), .awvalid_o(awvalid), .awready_i(awready),
    .wdata_o(wdata), .wvalid_o(wvalid), .wstrb_o(wstrb), .wready_i(wready),
    .bvalid_i(bvalid), .bresp_i(bresp), .bready_o(bready),
    .araddr_o(araddr), .arvalid_o(arvalid), .arready_i(arready),
    .rdata_i(rdata), .rvalid_i(rvalid), .rresp_i(rresp), .rready_o(rready)
  );

  // AXI RAM slave
  axi4_ram_slave #(.MEM_WORDS(64)) ram (
    .clk(clk), .resetn(resetn),
    .awaddr(awaddr), .awvalid(awvalid), .awready(awready),
    .wdata(wdata), .wstrb(wstrb), .wvalid(wvalid), .wready(wready),
    .bresp(bresp), .bvalid(bvalid), .bready(bready),
    .araddr(araddr), .arvalid(arvalid), .arready(arready),
    .rdata(rdata), .rresp(rresp), .rvalid(rvalid), .rready(rready)
  );

  // Clock
  initial clk = 0; always #5 clk = ~clk;

  // VCD
  initial begin
    $dumpfile("dma_engine_tb.vcd");
    $dumpvars(0, dma_engine_tb);
  end

  // Global timeout to prevent stalls
  initial begin
    #1_000_000; // 1 ms cutoff
    $display("[dma_engine_tb] Global timeout reached — finishing.");
    $finish;
  end

  // Utilities
  task kick_dma(input [31:0] addr, input [31:0] len, input dir, input [3:0] bsz, input inc);
  begin
    dma_addr <= addr; dma_len <= len; dma_dir <= dir; burst <= bsz; incr <= inc;
    @(posedge clk); dma_en <= 1; @(posedge clk); dma_en <= 0;
  end
  endtask

  // Capture buffer for mem->TX FIFO path
  reg [31:0] cap [0:7]; integer wcnt;

  // Feed for RX FIFO->mem path
  reg [31:0] src [0:7]; integer rcnt;

  always @(posedge clk) begin
    if (tx_we) begin
      cap[wcnt] <= tx_data; wcnt <= wcnt + 1;
    end
    // FWFT-like behavior for RX: present head word continuously
    rx_data <= src[rcnt];
    if (rx_re) begin
      if (rx_level != 0) begin
        rx_level <= rx_level - 1;
        rcnt <= rcnt + 1;
      end
    end
  end

  integer i;
  reg [31:0] tmp;

  initial begin
    resetn=0; dma_en=0; dma_dir=0; burst=0; incr=1; dma_addr=0; dma_len=0;
    tx_level=0; rx_level=0; rx_data=0; wcnt=0; rcnt=0;
    for (i=0;i<8;i=i+1) begin cap[i]=0; src[i]=0; end
    repeat (5) @(posedge clk); resetn=1;

    // Preload RAM with pattern
    ram.mem[0]=32'h0102_0304; ram.mem[1]=32'h1112_1314; ram.mem[2]=32'h2122_2324; ram.mem[3]=32'h3132_3334;

    // Case 1: mem -> TX FIFO (dir=0) length=16
    kick_dma(32'h0000_0000, 32'd16, 1'b0, 4'd4, 1'b1);
    wait (dma_done);
    if (wcnt != 4) $fatal(1, "mem->tx: wrong word count %0d", wcnt);
    if (cap[0]!==ram.mem[0] || cap[1]!==ram.mem[1] || cap[2]!==ram.mem[2] || cap[3]!==ram.mem[3])
      $fatal(1, "mem->tx: data mismatch");

    // Case 2: RX FIFO -> mem (dir=1) length=16
    src[0]=32'hA5A5_A5A5; src[1]=32'h5A5A_5A5A; src[2]=32'hDEAD_BEEF; src[3]=32'hC0DE_CAFE;
    rx_level=4; rcnt=0;
    kick_dma(32'h0000_0010, 32'd16, 1'b1, 4'd4, 1'b1);
    wait (dma_done);
    if (ram.mem[4]!==32'hA5A5_A5A5 || ram.mem[5]!==32'h5A5A_5A5A || ram.mem[6]!==32'hDEAD_BEEF || ram.mem[7]!==32'hC0DE_CAFE)
      $fatal(1, "rx->mem: data mismatch");

    $display("DMA engine test passed");
    $finish;
  end
endmodule
