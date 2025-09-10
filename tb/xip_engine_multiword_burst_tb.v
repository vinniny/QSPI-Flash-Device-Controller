`timescale 1ns/1ps

// xip_engine_multiword_burst_tb - Issue multiple sequential AXI reads under XIP
// Uses continuous read + CS hold to simulate multiword fetches.
module xip_engine_multiword_burst_tb;
  reg clk, resetn;

  // AXI-Lite
  reg  [31:0] araddr; reg arvalid; wire arready; wire [31:0] rdata; wire [1:0] rresp; wire rvalid; reg rready;
  // Unused write
  reg  [31:0] awaddr; reg awvalid; wire awready; reg [31:0] wdata; reg [3:0] wstrb; reg wvalid; wire wready; wire [1:0] bresp; wire bvalid; reg bready;

  // XIP config
  reg        xip_en;
  reg [1:0]  xip_addr_bytes; reg [1:0] xip_data_lanes; reg [3:0] xip_dummy;
  reg        xip_cont_read, xip_mode_en, xip_write_en;
  reg [7:0]  xip_read_op, xip_mode_bits, xip_write_op;
  reg [2:0]  clk_div; reg cpol, cpha, quad_en, cs_auto;

  // Interconnect
  wire [31:0] rx_word; wire rx_full, rx_empty; wire [4:0] rx_level;
  wire fsm_done, fsm_tx_ren; wire [31:0] fsm_rx_data; wire fsm_rx_wen;
  wire xip_start; wire [1:0] cmd_lanes_w, addr_lanes_w, data_lanes_w, addr_bytes_w; wire mode_en_w; wire [3:0] dummy_w;
  wire dir_w; wire quad_w; wire cs_auto_w; wire xip_cont_w; wire [7:0] opcode_w, mode_bits_w; wire [31:0] addr_w, len_w, clk_div_w; wire cpol_w, cpha_w; wire [31:0] tx_data_w; wire tx_empty_w; wire fifo_rx_re_w;

  // Pads
  wire sclk, cs_n, io0, io1, io2, io3;

  xip_engine u_xip (
    .clk(clk), .resetn(resetn),
    .xip_en_i(xip_en), .xip_addr_bytes_i(xip_addr_bytes), .xip_data_lanes_i(xip_data_lanes), .xip_dummy_cycles_i(xip_dummy),
    .xip_cont_read_i(xip_cont_read), .xip_mode_en_i(xip_mode_en), .xip_write_en_i(xip_write_en),
    .xip_read_op_i(xip_read_op), .xip_mode_bits_i(xip_mode_bits), .xip_write_op_i(xip_write_op),
    .clk_div_i(clk_div), .cpol_i(cpol), .cpha_i(cpha), .quad_en_i(quad_en), .cs_auto_i(cs_auto), .cmd_busy_i(1'b0),
    .awaddr_i(awaddr), .awvalid_i(awvalid), .awready_o(awready),
    .wdata_i(wdata), .wstrb_i(wstrb), .wvalid_i(wvalid), .wready_o(wready),
    .bresp_o(bresp), .bvalid_o(bvalid), .bready_i(bready),
    .araddr_i(araddr), .arvalid_i(arvalid), .arready_o(arready),
    .rdata_o(rdata), .rresp_o(rresp), .rvalid_o(rvalid), .rready_i(rready),
    .fifo_rx_data_i(rx_word), .fifo_rx_re_o(fifo_rx_re_w),
    .start_o(xip_start), .done_i(fsm_done), .tx_ren_i(fsm_tx_ren), .tx_data_o(tx_data_w), .tx_empty_o(tx_empty_w),
    .cmd_lanes_o(cmd_lanes_w), .addr_lanes_o(addr_lanes_w), .data_lanes_o(data_lanes_w), .addr_bytes_o(addr_bytes_w),
    .mode_en_o(mode_en_w), .dummy_cycles_o(dummy_w), .dir_o(dir_w), .quad_en_o(quad_w), .cs_auto_o(cs_auto_w), .xip_cont_read_o(xip_cont_w),
    .opcode_o(opcode_w), .mode_bits_o(mode_bits_w), .addr_o(addr_w), .len_o(len_w), .clk_div_o(clk_div_w), .cpol_o(cpol_w), .cpha_o(cpha_w),
    .busy_o(), .xip_active_o()
  );

  fifo_rx #(.WIDTH(32), .DEPTH(16)) u_frx (
    .clk(clk), .resetn(resetn), .wr_en_i(fsm_rx_wen), .wr_data_i(fsm_rx_data),
    .rd_en_i(fifo_rx_re_w), .rd_data_o(rx_word), .full_o(rx_full), .empty_o(rx_empty), .level_o(rx_level)
  );

  qspi_fsm u_fsm (
    .clk(clk), .resetn(resetn), .start(xip_start), .done(fsm_done),
    .cmd_lanes_sel(cmd_lanes_w), .addr_lanes_sel(addr_lanes_w), .data_lanes_sel(data_lanes_w), .addr_bytes_sel(addr_bytes_w),
    .mode_en(mode_en_w), .dummy_cycles(dummy_w), .dir(dir_w), .quad_en(quad_w), .cs_auto(cs_auto_w), .cs_delay(2'b00), .xip_cont_read(xip_cont_w),
    .cmd_opcode(opcode_w), .mode_bits(mode_bits_w), .addr(addr_w), .len_bytes(len_w),
    .clk_div(clk_div_w), .cpol(cpol_w), .cpha(cpha_w),
    .tx_data_fifo(tx_data_w), .tx_empty(tx_empty_w), .tx_ren(fsm_tx_ren),
    .rx_data_fifo(fsm_rx_data), .rx_wen(fsm_rx_wen), .rx_full(rx_full),
    .sclk(sclk), .cs_n(cs_n), .io0(io0), .io1(io1), .io2(io2), .io3(io3)
  );

  qspi_device dev (
    .qspi_sclk(sclk), .qspi_cs_n(cs_n), .qspi_io0(io0), .qspi_io1(io1), .qspi_io2(io2), .qspi_io3(io3)
  );

  // Clock/reset
  initial clk=0; always #5 clk=~clk;

  task axi_read(input [31:0] addr, output [31:0] data);
  begin
    araddr <= addr; arvalid <= 1'b1; rready <= 1'b0;
    @(posedge clk); while(!arready) @(posedge clk);
    arvalid <= 1'b0;
    while(!rvalid) @(posedge clk);
    @(posedge clk); data = rdata;
    rready <= 1'b1; @(posedge clk); rready <= 1'b0;
  end endtask

  integer i; reg [31:0] d[0:7];
  initial begin
    $dumpfile("xip_engine_multiword_burst_tb.vcd"); $dumpvars(0, xip_engine_multiword_burst_tb);
    resetn=0; araddr=0; arvalid=0; rready=0; awaddr=0; awvalid=0; wdata=0; wstrb=0; wvalid=0; bready=1;
    // 1-1-1 fast read, continuous-read enabled, hold CS (cs_auto=0)
    xip_en=0; xip_addr_bytes=2'b01; xip_data_lanes=2'b00; xip_dummy=4'd8; xip_cont_read=1'b1; xip_mode_en=1'b0; xip_write_en=1'b0;
    xip_read_op=8'h0B; xip_mode_bits=8'h00; xip_write_op=8'h02; clk_div=3'd0; cpol=0; cpha=0; quad_en=0; cs_auto=0;
    repeat(8) @(posedge clk); resetn=1;

    xip_en=1'b1; repeat(4) @(posedge clk);
    for (i=0; i<8; i=i+1) begin
      axi_read(32'h0000_0000 + (i<<2), d[i]);
      if (d[i]!==32'hFFFF_FFFF && d[i]!==32'h7FFF_FFFF)
        $fatal(1, "XIP burst word[%0d] mismatch: %08h", i, d[i]);
    end
    $display("xip_engine_multiword_burst_tb: PASS (test passed)");
    $finish;
  end

  initial begin #1_000_000; $display("[xip_engine_multiword_burst_tb] Global timeout reached — finishing."); $finish; end
endmodule

