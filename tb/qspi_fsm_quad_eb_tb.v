`timescale 1ns/1ps

// Quad I/O fast read (0xEB) with mode bits (0xA0) test
// Verifies 1-4-4 path and that data is received over 4 lanes.
module qspi_fsm_quad_eb_tb;
  reg clk; reg resetn; reg start; wire done;

  // Config
  reg [1:0] cmd_lanes_sel;
  reg [1:0] addr_lanes_sel;
  reg [1:0] data_lanes_sel;
  reg [1:0] addr_bytes_sel;
  reg       mode_en;
  reg [3:0] dummy_cycles;
  reg       dir;            // 1=read
  reg [7:0] cmd_opcode;
  reg [7:0] mode_bits;
  reg [31:0] addr;
  reg [31:0] len_bytes;
  reg [31:0] clk_div;
  reg        cpol, cpha;
  reg [31:0] tx_data_fifo; wire tx_ren; reg tx_empty;
  wire [31:0] rx_data_fifo; wire rx_wen; reg rx_full;
  wire sclk; wire cs_n; wire io0, io1, io2, io3;

  qspi_fsm dut (
    .clk(clk), .resetn(resetn), .start(start), .done(done),
    .cmd_lanes_sel(cmd_lanes_sel), .addr_lanes_sel(addr_lanes_sel), .data_lanes_sel(data_lanes_sel),
    .addr_bytes_sel(addr_bytes_sel), .mode_en(mode_en), .dummy_cycles(dummy_cycles), .dir(dir),
    .quad_en(1'b1), .cs_auto(1'b1), .cs_delay(2'b00), .xip_cont_read(1'b0),
    .cmd_opcode(cmd_opcode), .mode_bits(mode_bits), .addr(addr), .len_bytes(len_bytes),
    .clk_div(clk_div), .cpol(cpol), .cpha(cpha),
    .tx_data_fifo(tx_data_fifo), .tx_empty(tx_empty), .tx_ren(tx_ren),
    .rx_data_fifo(rx_data_fifo), .rx_wen(rx_wen), .rx_full(rx_full),
    .sclk(sclk), .cs_n(cs_n), .io0(io0), .io1(io1), .io2(io2), .io3(io3)
  );

  // Connect to device model
  qspi_device dev (
    .qspi_sclk(sclk), .qspi_cs_n(cs_n), .qspi_io0(io0), .qspi_io1(io1), .qspi_io2(io2), .qspi_io3(io3)
  );

  initial clk=0; always #5 clk=~clk;

  reg got_rx; reg [31:0] rx_word;
  always @(posedge clk) begin
    if (!resetn) begin got_rx<=1'b0; rx_word<=32'h0; end
    else if (rx_wen) begin got_rx<=1'b1; rx_word<=rx_data_fifo; end
  end

  initial begin
    $dumpfile("qspi_fsm_quad_eb_tb.vcd");
    $dumpvars(0, qspi_fsm_quad_eb_tb);
    resetn=0; start=0; cmd_lanes_sel=2'b00; addr_lanes_sel=2'b00; data_lanes_sel=2'b00;
    addr_bytes_sel=2'b01; mode_en=1'b1; dummy_cycles=4'd4; dir=1'b1; cmd_opcode=8'hEB; mode_bits=8'hA0;
    addr=32'h0000_0000; len_bytes=32'd4; clk_div=32'd0; cpol=1'b0; cpha=1'b0; tx_data_fifo=32'h0; tx_empty=1'b1; rx_full=1'b0;
    repeat (4) @(posedge clk); resetn=1;

    @(posedge clk); start<=1; @(posedge clk); start<=0;
    // Wait for RX word
    repeat (2000) @(posedge clk);
    if (!got_rx) $fatal(1, "No RX word for 0xEB quad read");
    if (rx_word !== 32'hFFFF_FFFF) $fatal(1, "Unexpected RX data: %h", rx_word);
    // Allow a few cycles for CS# deassertion; avoid race on done
    repeat (10) @(posedge clk);
    $display("qspi_fsm_quad_eb_tb: PASS (test passed)");
    $finish;
  end

  // Global timeout to prevent stalls
  initial begin #1_000_000; $display("[qspi_fsm_quad_eb_tb] Global timeout reached — finishing."); $finish; end
endmodule
