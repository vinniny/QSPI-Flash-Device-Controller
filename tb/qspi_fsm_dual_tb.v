`timescale 1ns/1ps

// Minimal standalone dual-lane command test to verify FSM accepts a fresh
// start after DONE, with no waits on CS or other dependencies.
module qspi_fsm_dual_tb;
  reg clk;
  reg resetn;
  reg start;
  wire done;

  // Config
  reg [1:0] cmd_lanes_sel;
  reg [1:0] addr_lanes_sel;
  reg [1:0] data_lanes_sel;
  reg [1:0] addr_bytes_sel;
  reg       mode_en;
  reg [3:0] dummy_cycles;
  reg       dir;
  reg [7:0] cmd_opcode;
  reg [7:0] mode_bits;
  reg [31:0] addr;
  reg [31:0] len_bytes;
  reg [31:0] clk_div;
  reg        cpol;
  reg        cpha;
  reg [31:0] tx_data_fifo;
  wire       tx_ren;
  reg        tx_empty;
  wire [31:0] rx_data_fifo;
  wire       rx_wen;
  reg        rx_full;
  wire       sclk;
  wire       cs_n;
  wire       io0, io1, io2, io3;

  qspi_fsm dut (
    .clk(clk), .resetn(resetn),
    .start(start), .done(done),
    .cmd_lanes_sel(cmd_lanes_sel), .addr_lanes_sel(addr_lanes_sel), .data_lanes_sel(data_lanes_sel),
    .addr_bytes_sel(addr_bytes_sel), .mode_en(mode_en), .dummy_cycles(dummy_cycles), .dir(dir),
    .quad_en(1'b0), .cs_auto(1'b1), .xip_cont_read(1'b0),
    .cmd_opcode(cmd_opcode), .mode_bits(mode_bits), .addr(addr), .len_bytes(len_bytes),
    .clk_div(clk_div), .cpol(cpol), .cpha(cpha),
    .tx_data_fifo(tx_data_fifo), .tx_ren(tx_ren), .tx_empty(tx_empty),
    .rx_data_fifo(rx_data_fifo), .rx_wen(rx_wen), .rx_full(rx_full),
    .sclk(sclk), .cs_n(cs_n), .io0(io0), .io1(io1), .io2(io2), .io3(io3)
  );

  initial clk=0; always #5 clk=~clk;

  initial begin
    $dumpfile("qspi_fsm_dual_tb.vcd");
    $dumpvars(0, qspi_fsm_dual_tb);
    resetn=0; start=0; cmd_lanes_sel=2'b00; addr_lanes_sel=2'b00; data_lanes_sel=2'b00;
    addr_bytes_sel=2'b00; mode_en=1'b0; dummy_cycles=4'd0; dir=1'b0; cmd_opcode=8'h00; mode_bits=8'h00;
    addr=32'h0; len_bytes=32'h0; clk_div=32'h0; cpol=1'b0; cpha=1'b0; tx_data_fifo=32'h0; tx_empty=1'b1; rx_full=1'b0;
    repeat (4) @(posedge clk); resetn=1;

    // First command: single-lane WREN to exercise a basic cycle
    cmd_lanes_sel=2'b00; cmd_opcode=8'h06; dir=1'b0; len_bytes=0; addr_bytes_sel=0; dummy_cycles=0;
    @(posedge clk); start<=1; @(posedge clk); start<=0;
    wait(done);
    $display("[dual_tb] First DONE at %0t cs_n=%b", $time, cs_n);
    repeat (2) @(posedge clk);

    // Second command: dual-lane opcode 0xAB, no addr/data
    cmd_lanes_sel=2'b01; cmd_opcode=8'hAB; dir=1'b0; len_bytes=0; addr_bytes_sel=0; dummy_cycles=0;
    @(posedge clk); start<=1; @(posedge clk); start<=0;
    wait(done);
    $display("[dual_tb] Second DONE at %0t cs_n=%b", $time, cs_n);

    $display("qspi_fsm_dual_tb: PASS");
    $finish;
  end

  initial begin
    #1_000_000; $display("[dual_tb] Global timeout"); $finish; end
endmodule

