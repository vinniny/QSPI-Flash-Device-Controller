/*
 * qspi_io.v - Physical IO shifter for QSPI pins
 * Handles tri-state control for IO[3:0] and routes SCLK/CS#.
 */

module qspi_io #(
  parameter integer NUM_IO = 4
)(
  input  wire                   clk,
  input  wire                   resetn,

  // Control from FSM
  input  wire                   sclk_i,
  input  wire                   cs_n_i,
  input  wire [NUM_IO-1:0]      io_oe_i,
  input  wire [NUM_IO-1:0]      out_bits_i,

  // Outputs to pads
  output wire                   sclk_o,
  output wire                   cs_n_o,
  output reg  [NUM_IO-1:0]      input_bits_o,
  inout  wire [NUM_IO-1:0]      io
);

  assign sclk_o = sclk_i;
  assign cs_n_o = cs_n_i;

  genvar i;
  generate
    for (i = 0; i < NUM_IO; i = i + 1) begin : gen_io
      assign io[i] = io_oe_i[i] ? out_bits_i[i] : 1'bz;
    end
  endgenerate

  always @(posedge clk) begin
    if (!resetn)
      input_bits_o <= {NUM_IO{1'b0}};
    else
      input_bits_o <= io;
  end

endmodule

