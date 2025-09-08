// fifo_rx.v - Simple synchronous RX FIFO (non-FWFT)
module fifo_rx #(
  parameter integer WIDTH = 32,
  parameter integer DEPTH = 16
)(
  input  wire                   clk,
  input  wire                   resetn,

  // Write port
  input  wire                   wr_en_i,
  input  wire [WIDTH-1:0]       wr_data_i,
  output wire                   full_o,
  output wire [$clog2(DEPTH):0] level_o,

  // Read port
  input  wire                   rd_en_i,
  output wire [WIDTH-1:0]       rd_data_o,
  output wire                   empty_o
);
  reg [WIDTH-1:0] mem [0:DEPTH-1];
  reg [$clog2(DEPTH)-1:0] wr_ptr, rd_ptr;
  reg [$clog2(DEPTH):0]   count;

  assign full_o   = (count == DEPTH[$clog2(DEPTH):0]);
  assign empty_o  = (count == {($clog2(DEPTH)+1){1'b0}});
  reg [WIDTH-1:0] rd_data_r;
  assign rd_data_o = rd_data_r;
  assign level_o   = count;

  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      wr_ptr <= {($clog2(DEPTH)){1'b0}};
      rd_ptr <= {($clog2(DEPTH)){1'b0}};
      count  <= {($clog2(DEPTH)+1){1'b0}};
      rd_data_r <= {WIDTH{1'b0}};
    end else begin
      // write
      if (wr_en_i && !full_o) begin
        mem[wr_ptr] <= wr_data_i;
        wr_ptr <= wr_ptr + {{($clog2(DEPTH)-1){1'b0}},1'b1};
      end
      if (rd_en_i && !empty_o) begin
        rd_data_r <= mem[rd_ptr];
        rd_ptr <= rd_ptr + {{($clog2(DEPTH)-1){1'b0}},1'b1};
      end
      case ({wr_en_i && !full_o, rd_en_i && !empty_o})
        2'b10: count <= count + {{($clog2(DEPTH)){1'b0}},1'b1};
        2'b01: count <= count - {{($clog2(DEPTH)){1'b0}},1'b1};
        default: count <= count;
      endcase
    end
  end
endmodule
