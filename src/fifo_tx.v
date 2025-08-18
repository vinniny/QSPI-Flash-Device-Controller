/*
 * fifo_tx.v - Transmit FIFO buffering data from APB/DMA to QSPI FSM
 */

module fifo_tx #(
  parameter integer WIDTH = 32,
  parameter integer DEPTH = 16
) (
  input  wire                   clk,
  input  wire                   resetn,

  // Write interface from CSR/DMA
  input  wire                   wr_en_i,
  input  wire [WIDTH-1:0]       wr_data_i,

  // Read interface to FSM
  input  wire                   rd_en_i,
  output reg  [WIDTH-1:0]       rd_data_o,

  // Status outputs
  output wire                   full_o,
  output wire                   empty_o,
  output reg  [$clog2(DEPTH+1)-1:0] level_o
);

  localparam integer AW = $clog2(DEPTH);
  localparam [AW:0] DEPTH_VAL = DEPTH[AW:0];

  reg [WIDTH-1:0]          mem [0:DEPTH-1];
  reg [AW-1:0]             wptr;
  reg [AW-1:0]             rptr;
  reg [AW:0]               count;

  assign full_o  = (count == DEPTH_VAL);
  assign empty_o = (count == {(AW+1){1'b0}});

  always @(posedge clk) begin
    if (!resetn) begin
      wptr     <= {AW{1'b0}};
      rptr     <= {AW{1'b0}};
      count    <= {(AW+1){1'b0}};
      rd_data_o <= {WIDTH{1'b0}};
      level_o  <= {(AW+1){1'b0}};
    end else begin
      // Write operation
      if (wr_en_i && !full_o) begin
        mem[wptr] <= wr_data_i;
        wptr      <= wptr + 1'b1;
      end

      // Read operation
      if (rd_en_i && !empty_o) begin
        rd_data_o <= mem[rptr];
        rptr      <= rptr + 1'b1;
      end

      // Count update
      case ({wr_en_i && !full_o, rd_en_i && !empty_o})
        2'b10: count <= count + 1'b1;
        2'b01: count <= count - 1'b1;
        default: count <= count;
      endcase

      level_o <= count;
    end
  end

endmodule

