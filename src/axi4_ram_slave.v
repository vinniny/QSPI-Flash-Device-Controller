/*
 * axi4_ram_slave.v - Simple AXI4-Lite RAM slave
 *
 * - 32-bit data bus, word-aligned accesses
 * - Single outstanding transaction per channel
 * - Supports byte strobes on writes
 * - Synchronous active-low reset
 */

module axi4_ram_slave #(
  parameter integer ADDR_WIDTH = 32,
  parameter integer MEM_WORDS  = 1024
)(
  input  wire                    clk,
  input  wire                    resetn,

  // Write address channel
  input  wire [ADDR_WIDTH-1:0]   awaddr,
  input  wire                    awvalid,
  output reg                     awready,

  // Write data channel
  input  wire [31:0]             wdata,
  input  wire [3:0]              wstrb,
  input  wire                    wvalid,
  output reg                     wready,

  // Write response channel
  output reg  [1:0]              bresp,
  output reg                     bvalid,
  input  wire                    bready,

  // Read address channel
  input  wire [ADDR_WIDTH-1:0]   araddr,
  input  wire                    arvalid,
  output reg                     arready,

  // Read data channel
  output reg  [31:0]             rdata,
  output reg  [1:0]              rresp,
  output reg                     rvalid,
  input  wire                    rready
);

  // ------------------------------------------------------------------
  // Internal memory
  // ------------------------------------------------------------------
  reg [31:0] mem [0:MEM_WORDS-1];
  reg [ADDR_WIDTH-1:0] wr_addr_q;
  reg [ADDR_WIDTH-1:0] rd_addr_q;
  reg [31:0] cur;
  reg aw_captured;

  // ------------------------------------------------------------------
  // Write channel
  // ------------------------------------------------------------------
  always @(posedge clk) begin
    if (!resetn) begin
      awready   <= 1'b0;
      wready    <= 1'b0;
      bvalid    <= 1'b0;
      bresp     <= 2'b00;
      wr_addr_q <= {ADDR_WIDTH{1'b0}};
      aw_captured <= 1'b0;
    end else begin
      // Default deassert
      awready <= 1'b0;
      wready  <= 1'b0;

      // Address accept
      if (awvalid && !awready && !aw_captured) begin
        awready    <= 1'b1;
        wr_addr_q  <= {awaddr[ADDR_WIDTH-1:2], 2'b00};
        aw_captured<= 1'b1;
      end

      // Data accept and write
      if (wvalid && !wready && aw_captured && !bvalid) begin
        wready <= 1'b1;
        // byte strobes
        cur = mem[wr_addr_q[ADDR_WIDTH-1:2]];
        if (wstrb[0]) cur[7:0]   = wdata[7:0];
        if (wstrb[1]) cur[15:8]  = wdata[15:8];
        if (wstrb[2]) cur[23:16] = wdata[23:16];
        if (wstrb[3]) cur[31:24] = wdata[31:24];
        mem[wr_addr_q[ADDR_WIDTH-1:2]] <= cur;
        bvalid <= 1'b1;
        bresp  <= 2'b00; // OKAY
      end

      // Response handshake
      if (bvalid && bready) begin
        bvalid      <= 1'b0;
        aw_captured <= 1'b0;
      end
    end
  end

  // ------------------------------------------------------------------
  // Read channel
  // ------------------------------------------------------------------
  always @(posedge clk) begin
    if (!resetn) begin
      arready   <= 1'b0;
      rvalid    <= 1'b0;
      rresp     <= 2'b00;
      rdata     <= 32'h0;
      rd_addr_q <= {ADDR_WIDTH{1'b0}};
    end else begin
      arready <= 1'b0;

      if (arvalid && !arready && !rvalid) begin
        arready   <= 1'b1;
        rd_addr_q <= {araddr[ADDR_WIDTH-1:2], 2'b00};
        rdata     <= mem[araddr[ADDR_WIDTH-1:2]];
        rresp     <= 2'b00; // OKAY
        rvalid    <= 1'b1;
      end

      if (rvalid && rready) begin
        rvalid <= 1'b0;
      end
    end
  end

endmodule
