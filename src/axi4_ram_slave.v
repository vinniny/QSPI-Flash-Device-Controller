// AXI4-Lite style RAM slave used by DMA/integration tests
// Parameterized size and a clean handshake implementation to match testbenches.
module axi4_ram_slave #(
    parameter integer ADDR_WIDTH = 32,
    parameter integer MEM_WORDS  = 16384
)(
    input  wire                 clk,
    input  wire                 resetn,

    // Write address channel
    input  wire [ADDR_WIDTH-1:0] awaddr,
    input  wire                  awvalid,
    output reg                   awready,

    // Write data channel
    input  wire [31:0]           wdata,
    input  wire [3:0]            wstrb,
    input  wire                  wvalid,
    output reg                   wready,

    // Write response channel
    output reg  [1:0]            bresp,
    output reg                   bvalid,
    input  wire                  bready,

    // Read address channel
    input  wire [ADDR_WIDTH-1:0] araddr,
    input  wire                  arvalid,
    output reg                   arready,

    // Read data channel
    output reg  [31:0]           rdata,
    output reg  [1:0]            rresp,
    output reg                   rvalid,
    input  wire                  rready
);

    // Simple little-endian word RAM
    reg [31:0] mem [0:MEM_WORDS-1];

    // Internal latches
    reg [ADDR_WIDTH-1:0] awaddr_q;
    reg [31:0]           wdata_q;
    reg [3:0]            wstrb_q;
    reg                  have_aw;
    reg                  have_w;

    integer i;
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            for (i = 0; i < MEM_WORDS; i = i + 1)
                mem[i] <= 32'hA5A5_0000 + i;
            awready  <= 1'b0;
            wready   <= 1'b0;
            bvalid   <= 1'b0;
            bresp    <= 2'b00;
            arready  <= 1'b0;
            rvalid   <= 1'b0;
            rresp    <= 2'b00;
            rdata    <= 32'h0;
            awaddr_q <= {ADDR_WIDTH{1'b0}};
            have_aw  <= 1'b0;
            have_w   <= 1'b0;
        end else begin
            // Default ready when not holding a transaction
            awready <= !have_aw;
            wready  <= !have_w;
            arready <= !rvalid; // ready if no pending read response

            // Latch AW when accepted
            if (awready && awvalid) begin
                awaddr_q <= awaddr;
                have_aw  <= 1'b1;
            end
            // Latch W when accepted
            if (wready && wvalid) begin
                have_w  <= 1'b1;
                wdata_q <= wdata;
                wstrb_q <= wstrb;
            end
            // Complete write response
            if (bvalid && bready) begin
                bvalid <= 1'b0;
            end

            // Perform write when both address and data are captured
            if (have_aw && have_w && !bvalid) begin
                integer widx;
                widx = awaddr_q[ADDR_WIDTH-1:2];
                if (wstrb_q[0]) mem[widx][7:0]   <= wdata_q[7:0];
                if (wstrb_q[1]) mem[widx][15:8]  <= wdata_q[15:8];
                if (wstrb_q[2]) mem[widx][23:16] <= wdata_q[23:16];
                if (wstrb_q[3]) mem[widx][31:24] <= wdata_q[31:24];
                bresp  <= 2'b00; // OKAY
                bvalid <= 1'b1;
                have_aw<= 1'b0;
                have_w <= 1'b0;
            end

            // Read address handshake
            if (arready && arvalid) begin
                integer ridx;
                ridx   = araddr[ADDR_WIDTH-1:2];
                rdata  <= mem[ridx];
                rresp  <= 2'b00; // OKAY
                rvalid <= 1'b1;
            end
            if (rvalid && rready) begin
                rvalid <= 1'b0;
            end
        end
    end

endmodule
