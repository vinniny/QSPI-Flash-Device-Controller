module apb_master (
    input  wire       clk,
    input  wire       rst_n,
    // APB interface
    output reg        psel,
    output reg        penable,
    output reg        pwrite,
    output reg [11:0] paddr,
    output reg [31:0] pwdata,
    input  wire [31:0] prdata,
    input  wire       pready,
    // Control interface (single-transaction)
    input  wire       start,
    input  wire       rw,    // 0 = read, 1 = write
    input  wire [11:0] addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output wire       idle,
    output wire       busy
);

  localparam [1:0]
    IDLE_S   = 2'b00,
    SETUP_S  = 2'b01,
    ACCESS_S = 2'b10;

  reg [1:0] state;
  reg       start_q;

  assign idle = (state == IDLE_S);
  assign busy = ~idle;

  // Latch control inputs on start edge to avoid changes mid-transfer
  reg        rw_l;
  reg [11:0] addr_l;
  reg [31:0] wdata_l;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state   <= IDLE_S;
      start_q <= 1'b0;
      psel    <= 1'b0;
      penable <= 1'b0;
      pwrite  <= 1'b0;
      paddr   <= 12'd0;
      pwdata  <= 32'd0;
      rdata   <= 32'd0;
      rw_l    <= 1'b0;
      addr_l  <= 12'd0;
      wdata_l <= 32'd0;
    end else begin
      start_q <= start;

      case (state)
        IDLE_S: begin
          psel    <= 1'b0;
          penable <= 1'b0;
          if (start & ~start_q) begin // rising edge
            // latch request
            rw_l    <= rw;
            addr_l  <= addr;
            wdata_l <= wdata;
            // drive setup
            psel    <= 1'b1;
            penable <= 1'b0;
            pwrite  <= rw;
            paddr   <= addr;
            pwdata  <= wdata;
            state   <= SETUP_S;
          end
        end

        SETUP_S: begin
          // advance to access phase
          penable <= 1'b1;
          state   <= ACCESS_S;
        end

        ACCESS_S: begin
          if (pready) begin
            if (!rw_l) rdata <= prdata;
            // complete transfer
            psel    <= 1'b0;
            penable <= 1'b0;
            state   <= IDLE_S;
          end
        end
      endcase
    end
  end

endmodule

