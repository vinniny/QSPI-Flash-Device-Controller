module axi_write_block (
    input wire          clk,
    input wire          rst_n,

    input wire          start,
    input wire  [31:0]  addr,
    input wire  [15:0]  transfer_size,

    // AXI4-lite write address channel
    output reg  [31:0]  awaddr,
    output reg          awvalid,
    input  wire         awready,

    // AXI4-lite write data channel
    output reg  [31:0]  wdata,
    output reg          wvalid,
    output reg  [3:0]   wstrb,
    input  wire         wready,

    // AXI4-lite write response channel
    input  wire         bvalid,
    output reg          bready,

    // FIFO input
    input  wire  [31:0] data_in,
    input  wire         empty,
    output reg          rd_en,

    output reg          busy,
    output reg          done
);

    localparam IDLE  = 2'd0,
               ADDR  = 2'd1,
               DATA  = 2'd2,
               RESP  = 2'd3;

    reg [1:0] state;
    reg [15:0] count;
    reg [31:0] addr_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            count     <= 0;
            addr_reg  <= 0;
            awaddr    <= 0;
            awvalid   <= 0;
            wdata     <= 0;
            wvalid    <= 0;
            wstrb     <= 4'b1111;
            bready    <= 0;
            rd_en     <= 0;
            busy      <= 0;
            done      <= 0;
        end else begin
            awvalid <= 0;
            wvalid  <= 0;
            bready  <= 0;
            rd_en   <= 0;
            done    <= 0;
            busy    <= (state != IDLE);

            case (state)
                IDLE: begin
                    if (start && !empty) begin
                        addr_reg <= {addr[31:2], 2'b00};
                        count    <= 0;
                        awaddr   <= {addr[31:2], 2'b00};
                        awvalid  <= 1;
                        state    <= ADDR;
                    end
                end

                ADDR: begin
                    if (awready) begin
                        if (!empty) begin
                            rd_en   <= 1;
                            wdata   <= data_in;
                            wvalid  <= 1;
                            wstrb   <= 4'b1111;
                            state   <= DATA;
                        end
                    end
                end

                DATA: begin
                    if (wready) begin
                        bready   <= 1;
                        count    <= count + 4;
                        if (count + 4 < transfer_size) begin
                            addr_reg <= addr_reg + 4;
                            awaddr   <= addr_reg + 4;
                            awvalid  <= 1;
                            state    <= ADDR;
                        end else begin
                            state <= RESP;
                        end
                    end
                end

                RESP: begin
                    if (bvalid) begin
                        done  <= 1;
                        state <= IDLE;
                    end
                end
            endcase
        end
    end

endmodule
