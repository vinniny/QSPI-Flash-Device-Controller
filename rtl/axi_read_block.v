module axi_read_block (
    input wire          clk,
    input wire          reset,

    input wire          start,
    input wire  [31:0]  addr,
    input wire  [15:0]  transfer_size,

    output reg  [31:0]  araddr,
    output reg          arvalid,
    input  wire         arready,

    input  wire         rvalid,
    input  wire [31:0]  rdata,
    output reg          rready,

    output reg  [31:0]  data_out,
    output reg          wr_en,
    input  wire         full,

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

    always @(posedge clk) begin
        if (reset) begin
            state     <= IDLE;
            count     <= 0;
            addr_reg  <= 0;
            araddr    <= 0;
            arvalid   <= 0;
            rready    <= 0;
            data_out  <= 0;
            wr_en     <= 0;
            busy      <= 0;
            done      <= 0;
        end else begin
            arvalid <= 0;
            rready  <= 0;
            wr_en   <= 0;
            done    <= 0;
            busy    <= (state != IDLE);

            case (state)
                IDLE: begin
                    if (start && !full) begin
                        addr_reg <= {addr[31:2], 2'b00};
                        count    <= 0;
                        araddr   <= {addr[31:2], 2'b00};
                        arvalid  <= 1;
                        state    <= ADDR;
                    end
                end

                ADDR: begin
                    if (arready) begin
                        rready  <= 1;
                        state   <= DATA;
                    end
                end

                DATA: begin
                    if (rvalid && !full) begin
                        data_out <= rdata;
                        wr_en    <= 1;
                        rready   <= 1;
                        count    <= count + 4;
                        if (count + 4 < transfer_size) begin
                            addr_reg <= addr_reg + 4;
                            araddr   <= addr_reg + 4;
                            arvalid  <= 1;
                            state    <= ADDR;
                        end else begin
                            state <= RESP;
                        end
                    end
                end

                RESP: begin
                    done  <= 1;
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule
