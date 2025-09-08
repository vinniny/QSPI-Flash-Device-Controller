// AXI4 Slave Memory Model with deterministic read data
module axi4_ram_slave (
    input wire 		clk,
    input wire 		rst_n,

    // Write address channel
    input wire 		awvalid,
    input wire [31:0] 	awaddr,
    output reg 		awready,

    // Write data channel
    input wire 		wvalid,
    input wire [31:0] 	wdata,
    input wire [3:0] 	wstrb,
    output reg 		wready,

    // Write response channel
    output reg 		bvalid,
    input wire 		bready,

    // Read address channel
    input wire 		arvalid,
    input wire [31:0] 	araddr,
    output reg 		arready,

    // Read data channel
    output reg 		rvalid,
    output reg [31:0] 	rdata,
    input wire 		rready
);

    // Simple RAM (64KB x 32-bit words = 16K entries)
    reg [31:0] mem [0:16383];
    wire [13:0] awidx = awaddr[15:2];
    wire [13:0] aridx = araddr[15:2];

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 16384; i = i + 1) // initialize with deterministic values
                mem[i] <= 32'hA5A50000 + i;
            awready <= 0;
            wready <= 0;
            bvalid <= 0;
            arready <= 0;
            rvalid <= 0;
            rdata <= 0;
        end
        else begin
            // Write address handshake
            if (awready && awvalid)
                awready <= 1;
            else
                awready <= 0;

            // Write data handshake
            if (wready && wvalid) begin
                wready <= 1;
                if (wstrb[0]) mem[awidx][7:0] <= wdata[7:0];
                if (wstrb[1]) mem[awidx][15:8] <= wdata[15:8];
                if (wstrb[2]) mem[awidx][23:16] <= wdata[23:16];
                if (wstrb[3]) mem[awidx][31:24] <= wdata[31:24];
            end
            else begin
                wready <= 0;
            end

            // Write response
            if (bvalid && bready)
                bvalid <= 0;

            // Read address handshake
            if (arready && arvalid)
                arready <= 1;
            else
                arready <= 0;

            // Read data channel
            if (arvalid && arready) begin
                rvalid <= 1;
                rdata <= mem[aridx];
            end
            else if (rvalid && rready) begin
                rvalid <= 0;
            end
        end
    end

endmodule