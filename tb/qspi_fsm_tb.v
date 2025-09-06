`timescale 1ns/1ps

module qspi_fsm_tb;
    reg clk;
    reg resetn;
    reg start;
    wire done;
    integer i;
    reg [7:0] cap;

    reg [1:0] cmd_lanes_sel;
    reg [1:0] addr_lanes_sel;
    reg [1:0] data_lanes_sel;
    reg [1:0] addr_bytes_sel;
    reg mode_en;
    reg [3:0] dummy_cycles;
    reg dir;
    reg [7:0] cmd_opcode;
    reg [7:0] mode_bits;
    reg [31:0] addr;
    reg [31:0] len_bytes;
    reg [31:0] clk_div;
    reg cpol;
    reg cpha;
    reg [31:0] tx_data_fifo;
    wire tx_ren;
    reg tx_empty;   // was wire
    wire [31:0] rx_data_fifo;
    wire rx_wen;
    reg rx_full;
    wire sclk;
    wire cs_n;
    wire io0;
    wire io1;
    wire io2;
    wire io3;

    qspi_fsm dut (
        .clk(clk),
        .resetn(resetn),
        .start(start),
        .done(done),
        .cmd_lanes_sel(cmd_lanes_sel),
        .addr_lanes_sel(addr_lanes_sel),
        .data_lanes_sel(data_lanes_sel),
        .addr_bytes_sel(addr_bytes_sel),
        .mode_en(mode_en),
        .dummy_cycles(dummy_cycles),
        .dir(dir),
        .cmd_opcode(cmd_opcode),
        .mode_bits(mode_bits),
        .addr(addr),
        .len_bytes(len_bytes),
        .clk_div(clk_div),
        .cpol(cpol),
        .cpha(cpha),
        .tx_data_fifo(tx_data_fifo),
        .tx_ren(tx_ren),
        .tx_empty(tx_empty),
        .rx_data_fifo(rx_data_fifo),
        .rx_wen(rx_wen),
        .rx_full(rx_full),
        .sclk(sclk),
        .cs_n(cs_n),
        .io0(io0),
        .io1(io1),
        .io2(io2),
        .io3(io3)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        $dumpfile("qspi_fsm_tb.vcd");
        $dumpvars(0, qspi_fsm_tb);
        resetn = 0;
        start = 0;
        cmd_lanes_sel = 0;
        addr_lanes_sel = 0;
        data_lanes_sel = 0;
        addr_bytes_sel = 0;
        mode_en = 0;
        dummy_cycles = 0;
        dir = 0;
        cmd_opcode = 8'h06;
        mode_bits = 0;
        addr = 0;
        len_bytes = 0;
        clk_div = 0;
        cpol = 0;
        cpha = 0;
        tx_data_fifo = 32'h0;
        tx_empty = 1'b1;
        rx_full = 0;
        cap = 0;
        #20 resetn = 1;

        // Test MSB-first and CS timing with WREN
        @(posedge clk); start <= 1; @(posedge clk); start <= 0;
        wait(!cs_n);
        for (i=0;i<8;i=i+1) begin
            @(posedge sclk); cap = {cap[6:0], io0}; @(negedge sclk);
        end
        wait(done);
        
        if (cap !== 8'h06) $fatal(1, "Opcode shifted LSB-first");
        if (cs_n) $fatal(1, "CS# high before done");
        wait(cs_n);

        // Test dual-lane opcode shifting
        cmd_lanes_sel = 2'b01; // dual
        cmd_opcode    = 8'hAB;
        tx_empty      = 1'b1;
        cap = 0;
        @(posedge clk); start <= 1; @(posedge clk); start <= 0;
        wait(!cs_n);
        @(posedge sclk); cap[1:0] = {io1, io0}; @(negedge sclk);
        wait(done);
        if (cap[1:0] !== 2'b10) $fatal(1, "Dual-lane MSB mismatch");

        // Test dummy cycle skip when len=0
        cmd_lanes_sel = 0;
        cmd_opcode    = 8'h0B; // fast read
        dummy_cycles  = 4'd8;
        len_bytes     = 0;
        @(posedge clk); start <= 1; @(posedge clk); start <= 0;
        wait(done);

        $display("QSPI FSM test passed");
        $finish;
    end
endmodule
