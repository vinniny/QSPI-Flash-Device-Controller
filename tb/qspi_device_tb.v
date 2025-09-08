`timescale 1ns/1ps

module qspi_device_tb;
    integer i;
    reg [7:0] rx;
    reg qspi_sclk;
    reg qspi_cs_n;
    reg master_oe;
    reg master_do;
    wire qspi_io0;
    wire qspi_io1;
    wire qspi_io2;
    wire qspi_io3;

    assign qspi_io0 = master_oe ? master_do : 1'bz;
    // other IO lines left floating

    qspi_device dut (
        .qspi_sclk(qspi_sclk),
        .qspi_cs_n(qspi_cs_n),
        .qspi_io0(qspi_io0),
        .qspi_io1(qspi_io1),
        .qspi_io2(qspi_io2),
        .qspi_io3(qspi_io3)
    );

    initial begin
        qspi_sclk = 0;
        qspi_cs_n = 1;
        master_oe = 0;
        master_do = 0;
        rx = 8'h00;

        // --- Read JEDEC ID ---
        #10 qspi_cs_n = 0; master_oe = 1;
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = (8'h9F >> i) & 1'b1;
            #5 qspi_sclk = 1; #5 qspi_sclk = 0;
        end
        master_oe = 0; rx = 0;
        for (i = 0; i < 8; i = i + 1) begin
            #5 qspi_sclk = 1; #1 rx = {rx[6:0], qspi_io1}; #4 qspi_sclk = 0;
        end
        qspi_cs_n = 1;
        if (rx !== 8'hC2) $fatal(1, "ID byte mismatch %h", rx);

        // --- Write Enable ---
        #10 qspi_cs_n = 0; master_oe = 1;
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = (8'h06 >> i) & 1'b1;
            #5 qspi_sclk = 1; #5 qspi_sclk = 0;
        end
        qspi_cs_n = 1; master_oe = 0;

        // --- Read Status to confirm WEL ---
        #10 qspi_cs_n = 0; master_oe = 1;
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = (8'h05 >> i) & 1'b1;
            #5 qspi_sclk = 1; #5 qspi_sclk = 0;
        end
        master_oe = 0; rx = 0;
        for (i = 0; i < 8; i = i + 1) begin
            #5 qspi_sclk = 1; #1 rx = {rx[6:0], qspi_io1}; #4 qspi_sclk = 0;
        end
        qspi_cs_n = 1;
        if (rx[1] !== 1'b1) $fatal(1, "WEL bit not set after WREN");

        // --- Page Program byte 0xAA at address 0 ---
        #10 qspi_cs_n = 0; master_oe = 1;
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = (8'h02 >> i) & 1'b1;
            #5 qspi_sclk = 1; #5 qspi_sclk = 0;
        end
        // address bytes (0x000000)
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = 1'b0; #5 qspi_sclk = 1; #5 qspi_sclk = 0; end
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = 1'b0; #5 qspi_sclk = 1; #5 qspi_sclk = 0; end
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = 1'b0; #5 qspi_sclk = 1; #5 qspi_sclk = 0; end
        // data byte 0xAA
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = (8'hAA >> i) & 1'b1;
            #5 qspi_sclk = 1; #5 qspi_sclk = 0;
        end
        qspi_cs_n = 1; master_oe = 0;

        // provide clocks for WIP completion
        for (i = 0; i < 120; i = i + 1) begin
            #5 qspi_sclk = 1; #5 qspi_sclk = 0;
        end

        // --- Read back programmed byte ---
        #10 qspi_cs_n = 0; master_oe = 1;
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = (8'h03 >> i) & 1'b1;
            #5 qspi_sclk = 1; #5 qspi_sclk = 0;
        end
        // address bytes 0x000000
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = 1'b0; #5 qspi_sclk = 1; #5 qspi_sclk = 0; end
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = 1'b0; #5 qspi_sclk = 1; #5 qspi_sclk = 0; end
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = 1'b0; #5 qspi_sclk = 1; #5 qspi_sclk = 0; end
        master_oe = 0; rx = 0;
        for (i = 0; i < 8; i = i + 1) begin
            #5 qspi_sclk = 1; #1 rx = {rx[6:0], qspi_io1}; #4 qspi_sclk = 0;
        end
        qspi_cs_n = 1;
        if (rx !== 8'hAA) $fatal(1, "Readback mismatch %h", rx);

        // --- Sector Erase at address 0 ---
        #10 qspi_cs_n = 0; master_oe = 1;
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = (8'h06 >> i) & 1'b1; // WREN
            #5 qspi_sclk = 1; #5 qspi_sclk = 0;
        end
        qspi_cs_n = 1; master_oe = 0;

        #10 qspi_cs_n = 0; master_oe = 1;
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = (8'h20 >> i) & 1'b1;
            #5 qspi_sclk = 1; #5 qspi_sclk = 0;
        end
        // address bytes 0x000000
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = 1'b0; #5 qspi_sclk = 1; #5 qspi_sclk = 0; end
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = 1'b0; #5 qspi_sclk = 1; #5 qspi_sclk = 0; end
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = 1'b0; #5 qspi_sclk = 1; #5 qspi_sclk = 0; end
        qspi_cs_n = 1; master_oe = 0;

        for (i = 0; i < 120; i = i + 1) begin
            #5 qspi_sclk = 1; #5 qspi_sclk = 0;
        end

        // --- Read back after erase (expect FF) ---
        #10 qspi_cs_n = 0; master_oe = 1;
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = (8'h03 >> i) & 1'b1;
            #5 qspi_sclk = 1; #5 qspi_sclk = 0;
        end
        // address bytes 0x000000
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = 1'b0; #5 qspi_sclk = 1; #5 qspi_sclk = 0; end
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = 1'b0; #5 qspi_sclk = 1; #5 qspi_sclk = 0; end
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = 1'b0; #5 qspi_sclk = 1; #5 qspi_sclk = 0; end
        master_oe = 0; rx = 0;
        for (i = 0; i < 8; i = i + 1) begin
            #5 qspi_sclk = 1; #1 rx = {rx[6:0], qspi_io1}; #4 qspi_sclk = 0;
        end
        qspi_cs_n = 1;
        if (rx !== 8'hFF) $fatal(1, "Erase failed %h", rx);

        $display("QSPI device test passed");
        $finish;
    end

    // Global timeout to prevent stalls
    initial begin
        #1_000_000; // 1 ms cutoff
        $display("[qspi_device_tb] Global timeout reached — finishing.");
        $finish;
    end
endmodule

