`timescale 1ns/1ps

module qspi_device_tb;
    integer i;
    reg [7:0] id_byte;
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
        #10 qspi_cs_n = 0;
        master_oe = 1;
        // send 0x9F command MSB-first
        for (i = 7; i >= 0; i = i - 1) begin
            master_do = (8'h9F >> i) & 1'b1;
            #5 qspi_sclk = 1;
            #5 qspi_sclk = 0;
        end
        master_oe = 0; // release for read
        // read manufacturer ID
        id_byte = 0;
        for (i = 0; i < 8; i = i + 1) begin
            #5 qspi_sclk = 1;
            id_byte = {id_byte[6:0], qspi_io1};
            #5 qspi_sclk = 0;
        end
        qspi_cs_n = 1;
        if (id_byte !== 8'hC2) $fatal(1, "ID byte mismatch %h", id_byte);
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
