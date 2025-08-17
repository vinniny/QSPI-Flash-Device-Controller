`timescale 1ns/1ps

module csr_tb;
    reg pclk;
    reg [31:0] rdata;
    reg presetn;
    reg psel;
    reg penable;
    reg pwrite;
    reg [11:0] paddr;
    reg [31:0] pwdata;
    reg [3:0] pstrb;
    wire [31:0] prdata;
    wire pready;
    wire pslverr;
    wire enable_o;

    csr dut (
        .pclk(pclk),
        .presetn(presetn),
        .psel(psel),
        .penable(penable),
        .pwrite(pwrite),
        .paddr(paddr),
        .pwdata(pwdata),
        .pstrb(pstrb),
        .prdata(prdata),
        .pready(pready),
        .pslverr(pslverr),
        .enable_o(enable_o),
        .xip_en_o(),
        .quad_en_o(),
        .cpol_o(),
        .cpha_o(),
        .lsb_first_o(),
        .cmd_trigger_o(),
        .dma_en_o(),
        .mode_en_o(),
        .hold_en_o(),
        .wp_en_o(),
        .cmd_trigger_clr_i(1'b0),
        .clk_div_o(),
        .cs_auto_o(),
        .cs_level_o(),
        .cs_delay_o(),
        .xip_addr_bytes_o(),
        .xip_data_lanes_o(),
        .xip_dummy_cycles_o(),
        .xip_cont_read_o(),
        .xip_mode_en_o(),
        .xip_write_en_o(),
        .xip_read_op_o(),
        .xip_mode_bits_o(),
        .xip_write_op_o(),
        .cmd_lanes_o(),
        .addr_lanes_o(),
        .data_lanes_o(),
        .addr_bytes_o(),
        .mode_en_cfg_o(),
        .dummy_cycles_o(),
        .is_write_o(),
        .opcode_o(),
        .mode_bits_o(),
        .cmd_addr_o(),
        .cmd_len_o(),
        .extra_dummy_o(),
        .burst_size_o(),
        .dma_dir_o(),
        .incr_addr_o(),
        .dma_addr_o(),
        .dma_len_o(),
        .fifo_tx_data_o(),
        .fifo_tx_we_o(),
        .fifo_rx_data_i(32'h0),
        .fifo_rx_re_o(),
        .int_en_o(),
        .cmd_done_set_i(1'b0),
        .dma_done_set_i(1'b0),
        .err_set_i(1'b0),
        .fifo_tx_empty_set_i(1'b0),
        .fifo_rx_full_set_i(1'b0),
        .busy_i(1'b0),
        .xip_active_i(1'b0),
        .cmd_done_i(1'b0),
        .dma_done_i(1'b0),
        .tx_level_i(4'h0),
        .rx_level_i(4'h0),
        .tx_empty_i(1'b1),
        .rx_full_i(1'b0),
        .timeout_i(1'b0),
        .overrun_i(1'b0),
        .underrun_i(1'b0),
        .axi_err_i(1'b0),
        .irq()
    );

    initial pclk = 0;
    always #5 pclk = ~pclk;

    task apb_write(input [11:0] addr, input [31:0] data);
    begin
        @(posedge pclk);
        psel <= 1;
        penable <= 0;
        pwrite <= 1;
        paddr <= addr;
        pwdata <= data;
        pstrb <= 4'hF;
        @(posedge pclk);
        penable <= 1;
        @(posedge pclk);
        psel <= 0;
        penable <= 0;
        pwrite <= 0;
        paddr <= 0;
        pwdata <= 0;
    end
    endtask

    task apb_read(input [11:0] addr, output [31:0] data);
    begin
        @(posedge pclk);
        psel <= 1;
        penable <= 0;
        pwrite <= 0;
        paddr <= addr;
        @(posedge pclk);
        penable <= 1;
        @(posedge pclk);
        data = prdata;
        psel <= 0;
        penable <= 0;
        paddr <= 0;
    end
    endtask

    initial begin
        psel = 0;
        penable = 0;
        pwrite = 0;
        paddr = 0;
        pwdata = 0;
        pstrb = 4'hF;
        presetn = 0;
        #20 presetn = 1;
        apb_read(12'h000, rdata);
        if (rdata !== 32'h1A001081) begin
            $fatal(1, "ID mismatch %h", rdata);
        end
        apb_write(12'h004, 32'h1);
        @(posedge pclk);
        if (!enable_o) begin
            $fatal(1, "Enable bit not set");
        end
        $display("CSR test passed");
        $finish;
    end
endmodule
