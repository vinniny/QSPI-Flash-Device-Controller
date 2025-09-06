`timescale 1ns/1ps

module top_tb;
    // clock/reset
    reg clk;
    reg resetn;

    // APB signals
    reg psel;
    reg penable;
    reg pwrite;
    reg [11:0] paddr;
    reg [31:0] pwdata;
    reg [3:0]  pstrb;
    wire [31:0] prdata;
    wire pready;
    wire pslverr;

    // DMA AXI master
    wire [31:0] m_awaddr;
    wire m_awvalid;
    reg  m_awready;
    wire [31:0] m_wdata;
    wire m_wvalid;
    wire [3:0]  m_wstrb;
    reg  m_wready;
    reg  m_bvalid;
    wire m_bready;
    wire [31:0] m_araddr;
    wire m_arvalid;
    reg  m_arready;
    reg [31:0] m_rdata;
    reg  m_rvalid;
    wire m_rready;

    // XIP AXI slave (host side)
    reg  s_arvalid;
    reg [31:0] s_araddr;
    wire s_arready;
    wire [31:0] s_rdata;
    wire s_rvalid;
    reg  s_rready;
    wire [1:0] s_rresp;

    // QSPI wires
    wire sclk;
    wire cs_n;
    wire [3:0] io;

    wire irq;

    qspi_controller dut (
        .clk(clk),
        .resetn(resetn),
        .psel(psel),
        .penable(penable),
        .pwrite(pwrite),
        .paddr(paddr),
        .pwdata(pwdata),
        .pstrb(pstrb),
        .prdata(prdata),
        .pready(pready),
        .pslverr(pslverr),
        .m_axi_awaddr(m_awaddr),
        .m_axi_awvalid(m_awvalid),
        .m_axi_awready(m_awready),
        .m_axi_wdata(m_wdata),
        .m_axi_wvalid(m_wvalid),
        .m_axi_wstrb(m_wstrb),
        .m_axi_wready(m_wready),
        .m_axi_bvalid(m_bvalid),
        .m_axi_bresp(2'b00),
        .m_axi_bready(m_bready),
        .m_axi_araddr(m_araddr),
        .m_axi_arvalid(m_arvalid),
        .m_axi_arready(m_arready),
        .m_axi_rdata(m_rdata),
        .m_axi_rvalid(m_rvalid),
        .m_axi_rresp(2'b00),
        .m_axi_rready(m_rready),
        .s_axi_awaddr(),
        .s_axi_awvalid(1'b0),
        .s_axi_awready(),
        .s_axi_wdata(),
        .s_axi_wstrb(),
        .s_axi_wvalid(1'b0),
        .s_axi_wready(),
        .s_axi_bresp(),
        .s_axi_bvalid(),
        .s_axi_bready(1'b0),
        .s_axi_araddr(s_araddr),
        .s_axi_arvalid(s_arvalid),
        .s_axi_arready(s_arready),
        .s_axi_rdata(s_rdata),
        .s_axi_rresp(s_rresp),
        .s_axi_rvalid(s_rvalid),
        .s_axi_rready(s_rready),
        .sclk(sclk),
        .cs_n(cs_n),
        .io(io),
        .irq(irq)
    );

    // simple AXI memory model for DMA writes
    reg [31:0] mem [0:3];
    reg [31:0] awaddr_reg;
    always @(posedge clk) begin
        m_awready <= 0;
        m_wready  <= 0;
        m_bvalid  <= 0;
        m_arready <= 0;
        m_rvalid  <= 0;
        if (m_awvalid) begin
            m_awready <= 1;
            awaddr_reg <= m_awaddr;
        end
        if (m_wvalid) begin
            m_wready <= 1;
            mem[awaddr_reg[3:2]] <= m_wdata;
            m_bvalid <= 1;
        end
        if (m_rready && m_rvalid) m_rvalid <= 0;
    end
    assign m_rdata = 32'h0;

    // qspi device model
    qspi_device flash (
        .qspi_sclk(sclk),
        .qspi_cs_n(cs_n),
        .qspi_io0(io[0]),
        .qspi_io1(io[1]),
        .qspi_io2(io[2]),
        .qspi_io3(io[3])
    );

    // clock
    initial clk = 0;
    always #5 clk = ~clk;

    // APB tasks
    task apb_write(input [11:0] addr, input [31:0] data);
    begin
        @(posedge clk);
        psel <= 1; penable <= 0; pwrite <= 1; paddr <= addr; pwdata <= data; pstrb <= 4'hF;
        @(posedge clk); penable <= 1; @(posedge clk);
        psel <= 0; penable <= 0; pwrite <= 0; paddr <= 0; pwdata <= 0;
    end
    endtask

    task apb_read(input [11:0] addr, output [31:0] data);
    begin
        @(posedge clk);
        psel <= 1; penable <= 0; pwrite <= 0; paddr <= addr; pstrb <= 4'h0;
        @(posedge clk); penable <= 1; @(posedge clk);
        data = prdata;
        psel <= 0; penable <= 0; paddr <= 0;
    end
    endtask

    // AXI/DMA monitors
    always @(posedge clk) begin
        if (m_awvalid && m_awready)
            $display("[AXI] AW 0x%08h @%0t", m_awaddr, $time);
        if (m_wvalid && m_wready)
            $display("[AXI]  W 0x%08h strb=%b @%0t", m_wdata, m_wstrb, $time);
        if (m_bvalid && m_bready)
            $display("[AXI]  B resp OK @%0t", $time);
    end

    // Observe early SCLK activity
    integer sclk_edges;
    initial sclk_edges = 0;
    always @(posedge sclk) begin
        sclk_edges = sclk_edges + 1;
        if (sclk_edges <= 8)
            $display("[QSPI] SCLK posedge #%0d @%0t", sclk_edges, $time);
    end

    // Decls for CSR probe
    integer j;
    reg [31:0] fstat, stat;
    reg [3:0] rx_lvl_prev, rx_lvl_now;
    reg busy_prev, busy_now;

    // main sequence
    initial begin
        $display("[top_tb] Starting test at %0t", $time);
        resetn = 0;
        psel = 0; penable=0; pwrite=0; paddr=0; pwdata=0; pstrb=4'hF;
        s_arvalid = 0; s_araddr = 0; s_rready = 0;
        #40 resetn = 1;

        // ---------------- Command + DMA read ----------------
        // cmd_cfg: lanes_cmd=1-1-1, addr_bytes=3B, dummy=8
        // bits: [11:8]=8 dummy, [7:6]=01 (3B addr), [5:4]=00 (data lanes 1), [3:2]=00, [1:0]=00
        apb_write(12'h024, 32'h00000840);
        apb_write(12'h028, 32'h0000000B); // opcode
        apb_write(12'h02C, 32'h00000000); // addr
        apb_write(12'h030, 32'h00000004); // len =4
        apb_write(12'h038, 32'h00000034); // dma cfg: burst=4, dir=read_to_mem, incr=1
        apb_write(12'h03C, 32'h00000000); // dma dst addr
        apb_write(12'h040, 32'h00000004); // dma len
        // Ensure CTRL.enable=1 first
        apb_write(12'h004, 32'h00000001);
        // Enable + dma_en + trigger in the same write to avoid busy gating
        apb_write(12'h004, 32'h00000141);

        // Probe STATUS busy and RX FIFO level via CSR every few cycles
        // FIFO_STAT layout: {22'd0, rx_full, tx_empty, rx_level[3:0], tx_level[3:0]}
        rx_lvl_prev = 4'd0;
        busy_prev   = 1'b0;
        for (j = 0; j < 400; j = j + 1) begin
            apb_read(12'h008, stat); // STATUS
            busy_now = stat[11];
            if (j < 8) $display("[CSR] STATUS=0x%08h @%0t", stat, $time);
            if (busy_now != busy_prev) begin
                $display("[CSR] busy %0d -> %0d @%0t (STATUS=0x%08h)", busy_prev, busy_now, $time, stat);
                busy_prev = busy_now;
            end
            apb_read(12'h04C, fstat);
            rx_lvl_now = (fstat[7:4]);
            if (j < 8) $display("[CSR] FIFO_STAT=0x%08h @%0t", fstat, $time);
            if (rx_lvl_now != rx_lvl_prev) begin
                $display("[CSR] RX level %0d -> %0d @%0t (FIFO_STAT=0x%08h)", rx_lvl_prev, rx_lvl_now, $time, fstat);
                rx_lvl_prev = rx_lvl_now;
            end
        end

        repeat (50) @(posedge clk);
        if (mem[0] !== 32'hFFFF_FFFF) $fatal(1, "DMA read failed");

        // ---------------- XIP read ----------------
        apb_write(12'h004, 32'h00000003); // enable + xip_en
        s_araddr  <= 32'h00000000; s_arvalid <= 1;
        @(posedge clk); while(!s_arready) @(posedge clk); s_arvalid<=0;
        s_rready <= 1; @(posedge clk); while(!s_rvalid) @(posedge clk);
        if (s_rdata !== 32'hFFFF_FFFF) $fatal(1, "XIP read mismatch");
        s_rready <= 0;

        $display("Top-level integration test passed");
        $finish;
    end

    // Global timeout to prevent stalls
    initial begin
      #3_000_000; // 3 ms cutoff
      $display("[top_tb] Global timeout reached — finishing.");
      $finish;
    end
endmodule
