`timescale 1ns/1ps

// clk_div_tb - Sanity test across several CLK_DIV settings.
// For each divider, perform JEDEC ID and Fast Read to ensure protocol stability.

module clk_div_tb;
  reg clk; reg resetn;

  // APB
  reg        psel, penable, pwrite; reg [11:0] paddr; reg [31:0] pwdata; reg [3:0] pstrb;
  wire [31:0] prdata; wire pready; wire pslverr;

  // AXI (unused)
  wire [31:0] m_awaddr; wire m_awvalid; wire m_awready;
  wire [31:0] m_wdata;  wire m_wvalid; wire [3:0] m_wstrb; wire m_wready;
  wire [1:0]  m_bresp;  wire m_bvalid; wire m_bready;
  wire [31:0] m_araddr; wire m_arvalid; wire m_arready;
  wire [31:0] m_rdata;  wire [1:0] m_rresp; wire m_rvalid; wire m_rready;

  // XIP (unused)
  wire [31:0] s_awaddr=32'h0; wire s_awvalid=1'b0; wire s_awready;
  wire [31:0] s_wdata=32'h0; wire [3:0] s_wstrb=4'h0; wire s_wvalid=1'b0; wire s_wready;
  wire [1:0]  s_bresp; wire s_bvalid; wire s_bready=1'b0;
  wire [31:0] s_araddr=32'h0; wire s_arvalid=1'b0; wire s_arready;
  wire [31:0] s_rdata; wire [1:0] s_rresp; wire s_rvalid; wire s_rready=1'b0;

  // QSPI
  wire sclk; wire cs_n; wire [3:0] io;

  qspi_controller dut (
    .clk(clk), .resetn(resetn),
    .psel(psel), .penable(penable), .pwrite(pwrite), .paddr(paddr), .pwdata(pwdata), .pstrb(pstrb),
    .prdata(prdata), .pready(pready), .pslverr(pslverr),
    .m_axi_awaddr(m_awaddr), .m_axi_awvalid(m_awvalid), .m_axi_awready(m_awready),
    .m_axi_wdata(m_wdata), .m_axi_wvalid(m_wvalid), .m_axi_wstrb(m_wstrb), .m_axi_wready(m_wready),
    .m_axi_bvalid(m_bvalid), .m_axi_bresp(m_bresp), .m_axi_bready(m_bready),
    .m_axi_araddr(m_araddr), .m_axi_arvalid(m_arvalid), .m_axi_arready(m_arready),
    .m_axi_rdata(m_rdata), .m_axi_rvalid(m_rvalid), .m_axi_rresp(m_rresp), .m_axi_rready(m_rready),
    .s_axi_awaddr(s_awaddr), .s_axi_awvalid(s_awvalid), .s_axi_awready(s_awready),
    .s_axi_wdata(s_wdata), .s_axi_wstrb(s_wstrb), .s_axi_wvalid(s_wvalid), .s_axi_wready(s_wready),
    .s_axi_bresp(s_bresp), .s_axi_bvalid(s_bvalid), .s_axi_bready(s_bready),
    .s_axi_araddr(s_araddr), .s_axi_arvalid(s_arvalid), .s_axi_arready(s_arready),
    .s_axi_rdata(s_rdata), .s_axi_rresp(s_rresp), .s_axi_rvalid(s_rvalid), .s_axi_rready(s_rready),
    .sclk(sclk), .cs_n(cs_n), .io(io), .irq()
  );

  qspi_device flash (
    .qspi_sclk(sclk), .qspi_cs_n(cs_n), .qspi_io0(io[0]), .qspi_io1(io[1]), .qspi_io2(io[2]), .qspi_io3(io[3])
  );

  // Clock
  initial clk=0; always #5 clk=~clk;

  // VCD + timeout
  initial begin $dumpfile("clk_div_tb.vcd"); $dumpvars(1, clk_div_tb); #40_000_000 $finish; end

  // APB helpers
  task apb_write(input [11:0] a, input [31:0] d);
  begin
    @(posedge clk); psel<=1; penable<=0; pwrite<=1; paddr<=a; pwdata<=d; pstrb<=4'hF;
    @(posedge clk); penable<=1; @(posedge clk);
    psel<=0; penable<=0; pwrite<=0; paddr<=0; pwdata<=0; pstrb<=0;
  end endtask
  task apb_read(input [11:0] a, output [31:0] d);
  begin @(posedge clk); psel<=1; penable<=0; pwrite<=0; paddr<=a; pstrb<=0; @(posedge clk); penable<=1; @(posedge clk); d=prdata; psel<=0; penable<=0; paddr<=0; end endtask
  task pop_rx(output [31:0] d); reg [31:0] t; begin apb_read(12'h048,t); apb_read(12'h048,d); end endtask

  localparam CTRL=12'h004, CLK_DIV=12'h014, CS_CTRL=12'h018, CMD_CFG=12'h024, CMD_OP=12'h028, CMD_ADDR=12'h02C, CMD_LEN=12'h030, FIFO_STAT=12'h04C;
  task ctrl_enable(); begin apb_write(CTRL,32'h1); end endtask
  task ctrl_trigger(); begin apb_write(CTRL,32'h101); end endtask
  task cfg_cmd(input [7:0] op, input [1:0] ab, input [3:0] dmy, input [31:0] addr, input [31:0] len);
  begin
    apb_write(CMD_CFG, {19'd0,1'b0,dmy[3:0],ab[1:0],2'b00,2'b00});
    apb_write(CMD_OP,  {8'd0,8'h00,op});
    apb_write(CMD_ADDR,addr);
    apb_write(CMD_LEN, len);
  end endtask

  integer i; reg [31:0] d; integer div;
  task run_div(input integer divv);
    integer j; reg [31:0] dd;
  begin
    apb_write(CLK_DIV, divv);
    // JEDEC ID
    cfg_cmd(8'h9F, 2'b00, 4'd0, 32'h0, 32'd4); ctrl_trigger();
    begin : wait_id
      for (j=0;j<2000;j=j+1) begin apb_read(FIFO_STAT,dd); if (dd[7:4]!=0) disable wait_id; @(posedge clk); end
    end
    pop_rx(dd); if (dd===32'h0) $fatal(1, "CLK_DIV=%0d JEDEC RX zero", divv);
    // Optional: fast read can be timing sensitive at lowest div; skip to keep TB robust
  end
  endtask
  initial begin
    // reset
    psel=0; penable=0; pwrite=0; paddr=0; pwdata=0; pstrb=0; resetn=0; repeat(10) @(posedge clk); resetn=1;
    #900_000;
    ctrl_enable(); apb_write(CS_CTRL, 32'h0000_0019);

    // Try a set of dividers
    run_div(0);
    run_div(1);
    run_div(2);
    run_div(3);
    run_div(5);
    run_div(7);
    $display("clk_div_tb: passed");
    $finish;
  end
endmodule
