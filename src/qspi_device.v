// qspi_device.v - Simplified behavioral QSPI flash model
// Supports a minimal set of commands to exercise the controller/testbenches:
// - 0x9F: Read JEDEC ID (returns 0xC2 on IO1)
// - 0x05: Read Status (bit0=WIP, bit1=WEL)
// - 0x06/0x04: WREN/WRDI (set/clear WEL in status[1])
// - 0x03/0x0B: Read (1-1-1) and Fast Read with 8 dummy cycles
// - 0x02: Page Program (1-1-1) — captures input bytes to memory
// - 0x20: Sector Erase (4KB) — sets sector bytes to 0xFF

module qspi_device (
    input  wire qspi_sclk,
    input  wire qspi_cs_n,
    inout  wire qspi_io0,
    inout  wire qspi_io1,
    inout  wire qspi_io2,
    inout  wire qspi_io3
);

    parameter integer MEM_SIZE    = 1024*1024;
    parameter integer ADDR_BITS   = 24;
    parameter integer PAGE_SIZE   = 256;
    parameter integer SECTOR_SIZE = 4096;
    parameter integer ERASE_TIME  = 100; // cycles to complete erase

    // Memory initialized to 0xFF
    reg [7:0] memory [0:MEM_SIZE-1];

    // IO control
    reg  [3:0] io_oe;
    reg  [3:0] io_do;
    wire [3:0] io_di = {qspi_io3, qspi_io2, qspi_io1, qspi_io0};

    assign qspi_io0 = io_oe[0] ? io_do[0] : 1'bz;
    assign qspi_io1 = io_oe[1] ? io_do[1] : 1'bz;
    assign qspi_io2 = io_oe[2] ? io_do[2] : 1'bz;
    assign qspi_io3 = io_oe[3] ? io_do[3] : 1'bz;

    // Registers
    reg [7:0]  status_reg = 8'h00;           // bit0=WIP, bit1=WEL
    reg [23:0] id_reg = 24'hC22017;          // 0xC2 0x20 0x17 (Macronix MX25L6436F)
    reg [7:0] cmd_reg = 8'h00;
    reg [7:0] nxt_cmd_reg = 8'h00;
    reg [ADDR_BITS-1:0] addr_reg = {ADDR_BITS{1'b0}};
    reg [7:0] mode_bits = 8'h00;        
    reg [7:0] shift_in = 8'h00;
    reg [7:0] shift_out = 8'h00;
    reg [3:0] lanes = 4'd0;
    reg [4:0] dummy_cycles = 5'd0;
    reg continuous_read = 1'b0;

    // Protocol state
    localparam [3:0]
        ST_IDLE       = 4'd0,
        ST_CMD        = 4'd1,
        ST_ADDR       = 4'd2,
        ST_MODE       = 4'd3,
        ST_DUMMY      = 4'd4,
        ST_DATA_READ  = 4'd5,
        ST_DATA_WRITE = 4'd6,
        ST_ERASE      = 4'd7,
        ST_STATUS     = 4'd8,
        ST_ID_READ    = 4'd9;

    reg [3:0]   state = ST_IDLE;
    
    reg [31:0]  bit_cnt = 32'd0;       // bits shifted in current phase
    reg [31:0]  nxt_bit_cnt = 32'd0;   // bits to shift in current phase
    reg [31:0]  nxt_addr_reg = 32'd0;  // next address for continuous read
    reg [31:0]  byte_cnt = 32'd0;      // bytes shifted in multi-byte phases
    reg wip = 1'b0;              // write-in-progress flag
    reg [31:0] erase_counter = 32'd0; // count erase operations

    initial begin
        integer i;
        for (i = 0; i < MEM_SIZE; i = i + 1)
            memory[i] = 8'hFF;
        wip = 1'b0;
    end

    always @(posedge qspi_sclk) begin
        if (wip) begin
            // Simulate write/erase completion after some cycles
            erase_counter <= erase_counter + 1;
            if (erase_counter >= ERASE_TIME) begin
                wip <= 1'b0;
                erase_counter <= 32'd0;
                // Clear WEL after a program/erase completes
                status_reg[1] <= 1'b0;
            end
        end
    end

    always @(posedge qspi_sclk or posedge qspi_cs_n) begin
        if (qspi_cs_n) begin
            // Chip select inactive: reset state
            state <= ST_IDLE;
            io_oe <= 4'b0000;
            continuous_read <= 1'b0;
            shift_in <= 8'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                   state <= ST_CMD;
                   bit_cnt <= 6'd0;
                   lanes <= 4'd1; // default to 1-1-1
                   shift_in <= 8'd0;
                end
                ST_CMD: begin
                    shift_in <= {shift_in[6:0], io_di[0]};
                    bit_cnt <= bit_cnt + 1;
                    if (bit_cnt == 7) begin
                        cmd_reg <= {shift_in[6:0], io_di[0]};
                        bit_cnt <= 6'd0;
                        byte_cnt <= 6'd0;
                        case (nxt_cmd_reg)
                            8'h9F: begin // Read JEDEC ID
                                state <= ST_ID_READ;
                                lanes <= 4'd1;
                                dummy_cycles <= 5'd0;
                                shift_out <= id_reg[23:16]; // first byte
                                io_oe <= 4'b0010; // drive IO1 (SO)
                            end
                            8'h05: begin // Read Status
                                state <= ST_STATUS;
                                lanes <= 4'd1;
                                shift_out <= status_reg;
                                io_oe <= 4'b0010; // drive IO1 (SO)
                            end
                            8'h06: begin // Write Enable
                                status_reg[1] <= 1'b1; // set WEL
                                state <= ST_IDLE;
                            end
                            8'h04: begin // Write Disable
                                status_reg[1] <= 1'b0; // clear WEL
                                state <= ST_IDLE;
                            end
                            8'h03, 8'h0B, 8'hBB, 8'h6B, 8'hEB: begin // 3B -> BB
                                state <= ST_ADDR;
                                addr_reg <= {ADDR_BITS{1'b0}};
                                shift_in <= 8'd0;
                                lanes <= (nxt_cmd_reg == 8'h03 || nxt_cmd_reg == 8'h0B || nxt_cmd_reg == 8'h6B) ? 4'd1 : (nxt_cmd_reg == 8'hBB) ? 4'd2 : 4'd4; 
                                dummy_cycles <= (nxt_cmd_reg == 8'h03) ? 5'd0 : (nxt_cmd_reg == 8'hEB) ? 5'd4 : (nxt_cmd_reg == 8'hBB) ? 5'd4 : 5'd8;
                            end
                            8'h02, 8'h38: begin // 32 => 38
                                dummy_cycles <= 5'd0;
                                if (status_reg[1]) begin
                                    state <= ST_ADDR;
                                    lanes <= (nxt_cmd_reg == 8'h02) ? 4'd1 : 4'd4; // 1-1-1 or 4-4-4
                                end else begin
                                    state <= ST_IDLE; // ignore if WEL not set
                                end
                            end
                            8'h20, 8'hD8: begin
                                if (status_reg[1]) begin
                                    state <= ST_ADDR;
                                end else begin
                                    state <= ST_IDLE; // ignore if WEL not set
                                end
                            end
                            8'hC7: begin // Chip Erase
                                if (status_reg[1]) begin
                                    // Start chip erase
                                    integer j;
                                    for (j = 0; j < MEM_SIZE; j = j + 1)
                                        memory[j] = 8'hFF;
                                    wip <= 1'b1;
                                end else begin
                                    state <= ST_IDLE; // Ignore if WEL not set
                                end
                            end
                            default: begin
                                state <= ST_IDLE; // Unsupported command
                            end
                        endcase
                    end
                end
                ST_ADDR: begin
                    if (lanes == 4'd1) begin
                        shift_in <= {shift_in[6:0], io_di[0]};
                    end else if (lanes == 4'd2) begin
                        shift_in <= {shift_in[5:0], io_di[1:0]};
                    end else if (lanes == 4'd4) begin
                        shift_in <= {shift_in[3:0], io_di[3:0]};
                    end
                    bit_cnt <= bit_cnt + lanes;
                    if ((bit_cnt == 7 && lanes == 4'd1) ||
                        (bit_cnt == 6 && lanes == 4'd2) ||
                        (bit_cnt == 4 && lanes == 4'd4)) begin
                        addr_reg <= {addr_reg[ADDR_BITS-9:0], lanes == 4'd1 ? {shift_in[6:0], io_di[0]} :
                                     lanes == 4'd2 ? {shift_in[5:0], io_di[1:0]} :
                                                     {shift_in[3:0], io_di[3:0]}};
                        bit_cnt <= 6'd0;
                        shift_in <= 8'd0;
                        byte_cnt <= byte_cnt + 1;
                        if (byte_cnt == (ADDR_BITS/8)-1) begin
                            if (cmd_reg == 8'hEB) state <= ST_MODE;
                            else if (cmd_reg == 8'h20 || cmd_reg == 8'hD8) begin
                                integer j;
                                if (cmd_reg == 8'h20) begin
                                    for (j = nxt_addr_reg; j < nxt_addr_reg + SECTOR_SIZE; j = j + 1)
                                        memory[j] <= 8'hFF;
                                end else if (cmd_reg == 8'hD8) begin
                                    for (j = nxt_addr_reg; j < nxt_addr_reg + 65536; j = j + 1)
                                        memory[j] <= 8'hFF;
                                end
                                wip <= 1'b1;
                                state <= ST_ERASE;
                            end
                            else if (dummy_cycles > 0) state <= ST_DUMMY;
                            else begin
                                state <= (cmd_reg == 8'h02 || cmd_reg == 8'h38) ? ST_DATA_WRITE : ST_DATA_READ;
                                bit_cnt <= 6'd0;
                                byte_cnt <= 6'd0;
                                if (cmd_reg == 8'h02 || cmd_reg == 8'h38) io_oe <= 4'b0000;
                                else if (cmd_reg == 8'h6B) io_oe <= 4'b1111;
                                else if (lanes == 4'd1) io_oe <= 4'b0010;
                                else if (lanes == 4'd2) io_oe <= 4'b0011;
                                else io_oe <= 4'b1111;
                                shift_out <= memory[addr_reg];
                            end
                        end
                    end
                end
                ST_MODE: begin
                    if (lanes == 4'd1) begin
                        shift_in <= {shift_in[6:0], io_di[0]};
                    end else if (lanes == 4'd2) begin
                        shift_in <= {shift_in[5:0], io_di[1:0]};
                    end else if (lanes == 4'd4) begin
                        shift_in <= {shift_in[3:0], io_di[3:0]};
                    end
                    bit_cnt <= bit_cnt + lanes;
                    if (bit_cnt + lanes >= 8) begin
                        mode_bits <= shift_in[7:0];
                        continuous_read <= (shift_in == 8'hA0);
                        state <= ST_DUMMY;
                        bit_cnt <= 6'd0;
                    end
                end
                ST_DUMMY: begin
                    bit_cnt <= bit_cnt + 1;
                    if (bit_cnt == dummy_cycles - 1) begin
                        state <= (cmd_reg == 8'h02 || cmd_reg == 8'h38) ? ST_DATA_WRITE : ST_DATA_READ;
                        if (cmd_reg == 8'h6B) lanes <= 4;
                        bit_cnt <= 0;
                        if (cmd_reg == 8'h02 || cmd_reg == 8'h38) io_oe <= 4'b0000;
                        else if (cmd_reg == 8'h6B) io_oe <= 4'b1111;
                        else if (lanes == 4'd1) io_oe <= 4'b0010;
                        else if (lanes == 4'd2) io_oe <= 4'b0011;
                        else io_oe <= 4'b1111;
                        shift_out <= memory[addr_reg];
                    end
                end
                ST_DATA_READ: begin
                    // Drive outputs early in the cycle so external master
                    // sampling on the same clock edge sees a stable value.
                    if (lanes == 1) io_do[1] = shift_out[7];
                    else if (lanes == 2) begin
                        io_do[0] = shift_out[6];
                        io_do[1] = shift_out[7];
                    end else if (lanes == 4) begin 
                        io_do[0] = shift_out[4];
                        io_do[1] = shift_out[5];
                        io_do[2] = shift_out[6];
                        io_do[3] = shift_out[7];
                    end
                    shift_out <= shift_out << lanes;
                    bit_cnt <= bit_cnt + lanes;
                    if ((bit_cnt == 7 && lanes == 1) || (bit_cnt == 6 && lanes == 2) || (bit_cnt == 4 && lanes == 4)) begin
                        addr_reg <= addr_reg + 1;
                        shift_out <= memory[addr_reg + 1];
                        bit_cnt <= 0;
                        if (!continuous_read && byte_cnt == MEM_SIZE - addr_reg) state <= ST_IDLE;
                        byte_cnt <= byte_cnt + 1;
                    end
                end
                ST_DATA_WRITE: begin
                    if (lanes == 1) shift_in <= {shift_in[6:0], io_di[0]};
                    else if (lanes == 2) shift_in <= {shift_in[5:0], io_di[1:0]};
                    else if (lanes == 4) shift_in <= {shift_in[3:0], io_di[3:0]};
                    bit_cnt <= bit_cnt + lanes;
                    if ((bit_cnt == 7 && lanes == 1) || (bit_cnt == 6 && lanes == 2) || (bit_cnt == 4 && lanes == 4)) begin
                        if (addr_reg < MEM_SIZE && (addr_reg % PAGE_SIZE != PAGE_SIZE - 1)) begin
                            memory[addr_reg] <= lanes == 1 ? {shift_in[6:0], io_di[0]} : lanes == 2 ? {shift_in[5:0], io_di[1:0]} : {shift_in[3:0], io_di[3:0]};
                            addr_reg <= addr_reg + 1;
                        end
                        bit_cnt <= 0;
                        byte_cnt <= byte_cnt + 1;
                        wip <= 1;
                    end
                end
                ST_ERASE: begin
                    state <= ST_IDLE;
                end
                ST_STATUS: begin
                    io_do[1] = shift_out[7];
                    shift_out <= shift_out << 1;
                    bit_cnt <= bit_cnt + 1;
                    if (bit_cnt == 7) begin
                        bit_cnt <= 0;
                        state <= ST_IDLE;
                    end
                end
                ST_ID_READ: begin
                    io_do[1] = shift_out[7];
                    shift_out <= shift_out << 1;
                    bit_cnt <= bit_cnt + 1;
                    if (bit_cnt == 7) begin
                        bit_cnt <= 0;
                        state <= ST_IDLE;
                    end
                end
            endcase
        end
    end

    always @* begin
        status_reg[0] = wip;
        nxt_cmd_reg = {shift_in[6:0], io_di[0]};
        nxt_bit_cnt = bit_cnt + 1;
        nxt_addr_reg = {addr_reg[ADDR_BITS-9:0], lanes == 1 ? {shift_in[6:0], io_di[0]} : lanes == 2 ? {shift_in[5:0], io_di[1:0]} : {shift_in[3:0], io_di[3:0]}};
    end
    

                            
        
endmodule
