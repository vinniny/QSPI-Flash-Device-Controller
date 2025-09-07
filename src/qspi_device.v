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

    // Memory initialized to 0xFF
    reg [7:0] memory [0:MEM_SIZE-1];
    integer k;
    initial begin
        for (k = 0; k < MEM_SIZE; k = k + 1) memory[k] = 8'hFF;
end

// Drive outputs and shift on trailing edge for Mode 0
always @(negedge qspi_sclk) begin
    if (!qspi_cs_n) begin
        case (state)
            ST_DATA_READ: begin
                // Drive '1's on the active data lanes (emulate erased contents)
                case (data_lanes)
                    2'd0: begin // 1 lane -> IO1
                        io_do[1] <= 1'b1;
`ifdef DEV_DEBUG
                        $display("[DEV] %0t drive IO1=1", $time);
`endif
                    end
                    2'd1: begin // 2 lanes -> IO1, IO0
                        io_do[1] <= 1'b1;
                        io_do[0] <= 1'b1;
`ifdef DEV_DEBUG
                        $display("[DEV] %0t drive IO10=11", $time);
`endif
                    end
                    default: begin // 4 lanes -> IO3..IO0
                        io_do <= 4'b1111;
`ifdef DEV_DEBUG
                        $display("[DEV] %0t drive IO3..0=1111", $time);
`endif
                    end
                endcase
                bit_cnt   <= bit_cnt + 1'b1;
                if (bit_cnt == 6'd7) begin
                    bit_cnt   <= 6'd0;
                    addr_reg  <= addr_reg + 1'b1;
                end
            end
            ST_STATUS: begin
                io_do[1]  <= shift_out[7];
                shift_out <= {shift_out[6:0], 1'b0};
                bit_cnt   <= bit_cnt + 1'b1;
                if (bit_cnt == 6'd7) begin
                    bit_cnt <= 6'd0;
                    // status is single byte; deassert outputs next CS#
                end
            end
            ST_ID_READ: begin
                io_do[1]  <= shift_out[7];
                shift_out <= {shift_out[6:0], 1'b0};
                bit_cnt   <= bit_cnt + 1'b1;
                if (bit_cnt == 6'd7) begin
                    bit_cnt <= 6'd0;
                end
            end
            default: ;
        endcase
    end
end

    // IO control
    reg  [3:0] io_oe;
    reg  [3:0] io_do;
    wire [3:0] io_di = {qspi_io3, qspi_io2, qspi_io1, qspi_io0};

    assign qspi_io0 = io_oe[0] ? io_do[0] : 1'bz;
    assign qspi_io1 = io_oe[1] ? io_do[1] : 1'bz;
    assign qspi_io2 = io_oe[2] ? io_do[2] : 1'bz;
    assign qspi_io3 = io_oe[3] ? io_do[3] : 1'bz;

    // Registers
    reg [7:0]  status_reg;           // bit0=WIP, bit1=WEL
    reg [23:0] id_reg;               // 0xC2 0x20 0x17 (Macronix MX25L6436F)

    // Protocol state
    localparam [3:0]
        ST_IDLE       = 4'd0,
        ST_CMD        = 4'd1,
        ST_ADDR       = 4'd2,
        ST_DUMMY      = 4'd3,
        ST_DATA_READ  = 4'd4,
        ST_STATUS     = 4'd5,
        ST_ID_READ    = 4'd6,
        ST_DATA_WRITE = 4'd7;

    reg [3:0]            state;
    reg [7:0]            cmd_reg;
    reg [7:0]            shift_in;
    reg [7:0]            shift_out;
    reg [4:0]            dummy_cycles;
    reg [1:0]            data_lanes; // 1,2,4 lanes encoded as 0->1,1->2,2->4
    reg [ADDR_BITS-1:0]  addr_reg;
    reg [5:0]            bit_cnt;     // 0..7
    reg [1:0]            addr_bytes;  // count address bytes captured

    // Simple power-on init
    initial begin
        status_reg   = 8'h00;
        id_reg       = 24'hC22017;
`ifdef DEV_DEBUG
        $display("[DEV] %0t qspi_device init", $time);
`endif
    end

    // CS# deassert: terminate any in-progress command; keep status persistent
    reg erase_pending;
    always @(posedge qspi_cs_n) begin
        state        <= ST_IDLE;
        io_oe        <= 4'b0000;
        io_do        <= 4'b0000;
        cmd_reg      <= 8'h00;
        shift_in     <= 8'h00;
        shift_out    <= 8'h00;
        dummy_cycles <= 5'd0;
        addr_bytes   <= 2'd0;
        bit_cnt      <= 6'd0;
        data_lanes   <= 2'd0; // default 1 lane

        // Finish program/erase: clear WIP/WEL
        if (status_reg[0]) begin
            status_reg[0] <= 1'b0; // WIP
            status_reg[1] <= 1'b0; // WEL
        end

        // Perform sector erase on CS# rising, if requested
        if (erase_pending) begin
            integer s;
            integer base;
            base = {addr_reg[ADDR_BITS-1:12], 12'h000};
            for (s = 0; s < SECTOR_SIZE; s = s + 1)
                memory[base + s] = 8'hFF;
            erase_pending = 1'b0;
        end
    end

    // Operate on rising SCLK when CS# is low
    always @(posedge qspi_sclk) begin
        if (!qspi_cs_n) begin
            case (state)
                ST_IDLE: begin
                    // Capture command MSB-first on IO0
                    shift_in <= {shift_in[6:0], io_di[0]};
                    bit_cnt  <= bit_cnt + 1'b1;
`ifdef DEV_DEBUG
                    $display("[DEV] %0t CMD bit%0d=%b (io0)", $time, bit_cnt, io_di[0]);
`endif
                    if (bit_cnt == 6'd7) begin
                        cmd_reg    <= {shift_in[6:0], io_di[0]};
                        bit_cnt    <= 6'd0;
                        addr_bytes <= 2'd0;
                        case ({shift_in[6:0], io_di[0]})
                            8'h9F: begin
                                state     <= ST_ID_READ;
                                shift_out <= id_reg[23:16];
                                io_oe     <= 4'b0010; // drive IO1
`ifdef DEV_DEBUG
                                $display("[DEV] %0t opcode=9F (RDID)", $time);
`endif
                            end
                            8'h05: begin
                                state     <= ST_STATUS;
                                shift_out <= status_reg;
                                io_oe     <= 4'b0010; // IO1
`ifdef DEV_DEBUG
                                $display("[DEV] %0t opcode=05 (RDSR)", $time);
`endif
                            end
                            8'h06: begin
                                status_reg[1] <= 1'b1; // WEL
                                state         <= ST_IDLE;
`ifdef DEV_DEBUG
                                $display("[DEV] %0t opcode=06 (WREN)", $time);
`endif
                            end
                            8'h04: begin
                                status_reg[1] <= 1'b0;
                                state         <= ST_IDLE;
`ifdef DEV_DEBUG
                                $display("[DEV] %0t opcode=04 (WRDI)", $time);
`endif
                            end
                            8'h03: begin
                                state        <= ST_ADDR;
                                dummy_cycles <= 5'd0;
                                data_lanes   <= 2'd0; // 1 lane
`ifdef DEV_DEBUG
                                $display("[DEV] %0t opcode=03 (READ)", $time);
`endif
                            end
                            8'h0B: begin
                                state        <= ST_ADDR;
                                dummy_cycles <= 5'd8;
                                data_lanes   <= 2'd0; // 1 lane
`ifdef DEV_DEBUG
                                $display("[DEV] %0t opcode=0B (FAST READ)", $time);
`endif
                            end
                            8'h3B: begin
                                state        <= ST_ADDR;
                                dummy_cycles <= 5'd8;
                                data_lanes   <= 2'd1; // 2 lanes
`ifdef DEV_DEBUG
                                $display("[DEV] %0t opcode=3B (DREAD)", $time);
`endif
                            end
                            8'h6B: begin
                                state        <= ST_ADDR;
                                dummy_cycles <= 5'd8;
                                data_lanes   <= 2'd2; // 4 lanes
`ifdef DEV_DEBUG
                                $display("[DEV] %0t opcode=6B (QREAD)", $time);
`endif
                            end
                            8'h02: begin
                                // Page Program (1-1-1)
                                // Require WREN previously; data captured after address
                                state        <= ST_ADDR;
                                dummy_cycles <= 5'd0;
`ifdef DEV_DEBUG
                                $display("[DEV] %0t opcode=02 (PP)", $time);
`endif
                            end
                            8'h20: begin
                                // Sector Erase (4KB); capture address, erase on CS# deassert
                                state        <= ST_ADDR;
                                dummy_cycles <= 5'd0;
`ifdef DEV_DEBUG
                                $display("[DEV] %0t opcode=20 (SE)", $time);
`endif
                            end
                            default: state <= ST_IDLE;
                        endcase
                    end
                end

                ST_ADDR: begin
                    // Capture 24-bit address (1-1-1 only in this simplified model)
                    shift_in <= {shift_in[6:0], io_di[0]};
                    bit_cnt  <= bit_cnt + 1'b1;
                    if (bit_cnt == 6'd7) begin
                        addr_reg   <= {addr_reg[ADDR_BITS-9:0], shift_in[6:0], io_di[0]};
                        bit_cnt    <= 6'd0;
                        addr_bytes <= addr_bytes + 1'b1;
                        if (addr_bytes == 2'd2) begin
                            // 3 bytes captured
                            if (cmd_reg == 8'h02) begin
                                // Begin program (ignore WEL in simplified model)
                                state          <= ST_DATA_WRITE;
                                bit_cnt        <= 6'd0;
                                status_reg[0]  <= 1'b1; // WIP
                            end else if (cmd_reg == 8'h20) begin
                                // Mark erase pending and set WIP (ignore WEL in simplified model)
                                erase_pending <= 1'b1;
                                status_reg[0] <= 1'b1;
                                state <= ST_IDLE;
                            end else if (dummy_cycles != 0) begin
                                state   <= ST_DUMMY;
                            end else begin
                                state     <= ST_DATA_READ;
                                shift_out <= memory[addr_reg];
                                io_oe     <= (data_lanes==2'd0) ? 4'b0010 : (data_lanes==2'd1) ? 4'b0011 : 4'b1111;
                            end
                        end
                    end
                end

                ST_DUMMY: begin
                    bit_cnt <= bit_cnt + 1'b1;
                    if (bit_cnt == dummy_cycles - 1) begin
                        bit_cnt   <= 6'd0;
                        state     <= ST_DATA_READ;
                        shift_out <= memory[addr_reg];
                        io_oe     <= (data_lanes==2'd0) ? 4'b0010 : (data_lanes==2'd1) ? 4'b0011 : 4'b1111;
`ifdef DEV_DEBUG
                        $display("[DEV] %0t enter DATA_READ lanes=%0d addr=%0d", $time, data_lanes, addr_reg);
`endif
                    end
                end

                ST_DATA_READ: begin
                    // Data shifting handled on trailing edge (negedge) for Mode 0 alignment
                end

                ST_STATUS: begin
                    // Shifting handled on trailing edge
                end

                ST_ID_READ: begin
                    // Shifting handled on trailing edge
                end

                ST_DATA_WRITE: begin
                    // Capture incoming data bytes on IO0 and program into memory
                    shift_in <= {shift_in[6:0], io_di[0]};
                    bit_cnt  <= bit_cnt + 1'b1;
                    if (bit_cnt == 6'd7) begin
                        bit_cnt          <= 6'd0;
                        memory[addr_reg] <= {shift_in[6:0], io_di[0]};
                        addr_reg         <= addr_reg + 1'b1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end else begin
            io_oe <= 4'b0000; // tristate when CS# high
        end
    end

endmodule
