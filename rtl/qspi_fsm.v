module qspi_fsm #(
    parameter ADDR_WIDTH = 32
)(
    input wire clk,
    input wire resetn,

    // Kick start & done
    input wire start,   // pulse to start a transaction
    output wire done,

    // CMD_CFG
    input wire [1:0] cmd_lanes_sel,  // 0:1bit,1:2bit,2:4bit (CMD LANES)
    input wire [1:0] addr_lanes_sel, // 0:1bit,1:2bit,2:4bit (ADDR LANES)
    input wire [1:0] data_lanes_sel, // 0:1bit,1:2bit,2:4bit (DATA LANES)
    input wire [1:0] addr_bytes_sel, // 0:0B, 1:3B, 2:4B
    input wire mode_en,              // MODE EN
    input wire [3:0] dummy_cycles,   // Number of dummy clock cycles
    input wire dir,                  // 0:Write(Tx) -1:Read(Rx) (DIR)

    // CMD_OP
    input wire [7:0] cmd_opcode,
    input wire [7:0] mode_bits,

    // CMD_ADDR
    input wire [ADDR_WIDTH-1:0] addr,

    // CMD_LEN
    input wire [31:0] len_bytes,

    // CTRL bits for SPI clocking
    input wire [31:0] clk_div,

    // CPOL and CPHA
    input wire cpol,
    input wire cpha,

    // Write path (DIR=0)
    input wire [31:0] tx_data_fifo,
    output wire tx_ren,
    output wire tx_empty,

    // Read path (DIR=1)
    output reg [31:0] rx_data_fifo,
    output reg rx_wen,
    input wire rx_full,

    // QSPI Pins
    output wire sclk,
    output reg cs_n,
    inout wire io0,
    inout wire io1,
    inout wire io2,
    inout wire io3
);

// CMD OPCODE
// Read
localparam READ       = 8'h03; // Read Data Bytes
localparam FAST_READ  = 8'h0B; // Read Data Bytes at Higher Speed
localparam READ_2     = 8'hBB; // 2 x I/O Read Mode
localparam QREAD      = 8'h6B; // Quad I/O Read Mode
localparam READ_4     = 8'hEB; // 4 x I/O Read Mode

// Write
localparam SE         = 8'h20; // Sector Erase
localparam BE         = 8'hD8; // Block Erase
localparam CE         = 8'h60; // Chip Erase
localparam PP         = 8'h02; // Page Program
localparam PP_4       = 8'h38; // 4 x I/O Page Program
localparam WREN       = 8'h06; // Write Enable

// State Machine
localparam IDLE       = 4'd0,
           CS         = 4'd1,
           CMD        = 4'd2,
           ADDR       = 4'd3,
           MODE       = 4'd4,
           DUMMY      = 4'd5,
           DATA       = 4'd6,
           STOP_CS    = 4'd7;

    reg [3:0] state, next_state;
    reg [31:0] bit_cnt, next_bit_cnt;
    reg [31:0] byte_cnt, next_byte_cnt;
    reg [3:0]  lanes, next_lanes;
    reg [31:0] data_shift_reg, next_data_shift_reg;
    reg [3:0]  io_oe, next_io_oe;
    reg [3:0]  dummy_left, next_dummy_left;

    integer cmd_lanes_eff, addr_lanes_eff, data_lanes_eff;
    integer addr_bits;

    // SCLK engine
    reg        sclk_en, next_sclk_en;
    reg [31:0] sclk_cnt;
    reg        sclk_q;
    reg        sclk_edge;
    reg        sclk_phase;

    // QSPI I/O
    wire [3:0] io_di = {io3, io2, io1, io0};
    reg  [3:0] out_bits;
    reg  [3:0] input_bits;

    // CPHA/CPOL edge decode
    wire shift_pulse  = sclk_edge & (cpha ? (sclk_phase==1'b0) : (sclk_phase==1'b1));
    wire sample_pulse = sclk_edge & ~shift_pulse;
    wire bit_tick     = sample_pulse;

    // Done
    assign done = (state == STOP_CS);

    // Tri-state mapping
    assign io3 = io_oe[3] ? out_bits[3] : 1'bz;
    assign io2 = io_oe[2] ? out_bits[2] : 1'bz;
    assign io1 = io_oe[1] ? out_bits[1] : 1'bz;
    assign io0 = io_oe[0] ? out_bits[0] : 1'bz;

    // Output bit mux
    always @* begin
        case (lanes)
            1: out_bits = {3'b000, data_shift_reg[31]};
            2: out_bits = {2'b00, data_shift_reg[31], data_shift_reg[30]};
            4: out_bits = {data_shift_reg[31], data_shift_reg[30], data_shift_reg[29], data_shift_reg[28]};
            default: out_bits = 4'b0000;
        endcase
        case (lanes)
            1: input_bits = {3'b000, io_di[1]};
            2: input_bits = {2'b00, io_di[1:0]};
            4: input_bits = io_di[3:0];
            default: input_bits = 4'b0000;
        endcase
    end

    // SCLK output
    assign sclk = sclk_en ? sclk_q : cpol;

    // SCLK generator
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            sclk_cnt   <= 0;
            sclk_q     <= cpol;
            sclk_edge  <= 1'b0;
            sclk_phase <= 1'b0;
        end else begin
            sclk_edge <= 1'b0;
            if (!sclk_en) begin
                sclk_cnt   <= 0;
                sclk_q     <= cpol;
                sclk_phase <= 1'b0;
            end else begin
                if (sclk_cnt >= clk_div) begin
                    sclk_cnt   <= 0;
                    sclk_q     <= ~sclk_q;
                    sclk_edge  <= 1'b1;
                    sclk_phase <= ~sclk_phase;
                end else begin
                    sclk_cnt <= sclk_cnt + 1;
                end
            end
        end
    end

    // Sequential state registers
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            state <= IDLE;
            cs_n  <= 1'b1;
            bit_cnt <= 0;
            byte_cnt <= 0;
            lanes <= 1;
            data_shift_reg <= 0;
            io_oe <= 0;
              sclk_en <= 0;
              dummy_left <= 0;
        end else begin
            state <= next_state;
            cs_n  <= (next_state==IDLE || next_state==STOP_CS);
            bit_cnt <= next_bit_cnt;
            byte_cnt <= next_byte_cnt;
            lanes <= next_lanes;
            data_shift_reg <= next_data_shift_reg;
            io_oe <= next_io_oe;
            sclk_en <= next_sclk_en;
            dummy_left <= next_dummy_left;
        end
    end

    // FSM
    always @* begin
        // defaults
        next_state = state;
        next_bit_cnt = bit_cnt;
        next_byte_cnt = byte_cnt;
        next_lanes = lanes;
        next_data_shift_reg = data_shift_reg;
        next_io_oe = io_oe;
        next_sclk_en = 1'b0;
        next_dummy_left = dummy_left;
        rx_wen = 1'b0;
        rx_data_fifo = 32'b0;

        // lane decode
        cmd_lanes_eff  = (cmd_lanes_sel==2'b00)?1:(cmd_lanes_sel==2'b01)?2:4;
        addr_lanes_eff = (addr_lanes_sel==2'b00)?1:(addr_lanes_sel==2'b01)?2:4;
        data_lanes_eff = (data_lanes_sel==2'b00)?1:(data_lanes_sel==2'b01)?2:4;

        case (state)
            IDLE: begin
                if (start) next_state = CS;
            end

            CS: begin
                next_state = CMD;
                next_bit_cnt = 0;
                next_lanes = cmd_lanes_eff;
                next_data_shift_reg = {24'b0, cmd_opcode};
                next_io_oe = (next_lanes==1)?4'b0001:(next_lanes==2)?4'b0011:4'b1111;
            end

            CMD: begin
                next_sclk_en = 1;
                if (shift_pulse) next_data_shift_reg = data_shift_reg << lanes;
                if (bit_tick) begin
                    next_bit_cnt = bit_cnt + lanes;
                    if (next_bit_cnt >= 8) begin
                        next_bit_cnt = 0;
                        case (cmd_opcode)
                            READ, FAST_READ, READ_2, QREAD, READ_4, PP, PP_4: next_state = ADDR;
                            SE, BE: next_state = ADDR;
                            CE, WREN: next_state = STOP_CS;
                            default: next_state = STOP_CS;
                        endcase
                        if (next_state==ADDR) begin
                            next_lanes = addr_lanes_eff;
                            next_data_shift_reg = (addr_bytes_sel==2'b01)?{8'b0,addr[23:0]}:addr;
                            next_io_oe = (next_lanes==1)?4'b0001:(next_lanes==2)?4'b0011:4'b1111;
                        end
                    end
                end
            end

            ADDR: begin
                next_sclk_en = 1;
                if (shift_pulse) next_data_shift_reg = data_shift_reg << lanes;
                  addr_bits = (addr_bytes_sel==2'b01)?24:32;
                if (bit_tick) begin
                    next_bit_cnt = bit_cnt + lanes;
                    if (next_bit_cnt >= addr_bits) begin
                        next_bit_cnt = 0;
                        case (cmd_opcode)
                            READ, FAST_READ, READ_2, QREAD, READ_4: begin
                                if (mode_en && (cmd_opcode==READ_2 || cmd_opcode==READ_4)) begin
                                    next_state = MODE;
                                    next_lanes = data_lanes_eff;
                                    next_data_shift_reg = {24'b0, mode_bits};
                                end else if (dummy_cycles!=0 || cmd_opcode!=READ) begin
                                    next_state = DUMMY;
                                    next_dummy_left = dummy_cycles;
                                end else begin
                                    next_state = DATA;
                                    next_lanes = data_lanes_eff;
                                    next_io_oe = dir?4'b0000:((next_lanes==1)?4'b0001:(next_lanes==2)?4'b0011:4'b1111);
                                    next_data_shift_reg = dir?32'b0:tx_data_fifo;
                                    next_byte_cnt = 0;
                                end
                            end
                            PP, PP_4: begin
                                next_state = DATA;
                                next_lanes = data_lanes_eff;
                                next_io_oe = (next_lanes==1)?4'b0001:(next_lanes==2)?4'b0011:4'b1111;
                                next_data_shift_reg = tx_data_fifo;
                                next_byte_cnt = 0;
                            end
                            SE, BE: next_state = STOP_CS;
                        endcase
                    end
                end
            end

            MODE: begin
                next_sclk_en = 1;
                if (shift_pulse) next_data_shift_reg = data_shift_reg << lanes;
                if (bit_tick) begin
                    next_bit_cnt = bit_cnt + lanes;
                    if (next_bit_cnt >= 8) begin
                        next_bit_cnt = 0;
                        if (dummy_cycles!=0) begin
                            next_state = DUMMY;
                            next_dummy_left = dummy_cycles;
                        end else begin
                            next_state = DATA;
                            next_lanes = data_lanes_eff;
                            next_io_oe = dir?4'b0000:((next_lanes==1)?4'b0001:(next_lanes==2)?4'b0011:4'b1111);
                            next_data_shift_reg = dir?32'b0:tx_data_fifo;
                            next_byte_cnt = 0;
                        end
                    end
                end
            end

            DUMMY: begin
                next_sclk_en = 1;
                next_io_oe = 0;
                if (bit_tick) begin
                    if (dummy_left>0) next_dummy_left = dummy_left - 1;
                    if (dummy_left==1) begin
                        next_state = DATA;
                        next_lanes = data_lanes_eff;
                        next_io_oe = dir?4'b0000:((next_lanes==1)?4'b0001:(next_lanes==2)?4'b0011:4'b1111);
                        next_data_shift_reg = dir?32'b0:tx_data_fifo;
                        next_byte_cnt = 0;
                    end
                end
            end

            DATA: begin
                next_sclk_en = 1;
                if (dir) begin
                    if (sample_pulse) next_data_shift_reg = (data_shift_reg<<lanes)|input_bits;
                    if (bit_tick) begin
                        next_bit_cnt = bit_cnt + lanes;
                        if (next_bit_cnt >= 32) begin
                            if (!rx_full) begin
                                rx_data_fifo = data_shift_reg;
                                rx_wen = 1;
                            end
                            next_data_shift_reg = 0;
                            next_byte_cnt = byte_cnt + 4;
                            next_bit_cnt = 0;
                        end
                        if (byte_cnt+4 >= len_bytes) next_state = STOP_CS;
                    end
                end else begin
                    if (shift_pulse) next_data_shift_reg = data_shift_reg<<lanes;
                    if (bit_tick) begin
                        next_bit_cnt = bit_cnt + lanes;
                        if (next_bit_cnt >= 32) begin
                            next_data_shift_reg = tx_data_fifo;
                            next_byte_cnt = byte_cnt + 4;
                            next_bit_cnt = 0;
                        end
                        if (byte_cnt+4 >= len_bytes) next_state = STOP_CS;
                    end
                end
            end

            STOP_CS: begin
                next_io_oe = 0;
                next_sclk_en = 0;
                next_state = IDLE;
            end
        endcase
    end

    // tx_ren: request FIFO word
    assign tx_ren = (state==DATA) & (~dir) & (bit_cnt+lanes>=32) & (byte_cnt<len_bytes) & ~tx_empty;

endmodule