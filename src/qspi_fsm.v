// qspi_fsm.v - QSPI flash transaction finite state machine
// Generates command, address, dummy and data phases for SPI/Dual/Quad
// operations. Shifts data MSB-first on IO[3:0] and interfaces with FIFO
// blocks for streaming write and read data. Chip select timing honours
// cs_auto and continuous read hold from CSR.

module qspi_fsm #(
    parameter ADDR_WIDTH = 32
)(
    input  wire        clk,
    input  wire        resetn,

    // start & completion
    input  wire        start,
    output wire        done,

    // configuration
    input  wire [1:0]  cmd_lanes_sel,
    input  wire [1:0]  addr_lanes_sel,
    input  wire [1:0]  data_lanes_sel,
    input  wire [1:0]  addr_bytes_sel,   // 0:0B 1:3B 2:4B
    input  wire        mode_en,
    input  wire [3:0]  dummy_cycles,
    input  wire        dir,              // 0:write 1:read
    input  wire        quad_en,
    input  wire        cs_auto,
    input  wire        xip_cont_read,

    // opcode/mode/address
    input  wire [7:0]  cmd_opcode,
    input  wire [7:0]  mode_bits,
    input  wire [ADDR_WIDTH-1:0] addr,
    input  wire [31:0] len_bytes,

    // clock control
    input  wire [31:0] clk_div,
    input  wire        cpol,
    input  wire        cpha,

    // write path
    input  wire [31:0] tx_data_fifo,
    input  wire        tx_empty,
    output wire        tx_ren,

    // read path
    output reg  [31:0] rx_data_fifo,
    output reg         rx_wen,
    input  wire        rx_full,

    // QSPI pins
    output wire        sclk,
    output reg         cs_n,
    inout  wire        io0,
    inout  wire        io1,
    inout  wire        io2,
    inout  wire        io3
);

// ------------------------------------------------------------
// Lane decode helpers
// ------------------------------------------------------------
function [2:0] lane_decode;
    input [1:0] sel;
    begin
        case (sel)
            2'b00: lane_decode = 3'd1;
            2'b01: lane_decode = 3'd2;
            default: lane_decode = quad_en ? 3'd4 : 3'd1;
        endcase
    end
endfunction

function [3:0] lane_mask;
    input [2:0] lanes;
    begin
        case (lanes)
            3'd1: lane_mask = 4'b0001;
            3'd2: lane_mask = 4'b0011;
            3'd4: lane_mask = 4'b1111;
            default: lane_mask = 4'b0000;
        endcase
    end
endfunction

// Opcode specific lane overrides
reg [2:0] cmd_lanes_eff, addr_lanes_eff, data_lanes_eff;
always @* begin
    case (cmd_opcode)
        8'h3B: begin // Dual Output Read 1-1-2
            cmd_lanes_eff  = 3'd1;
            addr_lanes_eff = 3'd1;
            data_lanes_eff = 3'd2;
        end
        8'hBB: begin // Dual I/O Read 1-2-2
            cmd_lanes_eff  = 3'd1;
            addr_lanes_eff = 3'd2;
            data_lanes_eff = 3'd2;
        end
        8'h6B: begin // Quad Output Read 1-1-4
            cmd_lanes_eff  = 3'd1;
            addr_lanes_eff = 3'd1;
            data_lanes_eff = quad_en ? 3'd4 : 3'd1;
        end
        8'hEB: begin // Quad I/O Read 1-4-4
            cmd_lanes_eff  = 3'd1;
            addr_lanes_eff = quad_en ? 3'd4 : 3'd1;
            data_lanes_eff = quad_en ? 3'd4 : 3'd1;
        end
        default: begin
            cmd_lanes_eff  = lane_decode(cmd_lanes_sel);
            addr_lanes_eff = lane_decode(addr_lanes_sel);
            data_lanes_eff = lane_decode(data_lanes_sel);
        end
    endcase
end

wire [5:0] addr_bits = (addr_bytes_sel==2'b01) ? 6'd24 :
                       (addr_bytes_sel==2'b10) ? 6'd32 : 6'd0;

// ------------------------------------------------------------
// SCLK generator
// ------------------------------------------------------------
reg        sclk_en, sclk_en_n;
reg [31:0] sclk_cnt;
reg        sclk_q;
reg        sclk_edge;
reg        sclk_q_prev;

assign sclk = sclk_en ? sclk_q : cpol;

always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
        sclk_cnt   <= 32'd0;
        sclk_q     <= 1'b0;
        sclk_q_prev<= 1'b0;
        sclk_edge  <= 1'b0;
    end else begin
        sclk_edge <= 1'b0;
        if (!sclk_en) begin
            sclk_cnt    <= 32'd0;
            sclk_q      <= cpol;
            sclk_q_prev <= cpol;
        end else begin
            if (sclk_cnt >= clk_div) begin
                sclk_cnt    <= 32'd0;
                sclk_q_prev <= sclk_q;
                sclk_q      <= ~sclk_q;
                sclk_edge   <= 1'b1;
            end else begin
                sclk_cnt <= sclk_cnt + 1;
            end
        end
    end
end

// Edge classification per CPOL/CPHA
wire leading_edge  = sclk_edge & (sclk_q_prev == cpol);   // transition away from idle
wire trailing_edge = sclk_edge & (sclk_q_prev != cpol);   // transition back toward idle

// SPI modes: CPHA=0 => sample on leading, shift on trailing
//            CPHA=1 => shift on leading, sample on trailing
wire sample_pulse = cpha ? trailing_edge : leading_edge;
wire shift_pulse  = cpha ? leading_edge  : trailing_edge;
wire bit_tick     = sample_pulse;

// ------------------------------------------------------------
// IO handling
// ------------------------------------------------------------
reg [2:0] lanes, lanes_n;
reg [31:0] shreg, shreg_n;
reg [5:0]  bit_cnt, bit_cnt_n;
reg [31:0] byte_cnt, byte_cnt_n;
reg [3:0]  dummy_cnt, dummy_cnt_n;
reg [3:0]  io_oe, io_oe_n;

wire [3:0] io_di = {io3, io2, io1, io0};
reg  [3:0] out_bits, in_bits;

always @* begin
    case (lanes)
        3'd1: begin
            // MSB-first on IO0 for single-lane mode; sample input from IO1 (SO)
            out_bits = {3'b000, shreg[31]};
            in_bits  = {3'b000, io_di[1]};
        end
        3'd2: begin
            out_bits = {2'b00, shreg[31], shreg[30]};
            in_bits  = {2'b00, io_di[1:0]};
        end
        3'd4: begin
            out_bits = shreg[31:28];
            in_bits  = io_di[3:0];
        end
        default: begin
            out_bits = 4'b0000;
            in_bits  = 4'b0000;
        end
    endcase
end

assign io0 = io_oe[0] ? out_bits[0] : 1'bz;
assign io1 = io_oe[1] ? out_bits[1] : 1'bz;
assign io2 = io_oe[2] ? out_bits[2] : 1'bz;
assign io3 = io_oe[3] ? out_bits[3] : 1'bz;

// ------------------------------------------------------------
// State machine
// ------------------------------------------------------------
localparam [3:0]
    IDLE      = 4'd0,
    CS_SETUP  = 4'd1,
    CMD_BIT   = 4'd2,
    ADDR_BIT  = 4'd3,
    MODE_BIT  = 4'd4,
    DUMMY_BIT = 4'd5,
    DATA_BIT  = 4'd6,
    CS_HOLD   = 4'd7,
    CS_DONE   = 4'd8;

reg [3:0] state, state_n;
reg       cs_n_n;

assign done = (state == CS_DONE) || (state == CS_HOLD);

always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
        state     <= IDLE;
        cs_n      <= 1'b1;
        lanes     <= 3'd1;
        shreg     <= 32'b0;
        bit_cnt   <= 6'd0;
        byte_cnt  <= 32'd0;
        dummy_cnt <= 4'd0;
        io_oe     <= 4'b0;
        sclk_en   <= 1'b0;
    end else begin
        state     <= state_n;
        cs_n      <= cs_n_n;
        lanes     <= lanes_n;
        shreg     <= shreg_n;
        bit_cnt   <= bit_cnt_n;
        byte_cnt  <= byte_cnt_n;
        dummy_cnt <= dummy_cnt_n;
        io_oe     <= io_oe_n;
        sclk_en   <= sclk_en_n;
    end
end

always @* begin
    // Defaults
    state_n     = state;
    lanes_n     = lanes;
    shreg_n     = shreg;
    bit_cnt_n   = bit_cnt;
    byte_cnt_n  = byte_cnt;
    dummy_cnt_n = dummy_cnt;
    io_oe_n     = 4'b0000;
    sclk_en_n   = 1'b0;
    rx_wen       = 1'b0;
    rx_data_fifo = 32'b0;
    cs_n_n       = cs_n;

    case (state)
        IDLE: begin
            cs_n_n = 1'b1;
            if (start) begin
                state_n   = CS_SETUP;
                lanes_n   = cmd_lanes_eff;
                shreg_n   = {cmd_opcode, 24'b0};
                bit_cnt_n = 6'd0;
            end
        end

        CS_SETUP: begin
            io_oe_n = 4'b0000;
            cs_n_n  = 1'b0;
            state_n = CMD_BIT;
        end

        CMD_BIT: begin
            sclk_en_n = 1'b1;
            io_oe_n   = lane_mask(lanes);
            // Shift on the shifting edge so that outputs settle half-cycle earlier
            if (shift_pulse)
                shreg_n = shreg << lanes;
            if (bit_tick) begin
                bit_cnt_n = bit_cnt + {3'b000, lanes};
                if (bit_cnt_n >= 6'd8) begin
                    bit_cnt_n = 6'd0;
                    if (addr_bits != 0) begin
                        state_n = ADDR_BIT;
                        lanes_n = addr_lanes_eff;
                        shreg_n = (addr_bytes_sel==2'b01) ? {addr[23:0],8'b0} :
                                  (addr_bytes_sel==2'b10) ? addr : 32'b0;
                    end else if (mode_en) begin
                        state_n = MODE_BIT;
                        lanes_n = data_lanes_eff;
                        shreg_n = {mode_bits,24'b0};
                    end else if (dummy_cycles != 0) begin
                        state_n   = DUMMY_BIT;
                        lanes_n   = data_lanes_eff;
                        dummy_cnt_n = dummy_cycles;
                    end else if (len_bytes != 0) begin
                        state_n   = DATA_BIT;
                        lanes_n   = data_lanes_eff;
                        shreg_n   = dir ? 32'b0 : tx_data_fifo;
                        io_oe_n   = dir ? 4'b0000 : lane_mask(data_lanes_eff);
                        byte_cnt_n = 32'd0;
                    end else begin
                        state_n = CS_DONE;
                    end
                end
            end
        end

        ADDR_BIT: begin
            sclk_en_n = 1'b1;
            io_oe_n   = lane_mask(lanes);
            if (sample_pulse)
                shreg_n = shreg << lanes;
            if (bit_tick) begin
                bit_cnt_n = bit_cnt + {3'b000, lanes};
                if (bit_cnt_n >= addr_bits) begin
                    bit_cnt_n = 6'd0;
                    if (mode_en) begin
                        state_n = MODE_BIT;
                        lanes_n = data_lanes_eff;
                        shreg_n = {mode_bits,24'b0};
                    end else if (dummy_cycles != 0) begin
                        state_n     = DUMMY_BIT;
                        lanes_n     = data_lanes_eff;
                        dummy_cnt_n = dummy_cycles;
                    end else if (len_bytes != 0) begin
                        state_n     = DATA_BIT;
                        lanes_n     = data_lanes_eff;
                        shreg_n     = dir ? 32'b0 : tx_data_fifo;
                        io_oe_n     = dir ? 4'b0000 : lane_mask(data_lanes_eff);
                        byte_cnt_n  = 32'd0;
                    end else begin
                        state_n = CS_DONE;
                    end
                end
            end
        end

        MODE_BIT: begin
            sclk_en_n = 1'b1;
            io_oe_n   = lane_mask(lanes);
            if (sample_pulse)
                shreg_n = shreg << lanes;
            if (bit_tick) begin
                bit_cnt_n = bit_cnt + {3'b000, lanes};
                if (bit_cnt_n >= 6'd8) begin
                    bit_cnt_n = 6'd0;
                    if (dummy_cycles != 0) begin
                        state_n   = DUMMY_BIT;
                        lanes_n   = data_lanes_eff;
                        dummy_cnt_n = dummy_cycles;
                    end else if (len_bytes != 0) begin
                        state_n   = DATA_BIT;
                        lanes_n   = data_lanes_eff;
                        shreg_n   = dir ? 32'b0 : tx_data_fifo;
                        io_oe_n   = dir ? 4'b0000 : lane_mask(data_lanes_eff);
                        byte_cnt_n = 32'd0;
                    end else begin
                        state_n = CS_DONE;
                    end
                end
            end
        end

        DUMMY_BIT: begin
            sclk_en_n = 1'b1;
            io_oe_n   = 4'b0000;
            if (bit_tick) begin
                if (dummy_cnt != 0)
                    dummy_cnt_n = dummy_cnt - 1;
                if (dummy_cnt == 1) begin
                    if (len_bytes != 0) begin
                        state_n   = DATA_BIT;
                        lanes_n   = data_lanes_eff;
                        shreg_n   = dir ? 32'b0 : tx_data_fifo;
                        io_oe_n   = dir ? 4'b0000 : lane_mask(data_lanes_eff);
                        byte_cnt_n = 32'd0;
                    end else begin
                        state_n = CS_DONE;
                    end
                end
            end
        end

        DATA_BIT: begin
            sclk_en_n = 1'b1;
            io_oe_n   = dir ? 4'b0000 : lane_mask(lanes);
            if (dir) begin
                if (sample_pulse)
                    shreg_n = (shreg << lanes) | {28'b0, in_bits};
                if (bit_tick) begin
                    bit_cnt_n = bit_cnt + {3'b000, lanes};
                    if (bit_cnt_n >= 6'd32) begin
                        bit_cnt_n = 6'd0;
                        byte_cnt_n = byte_cnt + 4;
                        if (!rx_full) begin
                            rx_wen = 1'b1;
                            rx_data_fifo = shreg;
                        end
                        shreg_n = 32'b0;
                    end
                    if (byte_cnt_n >= len_bytes) begin
                        if (xip_cont_read && !cs_auto)
                            state_n = CS_HOLD;
                        else
                            state_n = CS_DONE;
                    end
                end
            end else begin
                if (sample_pulse)
                    shreg_n = shreg << lanes;
                if (bit_tick) begin
                    bit_cnt_n = bit_cnt + {3'b000, lanes};
                    if (bit_cnt_n >= 6'd32) begin
                        bit_cnt_n  = 6'd0;
                        byte_cnt_n = byte_cnt + 4;
                        if ((byte_cnt + 4) < len_bytes && !tx_empty)
                            shreg_n = tx_data_fifo;
                        else
                            shreg_n = 32'b0;
                    end
                    if (byte_cnt_n >= len_bytes)
                        state_n = CS_DONE;
                end
            end
        end

        CS_HOLD: begin
            cs_n_n = 1'b0;
            if (cs_auto)
                state_n = CS_DONE;
        end

        CS_DONE: begin
            cs_n_n  = 1'b1;
            state_n = IDLE;
        end

        default: state_n = IDLE;
    endcase
end

// ------------------------------------------------------------
// TX FIFO read enable
// ------------------------------------------------------------
assign tx_ren = (state==DATA_BIT) && !dir &&
                (bit_cnt + {3'b000, lanes} >= 6'd32) &&
                ((byte_cnt + 4) < len_bytes) && !tx_empty;

endmodule
