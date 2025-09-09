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
    input  wire [1:0]  cs_delay,        // extra setup/hold cycles (from CSR)
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
reg        sclk_armed;   // ensure first toggle occurs one cycle after enable

assign sclk = sclk_en ? sclk_q : cpol;

always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
        sclk_cnt   <= 32'd0;
        sclk_q     <= 1'b0;
        sclk_q_prev<= 1'b0;
        sclk_edge  <= 1'b0;
        sclk_armed <= 1'b0;
    end else begin
        sclk_edge <= 1'b0;
        if (!sclk_en) begin
            sclk_cnt    <= 32'd0;
            sclk_q      <= cpol;
            sclk_q_prev <= cpol;
            sclk_armed  <= 1'b0;
        end else begin
            // arm on first cycle after enable to provide data setup time
            if (!sclk_armed) begin
                sclk_armed <= 1'b1;
                sclk_cnt   <= 32'd0;
            end else if (sclk_cnt >= clk_div) begin
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

// Byte bit-reversal helper (MSB<->LSB within each 8-bit lane)
function [7:0] rev8;
    input [7:0] b;
    begin
        rev8 = {b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]};
    end
endfunction

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
    CS_DONE   = 4'd8,
    ERASE     = 4'd9,  // explicit state for erase-type ops (e.g., 0x20 sector erase)
    WR_SETUP  = 4'd10, // one-cycle setup before write DATA to allow TX prefetch
    RD_SETUP  = 4'd11; // one-cycle setup before read DATA to ensure IO tri-state

reg [3:0] state, state_n;
reg       cs_n_n;
// Expand CS hold counter to allow longer post-command high times
reg [7:0] cs_cnt, cs_cnt_n;
// One-shot warmup for READ data to avoid sampling too early
reg       rd_warmup, rd_warmup_n;
reg [3:0] rd_warmup_cnt, rd_warmup_cnt_n; // number of sample ticks to skip at read start
// Latch command direction to decide post-command hold extension for writes
reg       is_write_cmd, is_write_cmd_n;
reg       post_hold_write, post_hold_write_n;

// Parameter: additional CS# high cycles after write-like commands
// to meet tSHSL_W requirements of external flash models (controller-centric)
localparam integer POST_WRITE_HOLD_CYCLES = 64; // core cycles (was 8)

// Scale CS delay units (from CSR[CS_CTRL[4:3]]) to core cycles to
// provide longer setup/hold without changing register map semantics.
localparam integer CS_DELAY_SHIFT = 4; // multiply by 16

// Derived CS delay cycles (8-bit) from CSR field
wire [7:0] cs_delay_cycles = ({6'd0, cs_delay} << CS_DELAY_SHIFT);

// Assert done only when CS_DONE post-hold has completed to avoid
// accepting the next command while CS# high time is still being enforced.
assign done = ((state == CS_DONE) && (cs_cnt == 8'd0)) || (state == CS_HOLD);

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
        cs_cnt    <= 8'd0;
        rd_warmup <= 1'b0;
        rd_warmup_cnt <= 4'd0;
        is_write_cmd <= 1'b0;
        post_hold_write <= 1'b0;
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
        cs_cnt    <= cs_cnt_n;
        rd_warmup <= rd_warmup_n;
        rd_warmup_cnt <= rd_warmup_cnt_n;
        is_write_cmd <= is_write_cmd_n;
        post_hold_write <= post_hold_write_n;
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
    cs_cnt_n     = cs_cnt;
    is_write_cmd_n = is_write_cmd;
    post_hold_write_n = post_hold_write;
    rd_warmup_n  = rd_warmup;
    rd_warmup_cnt_n = rd_warmup_cnt;

    // Identify erase-type commands (sector/block/chip)
    // Used to optionally enter ERASE state after address phase
    // Supported opcodes: 0x20 (4KB sector), 0xD8 (64KB block), 0xC7/0x60 (chip erase)
    // For non-addressed erases (chip erase), transition may occur directly after CMD_BIT
    // when len_bytes==0 and no dummy/mode phases are configured.
    // This FSM does not time the erase operation; higher-level logic polls status.
    
    case (state)
        IDLE: begin
            cs_n_n = 1'b1;
            if (start) begin
                state_n   = CS_SETUP;
                lanes_n   = cmd_lanes_eff;
                shreg_n   = {cmd_opcode, 24'b0};
                bit_cnt_n = 6'd0;
                cs_cnt_n  = {6'd0, cs_delay};  // load CS setup counter
                is_write_cmd_n = ~dir;          // latch direction for this command (dir=0 means write data phase)
                // Treat certain opcodes as write-like (require extra CS# post hold)
                // WREN(0x06), WRSR(0x01), PP(0x02), SE(0x20), BE(0xD8), CE(0xC7/0x60)
                post_hold_write_n = (~dir) ||
                                     (cmd_opcode==8'h06) || (cmd_opcode==8'h01) || (cmd_opcode==8'h02) ||
                                     (cmd_opcode==8'h20) || (cmd_opcode==8'hD8) ||
                                     (cmd_opcode==8'hC7) || (cmd_opcode==8'h60);
            end
        end

        CS_SETUP: begin
            io_oe_n = 4'b0000;
            cs_n_n  = 1'b0;
            // hold CS low for programmable setup cycles before SCLK starts
            if (cs_cnt != 0)
                cs_cnt_n = cs_cnt - 1'b1;
            else
                state_n = CMD_BIT;
        end

        CMD_BIT: begin
            sclk_en_n = 1'b1;
            io_oe_n   = lane_mask(lanes);
            // Shift on the shifting edge so that outputs settle half-cycle earlier
            // For command phase, shift one edge earlier to ensure MSB alignment
            if (sample_pulse)
                shreg_n = shreg << lanes;
            if (bit_tick) begin
`ifdef FSM_DEBUG
                $display("[FSM] %0t CMD bit=%0d out=%b", $time, bit_cnt, out_bits[0]);
`endif
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
                        lanes_n     = data_lanes_eff;
                        io_oe_n     = dir ? 4'b0000 : lane_mask(data_lanes_eff);
                        byte_cnt_n  = 32'd0;
                        if (dir) begin
                            state_n = RD_SETUP; // ensure IO released before sampling
                            rd_warmup_n = 1'b1; // and burn first sampling edge
                        end
                        else
                            state_n = WR_SETUP; // allow TX data to be prefetched and latched
                    end else if ((cmd_opcode==8'h20) || (cmd_opcode==8'hD8) || (cmd_opcode==8'hC7) || (cmd_opcode==8'h60)) begin
                        // Chip/block erase opcodes without address phase
                        state_n = ERASE;
                    end else begin
                        state_n = CS_DONE;
                        cs_cnt_n = cs_delay_cycles + (post_hold_write ? POST_WRITE_HOLD_CYCLES[7:0] : 8'd0);
                    end
                end
            end
        end

        ADDR_BIT: begin
            sclk_en_n = 1'b1;
            io_oe_n   = lane_mask(lanes);
            if (shift_pulse)
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
                        lanes_n     = data_lanes_eff;
                        io_oe_n     = dir ? 4'b0000 : lane_mask(data_lanes_eff);
                        byte_cnt_n  = 32'd0;
                        if (dir) begin
                            state_n = RD_SETUP;
                            rd_warmup_n = 1'b1;
                        end
                        else
                            state_n = WR_SETUP;
                    end else if ((cmd_opcode==8'h20) || (cmd_opcode==8'hD8)) begin
                        // Addressed erase commands (sector/block)
                        state_n = ERASE;
                    end else begin
                        state_n = CS_DONE;
                        cs_cnt_n = cs_delay_cycles + (post_hold_write ? POST_WRITE_HOLD_CYCLES[7:0] : 8'd0);
                    end
                end
            end
        end

        MODE_BIT: begin
            sclk_en_n = 1'b1;
            io_oe_n   = lane_mask(lanes);
            if (shift_pulse)
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
                        lanes_n     = data_lanes_eff;
                        io_oe_n     = dir ? 4'b0000 : lane_mask(data_lanes_eff);
                        byte_cnt_n  = 32'd0;
                        if (dir) begin
                            state_n = RD_SETUP;
                            rd_warmup_n = 1'b1;
                        end
                        else
                            state_n = WR_SETUP;
                    end else begin
                        state_n = CS_DONE;
                        cs_cnt_n = cs_delay_cycles + (post_hold_write ? POST_WRITE_HOLD_CYCLES[7:0] : 8'd0);
                    end
                end
            end
        end

        // One-cycle read setup to guarantee IO lines are released before sampling
        RD_SETUP: begin
            sclk_en_n = 1'b0;                     // hold SCLK idle
            io_oe_n   = 4'b0000;                  // tri-state all IO
            // Add SCLK-period warmup specifically for 0x03/0x05 (no dummy)
            if (cmd_opcode==8'h03)
                rd_warmup_cnt_n = 4'd2;          // skip 2 sample ticks
            else if (cmd_opcode==8'h05)
                rd_warmup_cnt_n = 4'd4;          // extra warmup for RDSR (align bit boundary)
            else
                rd_warmup_cnt_n = 4'd0;
            state_n   = DATA_BIT;                 // proceed next cycle
        end

        DUMMY_BIT: begin
            sclk_en_n = 1'b1;
            io_oe_n   = 4'b0000;
            if (bit_tick) begin
                if (dummy_cnt != 0)
                    dummy_cnt_n = dummy_cnt - 1;
                if (dummy_cnt == 1) begin
                    if (len_bytes != 0) begin
                        state_n      = DATA_BIT;
                        lanes_n      = data_lanes_eff;
                        shreg_n      = dir ? 32'b0 : tx_data_fifo;
                        io_oe_n      = dir ? 4'b0000 : lane_mask(data_lanes_eff);
                        byte_cnt_n   = 32'd0;
                        rd_warmup_n  = dir ? 1'b1 : 1'b0; // skip first sample after dummy for reads
                        rd_warmup_cnt_n = 4'd0;
                    end else begin
                        state_n = CS_DONE;
                        cs_cnt_n = {6'd0, cs_delay} + (post_hold_write ? POST_WRITE_HOLD_CYCLES[7:0] : 8'd0);
                    end
                end
            end
        end

        DATA_BIT: begin
            sclk_en_n = 1'b1;
            io_oe_n   = dir ? 4'b0000 : lane_mask(lanes);
            if (dir) begin
                // Skip early sample edges if requested
                if ( (rd_warmup_cnt==4'd0) && !rd_warmup ) begin
                    if (sample_pulse) begin
                        // Append freshly sampled input bits (per current lane width)
                        case (lanes)
                            3'd1: shreg_n = (shreg << 1) | {31'b0, in_bits[0]};
                            3'd2: shreg_n = (shreg << 2) | {30'b0, in_bits[1:0]};
                            3'd4: shreg_n = (shreg << 4) | {28'b0, in_bits[3:0]};
                            default: shreg_n = (shreg << 1);
                        endcase
                    end
                end
                if (bit_tick) begin
                    if (rd_warmup_cnt != 4'd0) begin
                        rd_warmup_cnt_n = rd_warmup_cnt - 1'b1; // consume SCLK periods
                    end else if (rd_warmup) begin
                        rd_warmup_n = 1'b0;                    // burn one extra sample edge
                    end else begin
                        bit_cnt_n = bit_cnt + {3'b000, lanes};
                        if (bit_cnt_n >= 6'd32) begin
                            bit_cnt_n = 6'd0;
                            byte_cnt_n = byte_cnt + 4;
                            if (!rx_full) begin
                                rx_wen = 1'b1;
                                // For status register reads (0x05) with no data lanes change,
                                // flip bit order within each byte to present MSB-first bytes.
                                if (cmd_opcode == 8'h05) begin
                                    rx_data_fifo = {rev8(shreg_n[31:24]), rev8(shreg_n[23:16]), rev8(shreg_n[15:8]), rev8(shreg_n[7:0])};
                                end else begin
                                    rx_data_fifo = shreg_n;
                                end
                            end
                            shreg_n = 32'b0;
                        end
                        if (byte_cnt_n >= len_bytes) begin
                            if (xip_cont_read && !cs_auto)
                                state_n = CS_HOLD;
                            else begin
                                state_n = CS_DONE;
                                cs_cnt_n = {6'd0, cs_delay} + (post_hold_write ? POST_WRITE_HOLD_CYCLES[7:0] : 8'd0);
                            end
                        end
                    end
                end
            end else begin
`ifdef FSM_DEBUG
                if (bit_cnt==6'd0 && byte_cnt==32'd0) begin
                    $display("[FSM] %0t WRITE start shreg=%08h lanes=%0d", $time, shreg, lanes);
                end
`endif
                if (shift_pulse) begin
`ifdef FSM_DEBUG
                    $display("[FSM] %0t WRITE shift out=%b shreg=%08h", $time, out_bits[0], shreg);
`endif
                    shreg_n = shreg << lanes;
                end
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
                    if (byte_cnt_n >= len_bytes) begin
                        state_n = CS_DONE;
                        cs_cnt_n = {6'd0, cs_delay} + (post_hold_write ? POST_WRITE_HOLD_CYCLES[7:0] : 8'd0);
                    end
                end
            end
        end

        // One-cycle write setup to latch first TX word before data shifting begins
        WR_SETUP: begin
            sclk_en_n = 1'b0;                     // hold SCLK idle
            io_oe_n   = lane_mask(lanes);         // drive IO for write
            shreg_n   = tx_data_fifo;             // latch first word from FIFO
            state_n   = DATA_BIT;                 // proceed next cycle
        end

        CS_HOLD: begin
            cs_n_n = 1'b0;
            if (cs_auto) begin
                state_n = CS_DONE;
                cs_cnt_n = cs_delay;
            end
        end

        CS_DONE: begin
            cs_n_n  = 1'b1;
            // enforce a minimum CS# high time using cs_delay
            if (cs_cnt != 0)
                cs_cnt_n = cs_cnt - 1'b1;
            else
                state_n = IDLE;
        end

        // Erase state: no data transfer; maintain CS low for one cycle
        // and then finish the command to allow the device/model to latch
        // the erase request on CS rising edge.
        ERASE: begin
            cs_n_n    = 1'b0;
            sclk_en_n = 1'b0;
            state_n   = CS_DONE;
            cs_cnt_n  = {6'd0, cs_delay} + (post_hold_write ? POST_WRITE_HOLD_CYCLES[7:0] : 8'd0);
        end

        default: state_n = IDLE;
    endcase
end

// ------------------------------------------------------------
// TX FIFO read enable
// ------------------------------------------------------------
assign tx_ren = !dir && (
                // Prefetch first TX word just before entering DATA_BIT
                ((state==CMD_BIT)  && (addr_bits==6'd0) && !mode_en && (dummy_cycles==4'd0) && (len_bytes!=32'd0) && bit_tick &&
                   (bit_cnt + {3'b000, lanes} >= 6'd8) && !tx_empty) ||
                ((state==ADDR_BIT) && bit_tick && (bit_cnt + {3'b000, lanes} >= addr_bits) && !mode_en && (dummy_cycles==4'd0) && (len_bytes!=32'd0) && !tx_empty) ||
                ((state==MODE_BIT) && bit_tick && (bit_cnt + {3'b000, lanes} >= 6'd8) && (dummy_cycles==4'd0) && (len_bytes!=32'd0) && !tx_empty) ||
                ((state==DUMMY_BIT) && bit_tick && (dummy_cnt==4'd1) && (len_bytes!=32'd0) && !tx_empty) ||
                // Subsequent words during DATA phase
                ((state==DATA_BIT) && (bit_cnt + {3'b000, lanes} >= 6'd32) && ((byte_cnt + 4) < len_bytes) && !tx_empty)
               );

// ------------------------------------------------------------
// Optional debug: print state transitions and first RX word
// Enable by compiling with -D FSM_DEBUG
// ------------------------------------------------------------
`ifdef FSM_DEBUG
reg [3:0] state_q;
reg       rx_wen_q;
always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
        state_q  <= IDLE;
        rx_wen_q <= 1'b0;
    end else begin
        if (state != state_q) begin
            $display("[FSM] %0t state %0d -> %0d (op=%02h len=%0d lanes_cmd=%0d/addr=%0d/data=%0d dir=%0d)",
                     $time, state_q, state, cmd_opcode, len_bytes, cmd_lanes_eff, addr_lanes_eff, data_lanes_eff, dir);
        end
        if (rx_wen && !rx_wen_q) begin
            $display("[FSM] %0t RX_WEN data=%08h io1=%b lanes=%0d", $time, rx_data_fifo, io1, lanes);
        end
        state_q  <= state;
        rx_wen_q <= rx_wen;
    end
end
`endif

endmodule
