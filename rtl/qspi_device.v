module qspi_device (
    input wire qspi_sclk,
    input wire qspi_cs_n,
    inout wire qspi_io0,
    inout wire qspi_io1,
    inout wire qspi_io2,
    inout wire qspi_io3
);

    parameter MEM_SIZE 	  = 1024 * 1024;
    parameter ADDR_BITS   = 24;
    parameter PAGE_SIZE   = 256;
    parameter SECTOR_SIZE = 4096;
    parameter ERASE_TIME  = 100;

    reg [7:0] memory [0:MEM_SIZE-1];

    reg [3:0] io_oe  = 0;
    reg [3:0] io_do  = 0;
    wire [3:0] io_di = {qspi_io3, qspi_io2, qspi_io1, qspi_io0};

    assign qspi_io0 = io_oe[0] ? io_do[0] : 1'bz;
    assign qspi_io1 = io_oe[1] ? io_do[1] : 1'bz;
    assign qspi_io2 = io_oe[2] ? io_do[2] : 1'bz;
    assign qspi_io3 = io_oe[3] ? io_do[3] : 1'bz;

    reg [7:0] 		status_reg 	 = 8'h00;
    reg [31:0] 		id_reg 		 = 32'h00C22017;
    reg [7:0] 		cmd_reg 	 = 0;
    reg [7:0] 		nxt_cmd_reg 	 = 0;
    reg [ADDR_BITS-1:0] addr_reg 	 = 0;
    reg [7:0] 		mode_bits 	 = 0;
    reg [7:0]		shift_in 	 = 0;
    reg [7:0]		shift_out 	 = 0;
    reg [3:0]		lanes 		 = 0;
    reg [4:0]		dummy_cycles 	 = 0;
    reg			continuous_read = 0;

    localparam ST_IDLE 		= 4'h0;
    localparam ST_CMD 		= 4'h1;
    localparam ST_ADDR 		= 4'h2;
    localparam ST_MODE 		= 4'h3;
    localparam ST_DUMMY 	= 4'h4;
    localparam ST_DATA_READ 	= 4'h5;
    localparam ST_DATA_WRITE 	= 4'h6;
    localparam ST_ERASE 	= 4'h7;
    localparam ST_STATUS 	= 4'h8;
    localparam ST_ID_READ 	= 4'h9;
    reg [3:0] state = ST_IDLE;

    reg [31:0] 	bit_cnt 	= 0;
    reg [31:0] 	nxt_bit_cnt 	= 0;
    reg [31:0] 	nxt_addr_reg 	= 0;
    reg [31:0] 	byte_cnt 	= 0;
    reg 	wip 		= 0;
    reg [31:0] 	erase_counter 	= 0;


    initial begin
    	integer i;
    	for (i = 0; i < MEM_SIZE; i = i + 1) begin
            memory[i] = 8'hFF;
    	end
    end

    always @(posedge qspi_sclk) begin
    	if (wip)
            erase_counter <= erase_counter + 1;

    	if (erase_counter >= ERASE_TIME) begin
            wip <= 0;
            erase_counter <= 0;
    	end
    end

    always @(posedge qspi_sclk or posedge qspi_cs_n) begin
        if (qspi_cs_n) begin
            state <= ST_IDLE;
            io_oe <= 4'b0000;
            continuous_read <= 0;
            shift_in <= 0;
    	end else begin
            case (state)
            	ST_IDLE: begin
                      	state <= ST_CMD;
                	bit_cnt <= 0;
                	lanes <= 1;
                	shift_in <= {shift_in[6:0], io_di[0]};
                end

                ST_CMD: begin
                	shift_in <= {shift_in[6:0], io_di[0]};
                	bit_cnt <= bit_cnt + 1;
                	if (nxt_bit_cnt == 7) begin
                    	    cmd_reg <= {shift_in[6:0], io_di[0]};
			    bit_cnt <= 0;
			    byte_cnt <= 0;
			    case (nxt_cmd_reg)
				8'h9F: begin
        				state <= ST_ID_READ;
        				lanes <= 1;
        				dummy_cycles <= 0;
        				shift_out <= id_reg[23:16];
        				io_oe <= 4'b0010;
    				end
    				8'h05: begin
        				state <= ST_STATUS;
        				lanes <= 1;
        				shift_out <= status_reg;
        				io_oe <= 4'b0010;
    				end
    				8'h06: begin
        				status_reg[1] <= 1;
        				state <= ST_IDLE;
    				end
    				8'h04: begin
        				status_reg[1] <= 0;
        				state <= ST_IDLE;
    				end
    				8'h03, 8'h0B, 8'h3B, 8'h6B, 8'hEB: begin
        				state <= ST_ADDR;
        				addr_reg <= 0;
        				shift_in <= 0;
        				lanes <= (nxt_cmd_reg == 8'h03 || nxt_cmd_reg == 8'h0B) ? 1 : (nxt_cmd_reg == 8'h3B) ? 2 : 4;
        				dummy_cycles <= (nxt_cmd_reg == 8'h03) ? 0 : (nxt_cmd_reg == 8'hEB) ? 6 : 8;
    				end
    				8'h02, 8'h32: begin
        				dummy_cycles <= 0;
        				if (status_reg[1]) begin
            				    state <= ST_ADDR;
            				    lanes <= (nxt_cmd_reg == 8'h02) ? 1 : 4;
        				end else state <= ST_IDLE;
    				end
    				8'h20, 8'hD8: begin
					if (status_reg[1]) begin
					    state <= ST_ADDR;
					end else state <= ST_IDLE;
				end
				8'hC7: begin
        				if (status_reg[1]) begin
            				    integer j;
            				    for (j = 0; j < MEM_SIZE; j = j + 1) memory[j] <= 8'hFF;
            				    state <= ST_ERASE;
            				    wip <= 1;
        				end else state <= ST_IDLE;
    				end
    				default: state <= ST_IDLE;
			    endcase
			end
		end

                ST_ADDR: begin
                	if (lanes == 1) shift_in <= {shift_in[6:0], io_di[0]};
                	else if (lanes == 2) shift_in <= {shift_in[5:0], io_di[1:0]};
                	else if (lanes == 4) shift_in <= {shift_in[3:0], io_di[3:0]};
                	bit_cnt <= bit_cnt + lanes;
                	if ((bit_cnt == 7 && lanes == 1) || (bit_cnt == 6 && lanes == 2) || (bit_cnt == 4 && lanes == 4)) begin
                    		addr_reg <= {addr_reg[ADDR_BITS-9:0], lanes == 1 ? {shift_in[6:0], io_di[0]} : lanes == 2 ? {shift_in[5:0], io_di[1:0]} : {shift_in[3:0], io_di[3:0]}};
    				bit_cnt <= 0; // reset shifter
    				shift_in <= 0; // reset shifter
    				byte_cnt <= byte_cnt + 1; // Increment first to avoid error
    				if (byte_cnt == (ADDR_BITS/8) - 1) begin // if last byte of address then state transition
        				if (cmd_reg == 8'hEB) state <= ST_MODE;
        				else if (cmd_reg == 8'h20 | cmd_reg == 8'hD8) begin // need to take care of erase here since CS deassert next cycle
            					integer j;
            					if (cmd_reg == 8'h20) begin
                					for (j = nxt_addr_reg; j < nxt_addr_reg + SECTOR_SIZE; j = j + 1) memory[j] <= 8'hFF;
            					end else if (cmd_reg == 8'hD8) begin
                					for (j = nxt_addr_reg; j < nxt_addr_reg + 65536; j = j + 1) memory[j] <= 8'hFF;
            					end
            					wip <= 1;
            					state <= ST_ERASE;
        				end else if (dummy_cycles > 0) state <= ST_DUMMY;
            				else begin
                				state <= (cmd_reg == 8'h02 || cmd_reg == 8'h32 ? ST_DATA_WRITE : ST_DATA_READ);
                				bit_cnt <= 0;
                				byte_cnt <= 0;
                				if (cmd_reg == 8'h02 || cmd_reg == 8'h32) io_oe <= 4'b0000;
                				else if (lanes == 1) io_oe <= 4'b0010;
                				else if (lanes == 2) io_oe <= 4'b0011;
                				else io_oe <= 4'b1111;
                				shift_out <= memory[addr_reg];
            				end
        			end
    			end
		end

            	ST_MODE: begin
                	if (lanes == 1) shift_in <= {shift_in[6:0], io_di[0]};
                	else if (lanes == 2) shift_in <= {shift_in[5:0], io_di[1:0]};
                	else if (lanes == 4) shift_in <= {shift_in[3:0], io_di[3:0]};
                	bit_cnt <= bit_cnt + lanes;
                	if (bit_cnt == 8) begin
                    	    mode_bits <= shift_in[7:0];
                    	    continuous_read <= (shift_in == 8'hA0);
                    	    state <= ST_DUMMY;
			    bit_cnt <= 0;
                	end
            	end

            	ST_DUMMY: begin
                	bit_cnt <= bit_cnt + 1;
                	if (bit_cnt == (dummy_cycles - 1)) begin
                    	    state <= (cmd_reg == 8'h02 || cmd_reg == 8'h32 ? ST_DATA_WRITE : ST_DATA_READ);
                    	    bit_cnt <= 0;
                    	    byte_cnt <= 0;
                    	    if (cmd_reg == 8'h02 || cmd_reg == 8'h32) io_oe <= 4'b0000;
                    	    else if (lanes == 1) io_oe <= 4'b0010;
                    	    else if (lanes == 2) io_oe <= 4'b0011;
                    	    else io_oe <= 4'b1111;
                    	    shift_out <= memory[addr_reg];
                	end
            	end

            	ST_DATA_READ: begin
                	if (lanes == 1) io_do[1] <= shift_out[7];
                	else if (lanes == 2) begin
                    		io_do[0] <= shift_out[6];
                    		io_do[1] <= shift_out[7];
                	end else if (lanes == 4) begin
                    		io_do[0] <= shift_out[4];
                    		io_do[1] <= shift_out[5];
                    		io_do[2] <= shift_out[6];
                    		io_do[3] <= shift_out[7];
                	end
                	shift_out <= shift_out << lanes;
                	bit_cnt <= bit_cnt + lanes;
                	if ((bit_cnt == 7 && lanes == 1) || (bit_cnt == 6 && lanes == 2) || (bit_cnt == 4 && lanes == 4)) begin
                    	    addr_reg <= addr_reg + 1;
                    	    shift_out <= memory[addr_reg + 1];
                    	    bit_cnt <= 0;
                	end
                        if (!continuous_read && byte_cnt == MEM_SIZE - addr_reg)
                            state <= ST_IDLE;
                        byte_cnt <= byte_cnt + 1;
                end

            	ST_DATA_WRITE: begin
                	if (lanes == 1) shift_in <= {shift_in[6:0], io_di[0]};
                	else if (lanes == 2) shift_in <= {shift_in[5:0], io_di[1:0]};
                	else if (lanes == 4) shift_in <= {shift_in[3:0], io_di[3:0]};
                	bit_cnt <= bit_cnt + lanes;
                        if ((bit_cnt == 7 && lanes == 1) || (bit_cnt == 6 && lanes == 2) || (bit_cnt == 4 && lanes == 4)) begin
                            if (addr_reg <= MEM_SIZE && (addr_reg % PAGE_SIZE != PAGE_SIZE - 1)) begin
                                memory[addr_reg] <= (lanes == 1) ? {shift_in[6:0], io_di[0]} :
                                                   (lanes == 2) ? {shift_in[5:0], io_di[1:0]} :
                                                                  {shift_in[3:0], io_di[3:0]};
                                addr_reg <= addr_reg + 1;
                            end
                            bit_cnt <= 0;
                            byte_cnt <= byte_cnt + 1;
                            wip <= 1;
                        end
            	end

            	ST_ERASE: begin
                	// device won't stay full cycle here since CS# will be de-asserted
                	state <= ST_IDLE;
            	end

            	ST_STATUS: begin
                	io_do[1] <= shift_out[7];
                	shift_out <= shift_out << 1;
                	bit_cnt <= bit_cnt + 1;
                	if (bit_cnt == 8) state <= ST_IDLE;
            	end

            	ST_ID_READ: begin // do only manufacturing id
                	io_do[1] <= shift_out[7];
                	shift_out <= shift_out << 1;
                	bit_cnt <= bit_cnt + 1;
                	if (bit_cnt == 8) state <= ST_IDLE;
            	end

            	default: state <= ST_IDLE;
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