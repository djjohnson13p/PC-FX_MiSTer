// Backup files: the internal 32 KB backup SRAM on framework slot 1 and the
// 128 KB FX-BMP card on slot 2, both moved through one 512-byte block
// buffer.
//
//   mount (size > 0)  read every block of the file and copy it in; a
//                     missing file reads as zeros
//   OSD opens         each memory the CPU wrote since the last save is
//                     written back whole, from block 0 up
//
// Housekeeping never stops a running game. A mount before the board runs
// (core load) holds it in reset until the copy is done; the SRAM goes in
// through its socket's second port, the card through pcfx_memory's bulk
// port (byte 2w low, 2w + 1 high of card word w). A card mounted while the
// game runs is pulled out (card_in low), written through pcfx_memory's
// background port and put back. Saves read the SRAM through the socket's
// second port and the card through background reads. A block the CPU writes
// during its copy may be saved half old, half new; that write marks the
// memory dirty for the next save. If a slot's image changes mid-transfer,
// the transfer stops at the end of the block in flight.

module save_slot
(
	input  logic        clk,
	input  logic        reset,

	input  logic  [1:0] img_mounted,    // slot 2, slot 1
	input  logic        img_readonly,
	input  logic [63:0] img_size,
	output logic [31:0] sd_lba,
	output logic  [1:0] sd_rd,
	output logic  [1:0] sd_wr,
	input  logic  [1:0] sd_ack,
	input  logic  [8:0] sd_buff_addr,
	input  logic  [7:0] sd_buff_dout,
	output logic  [7:0] sd_buff_din,
	input  logic        sd_buff_wr,

	input  logic        osd_status,
	input  logic        bram_we,        // CPU write strobes
	input  logic        fx_we,
	input  logic        running,        // the board is out of its boot hold

	output logic        loading,        // hold the board in reset
	output logic        card_in,        // the card is in its slot
	input  logic        paused,         // board held and memory frozen
	output logic        own_req,        // asks for the socket and bulk ports
	input  logic        gnt,

	output logic        mem_sel,        // backup SRAM socket, second port
	output logic [14:0] mem_addr,
	output logic        mem_we,
	output logic  [7:0] mem_wdata,
	input  logic  [7:0] mem_rdata,

	output logic        bk_req,         // pcfx_memory bulk port
	output logic        bk_we,
	output logic [23:0] bk_addr,
	output logic [15:0] bk_d,
	output logic  [1:0] bk_be,
	input  logic        bk_ack,

	output logic        bg_req,         // pcfx_memory background card port
	output logic        bg_we,
	output logic [15:0] bg_addr,
	output logic [15:0] bg_d,
	input  logic        bg_ack,
	input  logic [15:0] bg_q
);

localparam logic [2:0] S_IDLE  = 3'd0;
localparam logic [2:0] S_REQ   = 3'd1;  // block request up, waiting for Linux
localparam logic [2:0] S_XFER  = 3'd2;  // Linux moving the block
localparam logic [2:0] S_GRANT = 3'd3;  // waiting for the ports
localparam logic [2:0] S_COPY  = 3'd4;
localparam logic [2:0] S_NEXT  = 3'd5;

logic [2:0] state;
logic [1:0] valid, ro, dirty, need_load, held, save_pend;
logic       slot;                       // 0 SRAM, 1 FX-BMP
logic       save;
logic       restart;                    // this slot's image changed
logic       ejected;                    // no card file mounted
logic [7:0] blk;
logic [8:0] idx;                        // byte in the block
logic [2:0] ph;
logic [7:0] lo;
logic       req;
logic       osd_q;

wire [7:0] last_blk = slot ? 8'd255 : 8'd63;

// One block buffer: Linux on port A, the copy on port B.
wire  [8:0] buf_addr;
wire        buf_we;
wire  [7:0] buf_wdata;
logic [7:0] buf_q;

cache_ram_dp #(.ADDR_WIDTH(9), .DATA_WIDTH(8)) u_buf (
	.clk_i     (clk),
	.addr_a_i  (sd_buff_addr),
	.wren_a_i  (sd_buff_wr && sd_ack != 2'b00),
	.wdata_a_i (sd_buff_dout),
	.q_a_o     (sd_buff_din),
	.addr_b_i  (buf_addr),
	.wren_b_i  (buf_we),
	.wdata_b_i (buf_wdata),
	.q_b_o     (buf_q)
);

// SRAM copy, one byte: ph 0 address, ph 1 data.
// Card load, one word: ph 0-1 read its bytes, 2 raise the request, 3 wait
// for ack, 4 wait for ack to drop. Card save: ph 0 request, 1 ack, 2 ack
// drop, 3-4 write its bytes. Held loads use the bulk port, the rest the
// background port.
wire copy   = state == S_COPY;
wire c_sram = copy && !slot;
wire c_card = copy && slot;
wire hi     = save ? ph == 3'd4 : ph == 3'd1;
wire use_bg = save || !held[1];
wire ack    = use_bg ? bg_ack : bk_ack;

assign mem_sel   = c_sram;
assign mem_addr  = {blk[5:0], idx};
assign mem_we    = c_sram && !save && ph[0];
assign mem_wdata = buf_q;

assign buf_addr  = c_card ? {idx[8:1], hi} : idx;
assign buf_we    = save && (c_sram ? ph[0] : (c_card && (ph == 3'd3 || ph == 3'd4)));
assign buf_wdata = c_sram ? mem_rdata : (hi ? bk_d[15:8] : bk_d[7:0]);

assign bk_req  = req && !use_bg;
assign bk_we   = 1'b1;
assign bk_be   = 2'b11;
assign bk_addr = {2'b01, 13'd1536 + {6'd0, blk[7:1]}, blk[0], idx[8:1]};
assign bg_req  = req && use_bg;
assign bg_we   = !save;
assign bg_addr = {blk, idx[8:1]};
assign bg_d    = bk_d;

assign sd_lba  = {24'd0, blk};
assign loading = (need_load & held) != 2'b00;
assign card_in = !need_load[1] && !ejected;
// Background card copies need no shared port.
assign own_req = (state == S_GRANT || copy) && !(slot && use_bg);

wire [1:0] slot_bit = slot ? 2'b10 : 2'b01;
wire       step_end = c_sram ? ph[0] : (save ? ph == 3'd4 : (ph == 3'd4 && !ack));
wire       blk_end  = c_sram ? idx == 9'd511 : idx == 9'd510;
wire       go       = (slot && use_bg) || (gnt && (save || paused));

always_ff @(posedge clk) begin
	if (reset) begin
		state     <= S_IDLE;
		valid     <= 2'b00;
		ro        <= 2'b00;
		dirty     <= 2'b00;
		need_load <= 2'b00;
		held      <= 2'b00;
		save_pend <= 2'b00;
		sd_rd     <= 2'b00;
		sd_wr     <= 2'b00;
		req       <= 1'b0;
		restart   <= 1'b0;
		ejected   <= 1'b1;
		osd_q     <= 1'b0;
		slot      <= 1'b0;
		save      <= 1'b0;
		blk       <= 8'd0;
		idx       <= 9'd0;
		ph        <= 3'd0;
	end else begin
		osd_q <= osd_status;
		if (osd_status && !osd_q)
			save_pend <= save_pend | (dirty & valid & ~ro);

		case (state)
		S_IDLE: begin
			blk     <= 8'd0;
			restart <= 1'b0;
			if (need_load != 2'b00) begin
				slot  <= !need_load[0];
				save  <= 1'b0;
				state <= S_REQ;
			end else if (save_pend != 2'b00) begin
				slot  <= !save_pend[0];
				save  <= 1'b1;
				state <= S_GRANT;
				save_pend[save_pend[0] ? 0 : 1] <= 1'b0;
				dirty[save_pend[0] ? 0 : 1]     <= 1'b0;
			end
		end

		S_REQ: begin
			sd_rd <= save ? 2'b00 : slot_bit;
			sd_wr <= save ? slot_bit : 2'b00;
			if (sd_ack[slot]) begin
				sd_rd <= 2'b00;
				sd_wr <= 2'b00;
				state <= S_XFER;
			end else if (restart) begin
				sd_rd <= 2'b00;
				sd_wr <= 2'b00;
				state <= S_IDLE;
			end
		end

		S_XFER:
			if (!sd_ack[slot])
				state <= restart ? S_IDLE : save ? S_NEXT : S_GRANT;

		S_GRANT: begin
			idx <= 9'd0;
			ph  <= 3'd0;
			if (restart)
				state <= S_IDLE;
			else if (go)
				state <= S_COPY;
		end

		S_COPY: begin
			case (ph)
			3'd0: begin
				ph <= 3'd1;
				if (c_card && save)
					req <= 1'b1;
			end
			3'd1:
				if (!c_card) begin
					ph <= 3'd0;
				end else if (!save) begin
					lo <= buf_q;
					ph <= 3'd2;
				end else if (ack) begin
					bk_d <= bg_q;
					req  <= 1'b0;
					ph   <= 3'd2;
				end
			3'd2:
				if (!save) begin
					bk_d <= {buf_q, lo};
					req  <= 1'b1;
					ph   <= 3'd3;
				end else if (!ack) begin
					ph <= 3'd3;
				end
			3'd3:
				if (save) begin
					ph <= 3'd4;
				end else if (ack) begin
					req <= 1'b0;
					ph  <= 3'd4;
				end
			default:
				if (step_end)
					ph <= 3'd0;
			endcase

			if (step_end) begin
				idx <= idx + (slot ? 9'd2 : 9'd1);
				if (blk_end)
					state <= save ? S_REQ : S_NEXT;
			end
		end

		S_NEXT: begin
			blk   <= blk + 8'd1;
			state <= save ? S_GRANT : S_REQ;
			if (restart) begin
				state <= S_IDLE;
			end else if (blk == last_blk) begin
				state <= S_IDLE;
				if (!save) begin
					need_load[slot] <= 1'b0;
					dirty[slot]     <= 1'b0;
				end
			end
		end

		default: state <= S_IDLE;
		endcase

		if (bram_we) dirty[0] <= 1'b1;
		if (fx_we)   dirty[1] <= 1'b1;

		// A new image replaces whatever was pending for its slot. The SRAM
		// always loads under the hold; the card only before the board runs.
		for (int s = 0; s < 2; s++) begin
			if (img_mounted[s]) begin
				valid[s]     <= img_size != 64'd0;
				ro[s]        <= img_readonly;
				need_load[s] <= img_size != 64'd0;
				held[s]      <= s == 0 || !running;
				save_pend[s] <= 1'b0;
				if (state != S_IDLE && slot == s[0])
					restart <= 1'b1;
			end
		end
		if (img_mounted[1])
			ejected <= img_size == 64'd0;
	end
end

endmodule
