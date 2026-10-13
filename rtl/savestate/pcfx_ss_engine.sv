// Savestate engine: capture at a quiet point, then copy the whole machine
// to or from one DDR3 slot (layout in pcfx_ss_pkg).
//
//   save/load -> wait for the HuC6261 window (rasters 0-19, 262)
//             -> pause the V810 at an instruction boundary
//             -> KING and Rainbow between words -> stop the board (own)
//             -> chain, V810 state and memory, chip RAMs, SDRAM regions
//             -> header, then V810 rewrite (save) | page, CD ring (load)
//             -> let the board run, then the V810
//
// The board is stopped by the run/freeze control through the port arbiter:
// own asks, paused answers. The chain turns once, one bit per clock without
// a gap, through a 512 x 16 buffer: a save reads it out while feeding it
// back (the machine is unchanged), a load shifts the buffered slot in. A load first checks size,
// magic, layout hash, BIOS CRC and TOC hash, and leaves everything alone
// if any differs.
//
// A load restarts the V810 from its saved registers and cache (a transient
// reset, then the window writes), which drops its prefetch. A save ends
// with the same restart from the words it just wrote, so going on after a
// save and going on after a load are the same.

module pcfx_ss_engine
	import pcfx_ss_pkg::*;
(
	input  logic        clk,
	input  logic        reset,

	input  logic        save,          // pulses from the slot manager
	input  logic        load,
	input  logic [24:0] slot_addr,     // DDR3 64-bit word offset of the slot
	input  logic        count_save,    // bump the header counter (HPS writes the file)
	output logic        busy,
	output logic        load_bad,      // pulse: slot refused

	input  logic        ss_window,
	input  logic        ss_busy,
	input  logic        cd_ss_busy,

	output logic        cpu_pause_req,
	input  logic        cpu_pause_ready,
	output logic        cpu_restore,
	output logic  [3:0] cpu_state_addr,
	output logic [63:0] cpu_state_wdata,
	output logic        cpu_state_wren,
	input  logic [63:0] cpu_state_rdata,
	output logic        cpu_mem_active,
	output logic [10:0] cpu_mem_addr,
	output logic        cpu_mem_rden,
	output logic        cpu_mem_wren,
	output logic  [7:0] cpu_mem_wdata,
	input  logic  [7:0] cpu_mem_rdata,

	output logic        own,
	input  logic        gnt,
	input  logic        paused,

	output logic        ss_shift,
	output logic        ss_out,        // into the chain (a save feeds ss_in back)
	input  logic        ss_in,         // out of the chain

	output logic  [2:0] ssm_chip,
	output logic        ssm_sel,
	output logic [16:0] ssm_addr,
	output logic        ssm_we,
	output logic [15:0] ssm_wdata,
	input  logic [15:0] ssm_rdata,

	output logic        bk_req,
	output logic        bk_we,
	output logic [23:0] bk_addr,
	output logic [15:0] bk_d,
	output logic  [1:0] bk_be,
	input  logic        bk_ack,
	input  logic [15:0] bk_q,

	input  logic [10:0] pg_q,
	output logic        pg_we,
	output logic [10:0] pg_d,

	input  logic [31:0] bios_crc,
	input  logic [31:0] toc_hash,

	output logic        dd_req,
	output logic        dd_we,
	output logic [24:0] dd_addr,
	output logic [63:0] dd_wdata,
	output logic  [7:0] dd_be,
	input  logic        dd_ack,
	input  logic [63:0] dd_rdata
);

typedef enum logic [4:0] {
	IDLE, WIN, PAUSING, STOP,
	TURN, L_PRE, L_PRE2, L_PRE3,
	E_START, E_ACCESS, E_DONE, DD_WAIT, S_HDR, S_HDR0,
	L_HDR, L_CHECK, L_FETCH, L_PAGE, L_RING,
	S_REWRITE, RELEASE, RESUME
} state_t;

state_t      st, after_dd;
logic        saving;
seg_t        seg;
logic [19:0] e;           // element in the segment
logic [19:0] last_e;
logic [21:0] k;           // element in the stream
logic [63:0] w;           // packing word
logic [15:0] elem;
logic  [4:0] cnt;
logic  [1:0] hw;          // header word being read or written
logic [31:0] h0;            // slot size
logic [63:0] h1, h2, h3;
logic [31:0] counter;
logic        acked;
logic        rewrite;     // a save putting the V810 back from its slot words
logic        need_w;      // the next element's DDR word is not in w yet

wire last_elem = seg == SG_LAST && e == last_e;

// Chain buffer: element n of the chain at address n (up to 8192 bits).
localparam logic [12:0] CHAIN_LAST = 13'(CHAIN_BITS - 1);

logic  [8:0] cb_addr;
logic        cb_we;
logic [15:0] cb_wdata, cb_q;
logic [12:0] bit_n;       // chain bit being turned
logic [15:0] cur;         // word being shifted in

cache_ram #(
	.ADDR_WIDTH (9),
	.DATA_WIDTH (16)
) u_chain (
	.clk_i   (clk),
	.addr_i  (cb_addr),
	.wren_i  (cb_we),
	.wdata_i (cb_wdata),
	.q_o     (cb_q)
);

// Where an element lives.
logic [16:0] ssm_a;
logic  [2:0] ssm_c;
logic [23:0] bulk_a;

always_comb begin
	ssm_c = 3'd0;
	case (seg)
		SG_VA:   ssm_c = 3'd1;
		SG_VB:   ssm_c = 3'd2;
		SG_BRAM: ssm_c = 3'd3;
		SG_PSG:  ssm_c = 3'd4;
		SG_RRAM: ssm_c = 3'd5;
		default: ;
	endcase
	if ((seg == SG_VA || seg == SG_VB) && e[16])
		ssm_a = {1'b1, 8'd0, e[7:0]};       // SATB after the 64K VRAM words
	else if (seg == SG_RRAM && e[14])
		ssm_a = {2'b01, 8'd0, e[6:0]};      // Q tables after the two R-RAMs
	else
		ssm_a = e[16:0];

	case (seg)
		SG_MAIN_LO: bulk_a = {2'd2, 3'd0, e[18:9], e[8:0]};
		SG_MAIN_HI: bulk_a = {2'd3, 3'd0, e[18:9], e[8:0]};
		SG_KRAM_A:  bulk_a = {2'd0, 4'd0, e[17:9], e[8:0]};
		SG_KRAM_B:  bulk_a = {2'd1, 4'd0, e[17:9], e[8:0]};
		default:    bulk_a = {2'd1, 13'd1536 + {6'd0, e[15:9]}, e[8:0]};
	endcase
end

wire is_ssm = seg >= SG_NT && seg <= SG_RRAM;

// ss_in is the bit that leaves at the end of the clock.
assign ss_shift = st == TURN;
assign ss_out   = saving ? ss_in : cur[bit_n[3:0]];

// The packing word with this element in place.
logic [63:0] w_in;
always_comb begin
	w_in = w;
	w_in[16 * k[1:0] +: 16] = elem;
end

assign busy = st != IDLE;

always_ff @(posedge clk) begin
	cpu_restore    <= 1'b0;
	cb_we          <= 1'b0;
	cpu_state_wren <= 1'b0;
	cpu_mem_rden   <= 1'b0;
	cpu_mem_wren   <= 1'b0;
	ssm_we         <= 1'b0;
	pg_we          <= 1'b0;
	load_bad       <= 1'b0;

	if (reset) begin
		st            <= IDLE;
		own           <= 1'b0;
		cpu_pause_req <= 1'b0;
		dd_req        <= 1'b0;
		bk_req        <= 1'b0;
		ssm_sel       <= 1'b0;
		cpu_mem_active <= 1'b0;
		counter       <= 32'd0;
		rewrite       <= 1'b0;
		need_w        <= 1'b0;
	end else begin
		case (st)
			IDLE: begin
				if (save || load) begin
					saving <= save;
					st     <= WIN;
				end
			end

			// Wait for the window, pause the V810, then stop the board while
			// the window lasts and KING is between words.
			WIN: begin
				if (ss_window) begin
					cpu_pause_req <= 1'b1;
					st            <= PAUSING;
				end
			end

			PAUSING: begin
				if (!ss_window) begin
					cpu_pause_req <= 1'b0;
					st            <= WIN;
				end else if (cpu_pause_ready && !ss_busy) begin
					own <= 1'b1;
					st  <= STOP;
				end
			end

			STOP: begin
				if (gnt && paused) begin
					seg <= SG_CHAIN;
					e   <= 20'd0;
					k   <= 22'd0;
					hw  <= 2'd0;
					bit_n <= 13'd0;
					if (saving) begin
						h3 <= {53'd0, pg_q};
						st <= TURN;
					end else begin
						st <= L_HDR;
					end
				end
			end

			// ---------------------------------------------------------- chain turn

			// A save keeps each 16 bits leaving the chain; a load shifts the
			// buffer in, reading one word ahead.
			TURN: begin
				bit_n <= bit_n + 13'd1;
				if (saving) begin
					elem[bit_n[3:0]] <= ss_in;
					if (bit_n[3:0] == 4'hF || bit_n == CHAIN_LAST) begin
						cb_we    <= 1'b1;
						cb_addr  <= bit_n[12:4];
						cb_wdata <= elem;
						cb_wdata[bit_n[3:0]] <= ss_in;
					end
				end else begin
					if (bit_n[3:0] == 4'h0)
						cb_addr <= bit_n[12:4] + 9'd1;
					if (bit_n[3:0] == 4'hF)
						cur <= cb_q;
				end
				if (bit_n == CHAIN_LAST)
					st <= E_START;
			end

			L_PRE: begin
				cb_addr <= 9'd0;
				st      <= L_PRE2;
			end

			L_PRE2: st <= L_PRE3;

			L_PRE3: begin
				cur   <= cb_q;
				bit_n <= 13'd0;
				st    <= TURN;
			end

			// ---------------------------------------------------------- elements

			E_START: begin
				last_e <= seg_len(seg) - 20'd1;
				cnt    <= 5'd0;
				acked  <= 1'b0;
				need_w <= 1'b0;
				if (!saving && (k[1:0] == 2'd0 || need_w)) begin
					dd_req   <= 1'b1;
					dd_we    <= 1'b0;
					dd_addr  <= slot_addr + 25'(HDR_WORDS) + 25'(k[21:2]);
					after_dd <= L_FETCH;
					st       <= DD_WAIT;
				end else begin
					elem <= saving ? 16'd0 : w[16 * k[1:0] +: 16];
					st   <= E_ACCESS;
				end
			end

			L_FETCH: begin
				elem <= w[16 * k[1:0] +: 16];
				st   <= E_ACCESS;
			end

			E_ACCESS: begin
				cnt <= cnt + 5'd1;
				case (seg)
					SG_CHAIN: begin
						cb_addr <= e[8:0];
						if (saving) begin
							if (cnt == 5'd2) begin
								elem <= cb_q;
								st   <= E_DONE;
							end
						end else begin
							cb_we    <= cnt == 5'd0;
							cb_wdata <= elem;
							st       <= E_DONE;
						end
					end

					SG_CPU_ST: begin
						cpu_state_addr <= e[5:2];
						if (saving) begin
							if (cnt == 5'd1) begin
								elem <= cpu_state_rdata[16 * e[1:0] +: 16];
								st   <= E_DONE;
							end
						end else begin
							cpu_state_wdata[16 * e[1:0] +: 16] <= elem;
							if (e[1:0] == 2'd3 && cnt == 5'd0)
								cpu_state_wren <= 1'b1;
							st <= E_DONE;
						end
					end

					SG_CPU_MEM: begin
						cpu_mem_active <= 1'b1;
						cpu_mem_addr   <= e[10:0];
						cpu_mem_wdata  <= elem[7:0];
						// The CPU drives read data only while rden is high.
						if (cnt != 5'd0)
							cpu_mem_rden <= saving;
						if (cnt == 5'd1)
							cpu_mem_wren <= !saving;
						if (cnt == 5'd9) begin
							if (saving)
								elem <= {8'd0, cpu_mem_rdata};
							st <= E_DONE;
						end
					end

					default: begin
						if (is_ssm) begin
							ssm_sel   <= 1'b1;
							ssm_chip  <= ssm_c;
							ssm_addr  <= ssm_a;
							ssm_wdata <= elem;
							if (cnt == 5'd1 && !saving)
								ssm_we <= 1'b1;
							if (cnt == 5'd3) begin
								if (saving)
									elem <= ssm_rdata;
								st <= E_DONE;
							end
						end else begin
							// bulk: four-phase handshake
							bk_addr <= bulk_a;
							bk_we   <= !saving;
							bk_d    <= elem;
							bk_be   <= 2'b11;
							if (!acked) begin
								bk_req <= 1'b1;
								if (bk_req && bk_ack) begin
									bk_req <= 1'b0;
									acked  <= 1'b1;
									if (saving)
										elem <= bk_q;
								end
							end else if (!bk_ack) begin
								st <= E_DONE;
							end
						end
					end
				endcase
			end

			E_DONE: begin
				ssm_sel <= 1'b0;
				if (saving)
					w <= w_in;
				k <= k + 22'd1;
				if (last_elem || e == last_e) begin
					seg <= seg_t'(seg + 4'd1);
					e   <= 20'd0;
					if (seg == SG_CPU_MEM)
						cpu_mem_active <= 1'b0;
				end else begin
					e <= e + 20'd1;
				end
				if (rewrite && seg == SG_CPU_MEM && e == last_e) begin
					st <= RELEASE;
				end else if (saving && (k[1:0] == 2'd3 || last_elem)) begin
					dd_req   <= 1'b1;
					dd_we    <= 1'b1;
					dd_be    <= 8'hFF;
					dd_addr  <= slot_addr + 25'(HDR_WORDS) + 25'(k[21:2]);
					dd_wdata <= w_in;
					after_dd <= last_elem ? S_HDR : E_START;
					st       <= DD_WAIT;
				end else if (!saving && seg == SG_CHAIN && e == last_e) begin
					st <= L_PRE;      // a load turns the chain once it is buffered
				end else begin
					st <= last_elem ? (saving ? S_HDR : L_PAGE) : E_START;
				end
			end

			DD_WAIT: begin
				if (dd_ack) begin
					dd_req <= 1'b0;
					if (!dd_we)
						w <= dd_rdata;
					st <= after_dd;
				end
			end

			// ---------------------------------------------------------- save header

			S_HDR: begin
				hw       <= hw + 2'd1;
				dd_req   <= 1'b1;
				dd_we    <= 1'b1;
				dd_be    <= 8'hFF;
				dd_addr  <= slot_addr + 25'(hw) + 25'd1;
				case (hw)
					2'd0:    dd_wdata <= {MAGIC, LAYOUT_HASH};
					2'd1:    dd_wdata <= {bios_crc, toc_hash};
					default: dd_wdata <= h3;
				endcase
				after_dd <= hw == 2'd2 ? S_HDR0 : S_HDR;
				st       <= DD_WAIT;
			end

			S_HDR0: begin
				counter  <= counter + 32'd1;
				dd_req   <= 1'b1;
				dd_we    <= 1'b1;
				dd_be    <= count_save ? 8'hFF : 8'hF0;
				dd_addr  <= slot_addr;
				dd_wdata <= {SIZE_DWORDS, counter + 32'd1};
				after_dd <= S_REWRITE;
				st       <= DD_WAIT;
			end

			S_REWRITE: begin
				cpu_restore <= 1'b1;
				saving      <= 1'b0;
				rewrite     <= 1'b1;
				need_w      <= 1'b1;
				seg         <= SG_CPU_ST;
				e           <= 20'd0;
				k           <= 22'(seg_len(SG_CHAIN));
				st          <= E_START;
			end

			// ---------------------------------------------------------- load header

			L_HDR: begin
				dd_req   <= 1'b1;
				dd_we    <= 1'b0;
				dd_addr  <= slot_addr + 25'(hw);
				after_dd <= L_CHECK;
				st       <= DD_WAIT;
			end

			L_CHECK: begin
				case (hw)
					2'd0: h0 <= w[63:32];
					2'd1: h1 <= w;
					2'd2: h2 <= w;
					default: h3 <= w;
				endcase
				hw <= hw + 2'd1;
				if (hw != 2'd3) begin
					st <= L_HDR;
				end else if (h0 == SIZE_DWORDS && h1 == {MAGIC, LAYOUT_HASH}
				             && h2 == {bios_crc, toc_hash}) begin
					cpu_restore <= 1'b1;
					st          <= E_START;
				end else begin
					load_bad <= 1'b1;
					st       <= RELEASE;
				end
			end

			L_PAGE: begin
				pg_we <= 1'b1;
				pg_d  <= h3[10:0];
				st    <= L_RING;
			end

			// The CD drive refills its ring while the board is still stopped.
			L_RING: begin
				if (!cd_ss_busy)
					st <= RELEASE;
			end

			// ---------------------------------------------------------- resume

			RELEASE: begin
				own <= 1'b0;
				if (!gnt && !paused)
					st <= RESUME;
			end

			RESUME: begin
				cpu_pause_req <= 1'b0;
				rewrite       <= 1'b0;
				st            <= IDLE;
			end

			default: st <= IDLE;
		endcase
	end
end

endmodule
