// KING compressed-picture transfer to the HuC6271 (Rainbow), registers
// 40-44, and the raster monitor (reg 44, RINT in reg 40) for king_irq.
//
// When the raster becomes reg 42 with REN set and no transfer running, a
// transfer of reg 43 blocks of 16 lines starts from reg 41 (bank A/B,
// A16:0; the address steps inside the bank). Regs 41-43 written during a
// transfer count from the next one. Clearing REN stops it at once.
//
// While -REQR is low KING reads ahead up to three words into a buffer and
// sends one word a dot, low byte first: strobes in ph 0-1 and 4-5. With the
// BG taking every display slot of the bank the transfer only gets blanking
// slots, and a byte a dot could not carry a strip there. When -REQR goes
// high the buffer keeps what was read ahead and sends it first next time
// (hence three 0000 words after each block). No word starts while the
// ADPCM transfer owns the bus.

module king_rb_xfer #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_osc,
	input  logic        ce_dck,
	input  logic  [3:0] ph,
	input  logic        hs_ev,
	input  logic        vs_ev,
	input  logic  [8:0] raster,

	/* verilator lint_off UNUSEDSIGNAL */
	input  huc6272_pkg::acc_t acc,
	/* verilator lint_on UNUSEDSIGNAL */
	input  logic        page,        // KRAM page for this block's transfers

	output huc6272_pkg::kreq_t req,
	output logic        req_bank,
	input  logic        gnt,
	input  logic [15:0] ka_q,
	input  logic [15:0] kb_q,

	input  logic        reqr_n,
	input  logic        kr_busy,
	output logic  [7:0] kr_d,
	output logic        kr_rb_wr_n,

	output logic        rint,
	output logic  [8:0] mon,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so,
	output logic        ss_busy
);

import huc6272_pkg::*;

rb_state_t st;
logic      cap, cap_ab;

wire wr_reg = acc.wr && !acc.ar;
wire dck    = ce_osc && ce_dck;
wire take   = dck && gnt;

// The transfer ends with the 16th line of its last block; a start on the
// same raster is allowed.
wire ends = st.act && hs_ev && st.lcnt == 4'd15 && st.bcnt == 5'd1;
wire go   = (hs_ev || vs_ev) && (!st.act || ends) && st.ctl[0]
         && raster == st.start && st.blocks != 5'd0;

wire send = st.act && !reqr_n && st.n != 2'd0 && !kr_busy;

assign req.v    = st.act && !reqr_n && {1'b0, st.n} + {1'b0, st.fly} < 3'd3;
assign req.we   = 1'b0;
assign req.page = page;
assign req.a    = st.a;
assign req.d    = 16'd0;
assign req_bank = st.ab;

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= rb_state_t'({st[RB_BITS-2:0], ss_si});
	end else if (ce_osc) begin
		if (wr_reg) begin
			case (acc.r)
				7'h41: if (acc.hi) st.sa[17:16] <= acc.d[1:0]; else st.sa[15:0] <= acc.d;
				7'h42: if (!acc.hi) st.start <= acc.d[8:0];
				7'h43: if (!acc.hi) st.blocks <= acc.d[4:0];
				7'h44: if (!acc.hi) st.mon <= acc.d[8:0];
				default: ;
			endcase
		end

		if (st.act && hs_ev) begin
			st.lcnt <= st.lcnt + 4'd1;
			if (st.lcnt == 4'd15)
				st.bcnt <= st.bcnt - 5'd1;
			if (ends)
				st.act <= 1'b0;
		end
		if (go) begin
			st.act  <= 1'b1;
			st.bcnt <= st.blocks;
			st.lcnt <= 4'd0;
			st.ab   <= st.sa[17];
			st.a    <= st.sa[16:0];
		end

		if (dck) begin
			st.fly <= take;
			if (take) begin
				st.fly_ab <= st.ab;
				st.a      <= st.a + 17'd1;
			end
			st.bv <= send;
			if (send) begin
				st.cur <= st.w[0];
				st.w   <= {16'd0, st.w[2:1]};
				st.n   <= st.n - 2'd1;
			end
		end

		if (cap) begin
			st.w[st.n] <= cap_ab ? kb_q : ka_q;
			st.n       <= st.n + 2'd1;
		end

		// Last, so clearing REN beats a start in the same clk.
		if (wr_reg && acc.r == 7'h40 && !acc.hi) begin
			st.ctl <= acc.d[1:0];
			if (!acc.d[0])
				st.act <= 1'b0;
		end
	end
end

assign ss_so = SAVESTATE ? st[RB_BITS-1] : ss_si;

// Read data is taken in the first clk of the dot after the read.
always_ff @(posedge clk) begin
	if (reset) begin
		cap <= 1'b0;
	end else if (ce_osc) begin
		cap    <= ce_dck && st.fly;
		cap_ab <= st.fly_ab;
	end
end

assign kr_d       = ph < 4'd4 ? st.cur[7:0] : st.cur[15:8];
assign kr_rb_wr_n = !(st.bv && (ph == 4'd0 || ph == 4'd1 || ph == 4'd4 || ph == 4'd5));
assign rint       = st.ctl[1];
assign mon        = st.mon;
assign ss_busy    = st.bv || st.fly || cap;

endmodule
