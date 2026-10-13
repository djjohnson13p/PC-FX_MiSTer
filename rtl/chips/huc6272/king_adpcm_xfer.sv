// KING ADPCM transfer to the HuC6230 over the K-R bus, registers 50-5E.
//
// READ EN 0 -> 1 takes effect at -HSYNC and loads the channel's pointer from
// its start register; READ EN -> 0 acts at once. Starting from no channel
// running restarts the transfer cycle, so the first word goes out in the
// next line's blanking and the HuC6230 plays from the -HSYNC after it. One
// word per running channel every 2, 4, 8 or 16 lines (DIV):
//
//   kx  280      285      286      287  288  289  290       309
//       request  read #1  read #2  #1L  #1H  #2L  #2H       -HSYNC
//       (refresh first)            K-R byte per dot, strobe in ph 0-3
//
// The pointer steps inside its bank (A16:0 wraps). Reading the end word
// sets END: a ring reloads the start, a sequential buffer stops and clears
// READ EN. Reading the half word sets HALF. Each sets its reg 53 bit, and
// the SOUND interrupt when enabled in reg 51/52; reading 53 clears it.

module king_adpcm_xfer #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_osc,
	input  logic        ce_dck,
	input  logic  [8:0] nkx,
	input  logic  [3:0] ph,
	input  logic        hs_ev,

	input  huc6272_pkg::acc_t acc,
	input  logic        page,        // KRAM page for this block's transfers
	output logic [15:0] rdata,

	output huc6272_pkg::kreq_t req,  // one bank, in blanking
	output logic        req_bank,
	input  logic        gnt,
	input  logic [15:0] ka_q,
	input  logic [15:0] kb_q,

	output logic  [7:0] kr_d,
	output logic        kr_cs0_n,
	output logic        kr_cs1_n,
	output logic        kr_rh_l,
	output logic        kr_wr_n,
	output logic        kr_busy,     // keeps Rainbow strobes off the bus

	output logic        set_sound,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

import huc6272_pkg::*;

adpcm_state_t st;
logic cap, cap_ch, cap_bank;

wire wr_reg = acc.wr && !acc.ar;
wire rd_53  = acc.rd && !acc.ar && acc.r == 7'h53;
wire dck    = ce_osc && ce_dck;

// 2^(DIV+1) - 1 lines between transfer lines
wire [3:0] last = {st.ctl[3:2] == 2'd3, st.ctl[3:2] >= 2'd2, st.ctl[3:2] >= 2'd1, 1'b1};

// The word read when the arbiter grants this dot.
wire        ch   = !st.pend[0];
wire [17:0] p    = st.ptr[ch];
wire        hit_end  = p == st.ea[ch];
wire        hit_half = p == {st.ha[ch], 6'd0};
wire        take = dck && gnt;

assign req.v    = st.pend[0] || st.pend[1];
assign req.we   = 1'b0;
assign req.page = page;
assign req.a    = p[16:0];
assign req.d    = 16'd0;
assign req_bank = p[17];

assign set_sound = take && ((hit_end && st.buf_m[ch][1]) || (hit_half && st.buf_m[ch][2]));

// Next byte, chosen as each dot starts: high after low, then the next word.
logic [1:0] have_n;
logic       bv_n;
logic [1:0] bsel_n;

always_comb begin
	have_n = st.have;
	if (st.bv && st.bsel[0])
		have_n[st.bsel[1]] = 1'b0;
	bv_n   = 1'b1;
	if (st.bv && !st.bsel[0])
		bsel_n = {st.bsel[1], 1'b1};
	else if (have_n[0])
		bsel_n = 2'b00;
	else if (have_n[1])
		bsel_n = 2'b10;
	else begin
		bsel_n = 2'b00;
		bv_n   = 1'b0;
	end
end

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= adpcm_state_t'({st[ADPCM_BITS-2:0], ss_si});
	end else if (ce_osc) begin
		if (wr_reg) begin
			case (acc.r)
				7'h51: if (!acc.hi) st.buf_m[0] <= acc.d[2:0];
				7'h52: if (!acc.hi) st.buf_m[1] <= acc.d[2:0];
				7'h58: if (!acc.hi) st.sa[0] <= acc.d[9:0];
				7'h5C: if (!acc.hi) st.sa[1] <= acc.d[9:0];
				7'h59: if (acc.hi) st.ea[0][17:16] <= acc.d[1:0]; else st.ea[0][15:0] <= acc.d;
				7'h5D: if (acc.hi) st.ea[1][17:16] <= acc.d[1:0]; else st.ea[1][15:0] <= acc.d;
				7'h5A: if (!acc.hi) st.ha[0] <= acc.d[11:0];
				7'h5E: if (!acc.hi) st.ha[1] <= acc.d[11:0];
				default: ;
			endcase
		end

		// READ EN 0 -> 1 lands on -HSYNC; the cycle restarts from idle.
		if (hs_ev) begin
			st.lc <= (st.lc == last || st.run == 2'b00) ? 4'd0 : st.lc + 4'd1;
			for (int c = 0; c < 2; c++)
				if (st.ctl[c] && !st.run[c]) begin
					st.run[c] <= 1'b1;
					st.ptr[c] <= {st.sa[c], 8'd0};
				end
		end

		if (dck && nkx == KX_REF && st.lc == 4'd0)
			st.pend <= st.run;

		if (dck) begin
			st.fly <= take;
			if (take) begin
				st.fly_ch   <= ch;
				st.fly_bank <= p[17];
				st.pend[ch] <= 1'b0;
				if (hit_end) begin
					if (st.buf_m[ch][0]) begin
						st.ptr[ch] <= {st.sa[ch], 8'd0};
					end else begin
						st.run[ch]  <= 1'b0;
						st.ctl[{1'b0, ch}] <= 1'b0;
					end
				end else begin
					st.ptr[ch] <= {p[17], p[16:0] + 17'd1};
				end
			end
			st.bv   <= bv_n;
			st.bsel <= bsel_n;
			st.have <= have_n;
		end

		if (cap) begin
			st.word[cap_ch] <= cap_bank ? kb_q : ka_q;
			st.have[cap_ch] <= 1'b1;
		end

		if (rd_53)
			st.stat <= 4'd0;
		if (take && hit_end)
			st.stat[{ch, 1'b0}] <= 1'b1;
		if (take && hit_half)
			st.stat[{ch, 1'b1}] <= 1'b1;

		// Last, so clearing READ EN beats a start or request in the same clk.
		if (wr_reg && acc.r == 7'h50 && !acc.hi) begin
			st.ctl <= acc.d[3:0];
			for (int c = 0; c < 2; c++)
				if (!acc.d[c]) begin
					st.run[c]  <= 1'b0;
					st.pend[c] <= 1'b0;
				end
		end
	end
end

assign ss_so = SAVESTATE ? st[ADPCM_BITS-1] : ss_si;

// Read data is taken in the first clk of the dot after the read.
always_ff @(posedge clk) begin
	if (reset) begin
		cap <= 1'b0;
	end else if (ce_osc) begin
		cap      <= ce_dck && st.fly;
		cap_ch   <= st.fly_ch;
		cap_bank <= st.fly_bank;
	end
end

assign rdata = (acc.r == 7'h53 && !acc.hi) ? {12'd0, st.stat} : 16'd0;

wire [15:0] bw = st.word[st.bsel[1]];

assign kr_d     = st.bv ? (st.bsel[0] ? bw[15:8] : bw[7:0]) : 8'hFF;
assign kr_cs1_n = !(st.bv && !st.bsel[1]);
assign kr_cs0_n = !(st.bv && st.bsel[1]);
assign kr_rh_l  = !(st.bv && st.bsel[0]);
assign kr_wr_n  = !(st.bv && ph < 4'd4);
assign kr_busy  = st.pend != 2'b00 || st.fly || cap || st.have != 2'b00 || st.bv;

endmodule
