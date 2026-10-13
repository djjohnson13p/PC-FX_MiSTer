// HuC6230 K-R bus receiver: ADPCM bytes from KING. -CS1/-CS0 = 01 picks
// ADPCM#1, 10 picks #2; RH/-L high marks the low byte, which comes first.
// A byte is taken on the rising edge of the write strobe. At each transfer
// cycle boundary a channel has data only if a low byte and then a high byte
// arrived since the last boundary; the last such pair wins. A byte on the
// boundary clock itself counts for the next cycle.

module huc6230_kr_rx #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_master,

	input  logic  [7:0] kr_d,
	input  logic        kr_cs0_n,
	input  logic        kr_cs1_n,
	input  logic        kr_rh_l,
	input  logic        kr_wr_n,

	input  logic        boundary,
	input  logic        unlock,     // hard or both-channel soft reset
	output logic        seen,
	output logic [1:0]  pair_ok,
	output logic [1:0][15:0] pair,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

import huc6230_pkg::*;

kr_state_t st;

wire [1:0] sel    = {kr_cs1_n && !kr_cs0_n, !kr_cs1_n && kr_cs0_n};
wire       commit = ce_master && kr_wr_n && !st.wr_q;

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
		st.wr_q <= 1'b1;
	end else if (SAVESTATE && ss_shift) begin
		st <= kr_state_t'({st[SS_KR_BITS-2:0], ss_si});
	end else begin
		if (ce_master) begin
			st.wr_q <= kr_wr_n;
			if (!kr_wr_n) begin
				st.h_ch <= sel;
				st.h_rh <= kr_rh_l;
				st.h_d  <= kr_d;
			end
		end

		if (boundary) begin
			st.got_lo <= 2'b00;
			st.got_hi <= 2'b00;
		end
		if (unlock)
			st.seen <= 1'b0;

		if (commit) begin
			for (int c = 0; c < 2; c++) begin
				if (st.h_ch[c]) begin
					st.seen <= !unlock;
					if (st.h_rh) begin
						st.pair[c][7:0] <= st.h_d;
						st.got_lo[c]    <= 1'b1;
						st.got_hi[c]    <= 1'b0;
					end else begin
						st.pair[c][15:8] <= st.h_d;
						st.got_hi[c]     <= st.got_lo[c] && !boundary;
					end
				end
			end
		end
	end
end

assign ss_so   = SAVESTATE ? st[SS_KR_BITS-1] : ss_si;
assign seen    = st.seen;
assign pair_ok = st.got_hi;
assign pair    = st.pair;

endmodule
