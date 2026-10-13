// HuC6230 LINE VCAs (R15, R16) for CD-DA. Each side has a 10-bit up/down
// level counter that walks toward the register: up to 16n + 15 (0 for
// n = 0), down to 16n (1023 for n = 3Fh). The step clock is a tap of one
// free-running master clock divider chosen by the counter's own top bit,
// 32 master clocks (1.49 us) per step at 512 and above, doubling per
// octave below; counter bits 9-1 pick the tap, so levels 0 and 1 use
// 16384 clocks. The CD-DA sample is held from its 44.1 kHz enable.

module huc6230_vca #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic       clk,
	input  logic       reset,
	input  logic       ce_master,

	input  logic [1:0][5:0] lvol,
	input  logic [15:0] cdda_l,
	input  logic [15:0] cdda_r,
	input  logic       cdda_ce,

	output logic [1:0][9:0]  level,
	output logic [1:0][15:0] line,

	input  logic       ss_shift,
	input  logic       ss_si,
	output logic       ss_so
);

import huc6230_pkg::*;

vca_state_t st;

logic [1:0] due, go_up, go_down;
logic [1:0][9:0]  lo_t, hi_t;
logic [1:0][14:0] mask;

always_comb begin
	for (int s = 0; s < 2; s++) begin
		lo_t[s] = lvol[s] == 6'h3F ? 10'd1023 : {lvol[s], 4'h0};
		hi_t[s] = lvol[s] == 6'h00 ? 10'd0    : {lvol[s], 4'hF};

		mask[s] = 15'h3FFF;
		for (int b = 1; b < 10; b++)
			if (st.level[s][b])
				mask[s] = 15'((32 << (9 - b)) - 1);
		due[s] = ce_master && (st.pre & mask[s]) == 15'd0;

		go_up[s]   = st.level[s] < lo_t[s] || (st.up[s] && st.level[s] < hi_t[s]);
		go_down[s] = st.level[s] > hi_t[s] || (st.down[s] && st.level[s] > lo_t[s]);
	end
end

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= vca_state_t'({st[SS_VCA_BITS-2:0], ss_si});
	end else begin
		if (ce_master)
			st.pre <= st.pre + 15'd1;
		for (int s = 0; s < 2; s++) begin
			if (due[s]) begin
				st.up[s]   <= go_up[s];
				st.down[s] <= go_down[s];
				if (go_up[s])
					st.level[s] <= st.level[s] + 10'd1;
				else if (go_down[s])
					st.level[s] <= st.level[s] - 10'd1;
			end
		end
		if (cdda_ce) begin
			st.line[0] <= cdda_l;
			st.line[1] <= cdda_r;
		end
	end
end

assign ss_so = SAVESTATE ? st[SS_VCA_BITS-1] : ss_si;
assign level = st.level;
assign line  = st.line;

endmodule
