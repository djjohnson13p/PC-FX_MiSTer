// HuC6230 ADPCM timing: 31.47 kHz ticks, one on each -HSYNC fall and one
// half a line later, and the transfer cycle of 2, 4, 8 or 16 lines. The
// cycle locks on the first -HSYNC fall after the first K-R byte and keeps
// that phase until a hard reset or a reset of both channels. With -HSYNC
// stopped there are no ticks, so both channels hold.

module huc6230_adpcm_tick #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic       clk,
	input  logic       reset,
	input  logic       ce_master,

	input  logic       hs_fall,
	input  logic [1:0] div,
	input  logic       unlock,
	input  logic       seen,

	output logic       tick,
	output logic       boundary,
	output logic [1:0] nib,       // nibble of the cycle being played
	output logic       first,     // first tick of that nibble

	input  logic       ss_shift,
	input  logic       ss_si,
	output logic       ss_so
);

import huc6230_pkg::*;

// Assumption: the mid-line tick comes 682 master clocks after the fall.
localparam logic [9:0] HALF = 10'd682;

tick_state_t st;

wire half_tick = ce_master && !hs_fall && st.lpos == HALF - 10'd1;
wire [3:0] last = {div == 2'd3, div >= 2'd2, div >= 2'd1, 1'b1};   // 2^(DIV+1) - 1

assign tick     = hs_fall || half_tick;
assign boundary = hs_fall && !unlock && (st.locked ? st.lcount == last : seen);

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
		st.lpos <= 10'h3FF;
	end else if (SAVESTATE && ss_shift) begin
		st <= tick_state_t'({st[SS_TICK_BITS-2:0], ss_si});
	end else begin
		if (hs_fall)
			st.lpos <= 10'd0;
		else if (ce_master && st.lpos != 10'h3FF)
			st.lpos <= st.lpos + 10'd1;

		if (boundary) begin
			st.lcount <= 4'd0;
			st.locked <= 1'b1;
		end else if (hs_fall) begin
			st.lcount <= st.lcount + 4'd1;
		end
		if (unlock)
			st.locked <= 1'b0;
	end
end

// Tick number in the cycle is {line, half}; a nibble lasts 2^DIV ticks.
logic [4:0] tnum;

always_comb begin
	tnum  = boundary ? 5'd0 : {hs_fall ? st.lcount + 4'd1 : st.lcount, !hs_fall};
	nib   = 2'(tnum >> div);
	first = (tnum & ((5'd1 << div) - 5'd1)) == 5'd0;
end

assign ss_so = SAVESTATE ? st[SS_TICK_BITS-1] : ss_si;

endmodule
