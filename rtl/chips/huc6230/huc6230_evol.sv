// HuC6230 ADPCM volumes (R11-R14): 3Fh is 0 dB, each step -1.5 dB, 1Bh
// and below silent. On the chip each setting drives the DAC reference
// through an RC, so a change glides. Here one first-order smoother is shared
// by the four volumes, one per 31.47 kHz tick, each moving 1/256 of the way:
// a time constant of about 32 ms (assumption: the board capacitor is not
// documented). Decoding goes on at any volume.

module huc6230_evol #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic       clk,
	input  logic       reset,

	input  logic       tick,
	input  logic [3:0][5:0] avol,

	output logic [3:0][15:0] gain,  // #1 L, R, #2 L, R; 65535 = 0 dB

	input  logic       ss_shift,
	input  logic       ss_si,
	output logic       ss_so
);

import huc6230_pkg::*;

evol_state_t st;

wire  [5:0] v      = avol[st.sel];
wire [15:0] target = v >= 6'h1C ? gain_db15(6'h3F - v) : 16'd0;
/* verilator lint_off UNUSEDSIGNAL */
wire [24:0] diff   = {1'b0, target, 8'd0} - {1'b0, st.g[st.sel]};
/* verilator lint_on UNUSEDSIGNAL */
wire [23:0] step   = {{7{diff[24]}}, diff[24:8]};

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= evol_state_t'({st[SS_EVOL_BITS-2:0], ss_si});
	end else if (tick) begin
		st.g[st.sel] <= st.g[st.sel] + step;
		st.sel       <= st.sel + 2'd1;
	end
end

assign ss_so = SAVESTATE ? st[SS_EVOL_BITS-1] : ss_si;

always_comb
	for (int i = 0; i < 4; i++)
		gain[i] = st.g[i][23:8];

endmodule
