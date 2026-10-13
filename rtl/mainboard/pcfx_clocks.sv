// Board clock enables, all counted on clk_sys = the 42.954545 MHz oscillator.
//
//   clk        1 2 3 4 5 6 ...
//   ce_sys     1 1 1 1 1 1      42.954 MHz, for logic on the oscillator itself
//   ce_master  1 0 1 0 1 0      21.477 MHz
//   ce_sck     1 0 1 0 1 0      V810 CLK
//   phi1       0 1 0 1 0 1      second half of each SCK cycle
//   ce_timer   every 30th clk, on a master clk
//   ce_kport   every KPORT_DIV-th clk, four per K-port CLK (/128 guessed)
//
// run low freezes every enable and divider; reset restarts them in phase.

module pcfx_clocks #(
	parameter int KPORT_DIV = 32,     // 2..256
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic clk,
	input  logic reset,
	input  logic run,

	output logic ce_sys,
	output logic ce_master,
	output logic ce_sck,
	output logic phi1,
	output logic ce_timer,
	output logic ce_kport,

	input  logic ss_shift,
	input  logic ss_si,
	output logic ss_so
);

import pcfx_clocks_pkg::*;

localparam logic [7:0] KPORT_LAST = 8'(KPORT_DIV - 1);

state_t st;

wire go = run && !reset;

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= state_t'({st[SS_BITS-2:0], ss_si});
	end else if (run) begin
		st.odd   <= !st.odd;
		st.timer <= (st.timer == 5'd29) ? 5'd0 : st.timer + 5'd1;
		st.kport <= (st.kport == KPORT_LAST) ? 8'd0 : st.kport + 8'd1;
	end
end

assign ss_so = SAVESTATE ? st[SS_BITS-1] : ss_si;

assign ce_sys    = go;
assign ce_master = go && !st.odd;
assign ce_sck    = ce_master;
assign ce_timer  = go && st.timer == 5'd0;
assign ce_kport  = go && st.kport == 8'd0;

always_ff @(posedge clk)
	phi1 <= ce_sck;

endmodule
