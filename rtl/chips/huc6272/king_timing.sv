// KING timing: dot and raster counters locked to the HuC6261's DCKKR,
// -HSYNC and -VSYNC, the sub-dot phase, and reset release.
//
// -HSYNC is sampled on each DCK; the dot where it is first seen low is kx
// 309 and the raster steps. -VSYNC falls just after the -HSYNC of raster 0,
// so the raster wraps at that -HSYNC by guessing the field is as long as the
// one two fields back (262 or 263 lines; interlace alternates). -VSYNC
// zeroes the line count and fixes a wrong guess. ph counts OSC clocks from
// the dot start.
// Reset is released on a DCK, then eight dots of KRAM refresh run; the
// counters restart from zero and lock at the next -HSYNC and -VSYNC.

module king_timing #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic       clk,
	input  logic       reset,
	input  logic       ce_osc,
	input  logic       ce_dck,
	input  logic       hsync_n,
	input  logic       vsync_n,

	output logic       rst_int,
	output logic       init,      // initial refresh dots
	output logic [8:0] kx,
	output logic [8:0] nkx,       // the dot being planned
	output logic [3:0] ph,
	output logic [8:0] raster,
	output logic       disp_line,
	output logic       hs_ev,     // just after the -HSYNC dot starts
	output logic       vs_ev,     // -VSYNC moved the raster to 0

	input  logic       ss_shift,
	input  logic       ss_si,
	output logic       ss_so
);

import huc6272_pkg::*;

timing_state_t st;
logic rst_q;

wire hs_fall = ce_dck && st.hs_q && !hsync_n;
wire vs_fall = ce_dck && st.vs_q && !vsync_n;
wire [8:0] flen = {8'h83, st.long_f[1]};   // 262 or 263

always_ff @(posedge clk) begin
	if (reset)
		rst_q <= 1'b1;
	else if (ce_dck)
		rst_q <= 1'b0;
end

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= timing_state_t'({st[TIMING_BITS-2:0], ss_si});
	end else if (ce_osc) begin
		st.ph    <= ce_dck ? 4'd0 : (st.ph == 4'hF ? st.ph : st.ph + 4'd1);
		st.hs_ev <= hs_fall;
		st.vs_ev <= vs_fall && raster != 9'd0;

		if (ce_dck) begin
			st.hs_q <= hsync_n;
			st.vs_q <= vsync_n;
			if (hs_fall)
				st.kx <= KX_HSYNC;
			else
				st.kx <= nkx;

			if (vs_fall) begin
				st.lc     <= 9'd0;
				st.long_f <= {st.long_f[0], st.lc == 9'd263};
			end else if (hs_fall) begin
				st.lc <= st.lc + 9'd1;
			end

			if (rst_q)
				st.init <= 4'd8;
			else if (st.init != 4'd0)
				st.init <= st.init - 4'd1;
		end
	end
end

assign ss_so = SAVESTATE ? st[TIMING_BITS-1] : ss_si;

assign rst_int   = rst_q;
assign init      = st.init != 4'd0;
assign kx        = st.kx;
assign nkx       = st.kx == KX_LAST ? 9'd0 : st.kx + 9'd1;
assign ph        = st.ph;
assign raster    = st.lc == flen ? 9'd0 : st.lc;
assign hs_ev     = st.hs_ev;
assign vs_ev     = st.vs_ev;
assign disp_line = raster >= RASTER_FIRST && raster <= RASTER_LAST;

endmodule
