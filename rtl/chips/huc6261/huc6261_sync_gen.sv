// HuC6261 sync generator: line and raster counters, dot clocks for KING,
// Rainbow and the 6270s, their syncs, and the board-edge video timing.
// The counters keep running through reset, as on the chip.

module huc6261_sync_gen #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic         clk,
	input  logic         reset,
	input  logic         ce_master,

	input  huc6261_pkg::frame_t cr,   // CR 7:0 as written

	output logic  [10:0] hc,
	output logic   [8:0] vc,
	output logic         oe,
	output huc6261_pkg::frame_t frm,
	output logic         disp,

	output logic         ce_dckkr,
	output logic         ce_dck70,
	output logic         hsynca_n,
	output logic         hsyncb_n,
	output logic         hsyncc_n,
	output logic         vsync_n,

	output logic         copy,       // line latch slot, word hc[4:0]

	output logic         tv_hs,
	output logic         tv_vs,
	output logic         tv_hblank,
	output logic         tv_vblank,
	output logic         tv_ce_pix,
	output logic         half_delay, // unshifted line in half-dot mode

	input  logic         ss_shift,
	input  logic         ss_si,
	output logic         ss_so
);

import huc6261_pkg::*;

sync_state_t st;

wire line_end = st.hc == H_LAST;
wire interlace = st.frm.dcc[1];

// 263 or 262 lines; interlace alternates 262 (odd) and 263 (even)
wire [8:0] v_last = (interlace ? st.oe : st.frm.dcc[0]) ? 9'd261 : 9'd262;
wire field_end = st.vc == v_last;

frame_t frm_next;
assign frm_next = cr;

// Output clock: hc less the mixer latency, so oc 0 shows window clock 0.
wire [10:0] lat = st.frm.dc7 ? LAT_7 : LAT_256;
wire [11:0] oc_raw = {1'b0, st.hc} - {1'b0, lat};
wire [10:0] oc = oc_raw[11] ? oc_raw[10:0] + 11'd1365 : oc_raw[10:0];
wire [10:0] oc_next = oc == H_LAST ? 11'd0 : oc + 11'd1;

always_ff @(posedge clk) begin
	if (SAVESTATE && ss_shift) begin
		st <= sync_state_t'({st[SS_SYNC_BITS-2:0], ss_si});
	end else begin
		if (ce_master) begin
			st.hc <= line_end ? 11'd0 : st.hc + 11'd1;
			st.p3 <= (line_end || st.p3 == 2'd2) ? 2'd0 : st.p3 + 2'd1;

			if (st.hc == H_RASTER - 11'd1) begin
				st.vc <= field_end ? 9'd0 : st.vc + 9'd1;
				if (field_end) begin
					st.frm <= frm_next;
					st.oe  <= frm_next.dcc[1] && !st.oe;
				end
			end

			if (st.hc == H_VS - 11'd1) begin
				if (st.vc == 9'd0) st.vs_n <= 1'b0;
				if (st.vc == 9'd3) st.vs_n <= 1'b1;
			end

			if (oc_next == (interlace && !st.oe ? TV_VS_EV : TV_HS)) begin
				if (st.vc == 9'd0) st.tv_vs <= 1'b1;
				if (st.vc == 9'd3) st.tv_vs <= 1'b0;
			end
		end
		if (reset)
			st.frm <= '0;
	end
end

assign ss_so = SAVESTATE ? st[SS_SYNC_BITS-1] : ss_si;

assign hc  = st.hc;
assign vc  = st.vc;
assign oe  = st.oe;
assign frm = st.frm;

wire v_disp = st.vc >= V_FIRST && st.vc <= V_LAST;
assign disp = v_disp && st.hc < H_WINDOW;

// A source loads dot n in the cycle hc steps to 4n; the line's last dot is
// five master clocks long. DCK70 in 7 MHz mode is every third master clock.
assign ce_dckkr = ce_master && ((st.hc[1:0] == 2'd3 && st.hc != H_LAST - 11'd1) || line_end);
assign ce_dck70 = st.frm.dc7 ? ce_master && st.p3 == 2'd2 : ce_dckkr;

wire [10:0] hs_start = st.frm.dc7 ? H_HS7 : H_HS;
assign hsynca_n = !(st.hc >= hs_start && st.hc < H_HS_END);
assign hsyncb_n = !(st.hc >= H_HS && st.hc < H_HS_END);
// The decoder holds its line count through the extra non-interlace line.
assign hsyncc_n = hsyncb_n || (!interlace && st.vc == 9'd262);
assign vsync_n  = st.vs_n;

assign copy = st.hc >= H_COPY && st.hc <= H_COPY_END;

// Half-dot mode shows 255.5 dots: lines shifted left lose their first half
// dot, the others their last.
wire half = st.frm.dcc == 2'b11;
wire de_h = oc < H_WINDOW && (!half || oc >= 11'd2);

assign tv_hblank  = !de_h;
assign tv_vblank  = !v_disp;
assign tv_hs      = oc >= TV_HS && oc < TV_HS_END;
assign tv_vs      = st.tv_vs;
assign half_delay = half && !(st.vc[0] ^ st.oe);
assign tv_ce_pix  = ce_master && (st.frm.dc7 || (half ? oc[0] : oc[1:0] == 2'd3));

endmodule
