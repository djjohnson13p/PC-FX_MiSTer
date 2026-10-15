// KING BG processing (C6272_2 3.6): registers 10, 12-16, 20-3D, the
// microprogram, address generation for both banks, BG0 rotation, the line
// pipes and the priority mix.
//
//   mprog --(bank A word, bank B word)--> addrgen A --> bank A arbiter
//                                     \-> addrgen B --> bank B arbiter
//   affine (BG0 rotated coordinates) --^       |
//   KRAM data --> linepipe (BAT latches, line RAM, cut out, mix) --> pixel
//
// Registers marked H in C6272_1 (12, 20-2B, 30-3D) are written to a CPU
// copy and take effect at the next -HSYNC; 10, 13-16 and 2C-2F act at
// once. The microprogram runs on display rasters 22-261 while MPSW is 1:
// kx 0-279 of each line, cycle kx mod 8.

module king_bg #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_osc,
	input  logic        ce_dck,
	input  logic  [3:0] ph,
	input  logic  [8:0] kx,
	input  logic  [8:0] nkx,
	input  logic  [8:0] raster,
	input  logic        disp_line,
	input  logic        hs_ev,

	input  huc6272_pkg::acc_t acc,
	input  logic        page,        // BGP

	output huc6272_pkg::kreq_t req_a,
	output huc6272_pkg::kreq_t req_b,
	input  logic        gnt_a,
	input  logic        gnt_b,
	input  logic [15:0] ka_q,
	input  logic [15:0] kb_q,

	output pcfx_pixel_pkg::pcfx_pixel_t pix,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

import huc6272_pkg::*;

bg_state_t st;
hregs_t    h;           // registers in effect for this line
logic      ss_mid, hs_ev2;

wire wr_reg = ce_osc && acc.wr && !acc.ar && !acc.hi;

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= bg_state_t'({st[BG_BITS-2:0], ss_si});
	end else if (wr_reg) begin
		case (acc.r)
			7'h10: st.mode <= acc.d;
			7'h12: {st.h.rsw, st.h.prio} <= acc.d[12:0];
			7'h16: st.cyl <= acc.d[3:0];
			7'h20: st.h.bat[0] <= acc.d[7:0];
			7'h21: st.h.cg[0]  <= acc.d[7:0];
			7'h22: st.h.bat0s  <= acc.d[7:0];
			7'h23: st.h.cg0s   <= acc.d[7:0];
			7'h24: st.h.bat[1] <= acc.d[7:0];
			7'h25: st.h.cg[1]  <= acc.d[7:0];
			7'h28: st.h.bat[2] <= acc.d[7:0];
			7'h29: st.h.cg[2]  <= acc.d[7:0];
			7'h2A: st.h.bat[3] <= acc.d[7:0];
			7'h2B: st.h.cg[3]  <= acc.d[7:0];
			7'h2C: st.size0    <= acc.d;
			7'h2D: st.size[0]  <= acc.d[7:0];
			7'h2E: st.size[1]  <= acc.d[7:0];
			7'h2F: st.size[2]  <= acc.d[7:0];
			7'h30: st.h.sx0    <= acc.d[10:0];
			7'h31: st.h.sy0    <= acc.d[10:0];
			7'h32: st.h.sx[0]  <= acc.d[9:0];
			7'h33: st.h.sy[0]  <= acc.d[9:0];
			7'h34: st.h.sx[1]  <= acc.d[9:0];
			7'h35: st.h.sy[1]  <= acc.d[9:0];
			7'h36: st.h.sx[2]  <= acc.d[9:0];
			7'h37: st.h.sy[2]  <= acc.d[9:0];
			7'h38: st.h.aa     <= acc.d;
			7'h39: st.h.ab     <= acc.d;
			7'h3A: st.h.ac     <= acc.d;
			7'h3B: st.h.ad     <= acc.d;
			7'h3C: st.h.x0     <= acc.d;
			7'h3D: st.h.y0     <= acc.d;
			default: ;
		endcase
	end
end

assign ss_mid = SAVESTATE ? st[BG_BITS-1] : ss_si;

// The line copy and line Y are rebuilt at every -HSYNC, so they stay out
// of the savestate chain (capture is on a vertical-blank line).
logic [3:0][10:0] yl;
wire  [8:0] ry = raster - RASTER_FIRST;   // -22..240

always_ff @(posedge clk) begin
	if (ce_osc) begin
		hs_ev2 <= hs_ev;
		if (hs_ev)
			h <= st.h;
		if (hs_ev2) begin
			yl[0] <= {{2{ry[8]}}, ry} + h.sy0;
			yl[1] <= {1'b0, {ry[8], ry} + h.sy[0]};
			yl[2] <= {1'b0, {ry[8], ry} + h.sy[1]};
			yl[3] <= {1'b0, {ry[8], ry} + h.sy[2]};
		end
	end
end

// ------------------------------------------------------------ microprogram

logic       mpsw;
mpw_t       wa, wb;
logic [1:0][3:0] has_cg;

king_bg_mprog #(.SAVESTATE(SAVESTATE)) mprog (
	.clk, .reset, .ce_osc,
	.acc,
	.cyc (nkx[2:0]),
	.mpsw, .wa, .wb, .has_cg,
	.ss_shift,
	.ss_si (ss_mid),
	.ss_so
);

wire active = mpsw && disp_line && nkx < KX_SEQ_END;

logic [3:0] shown;
always_comb begin
	for (int i = 0; i < 4; i++)
		shown[i] = mpsw && h.prio[i] != 3'd0
		        && (has_cg[h.cg[i][7]][i] || (i == 0 && has_cg[h.cg0s[7]][0]))
		        && st.mode[i*4 +: 4] != 4'h0
		        && st.mode[i*4 +: 4] != 4'h6 && st.mode[i*4 +: 4] != 4'h7 && st.mode[i*4 +: 4] != 4'h8;
end

// ------------------------------------------------------------ rotation

logic [15:0] ax, ay, bx, by;

king_bg_affine affine (
	.clk, .ce_osc, .ce_dck,
	.start (hs_ev2),
	.kx, .raster,
	.sx0 (h.sx0), .sy0 (h.sy0),
	.a (h.aa), .b (h.ab), .c (h.ac), .d (h.ad),
	.x0 (h.x0), .y0 (h.y0),
	.ax, .ay, .bx, .by
);

// ------------------------------------------------------------ addresses

logic [3:0][15:0] batl;
logic             va, vb;
logic      [16:0] aa, ab;
slot_rec_t        ra, rb;

king_bg_addrgen #(.BANK(1'b0)) gen_a (
	.clk, .ce_osc, .ph, .active, .nkx,
	.w (wa),
	.prio (h.prio), .mode (st.mode), .size0 (st.size0), .size (st.size), .cyl (st.cyl),
	.bat (h.bat), .cg (h.cg), .bat0s (h.bat0s), .cg0s (h.cg0s),
	.sx0 (h.sx0), .sx (h.sx), .yl, .batl,
	.ax, .ay, .bx, .by,
	.req_v (va), .req_a (aa), .rec (ra)
);

king_bg_addrgen #(.BANK(1'b1)) gen_b (
	.clk, .ce_osc, .ph, .active, .nkx,
	.w (wb),
	.prio (h.prio), .mode (st.mode), .size0 (st.size0), .size (st.size), .cyl (st.cyl),
	.bat (h.bat), .cg (h.cg), .bat0s (h.bat0s), .cg0s (h.cg0s),
	.sx0 (h.sx0), .sx (h.sx), .yl, .batl,
	.ax, .ay, .bx, .by,
	.req_v (vb), .req_a (ab), .rec (rb)
);

always_comb begin
	req_a      = '0;
	req_a.v    = va;
	req_a.page = page;
	req_a.a    = aa;
	req_b      = '0;
	req_b.v    = vb;
	req_b.page = page;
	req_b.a    = ab;
end

// ------------------------------------------------------------ line pipes

slot_rec_t  ga, gb;
logic       bat_we;
logic [1:0] bat_n;
logic [15:0] bat_d;

always_comb begin
	ga = ra;
	gb = rb;
	ga.v = ra.v && gnt_a;
	gb.v = rb.v && gnt_b;
end

// Registers, not a RAM: two address generators read them at once.
always_ff @(posedge clk) begin
	if (reset)
		batl <= '0;
	else if (bat_we)
		batl[bat_n] <= bat_d;
end

king_bg_linepipe pipe (
	.clk, .ce_osc, .ce_dck, .ph,
	.rec_a (ga), .rec_b (gb),
	.ka_q, .kb_q,
	.bat_we, .bat_n, .bat_d,
	.nkx, .disp_line, .shown,
	.mode (st.mode), .prio (h.prio), .rot0 (h.rsw),
	.fine ({h.sx[2][2:0], h.sx[1][2:0], h.sx[0][2:0], h.rsw ? 3'd0 : h.sx0[2:0]}),
	.pix
);

endmodule
