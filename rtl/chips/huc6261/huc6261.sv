// HuC6261 video colour encoder: the PC-FX video timing master, palette,
// priority, chroma key and cellophane mixer. Y/U/V and TV timing come out
// digitally in place of the DACs.
//
//   CPU --> cpu_if --(R04-R15 file)--> pal_lookup <-- line copy
//            |  CR                       ^  palette lookups
//            v                           |
//   sync_gen --> DCKKR, DCK70, -HSYNCA/B/C, -VSYNC, OD/-EV, TV timing
//   VDA/VDB --> in_vdc --\
//   KING    --> in_kr  ---> prio_sort --> cello_alu (256 dots) --> yuv_out
//   Rainbow --> in_kr  --/            \-> hd_path   (DC7 = 1)  -/
//
// Board: ce_osc = ce_sys, ce_master from the clock block. CS from the glue's
// 0x300 select, A1 from I/O address bit 2; a 32-bit access arrives as two
// 16-bit cycles. Y/U/V and tv_* go to yuv2rgb, whose ce_pix_o is CE_PIXEL;
// tv_field drives VGA_F1 while tv_interlace is set.
//
// Savestate: SS_BITS of chain plus words 0-533 of the RAM port (palette,
// then R04-R15 at 516-533). Capture while ss_window is high (rasters 0-19
// and 262): the line registers reload from the RAM every line before display. The sync
// counters keep running through reset; reset clears the registers and,
// within 32 clk_sys, palette 0 and R04-R15.

module huc6261 #(
	parameter bit SAVESTATE = 1'b1,
	parameter bit DEBUG     = 1'b0
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_osc,       // OSC, 42.95 MHz
	input  logic        ce_master,    // OSC / 2, on the same edges as ce_sck

	input  logic        cs_n,
	input  logic        rd_n,
	input  logic        wr_n,
	input  logic        a1,           // CPU A2 on the PC-FX
	input  logic [15:0] d_i,
	output logic [15:0] d_o,
	output logic        d_oe,

	output logic        ce_dckkr,     // KING and Rainbow dot clock
	output logic        ce_dck70,     // 6270 dot clock
	output logic        hsynca_n,     // 6270s
	output logic        hsyncb_n,     // KING, Rainbow
	output logic        hsyncc_n,     // 6230
	output logic        vsync_n,
	output logic        od_ev,

	input  logic  [7:0] vda,          // lower 6270
	input  logic        spbg_a,
	input  logic  [7:0] vdb,          // upper 6270
	input  logic        spbg_b,
	input  pcfx_pixel_pkg::pcfx_pixel_t king,
	input  pcfx_pixel_pkg::pcfx_pixel_t rainbow,

	output logic  [7:0] y,
	output logic  [7:0] u,
	output logic  [7:0] v,
	output logic        tv_hs,
	output logic        tv_vs,
	output logic        tv_hblank,
	output logic        tv_vblank,
	output logic        tv_ce_pix,    // 5.37, 10.74 or 21.48 MHz, per field
	output logic        tv_field,     // 1 = odd field
	output logic        tv_interlace,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so,
	output logic        ss_window,    // quiet rasters for a capture
	input  logic        ss_mem_sel,
	input  logic [huc6261_pkg::SS_MEM_AW-1:0] ss_mem_addr,
	input  logic        ss_mem_we,
	input  logic [15:0] ss_mem_wdata,
	output logic [15:0] ss_mem_rdata,

	output logic [10:0] dbg_hc,
	output logic  [8:0] dbg_vc
);

import huc6261_pkg::*;

frame_t     cr_frame;
line_regs_t lr;
frame_t     frm;
logic [10:0] hc;
logic  [8:0] vc;
logic        oe, disp, copy, half_delay, ss_mid;

logic  [9:0] cpu_addr;
logic        cpu_we;
logic [15:0] q_a, q_b;

huc6261_cpu_if #(.SAVESTATE(SAVESTATE)) cpu_if (
	.clk, .reset, .ce_master,
	.cs_n, .rd_n, .wr_n, .a1, .d_i, .d_o, .d_oe,
	.vc, .oe, .disp,
	.ram_addr (cpu_addr),
	.ram_we   (cpu_we),
	.ram_q    (q_a),
	.copy,
	.copy_word (hc[4:0]),
	.copy_q    (q_b),
	.cr_frame, .lr,
	.ss_shift, .ss_si,
	.ss_so (ss_mid)
);

huc6261_sync_gen #(.SAVESTATE(SAVESTATE)) sync_gen (
	.clk, .reset, .ce_master,
	.cr (cr_frame),
	.hc, .vc, .oe, .frm, .disp,
	.ce_dckkr, .ce_dck70, .hsynca_n, .hsyncb_n, .hsyncc_n, .vsync_n,
	.copy,
	.tv_hs, .tv_vs, .tv_hblank, .tv_vblank, .tv_ce_pix, .half_delay,
	.ss_shift,
	.ss_si (ss_mid),
	.ss_so
);

assign od_ev        = oe;
assign tv_field     = oe;
assign tv_interlace = frm.dcc[1];

logic [9:0] vdc_pix;

huc6261_in_vdc in_vdc (
	.clk, .ce_master, .ce_dck70,
	.dc7   (frm.dc7),
	.bg256 (frm.bg256),
	.sp256 (frm.sp256),
	.vda, .spbg_a, .vdb, .spbg_b,
	.pix   (vdc_pix)
);

logic        k_key, k_is_pal, r_key, r_is_pal;
logic  [1:0] k_plane, unused_r_plane;
logic  [7:0] k_idx, r_idx;
logic [23:0] k_yuv, r_yuv;

huc6261_in_kr #(.RAINBOW(1'b0)) in_king (
	.clk, .ce_dckkr,
	.bus    (king),
	.dc7    (frm.dc7),
	.cky    (lr.cky),
	.cku    (lr.cku),
	.ckv    (lr.ckv),
	.key    (k_key),
	.is_pal (k_is_pal),
	.plane  (k_plane),
	.idx    (k_idx),
	.yuv    (k_yuv)
);

huc6261_in_kr #(.RAINBOW(1'b1)) in_rainbow (
	.clk, .ce_dckkr,
	.bus    (rainbow),
	.dc7    (frm.dc7),
	.cky    (lr.cky),
	.cku    (lr.cku),
	.ckv    (lr.ckv),
	.key    (r_key),
	.is_pal (r_is_pal),
	.plane  (unused_r_plane),
	.idx    (r_idx),
	.yuv    (r_yuv)
);

cand_t      cand [3];
logic [1:0] order [3];

huc6261_prio_sort prio_sort (
	.vdc (vdc_pix),
	.k_key, .k_is_pal, .k_plane, .k_idx, .k_yuv,
	.r_key, .r_is_pal, .r_idx, .r_yuv,
	.lr,
	.cand, .order
);

logic  [7:0] idx_256, off_256, idx_7, off_7;
logic [23:0] res_256, res_7;

huc6261_cello_alu cello_alu (
	.clk, .ce_osc, .ce_master,
	.q (hc[1:0]),
	.cand, .order, .lr,
	.look_idx (idx_256),
	.look_off (off_256),
	.q_b,
	.res (res_256)
);

huc6261_hd_path hd_path (
	.cand, .order,
	.blank (lr.blank),
	.look_idx (idx_7),
	.look_off (off_7),
	.q_b,
	.res (res_7)
);

huc6261_pal_lookup #(.SAVESTATE(SAVESTATE)) pal_lookup (
	.clk, .reset,
	.cpu_addr,
	.cpu_we,
	.cpu_wdata (d_i),
	.q_a,
	.disp_idx (frm.dc7 ? idx_7 : idx_256),
	.disp_off (frm.dc7 ? off_7 : off_256),
	.copy,
	.copy_word (hc[4:0]),
	.q_b,
	.ss_mem_sel, .ss_mem_addr, .ss_mem_we, .ss_mem_wdata
);

assign ss_mem_rdata = SAVESTATE ? q_a : 16'd0;
assign ss_window    = SAVESTATE && (vc < 9'd20 || vc == 9'd262);

huc6261_yuv_out yuv_out (
	.clk, .ce_master,
	.dc7 (frm.dc7),
	.q   (hc[1:0]),
	.res_256,
	.res_7,
	.half_delay,
	.de  (!tv_hblank && !tv_vblank),
	.y, .u, .v
);

assign dbg_hc = DEBUG ? hc : 11'd0;
assign dbg_vc = DEBUG ? vc : 9'd0;

endmodule
