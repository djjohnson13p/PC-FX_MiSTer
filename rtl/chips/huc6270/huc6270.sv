// HuC6270 video display controller (VDC), at its package pins.
//
//   CPU bus --> cpu (AR, R05-R0E, status, BUSY)
//           \-> vram (MAWR/MARR/VWR/VRR, DMA, SATB DMA) <--> MA/MD/MRD/MWR
//   /HSYN /VSYN --> timing (counters, slot plan, VD/RR IRQ)
//   slots --> bg (BAT/CG) and sprite (SATB search, SG fetch, line buffer)
//   bg + sprite --> out (priority, 8-dot pipe) --> SPBG, VD7-0
//
// CK is the DCK enable from the HuC6261. /HSYN and /VSYN are sampled at
// each DCK. The EX output modes drive the sync pins from the internal
// counters while the counters keep following the pin inputs.
//
// Savestate: the chain holds the registers, counters, transfer state and
// status. VRAM is read and written through the MA/MD pins and the SATB
// through its block-transfer port while ss_mem_sel is high (address bit
// 16 picks the SATB); read data follows the address by one clock. Capture
// on a line with no sprite work: outside VDS_END-1 .. VDISP_END (rasters
// 0-19 and 262 with the usual 1102h/00EFh), with the bus idle.

module huc6270
	import huc6270_pkg::*;
#(
	parameter bit SAVESTATE = 1'b1,
	parameter bit DEBUG     = 1'b0
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_dck,

	input  logic        cs_n,
	input  logic        rd_n,
	input  logic        wr_n,
	input  logic        a1,
	input  logic        a0,
	input  logic        mode8_16,          // 8/16 pin: high selects the 8-bit bus
	input  logic [15:0] d_i,
	output logic [15:0] d_o,
	output logic        d_oe,
	output logic        busy_n,
	output logic        irq_n,

	input  logic        hsyn_n_i,
	output logic        hsyn_n_o,
	output logic        hsyn_n_oe,
	input  logic        vsyn_n_i,
	output logic        vsyn_n_o,
	output logic        vsyn_n_oe,

	output logic        disp,
	output logic        spbg,
	output logic  [7:0] vd,

	output logic [15:0] ma,
	input  logic [15:0] md_i,
	output logic [15:0] md_o,
	output logic        md_oe,
	output logic        mrd_n,
	output logic        mwr_n,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so,
	input  logic        ss_mem_sel,
	input  logic [16:0] ss_mem_addr,
	input  logic        ss_mem_we,
	input  logic [15:0] ss_mem_wdata,
	output logic [15:0] ss_mem_rdata,

	output logic  [3:0] dbg_slot,
	output logic  [9:0] dbg_line
);

regs_state_t regs;

logic        acc_wr, acc_rd, acc_a1, acc_lo, acc_hi, sr_rd;
logic [15:0] acc_wd;
logic        vram_busy;
logic [15:0] vrr;
logic        irq_vbl, irq_dma, irq_dmas, irq_rcr, irq_ovf, irq_col;

slot_t       slot;
logic        dot0, hs_f;
logic        at_hds_d3, at_hds_d7, at_hdsm2_d7, at_hdsm3_d7, at_hdisp_d3, at_hdisp_d7, brk_d7;
logic        v_ge, v_gt, v_first, v_idle, vbl_start;
logic        vdisp, burst, bg_out, bb, sb;
logic  [1:0] vm, sm;
logic  [2:0] screen;
logic  [9:0] rc_cnt, line;
logic        hsyn_int_n, vsyn_int_n;

logic        dma_exec, dmas_exec;
logic  [7:0] sat_addr;
logic        sat_we;
logic [15:0] bg_addr, spr_addr;
logic  [7:0] bg_pix;
logic  [8:0] spr_pix;
logic        spr_fetch, spr_fetch_en, spr_cg;
logic [15:0] sat_qa;

logic [15:0] eng_ma, eng_md;
logic        eng_mrd_n, eng_mwr_n;
logic  [8:0] vd9;

logic        ss_c0, ss_c1, ss_c2;

huc6270_cpu #(.SAVESTATE(SAVESTATE)) u_cpu (
	.clk       (clk),
	.reset     (reset),
	.ce_dck    (ce_dck),
	.cs_n      (cs_n),
	.rd_n      (rd_n),
	.wr_n      (wr_n),
	.a1        (a1),
	.a0        (a0),
	.mode8     (mode8_16),
	.d_i       (d_i),
	.d_o       (d_o),
	.d_oe      (d_oe),
	.busy_n    (busy_n),
	.acc_wr    (acc_wr),
	.acc_rd    (acc_rd),
	.acc_a1    (acc_a1),
	.acc_wd    (acc_wd),
	.acc_lo    (acc_lo),
	.acc_hi    (acc_hi),
	.sr_rd     (sr_rd),
	.vram_busy (vram_busy),
	.vrr       (vrr),
	.irq_flags ({irq_vbl, irq_dma, irq_dmas, irq_rcr, irq_ovf, irq_col}),
	.byr_clr   (at_hdsm3_d7),
	.regs      (regs),
	.ss_shift  (ss_shift),
	.ss_si     (ss_si),
	.ss_so     (ss_c0)
);

huc6270_vram #(.SAVESTATE(SAVESTATE)) u_vram (
	.clk       (clk),
	.reset     (reset),
	.ce_dck    (ce_dck),
	.acc_wr    (acc_wr),
	.acc_rd    (acc_rd),
	.acc_a1    (acc_a1),
	.acc_wd    (acc_wd),
	.acc_lo    (acc_lo),
	.acc_hi    (acc_hi),
	.sr_rd     (sr_rd),
	.ar        (regs.ar),
	.cr_iw     (regs.cr_iw),
	.slot      (slot),
	.dot0      (dot0),
	.burst     (burst),
	.vdisp     (vdisp),
	.vbl_start (vbl_start),
	.bg_addr   (bg_addr),
	.spr_addr  (spr_addr),
	.busy      (vram_busy),
	.vrr       (vrr),
	.dma_exec  (dma_exec),
	.dmas_exec (dmas_exec),
	.irq_dma   (irq_dma),
	.irq_dmas  (irq_dmas),
	.sat_addr  (sat_addr),
	.sat_we    (sat_we),
	.ma        (eng_ma),
	.md_i      (md_i),
	.md_o      (eng_md),
	.mrd_n     (eng_mrd_n),
	.mwr_n     (eng_mwr_n),
	.ss_shift  (ss_shift),
	.ss_si     (ss_c0),
	.ss_so     (ss_c1)
);

huc6270_timing #(.SAVESTATE(SAVESTATE)) u_timing (
	.clk          (clk),
	.reset        (reset),
	.ce_dck       (ce_dck),
	.hsyn_n       (hsyn_n_i),
	.vsyn_n       (vsyn_n_i),
	.regs         (regs),
	.sr_rd        (sr_rd),
	.dmas_exec    (dmas_exec),
	.dma_exec     (dma_exec),
	.spr_fetch    (spr_fetch),
	.spr_fetch_en (spr_fetch_en),
	.spr_cg       (spr_cg),
	.slot         (slot),
	.dot0         (dot0),
	.hs_f         (hs_f),
	.at_hds_d3    (at_hds_d3),
	.at_hds_d7    (at_hds_d7),
	.at_hdsm2_d7  (at_hdsm2_d7),
	.at_hdsm3_d7  (at_hdsm3_d7),
	.at_hdisp_d3  (at_hdisp_d3),
	.at_hdisp_d7  (at_hdisp_d7),
	.brk_d7       (brk_d7),
	.v_ge         (v_ge),
	.v_gt         (v_gt),
	.v_first      (v_first),
	.v_idle       (v_idle),
	.vbl_start    (vbl_start),
	.vdisp        (vdisp),
	.burst        (burst),
	.bg_out       (bg_out),
	.bb           (bb),
	.sb           (sb),
	.vm           (vm),
	.sm           (sm),
	.screen       (screen),
	.rc_cnt       (rc_cnt),
	.irq_vbl      (irq_vbl),
	.irq_rcr      (irq_rcr),
	.hsyn_int_n   (hsyn_int_n),
	.vsyn_int_n   (vsyn_int_n),
	.line         (line),
	.ss_shift     (ss_shift),
	.ss_si        (ss_c1),
	.ss_so        (ss_c2)
);

huc6270_bg u_bg (
	.clk         (clk),
	.reset       (reset),
	.ce_dck      (ce_dck),
	.slot        (slot),
	.md_i        (md_i),
	.vm          (vm),
	.screen      (screen),
	.hs_f        (hs_f),
	.at_hdsm2_d7 (at_hdsm2_d7),
	.at_hdsm3_d7 (at_hdsm3_d7),
	.v_first     (v_first),
	.bg_out      (bg_out),
	.vdisp       (vdisp),
	.bxr         (regs.bxr),
	.byr         (regs.byr),
	.byr_set     (regs.byr_set),
	.bg_addr     (bg_addr),
	.bg_pix      (bg_pix)
);

// The SATB's block-transfer port doubles as its savestate port.
wire ss_sat  = SAVESTATE && ss_mem_sel &&  ss_mem_addr[16];
wire ss_vram = SAVESTATE && ss_mem_sel && !ss_mem_addr[16];

huc6270_sprite #(.SAVESTATE(SAVESTATE)) u_sprite (
	.clk          (clk),
	.reset        (reset),
	.ce_dck       (ce_dck),
	.slot         (slot),
	.md_i         (md_i),
	.sm           (sm),
	.rc_cnt       (rc_cnt),
	.at_hds_d3    (at_hds_d3),
	.at_hds_d7    (at_hds_d7),
	.at_hdsm2_d7  (at_hdsm2_d7),
	.at_hdisp_d3  (at_hdisp_d3),
	.at_hdisp_d7  (at_hdisp_d7),
	.brk_d7       (brk_d7),
	.v_ge         (v_ge),
	.v_gt         (v_gt),
	.v_idle       (v_idle),
	.bg_out       (bg_out),
	.vdisp        (vdisp),
	.cr_sb        (regs.cr_sb),
	.ie_oc        (regs.cr_ie[1]),
	.ie_cc        (regs.cr_ie[0]),
	.sr_rd        (sr_rd),
	.sat_wa       (ss_sat ? ss_mem_addr[7:0] : sat_addr),
	.sat_we       (ss_sat ? ss_mem_we : sat_we),
	.sat_wd       (ss_sat ? ss_mem_wdata : md_i),
	.sat_qa       (sat_qa),
	.spr_fetch    (spr_fetch),
	.spr_fetch_en (spr_fetch_en),
	.spr_cg       (spr_cg),
	.spr_addr     (spr_addr),
	.spr_pix      (spr_pix),
	.irq_ovf      (irq_ovf),
	.irq_col      (irq_col),
	.ss_shift     (ss_shift),
	.ss_si        (ss_c2),
	.ss_so        (ss_so)
);

huc6270_out u_out (
	.clk     (clk),
	.reset   (reset),
	.ce_dck  (ce_dck),
	.hs_f    (hs_f),
	.bg_out  (bg_out),
	.vdisp   (vdisp),
	.burst   (burst),
	.bb      (bb),
	.sb      (sb),
	.bg_pix  (bg_pix),
	.spr_pix (spr_pix),
	.vd      (vd9)
);

assign ma     = ss_vram ? ss_mem_addr[15:0] : eng_ma;
assign md_o   = ss_vram ? ss_mem_wdata : eng_md;
// The slot decode follows the chain bits while they shift; keep MWR off.
assign mwr_n  = ss_vram ? !ss_mem_we : eng_mwr_n || (SAVESTATE && ss_shift);
assign mrd_n  = ss_vram ? ss_mem_we : eng_mrd_n;
assign md_oe  = !mwr_n;

assign ss_mem_rdata = ss_mem_addr[16] ? sat_qa : md_i;

assign spbg = vd9[8];
assign vd   = vd9[7:0];
assign irq_n = !(irq_col || irq_ovf || irq_rcr || irq_dmas || irq_dma || irq_vbl);

// DISP pin: display period, or the internal horizontal sync (TE).
always_comb begin
	case (regs.cr_te)
		2'd0:    disp = vd9 != 9'h100;
		2'd2:    disp = hsyn_int_n;
		default: disp = 1'b1;
	endcase
end

assign hsyn_n_o  = hsyn_int_n;
assign hsyn_n_oe = regs.cr_ex[0];
assign vsyn_n_o  = vsyn_int_n;
assign vsyn_n_oe = regs.cr_ex == 2'd3;

assign dbg_slot = DEBUG ? slot : 4'd0;
assign dbg_line = DEBUG ? line : 10'd0;

endmodule
