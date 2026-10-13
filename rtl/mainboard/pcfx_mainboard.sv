// PC-FX mainboard (PWH-6010): chips and the nets between them.
//
//   upd70732 V810 -- A31:1 D31:0 BE MRQ R/W BCYST READY SZRQ INT INTV NMI ADRSERR
//    |
//   upd65641 glue --+-- RAS CAS MA WE OE -----------> main RAM (pcfx_memory)
//    |              +-- LD15:0, LA1 ----+-- mrom_msm538022c BIOS -> pcfx_memory
//    |              |                   +-- sram_upd43256 backup (LD7:0)
//    |              |                   +-- fx_bmp edge (LD7:0) -> pcfx_memory
//    |              +-- IOA5:1 IOD15:0 CS RD WR BUSY
//    |              |      +-- huc6230 SoundBox  100   -> audio_l/r; CD-DA in
//    |              |      +-- huc6271 Rainbow   200   -- sram_2264 R-RAM A/B
//    |              |      +-- huc6261           300   video master
//    |              |      +-- huc6270 #0        400 -- sram_srm20116 VRAM
//    |              |      +-- huc6270 #1        500 -- sram_srm20116 VRAM
//    |              |      +-- huc6272 KING      600 -- KRAM A/B (pcfx_memory)
//    |              |                                 -- CN401 -- pcfx_cd_drive <- HPS feed
//    |              +-- K-port 0/1 pins, ERST
//   huc6261 -- DCK70, -HSYNCA, -VSYNC to both 6270s; VDA/VDB in; DCKKR out
//
// I/O: 000 K-port 0, 080 K-port 1, 100-600 HuC chips, 700 ERST, C00/C40
// 6270 AR hold, C80 backup enables, CC0 version, E00 ITC, F00 timer,
// 0040_0000-007F_FFFF expansion (nothing fitted); also at 8000_0000 in
// memory space.
//
// Interrupt inputs of the glue and their reset levels: VDC #1 (7B) 14,
// KING (72) 13, VDC #0 (7A) 12, K-port 11, expansion 10, timer 9; 73 is the
// PC-FXGA's HuC6273 and idle here. ADRSERR -> glue AER (ISR D7). The glue's
// NMI pin is left unconnected on retail boards (AER_NMI = 0): misaligned
// accesses just run with the address masked.
//
// Undriven buses read high (pull-ups). V810 straps: 32-bit bus (SIZ16B
// low), cache allowed (ICHEEN low), no bus master (HLDRQ high).
//
// Savestate chain: glue, HuC6261, HuC6270 #0, HuC6270 #1, HuC6230, KING, Rainbow,
// CD. RAMs through ss_mem_*: 0 HuC6261, 1 HuC6270 #0, 2 HuC6270 #1, 3 backup
// SRAM, 4 HuC6230 PSG waves, 5 Rainbow R-RAM A/B and Q tables. The V810
// keeps its own window (cpu_ss_*).

module pcfx_mainboard #(
	parameter bit SAVESTATE = 1'b1,
	parameter bit DEBUG     = 1'b0,
	parameter bit AER_NMI   = 1'b0     // glue NMI pin wired to the V810 (development kits)
)
(
	input  logic        clk,
	/* verilator lint_off SYNCASYNCNET */
	input  logic        reset,          // the V810 core takes it asynchronously
	/* verilator lint_on SYNCASYNCNET */
	input  logic        ce_sys,
	input  logic        ce_sck,
	input  logic        phi1,
	input  logic        ce_timer,
	input  logic        ce_kport,

	output logic        mr_ras_n,
	output logic  [3:0] mr_cas_n,
	output logic  [9:0] mr_ma,
	output logic        mr_we_n,
	output logic [31:0] mr_d,
	input  logic [31:0] mr_q,

	output logic        rom_ce_n,
	output logic        rom_oe_n,
	output logic [19:1] rom_a,
	input  logic [15:0] rom_q,

	output logic        fx_cs_n,
	output logic        fx_rd_n,
	output logic        fx_wr_n,
	output logic [16:0] fx_a,
	output logic  [7:0] fx_d,
	input  logic  [7:0] fx_q,
	input  logic        fx_inserted,
	input  logic        fx_battery_ok,

	output logic        ce_dot,

	output logic        ka_req,
	output logic        ka_we,
	output logic  [8:0] ka_row,
	output logic  [8:0] ka_col,
	output logic  [1:0] ka_be,
	output logic [15:0] ka_d,
	input  logic [15:0] ka_q,
	output logic        kb_req,
	output logic        kb_we,
	output logic  [8:0] kb_row,
	output logic  [8:0] kb_col,
	output logic  [1:0] kb_be,
	output logic [15:0] kb_d,
	input  logic [15:0] kb_q,

	output logic  [7:0] y,
	output logic  [7:0] u,
	output logic  [7:0] v,
	output logic        tv_hs,
	output logic        tv_vs,
	output logic        tv_hblank,
	output logic        tv_vblank,
	output logic        tv_ce_pix,
	output logic        tv_field,
	output logic        tv_interlace,

	output logic        k0_latch_n,
	output logic        k0_clk,
	output logic        k0_rw,
	input  logic        k0_data_i,
	output logic        k0_data_o,
	output logic        k0_data_oe,
	output logic        k1_latch_n,
	output logic        k1_clk,
	output logic        k1_rw,
	input  logic        k1_data_i,
	output logic        k1_data_o,
	output logic        k1_data_oe,

	// CD board feed from the HPS (allowance 5)
	output logic [19:0] cd_blk_lba,
	output logic [15:0] cd_blk_base,
	output logic  [2:0] cd_blk_cnt,
	output logic        cd_blk_rd,
	input  logic        cd_blk_done,
	input  logic        cd_blk_err,
	output logic [15:0] cd_buf_addr,
	input  logic  [7:0] cd_buf_dout,
	input  logic        cd_disc,
	input  logic        cd_toc_busy,
	input  logic        cd_toc_new,
	input  logic  [6:0] cd_toc_first,
	input  logic  [6:0] cd_toc_last,
	output logic        cd_ss_busy,
	input  logic  [1:0] cd_speed,       // OSD: 0 the real drive, 1 and 2 faster

	output logic [15:0] audio_l,        // SoundBox mix, signed
	output logic [15:0] audio_r,
	input  logic        adpcm_enc_compat, // OSD options for the SoundBox, off on hardware
	input  logic        adpcm_no_click,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so,
	input  logic  [2:0] ss_mem_chip,
	input  logic        ss_mem_sel,
	input  logic [16:0] ss_mem_addr,
	input  logic        ss_mem_we,
	input  logic [15:0] ss_mem_wdata,
	output logic [15:0] ss_mem_rdata,
	output logic        bram_we,          // backup SRAM being written, for the save-file service
	output logic        ss_window,        // HuC6261 quiet rasters: a savestate may be captured
	output logic        ss_busy,          // a chip is mid-sequence: do not freeze yet

	input  logic        cpu_ss_pause_req,
	output logic        cpu_ss_pause_ready,
	input  logic        cpu_ss_restore,
	input  logic  [3:0] cpu_ss_state_addr,
	input  logic [63:0] cpu_ss_state_wdata,
	input  logic        cpu_ss_state_wren,
	output logic [63:0] cpu_ss_state_rdata,
	input  logic        cpu_ss_mem_active,
	input  logic [10:0] cpu_ss_mem_addr,
	input  logic        cpu_ss_mem_rden,
	input  logic        cpu_ss_mem_wren,
	input  logic  [7:0] cpu_ss_mem_wdata,
	output logic  [7:0] cpu_ss_mem_rdata,

	output logic        dbg_retire,
	output logic [31:0] dbg_retire_pc,
	output logic [31:0] dbg_a,
	output logic [31:0] dbg_d,
	output logic  [3:0] dbg_be_n,
	output logic        dbg_mrq_n,
	output logic        dbg_rw,
	output logic        dbg_bcyst_n,
	output logic        dbg_ready_n
);

// ------------------------------------------------------------------ V810

logic [31:1] a;
logic [31:0] cpu_d_o;
logic  [3:0] cpu_d_oe;
logic  [3:0] be_n;
logic        mrq_n, rw, bcyst_n, ready_n, szrq_n;
logic        intr, nmi_n, adrserr_n;
logic  [3:0] intv_n;
logic [31:0] d;          // the CPU data bus
logic [31:0] d_ext;      // the same bus less the glue's own drive

/* verilator lint_off UNUSEDSIGNAL */
logic        a_oe, st_oe, da_n, da_oe, mrq_oe, rw_oe, bcyst_oe, hldak_n, block;
logic  [3:0] be_oe;
logic  [1:0] st;
logic [31:0] cpu_pc, cpu_psw;
logic [15:0] cpu_insn;
logic        cpu_halted, cpu_illegal;
/* verilator lint_on UNUSEDSIGNAL */

upd70732 #(
	.DEBUG (DEBUG)
) u_cpu (
	.clk             (clk),
	.reset           (reset),
	.ce_clk          (ce_sck),
	.phi1            (phi1),
	.a               (a),
	.a_oe            (a_oe),
	.d_i             (d),
	.d_o             (cpu_d_o),
	.d_oe            (cpu_d_oe),
	.be_n            (be_n),
	.be_oe           (be_oe),
	.st              (st),
	.st_oe           (st_oe),
	.da_n            (da_n),
	.da_oe           (da_oe),
	.mrq_n           (mrq_n),
	.mrq_oe          (mrq_oe),
	.rw              (rw),
	.rw_oe           (rw_oe),
	.bcyst_n         (bcyst_n),
	.bcyst_oe        (bcyst_oe),
	.ready_n         (ready_n),
	.hldrq_n         (1'b1),
	.hldak_n         (hldak_n),
	.szrq_n          (szrq_n),
	.siz16b          (1'b0),
	.block           (block),
	.icheen_n        (1'b0),
	.intr            (intr),
	.intv_n          (intv_n),
	.nmi_n           (nmi_n || !AER_NMI),
	.adrserr_n       (adrserr_n),
	.ss_pause_req    (cpu_ss_pause_req),
	.ss_pause_ready  (cpu_ss_pause_ready),
	.ss_restore      (cpu_ss_restore),
	.ss_state_addr   (cpu_ss_state_addr),
	.ss_state_wdata  (cpu_ss_state_wdata),
	.ss_state_wren   (cpu_ss_state_wren),
	.ss_state_rdata  (cpu_ss_state_rdata),
	.ss_mem_active   (cpu_ss_mem_active),
	.ss_mem_addr     (cpu_ss_mem_addr),
	.ss_mem_rden     (cpu_ss_mem_rden),
	.ss_mem_wren     (cpu_ss_mem_wren),
	.ss_mem_wdata    (cpu_ss_mem_wdata),
	.ss_mem_rdata    (cpu_ss_mem_rdata),
	.dbg_pc          (cpu_pc),
	.dbg_psw         (cpu_psw),
	.dbg_retire      (dbg_retire),
	.dbg_retire_pc   (dbg_retire_pc),
	.dbg_retire_insn (cpu_insn),
	.dbg_halted      (cpu_halted),
	.dbg_illegal     (cpu_illegal)
);

// ------------------------------------------------------------------ glue

logic [31:0] ga_d_o;
logic        ga_d_oe;
logic        ram_oe_n;
logic [15:0] ld, ga_ld_o;
logic        ga_ld_oe, la1, lrd_n, lwr_n, bram_cs_n, cartsel_n, rom_ce_n_pin;
logic [15:0] iod, ga_iod_o;
logic        ga_iod_oe, iord_n, iowr_n;
logic  [5:1] ioa;
logic        cs_sb_n, cs_ra_n, cs_nt_n, cs_7a_n, cs_7b_n, cs_72_n;
logic        busy_7a_n, busy_7b_n, busy_72_n;
logic        irq_7a_n, irq_7b_n, irq_72_n;
logic        ss_ga, ga_ss_busy, king_ss_busy;

/* verilator lint_off UNUSEDSIGNAL */
logic        exb_cs_n, erst;
logic  [7:0] ga_dbg_isr;
logic  [3:0] ga_dbg_kind;
/* verilator lint_on UNUSEDSIGNAL */

upd65641 #(
	.SAVESTATE (SAVESTATE),
	.DEBUG     (DEBUG)
) u_glue (
	.clk        (clk),
	.reset      (reset),
	.ce_sys     (ce_sys),
	.ce_sck     (ce_sck),
	.ce_timer   (ce_timer),
	.ce_kport   (ce_kport),
	.a          (a[31:2]),
	.d_i        (d_ext),
	.d_o        (ga_d_o),
	.d_oe       (ga_d_oe),
	.be_n       (be_n),
	.mrq_n      (mrq_n),
	.rw         (rw),
	.bcyst_n    (bcyst_n),
	.ready_n    (ready_n),
	.szrq_n     (szrq_n),
	.intr       (intr),
	.intv_n     (intv_n),
	.nmi_n      (nmi_n),
	.adrserr_n  (adrserr_n),
	.ras_n      (mr_ras_n),
	.cas_n      (mr_cas_n),
	.ma         (mr_ma),
	.we_n       (mr_we_n),
	.oe_n       (ram_oe_n),
	.ld_i       (ld),
	.ld_o       (ga_ld_o),
	.ld_oe      (ga_ld_oe),
	.la1        (la1),
	.lrd_n      (lrd_n),
	.lwr_n      (lwr_n),
	.rom_ce_n   (rom_ce_n_pin),
	.bram_cs_n  (bram_cs_n),
	.cartsel_n  (cartsel_n),
	.exb_cs_n   (exb_cs_n),
	.xeb_n      (1'b1),
	.erst       (erst),
	.ioa        (ioa),
	.iod_i      (iod),
	.iod_o      (ga_iod_o),
	.iod_oe     (ga_iod_oe),
	.iord_n     (iord_n),
	.iowr_n     (iowr_n),
	.cs_sb_n    (cs_sb_n),
	.cs_ra_n    (cs_ra_n),
	.cs_nt_n    (cs_nt_n),
	.cs_7a_n    (cs_7a_n),
	.cs_7b_n    (cs_7b_n),
	.cs_72_n    (cs_72_n),
	.busy_7a_n  (busy_7a_n),
	.busy_7b_n  (busy_7b_n),
	.busy_72_n  (busy_72_n),
	.int7a_n    (irq_7a_n),
	.int72_n    (irq_72_n),
	.int7b_n    (irq_7b_n),
	.int73_n    (1'b1),
	.intex_n    (1'b1),
	.k0_latch_n (k0_latch_n),
	.k0_clk     (k0_clk),
	.k0_rw      (k0_rw),
	.k0_data_i  (k0_data_i),
	.k0_data_o  (k0_data_o),
	.k0_data_oe (k0_data_oe),
	.k1_latch_n (k1_latch_n),
	.k1_clk     (k1_clk),
	.k1_rw      (k1_rw),
	.k1_data_i  (k1_data_i),
	.k1_data_o  (k1_data_o),
	.k1_data_oe (k1_data_oe),
	.ss_shift   (ss_shift),
	.ss_si      (ss_si),
	.ss_so      (ss_ga),
	.ss_busy    (ga_ss_busy),
	.dbg_isr    (ga_dbg_isr),
	.dbg_kind   (ga_dbg_kind)
);

// Main RAM: its data pins sit on D31:0 and drive while OE is low.
wire ram_drive = !ram_oe_n;

// The DRAMs only latch D on writes, when the V810 drives it.
assign mr_d = cpu_d_o;

always_comb begin
	for (int i = 0; i < 4; i++) begin
		if (cpu_d_oe[i])
			d_ext[8*i +: 8] = cpu_d_o[8*i +: 8];
		else if (ram_drive)
			d_ext[8*i +: 8] = mr_q[8*i +: 8];
		else
			d_ext[8*i +: 8] = 8'hFF;
	end
end

assign d = ga_d_oe ? ga_d_o : d_ext;

// ------------------------------------------------------- local 16-bit bus

logic [15:0] rom_d;
logic        rom_d_oe;

mrom_msm538022c u_rom (
	.ce_n     (rom_ce_n_pin),
	.oe_n     (lrd_n),
	.a        ({a[19:2], la1}),
	.d_o      (rom_d),
	.d_oe     (rom_d_oe),
	.mem_ce_n (rom_ce_n),
	.mem_oe_n (rom_oe_n),
	.mem_a    (rom_a),
	.mem_q    (rom_q)
);

logic [7:0] bram_d, ss_bram_rdata;
logic       bram_d_oe;

sram_upd43256 #(
	.SAVESTATE (SAVESTATE)
) u_backup (
	.clk          (clk),
	.cs_n         (bram_cs_n),
	.oe_n         (lrd_n),
	.we_n         (lwr_n),
	.a            ({a[15:2], la1}),
	.d_i          (ld[7:0]),
	.d_o          (bram_d),
	.d_oe         (bram_d_oe),
	.ss_mem_sel   (ss_mem_sel && ss_mem_chip == 3'd3),
	.ss_mem_addr  (ss_mem_addr[14:0]),
	.ss_mem_we    (ss_mem_we),
	.ss_mem_wdata (ss_mem_wdata[7:0]),
	.ss_mem_rdata (ss_bram_rdata)
);

assign bram_we = !bram_cs_n && !lwr_n;

logic [7:0] bmp_d;
logic       bmp_d_oe;

fx_bmp u_fx_bmp (
	.inserted   (fx_inserted),
	.battery_ok (fx_battery_ok),
	.cartsel_n  (cartsel_n),
	.oe_n       (lrd_n),
	.we_n       (lwr_n),
	.a26        (a[26]),
	.a25        (a[25]),
	.a          ({a[17:2], la1}),
	.d_i        (ld[7:0]),
	.d_o        (bmp_d),
	.d_oe       (bmp_d_oe),
	.mem_cs_n   (fx_cs_n),
	.mem_rd_n   (fx_rd_n),
	.mem_wr_n   (fx_wr_n),
	.mem_a      (fx_a),
	.mem_d      (fx_d),
	.mem_q      (fx_q)
);

always_comb begin
	if (ga_ld_oe)
		ld = ga_ld_o;
	else if (rom_d_oe)
		ld = rom_d;
	else if (bram_d_oe)
		ld = {8'hFF, bram_d};
	else if (bmp_d_oe)
		ld = {8'hFF, bmp_d};
	else
		ld = 16'hFFFF;
end

// ------------------------------------------------------- HuC I/O bus

logic [15:0] nt_d, va_d, vb_d, king_d;
logic        nt_d_oe, va_d_oe, vb_d_oe, king_d_oe;

always_comb begin
	if (ga_iod_oe)
		iod = ga_iod_o;
	else if (nt_d_oe)
		iod = nt_d;
	else if (va_d_oe)
		iod = va_d;
	else if (vb_d_oe)
		iod = vb_d;
	else if (king_d_oe)
		iod = king_d;
	else
		iod = 16'hFFFF;
end

pcfx_pixel_pkg::pcfx_pixel_t rainbow_pix, king_pix;

// ------------------------------------------------------------------ video

localparam int NT_SS_AW = huc6261_pkg::SS_MEM_AW;

logic        ce_dck70, hsynca_n, vsync_n;
logic  [7:0] vda, vdb;
logic        spbg_a, spbg_b;
logic        ss_nt, ss_va, ss_vb;
logic [15:0] ss_nt_rdata, ss_va_rdata, ss_vb_rdata;

/* verilator lint_off UNUSEDSIGNAL */
logic        hsyncb_n, hsyncc_n, od_ev;
logic [10:0] nt_dbg_hc;
logic  [8:0] nt_dbg_vc;
/* verilator lint_on UNUSEDSIGNAL */

huc6261 #(
	.SAVESTATE (SAVESTATE),
	.DEBUG     (DEBUG)
) u_nt (
	.clk          (clk),
	.reset        (reset),
	.ce_osc       (ce_sys),
	.ce_master    (ce_sck),
	.cs_n         (cs_nt_n),
	.rd_n         (iord_n),
	.wr_n         (iowr_n),
	.a1           (ioa[2]),
	.d_i          (iod),
	.d_o          (nt_d),
	.d_oe         (nt_d_oe),
	.ce_dckkr     (ce_dot),
	.ce_dck70     (ce_dck70),
	.hsynca_n     (hsynca_n),
	.hsyncb_n     (hsyncb_n),
	.hsyncc_n     (hsyncc_n),
	.vsync_n      (vsync_n),
	.od_ev        (od_ev),
	.vda          (vda),
	.spbg_a       (spbg_a),
	.vdb          (vdb),
	.spbg_b       (spbg_b),
	.king         (king_pix),
	.rainbow      (rainbow_pix),
	.y            (y),
	.u            (u),
	.v            (v),
	.tv_hs        (tv_hs),
	.tv_vs        (tv_vs),
	.tv_hblank    (tv_hblank),
	.tv_vblank    (tv_vblank),
	.tv_ce_pix    (tv_ce_pix),
	.tv_field     (tv_field),
	.tv_interlace (tv_interlace),
	.ss_window    (ss_window),
	.ss_shift     (ss_shift),
	.ss_si        (ss_ga),
	.ss_so        (ss_nt),
	.ss_mem_sel   (ss_mem_sel && ss_mem_chip == 3'd0),
	.ss_mem_addr  (ss_mem_addr[NT_SS_AW-1:0]),
	.ss_mem_we    (ss_mem_we),
	.ss_mem_wdata (ss_mem_wdata),
	.ss_mem_rdata (ss_nt_rdata),
	.dbg_hc       (nt_dbg_hc),
	.dbg_vc       (nt_dbg_vc)
);

pcfx_vdc_slot #(
	.SAVESTATE (SAVESTATE),
	.DEBUG     (DEBUG)
) u_vdc_a (
	.clk          (clk),
	.reset        (reset),
	.ce_dck       (ce_dck70),
	.cs_n         (cs_7a_n),
	.rd_n         (iord_n),
	.wr_n         (iowr_n),
	.a1           (ioa[2]),
	.d_i          (iod),
	.d_o          (va_d),
	.d_oe         (va_d_oe),
	.busy_n       (busy_7a_n),
	.irq_n        (irq_7a_n),
	.hsyn_n       (hsynca_n),
	.vsyn_n       (vsync_n),
	.spbg         (spbg_a),
	.vd           (vda),
	.ss_shift     (ss_shift),
	.ss_si        (ss_nt),
	.ss_so        (ss_va),
	.ss_mem_sel   (ss_mem_sel && ss_mem_chip == 3'd1),
	.ss_mem_addr  (ss_mem_addr),
	.ss_mem_we    (ss_mem_we),
	.ss_mem_wdata (ss_mem_wdata),
	.ss_mem_rdata (ss_va_rdata)
);

pcfx_vdc_slot #(
	.SAVESTATE (SAVESTATE),
	.DEBUG     (DEBUG)
) u_vdc_b (
	.clk          (clk),
	.reset        (reset),
	.ce_dck       (ce_dck70),
	.cs_n         (cs_7b_n),
	.rd_n         (iord_n),
	.wr_n         (iowr_n),
	.a1           (ioa[2]),
	.d_i          (iod),
	.d_o          (vb_d),
	.d_oe         (vb_d_oe),
	.busy_n       (busy_7b_n),
	.irq_n        (irq_7b_n),
	.hsyn_n       (hsynca_n),
	.vsyn_n       (vsync_n),
	.spbg         (spbg_b),
	.vd           (vdb),
	.ss_shift     (ss_shift),
	.ss_si        (ss_va),
	.ss_so        (ss_vb),
	.ss_mem_sel   (ss_mem_sel && ss_mem_chip == 3'd2),
	.ss_mem_addr  (ss_mem_addr),
	.ss_mem_we    (ss_mem_we),
	.ss_mem_wdata (ss_mem_wdata),
	.ss_mem_rdata (ss_vb_rdata)
);

// ------------------------------------------------------------------ KING

// K-R bus, KING to HuC6230 and Rainbow.
logic  [7:0] kr_d;
logic        kr_cs0_n, kr_cs1_n, kr_rh_l, kr_wr_n;
logic        ss_sb, ss_ra, kr_rb_wr_n, reqr_n;

// CN401 to the CD board: every line low-true open drain with a pull-up,
// so each net is the AND of what both ends pull down.
logic  [7:0] scsi_db_n_o, cd_db_n_o, cn_db_n;
logic        scsi_db_n_oe, cd_db_n_oe;
logic  [3:0] scsi_o, scsi_oe;   // RST ACK SEL ATN
logic        cn_rst_n, cn_ack_n, cn_sel_n, cn_atn_n, cn_bsy_n, cn_req_n, cn_cd_n, cn_io_n, cn_msg_n;
logic        cd_bsy_n_o, cd_bsy_n_oe, cd_req_n_o, cd_req_n_oe, cd_cd_n_o, cd_cd_n_oe;
logic        cd_io_n_o, cd_io_n_oe, cd_msg_n_o, cd_msg_n_oe;
logic        cd_wfck, cd_scor, cd_sbso, sbck, ss_king;
logic [15:0] cdda_l, cdda_r;
logic        cdda_ce;
/* verilator lint_off UNUSEDSIGNAL */
logic  [8:0] king_dbg_kx, king_dbg_raster;
logic        cd_sel_n_o, cd_sel_n_oe, cd_atn_n_o, cd_atn_n_oe, cd_ack_n_o, cd_ack_n_oe;
logic        cd_rst_n_o, cd_rst_n_oe, cd_present_n;
logic [15:0] cd_dbg;
/* verilator lint_on UNUSEDSIGNAL */

assign cn_db_n  = (cd_db_n_oe ? cd_db_n_o : 8'hFF) & (scsi_db_n_oe ? scsi_db_n_o : 8'hFF);
assign cn_rst_n = !(scsi_oe[3] && !scsi_o[3]);
assign cn_ack_n = !(scsi_oe[2] && !scsi_o[2]);
assign cn_sel_n = !(scsi_oe[1] && !scsi_o[1]);
assign cn_atn_n = !(scsi_oe[0] && !scsi_o[0]);
assign cn_bsy_n = !(cd_bsy_n_oe && !cd_bsy_n_o);
assign cn_req_n = !(cd_req_n_oe && !cd_req_n_o);
assign cn_cd_n  = !(cd_cd_n_oe && !cd_cd_n_o);
assign cn_io_n  = !(cd_io_n_oe && !cd_io_n_o);
assign cn_msg_n = !(cd_msg_n_oe && !cd_msg_n_o);

huc6272 #(
	.SAVESTATE (SAVESTATE),
	.DEBUG     (DEBUG)
) u_king (
	.clk           (clk),
	.reset         (reset),
	.ce_osc        (ce_sys),
	.cs_n          (cs_72_n),
	.rd_n          (iord_n),
	.wr_n          (iowr_n),
	.a2            (ioa[2]),
	.a1            (ioa[1]),
	.d_i           (iod),
	.d_o           (king_d),
	.d_oe          (king_d_oe),
	.busy_n        (busy_72_n),
	.irq_n         (irq_72_n),
	.ce_dck        (ce_dot),
	.hsync_n       (hsyncb_n),
	.vsync_n       (vsync_n),
	.od_ev         (od_ev),
	.pix           (king_pix),
	.kra_req       (ka_req),
	.kra_we        (ka_we),
	.kra_row       (ka_row),
	.kra_col       (ka_col),
	.kra_be        (ka_be),
	.kra_d         (ka_d),
	.kra_q         (ka_q),
	.krb_req       (kb_req),
	.krb_we        (kb_we),
	.krb_row       (kb_row),
	.krb_col       (kb_col),
	.krb_be        (kb_be),
	.krb_d         (kb_d),
	.krb_q         (kb_q),
	.scsi_db_n_i   (cn_db_n),
	.scsi_db_n_o   (scsi_db_n_o),
	.scsi_db_n_oe  (scsi_db_n_oe),
	.scsi_rst_n_i  (cn_rst_n),
	.scsi_rst_n_o  (scsi_o[3]),
	.scsi_rst_n_oe (scsi_oe[3]),
	.scsi_ack_n_i  (cn_ack_n),
	.scsi_ack_n_o  (scsi_o[2]),
	.scsi_ack_n_oe (scsi_oe[2]),
	.scsi_sel_n_i  (cn_sel_n),
	.scsi_sel_n_o  (scsi_o[1]),
	.scsi_sel_n_oe (scsi_oe[1]),
	.scsi_atn_n_i  (cn_atn_n),
	.scsi_atn_n_o  (scsi_o[0]),
	.scsi_atn_n_oe (scsi_oe[0]),
	.scsi_bsy_n_i  (cn_bsy_n),
	.scsi_req_n_i  (cn_req_n),
	.scsi_cd_n_i   (cn_cd_n),
	.scsi_io_n_i   (cn_io_n),
	.scsi_msg_n_i  (cn_msg_n),
	.sbcf_i        (cd_wfck),
	.sbsc_i        (cd_scor),
	.sbdt_i        (cd_sbso),
	.sbck_o        (sbck),
	.kr_d          (kr_d),
	.kr_cs0_n      (kr_cs0_n),
	.kr_cs1_n      (kr_cs1_n),
	.kr_rh_l       (kr_rh_l),
	.kr_wr_n       (kr_wr_n),
	.kr_rb_wr_n    (kr_rb_wr_n),
	.reqr_n        (reqr_n),
	.ss_shift      (ss_shift),
	.ss_si         (ss_sb),
	.ss_so         (ss_king),
	.ss_busy       (king_ss_busy),
	.dbg_kx        (king_dbg_kx),
	.dbg_raster    (king_dbg_raster)
);

// ------------------------------------------------------------------ Rainbow

logic        rma_cs_n, rma_oe_n, rma_we_n, rmb_cs_n, rmb_oe_n, rmb_we_n;
logic [12:0] rma_a, rmb_a;
logic  [7:0] rma_d, rmb_d, rma_q, rmb_q, ss_ra_rdata;
/* verilator lint_off UNUSEDSIGNAL */
logic        rma_d_oe, rmb_d_oe, rma_q_oe, rmb_q_oe, ra_ss_busy, ra_invalid;
logic  [3:0] ra_dbg_jc, ra_dbg_dl;
/* verilator lint_on UNUSEDSIGNAL */

huc6271 #(
	.SAVESTATE (SAVESTATE),
	.DEBUG     (DEBUG)
) u_rainbow (
	.clk          (clk),
	.reset        (reset),
	.ce_osc       (ce_sys),
	.cs_n         (cs_ra_n),
	.wr_n         (iowr_n),
	.a            (ioa[4:1]),
	.d_i          (iod[7:0]),
	.ce_dck       (ce_dot),
	.hsync_n      (hsyncb_n),
	.kr_d         (kr_d),
	.kr_wr_n      (kr_rb_wr_n),
	.reqr_n       (reqr_n),
	.invalid      (ra_invalid),
	.rma_cs_n     (rma_cs_n),
	.rma_oe_n     (rma_oe_n),
	.rma_we_n     (rma_we_n),
	.rma_a        (rma_a),
	.rma_d_i      (rma_q),
	.rma_d_o      (rma_d),
	.rma_d_oe     (rma_d_oe),
	.rmb_cs_n     (rmb_cs_n),
	.rmb_oe_n     (rmb_oe_n),
	.rmb_we_n     (rmb_we_n),
	.rmb_a        (rmb_a),
	.rmb_d_i      (rmb_q),
	.rmb_d_o      (rmb_d),
	.rmb_d_oe     (rmb_d_oe),
	.pix          (rainbow_pix),
	.ss_shift     (ss_shift),
	.ss_si        (ss_king),
	.ss_so        (ss_ra),
	.ss_busy      (ra_ss_busy),
	.ss_mem_sel   (ss_mem_sel && ss_mem_chip == 3'd5),
	.ss_mem_addr  (ss_mem_addr[14:0]),
	.ss_mem_we    (ss_mem_we),
	.ss_mem_wdata (ss_mem_wdata[7:0]),
	.ss_mem_rdata (ss_ra_rdata),
	.dbg_jc       (ra_dbg_jc),
	.dbg_dl       (ra_dbg_dl)
);

assign ss_busy = ga_ss_busy || king_ss_busy || ra_ss_busy;

sram_2264 u_rram_a (
	.clk  (clk),
	.cs_n (rma_cs_n),
	.oe_n (rma_oe_n),
	.we_n (rma_we_n),
	.a    (rma_a),
	.d_i  (rma_d),
	.d_o  (rma_q),
	.d_oe (rma_q_oe)
);

sram_2264 u_rram_b (
	.clk  (clk),
	.cs_n (rmb_cs_n),
	.oe_n (rmb_oe_n),
	.we_n (rmb_we_n),
	.a    (rmb_a),
	.d_i  (rmb_d),
	.d_o  (rmb_q),
	.d_oe (rmb_q_oe)
);

pcfx_cd_drive #(
	.SAVESTATE (SAVESTATE),
	.DEBUG     (DEBUG)
) u_cd (
	.clk        (clk),
	.reset      (reset),
	.ce_sys     (ce_sys),
	.speed      (cd_speed),
	.db_n_i     (cn_db_n),
	.db_n_o     (cd_db_n_o),
	.db_n_oe    (cd_db_n_oe),
	.sel_n_i    (cn_sel_n),
	.sel_n_o    (cd_sel_n_o),
	.sel_n_oe   (cd_sel_n_oe),
	.atn_n_i    (cn_atn_n),
	.atn_n_o    (cd_atn_n_o),
	.atn_n_oe   (cd_atn_n_oe),
	.ack_n_i    (cn_ack_n),
	.ack_n_o    (cd_ack_n_o),
	.ack_n_oe   (cd_ack_n_oe),
	.exrst_n_i  (cn_rst_n),
	.exrst_n_o  (cd_rst_n_o),
	.exrst_n_oe (cd_rst_n_oe),
	.bsy_n_o    (cd_bsy_n_o),
	.bsy_n_oe   (cd_bsy_n_oe),
	.req_n_o    (cd_req_n_o),
	.req_n_oe   (cd_req_n_oe),
	.cd_n_o     (cd_cd_n_o),
	.cd_n_oe    (cd_cd_n_oe),
	.io_n_o     (cd_io_n_o),
	.io_n_oe    (cd_io_n_oe),
	.msg_n_o    (cd_msg_n_o),
	.msg_n_oe   (cd_msg_n_oe),
	.wfck       (cd_wfck),
	.scor       (cd_scor),
	.sbso       (cd_sbso),
	.exck       (sbck),
	.present_n  (cd_present_n),
	.cdda_ce    (cdda_ce),
	.cdda_l     (cdda_l),
	.cdda_r     (cdda_r),
	.blk_lba    (cd_blk_lba),
	.blk_base   (cd_blk_base),
	.blk_cnt    (cd_blk_cnt),
	.blk_rd     (cd_blk_rd),
	.blk_done   (cd_blk_done),
	.blk_err    (cd_blk_err),
	.buf_addr   (cd_buf_addr),
	.buf_dout   (cd_buf_dout),
	.disc       (cd_disc),
	.toc_busy   (cd_toc_busy),
	.toc_new    (cd_toc_new),
	.toc_first  (cd_toc_first),
	.toc_last   (cd_toc_last),
	.ss_shift   (ss_shift),
	.ss_si      (ss_ra),
	.ss_so      (ss_so),
	.ss_busy    (cd_ss_busy),
	.dbg        (cd_dbg)
);

// ------------------------------------------------------------------ audio

logic  [4:0] ss_sb_rdata;
/* verilator lint_off UNUSEDSIGNAL */
logic [19:0] sb_dbg_dac;
/* verilator lint_on UNUSEDSIGNAL */

huc6230 #(
	.SAVESTATE (SAVESTATE),
	.DEBUG     (DEBUG)
) u_soundbox (
	.clk          (clk),
	.reset        (reset),
	.ce_master    (ce_sck),
	.cs_n         (cs_sb_n),
	.wr_n         (iowr_n),
	.a            (ioa[5:1]),
	.d_i          (iod[7:0]),
	.hsync_n      (hsyncc_n),
	.kr_d         (kr_d),
	.kr_cs0_n     (kr_cs0_n),
	.kr_cs1_n     (kr_cs1_n),
	.kr_rh_l      (kr_rh_l),
	.kr_wr_n      (kr_wr_n),
	.line_l       (cdda_l),
	.line_r       (cdda_r),
	.line_ce      (cdda_ce),
	.enc_compat   (adpcm_enc_compat),
	.no_click     (adpcm_no_click),
	.slout        (audio_l),
	.srout        (audio_r),
	.ss_shift     (ss_shift),
	.ss_si        (ss_vb),
	.ss_so        (ss_sb),
	.ss_mem_sel   (ss_mem_sel && ss_mem_chip == 3'd4),
	.ss_mem_addr  (ss_mem_addr[7:0]),
	.ss_mem_we    (ss_mem_we),
	.ss_mem_wdata (ss_mem_wdata[4:0]),
	.ss_mem_rdata (ss_sb_rdata),
	.dbg_dac      (sb_dbg_dac)
);

always_comb begin
	case (ss_mem_chip)
		3'd0:    ss_mem_rdata = ss_nt_rdata;
		3'd1:    ss_mem_rdata = ss_va_rdata;
		3'd2:    ss_mem_rdata = ss_vb_rdata;
		3'd4:    ss_mem_rdata = {11'd0, ss_sb_rdata};
		3'd5:    ss_mem_rdata = {8'd0, ss_ra_rdata};
		default: ss_mem_rdata = {8'd0, ss_bram_rdata};
	endcase
end

// ------------------------------------------------------------------ debug

assign dbg_a       = DEBUG ? {a, 1'b0} : 32'd0;
assign dbg_d       = DEBUG ? d : 32'd0;
assign dbg_be_n    = DEBUG ? be_n : 4'd0;
assign dbg_mrq_n   = DEBUG ? mrq_n : 1'b0;
assign dbg_rw      = DEBUG ? rw : 1'b0;
assign dbg_bcyst_n = DEBUG ? bcyst_n : 1'b0;
assign dbg_ready_n = DEBUG ? ready_n : 1'b0;

endmodule
