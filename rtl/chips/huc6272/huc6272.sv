// HuC6272 KING: KRAM controller, BG engine, SCSI host and the transfers to
// the HuC6271 and HuC6230 (C6272_1-3).
//
//   M-bus --> cpuif --+--> cpu_kram --\
//                     +--> bg -------+--> kbus_arb A --> kram_ctl A --> KRAM A
//                     +--> scsi -----+--> kbus_arb B --> kram_ctl B --> KRAM B
//                     +--> adpcm_xfer -/      (one access per bank per dot)
//                     +--> rb_xfer --/
//                     +--> irq --> -IRQ
//   DCKKR, -HSYNC, -VSYNC --> timing (kx, raster, phases)
//   bg --> pixel bus to the HuC6261
//
// Board: CS from the glue's 0x600 select, A2/A1 from I/O address bits 2/1,
// D15:0; a 32-bit access arrives as two 16-bit cycles. BUSY extends only
// KING's own bus cycle. ce_osc is OSC (clk_sys), ce_dck the 6261's DCKKR;
// KRAM data from the sockets arrives 8 clk_sys after the dot's request.
//
// KRAM pins: each bank's DRAM cycle for a dot is given to its socket at
// the dot start as {row, column, write, byte lanes, data}.
//
// Savestate: SS_BITS of chain, no RAM. Capture on a vertical-blank line:
// the line pipes, BAT latches, slot pipeline and the H-register line copy
// are idle there and rebuilt by the next -HSYNC.

module huc6272 #(
	parameter bit SAVESTATE = 1'b1,
	parameter bit DEBUG     = 1'b0
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_osc,

	input  logic        cs_n,
	input  logic        rd_n,
	input  logic        wr_n,
	input  logic        a2,
	input  logic        a1,
	input  logic [15:0] d_i,
	output logic [15:0] d_o,
	output logic        d_oe,
	output logic        busy_n,
	output logic        irq_n,

	input  logic        ce_dck,       // DCKKR from the HuC6261
	input  logic        hsync_n,
	input  logic        vsync_n,
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic        od_ev,
	/* verilator lint_on UNUSEDSIGNAL */

	output pcfx_pixel_pkg::pcfx_pixel_t pix,

	output logic        kra_req,
	output logic        kra_we,
	output logic  [8:0] kra_row,
	output logic  [8:0] kra_col,
	output logic  [1:0] kra_be,
	output logic [15:0] kra_d,
	input  logic [15:0] kra_q,
	output logic        krb_req,
	output logic        krb_we,
	output logic  [8:0] krb_row,
	output logic  [8:0] krb_col,
	output logic  [1:0] krb_be,
	output logic [15:0] krb_d,
	input  logic [15:0] krb_q,

	input  logic  [7:0] scsi_db_n_i,
	output logic  [7:0] scsi_db_n_o,
	output logic        scsi_db_n_oe,
	input  logic        scsi_rst_n_i,
	output logic        scsi_rst_n_o,
	output logic        scsi_rst_n_oe,
	input  logic        scsi_ack_n_i,
	output logic        scsi_ack_n_o,
	output logic        scsi_ack_n_oe,
	input  logic        scsi_sel_n_i,
	output logic        scsi_sel_n_o,
	output logic        scsi_sel_n_oe,
	input  logic        scsi_atn_n_i,
	output logic        scsi_atn_n_o,
	output logic        scsi_atn_n_oe,
	input  logic        scsi_bsy_n_i,
	input  logic        scsi_req_n_i,
	input  logic        scsi_cd_n_i,
	input  logic        scsi_io_n_i,
	input  logic        scsi_msg_n_i,
	input  logic        sbcf_i,
	input  logic        sbsc_i,
	input  logic        sbdt_i,
	output logic        sbck_o,

	output logic  [7:0] kr_d,
	output logic        kr_cs0_n,
	output logic        kr_cs1_n,
	output logic        kr_rh_l,
	output logic        kr_wr_n,
	output logic        kr_rb_wr_n,
	output logic        rb_stream_start,
	input  logic        reqr_n,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so,
	output logic        ss_busy,      // a K-R transfer is mid-word: do not freeze

	output logic  [8:0] dbg_kx,
	output logic  [8:0] dbg_raster
);

import huc6272_pkg::*;

logic       rst_int, init, disp_line, hs_ev, vs_ev;
logic [8:0] kx, nkx, raster;
logic [3:0] ph;
logic       ss_1, ss_2, ss_3, ss_4, ss_5, ss_6, ss_7;

king_timing #(.SAVESTATE(SAVESTATE)) timing (
	.clk, .reset, .ce_osc, .ce_dck, .hsync_n, .vsync_n,
	.rst_int, .init, .kx, .nkx, .ph, .raster, .disp_line, .hs_ev, .vs_ev,
	.ss_shift, .ss_si,
	.ss_so (ss_1)
);

// ------------------------------------------------------------ CPU side

acc_t        acc;
logic  [6:0] ar;
logic        pend_wr, pend_rd, hold;
logic [15:0] rd_kram, rd_scsi, rd_adpcm;
logic  [7:0] bus_st, subcode;
logic  [4:0] flags;
logic        busy_bit;
logic [15:8] st_hi;

always_comb begin
	st_hi = 8'd0;
	st_hi[ST_BUSY]   = busy_bit;
	st_hi[ST_SOUND]  = flags[0];
	st_hi[ST_LUSTER] = flags[1];
	st_hi[ST_SBCODE] = flags[2];
	st_hi[ST_SCDMA]  = flags[3];
	st_hi[ST_SCSIBK] = flags[4];
end

king_cpuif #(.SAVESTATE(SAVESTATE)) cpuif (
	.clk,
	.reset (rst_int),
	.ce_osc,
	.cs_n, .rd_n, .wr_n, .a2, .a1, .d_i, .d_o, .d_oe, .busy_n,
	.ar, .pend_wr, .pend_rd, .acc, .hold,
	.rdata  (rd_kram | rd_scsi | rd_adpcm),
	.st_hi,
	.st_bus ({subcode, bus_st}),
	.ss_shift,
	.ss_si (ss_1),
	.ss_so (ss_2)
);

logic  [3:0] pages;
logic        mode4m, cpu_bank, gnt_cpu_a, gnt_cpu_b;
kreq_t       cpu_req;

king_cpu_kram #(.SAVESTATE(SAVESTATE)) cpu_kram (
	.clk,
	.reset (rst_int),
	.ce_osc, .ce_dck, .init,
	.acc, .pend_wr, .pend_rd, .ar, .hold,
	.rdata (rd_kram),
	.busy_bit, .pages, .mode4m,
	.req (cpu_req),
	.req_bank (cpu_bank),
	.gnt (gnt_cpu_a || gnt_cpu_b),
	.ka_q (kra_q),
	.kb_q (krb_q),
	.ss_shift,
	.ss_si (ss_2),
	.ss_so (ss_3)
);

logic        rint, set_sound, set_sbcode, set_scdma, set_scsibk;
logic  [8:0] mon;

king_irq #(.SAVESTATE(SAVESTATE)) irq (
	.clk,
	.reset (rst_int),
	.ce_osc, .acc,
	.raster_ev (hs_ev || vs_ev),
	.raster, .mon, .rint,
	.set_sound, .set_sbcode, .set_scdma, .set_scsibk,
	.flags, .irq_n,
	.ss_shift,
	.ss_si (ss_3),
	.ss_so (ss_4)
);

// ------------------------------------------------------------ BG

kreq_t bg_a, bg_b;
logic  gnt_bg_a, gnt_bg_b;

king_bg #(.SAVESTATE(SAVESTATE)) bg (
	.clk,
	.reset (rst_int),
	.ce_osc, .ce_dck, .ph, .kx, .nkx, .raster, .disp_line, .hs_ev,
	.acc,
	.page (pages[1]),
	.req_a (bg_a), .req_b (bg_b),
	.gnt_a (gnt_bg_a), .gnt_b (gnt_bg_b),
	.ka_q (kra_q), .kb_q (krb_q),
	.pix,
	.ss_shift,
	.ss_si (ss_4),
	.ss_so (ss_5)
);

// ------------------------------------------------------------ SCSI, transfers

kreq_t scsi_req;
logic  scsi_bank, gnt_scsi_a, gnt_scsi_b;

king_scsi #(.SAVESTATE(SAVESTATE)) scsi (
	.clk,
	.reset (rst_int),
	.ce_osc, .ce_dck, .acc, .ar,
	.page (pages[0]),
	.rdata (rd_scsi),
	.bus_st, .subcode,
	.scsibk (flags[4]),
	.req (scsi_req), .req_bank (scsi_bank),
	.gnt (gnt_scsi_a || gnt_scsi_b),
	.db_n_i  (scsi_db_n_i),  .db_n_o  (scsi_db_n_o),  .db_n_oe  (scsi_db_n_oe),
	.rst_n_i (scsi_rst_n_i), .rst_n_o (scsi_rst_n_o), .rst_n_oe (scsi_rst_n_oe),
	.ack_n_i (scsi_ack_n_i), .ack_n_o (scsi_ack_n_o), .ack_n_oe (scsi_ack_n_oe),
	.sel_n_i (scsi_sel_n_i), .sel_n_o (scsi_sel_n_o), .sel_n_oe (scsi_sel_n_oe),
	.atn_n_i (scsi_atn_n_i), .atn_n_o (scsi_atn_n_o), .atn_n_oe (scsi_atn_n_oe),
	.bsy_n_i (scsi_bsy_n_i), .req_n_i (scsi_req_n_i), .cd_n_i (scsi_cd_n_i),
	.io_n_i  (scsi_io_n_i),  .msg_n_i (scsi_msg_n_i),
	.sbcf_i, .sbsc_i, .sbdt_i, .sbck_o,
	.set_scsibk, .set_scdma, .set_sbcode,
	.ss_shift,
	.ss_si (ss_5),
	.ss_so (ss_6)
);

kreq_t      adpcm_req, rb_req;
logic       adpcm_bank, rb_bank, gnt_adpcm_a, gnt_adpcm_b, gnt_rb_a, gnt_rb_b;
logic [7:0] kr_d_adpcm, kr_d_rb;
logic       adpcm_kr_busy;      // Rainbow holds its strobes while high

king_adpcm_xfer #(.SAVESTATE(SAVESTATE)) adpcm (
	.clk,
	.reset (rst_int),
	.ce_osc, .ce_dck, .nkx, .ph, .hs_ev, .acc,
	.page (pages[2]),
	.rdata (rd_adpcm),
	.req (adpcm_req),
	.req_bank (adpcm_bank),
	.gnt (gnt_adpcm_a || gnt_adpcm_b),
	.ka_q (kra_q),
	.kb_q (krb_q),
	.kr_d (kr_d_adpcm),
	.kr_cs0_n, .kr_cs1_n, .kr_rh_l, .kr_wr_n,
	.kr_busy (adpcm_kr_busy),
	.set_sound,
	.ss_shift,
	.ss_si (ss_6),
	.ss_so (ss_7)
);

logic rb_ss_busy;                // a K-R byte or word in flight

king_rb_xfer #(.SAVESTATE(SAVESTATE)) rb (
	.clk,
	.reset (rst_int),
	.ce_osc, .ce_dck, .ph, .hs_ev, .vs_ev, .raster, .acc,
	.page (pages[3]),
	.req (rb_req),
	.req_bank (rb_bank),
	.gnt (gnt_rb_a || gnt_rb_b),
	.ka_q (kra_q),
	.kb_q (krb_q),
	.reqr_n,
	.kr_busy (adpcm_kr_busy),
	.kr_d (kr_d_rb),
	.kr_rb_wr_n,
	.stream_start (rb_stream_start),
	.rint, .mon,
	.ss_shift,
	.ss_si (ss_7),
	.ss_so,
	.ss_busy (rb_ss_busy)
);

// The K-R bus carries whichever transfer is strobing.
assign kr_d = !kr_rb_wr_n ? kr_d_rb : kr_d_adpcm;

// ------------------------------------------------------------ K-bus

wire display = disp_line && nkx < KX_SEQ_END;
wire refresh = init || (nkx >= KX_REF && nkx < KX_REF_END);

kreq_t sel_a, sel_b, cpu_a, cpu_b, adpcm_a, adpcm_b, rb_a, rb_b, scsi_a, scsi_b;

always_comb begin
	cpu_a   = cpu_req;   cpu_a.v   = cpu_req.v && !cpu_bank;
	cpu_b   = cpu_req;   cpu_b.v   = cpu_req.v && cpu_bank;
	adpcm_a = adpcm_req; adpcm_a.v = adpcm_req.v && !adpcm_bank;
	adpcm_b = adpcm_req; adpcm_b.v = adpcm_req.v && adpcm_bank;
	rb_a    = rb_req;    rb_a.v    = rb_req.v && !rb_bank;
	rb_b    = rb_req;    rb_b.v    = rb_req.v && rb_bank;
	scsi_a  = scsi_req;  scsi_a.v  = scsi_req.v && !scsi_bank;
	scsi_b  = scsi_req;  scsi_b.v  = scsi_req.v && scsi_bank;
end

king_kbus_arb arb_a (
	.display, .refresh,
	.bg (bg_a), .adpcm (adpcm_a), .rb (rb_a), .scsi (scsi_a), .cpu (cpu_a),
	.sel (sel_a),
	.gnt_bg (gnt_bg_a), .gnt_adpcm (gnt_adpcm_a), .gnt_rb (gnt_rb_a),
	.gnt_scsi (gnt_scsi_a), .gnt_cpu (gnt_cpu_a)
);

king_kbus_arb arb_b (
	.display, .refresh,
	.bg (bg_b), .adpcm (adpcm_b), .rb (rb_b), .scsi (scsi_b), .cpu (cpu_b),
	.sel (sel_b),
	.gnt_bg (gnt_bg_b), .gnt_adpcm (gnt_adpcm_b), .gnt_rb (gnt_rb_b),
	.gnt_scsi (gnt_scsi_b), .gnt_cpu (gnt_cpu_b)
);

king_kram_ctl kram_a (
	.clk,
	.reset (rst_int),
	.ce_dck (ce_osc && ce_dck),
	.mode4m,
	.sel (sel_a),
	.kram_req (kra_req), .kram_we (kra_we), .kram_row (kra_row),
	.kram_col (kra_col), .kram_be (kra_be), .kram_d (kra_d)
);

king_kram_ctl kram_b (
	.clk,
	.reset (rst_int),
	.ce_dck (ce_osc && ce_dck),
	.mode4m,
	.sel (sel_b),
	.kram_req (krb_req), .kram_we (krb_we), .kram_row (krb_row),
	.kram_col (krb_col), .kram_be (krb_be), .kram_d (krb_d)
);

assign dbg_kx     = DEBUG ? kx : 9'd0;
assign dbg_raster = DEBUG ? raster : 9'd0;

// SCSI, the CPU port and BG keep all state in the chain; only a K-R word
// in flight must not be split.
assign ss_busy = adpcm_kr_busy || rb_ss_busy;

endmodule
