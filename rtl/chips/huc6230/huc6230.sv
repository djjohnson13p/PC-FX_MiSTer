// HuC6230 SoundBox: PSG, two ADPCM decoders fed by KING over the K-R bus,
// four ADPCM volumes, two LINE VCAs, and the 4-source stereo mix.
//
//   CPU  -> cpu_if -> psg ------------------------------+
//                  -> R10 -> adpcm_tick -> adpcm_dec x2 -+-> mixer -> SLOUT/SROUT
//   K-R  -> kr_rx --------------------^                  |
//                  -> R11-R14 -> evol x4 ----------------+
//   CD-DA ---------> R15-R16 -> vca x2 ------------------+
//
// The chip is write-only: A4-A0 come from CPU A5-A1. Its 21.48 MHz clock
// is the board master clock (ce_master); fmaster = master / 3. The analog
// LINE inputs and mix outputs are signed 16-bit samples here.

module huc6230 #(
	parameter bit SAVESTATE = 1'b1,
	parameter bit DEBUG     = 1'b0
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_master,

	input  logic        cs_n,
	input  logic        wr_n,
	input  logic  [4:0] a,
	input  logic  [7:0] d_i,
	input  logic        hsync_n,

	input  logic  [7:0] kr_d,
	input  logic        kr_cs0_n,
	input  logic        kr_cs1_n,
	input  logic        kr_rh_l,
	input  logic        kr_wr_n,

	input  logic [15:0] line_l,       // VCA inputs (CD-DA)
	input  logic [15:0] line_r,
	input  logic        line_ce,      // a new L/R pair, 44.1 kHz

	input  logic        enc_compat,   // OSD: ADPCM as the faulty encoder assumed
	input  logic        no_click,     // OSD: fade ADPCM soft-reset jumps

	output logic [15:0] slout,
	output logic [15:0] srout,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so,
	input  logic        ss_mem_sel,   // PSG wave RAM, 6 x 32 at 32c + n
	input  logic  [7:0] ss_mem_addr,
	input  logic        ss_mem_we,
	input  logic  [4:0] ss_mem_wdata,
	output logic  [4:0] ss_mem_rdata,

	output logic [19:0] dbg_dac       // ADPCM DAC codes, {#2, #1}
);

logic        psg_we, hs_fall;
logic  [3:0] psg_a;
logic  [5:0] r10_now;
logic [3:0][5:0] avol;
logic [1:0][5:0] lvol;
logic        ss_cpu, ss_psg, ss_kr, ss_tick, ss_dec, ss_evol, ss_vca;

huc6230_cpu_if #(
	.SAVESTATE (SAVESTATE)
) u_cpu_if (
	.clk       (clk),
	.reset     (reset),
	.ce_master (ce_master),
	.cs_n      (cs_n),
	.wr_n      (wr_n),
	.a         (a),
	.d_i       (d_i),
	.hsync_n   (hsync_n),
	.psg_we    (psg_we),
	.psg_a     (psg_a),
	.hs_fall   (hs_fall),
	.r10_now   (r10_now),
	.avol      (avol),
	.lvol      (lvol),
	.ss_shift  (ss_shift),
	.ss_si     (ss_si),
	.ss_so     (ss_cpu)
);

logic  [2:0] v_ch;
logic signed [5:0] v_samp;
logic  [4:0] v_att_l, v_att_r;
logic        v_sil_l, v_sil_r;
logic        ce_fm;

huc6230_psg #(
	.SAVESTATE (SAVESTATE)
) u_psg (
	.clk          (clk),
	.reset        (reset),
	.ce_master    (ce_master),
	.ce_fm        (ce_fm),
	.we           (psg_we),
	.a            (psg_a),
	.d            (d_i),
	.v_ch         (v_ch),
	.v_samp       (v_samp),
	.v_att_l      (v_att_l),
	.v_att_r      (v_att_r),
	.v_sil_l      (v_sil_l),
	.v_sil_r      (v_sil_r),
	.ss_shift     (ss_shift),
	.ss_si        (ss_cpu),
	.ss_so        (ss_psg),
	.ss_mem_sel   (ss_mem_sel),
	.ss_mem_addr  (ss_mem_addr),
	.ss_mem_we    (ss_mem_we),
	.ss_mem_wdata (ss_mem_wdata),
	.ss_mem_rdata (ss_mem_rdata)
);

// Both channels in soft reset restart the transfer cycle lock.
wire unlock = r10_now[5] && r10_now[4];

logic        seen, tick, boundary, first;
logic  [1:0] pair_ok, nib;
logic [1:0][15:0] pair;

huc6230_kr_rx #(
	.SAVESTATE (SAVESTATE)
) u_kr_rx (
	.clk       (clk),
	.reset     (reset),
	.ce_master (ce_master),
	.kr_d      (kr_d),
	.kr_cs0_n  (kr_cs0_n),
	.kr_cs1_n  (kr_cs1_n),
	.kr_rh_l   (kr_rh_l),
	.kr_wr_n   (kr_wr_n),
	.boundary  (boundary),
	.unlock    (unlock),
	.seen      (seen),
	.pair_ok   (pair_ok),
	.pair      (pair),
	.ss_shift  (ss_shift),
	.ss_si     (ss_psg),
	.ss_so     (ss_kr)
);

huc6230_adpcm_tick #(
	.SAVESTATE (SAVESTATE)
) u_tick (
	.clk       (clk),
	.reset     (reset),
	.ce_master (ce_master),
	.hs_fall   (hs_fall),
	.div       (r10_now[1:0]),
	.unlock    (unlock),
	.seen      (seen),
	.tick      (tick),
	.boundary  (boundary),
	.nib       (nib),
	.first     (first),
	.ss_shift  (ss_shift),
	.ss_si     (ss_kr),
	.ss_so     (ss_tick)
);

logic [1:0][9:0] dac;

huc6230_adpcm_dec #(
	.SAVESTATE (SAVESTATE)
) u_dec (
	.clk       (clk),
	.reset     (reset),
	.tick      (tick),
	.boundary  (boundary),
	.nib       (nib),
	.first     (first),
	.r10       (r10_now),
	.pair_ok   (pair_ok),
	.pair      (pair),
	.enc_compat (enc_compat),
	.no_click  (no_click),
	.dac       (dac),
	.ss_shift  (ss_shift),
	.ss_si     (ss_tick),
	.ss_so     (ss_dec)
);

logic [3:0][15:0] gain;

huc6230_evol #(
	.SAVESTATE (SAVESTATE)
) u_evol (
	.clk       (clk),
	.reset     (reset),
	.tick      (tick),
	.avol      (avol),
	.gain      (gain),
	.ss_shift  (ss_shift),
	.ss_si     (ss_dec),
	.ss_so     (ss_evol)
);

logic [1:0][9:0]  level;
logic [1:0][15:0] line;

huc6230_vca #(
	.SAVESTATE (SAVESTATE)
) u_vca (
	.clk       (clk),
	.reset     (reset),
	.ce_master (ce_master),
	.lvol      (lvol),
	.cdda_l    (line_l),
	.cdda_r    (line_r),
	.cdda_ce   (line_ce),
	.level     (level),
	.line      (line),
	.ss_shift  (ss_shift),
	.ss_si     (ss_evol),
	.ss_so     (ss_vca)
);

huc6230_mixer #(
	.SAVESTATE (SAVESTATE)
) u_mixer (
	.clk      (clk),
	.reset    (reset),
	.ce_fm    (ce_fm),
	.v_ch     (v_ch),
	.v_samp   (v_samp),
	.v_att_l  (v_att_l),
	.v_att_r  (v_att_r),
	.v_sil_l  (v_sil_l),
	.v_sil_r  (v_sil_r),
	.dac      (dac),
	.gain     (gain),
	.level    (level),
	.line     (line),
	.slout    (slout),
	.srout    (srout),
	.ss_shift (ss_shift),
	.ss_si    (ss_vca),
	.ss_so    (ss_so)
);

assign dbg_dac = DEBUG ? {dac[1], dac[0]} : 20'd0;

endmodule
