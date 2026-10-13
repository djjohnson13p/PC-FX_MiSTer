// PC-FX CD board (PWA-3387) as one drive behind the 40-pin CN401: SCSI
// target, mechanism, subcode port and CD-DA, fed with disc blocks and the
// TOC by the HPS.
//
//   CN401 --- scsi_target ---- cd_mech ---- ring / TOC in the slot buffer
//                 |              |   \_____ HPS block requests
//                 |              +-- cd_subcode_gen -- WFCK SCOR SBSO / EXCK
//                 +-- buffer reads (one port, four readers) ----+
//
// Every CN401 line is low-true and open drain; each is split into _i, _o
// and _oe, _o being 0 wherever _oe can be 1. The analogue CD-DA pair is a
// signed 16-bit stereo sample with a 44.1 kHz enable. The board's own
// 16.9344 MHz crystal is an enable made from clk_sys.

module pcfx_cd_drive #(
	parameter bit SAVESTATE = 1'b1,
	parameter bit DEBUG     = 1'b0,
	parameter int XTAL_NUM  = 1232,     // 16.9344 MHz / 42.954545 MHz = 1232 / 3125
	parameter int XTAL_DEN  = 3125
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_sys,         // board running
	input  logic  [1:0] speed,          // 0 the real drive; 1, 2 data at 4x, 8x and quicker seeks

	// CN401
	input  logic  [7:0] db_n_i,
	output logic  [7:0] db_n_o,
	output logic        db_n_oe,
	input  logic        sel_n_i,
	output logic        sel_n_o,
	output logic        sel_n_oe,
	input  logic        atn_n_i,
	output logic        atn_n_o,
	output logic        atn_n_oe,
	input  logic        ack_n_i,
	output logic        ack_n_o,
	output logic        ack_n_oe,
	input  logic        exrst_n_i,
	output logic        exrst_n_o,
	output logic        exrst_n_oe,
	output logic        bsy_n_o,
	output logic        bsy_n_oe,
	output logic        req_n_o,
	output logic        req_n_oe,
	output logic        cd_n_o,
	output logic        cd_n_oe,
	output logic        io_n_o,
	output logic        io_n_oe,
	output logic        msg_n_o,
	output logic        msg_n_oe,
	output logic        wfck,
	output logic        scor,
	output logic        sbso,
	input  logic        exck,
	output logic        present_n,

	output logic        cdda_ce,
	output logic [15:0] cdda_l,
	output logic [15:0] cdda_r,

	// HPS feed
	output logic [19:0] blk_lba,
	output logic [15:0] blk_base,
	output logic  [2:0] blk_cnt,        // blocks in the request, less one
	output logic        blk_rd,
	input  logic        blk_done,
	input  logic        blk_err,
	output logic [15:0] buf_addr,
	input  logic  [7:0] buf_dout,
	input  logic        disc,
	input  logic        toc_busy,
	input  logic        toc_new,
	input  logic  [6:0] toc_first,
	input  logic  [6:0] toc_last,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so,
	output logic        ss_busy,        // after a load: blocks still on their way

	output logic [15:0] dbg
);

import pcfx_cd_pkg::*;

// ---- the crystal and its slow ticks ----

clk_t ck;
logic ce_x, ce_t, ce_fs, ce_d;
logic [12:0] acc_n;

always_comb acc_n = {1'b0, ck.acc} + 13'(XTAL_NUM);

always_ff @(posedge clk) begin
	ce_x  <= 1'b0;
	ce_t  <= 1'b0;
	ce_fs <= 1'b0;
	ce_d  <= 1'b0;
	if (reset) begin
		ck <= '0;
	end else if (SAVESTATE && ss_shift) begin
		ck <= clk_t'({ck[CLK_BITS-2:0], ss_si});
	end else if (ce_sys) begin
		if (acc_n >= 13'(XTAL_DEN)) begin
			ck.acc <= 12'(acc_n - 13'(XTAL_DEN));
			ce_x   <= 1'b1;
			ck.div <= ck.div == 8'd191 ? 8'd0 : ck.div + 8'd1;
			// the data tick: ce_t, or two or four times as often
			ce_d   <= ck.div == 8'd191 || (speed != 2'd0 && ck.div == 8'd95) ||
			          (speed[1] && (ck.div == 8'd47 || ck.div == 8'd143));
			if (ck.div == 8'd191) begin
				ce_t  <= 1'b1;
				ck.fs <= !ck.fs;
				ce_fs <= ck.fs;
			end
		end else begin
			ck.acc <= acc_n[11:0];
		end
	end
end

// ---- pins in ----

logic       sel, atn, ack, rst, exck_r;
logic [7:0] db;
always_ff @(posedge clk) begin
	sel    <= !sel_n_i;
	atn    <= !atn_n_i;
	ack    <= !ack_n_i;
	rst    <= !exrst_n_i;
	db     <= ~db_n_i;
	exck_r <= exck;
end

// ---- buffer port: subcode, samples, TOC, target ----

logic [3:0]  rq, gnt;
logic [15:0] ra [4];
logic [1:0]  ag;
logic [1:0]  astep;

always_ff @(posedge clk) begin
	if (reset || (SAVESTATE && ss_shift)) begin
		astep <= 2'd0;
	end else begin
		case (astep)
		2'd0: if (|rq && !toc_busy) begin
			ag       <= rq[0] ? 2'd0 : rq[1] ? 2'd1 : rq[2] ? 2'd2 : 2'd3;
			buf_addr <= rq[0] ? ra[0] : rq[1] ? ra[1] : rq[2] ? ra[2] : ra[3];
			astep    <= 2'd1;
		end
		2'd1: astep <= 2'd2;
		default: astep <= 2'd0;
		endcase
	end
end

// a reader that let go meanwhile does not get the byte
always_comb begin
	gnt = 4'd0;
	gnt[ag] = astep == 2'd2 && rq[ag] && ra[ag] == buf_addr;
end

// ---- blocks ----

logic        bus_rst;
logic        m_go, m_done, m_ready, m_nodisc, m_change, m_busy_end;
logic  [3:0] m_op;
logic [18:0] m_lba, m_end, m_leadout;
logic  [2:0] m_mode;
logic  [3:0] m_err;
logic  [1:0] m_play;
logic        p_ready, p_eov, p_slow, p_next;
logic [15:0] p_base;
logic [79:0] q_now;
logic        q_hold, q_busy;
logic        f_start, spun, rw_ok;
logic  [6:0] f_idx;
logic [15:0] rw_base;
logic  [1:0] port0_sel, port1_sel;
logic        bsy, req, cd, io, msg, db_oe;
logic  [7:0] db_o;
logic        ss_a, ss_b, ss_c;

cd_scsi_target #(.SAVESTATE(SAVESTATE)) scsi (
	.clk, .reset, .ce(ce_sys), .ce_x, .ce_t, .fast(speed != 2'd0),
	.sel, .atn, .ack, .rst, .db_i(db),
	.bsy, .req, .cd, .io, .msg, .db_o, .db_oe, .bus_rst,
	.m_go, .m_op, .m_lba, .m_end, .m_mode, .m_done, .m_err, .m_ready, .m_nodisc, .m_change,
	.m_play, .m_busy_end, .m_leadout,
	.p_ready, .p_eov, .p_slow, .p_base, .p_next,
	.q_now, .q_hold, .q_busy, .toc_first, .toc_last,
	.mr_rq(rq[3]), .mr_addr(ra[3]), .mr_ack(gnt[3]), .mr_q(buf_dout),
	.port0_sel, .port1_sel,
	.ss_shift, .ss_si(ck[CLK_BITS-1]), .ss_so(ss_a)
);

cd_mech #(.SAVESTATE(SAVESTATE)) mech (
	.clk, .reset, .ce(ce_sys), .ce_t, .ce_d, .ce_fs,
	.bus_rst,
	.disc, .toc_busy, .toc_new, .toc_first, .toc_last,
	.go(m_go), .op(m_op), .lba(m_lba), .lend(m_end), .pmode_i(m_mode),
	.done(m_done), .err(m_err), .ready(m_ready), .no_disc(m_nodisc), .changed(m_change),
	.play_st(m_play), .busy_end(m_busy_end), .leadout(m_leadout),
	.p_ready, .p_eov, .p_slow, .p_base, .p_next,
	.q_now, .q_hold, .q_busy,
	.f_start, .f_idx, .wfck, .spun, .rw_ok, .rw_base,
	.port0_sel, .port1_sel, .cdda_ce, .cdda_l, .cdda_r,
	.sm_rq(rq[1]), .sm_addr(ra[1]), .sm_ack(gnt[1]),
	.tm_rq(rq[2]), .tm_addr(ra[2]), .tm_ack(gnt[2]), .m_q(buf_dout),
	.blk_lba, .blk_base, .blk_cnt, .blk_rd, .blk_done, .blk_err,
	.ss_shift, .ss_si(ss_a), .ss_so(ss_b), .ss_busy
);

cd_subcode_gen #(.SAVESTATE(SAVESTATE)) sub (
	.clk, .reset, .ce(ce_sys),
	.spun, .f_start, .f_idx, .q_now, .rw_ok, .rw_base,
	.exck(exck_r), .scor, .sbso,
	.rw_rq(rq[0]), .rw_addr(ra[0]), .rw_ack(gnt[0]), .rw_q(buf_dout),
	.ss_shift, .ss_si(ss_b), .ss_so(ss_c)
);

assign ss_so = SAVESTATE ? ss_c : ss_si;

// ---- pins out ----

assign db_n_o     = ~db_o;
assign db_n_oe    = db_oe;
assign bsy_n_o    = 1'b0;
assign bsy_n_oe   = bsy;
assign req_n_o    = 1'b0;
assign req_n_oe   = req;
assign cd_n_o     = 1'b0;
assign cd_n_oe    = cd;
assign io_n_o     = 1'b0;
assign io_n_oe    = io;
assign msg_n_o    = 1'b0;
assign msg_n_oe   = msg;
// the drive never drives these
assign sel_n_o    = 1'b0;
assign sel_n_oe   = 1'b0;
assign atn_n_o    = 1'b0;
assign atn_n_oe   = 1'b0;
assign ack_n_o    = 1'b0;
assign ack_n_oe   = 1'b0;
assign exrst_n_o  = 1'b0;
assign exrst_n_oe = 1'b0;
assign present_n  = 1'b0;

assign dbg = DEBUG ? {m_play, p_ready, m_ready, m_op, bsy, req, cd, io, msg, spun, rw_ok, m_go} : 16'd0;

endmodule
