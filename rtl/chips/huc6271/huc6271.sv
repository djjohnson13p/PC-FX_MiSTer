// HuC6271 Rainbow: real-time decoder of 16-line strips (DCT or run length)
// from KING into two R-RAMs, one shown while the other fills (C6271).
//
//   KR7:0 --> kr_rx --> parser --> FIFO --> vlc --> dequant --> idct --\
//             -REQR <--/               \--> null / palette runs -------+--> mb_asm
//   mb_asm --> R-RAM (decode) ... R-RAM (display) --> out (fts) --> pixel bus
//   M-bus --> cpu_if (R00-R05)
//
// Strip timing: at the 16th -HSYNC fall after a header the decoders start
// over; a complete strip swaps the R-RAMs and shows from the next line for
// 16 lines, an incomplete one leaves the picture transparent.
//
// Board: A3:A0 = CPU A4:A1, D7:0, write only. -HSYNC is the 6261's -HSYNCB
// and DCKKR its KING/Rainbow dot clock; ce_osc is OSC (clk_sys).
//
// Savestate: SS_BITS of chain plus R-RAM A, B and the Q tables through
// ss_mem (address bits 14:13: 0 A, 1 B, 2 Q tables). ss_busy is high while
// a strip is being received or decoded; the bit reader, coefficient and
// transpose RAMs, IDCT pipe, FIFO and run counters are not saved, so the
// capture waits for it to drop (rasters 262-5 with the usual start raster
// of 6).

module huc6271 #(
	parameter bit SAVESTATE = 1'b1,
	parameter bit DEBUG     = 1'b0
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_osc,

	input  logic        cs_n,
	input  logic        wr_n,
	input  logic  [3:0] a,
	input  logic  [7:0] d_i,

	input  logic        ce_dck,
	input  logic        hsync_n,

	input  logic  [7:0] kr_d,
	input  logic        kr_wr_n,
	output logic        reqr_n,
	output logic        invalid,      // to KING: the 16 lines an incomplete strip leaves transparent

	output logic        rma_cs_n,
	output logic        rma_oe_n,
	output logic        rma_we_n,
	output logic [12:0] rma_a,
	input  logic  [7:0] rma_d_i,
	output logic  [7:0] rma_d_o,
	output logic        rma_d_oe,
	output logic        rmb_cs_n,
	output logic        rmb_oe_n,
	output logic        rmb_we_n,
	output logic [12:0] rmb_a,
	input  logic  [7:0] rmb_d_i,
	output logic  [7:0] rmb_d_o,
	output logic        rmb_d_oe,

	output pcfx_pixel_pkg::pcfx_pixel_t pix,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so,
	output logic        ss_busy,
	input  logic        ss_mem_sel,
	input  logic [14:0] ss_mem_addr,
	input  logic        ss_mem_we,
	input  logic  [7:0] ss_mem_wdata,
	output logic  [7:0] ss_mem_rdata,

	output logic  [3:0] dbg_jc,
	output logic  [3:0] dbg_dl
);

import huc6271_pkg::*;

/* verilator lint_off UNUSEDSIGNAL */
regs_t   r;         // HSR is kept but not applied (00 with the HuC6261)
/* verilator lint_on UNUSEDSIGNAL */
strip_t  st;
pstate_t ps;
logic    ss_1, ss_2;

wire ce      = ce_osc;
wire hs_fall = ce_dck && st.hs_q && !hsync_n;
wire [8:0] nkx = st.kx == 9'd340 ? 9'd0 : st.kx + 9'd1;

huc6271_cpu_if #(.SAVESTATE(SAVESTATE)) cpu_if (
	.clk, .reset, .ce,
	.cs_n, .wr_n, .a, .d_i,
	.hs_fall,
	.r,
	.ss_shift, .ss_si,
	.ss_so (ss_1)
);

// ------------------------------------------------------------ input

logic       rx_v, hdr, push, empty, pop;
logic [7:0] rx_d, kind, push_d, fq;
logic       judge, init;

assign init = judge || !r.run;

// The header search takes a byte a dot, so while hunting the chip asks
// for KING's word only every other dot; strip data comes at bus speed.
huc6271_kr_rx kr_rx (
	.clk, .reset, .ce,
	.flush (init),
	.kr_d, .kr_wr_n, .rx_v, .rx_d,
	.want (r.run && ps != P_DONE && (ps == P_DATA || st.kx[0])),
	.reqr_n,
	.push, .push_d, .empty, .pop,
	.q (fq)
);

huc6271_parser #(.SAVESTATE(SAVESTATE)) parser (
	.clk, .reset, .ce, .init,
	.run (r.run),
	.rx_v, .rx_d,
	.hdr, .kind, .push, .push_d, .ps,
	.ss_shift,
	.ss_si (ss_2),
	.ss_so
);

// ------------------------------------------------------------ decode

// NRL colours and FTS apply from the next strip on.
logic [7:0] nry_s, nru_s, nrv_s;

always_ff @(posedge clk)
	if (ce && hdr)
		{nry_s, nru_s, nrv_s} <= {r.nry, r.nru, r.nrv};

logic        qcx_wr, qw_v, cbuf, blk_start, blk_end, cw_v, nrl_v, run_v;
logic        run_rdy, rle_full, vlc_busy, vlc_done, pipe_idle;
logic  [3:0] qcx_d, vmb;
logic  [6:0] qw_a, run_idx;
logic  [7:0] qw_d;
logic  [2:0] vblk;
logic  [5:0] cw_idx;
logic [11:0] cw_val;
logic  [4:0] nrl_n;
logic  [8:0] run_len;
logic  [1:0] cb_full;

huc6271_vlc vlc (
	.clk, .reset, .ce, .init,
	.start (hdr),
	.kind,
	.empty, .pop,
	.q (fq),
	.in_end (ps == P_DONE && !push && empty),
	.qcx_wr, .qcx_d, .qw_v, .qw_a, .qw_d,
	.cb_full, .cbuf, .blk_start, .blk_end,
	.mb (vmb), .blk (vblk),
	.cw_v, .cw_idx, .cw_val,
	.pipe_idle, .nrl_v, .nrl_n,
	.run_rdy, .rle_full, .run_v, .run_idx, .run_len,
	.busy (vlc_busy),
	.done (vlc_done)
);

logic        dq_v, dq_start, dq_end, dq_buf, dq_busy;
logic  [5:0] dq_idx;
logic [11:0] dq_val;
logic  [3:0] dq_mb;
logic  [2:0] dq_blk;
logic  [7:0] ss_q_rdata;
wire         ss_q = ss_mem_sel && ss_mem_addr[14];

huc6271_dequant dequant (
	.clk,
	.reset (reset || init),
	.ce,
	.qcx (st.qcx),
	.qw_v, .qw_a, .qw_d,
	.cw_v,
	.cw_c (vblk[2]),
	.cw_idx, .cw_val,
	.start_i (blk_start), .end_i (blk_end), .buf_i (cbuf), .mb_i (vmb), .blk_i (vblk),
	.dq_v, .dq_idx, .dq_val,
	.start_o (dq_start), .end_o (dq_end), .buf_o (dq_buf), .mb_o (dq_mb), .blk_o (dq_blk),
	.busy (dq_busy),
	.ss_sel   (ss_q),
	.ss_addr  (ss_mem_addr[6:0]),
	.ss_we    (ss_mem_we),
	.ss_wdata (ss_mem_wdata),
	.ss_rdata (ss_q_rdata)
);

logic       px_v, idct_busy;
logic [3:0] px_mb;
logic [2:0] px_blk, px_y, px_x;
logic [7:0] px_val;

huc6271_idct idct (
	.clk, .reset, .ce, .init,
	.dq_v, .dq_buf, .dq_idx, .dq_val, .dq_start, .dq_end, .dq_mb, .dq_blk,
	.full (cb_full),
	.px_v, .px_mb, .px_blk, .px_y, .px_x, .px_val,
	.busy (idct_busy)
);

logic        wr_v, mb_busy;
logic [12:0] wr_a;
logic  [7:0] wr_d;

huc6271_mb_asm mb_asm (
	.clk,
	.reset (reset || init),
	.ce,
	.start (hdr),
	.fts (st.w_fts && st.w_dct),
	.nry (nry_s), .nru (nru_s), .nrv (nrv_s),
	.px_v, .px_mb, .px_blk, .px_y, .px_x, .px_val,
	.nrl_v,
	.nrl_mb (vmb),
	.nrl_n,
	.run_v, .run_idx, .run_len, .run_rdy, .rle_full,
	.wr_v, .wr_a, .wr_d,
	.busy (mb_busy)
);

assign pipe_idle = !(cw_v || blk_start || blk_end || dq_busy || idct_busy || mb_busy);

// ------------------------------------------------------------ strip timing

wire strip_ok = vlc_done && pipe_idle;
assign judge = ce && hs_fall && r.run && st.jact && st.jc == 4'd15;

always_ff @(posedge clk) begin
	if (reset) begin
		st     <= '0;
		st.qcx <= 4'd4;
	end else if (SAVESTATE && ss_shift) begin
		st <= strip_t'({st[STRIP_BITS-2:0], ss_1});
	end else if (ce) begin
		if (ce_dck) begin
			st.hs_q <= hsync_n;
			st.kx   <= hs_fall ? 9'd309 : nkx;
		end

		if (qcx_wr)
			st.qcx <= qcx_d;

		if (!r.run) begin
			st.jact   <= 1'b0;
			st.dvalid <= 1'b0;
			st.inv    <= 1'b0;
			st.w_done <= 1'b0;
			st.qcx    <= 4'd4;
		end else begin
			if (hdr) begin
				st.jact   <= 1'b1;
				st.jc     <= 4'd0;
				st.w_dct  <= kind == 8'hFF || kind == 8'hF8;
				st.w_fts  <= r.cr[2];
				st.w_done <= 1'b0;
			end else if (strip_ok && st.jact) begin
				st.w_done <= 1'b1;
			end

			if (hs_fall) begin
				st.w_ready <= st.w_done;
				if (st.dvalid || st.inv) begin
					st.dl <= st.dl + 4'd1;
					if (st.dl == 4'd15) begin
						st.dvalid <= 1'b0;
						st.inv    <= 1'b0;
					end
				end
				if (st.jact)
					st.jc <= st.jc + 4'd1;
				if (judge) begin
					st.jact   <= 1'b0;
					st.w_done <= 1'b0;
					st.dvalid <= st.w_done;
					st.inv    <= !st.w_done;
					st.dl     <= 4'd0;
					if (st.w_done) begin
						st.disp  <= !st.disp;
						st.d_dct <= st.w_dct;
						st.d_fts <= st.w_fts;
					end
				end
			end
		end
	end
end

assign ss_2 = SAVESTATE ? st[STRIP_BITS-1] : ss_1;

// ------------------------------------------------------------ R-RAM, display

logic [12:0] rd_a, fb_a;
logic  [7:0] rd_q, fb_q, ss_r_rdata;

// Filter B: last line of a DCT strip when the next DCT strip was ready as
// the line began and takes over at the next -HSYNC.
wire fb_on = st.dvalid && st.d_fts && st.d_dct && st.dl == 4'd15
          && st.w_ready && st.w_dct && st.jact && st.jc == 4'd15;

huc6271_rram_ctl rram (
	.disp (st.disp),
	.rd_a, .rd_q,
	.fb_rd (fb_on),
	.fb_a, .fb_q,
	.wr_v, .wr_a, .wr_d,
	.ss_sel   (ss_mem_sel && !ss_mem_addr[14]),
	.ss_addr  (ss_mem_addr[13:0]),
	.ss_we    (ss_mem_we),
	.ss_wdata (ss_mem_wdata),
	.ss_rdata (ss_r_rdata),
	.rma_cs_n, .rma_oe_n, .rma_we_n, .rma_a, .rma_d_i, .rma_d_o, .rma_d_oe,
	.rmb_cs_n, .rmb_oe_n, .rmb_we_n, .rmb_a, .rmb_d_i, .rmb_d_o, .rmb_d_oe
);

huc6271_out out (
	.clk, .reset, .ce, .ce_dck, .nkx,
	.run    (r.run),
	.dvalid (st.dvalid),
	.d_dct  (st.d_dct),
	.d_fts  (st.d_fts),
	.dl     (st.dl),
	.fb_on,
	.scx    (r.scx_op),
	.chr    (r.chr_op),
	.rd_a, .fb_a, .rd_q, .fb_q,
	.pix
);

assign ss_mem_rdata = ss_mem_addr[14] ? ss_q_rdata : ss_r_rdata;
assign ss_busy      = (ps != P_HUNT && ps != P_DONE) || !empty || vlc_busy || !pipe_idle;

assign invalid      = st.inv;

assign dbg_jc = DEBUG ? st.jc : 4'd0;
assign dbg_dl = DEBUG ? st.dl : 4'd0;

endmodule
