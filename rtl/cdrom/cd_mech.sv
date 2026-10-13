// The drive mechanism and its controller: spin-up, the head, seeks, the
// read-ahead buffer, CD-DA play, frame timing and sub-channel Q.
//
// The disc turns at 2x for data and 1x for audio. One frame is 6 or 12
// slow ticks, one sector 98 frames. A fast drive counts data frames and
// seeks on a quicker tick; audio stays 1x. head is the sector under the pickup
// next; with nothing to read the pickup jumps back one turn each turn, as
// the tracking servo does, and sub-channel Q shows those sectors.
//
//   READ   sectors p .. rd_end-1 are buffered. A hit costs only the
//          controller's time, a short gap ahead is read through, anything
//          else is a seek. Read-ahead stops when the buffer is full.
//   PLAY   head advances at 1x; samples come from the slot buffer.
//
// Sector data sits in a ring of three blocks fetched from the HPS slot,
// kept ahead of p (reading) or the head (playing). The ring is not saved:
// after a load the blocks are fetched again.

module cd_mech #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce,
	input  logic        ce_t,
	input  logic        ce_d,       // ce_t, or faster for a fast drive: data frames and seeks
	input  logic        ce_fs,      // 44.1 kHz

	input  logic        bus_rst,

	input  logic        disc,
	input  logic        toc_busy,
	input  logic        toc_new,
	input  logic  [6:0] toc_first,
	input  logic  [6:0] toc_last,

	// held by the target until done
	input  logic        go,
	input  logic  [3:0] op,
	input  logic [18:0] lba,
	input  logic [18:0] lend,
	input  logic  [2:0] pmode_i,
	output logic        done,
	output logic  [3:0] err,
	output logic        ready,
	output logic        no_disc,    // gave up looking for a disc
	output logic        changed,    // a new disc has spun up
	output logic  [1:0] play_st,
	output logic        busy_end,
	output logic [18:0] leadout,

	output logic        p_ready,
	output logic        p_eov,
	output logic        p_slow,     // the hit found the read-ahead stopped
	output logic [15:0] p_base,
	input  logic        p_next,

	output logic [79:0] q_now,
	input  logic        q_hold,     // a sub-channel reply is being built: Q stands still
	output logic        q_busy,

	output logic        f_start,
	output logic  [6:0] f_idx,
	output logic        wfck,
	output logic        spun,
	output logic        rw_ok,
	output logic [15:0] rw_base,

	input  logic  [1:0] port0_sel,
	input  logic  [1:0] port1_sel,
	output logic        cdda_ce,
	output logic [15:0] cdda_l,
	output logic [15:0] cdda_r,

	output logic        sm_rq,      // sample reads
	output logic [15:0] sm_addr,
	input  logic        sm_ack,
	output logic        tm_rq,      // TOC reads
	output logic [15:0] tm_addr,
	input  logic        tm_ack,
	input  logic  [7:0] m_q,

	output logic [19:0] blk_lba,
	output logic [15:0] blk_base,
	output logic  [2:0] blk_cnt,    // blocks in the request, less one
	output logic        blk_rd,
	input  logic        blk_done,
	input  logic        blk_err,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so,
	output logic        ss_busy
);

import pcfx_cd_pkg::*;

localparam logic [2:0] D_IDLE = 3'd0, D_SPIN = 3'd1, D_SEEK = 3'd2, D_GOTO = 3'd3,
                       D_READ = 3'd4, D_PLAY = 3'd5, D_HOLD = 3'd6;

mech_t st;

reg  ss_d;
wire ss_end = ss_d && !ss_shift;
always_ff @(posedge clk) ss_d <= ss_shift;

// ---- helpers ----

function automatic logic [7:0] bcd(input logic [6:0] bin7);
	logic [3:0] t;
	logic [6:0] r;
	begin
		r = bin7;
		t = 4'd0;
		if (r >= 7'd80) begin r = r - 7'd80; t = t + 4'd8; end
		if (r >= 7'd40) begin r = r - 7'd40; t = t + 4'd4; end
		if (r >= 7'd20) begin r = r - 7'd20; t = t + 4'd2; end
		if (r >= 7'd10) begin r = r - 7'd10; t = t + 4'd1; end
		bcd = {t, r[3:0]};
	end
endfunction

// a - b - c on {M, S, F}
function automatic logic [19:0] msf_sub(input logic [19:0] a, input logic [19:0] b, input logic c);
	logic [7:0] f, s;
	logic [6:0] m;
	logic bf, bs;
	begin
		f  = {1'b0, a[6:0]} - {1'b0, b[6:0]} - {7'd0, c};
		bf = f[7];
		if (bf) f = f + 8'd75;
		s  = {2'b0, a[12:7]} - {2'b0, b[12:7]} - {7'd0, bf};
		bs = s[7];
		if (bs) s = s + 8'd60;
		m  = a[19:13] - b[19:13] - {6'd0, bs};
		msf_sub = {m, s[5:0], f[6:0]};
	end
endfunction

// sectors in one turn at this radius (constant linear velocity, 25-58 mm)
function automatic logic [4:0] turn(input logic [3:0] h);
	case (h)
	4'd0:        turn = 5'd9;
	4'd1:        turn = 5'd11;
	4'd2:        turn = 5'd12;
	4'd3, 4'd4:  turn = 5'd14;
	4'd5, 4'd6:  turn = 5'd16;
	4'd7, 4'd8:  turn = 5'd18;
	4'd9, 4'd10: turn = 5'd19;
	default:     turn = 5'd21;
	endcase
endfunction

// slot s of the ring starts at s * 2448
function automatic logic [15:0] slot_base(input logic [4:0] s);
	slot_base = {s, 11'd0} + {3'd0, s, 8'd0} + {4'd0, s, 7'd0} + {7'd0, s, 4'd0};
endfunction

function automatic logic [4:0] slot_add(input logic [4:0] a, input logic [4:0] b);
	logic [5:0] t;
	begin
		t = {1'b0, a} + {1'b0, b};
		slot_add = t >= {1'b0, RING_N} ? 5'(t - {1'b0, RING_N}) : t[4:0];
	end
endfunction

// ---- frame timing ----

wire spinning = st.spin == 2'd2;
// 2x only while reading data
wire audio    = !(st.mode == D_READ || ((st.mode == D_SEEK || st.mode == D_GOTO) && st.after == D_READ));
wire [3:0] ft_last = audio ? 4'd11 : 4'd5;
wire tick      = audio ? ce_t : ce_d;
wire frame_end = spinning && tick && st.ft >= ft_last;
wire sec_end   = frame_end && st.fr == 7'd97;

assign spun  = spinning;
assign wfck  = spinning && st.ft > (audio ? 4'd5 : 4'd2);
assign f_idx = st.fr;

// ---- ring of 24 blocks, six to a request ----
//
// Linux answers a request in its own time and caches six sectors, so
// blocks are asked for six at a time, aligned, and the ring holds up to
// 24: a request goes out with 18 still held, 120 ms at 2x, which rides
// out a congested round trip. A play that runs past a block still on its
// way skips it rather than start over.

reg  [18:0] ring_lba;      // first block wanted
reg   [4:0] ring_rp;       // its slot
reg   [4:0] ring_cnt;      // blocks held from ring_lba on
reg   [2:0] ring_skip;     // blocks of the request already passed
reg         ring_fetch;    // a request on its way
reg         ring_stale;    // and no longer wanted

wire        playing = st.mode == D_PLAY;
wire        paused  = st.mode == D_HOLD && st.pst != PS_STOP;
wire [18:0] want    = playing ? st.head : paused ? st.tgt : st.p;
wire        want_on = spinning && disc && !toc_busy &&
                      (st.mode == D_READ || st.mode == D_SEEK || st.mode == D_GOTO || playing || paused);
wire        landed  = ring_fetch && blk_done && !ring_stale && !blk_err;
wire        next1   = want == ring_lba + 19'd1;
wire        adv     = next1 && ring_cnt != 5'd0;
wire        pass    = next1 && ring_cnt == 5'd0 && ring_fetch && !ring_stale && ring_skip != 3'd5;
wire  [4:0] got     = landed ? REQ_N - {2'd0, ring_skip} - {4'd0, pass} : 5'd0;

assign blk_cnt = 3'(REQ_N - 5'd1);

always_ff @(posedge clk) begin
	if (reset) begin
		ring_lba   <= 19'd0;
		ring_rp    <= 5'd0;
		ring_cnt   <= 5'd0;
		ring_skip  <= 3'd0;
		ring_fetch <= 1'b0;
		ring_stale <= 1'b0;
		blk_rd     <= 1'b0;
	end else begin
		if (blk_done) begin
			blk_rd     <= 1'b0;
			ring_fetch <= 1'b0;
			ring_stale <= 1'b0;
			ring_skip  <= 3'd0;
		end
		if (SAVESTATE && ss_shift) begin
			// the ring stands still while the chain turns
			if (landed) ring_cnt <= ring_cnt + REQ_N - {2'd0, ring_skip};
		end else if (!want_on || toc_new || (want != ring_lba && !adv && !pass)) begin
			ring_lba <= want;
			ring_cnt <= 5'd0;
			if (ring_fetch && !blk_done) ring_stale <= 1'b1;
		end else begin
			ring_lba <= want;
			if (adv || pass) ring_rp <= slot_add(ring_rp, 5'd1);
			if (pass && !blk_done) ring_skip <= ring_skip + 3'd1;
			ring_cnt <= ring_cnt - {4'd0, adv} + got;
			if (!ring_fetch && ring_cnt <= RING_N - REQ_N) begin
				ring_fetch <= 1'b1;
				blk_rd     <= 1'b1;
				blk_lba    <= {1'b0, ring_lba + {14'd0, ring_cnt}};
				blk_base   <= slot_base(slot_add(ring_rp, ring_cnt));
			end
		end
	end
end

wire have_p    = ring_cnt != 5'd0 && ring_lba == st.p;
wire have_head = ring_cnt != 5'd0 && ring_lba == st.head;

assign p_ready = st.mode == D_READ && st.rd_end > st.p && have_p;
assign p_base  = slot_base(ring_rp);
assign p_slow  = st.resume;
assign rw_ok   = playing && have_head;
assign rw_base = slot_base(ring_rp);
// a load into a drive whose ring holds other blocks waits for them
assign ss_busy = SAVESTATE && want_on && !(ring_cnt != 5'd0 && ring_lba == want);

// ---- CD-DA: one stereo sample each 44.1 kHz tick, six to a frame ----

reg  [2:0] sm_step;        // 0 idle, 1-4 bytes, 5 out
reg  [9:0] sm_idx;

wire mute = st.pmode == 3'd0 || st.q_ctrl[2];
wire [1:0] sm_k = 2'(sm_step - 3'd1);

always_ff @(posedge clk) begin
	cdda_ce <= 1'b0;
	if (reset) begin
		sm_step <= 3'd0;
		sm_rq   <= 1'b0;
		cdda_l  <= 16'd0;
		cdda_r  <= 16'd0;
	end else if (SAVESTATE && ss_shift) begin
		sm_step <= 3'd0;
		sm_rq   <= 1'b0;
	end else if (ce) begin
		case (sm_step)
		3'd0: if (ce_fs) begin
			sm_idx <= {st.fr, 3'd0} - {2'd0, st.fr, 1'b0} + {7'd0, st.ft[3:1]};
			if (playing && st.pst == PS_PLAY && have_head && !mute) begin
				sm_step <= 3'd1;
				if (port0_sel == 2'd0) cdda_l <= 16'd0;
				if (port1_sel == 2'd0) cdda_r <= 16'd0;
			end else begin
				cdda_l  <= 16'd0;
				cdda_r  <= 16'd0;
				cdda_ce <= 1'b1;
			end
		end
		3'd5: begin
			sm_step <= 3'd0;
			cdda_ce <= 1'b1;
		end
		default: begin
			if (!sm_rq) begin
				sm_rq   <= 1'b1;
				sm_addr <= slot_base(ring_rp) + {4'd0, sm_idx, 2'b00} + {14'd0, sm_k};
			end else if (sm_ack) begin
				// bytes L lo, L hi, R lo, R hi, each to the outputs that carry its channel
				sm_rq   <= 1'b0;
				sm_step <= sm_step + 3'd1;
				if (sm_k[1] ? (port0_sel == 2'd2) : port0_sel[0]) begin
					if (sm_k[0]) cdda_l[15:8] <= m_q; else cdda_l[7:0] <= m_q;
				end
				if (sm_k[1] ? (port1_sel == 2'd2) : port1_sel[0]) begin
					if (sm_k[0]) cdda_r[15:8] <= m_q; else cdda_r[7:0] <= m_q;
				end
			end
		end
		endcase
	end
end

// ---- frames to {M, S, F}, a minute or a second a clock ----

reg [18:0] cv_v;
reg  [6:0] cv_m;
reg  [5:0] cv_s;

wire [19:0] cv_msf = {cv_m, cv_s, cv_v[6:0]};

// ---- engine: operations, Q, the lead-out of a new disc ----

localparam logic [3:0] E_IDLE = 4'd0, E_CONV = 4'd1, E_RD = 4'd2, E_ENTRY = 4'd3, E_RDX = 4'd4,
                       E_IDX = 4'd5, E_END = 4'd6, E_OP = 4'd7, E_Q = 4'd8, E_BCD = 4'd9,
                       E_DIG = 4'd10, E_RDL = 4'd11, E_LOF = 4'd12;

reg  [3:0] es;
reg  [1:0] rk;             // byte of the entry
reg  [6:0] sc_t;
reg [19:0] sc_x;           // position looked up
reg [31:0] sc_e;           // entry bytes as read
reg        sc_q;           // looking up for Q
reg  [6:0] f_trk, n_trk;
reg  [3:0] f_ctrl, n_ctrl;
reg [19:0] f_begin, n_start, n_idx0;
reg        n_ok, before1;
reg  [2:0] bstep;
reg [19:0] lo_msf;         // lead-out of the disc, not saved
reg [18:0] lo_f;

wire [19:0] e_msf   = {sc_e[22:16], sc_e[13:8], sc_e[6:0]};
wire [15:0] ent_adr = TOC_BASE + {7'd0, sc_t - 7'd1, 2'b00} + {14'd0, rk};
wire [15:0] idx_adr = IDX0_BASE + {7'd0, n_trk - 7'd1, 2'b00} + {14'd0, rk};

// one MSF subtractor for the relative time, one BCD converter for the Q fields
wire        q_lead  = sc_x >= lo_msf;
wire        q_pause = !q_lead && (before1 || (n_ok && sc_x >= n_idx0));
wire [19:0] rel     = msf_sub(q_pause ? n_start : sc_x, q_lead ? lo_msf : q_pause ? sc_x : f_begin, q_pause);
wire  [6:0] q_tno   = q_lead ? 7'd0 : q_pause ? n_trk : f_trk;

reg [6:0] bcd_in;
always_comb begin
	case (bstep)
	3'd0:    bcd_in = q_tno;
	3'd2:    bcd_in = rel[19:13];
	3'd3:    bcd_in = {1'b0, rel[12:7]};
	3'd4:    bcd_in = rel[6:0];
	3'd5:    bcd_in = sc_x[19:13];
	3'd6:    bcd_in = {1'b0, sc_x[12:7]};
	default: bcd_in = sc_x[6:0];
	endcase
end
wire [7:0] bcd_q = bcd(bcd_in);

wire scan_op = op == MO_READ || op == MO_SEEK || op == MO_PLAY || op == MO_SEARCH;

// positioning for the op
wire [18:0] gap   = lba >= st.head ? lba - st.head : st.head - lba;
wire [19:0] seek_t = {4'd0, M_SEEK0} + {gap, 1'b0};
wire        moving = st.mode == D_READ || st.mode == D_HOLD || st.mode == D_PLAY || st.mode == D_GOTO;
wire        near   = moving && lba >= st.head && gap < {14'd0, M_THRU};
// at the head, or at the point a pause loops back to
wire        at_now = (moving && lba == st.head) || (st.mode == D_HOLD && lba == st.tgt);
wire        hit    = st.mode == D_READ && lba >= st.p && lba < st.rd_end;
wire        srch_pause = op == MO_SEARCH && !pmode_i[0];

always_ff @(posedge clk) begin
	done     <= 1'b0;
	changed  <= 1'b0;
	busy_end <= 1'b0;
	f_start  <= 1'b0;
	if (reset) begin
		st         <= '0;
		st.spin    <= 2'd1;
		st.timer   <= M_SPINUP;
		st.mode    <= D_SPIN;
		st.pst     <= PS_STOP;
		st.head    <= 19'd150;
		st.p       <= 19'd150;
		st.rd_end  <= 19'd150;
		st.q_pend  <= 1'b1;
		es         <= E_DIG;
		rk         <= 2'd0;
		tm_rq      <= 1'b0;
		lo_msf     <= 20'hFFFFF;
		lo_f       <= 19'h7FFFF;
	end else if (SAVESTATE && ss_shift) begin
		st      <= mech_t'({st[MECH_BITS-2:0], ss_si});
		es      <= E_IDLE;
		tm_rq   <= 1'b0;
	end else if (SAVESTATE && ss_end) begin
		// a load restarts the engine: an op not yet done runs again, Q is rebuilt
		st.op_pend <= st.obusy && !st.wait_arr;
		st.q_pend  <= 1'b1;
	end else if (ce) begin
		// ---- spin-up after a reset or a new disc ----
		if (toc_new || bus_rst) begin
			st.spin    <= 2'd1;
			st.timer   <= disc ? M_SPINUP : M_NODISC;
			st.mode    <= D_SPIN;
			st.pst     <= PS_STOP;
			st.head    <= 19'd150;
			st.p       <= 19'd150;
			st.rd_end  <= 19'd150;
			st.resume  <= 1'b0;
			st.op_pend <= 1'b0;
			st.wait_arr <= 1'b0;
			st.newdisc <= toc_new ? disc : st.newdisc;
			if (st.obusy) begin
				st.obusy <= 1'b0;
				st.err   <= E_NOT_READY;
				done     <= 1'b1;
			end
			es    <= toc_new ? E_DIG : E_IDLE;
			tm_rq <= 1'b0;
		end else begin
			if (st.mode == D_SPIN && ce_t) begin
				if (st.timer != 20'd0) begin
					st.timer <= st.timer - 20'd1;
				end else begin
					st.spin    <= disc ? 2'd2 : 2'd0;
					st.mode    <= disc ? D_HOLD : D_IDLE;
					st.tgt     <= 19'd150;
					changed    <= st.newdisc && disc;
					st.newdisc <= 1'b0;
				end
			end

			// ---- frames and sectors ----
			if (frame_end) begin
				st.ft   <= 4'd0;
				st.fr   <= st.fr == 7'd97 ? 7'd0 : st.fr + 7'd1;
				f_start <= 1'b1;
			end else if (tick && spinning) begin
				st.ft <= st.ft + 4'd1;
			end

			if (sec_end) begin
				st.q_pend <= 1'b1;
				case (st.mode)
				D_GOTO: begin
					st.head <= st.head + 19'd1;
					if (st.head + 19'd1 == st.tgt) begin
						st.mode <= st.after;
						if (st.wait_arr) begin
							done        <= 1'b1;
							st.obusy    <= 1'b0;
							st.wait_arr <= 1'b0;
						end
					end
				end
				D_READ: begin
					if (st.head != st.rd_end) begin
						st.head <= st.head + 19'd1;
					end else if (st.rd_end < st.p || st.rd_end - st.p < {11'd0, M_RA_MAX}) begin
						st.head   <= st.head + 19'd1;
						st.rd_end <= st.rd_end + 19'd1;
					end else begin
						st.head <= st.head - {14'd0, turn(st.head[18:15])};
					end
				end
				D_PLAY: begin
					if (st.head + 19'd1 < st.pend) begin
						st.head <= st.head + 19'd1;
					end else if (st.pmode == 3'd4) begin
						st.tgt   <= st.pstart;
						st.after <= D_PLAY;
						st.mode  <= D_SEEK;
						st.timer <= {4'd0, M_SEEK0};
					end else begin
						st.head  <= st.pend;
						st.tgt   <= st.pend;
						st.mode  <= D_HOLD;
						st.pst   <= PS_STOP;
						busy_end <= 1'b1;
					end
				end
				D_HOLD: begin
					if (st.head + 19'd1 >= st.tgt)
						st.head <= st.tgt - {14'd0, turn(st.tgt[18:15])};
					else
						st.head <= st.head + 19'd1;
				end
				default: ;
				endcase
			end

			// a seek lands on a sector boundary, where the next block starts
			if (st.mode == D_SEEK) begin
				if (st.timer != 20'd0) begin
					if (ce_d) st.timer <= st.timer - 20'd1;
				end else if (sec_end) begin
					st.head   <= st.tgt;
					st.rd_end <= st.tgt;
					st.mode   <= st.after;
					st.q_pend <= 1'b1;
					if (st.wait_arr) begin
						done        <= 1'b1;
						st.obusy    <= 1'b0;
						st.wait_arr <= 1'b0;
					end
				end
			end

			if (p_next) begin
				st.p      <= st.p + 19'd1;
				st.resume <= 1'b0;
			end

			if (go) begin
				st.op_pend <= 1'b1;
				st.obusy   <= 1'b1;
			end
		end

		// ---- engine ----
		case (es)
		E_IDLE: begin
			rk <= 2'd0;
			if (st.op_pend && !toc_busy) begin
				st.op_pend <= 1'b0;
				sc_q    <= 1'b0;
				cv_v    <= lba;
				cv_m    <= 7'd0;
				cv_s    <= 6'd0;
				es      <= E_CONV;
			end else if (st.q_pend && spinning && !toc_busy && !q_hold) begin
				st.q_pend <= 1'b0;
				sc_q    <= 1'b1;
				cv_v    <= st.head;
				cv_m    <= 7'd0;
				cv_s    <= 6'd0;
				es      <= E_CONV;
			end
		end

		E_CONV: begin
			if (cv_v >= 19'd4500) begin
				cv_v <= cv_v - 19'd4500;
				cv_m <= cv_m + 7'd1;
			end else if (cv_v >= 19'd75) begin
				cv_v <= cv_v - 19'd75;
				cv_s <= cv_s + 6'd1;
			end else begin
				sc_x    <= cv_msf;
				sc_t    <= toc_first;
				before1 <= 1'b1;
				n_ok    <= 1'b0;
				n_trk   <= 7'd100;
				n_start <= lo_msf;
				n_idx0  <= lo_msf;
				n_ctrl  <= 4'd0;
				f_trk   <= 7'd0;
				f_ctrl  <= 4'd0;
				f_begin <= 20'd0;
				es      <= (sc_q || scan_op) ? E_RD : E_OP;
			end
		end

		E_RD: begin
			if (!tm_rq) begin
				tm_rq   <= 1'b1;
				tm_addr <= ent_adr;
			end else if (tm_ack) begin
				tm_rq <= 1'b0;
				sc_e  <= {m_q, sc_e[31:8]};
				rk    <= rk + 2'd1;
				if (rk == 2'd3) es <= E_ENTRY;
			end
		end

		E_ENTRY: begin
			if (e_msf <= sc_x) begin
				before1 <= 1'b0;
				f_trk   <= sc_t;
				f_ctrl  <= sc_e[27:24];
				f_begin <= e_msf;
				if (sc_t >= toc_last) begin
					es <= E_END;
				end else begin
					sc_t <= sc_t + 7'd1;
					es   <= E_RD;
				end
			end else begin
				n_ok    <= 1'b1;
				n_trk   <= sc_t;
				n_ctrl  <= sc_e[27:24];
				n_start <= e_msf;
				n_idx0  <= e_msf;
				// the first track's pregap is always the whole lead-in side
				es      <= sc_t == toc_first ? E_END : E_RDX;
			end
		end

		E_RDX: begin
			if (!tm_rq) begin
				tm_rq   <= 1'b1;
				tm_addr <= idx_adr;
			end else if (tm_ack) begin
				tm_rq <= 1'b0;
				sc_e  <= {m_q, sc_e[31:8]};
				rk    <= rk + 2'd1;
				if (rk == 2'd2) es <= E_IDX;
			end
		end
		E_IDX: begin
			if (sc_e[31:8] != 24'd0)
				n_idx0 <= {sc_e[30:24], sc_e[21:16], sc_e[14:8]};
			es <= E_END;
		end

		E_END: if (!(sc_q && q_hold)) es <= sc_q ? E_Q : E_OP;

		// lead-out, a track's pause (relative time counts down), or a track
		E_Q: begin
			st.q_ctrl <= q_pause ? n_ctrl : f_ctrl;
			bstep     <= 3'd0;
			es        <= E_BCD;
		end

		E_BCD: begin
			bstep <= bstep + 3'd1;
			case (bstep)
			3'd0: st.q_tno        <= q_tno == 7'd0 ? 8'hAA : bcd_q;
			3'd1: st.q_idx        <= !q_pause;
			3'd2: st.q_rel[23:16] <= bcd_q;
			3'd3: st.q_rel[15:8]  <= bcd_q;
			3'd4: st.q_rel[7:0]   <= bcd_q;
			3'd5: st.q_abs[23:16] <= bcd_q;
			3'd6: st.q_abs[15:8]  <= bcd_q;
			default: begin
				st.q_abs[7:0] <= bcd_q;
				es <= E_IDLE;
			end
			endcase
		end

		// the lead-out of a new disc, as {M, S, F} and as a frame number
		E_DIG: begin
			sc_t <= 7'd100;
			rk   <= 2'd0;
			if (!toc_busy) es <= E_RDL;
		end
		E_RDL: begin
			if (!tm_rq) begin
				tm_rq   <= 1'b1;
				tm_addr <= ent_adr;
			end else if (tm_ack) begin
				tm_rq <= 1'b0;
				sc_e  <= {m_q, sc_e[31:8]};
				rk    <= rk + 2'd1;
				if (rk == 2'd2) begin
					lo_msf <= {m_q[6:0], sc_e[29:24], sc_e[22:16]};
					lo_f   <= {12'd0, sc_e[22:16]};
					cv_m   <= m_q[6:0];
					cv_s   <= sc_e[29:24];
					es     <= E_LOF;
				end
			end
		end
		E_LOF: begin
			if (cv_m != 7'd0) begin
				lo_f <= lo_f + 19'd4500;
				cv_m <= cv_m - 7'd1;
			end else if (cv_s != 6'd0) begin
				lo_f <= lo_f + 19'd75;
				cv_s <= cv_s - 6'd1;
			end else begin
				es <= E_IDLE;
			end
		end

		E_OP: begin
			es       <= E_IDLE;
			st.err   <= E_NONE;
			st.obusy <= 1'b0;
			done     <= 1'b1;
			case (op)
			MO_READ, MO_SEEK: begin
				if (sc_x >= lo_msf || before1) begin
					st.err <= E_EOV;
				end else if (op == MO_READ && !f_ctrl[2]) begin
					st.err <= E_NOT_DATA;
				end else begin
					st.pst    <= PS_STOP;
					st.p      <= lba;
					st.tgt    <= lba;
					st.resume <= 1'b0;
					if (op == MO_READ && hit) begin
						st.resume <= st.head != st.rd_end;
					end else if (op == MO_READ && near) begin
						st.rd_end <= st.head;
						st.mode   <= D_READ;
					end else begin
						st.rd_end   <= lba;
						st.after    <= D_READ;
						st.mode     <= D_SEEK;
						st.timer    <= seek_t;
						st.wait_arr <= op == MO_SEEK;
						st.obusy    <= op == MO_SEEK;
						done        <= op == MO_READ;
					end
				end
			end
			MO_PLAY, MO_SEARCH: begin
				if (sc_x > lo_msf || before1) begin
					st.err <= E_BAD_PARAM;
				end else if (op == MO_PLAY && lend == lba) begin
					st.err <= E_NONE;
				end else if (f_ctrl[2]) begin
					st.err <= E_NOT_AUDIO;
				end else begin
					st.tgt    <= lba;
					st.pstart <= lba;
					st.pend   <= op == MO_PLAY ? lend : lo_f;
					st.pmode  <= op == MO_PLAY ? pmode_i : 3'd3;
					st.pst    <= srch_pause ? PS_SEARCH : PS_PLAY;
					st.after  <= srch_pause ? D_HOLD : D_PLAY;
					if (at_now) begin
						// on from the next block
						st.mode <= D_GOTO;
						st.head <= lba - 19'd1;
					end else begin
						st.mode     <= near ? D_GOTO : D_SEEK;
						st.timer    <= seek_t;
						st.wait_arr <= op == MO_SEARCH;
						st.obusy    <= op == MO_SEARCH;
						done        <= op == MO_PLAY;
					end
				end
			end
			MO_SETEND: begin
				st.pend  <= lend;
				st.pmode <= pmode_i;
				st.pst   <= PS_PLAY;
				if (st.mode == D_HOLD) begin
					st.mode <= D_PLAY;
					st.head <= st.tgt;
				end
				if (st.after == D_HOLD) st.after <= D_PLAY;
			end
			MO_PAUSE: begin
				st.pst <= PS_STILL;
				if (st.mode == D_PLAY) begin
					st.mode <= D_HOLD;
					st.tgt  <= st.head;
				end
				if (st.after == D_PLAY) st.after <= D_HOLD;
			end
			MO_FLUSH: begin
				// the next read seeks, whatever is near
				if (st.mode == D_READ) st.mode <= D_IDLE;
			end
			MO_RESUME: begin
				st.pst <= PS_PLAY;
				if (st.mode == D_HOLD) begin
					st.mode <= D_PLAY;
					st.head <= st.tgt;
				end
				if (st.after == D_HOLD) st.after <= D_PLAY;
			end
			default: begin
				// stop, or a message out: audio stops, read-ahead ends
				st.pst <= PS_STOP;
				if (st.mode == D_PLAY || (op == MO_ABORT && st.mode == D_READ)) begin
					st.mode <= D_HOLD;
					st.tgt  <= st.head;
				end
				if (st.mode == D_SEEK || st.mode == D_GOTO) st.after <= D_HOLD;
				st.wait_arr <= 1'b0;
			end
			endcase
		end

		default: es <= E_IDLE;
		endcase
	end
end

assign err     = st.err;
assign ready   = spinning && disc;
assign no_disc = st.spin == 2'd0;
assign play_st = st.pst;
assign leadout = lo_f;
assign p_eov   = st.p >= lo_f;
assign q_busy  = es == E_Q || es == E_BCD;
assign q_now   = {st.q_ctrl, 4'h1, st.q_tno, 7'd0, st.q_idx, st.q_rel, 8'h00, st.q_abs};

assign ss_so = SAVESTATE ? st[MECH_BITS-1] : ss_si;

wire unused_ok = &{1'b0, sc_e[7]};

endmodule
