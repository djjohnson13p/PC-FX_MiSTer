// HuC6270 sprites: SATB, the per-line search, pattern fetch in horizontal
// blanking, and the line buffer the display reads back.
//
//   display of line n-1: search SATB for sprites on line n (one word a dot)
//   blanking:            fetch their patterns in SG slots
//   after each pattern:  write 16 dots into the line buffer, two per clock,
//                        skipping dots an earlier sprite already took
//   display of line n:   read each dot out
//
// The line buffer is split into even and odd dots so a tile writes two dots
// a clock; each entry holds {tag, hit, sprite 0, SPBG priority, colour}.
// The tag counts fetches, so a dot left from an earlier line (right of the
// display, or after a line cut by /HSYN) reads as free and never shows.
// Hit marks a sprite 0 dot another sprite covered; CR rises when it is
// displayed. Lines with no search, fetch or output sweep both halves to
// zero, so nothing here but the status flags is saved.

module huc6270_sprite
	import huc6270_pkg::*;
#(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_dck,

	input  slot_t       slot,
	input  logic [15:0] md_i,
	input  logic  [1:0] sm,
	input  logic  [9:0] rc_cnt,

	input  logic        at_hds_d3,
	input  logic        at_hds_d7,
	input  logic        at_hdsm2_d7,
	input  logic        at_hdisp_d3,
	input  logic        at_hdisp_d7,
	input  logic        brk_d7,
	input  logic        v_ge,
	input  logic        v_gt,
	input  logic        v_idle,
	input  logic        bg_out,
	input  logic        vdisp,

	input  logic        cr_sb,
	input  logic        ie_oc,
	input  logic        ie_cc,
	input  logic        sr_rd,

	input  logic  [7:0] sat_wa,
	input  logic        sat_we,
	input  logic [15:0] sat_wd,
	output logic [15:0] sat_qa,

	output logic        spr_fetch,
	output logic        spr_fetch_en,
	output logic        spr_cg,
	output logic [15:0] spr_addr,
	output logic  [8:0] spr_pix,
	output logic        irq_ovf,
	output logic        irq_col,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

// ---- SATB: block transfers write port A, the search reads port B ----

logic  [7:0] eval_x;
/* verilator lint_off UNUSEDSIGNAL */
logic [15:0] sat_q;
/* verilator lint_on UNUSEDSIGNAL */

cache_ram_dp #(.ADDR_WIDTH(8), .DATA_WIDTH(16)) u_sat (
	.clk_i     (clk),
	.addr_a_i  (sat_wa),
	.wren_a_i  (sat_we),
	.wdata_a_i (sat_wd),
	.q_a_o     (sat_qa),
	.addr_b_i  (eval_x),
	.wren_b_i  (1'b0),
	.wdata_b_i (16'd0),
	.q_b_o     (sat_q)
);

// ---- Search: the first 16 cells on the next line go to the cache ----

typedef struct packed {
	logic  [5:0] y;
	logic  [9:0] x;
	logic [10:0] pc;
	logic  [3:0] pal;
	logic        prio;
	logic        cgx;
	logic        wide;
	logic  [1:0] cgy;
	logic        hf;
	logic        vf;
	logic        spr0;
} cache_t;

logic        eval, eval_done, eval_full, find;
logic  [4:0] eval_cnt, eval_cells;
logic  [9:0] spr_y, spr_x;
logic [10:0] spr_pc;

logic        fetch, fetch_en, fetch_w, fetch_done, fetch_cut;
logic  [3:0] fetch_cnt;
logic [15:0] ch0, ch1, ch2;

cache_t      spr;
cache_t      cache_wd;

wire  [5:0] spr_h    = {sat_q[13], sat_q[13] | sat_q[12], 4'hF};
wire        in_range = rc_cnt >= spr_y && rc_cnt <= spr_y + {4'd0, spr_h};
wire        eval_hit = ce_dck && eval && !eval_done && eval_x[1:0] == 2'd3 && in_range;
wire        cache_we = eval_hit && !eval_full;

// A CGX sprite takes two of the 16 cells; with one left, only its left half.
wire        one_left   = eval_cells == 5'd15;
wire        wide       = sat_q[8] && !one_left;
wire  [4:0] cells_next = eval_cells + (wide ? 5'd2 : 5'd1);

// OR: a sprite found with no cell for it, or patterns still unfetched when
// blanking ends.
wire        ovf_set  = ie_oc && ((eval_hit && (eval_full || (sat_q[8] && one_left))) || (fetch_cut && fetch_en));

wire        fetch_start = (at_hdisp_d7 && v_ge) || (brk_d7 && v_gt);
// SM 2 and 3 search four dots less: 2d sprites rather than 2d+1.
wire        eval_end    = (sm[1] ? at_hdisp_d3 : at_hdisp_d7) || brk_d7;

// {y, x, pc, pal, prio, cgx, wide, cgy, hf, vf, spr0}
assign cache_wd = {spr_y[5:0], spr_x, spr_pc, sat_q[3:0], sat_q[7], sat_q[8], wide, sat_q[13:12],
                   sat_q[11], sat_q[15], eval_x[7:2] == 6'd0};

cache_ram #(.ADDR_WIDTH(4), .DATA_WIDTH(39)) u_cache (
	.clk_i   (clk),
	.addr_i  (eval ? eval_cnt[3:0] : fetch_cnt),
	.wren_i  (cache_we),
	.wdata_i (cache_wd),
	.q_o     (spr)
);

// ---- Fetch: SG words for the cached sprites, in blanking ----

wire [5:0] ofs_y = rc_cnt[5:0] - spr.y - 6'd1;
wire [5:0] sline = ofs_y ^ {6{spr.vf}};

logic [2:0] tile_n;
always_comb begin
	tile_n[0] = spr.cgx ? fetch_w ^ spr.hf : spr.pc[1];
	case (spr.cgy)
		2'd0:    tile_n[2:1] = spr.pc[3:2];
		2'd1:    tile_n[2:1] = {spr.pc[3], sline[4]};
		default: tile_n[2:1] = sline[5:4];
	endcase
end

always_comb begin
	case (slot)
		SL_SG0:  spr_addr = {spr.pc[10:4], tile_n, 2'd0, sline[3:0]};
		SL_SG1:  spr_addr = {spr.pc[10:4], tile_n, 2'd1, sline[3:0]};
		SL_SG2:  spr_addr = {spr.pc[10:4], tile_n, 2'd2, sline[3:0]};
		SL_SG3:  spr_addr = {spr.pc[10:4], tile_n, 2'd3, sline[3:0]};
		default: spr_addr = 16'd0;
	endcase
end

// The last SG word of a 16-dot tile.
wire tile_done = (sm == 2'd1 && !spr.pc[0] && slot == SL_SG1) || slot == SL_SG3;

logic [15:0] p0, p1, p2, p3;
logic  [9:0] tile_x;
logic        tile_hf, tile_prio, tile_spr0, tile_save;
logic  [3:0] tile_pal;

always_ff @(posedge clk) begin
	tile_save <= 1'b0;
	fetch_cut <= ce_dck && fetch && at_hdsm2_d7;
	if (reset) begin
		eval       <= 1'b0;
		eval_x     <= 8'd0;
		eval_done  <= 1'b0;
		eval_full  <= 1'b0;
		eval_cnt   <= 5'd0;
		eval_cells <= 5'd0;
		find       <= 1'b0;
		fetch      <= 1'b0;
		fetch_en   <= 1'b0;
		fetch_w    <= 1'b0;
		fetch_done <= 1'b0;
		fetch_cnt  <= 4'd0;
	end else if (ce_dck) begin
		if (at_hds_d3 && v_ge) begin
			eval       <= 1'b1;
			eval_x     <= 8'd0;
			eval_cnt   <= 5'd0;
			eval_cells <= 5'd0;
			eval_done  <= 1'b0;
			eval_full  <= 1'b0;
			find       <= 1'b0;
		end else if (eval_end) begin
			eval <= 1'b0;
		end

		if (fetch_start && !fetch) begin
			fetch      <= 1'b1;
			fetch_en   <= cr_sb && find;
			fetch_cnt  <= 4'd0;
			fetch_w    <= 1'b0;
			fetch_done <= 1'b0;
		end else if (at_hdsm2_d7) begin
			fetch <= 1'b0;
		end

		if (eval && !eval_done) begin
			case (eval_x[1:0])
				2'd0: spr_y  <= sat_q[9:0];
				2'd1: spr_x  <= sat_q[9:0];
				2'd2: spr_pc <= sat_q[10:0];
				default: if (in_range) begin
					find <= 1'b1;
					if (!eval_full) begin
						eval_cnt   <= eval_cnt + 5'd1;
						eval_cells <= cells_next;
						if (cells_next == 5'd16)
							eval_full <= 1'b1;
					end
				end
			endcase
			eval_x <= eval_x + 8'd1;
			if (eval_x == 8'hFF)
				eval_done <= 1'b1;
		end

		if (fetch && !fetch_done && find) begin
			case (slot)
				SL_SG0:  ch0 <= md_i;
				SL_SG1:  ch1 <= md_i;
				SL_SG2:  ch2 <= md_i;
				default: ;
			endcase

			if (tile_done) begin
				fetch_w <= 1'b1;
				if (fetch_w == spr.wide) begin
					fetch_w <= 1'b0;
					if ({1'b0, fetch_cnt} == eval_cnt - 5'd1) begin
						fetch_done <= 1'b1;
						fetch_en   <= 1'b0;
					end else begin
						fetch_cnt <= fetch_cnt + 4'd1;
					end
				end

				// Two-dot mode carries SG0/SG1, or SG2/SG3 when PC0 is set,
				// on colour bits 0 and 1.
				if (sm == 2'd1 && spr.pc[0]) begin
					p0 <= ch2;
					p1 <= md_i;
					p2 <= 16'd0;
					p3 <= 16'd0;
				end else if (sm == 2'd1) begin
					p0 <= ch0;
					p1 <= md_i;
					p2 <= 16'd0;
					p3 <= 16'd0;
				end else begin
					p0 <= ch0;
					p1 <= ch1;
					p2 <= ch2;
					p3 <= md_i;
				end
				tile_x    <= spr.x - 10'd32 + {5'd0, fetch_w, 4'd0};
				tile_hf   <= spr.hf;
				tile_pal  <= spr.pal;
				tile_prio <= spr.prio;
				tile_spr0 <= spr.spr0;
				tile_save <= 1'b1;
			end
		end
	end
end

assign spr_fetch    = fetch;
assign spr_fetch_en = fetch_en;
assign spr_cg       = spr.pc[0];

// ---- Line buffer ----

logic [3:0] pix;
logic [9:0] out_x;
logic       out_en;
logic [8:0] sweep;
logic [8:0] tag, disp_tag;

wire active = tile_save || pix != 4'd0;

// The two dots this clock writes, in tile order.
wire [3:0] n0 = pix ^ {4{!tile_hf}};
wire [3:0] n1 = (pix + 4'd1) ^ {4{!tile_hf}};

wire [3:0] lc0 = {p3[n0], p2[n0], p1[n0], p0[n0]};
wire [3:0] lc1 = {p3[n1], p2[n1], p1[n1], p0[n1]};
wire [9:0] lx0 = tile_x + {6'd0, pix};
/* verilator lint_off UNUSEDSIGNAL */
wire [9:0] lx1 = lx0 + 10'd1;
/* verilator lint_on UNUSEDSIGNAL */
wire       lv0 = active && lx0[9:8] != 2'b11 && lc0 != 4'd0;
wire       lv1 = active && lx1[9:8] != 2'b11 && lc1 != 4'd0;

// Dot 0 goes to the half its X parity picks, dot 1 to the other.
wire       odd = lx0[0];
wire [8:0] wa0 = odd ? lx1[9:1] : lx0[9:1];
wire [8:0] wa1 = odd ? lx0[9:1] : lx1[9:1];

logic       s1_v0, s1_v1;
logic [8:0] s1_a0, s1_a1;
logic [8:0] s1_d0, s1_d1;
logic       s1_spr0;

logic [19:0] q0, q1;

always_ff @(posedge clk) begin
	if (reset) begin
		pix      <= 4'd0;
		out_x    <= 10'd0;
		out_en   <= 1'b0;
		sweep    <= 9'd0;
		tag      <= 9'd0;
		disp_tag <= 9'd0;
		s1_v0    <= 1'b0;
		s1_v1    <= 1'b0;
	end else begin
		if (active)
			pix <= pix + 4'd2;

		s1_v0 <= odd ? lv1 : lv0;
		s1_v1 <= odd ? lv0 : lv1;
		s1_a0 <= wa0;
		s1_a1 <= wa1;
		s1_d0 <= {tile_prio, tile_pal, odd ? lc1 : lc0};
		s1_d1 <= {tile_prio, tile_pal, odd ? lc0 : lc1};
		s1_spr0 <= tile_spr0;

		if (v_idle && !active)
			sweep <= sweep + 9'd1;

		if (ce_dck) begin
			if (fetch_start && !fetch)
				tag <= tag + 9'd1;
			if (at_hds_d7) begin
				out_x    <= 10'd0;
				out_en   <= 1'b1;
				disp_tag <= tag;
			end else if (at_hdisp_d7 || brk_d7) begin
				out_en <= 1'b0;
			end
			if (out_en)
				out_x <= out_x + 10'd1;
		end
	end
end

// A dot is free when it is clear or was written for an earlier line. A dot
// sprite 0 holds is rewritten with hit set when another sprite covers it.
wire        free0 = q0[3:0] == 4'd0 || q0[19:11] != tag;
wire        free1 = q1[3:0] == 4'd0 || q1[19:11] != tag;
wire [19:0] wd0   = free0 ? {tag, 1'b0, s1_spr0, s1_d0} : {q0[19:11], 1'b1, q0[9:0]};
wire [19:0] wd1   = free1 ? {tag, 1'b0, s1_spr0, s1_d1} : {q1[19:11], 1'b1, q1[9:0]};

// Port A writes; port B reads ahead for the writer, sweeps, or reads for
// the display.
wire [8:0] addr_b0  = active ? wa0 : v_idle ? sweep : out_x[9:1];
wire [8:0] addr_b1  = active ? wa1 : v_idle ? sweep : out_x[9:1];
wire       sweep_we = !active && v_idle;

/* verilator lint_off PINCONNECTEMPTY */
cache_ram_dp #(.ADDR_WIDTH(9), .DATA_WIDTH(20)) u_line0 (
	.clk_i     (clk),
	.addr_a_i  (s1_a0),
	.wren_a_i  (s1_v0 && (free0 || q0[9])),
	.wdata_a_i (wd0),
	.q_a_o     (),
	.addr_b_i  (addr_b0),
	.wren_b_i  (sweep_we),
	.wdata_b_i (20'd0),
	.q_b_o     (q0)
);

cache_ram_dp #(.ADDR_WIDTH(9), .DATA_WIDTH(20)) u_line1 (
	.clk_i     (clk),
	.addr_a_i  (s1_a1),
	.wren_a_i  (s1_v1 && (free1 || q1[9])),
	.wdata_a_i (wd1),
	.q_a_o     (),
	.addr_b_i  (addr_b1),
	.wren_b_i  (sweep_we),
	.wdata_b_i (20'd0),
	.q_b_o     (q1)
);
/* verilator lint_on PINCONNECTEMPTY */

/* verilator lint_off UNUSEDSIGNAL */
wire [19:0] qd   = out_x[0] ? q1 : q0;
/* verilator lint_on UNUSEDSIGNAL */
wire        dvis = qd[3:0] != 4'd0 && qd[19:11] == disp_tag;

assign spr_pix = dvis ? qd[8:0] : 9'd0;

// CR from a covered sprite 0 dot as the display passes it.
wire col_set = ie_cc && ce_dck && bg_out && vdisp && dvis && qd[10];

// ---- Status flags (saved) ----

sprite_state_t st;

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= sprite_state_t'({st[SPRITE_BITS-2:0], ss_si});
	end else begin
		// A flag raised on the clock of a status read stays set.
		if (sr_rd) begin
			st.irq_ovf <= 1'b0;
			st.irq_col <= 1'b0;
		end
		if (ovf_set)
			st.irq_ovf <= 1'b1;
		if (col_set)
			st.irq_col <= 1'b1;
	end
end

assign ss_so   = SAVESTATE ? st[SPRITE_BITS-1] : ss_si;
assign irq_ovf = st.irq_ovf;
assign irq_col = st.irq_col;

endmodule
