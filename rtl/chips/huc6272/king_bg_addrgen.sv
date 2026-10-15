// KING BG address generator for one KRAM bank: turns the microprogram word
// of the planned dot into a KRAM address and a record of where the data
// goes. Three stages at ph 2, 3 and 4 of the current dot.
//
// Each word names the dot (or 8-dot group) its data is for, counted from
// fetch group 0 (tp 0 = first dot of group 0):
//
//   group BAT (block modes)      the group of nkx + 2 (C6272_2: its CG reads
//                                follow 2-7 cycles later, wrapping 7 -> 0)
//   group CG                     the group of nkx
//   per-dot BAT / direct CG      nkx - 8   (rotated, external dot)
//   per-dot BAT-led CG           nkx - 10  (two cycles after its BAT)
//
// A dot lies in the main screen, else (cylinder mode or inside it) in the
// sub screen, else nowhere (fetched, stored as transparent). Addresses:
//
//   BAT, block   BAT + (Y/8 << (n-3)) + X/8
//   BAT, dot     BAT + (Y << (n-s)) + X >> s           s = 1 for 16M
//   CG, direct   CG  + (Y << (n+k-3)) + X >> (3-k) + offset
//   CG, block    CG  + (char << (3+k)) + (Y%8 << k) + offset or dot word
//   CG, dot      CG  + (char << s) + offset
//
// n = log2 screen width, 2 << k bits per dot (k = 3 for 64K and 16M).
// Base registers select the bank and A16:A10.

module king_bg_addrgen
	import huc6272_pkg::*;
#(
	parameter bit BANK = 1'b0
)
(
	input  logic             clk,
	input  logic             ce_osc,
	input  logic       [3:0] ph,

	input  logic             active,     // sequencer runs for the planned dot
	input  logic       [8:0] nkx,
	input  mpw_t             w,

	input  logic [3:0][2:0]  prio,
	input  logic [3:0][3:0]  mode,
	input  logic      [15:0] size0,
	input  logic [2:0][7:0]  size,
	input  logic       [3:0] cyl,
	input  logic [3:0][7:0]  bat,
	input  logic [3:0][7:0]  cg,
	input  logic       [7:0] bat0s,
	input  logic       [7:0] cg0s,
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic      [10:0] sx0,        // fine scroll is applied when reading
	input  logic [2:0][9:0]  sx,
	/* verilator lint_on UNUSEDSIGNAL */
	input  logic [3:0][10:0] yl,         // line Y per BG, wrapped
	input  logic [3:0][15:0] batl,       // BAT latches
	input  logic      [15:0] ax,
	input  logic      [15:0] ay,
	input  logic      [15:0] bx,
	input  logic      [15:0] by,

	output logic             req_v,
	output logic      [16:0] req_a,
	output slot_rec_t        rec
);

// ------------------------------------------------------------ stage A

typedef enum logic [2:0] { T_BATB, T_BATD, T_CGD, T_CGB, T_CGX } kind_t;

logic        a_v, a_rot, a_pp, a_16m, a_ext;
kind_t       a_kind;
logic  [1:0] a_n, a_k;
logic  [2:0] a_off;
logic  [4:0] a_tp;
logic [15:0] a_x, a_y, a_ch;

wire  [1:0] n    = w.bg;
wire  [3:0] md   = mode[n];
wire        ext  = md[3:1] == 3'b111;
wire        m16m = md == 4'h5 || md == 4'hD || md == 4'hF;
wire        pp   = w.rot || ext;
wire  [1:0] k    = (md[2:0] == 3'd1) ? 2'd0 :
                   (md[2:0] == 3'd2) ? 2'd1 :
                   (md[2:0] == 3'd3) ? 2'd2 : 2'd3;

logic signed [9:0] tp;
always_comb begin
	if (w.bat && !pp)
		tp = signed'({1'b0, (nkx + 9'd2) & 9'h1F8}) - 10'sd8;
	else if (!pp)
		tp = signed'({1'b0, nkx & 9'h1F8}) - 10'sd8;
	else if (w.ind && !w.bat)
		tp = signed'({1'b0, nkx}) - 10'sd10 - signed'({9'd0, ext && m16m && w.off[0]});
	else
		tp = signed'({1'b0, nkx}) - 10'sd8;
end

wire tp_ok = tp >= 10'sd0 && tp < 10'sd264;

// Scrolled position of a non-rotated dot, wrapped to the BG's space.
wire [10:0] xs0 = {sx0[10:3], 3'd0} + {2'd0, tp[8:0]};
wire  [6:0] sxn = n == 2'd1 ? sx[0][9:3] : n == 2'd2 ? sx[1][9:3] : sx[2][9:3];
wire  [9:0] xs  = {sxn, 3'd0} + tp[9:0];

always_ff @(posedge clk) begin
	if (ce_osc && ph == 4'd2) begin
		a_v   <= active && !w.nop && prio[n] != 3'd0 && tp_ok;
		a_rot <= w.rot;
		a_pp  <= pp;
		a_ext <= ext;
		a_16m <= m16m;
		a_n   <= n;
		a_k   <= k;
		a_off <= w.off;
		a_tp  <= tp[4:0];
		a_ch  <= batl[n];
		a_kind <= w.bat ? (ext ? T_BATD : T_BATB) :
		          w.ind ? (ext ? T_CGX : T_CGB) : T_CGD;
		if (w.rot) begin
			a_x <= w.ind && !w.bat ? bx : ax;
			a_y <= w.ind && !w.bat ? by : ay;
		end else if (n == 2'd0) begin
			a_x <= {5'd0, xs0};
			a_y <= {5'd0, yl[0]};
		end else begin
			a_x <= {6'd0, xs};
			a_y <= {6'd0, yl[n][9:0]};
		end
	end
end

// ------------------------------------------------------------ stage B

logic        b_v, b_rot, b_pp, b_16m, b_bad, b_none;
kind_t       b_kind;
logic  [1:0] b_n, b_k;
logic  [2:0] b_off;
logic  [4:0] b_tp;
logic [15:0] b_ch;
logic  [9:0] b_x, b_y;
logic  [3:0] b_lw;
logic  [7:0] b_bat, b_cg;

wire  [7:0] sz_n = a_n == 2'd1 ? size[0] : a_n == 2'd2 ? size[1] : size[2];
wire  [7:0] sz_m = a_n == 2'd0 ? size0[7:0]  : sz_n;
wire  [7:0] sz_s = a_n == 2'd0 ? size0[15:8] : sz_n;
wire  [3:0] lw_m = size_log2(sz_m[7:4]);
wire  [3:0] lh_m = size_log2(sz_m[3:0]);
wire  [3:0] lw_s = size_log2(sz_s[7:4]);
wire  [3:0] lh_s = size_log2(sz_s[3:0]);

wire in_main = (a_x >> lw_m) == 16'd0 && (a_y >> lh_m) == 16'd0;
wire in_sub  = (a_x >> lw_s) == 16'd0 && (a_y >> lh_s) == 16'd0;
wire use_sub = !in_main && (cyl[a_n] || in_sub);

wire  [3:0] lw = use_sub ? lw_s : lw_m;
wire  [3:0] lh = use_sub ? lh_s : lh_m;
wire [15:0] xm = 16'(~(16'hFFFF << lw));
wire [15:0] ym = 16'(~(16'hFFFF << lh));
wire  [9:0] xe = 10'(a_x & xm);
wire  [9:0] ye = 10'(a_y & ym);

always_ff @(posedge clk) begin
	if (ce_osc && ph == 4'd3) begin
		b_v    <= a_v;
		b_rot  <= a_rot;
		b_pp   <= a_pp;
		b_16m  <= a_16m && a_ext;
		b_kind <= a_kind;
		b_n    <= a_n;
		b_k    <= a_k;
		b_off  <= a_off;
		b_tp   <= a_tp;
		b_ch   <= a_ch;
		b_none <= !in_main && !use_sub;
		b_bad  <= size_bad(use_sub ? sz_s[7:4] : sz_m[7:4]);
		b_lw   <= lw;
		b_x    <= xe;
		b_y    <= ye;
		b_bat  <= (a_n == 2'd0 && use_sub) ? bat0s : bat[a_n];
		b_cg   <= (a_n == 2'd0 && use_sub) ? cg0s : cg[a_n];
	end
end

// ------------------------------------------------------------ stage C

logic  [9:0] row_v;
logic  [4:0] row_s;
logic [16:0] col, extra;
logic  [7:0] base;

wire [9:0] y_row = b_bad ? 10'd0 : b_y;
wire [2:0] k3    = 3'd3 - {1'b0, b_k};

always_comb begin
	row_v = 10'd0;
	row_s = 5'd0;
	col   = 17'd0;
	extra = 17'd0;
	base  = b_cg;
	case (b_kind)
		T_BATB: begin
			base  = b_bat;
			row_v = {3'd0, y_row[9:3]};
			row_s = {1'b0, b_lw} - 5'd3;
			col   = {10'd0, b_x[9:3]};
		end
		T_BATD: begin
			base  = b_bat;
			row_v = y_row;
			row_s = {1'b0, b_lw} - {4'd0, b_16m};
			col   = {7'd0, b_x} >> b_16m;
		end
		T_CGD: begin
			row_v = y_row;
			row_s = {1'b0, b_lw} + {3'd0, b_k} - 5'd3;
			col   = ({7'd0, b_x} >> k3) + (b_pp ? 17'd0 : {14'd0, b_off});
		end
		T_CGB: begin
			row_v = {7'd0, b_y[2:0]};
			row_s = {3'd0, b_k};
			col   = b_pp ? {14'd0, b_x[2:0] >> k3} : {14'd0, b_off};
			extra = {2'd0, b_ch[11:0], 3'd0} << b_k;
		end
		default: begin  // T_CGX
			col   = {14'd0, b_off};
			extra = {1'b0, b_ch} << b_16m;
		end
	endcase
end

wire [16:0] row = 17'({7'd0, row_v} << row_s);

// Bits a rotated dot is shifted by so it sits at the top of its word.
wire [2:0] pos   = b_x[2:0] & (3'd7 >> b_k);
wire       b_isbat = b_kind == T_BATB || b_kind == T_BATD;
wire [3:0] shamt = b_rot && !b_isbat ? 4'({pos, 1'b0} << b_k) : 4'd0;

always_ff @(posedge clk) begin
	if (ce_osc && ph == 4'd4) begin
		req_v     <= b_v && base[7] == BANK;
		req_a     <= {base[6:0], 10'd0} + row + col + extra;
		rec.v     <= b_v && base[7] == BANK;
		rec.bat   <= b_isbat;
		rec.bg    <= b_n;
		rec.grp   <= b_tp[4:3];
		rec.widx  <= !b_pp ? b_off : b_16m ? {b_tp[2:1], b_off[0]} : b_tp[2:0];
		rec.shift <= shamt;
		rec.pbn   <= b_kind == T_CGB ? b_ch[15:12] : 4'd0;
		rec.zero  <= b_none;
	end
end

endmodule
