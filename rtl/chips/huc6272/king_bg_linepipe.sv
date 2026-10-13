// KING BG line pipes for all four BGs in one block RAM, and the priority
// mix into one dot for the HuC6261.
//
// The RAM holds four 8-word groups per BG ({pbn, word}, a ring): the slot
// data a dot returns is written in phase 0 (bank A) or 1 (bank B) of the
// next dot, BAT words go to the BG's BAT latch instead. During the dot
// before window dot x, phases 2-5 read each BG's word for x (group and
// word from x plus the fine scroll), phase 6 the U/V word of a 16M BG; the
// dot is cut out of its word, and at the DCK the highest {priority, BG#}
// that is not transparent goes on the bus.
//
//   ph   0        1        2    3    4    5    6      7 (9)
//        write A  write B  BG0  BG1  BG2  BG3  16M UV  mix, load bus

module king_bg_linepipe
	import huc6272_pkg::*;
(
	input  logic             clk,
	input  logic             ce_osc,
	input  logic             ce_dck,
	input  logic       [3:0] ph,

	input  slot_rec_t        rec_a,      // slots taking the dot this DCK starts
	input  slot_rec_t        rec_b,
	input  logic      [15:0] ka_q,
	input  logic      [15:0] kb_q,
	output logic             bat_we,
	output logic       [1:0] bat_n,
	output logic      [15:0] bat_d,

	input  logic       [8:0] nkx,
	input  logic             disp_line,
	input  logic       [3:0] shown,
	input  logic [3:0][3:0]  mode,
	input  logic [3:0][2:0]  prio,
	input  logic             rot0,       // BG0 rotated: one stored dot per word
	input  logic [3:0][2:0]  fine,       // fine scroll (0 when rotated)

	output pcfx_pixel_pkg::pcfx_pixel_t pix
);

import pcfx_pixel_pkg::*;

// ------------------------------------------------------------ capture

slot_rec_t fa, fb, da, db;   // in flight this dot, returning this dot

always_ff @(posedge clk) begin
	if (ce_osc && ce_dck) begin
		fa <= rec_a;
		fb <= rec_b;
		da <= fa;
		db <= fb;
	end
end

wire        cap_b = ph == 4'd1;
slot_rec_t  cr;
assign cr = cap_b ? db : da;
wire [15:0] cq = cap_b ? kb_q : ka_q;
wire        cap = ce_osc && ph <= 4'd1 && cr.v;

assign bat_we = cap && cr.bat;
assign bat_n  = cr.bg;
assign bat_d  = cq;

wire [15:0] cw = cr.zero ? 16'd0 : 16'(cq << cr.shift);

// ------------------------------------------------------------ read

function automatic logic [1:0] bpp_k(input logic [2:0] m);
	bpp_k = (m == 3'd1) ? 2'd0 : (m == 3'd2) ? 2'd1 : (m == 3'd3) ? 2'd2 : 2'd3;
endfunction

function automatic logic is_16m(input logic [3:0] m);
	is_16m = m == 4'h5 || m == 4'hD || m == 4'hF;
endfunction

function automatic logic is_64k(input logic [3:0] m);
	is_64k = m == 4'h4 || m == 4'hC || m == 4'hE;
endfunction

wire [8:0] x = nkx - KX_WIN;

// Dot position in the BG's fetched groups.
function automatic logic [4:0] dotpos(input logic [8:0] xx, input logic [2:0] f);
	dotpos = 5'(xx + {6'd0, f});
endfunction

logic [1:0] rn;
always_comb begin
	if (ph == 4'd3)      rn = 2'd1;
	else if (ph == 4'd4) rn = 2'd2;
	else if (ph == 4'd5) rn = 2'd3;
	else                 rn = 2'd0;
end

logic [1:0] c16;
logic       c16_v;
always_comb begin
	c16 = 2'd0;
	c16_v = 1'b0;
	for (int i = 3; i >= 0; i--) begin
		if (shown[i] && is_16m(mode[i])) begin
			c16 = 2'(i);
			c16_v = 1'b1;
		end
	end
end

wire       rot_r = rn == 2'd0 && rot0;
wire [3:0] md_r  = mode[rn];
wire [1:0] k_r   = bpp_k(md_r[2:0]);
wire [4:0] p_r   = dotpos(x, fine[rn]);
wire [2:0] w_r   = rot_r ? p_r[2:0] : is_16m(md_r) ? {p_r[2:1], 1'b0} : p_r[2:0] >> (3'd3 - {1'b0, k_r});
wire [2:0] s_r   = rot_r ? 3'd0 : is_16m(md_r) ? {2'd0, p_r[0]} : p_r[2:0] & (3'd7 >> k_r);
wire [4:0] p_c   = dotpos(x, fine[c16]);

logic [6:0]  ram_a, ram_a_n, ram_a_q;
logic        ram_we;
logic [19:0] ram_q;

always_comb begin
	ram_we = 1'b0;
	if (ph <= 4'd1) begin
		ram_a_n = {cr.bg, cr.grp, cr.widx};
		ram_we  = cap && !cr.bat;
	end else if (ph <= 4'd5) begin
		ram_a_n = {rn, p_r[4:3], w_r};
	end else begin
		ram_a_n = {c16, p_c[4:3], p_c[2:1], 1'b1};
	end
end

// The RAM has no enable: while ce_osc is low keep reading the last address
// so its output still holds that word.
always_ff @(posedge clk)
	if (ce_osc) ram_a_q <= ram_a_n;

assign ram_a = ce_osc ? ram_a_n : ram_a_q;

cache_ram #(
	.ADDR_WIDTH (7),
	.DATA_WIDTH (20)
) ram (
	.clk_i   (clk),
	.addr_i  (ram_a),
	.wren_i  (ram_we),
	.wdata_i ({cr.pbn, cw}),
	.q_o     (ram_q)
);

// ------------------------------------------------------------ cut out

logic [1:0] e_n;
logic [2:0] e_sub;
logic [3:0] e_md;
logic       e_go;

always_ff @(posedge clk) begin
	if (ce_osc) begin
		e_go  <= ph >= 4'd2 && ph <= 4'd5;
		e_n   <= rn;
		e_sub <= s_r;
		e_md  <= md_r;
	end
end

logic [3:0]       r_opq;
logic [3:0][1:0]  r_kind;
logic [3:0][15:0] r_data;

wire  [1:0] e_k = bpp_k(e_md[2:0]);
wire [15:0] e_w = ram_q[15:0];
wire  [3:0] pbn = ram_q[19:16];
/* verilator lint_off UNUSEDSIGNAL */
wire [15:0] e_sh = e_w << ({e_sub, 1'b0} << e_k);   // the dot moved to the top
/* verilator lint_on UNUSEDSIGNAL */
wire  [7:0] e_s  = e_sh[15:8];

always_ff @(posedge clk) begin
	if (ce_osc && e_go) begin
		if (is_16m(e_md)) begin
			r_kind[e_n] <= e_sub[0] ? PIX_YUV_V : PIX_YUV_U;
			r_data[e_n] <= {e_sub[0] ? e_w[7:0] : e_w[15:8], 8'd0};
			r_opq[e_n]  <= (e_sub[0] ? e_w[7:0] : e_w[15:8]) != 8'd0;
		end else if (is_64k(e_md)) begin
			r_kind[e_n] <= PIX_YUV16;
			r_data[e_n] <= e_w;
			r_opq[e_n]  <= e_w[15:8] != 8'd0;
		end else begin
			r_kind[e_n] <= PIX_PAL;
			case (e_k)
				2'd0: begin
					r_data[e_n] <= {10'd0, pbn, e_s[7:6]};
					r_opq[e_n]  <= e_s[7:6] != 2'd0;
				end
				2'd1: begin
					r_data[e_n] <= {8'd0, pbn, e_s[7:4]};
					r_opq[e_n]  <= e_s[7:4] != 4'd0;
				end
				default: begin
					r_data[e_n] <= {8'd0, e_s};
					r_opq[e_n]  <= e_s != 8'd0;
				end
			endcase
		end
	end
end

// ------------------------------------------------------------ mix

logic       win;
logic [1:0] best;
logic [2:0] best_p;

// With no winner the 16M plane still goes out: the HuC6261 pairs U and V
// across neighbouring words, transparent or not.
always_comb begin
	win    = 1'b0;
	best   = c16;
	best_p = 3'd0;
	for (int i = 0; i < 4; i++) begin
		if (shown[i] && r_opq[i] && (!win || prio[i] >= best_p)) begin
			win    = 1'b1;
			best   = 2'(i);
			best_p = prio[i];
		end
	end
end

wire [7:0] chroma = p_c[0] ? ram_q[7:0] : ram_q[15:8];

always_ff @(posedge clk) begin
	if (ce_osc && ce_dck) begin
		pix.valid       <= disp_line && nkx >= KX_WIN && nkx < KX_WIN_END;
		pix.kind        <= r_kind[best];
		pix.plane       <= best;
		pix.transparent <= !win;
		// 4:2:2 kinds (2, 3) take their chroma from the U/V word
		pix.data        <= r_kind[best][1] && c16_v
		                 ? {r_data[best][15:8], chroma} : r_data[best];
	end
end

endmodule
