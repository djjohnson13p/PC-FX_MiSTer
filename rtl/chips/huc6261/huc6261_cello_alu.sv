// HuC6261 cellophane unit for 256-dot mode. Four operations per dot, one
// per master clock, laying the layers back to front over an accumulator,
// then front cellophane:
//
//   Y = mY*YA/8 + nY*YB/8,  U = mU*(UA-80h)/8 + nU*(UB-80h)/8 + 80h,  V alike
//
// clipped to 00-FFh, each term truncated toward zero. A is the layer, B the
// accumulator. A layer that does not mix uses m = 8, n = 0; an absent one
// m = 0, n = 8. Each operation is set up from the palette data into
// registers, then one set of three shift-add multipliers does the A terms
// and the B terms on the next two clocks.
//
//   period q   P: lookup      M: set up        next P: A terms   next M: B terms
//   0          back layer     op 0             op 3 of last dot  op 3 of last dot
//   1          middle layer   op 1             op 0              op 0
//   2          front layer    op 2             op 1              op 1
//   3          palette 0      op 3 (front)     op 2              op 2

module huc6261_cello_alu
(
	input  logic        clk,
	input  logic        ce_osc,
	input  logic        ce_master,
	input  logic  [1:0] q,

	input  huc6261_pkg::cand_t cand [3],
	input  logic  [1:0] order [3],
	/* verilator lint_off UNUSEDSIGNAL */
	input  huc6261_pkg::line_regs_t lr,
	/* verilator lint_on UNUSEDSIGNAL */

	output logic  [7:0] look_idx,
	output logic  [7:0] look_off,
	input  logic [15:0] q_b,

	output logic [23:0] res
);

import huc6261_pkg::*;

wire ce_p = ce_osc && !ce_master;

wire back  = lr.ble[14] && !lr.ble[15];
wire front = lr.ble[14] &&  lr.ble[15];
wire [23:0] ccr = expand(lr.ccr);

wire last = q == 2'd3;
wire [1:0] src = order[last ? 2'd0 : q];
cand_t c;
assign c = cand[src];

assign look_idx = last ? 8'd0 : c.idx;
assign look_off = last ? 8'd0 : c.off;

logic [23:0] acc;
logic        acc_real;
logic [15:0] pal0;
logic  [9:0] ta_y, ta_u, ta_v;
logic [23:0] a_q;
logic [11:0] m_q, n_q;
logic        real_q;

// Operation set up on the master-clock cycle; real_q says the layers so
// far left real data under the next one.
wire        pres = last ? front : c.pres;
wire  [1:0] set  = last ? 2'd1 : c.cel;
wire [23:0] a    = last ? ccr : c.is_pal ? expand(q_b) : c.yuv;
wire        realb = q == 2'd0 ? back : real_q;
wire        mix  = pres && set != 2'd0 && realb;

logic [11:0] set_a, set_b;
always_comb begin
	case (set)
		2'd1:    begin set_a = lr.bl1a; set_b = lr.bl1b; end
		2'd2:    begin set_a = lr.bl2a; set_b = lr.bl2b; end
		default: begin set_a = lr.bl3a; set_b = lr.bl3b; end
	endcase
end

wire [11:0] m = !pres ? 12'h000 : mix ? set_a : 12'h888;
wire [11:0] n = !pres ? 12'h888 : mix ? set_b : 12'h000;

// Shared multipliers: A and m, then B and n.
wire [23:0] b  = q == 2'd1 ? ccr : acc;
wire [23:0] x  = ce_master ? b : a_q;
wire [11:0] k  = ce_master ? n_q : m_q;

// c * f / 8, rounded down
function automatic logic [8:0] mul(input logic [7:0] col, input logic [3:0] f);
	/* verilator lint_off UNUSEDSIGNAL */
	logic [11:0] p;
	/* verilator lint_on UNUSEDSIGNAL */
	p = (f[0] ? {4'd0, col} : 12'd0)
	  + (f[1] ? {3'd0, col, 1'b0} : 12'd0)
	  + (f[2] ? {2'd0, col, 2'b0} : 12'd0)
	  + (f[3] ? {1'b0, col, 3'b0} : 12'd0);
	mul = p[11:3];
endfunction

// Colour difference term, signed about 80h, truncated toward zero.
function automatic logic [9:0] uv_term(input logic [7:0] col, input logic [3:0] f);
	logic [7:0] mag;
	mag = col[7] ? {1'b0, col[6:0]} : 8'd128 - col;
	uv_term = col[7] ? {1'b0, mul(mag, f)} : 10'd0 - {1'b0, mul(mag, f)};
endfunction

wire  [9:0] ty = {1'b0, mul(x[23:16], k[11:8])};
wire  [9:0] tu = uv_term(x[15:8], k[7:4]);
wire  [9:0] tv = uv_term(x[7:0], k[3:0]);

function automatic logic [7:0] clip_y(input logic [9:0] s);
	clip_y = s[9:8] != 2'd0 ? 8'hFF : s[7:0];
endfunction

function automatic logic [7:0] clip_uv(input logic [9:0] t1, input logic [9:0] t2);
	logic [10:0] s;
	s = {t1[9], t1} + {t2[9], t2} + 11'd128;
	clip_uv = s[10] ? 8'h00 : s[9:8] != 2'd0 ? 8'hFF : s[7:0];
endfunction

always_ff @(posedge clk) begin
	if (ce_master) begin
		a_q    <= a;
		m_q    <= m;
		n_q    <= n;
		real_q <= pres || realb;
		if (last)
			pal0 <= q_b;
		acc      <= {clip_y(ta_y + ty), clip_uv(ta_u, tu), clip_uv(ta_v, tv)};
		acc_real <= real_q;
	end
	if (ce_p) begin
		ta_y <= ty;
		ta_u <= tu;
		ta_v <= tv;
	end
end

assign res = lr.blank == 7'd0 ? BLACK : acc_real ? acc : expand(pal0);

endmodule
