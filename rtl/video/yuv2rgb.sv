// Y/U/V to RGB at the board edge, U' = U - 80h, V' = V - 80h:
//
//   jfif 0:  R = Y + 1.1398 V'   G = Y - 0.3946 U' - 0.5805 V'   B = Y + 2.032 U'
//   jfif 1:  R = Y + 1.402 V'    G = Y - 0.34414 U' - 0.71414 V'  B = Y + 1.772 U'
//
// The first truncates each term toward zero, the second rounds. The terms
// come from two 512-entry ROMs indexed by {jfif, U} and {jfif, V}; they are
// computed at elaboration, which the block-RAM wrapper cannot take, so the
// ROMs are inferred here. Two clocks of latency on every signal.

module yuv2rgb
(
	input  logic       clk,
	input  logic       jfif,

	input  logic       ce_pix,
	input  logic       hs,
	input  logic       vs,
	input  logic       hblank,
	input  logic       vblank,
	input  logic [7:0] y,
	input  logic [7:0] u,
	input  logic [7:0] v,

	output logic       ce_pix_o,
	output logic       hs_o,
	output logic       vs_o,
	output logic       hblank_o,
	output logic       vblank_o,
	output logic [7:0] r,
	output logic [7:0] g,
	output logic [7:0] b
);

// c * x, c in 1/65536, toward zero or to nearest
function automatic integer term(input integer c, input integer x, input bit near);
	integer p;
	p = c * x;
	if (near)
		p = p + (p < 0 ? -32768 : 32768);
	term = p / 65536;
endfunction

// {gU[6:0], bU[9:0]} and {rV[8:0], gV[7:0]}
logic [16:0] rom_u [512];
logic [16:0] rom_v [512];

/* verilator lint_off UNUSEDSIGNAL */
integer i, d, gu, bu, rv, gv;
/* verilator lint_on UNUSEDSIGNAL */
bit j;

initial begin
	for (i = 0; i < 512; i = i + 1) begin
		j  = i >= 256;
		d  = (i & 255) - 128;
		gu = term(j ? -22554  : -25861, d, j);
		bu = term(j ? 116130  : 133169, d, j);
		rv = term(j ? 91881   : 74700,  d, j);
		gv = term(j ? -46802  : -38044, d, j);
		rom_u[i] = {7'(gu), 10'(bu)};
		rom_v[i] = {9'(rv), 8'(gv)};
	end
end

logic [16:0] qu, qv;
logic  [7:0] y1;
logic        ce1, hs1, vs1, hb1, vb1;

always_ff @(posedge clk) begin
	qu  <= rom_u[{jfif, u}];
	qv  <= rom_v[{jfif, v}];
	y1  <= y;
	ce1 <= ce_pix;
	hs1 <= hs;
	vs1 <= vs;
	hb1 <= hblank;
	vb1 <= vblank;
end

function automatic logic [7:0] clamp(input logic signed [10:0] s);
	clamp = s < 0 ? 8'h00 : s > 11'sd255 ? 8'hFF : s[7:0];
endfunction

wire signed [10:0] ys = {3'b000, y1};
wire signed [10:0] rs = ys + 11'(signed'(qv[16:8]));
wire signed [10:0] gs = ys + 11'(signed'(qu[16:10])) + 11'(signed'(qv[7:0]));
wire signed [10:0] bs = ys + 11'(signed'(qu[9:0]));

always_ff @(posedge clk) begin
	r        <= clamp(rs);
	g        <= clamp(gs);
	b        <= clamp(bs);
	ce_pix_o <= ce1;
	hs_o     <= hs1;
	vs_o     <= vs1;
	hblank_o <= hb1;
	vblank_o <= vb1;
end

endmodule
