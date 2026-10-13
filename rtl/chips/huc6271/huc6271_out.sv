// HuC6271 display read-out: one dot of the strip on display per DCKKR,
// window dot n at kx n + 32 as KING counts it. The dot is read during the
// one before it: Y (or the palette byte) then chroma, through the filters,
// onto the pixel bus at the next DCK.
//
// SCX is the image dot under window dot 0, 9 bits. With CHR the line wraps
// (image dot 255 is followed by 0); without it dots outside 0-255 are
// transparent. Internally Y, U and V are two's complement; the bus gets
// them with the MSB inverted (Y straight, U/V offset binary).

module huc6271_out
	import huc6271_pkg::*, pcfx_pixel_pkg::*;
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce,
	input  logic        ce_dck,
	input  logic  [8:0] nkx,

	input  logic        run,
	input  logic        dvalid,
	input  logic        d_dct,
	input  logic        d_fts,
	input  logic  [3:0] dl,
	input  logic        fb_on,
	input  logic  [8:0] scx,
	input  logic        chr,

	output logic [12:0] rd_a,
	output logic [12:0] fb_a,
	input  logic  [7:0] rd_q,
	input  logic  [7:0] fb_q,

	output pcfx_pixel_t pix
);

logic  [3:0] ph;
logic        in_win, in_area;
logic  [7:0] ix;
logic  [7:0] yq, fbq;
pcfx_pixel_t word;

wire [8:0] sx  = nkx - 9'd32;
wire [8:0] ix9 = sx + scx;
wire [7:0] y_f;

huc6271_fts fts (
	.clk,
	.step    (ce && ph == 4'd3),
	.on      (d_fts && d_dct && dvalid),
	.vert    (fb_on),
	.in_area,
	.ix_lo   (ix[2:0]),
	.y       (yq),
	.y_next  (fbq),
	.y_out   (y_f)
);

always_ff @(posedge clk) begin
	if (reset) begin
		ph  <= 4'd0;
		pix <= '0;
	end else if (ce) begin
		ph <= ce_dck ? 4'd0 : ph == 4'hF ? ph : ph + 4'd1;
		case (ph)
			4'd0: begin
				in_win  <= nkx >= 9'd32 && nkx < 9'd288;
				in_area <= chr || !ix9[8];
				ix      <= ix9[7:0];
				rd_a    <= d_dct ? {dl, ix9[7:1], 1'b0, ix9[0]} : {1'b0, dl, ix9[7:0]};
				fb_a    <= {4'd0, ix9[7:1], 1'b0, ix9[0]};
			end
			4'd1: rd_a <= {dl, ix[7:1], 1'b1, ix[0]};
			4'd2: begin
				yq  <= rd_q;
				fbq <= fb_q;
			end
			4'd3: begin
				word.valid       <= run && in_win;
				word.plane       <= 2'd0;
				word.transparent <= !dvalid || !in_area;
				if (d_dct) begin
					word.kind <= ix[0] ? PIX_YUV_V : PIX_YUV_U;
					word.data <= {y_f ^ 8'h80, rd_q ^ 8'h80};
				end else begin
					word.kind <= PIX_PAL;
					word.data <= {8'd0, yq};
				end
			end
			default: ;
		endcase
		if (ce_dck)
			pix <= word;
	end
end

endmodule
