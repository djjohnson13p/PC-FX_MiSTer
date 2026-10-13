// HuC6261 output stage in place of the Y/U/V DACs: holds each mixed dot,
// delays the unshifted lines of half-dot mode by half a dot, and shows
// black outside the display.

module huc6261_yuv_out
(
	input  logic        clk,
	input  logic        ce_master,
	input  logic        dc7,
	input  logic  [1:0] q,
	input  logic [23:0] res_256,
	input  logic [23:0] res_7,
	input  logic        half_delay,
	input  logic        de,

	output logic  [7:0] y,
	output logic  [7:0] u,
	output logic  [7:0] v
);

import huc6261_pkg::*;

logic [23:0] pix, d1, d2;

always_ff @(posedge clk) begin
	if (ce_master) begin
		if (dc7)
			pix <= res_7;
		else if (q == 2'd1)
			pix <= res_256;
		d1 <= pix;
		d2 <= d1;
	end
end

wire [23:0] o = !de ? BLACK : half_delay ? d2 : pix;

assign y = o[23:16];
assign u = o[15:8];
assign v = o[7:0];

endmodule
