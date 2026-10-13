// HuC6271 block-noise filters B, C and D on the luma being displayed.
//
//   B  last line of a strip, averaged with the first line of the next one
//      (the next strip complete, DCT, and shown right after this one)
//   C  first dot of each 8-dot block, averaged with the dot before it
//   D  as C across the left/right join in cylinder mode
//
// C and D come for free: the dot before image dot 0 is image dot 255 in
// cylinder mode and outside the picture otherwise. Filter A is in mb_asm.

module huc6271_fts
	import huc6271_pkg::*;
(
	input  logic       clk,
	input  logic       step,       // one clock per dot, in screen order
	input  logic       on,         // FTS and a DCT strip on display
	input  logic       vert,       // filter B this line
	input  logic       in_area,
	input  logic [2:0] ix_lo,
	input  logic [7:0] y,
	input  logic [7:0] y_next,
	output logic [7:0] y_out
);

logic [7:0] prev;
logic       prev_in;

wire [7:0] yb = on && vert ? fts_avg(y, y_next) : y;

always_ff @(posedge clk) begin
	if (step) begin
		prev    <= yb;
		prev_in <= in_area;
	end
end

assign y_out = on && ix_lo == 3'd0 && prev_in ? fts_avg(prev, yb) : yb;

endmodule
