// HuC6261 7 MHz mode (DC7 = 1): every master clock shows the front-most
// layer that is not key, with no cellophane of any kind. One palette
// lookup per master clock; palette 0 when every layer is key.

module huc6261_hd_path
(
	input  huc6261_pkg::cand_t cand [3],
	input  logic  [1:0] order [3],
	input  logic  [6:0] blank,

	output logic  [7:0] look_idx,
	output logic  [7:0] look_off,
	input  logic [15:0] q_b,

	output logic [23:0] res
);

import huc6261_pkg::*;

logic  any;
/* verilator lint_off UNUSEDSIGNAL */
cand_t w;
/* verilator lint_on UNUSEDSIGNAL */

always_comb begin
	any = 1'b1;
	if (cand[order[2]].pres)
		w = cand[order[2]];
	else if (cand[order[1]].pres)
		w = cand[order[1]];
	else begin
		w = cand[order[0]];
		any = cand[order[0]].pres;
	end
end

assign look_idx = any ? w.idx : 8'd0;
assign look_off = any ? w.off : 8'd0;
assign res = blank == 7'd0 ? BLACK : any && !w.is_pal ? w.yuv : expand(q_b);

endmodule
