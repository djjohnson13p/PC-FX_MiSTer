// KING DRAM control for one KRAM bank (a uPD424260, 256K x 16). One RAS
// cycle per dot, given to the socket at the dot start as {row, column,
// write, data}; read data comes back on kram_q from 8 clk_sys later.
//
// 4M mode: row = {page, A16:A9}, column = A8:A0. 1M mode (the reset
// default): page and A16 do not exist, row = A15:A8, column = A7:A0.

module king_kram_ctl
	import huc6272_pkg::*;
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_dck,
	input  logic        mode4m,
	input  kreq_t       sel,

	output logic        kram_req,
	output logic        kram_we,
	output logic  [8:0] kram_row,
	output logic  [8:0] kram_col,
	output logic  [1:0] kram_be,
	output logic [15:0] kram_d
);

always_ff @(posedge clk) begin
	if (reset) begin
		kram_req <= 1'b0;
	end else if (ce_dck) begin
		kram_req <= sel.v;
		kram_we  <= sel.we;
		kram_d   <= sel.d;
		kram_be  <= 2'b11;
		if (mode4m) begin
			kram_row <= {sel.page, sel.a[16:9]};
			kram_col <= sel.a[8:0];
		end else begin
			kram_row <= {1'b0, sel.a[15:8]};
			kram_col <= {1'b0, sel.a[7:0]};
		end
	end
end

endmodule
