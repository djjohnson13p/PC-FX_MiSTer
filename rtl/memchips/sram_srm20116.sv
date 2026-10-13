// HSRM20116 VRAM socket: 64K x 16 static RAM in block RAM.
//
// The RAM samples A every clock, so read data is on D one clock after the
// address settles, well inside a DCK. A write happens on every clock /WE is
// low, so a strobe held for a whole dot lands as one write.

module sram_srm20116
(
	input  logic        clk,

	input  logic        cs_n,
	input  logic        oe_n,
	input  logic        we_n,
	input  logic [15:0] a,
	input  logic [15:0] d_i,
	output logic [15:0] d_o,
	output logic        d_oe
);

cache_ram #(.ADDR_WIDTH(16), .DATA_WIDTH(16)) u_ram (
	.clk_i   (clk),
	.addr_i  (a),
	.wren_i  (!cs_n && !we_n),
	.wdata_i (d_i),
	.q_o     (d_o)
);

assign d_oe = !cs_n && !oe_n && we_n;

endmodule
