// HSRM2264 R-RAM socket: 8K x 8 static RAM in block RAM (CS2 tied high).
//
// The RAM samples A every clock, so read data is on D one clock after the
// address settles. A write happens on every clock /WE is low.

module sram_2264
(
	input  logic        clk,

	input  logic        cs_n,
	input  logic        oe_n,
	input  logic        we_n,
	input  logic [12:0] a,
	input  logic  [7:0] d_i,
	output logic  [7:0] d_o,
	output logic        d_oe
);

cache_ram #(.ADDR_WIDTH(13), .DATA_WIDTH(8)) u_ram (
	.clk_i   (clk),
	.addr_i  (a),
	.wren_i  (!cs_n && !we_n),
	.wdata_i (d_i),
	.q_o     (d_o)
);

assign d_oe = !cs_n && !oe_n && we_n;

endmodule
