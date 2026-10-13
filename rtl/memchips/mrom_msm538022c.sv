// MSM538022C BIOS mask ROM socket: 512K x 16. The contents live in SDRAM
// (banks 0 and 1, rows 512-1535) and pcfx_memory fetches the word when CE
// and OE fall; the data is on D from 7 clk_sys after that, one clock before
// the glue's 4-SCK cycle samples it.

module mrom_msm538022c
(
	input  logic        ce_n,
	input  logic        oe_n,
	input  logic [19:1] a,
	output logic [15:0] d_o,
	output logic        d_oe,

	output logic        mem_ce_n,
	output logic        mem_oe_n,
	output logic [19:1] mem_a,
	input  logic [15:0] mem_q
);

assign mem_ce_n = ce_n;
assign mem_oe_n = oe_n;
assign mem_a    = a;
assign d_o      = mem_q;
assign d_oe     = !ce_n && !oe_n;

endmodule
