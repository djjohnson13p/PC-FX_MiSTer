// uPD43256 backup SRAM socket: 32K x 8 in block RAM, on LD7:0.
//
// The RAM samples A every clock, so read data is on D one clock after the
// address settles, well inside the glue's 4-clock cycle. A write happens on
// every clock /WE is low, so a strobe held for several clocks lands as one
// write. The RAM's second port is ss_mem_*: backup files read it while the
// game runs and savestates use it while frozen, in every build. A byte the
// CPU writes in the same clock that port reads it may read either way.

module sram_upd43256 #(
	/* verilator lint_off UNUSEDPARAM */
	parameter bit SAVESTATE = 1'b1
	/* verilator lint_on UNUSEDPARAM */
)
(
	input  logic        clk,

	input  logic        cs_n,
	input  logic        oe_n,
	input  logic        we_n,
	input  logic [14:0] a,
	input  logic  [7:0] d_i,
	output logic  [7:0] d_o,
	output logic        d_oe,

	input  logic        ss_mem_sel,
	input  logic [14:0] ss_mem_addr,
	input  logic        ss_mem_we,
	input  logic  [7:0] ss_mem_wdata,
	output logic  [7:0] ss_mem_rdata
);

cache_ram_dp #(.ADDR_WIDTH(15), .DATA_WIDTH(8)) u_ram (
	.clk_i     (clk),
	.addr_a_i  (a),
	.wren_a_i  (!cs_n && !we_n),
	.wdata_a_i (d_i),
	.q_a_o     (d_o),
	.addr_b_i  (ss_mem_addr),
	.wren_b_i  (ss_mem_sel && ss_mem_we),
	.wdata_b_i (ss_mem_wdata),
	.q_b_o     (ss_mem_rdata)
);

assign d_oe = !cs_n && !oe_n && we_n;

endmodule
