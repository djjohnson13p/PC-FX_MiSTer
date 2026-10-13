// HuC6261 colour palette RAM, 512 x Y8 U4 V4, sharing one dual-port block
// with the R04-R15 register file (512 + register number).
//
// Port A: CPU, the reset clear, and the savestate engine while frozen.
// Port B: display lookups on the clk_sys after each master clock, and the
// once-per-line register copy, which takes priority.

module huc6261_pal_lookup #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,

	input  logic  [9:0] cpu_addr,
	input  logic        cpu_we,
	input  logic [15:0] cpu_wdata,
	output logic [15:0] q_a,

	input  logic  [7:0] disp_idx,    // palette data from the source
	input  logic  [7:0] disp_off,    // that plane's offset register
	input  logic        copy,
	input  logic  [4:0] copy_word,
	output logic [15:0] q_b,

	input  logic        ss_mem_sel,
	input  logic  [9:0] ss_mem_addr,
	input  logic        ss_mem_we,
	input  logic [15:0] ss_mem_wdata
);

// Reset clears palette 0 and R04-R15; R0F (SPBL) becomes 00FFh.
logic [4:0] clr;
always_ff @(posedge clk)
	clr <= reset ? clr + 5'd1 : 5'd0;

logic  [9:0] a_addr;
logic        a_we;
logic [15:0] a_wdata;

always_comb begin
	if (SAVESTATE && ss_mem_sel) begin
		a_addr  = ss_mem_addr;
		a_we    = ss_mem_we;
		a_wdata = ss_mem_wdata;
	end else if (reset) begin
		a_addr  = clr == 5'd0 ? 10'd0 : {5'b10000, clr};
		a_we    = 1'b1;
		a_wdata = clr == 5'h0F ? 16'h00FF : 16'h0000;
	end else begin
		a_addr  = cpu_addr;
		a_we    = cpu_we;
		a_wdata = cpu_wdata;
	end
end

// Palette address = data + 2 x offset, wrapping at 512.
wire [8:0] disp_addr = {1'b0, disp_idx} + {disp_off, 1'b0};
wire [9:0] b_addr = copy ? {5'b10000, copy_word} : {1'b0, disp_addr};

cache_ram_dp #(
	.ADDR_WIDTH (10),
	.DATA_WIDTH (16)
) ram (
	.clk_i     (clk),
	.addr_a_i  (a_addr),
	.wren_a_i  (a_we),
	.wdata_a_i (a_wdata),
	.q_a_o     (q_a),
	.addr_b_i  (b_addr),
	.wren_b_i  (1'b0),
	.wdata_b_i (16'd0),
	.q_b_o     (q_b)
);

endmodule
