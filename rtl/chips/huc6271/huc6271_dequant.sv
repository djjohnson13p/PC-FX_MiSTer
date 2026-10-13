// HuC6271 dequantiser: coefficient times its Q-table entry scaled by QCx.
//
//   q   = Q * QCx / 4, at most 255, 0 becomes 1; the C table's DC entry
//         always uses QCx = 1
//   out = coefficient * q, clipped to 12 bits
//
// Q tables: 128 bytes, Y 00-3F then C 40-7F, row * 8 + column. Three
// clocks from coefficient to coefficient RAM write; the block-end mark
// travels with them.

module huc6271_dequant
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce,

	input  logic  [3:0] qcx,
	input  logic        qw_v,
	input  logic  [6:0] qw_a,
	input  logic  [7:0] qw_d,

	input  logic        cw_v,
	input  logic        cw_c,
	input  logic  [5:0] cw_idx,
	input  logic [11:0] cw_val,
	input  logic        start_i,    // block marks, carried with the data
	input  logic        end_i,
	input  logic        buf_i,
	input  logic  [3:0] mb_i,
	input  logic  [2:0] blk_i,

	output logic        dq_v,
	output logic  [5:0] dq_idx,
	output logic [11:0] dq_val,
	output logic        start_o,
	output logic        end_o,
	output logic        buf_o,
	output logic  [3:0] mb_o,
	output logic  [2:0] blk_o,
	output logic        busy,

	input  logic        ss_sel,
	input  logic  [6:0] ss_addr,
	input  logic        ss_we,
	input  logic  [7:0] ss_wdata,
	output logic  [7:0] ss_rdata
);

logic  [7:0] qt;
logic        v1, v2, c1, e1, e2, s1, s2, b1, b2;
logic  [6:0] t1, t2;
logic  [5:0] i1, i2;
logic [11:0] val1;
logic signed [11:0] val2;
logic  [7:0] qe;
logic signed [21:0] prod;

cache_ram #(.ADDR_WIDTH(7), .DATA_WIDTH(8)) u_qtab (
	.clk_i   (clk),
	.addr_i  (ss_sel ? ss_addr : qw_v ? qw_a : {cw_c, cw_idx}),
	.wren_i  (ss_sel ? ss_we : ce && qw_v),
	.wdata_i (ss_sel ? ss_wdata : qw_d),
	.q_o     (qt)
);

assign ss_rdata = qt;

// Q * M by shift and add, M = QCx or 1.
wire [3:0]  m  = c1 && i1 == 6'd0 ? 4'd1 : qcx;
/* verilator lint_off UNUSEDSIGNAL */
wire [11:0] qm = (m[0] ? {4'd0, qt}        : 12'd0)
               + (m[1] ? {3'd0, qt, 1'b0}  : 12'd0)
               + (m[2] ? {2'd0, qt, 2'b0}  : 12'd0)
               + (m[3] ? {1'b0, qt, 3'b0}  : 12'd0);
/* verilator lint_on UNUSEDSIGNAL */
wire [9:0]  q4 = qm[11:2];
wire [7:0]  qs = q4 > 10'd255 ? 8'd255 : q4 == 10'd0 ? 8'd1 : q4[7:0];

always_ff @(posedge clk) begin
	if (reset) begin
		{v1, v2, dq_v, e1, e2, end_o, s1, s2, start_o} <= '0;
	end else if (ce) begin
		// Stage 1: Q-table read
		v1   <= cw_v;
		c1   <= cw_c;
		i1   <= cw_idx;
		val1 <= cw_val;
		e1   <= end_i;
		s1   <= start_i;
		b1   <= buf_i;
		t1   <= {mb_i, blk_i};
		// Stage 2: scaled q
		v2   <= v1;
		i2   <= i1;
		val2 <= val1;
		qe   <= qs;
		e2   <= e1;
		s2   <= s1;
		b2   <= b1;
		t2   <= t1;
		// Stage 3: product, clip
		prod   <= val2 * $signed({1'b0, qe});
		dq_v   <= v2;
		dq_idx <= i2;
		end_o  <= e2;
		start_o <= s2;
		buf_o  <= b2;
		{mb_o, blk_o} <= t2;
	end
end

assign dq_val = prod > 22'sd2047 ? 12'h7FF : prod < -22'sd2048 ? 12'h800 : prod[11:0];
assign busy   = v1 || v2 || dq_v || e1 || e2 || end_o || s1 || s2 || start_o;

endmodule
