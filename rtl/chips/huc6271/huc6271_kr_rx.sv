// HuC6271 K-R bus input: a byte is taken from KR7:0 on the rising edge of
// KING's strobe (the last value seen while it was low). Strip data bytes
// from the parser wait in a 128-byte FIFO for the decoder. -REQR asks for
// more while the chip runs, the parser still wants the strip and the FIFO
// has room for what KING may already have on its way.

module huc6271_kr_rx
	import huc6271_pkg::*;
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce,
	input  logic        flush,

	input  logic  [7:0] kr_d,
	input  logic        kr_wr_n,
	output logic        rx_v,
	output logic  [7:0] rx_d,

	input  logic        want,       // running and the parser not done
	output logic        reqr_n,

	input  logic        push,
	input  logic  [7:0] push_d,
	output logic        empty,
	input  logic        pop,        // data on q the next clock
	output logic  [7:0] q
);

logic               wr_q;
logic [FIFO_AW-1:0] wp, rp;
logic [FIFO_AW:0]   cnt;
logic [7:0]         fifo_q, q_hold;
logic               ce_q;

always_ff @(posedge clk) begin
	if (reset) begin
		wr_q <= 1'b1;
		rx_v <= 1'b0;
	end else if (ce) begin
		wr_q <= kr_wr_n;
		rx_v <= !wr_q && kr_wr_n;
		if (!kr_wr_n)
			rx_d <= kr_d;
	end else begin
		rx_v <= 1'b0;
	end
end

wire do_push = push && !cnt[FIFO_AW];
wire do_pop  = pop && cnt != '0;

always_ff @(posedge clk) begin
	if (reset || flush) begin
		wp  <= '0;
		rp  <= '0;
		cnt <= '0;
	end else if (ce) begin
		if (do_push)
			wp <= wp + 1'd1;
		if (do_pop)
			rp <= rp + 1'd1;
		cnt <= cnt + (FIFO_AW+1)'(do_push) - (FIFO_AW+1)'(do_pop);
	end
end

always_ff @(posedge clk) begin
	if (reset)
		reqr_n <= 1'b1;
	else if (ce)
		reqr_n <= !(want && cnt < (FIFO_AW+1)'(FIFO_STOP));
end

/* verilator lint_off PINCONNECTEMPTY */
cache_ram_dp #(.ADDR_WIDTH(FIFO_AW), .DATA_WIDTH(8)) u_fifo (
	.clk_i     (clk),
	.addr_a_i  (wp),
	.wren_a_i  (ce && do_push),
	.wdata_a_i (push_d),
	.q_a_o     (),
	.addr_b_i  (rp),
	.wren_b_i  (1'b0),
	.wdata_b_i (8'd0),
	.q_b_o     (fifo_q)
);
/* verilator lint_on PINCONNECTEMPTY */

// The RAM clocks while ce is low. Keep the response to the last enabled
// read until the decoder can consume it; continuous ce keeps its latency.
always_ff @(posedge clk) begin
	if (reset || flush) begin
		ce_q   <= 1'b0;
		q_hold <= 8'd0;
	end else begin
		ce_q <= ce;
		if (ce_q)
			q_hold <= fifo_q;
	end
end

assign q = ce_q ? fifo_q : q_hold;
assign empty = cnt == '0;

endmodule
