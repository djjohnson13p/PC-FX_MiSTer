// HuC6271 macroblock assembly: decoded pixels, null runs and palette runs
// to R-RAM writes for the strip being decoded.
//
// R-RAM line l (512 bytes) holds pairs of dots as Y0 Y1 U V:
//   Y of dot d  at {l, d / 2, 0, d & 1}
//   U, V of d   at {l, d / 2, 1, 0 / 1}   (4:2:0: each chroma row on two lines)
// A palette strip is one byte a dot from address 0, line after line.
//
// Filter A (FTS): the top row of a lower Y block is averaged with the
// bottom row of the block above it.

module huc6271_mb_asm
	import huc6271_pkg::*;
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce,
	input  logic        start,
	input  logic        fts,
	input  logic  [7:0] nry,
	input  logic  [7:0] nru,
	input  logic  [7:0] nrv,

	input  logic        px_v,
	input  logic  [3:0] px_mb,
	input  logic  [2:0] px_blk,
	input  logic  [2:0] px_y,
	input  logic  [2:0] px_x,
	input  logic  [7:0] px_val,

	input  logic        nrl_v,
	input  logic  [3:0] nrl_mb,
	input  logic  [4:0] nrl_n,

	input  logic        run_v,
	input  logic  [6:0] run_idx,
	input  logic  [8:0] run_len,
	output logic        run_rdy,
	output logic        rle_full,

	output logic        wr_v,
	output logic [12:0] wr_a,
	output logic  [7:0] wr_d,
	output logic        busy
);

logic [7:0][7:0] row7;
logic        c2_v;          // second line of a chroma sample
logic [12:0] c2_a;
logic  [7:0] c2_d;

logic        nf;            // null-run fill
logic  [3:0] nf_mb;
logic  [4:0] nf_left;
logic  [8:0] nf_i;          // {line, pair, byte}

logic        rf;            // palette run
logic  [8:0] rf_left;
logic  [6:0] rf_idx;
logic [12:0] rpos;

wire       luma  = !px_blk[2];
wire [3:0] y_l   = {px_blk[0], px_y};
wire [6:0] y_p   = {px_mb, px_blk[1], px_x[2:1]};
wire [6:0] c_p   = {px_mb, px_x};
wire       fa    = fts && luma && px_blk[0] && px_y == 3'd0;
wire [7:0] y_val = fa ? fts_avg(row7[px_x], px_val) : px_val;

always_ff @(posedge clk) begin
	if (reset || start) begin
		wr_v <= 1'b0;
		c2_v <= 1'b0;
		nf   <= 1'b0;
		rf   <= 1'b0;
		rpos <= 13'd0;
	end else if (ce) begin
		wr_v <= 1'b0;
		c2_v <= 1'b0;

		if (px_v) begin
			wr_v <= 1'b1;
			if (luma) begin
				wr_a <= {y_l, y_p, 1'b0, px_x[0]};
				wr_d <= y_val;
				if (!px_blk[0] && px_y == 3'd7)
					row7[px_x] <= px_val;
			end else begin
				wr_a <= {px_y, 1'b0, c_p, 1'b1, px_blk[0]};
				wr_d <= px_val;
				c2_v <= 1'b1;
				c2_a <= {px_y, 1'b1, c_p, 1'b1, px_blk[0]};
				c2_d <= px_val;
			end
		end else if (c2_v) begin
			wr_v <= 1'b1;
			wr_a <= c2_a;
			wr_d <= c2_d;
		end

		if (nrl_v) begin
			nf      <= 1'b1;
			nf_mb   <= nrl_mb;
			nf_left <= nrl_n;
			nf_i    <= 9'd0;
		end else if (nf) begin
			wr_v <= 1'b1;
			wr_a <= {nf_i[8:5], nf_mb, nf_i[4:0]};
			wr_d <= !nf_i[1] ? nry : nf_i[0] ? nrv : nru;
			nf_i <= nf_i + 9'd1;
			if (nf_i == 9'd511) begin
				nf_mb   <= nf_mb + 4'd1;
				nf_left <= nf_left - 5'd1;
				if (nf_left == 5'd1)
					nf <= 1'b0;
			end
		end

		if (run_v) begin
			rf      <= 1'b1;
			rf_left <= run_len;
			rf_idx  <= run_idx;
		end else if (rf) begin
			if (rpos[12]) begin
				rf <= 1'b0;
			end else begin
				wr_v    <= 1'b1;
				wr_a    <= {1'b0, rpos[11:0]};
				wr_d    <= {1'b0, rf_idx};
				rpos    <= rpos + 13'd1;
				rf_left <= rf_left - 9'd1;
				if (rf_left == 9'd1)
					rf <= 1'b0;
			end
		end
	end
end

assign run_rdy  = !rf && !run_v;
assign rle_full = rpos[12];
assign busy     = c2_v || nf || nrl_v || rf || run_v;

endmodule
