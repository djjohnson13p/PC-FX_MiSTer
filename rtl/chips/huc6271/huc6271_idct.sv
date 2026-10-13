// HuC6271 inverse DCT: an even/odd matrix product on one sum-of-two
// multiplier pair, rows (pass 1) then columns (pass 2), 263 clocks a block.
//
//   E[x] = sum over even k of c(k,x) X[k]     out[x]   = E[x] + O[x]
//   O[x] = sum over odd k  of c(k,x) X[k]     out[7-x] = E[x] - O[x]
//
// Four clocks an output pair, two products a clock: k 0+2, 4+6, 1+3, 5+7.
// So inputs with k bit 1 clear and set live in separate RAMs. Coefficients
// arrive in one of two buffers with a bit per entry that says it was
// written; unwritten entries read as 0. Pass 1 writes a transpose RAM pair
// split the same way by row, pass 2 reads it and sends two pixels every
// four clocks, two clocks apart. Pass 2 waits seven clocks for pass 1's
// last writes.

module huc6271_idct
	import huc6271_pkg::*;
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce,
	input  logic        init,

	input  logic        dq_v,
	input  logic        dq_buf,
	input  logic  [5:0] dq_idx,
	input  logic [11:0] dq_val,
	input  logic        dq_start,   // the buffer starts a new block
	input  logic        dq_end,     // ... and is complete
	input  logic  [3:0] dq_mb,
	input  logic  [2:0] dq_blk,

	output logic  [1:0] full,

	output logic        px_v,
	output logic  [3:0] px_mb,
	output logic  [2:0] px_blk,
	output logic  [2:0] px_y,
	output logic  [2:0] px_x,
	output logic  [7:0] px_val,

	output logic        busy
);

typedef enum logic [1:0] { I_IDLE, I_P1, I_GAP, I_P2 } istate_t;

istate_t     is;
logic  [6:0] cnt;               // pass 1 {row, x, j}; pass 2 {column, y, j}
logic        rb;                // buffer being transformed
logic [1:0][63:0] vmask;
logic [1:0][6:0]  info;         // {mb, blk} per buffer
logic  [6:0] cur1, cur2;         // block in pass 1, in pass 2

// ------------------------------------------------------------ RAMs

logic [11:0] cl_q, ch_q;
logic [17:0] tl_q, th_q;
logic        tw_v;
logic  [4:0] tw_a;
logic        tw_hi;
logic [17:0] tw_d;

wire       issue = is == I_P1 || is == I_P2;
wire [2:0] c_hi  = cnt[6:4];
wire [1:0] c_x   = cnt[3:2];
wire [1:0] c_j   = cnt[1:0];
// This clock's k pair: {c_j[0], 0/1, c_j[1]}; RAM index {k[2], k[0]}.
wire [1:0] c_k   = {c_j[0], c_j[1]};

/* verilator lint_off PINCONNECTEMPTY */
// Coefficient F(row, k) at {buffer, row, k[2], k[0]}, RAM by k[1].
cache_ram_dp #(.ADDR_WIDTH(6), .DATA_WIDTH(12)) u_clo (
	.clk_i (clk),
	.addr_a_i ({dq_buf, dq_idx[5:3], dq_idx[2], dq_idx[0]}), .wren_a_i (ce && dq_v && !dq_idx[1]),
	.wdata_a_i (dq_val), .q_a_o (),
	.addr_b_i ({rb, c_hi, c_k}), .wren_b_i (1'b0), .wdata_b_i (12'd0), .q_b_o (cl_q)
);

cache_ram_dp #(.ADDR_WIDTH(6), .DATA_WIDTH(12)) u_chi (
	.clk_i (clk),
	.addr_a_i ({dq_buf, dq_idx[5:3], dq_idx[2], dq_idx[0]}), .wren_a_i (ce && dq_v && dq_idx[1]),
	.wdata_a_i (dq_val), .q_a_o (),
	.addr_b_i ({rb, c_hi, c_k}), .wren_b_i (1'b0), .wdata_b_i (12'd0), .q_b_o (ch_q)
);

// Transpose: T(row, column) at {row[2], row[0], column}, RAM by row[1].
cache_ram_dp #(.ADDR_WIDTH(5), .DATA_WIDTH(18)) u_tlo (
	.clk_i (clk),
	.addr_a_i (tw_a), .wren_a_i (ce && tw_v && !tw_hi), .wdata_a_i (tw_d), .q_a_o (),
	.addr_b_i ({c_k, c_hi}), .wren_b_i (1'b0), .wdata_b_i (18'd0), .q_b_o (tl_q)
);

cache_ram_dp #(.ADDR_WIDTH(5), .DATA_WIDTH(18)) u_thi (
	.clk_i (clk),
	.addr_a_i (tw_a), .wren_a_i (ce && tw_v && tw_hi), .wdata_a_i (tw_d), .q_a_o (),
	.addr_b_i ({c_k, c_hi}), .wren_b_i (1'b0), .wdata_b_i (18'd0), .q_b_o (th_q)
);
/* verilator lint_on PINCONNECTEMPTY */

// ------------------------------------------------------------ control

always_ff @(posedge clk) begin
	if (reset || init) begin
		is    <= I_IDLE;
		full  <= 2'b00;
		rb    <= 1'b0;
		vmask <= '0;
	end else if (ce) begin
		if (dq_start) begin
			vmask[dq_buf] <= 64'd0;
			info[dq_buf]  <= {dq_mb, dq_blk};
		end
		if (dq_v)
			vmask[dq_buf][dq_idx] <= 1'b1;
		if (dq_end)
			full[dq_buf] <= 1'b1;

		case (is)
			I_IDLE: if (full[rb]) begin
				cur1 <= info[rb];
				cnt <= 7'd0;
				is  <= I_P1;
			end
			I_P1: begin
				cnt <= cnt + 7'd1;
				if (cnt == 7'd127) begin
					full[rb] <= 1'b0;
					rb       <= !rb;
					cnt      <= 7'd0;
					is       <= I_GAP;
				end
			end
			I_GAP: begin
				cnt <= cnt + 7'd1;
				if (cnt == 7'd6) begin
					cnt  <= 7'd0;
					cur2 <= cur1;
					is   <= I_P2;
				end
			end
			I_P2: begin
				cnt <= cnt + 7'd1;
				if (cnt == 7'd127) begin
					if (full[rb]) begin
						cur1 <= info[rb];
						cnt <= 7'd0;
						is  <= I_P1;
					end else begin
						is  <= I_IDLE;
					end
				end
			end
		endcase
	end
end

// ------------------------------------------------------------ datapath

// Stage tags: valid, pass 2, the row/column (hi), x or y, j
logic        v1, v2, v3, v4;
logic        p1, p2, p3, p4;
logic  [2:0] h1, h2, h3, h4;
logic  [1:0] x1, x2, x3, x4;
logic  [1:0] j1, j2, j3;
logic        m1l, m1h;
logic signed [12:0] k1l, k1h, k2l, k2h;
logic signed [17:0] a2l, a2h;
logic signed [31:0] m3;
logic signed [33:0] acc_e, acc_o;

wire [5:0] vi = {c_hi, c_j[0], 1'b0, c_j[1]};

always_ff @(posedge clk) begin
	if (reset || init) begin
		v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0; v4 <= 1'b0;
	end else if (ce) begin
		// 1: RAM address out, mask bits and coefficients
		v1  <= issue;
		p1  <= is == I_P2;
		h1  <= c_hi;
		x1  <= c_x;
		j1  <= c_j;
		m1l <= is == I_P2 || vmask[rb][vi];
		m1h <= is == I_P2 || vmask[rb][vi | 6'd2];
		k1l <= idct_k({c_j[0], 1'b0, c_j[1]}, c_x);
		k1h <= idct_k({c_j[0], 1'b1, c_j[1]}, c_x);
		// 2: operands
		v2  <= v1;
		p2  <= p1;
		h2  <= h1;
		x2  <= x1;
		j2  <= j1;
		k2l <= k1l;
		k2h <= k1h;
		a2l <= !m1l ? 18'sd0 : p1 ? $signed(tl_q) : 18'(signed'(cl_q));
		a2h <= !m1h ? 18'sd0 : p1 ? $signed(th_q) : 18'(signed'(ch_q));
		// 3: products
		v3  <= v2;
		p3  <= p2;
		h3  <= h2;
		x3  <= x2;
		j3  <= j2;
		m3  <= a2l * k2l + a2h * k2h;
		// 4: E from j 0-1, O from j 2-3; the pair is complete after j = 3
		v4  <= v3 && j3 == 2'd3;
		p4  <= p3;
		h4  <= h3;
		x4  <= x3;
		if (v3) begin
			if (!j3[1])
				acc_e <= (j3[0] ? acc_e : 34'sd0) + 34'(m3);
			else
				acc_o <= (j3[0] ? acc_o : 34'sd0) + 34'(m3);
		end
	end
end

// 5: outputs out[x] and out[7-x], rounded once here
wire signed [33:0] s_a = acc_e + acc_o;
wire signed [33:0] s_b = acc_e - acc_o;
wire signed [17:0] r1a = 18'((s_a + 34'sd512) >>> IDCT_SH1);
wire signed [17:0] r1b = 18'((s_b + 34'sd512) >>> IDCT_SH1);
wire signed [33:0] r2a = (s_a + 34'sd8192) >>> IDCT_SH2;
wire signed [33:0] r2b = (s_b + 34'sd8192) >>> IDCT_SH2;

function automatic logic [7:0] clamp8(input logic signed [33:0] s);
	return s > 34'sd127 ? 8'h7F : s < -34'sd128 ? 8'h80 : s[7:0];
endfunction

logic        o_v, o_p2, o_second;
logic  [2:0] o_h;
logic  [1:0] o_x;
logic [17:0] o_b;
logic  [7:0] o_pb;

always_ff @(posedge clk) begin
	if (reset || init) begin
		o_v      <= 1'b0;
		o_second <= 1'b0;
		tw_v     <= 1'b0;
		px_v     <= 1'b0;
	end else if (ce) begin
		tw_v     <= 1'b0;
		px_v     <= 1'b0;
		o_second <= 1'b0;
		if (v4) begin
			o_v  <= 1'b1;
			o_p2 <= p4;
			o_h  <= h4;
			o_x  <= x4;
			o_b  <= r1b;
			o_pb <= clamp8(r2b);
			if (!p4) begin
				// T(row, x) now, T(row, 7 - x) next clock
				tw_v   <= 1'b1;
				tw_hi  <= h4[1];
				tw_a   <= {h4[2], h4[0], 1'b0, x4};
				tw_d   <= r1a;
			end else begin
				px_v   <= 1'b1;
				px_y   <= {1'b0, x4};
				px_x   <= h4;
				px_val <= clamp8(r2a);
			end
		end else if (o_v) begin
			o_v      <= 1'b0;
			o_second <= o_p2;
			if (!o_p2) begin
				tw_v <= 1'b1;
				tw_a <= {o_h[2], o_h[0], 1'b1, ~o_x};
				tw_d <= o_b;
			end
		end else if (o_second) begin
			px_v   <= 1'b1;
			px_y   <= {1'b1, ~o_x};
			px_x   <= o_h;
			px_val <= o_pb;
		end
	end
end

assign px_mb  = cur2[6:3];
assign px_blk = cur2[2:0];
assign busy   = is != I_IDLE || full != 2'b00 || v1 || v2 || v3 || v4 || o_v || o_second || px_v;

endmodule
