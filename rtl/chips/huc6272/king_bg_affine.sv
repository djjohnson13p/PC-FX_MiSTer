// KING BG0 rotation: inverse affine map of each fetched dot (C6272_3 c).
//
//   X1 = A(X2 - X0) + B(Y2 - Y0) + X0     X2 = screen x + BSX
//   Y1 = C(X2 - X0) + D(Y2 - Y0) + Y0     Y2 = raster - 22 + BSY
//
// A-D are 8.8, X0/Y0 integers. Each line's start point (screen x 0) is
// computed in blanking with one multiplier over four clocks; along the
// line +A/+C per dot. Coordinates are 16.8 (the integer part wraps at 16
// bits). During dot kx the outputs give the dot fetched next: (ax, ay) for
// screen x = kx - 7, (bx, by) for x = kx - 9 (the BAT-led CG reads).

module king_bg_affine
(
	input  logic        clk,
	input  logic        ce_osc,
	input  logic        ce_dck,
	input  logic        start,       // registers for the new line are in place
	input  logic  [8:0] kx,
	input  logic  [8:0] raster,

	input  logic [10:0] sx0,
	input  logic [10:0] sy0,
	input  logic [15:0] a,
	input  logic [15:0] b,
	input  logic [15:0] c,
	input  logic [15:0] d,
	input  logic [15:0] x0,
	input  logic [15:0] y0,

	output logic [15:0] ax,
	output logic [15:0] ay,
	output logic [15:0] bx,
	output logic [15:0] by
);

logic  [2:0] cnt;
logic signed [17:0] ma;
logic signed [15:0] mb;
logic [23:0] prod;
logic [23:0] xs, ys, xacc, yacc;
logic [15:0] x1, y1, x2, y2;

// Offsets from the centre, 18 bits signed.
wire signed [17:0] dx = 18'(signed'(sx0)) - 18'(signed'(x0));
wire signed [17:0] dy = 18'(signed'({1'b0, raster}) - 10'sd22) + 18'(signed'(sy0)) - 18'(signed'(y0));

always_comb begin
	ma = (cnt == 3'd1 || cnt == 3'd3) ? dx : dy;
	case (cnt)
		3'd1:    mb = a;
		3'd2:    mb = b;
		3'd3:    mb = c;
		default: mb = d;
	endcase
end

always_ff @(posedge clk) begin
	if (ce_osc) begin
		prod <= 24'(ma * mb);

		if (start)
			cnt <= 3'd1;
		else if (cnt == 3'd6)
			cnt <= 3'd0;
		else if (cnt != 3'd0)
			cnt <= cnt + 3'd1;

		case (cnt)
			3'd2: xs <= prod + {x0, 8'd0};
			3'd3: xs <= xs + prod;
			3'd4: ys <= prod + {y0, 8'd0};
			3'd5: ys <= ys + prod;
			default: ;
		endcase

		if (cnt == 3'd6) begin
			xacc <= xs;
			yacc <= ys;
		end else if (ce_dck && kx >= 9'd7 && kx < 9'd279) begin
			xacc <= xacc + {{8{a[15]}}, a};
			yacc <= yacc + {{8{c[15]}}, c};
		end

		if (ce_dck) begin
			x1 <= xacc[23:8];
			y1 <= yacc[23:8];
			x2 <= x1;
			y2 <= y1;
		end
	end
end

assign ax = xacc[23:8];
assign ay = yacc[23:8];
assign bx = x2;
assign by = y2;

endmodule
