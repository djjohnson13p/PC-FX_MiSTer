// PC-FX mouse on a K-port connector.
//
// Motion adds up between scans. The LATCH fall takes up to +-127 counts per
// axis into the frame and leaves the rest for the next scan, with the
// buttons. Shifting and DATA as on the pad.
//
//   bit  31-28  17     16    15-8          7-0
//        1101   right  left  X (right +)   Y (down +)

module kport_mouse
	import kport_pkg::*;
#(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,

	input  logic        latch_n,
	input  logic        kclk,
	input  logic        rw,
	output logic        data_o,
	output logic        data_oe,

	input  logic        move,           // one clock per motion report
	input  logic  [8:0] dx,
	input  logic  [8:0] dy,
	input  logic        btn_l,
	input  logic        btn_r,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

mouse_state_t st;

function automatic logic [7:0] clamp127(input logic [9:0] v);
	if (!v[9] && v[8:7] != 2'b00)
		clamp127 = 8'h7F;
	else if (v[9] && v[8:7] != 2'b11)
		clamp127 = 8'h81;
	else
		clamp127 = v[7:0];
endfunction

function automatic logic [9:0] sat10(input logic [11:0] v);
	if (!v[11] && v[10:9] != 2'b00)
		sat10 = 10'h1FF;
	else if (v[11] && v[10:9] != 2'b11)
		sat10 = 10'h201;
	else
		sat10 = v[9:0];
endfunction

wire       take = st.latch_q && !latch_n;
wire [7:0] sx   = clamp127(st.dx);
wire [7:0] sy   = clamp127(st.dy);

wire [11:0] nx = {{2{st.dx[9]}}, st.dx} + (move ? {{3{dx[8]}}, dx} : 12'd0) - (take ? {{4{sx[7]}}, sx} : 12'd0);
wire [11:0] ny = {{2{st.dy[9]}}, st.dy} + (move ? {{3{dy[8]}}, dy} : 12'd0) - (take ? {{4{sy[7]}}, sy} : 12'd0);

always_ff @(posedge clk) begin
	if (reset) begin
		st         <= '0;
		st.latch_q <= 1'b1;
		st.clk_q   <= 1'b1;
	end else if (SAVESTATE && ss_shift) begin
		st <= mouse_state_t'({st[MOUSE_BITS-2:0], ss_si});
	end else begin
		st.latch_q <= latch_n;
		st.clk_q   <= kclk;
		st.dx      <= sat10(nx);
		st.dy      <= sat10(ny);
		if (take)
			st.snap <= {btn_r, btn_l, sx, sy};
		if (!latch_n)
			st.bit_n <= 5'd0;
		else if (kclk && !st.clk_q)
			st.bit_n <= st.bit_n + 5'd1;
	end
end

assign ss_so = SAVESTATE ? st[MOUSE_BITS-1] : ss_si;

logic bit_v;
always_comb begin
	if (st.bit_n < 5'd18)
		bit_v = st.snap[st.bit_n];
	else
		bit_v = st.bit_n == 5'd28 || st.bit_n == 5'd30 || st.bit_n == 5'd31;
end

assign data_o  = !bit_v;
assign data_oe = !rw;

endmodule
