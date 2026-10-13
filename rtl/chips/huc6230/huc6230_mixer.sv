// HuC6230 output mix, the digital stand-in for SLOUT/SROUT. Every fourth
// fmaster clock starts a pass of 18 products through one multiplier, one
// per clock: six PSG voices L and R, ADPCM #1 and #2 L and R, LINE L and R.
// Mix ratio ADPCM#1 : ADPCM#2 : LINE : PSG = 1 : 1 : 1 : 2. Each ADPCM
// channel and LINE at full scale give half the 16-bit range, all six PSG
// voices at full scale the whole range (assumption: the chip's absolute
// source levels are not documented). The sum saturates to signed 16 bits.
//
//   clock    0   1   2   3 ...  17  18  19  20  21
//   voice    i0  i1  i2 ...
//   operand      i0  i1  i2 ...     i17
//   product          i0  i1 ...         i17
//   sum                  i0 ...             i17 out

module huc6230_mixer #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_fm,

	output logic  [2:0] v_ch,
	input  logic signed [5:0] v_samp,
	input  logic  [4:0] v_att_l,
	input  logic  [4:0] v_att_r,
	input  logic        v_sil_l,
	input  logic        v_sil_r,

	input  logic [1:0][9:0]  dac,
	input  logic [3:0][15:0] gain,
	input  logic [1:0][9:0]  level,
	input  logic [1:0][15:0] line,

	output logic [15:0] slout,
	output logic [15:0] srout,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

import huc6230_pkg::*;

localparam logic [4:0] IDLE = 5'd31;

mix_state_t st;
logic [4:0] ms, i1;

wire start = ce_fm && st.fmc == 2'd3;

always_ff @(posedge clk) begin
	if (reset)
		st <= '0;
	else if (SAVESTATE && ss_shift)
		st <= mix_state_t'({st[SS_MIX_BITS-2:0], ss_si});
	else if (ce_fm)
		st.fmc <= st.fmc + 2'd1;
end

assign ss_so = SAVESTATE ? st[SS_MIX_BITS-1] : ss_si;

always_ff @(posedge clk) begin
	if (reset)
		ms <= IDLE;
	else if (start)
		ms <= 5'd0;
	else if (ms != IDLE)
		ms <= ms + 5'd1;
	i1 <= ms;
end

assign v_ch = ms < 5'd12 ? ms[3:1] : 3'd0;

// Operands for item i1.
logic signed [17:0] a_op, b_op, a_r, b_r;
logic signed [9:0]  psg11;
logic        side1, use1, psg1;

always_comb begin
	side1 = i1[0];
	use1  = i1 < 5'd18;
	psg1  = i1 < 5'd12;
	psg11 = (10'(v_samp) <<< 3) + (10'(v_samp) <<< 1) + 10'(v_samp);   // x 11
	if (psg1) begin
		a_op = (side1 ? v_sil_r : v_sil_l) ? 18'sd0 : 18'(psg11);
		b_op = {2'b00, gain_db15({1'b0, side1 ? v_att_r : v_att_l})};
	end else if (i1 < 5'd16) begin
		a_op = 18'($signed({~dac[i1[1]][9], dac[i1[1]][8:0]}));
		b_op = {2'b00, gain[i1[1:0]]};
	end else begin
		a_op = 18'($signed(line[side1]));
		b_op = {8'd0, level[side1]};
	end
end

logic signed [35:0] p;
logic signed [29:0] acc_l, acc_r, term;
logic        psg2, psg3, side2, side3, use2, use3, start2, start3;

always_ff @(posedge clk) begin
	a_r    <= a_op;
	b_r    <= b_op;
	use2   <= use1;
	psg2   <= psg1;
	side2  <= side1;
	start2 <= i1 < 5'd2;

	p      <= a_r * b_r;
	use3   <= use2;
	psg3   <= psg2;
	side3  <= side2;
	start3 <= start2;

	if (use3) begin
		if (side3)
			acc_r <= (start3 ? 30'sd0 : acc_r) + term;
		else
			acc_l <= (start3 ? 30'sd0 : acc_l) + term;
	end

	if (ms == 5'd21) begin
		slout <= sat16(acc_l);
		srout <= sat16(acc_r);
	end
end

assign term = psg3 ? 30'(p >>> 1) : 30'(p);

function automatic logic [15:0] sat16(input logic signed [29:0] acc);
	logic signed [18:0] s;
	s = 19'(acc >>> 11);
	if (s > 19'sd32767)
		sat16 = 16'h7FFF;
	else if (s < -19'sd32768)
		sat16 = 16'h8000;
	else
		sat16 = s[15:0];
endfunction

endmodule
