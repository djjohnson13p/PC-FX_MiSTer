// Interrupt controller.
//
//   E00 R  ISR   D7 AER, D6 TM, D5 EX, D4 KP, D3 7A, D2 72, D1 7B, D0 73
//   E00 W        clears AER (data ignored)
//   E40    IMR   D6-0, 1 = masked, reset 7F
//   E80    ILR0  7A D11-9, 72 D8-6, 7B D5-3, 73 D2-0, reset 977
//   EC0    ILR1  TM D8-6, EX D5-3, KP D2-0, reset 053
//
// Sources are levels held by their chips and shown live. Levels are
// writable only while every source is masked. INT goes with the highest
// level among unmasked requests, INTV = 8 + level. AER latches ADRSERR and
// holds NMI low until cleared.

module ga_itc
	import upd65641_pkg::*;
#(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_sys,
	input  logic        ce_sck,

	input  logic        we,
	input  logic  [1:0] sel,
	input  logic [11:0] wd,
	output logic [15:0] rd,

	input  logic  [6:0] src,
	input  logic        adrserr_n,

	output logic        intr,
	output logic  [3:0] intv_n,
	output logic        nmi_n,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

itc_state_t st;

wire [6:0] pend = src & ~st.imr;

// Highest level with an unmasked request.
logic [7:0] at_level;
logic [2:0] top;
wire [20:0] levels = {st.ilr1, st.ilr0};
always_comb begin
	at_level = 8'd0;
	for (int i = 0; i < 7; i++)
		if (pend[i])
			at_level[levels[3*i +: 3]] = 1'b1;
	top = 3'd0;
	for (int l = 1; l < 8; l++)
		if (at_level[l])
			top = 3'(l);
end

always_ff @(posedge clk) begin
	if (reset) begin
		st      <= '0;
		st.imr  <= 7'h7F;
		st.ilr0 <= 12'h977;
		st.ilr1 <= 9'h053;
	end else if (SAVESTATE && ss_shift) begin
		st <= itc_state_t'({st[ITC_BITS-2:0], ss_si});
	end else if (ce_sys) begin
		// INT and INTV change only on CPU clock edges.
		if (ce_sck) begin
			st.intr <= |pend;
			st.lvl  <= top;
		end

		if (we) begin
			case (sel)
				2'd0: st.aer <= 1'b0;
				2'd1: st.imr <= wd[6:0];
				2'd2: if (&st.imr) st.ilr0 <= wd;
				default: if (&st.imr) st.ilr1 <= wd[8:0];
			endcase
		end

		if (ce_sck && !adrserr_n)
			st.aer <= 1'b1;
	end
end

assign ss_so = SAVESTATE ? st[ITC_BITS-1] : ss_si;

always_comb begin
	case (sel)
		2'd0:    rd = {8'd0, st.aer, src};
		2'd1:    rd = {9'd0, st.imr};
		2'd2:    rd = {4'd0, st.ilr0};
		default: rd = {7'd0, st.ilr1};
	endcase
end

assign intr   = st.intr;
assign intv_n = ~{1'b1, st.lvl};
assign nmi_n  = !st.aer;

endmodule
