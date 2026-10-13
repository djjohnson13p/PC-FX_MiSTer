// Timer: 16-bit down counter at 42.95454 MHz / 30.
//
//   F00 TCR  D2 TIREQ, D1 TCE, D0 TIEN, reset 0
//   F80 TMR  reload value, reset FFFF (0 is forbidden: it borrows every tick)
//   FC0 DWC  counter, read only, reset FFFF
//
// TCE 0->1 loads DWC from TMR. The tick that would take DWC below 1 is the
// borrow: TIREQ sets and DWC reloads, so one period is TMR ticks. XINTTM =
// TIEN & TCE & TIREQ; writing TIREQ = 0 clears it, all three = 1 forces it.

module ga_tmc
	import upd65641_pkg::*;
#(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_sys,
	input  logic        ce_timer,

	input  logic        we,
	input  logic  [1:0] sel,
	input  logic [15:0] wd,
	output logic [15:0] rd,

	output logic        irq,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

tmc_state_t st;

wire tick   = ce_timer && st.tce;
wire borrow = tick && st.dwc[15:1] == 15'd0;
wire tcr_we = we && sel == 2'd0;

always_ff @(posedge clk) begin
	if (reset) begin
		st     <= '0;
		st.tmr <= 16'hFFFF;
		st.dwc <= 16'hFFFF;
	end else if (SAVESTATE && ss_shift) begin
		st <= tmc_state_t'({st[TMC_BITS-2:0], ss_si});
	end else if (ce_sys) begin
		if (tcr_we && !st.tce && wd[1])
			st.dwc <= st.tmr;
		else if (borrow)
			st.dwc <= st.tmr;
		else if (tick)
			st.dwc <= st.dwc - 16'd1;

		if (tcr_we) begin
			st.tien  <= wd[0];
			st.tce   <= wd[1];
			st.tireq <= wd[2] || borrow;
		end else if (borrow) begin
			st.tireq <= 1'b1;
		end

		if (we && sel == 2'd2)
			st.tmr <= wd;
	end
end

assign ss_so = SAVESTATE ? st[TMC_BITS-1] : ss_si;

always_comb begin
	case (sel)
		2'd0:    rd = {13'd0, st.tireq, st.tce, st.tien};
		2'd2:    rd = st.tmr;
		2'd3:    rd = st.dwc;
		default: rd = 16'd0;
	endcase
end

assign irq = st.tien && st.tce && st.tireq;

endmodule
