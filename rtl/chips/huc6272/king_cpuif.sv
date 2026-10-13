// KING M-bus interface: address register, status register, and one access
// per strobe to the register AR selects.
//
// A strobe acts once, on the first OSC clock it is seen low, unless a block
// holds it (a KRAM access still pending): then -BUSY goes low and the access
// acts when the hold lifts. Read data is registered when the access acts and
// kept until the next one.
//
//   A2 A1   write                     read
//   0  0    AR (D6:0)                 status: AR, BUSY, interrupt flags
//   0  1    -                         SCSI bus (D7:1), subcode byte (D15:8)
//   1  x    register AR, half A1      register AR, half A1

module king_cpuif #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_osc,

	input  logic        cs_n,
	input  logic        rd_n,
	input  logic        wr_n,
	input  logic        a2,
	input  logic        a1,
	input  logic [15:0] d_i,
	output logic [15:0] d_o,
	output logic        d_oe,
	output logic        busy_n,

	output logic  [6:0] ar,
	output logic        pend_wr,     // a register strobe waits to act
	output logic        pend_rd,
	output huc6272_pkg::acc_t acc,   // the access acting this clk
	input  logic        hold,
	input  logic [15:0] rdata,       // selected register, A2 = 1
	input  logic  [7:0] st_hi,       // status D15:8
	input  logic [15:0] st_bus,      // status upper half

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

import huc6272_pkg::*;

cpuif_state_t st;
logic busy_q;

wire wr_s = !cs_n && !wr_n;
wire rd_s = !cs_n && !rd_n;
wire pend = (wr_s || rd_s) && !st.done;
wire held = a2 && hold;
wire go   = ce_osc && pend && !held;

assign pend_wr = pend && wr_s && a2;
assign pend_rd = pend && rd_s && a2;

always_comb begin
	acc.wr = go && wr_s && a2;
	acc.rd = go && rd_s;
	acc.ar = !a2;
	acc.hi = a1;
	acc.r  = st.ar;
	acc.d  = d_i;
end

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= cpuif_state_t'({st[CPUIF_BITS-2:0], ss_si});
	end else if (ce_osc) begin
		if (go) begin
			st.done <= 1'b1;
			if (wr_s && !a2 && !a1)
				st.ar <= d_i[6:0];
		end else if (!wr_s && !rd_s) begin
			st.done <= 1'b0;
		end
	end
end

assign ss_so = SAVESTATE ? st[CPUIF_BITS-1] : ss_si;

always_ff @(posedge clk) begin
	if (reset)
		busy_q <= 1'b0;
	else if (ce_osc)
		busy_q <= pend && held;
end

always_ff @(posedge clk) begin
	if (ce_osc) begin
		if (go && rd_s)
			d_o <= a2 ? rdata : a1 ? st_bus : {st_hi, 1'b0, st.ar};
	end
end

assign ar     = st.ar;
assign d_oe   = rd_s;
assign busy_n = !busy_q;

endmodule
