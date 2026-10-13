// HuC6230 CPU interface: write-only. A4 = 0 goes to the PSG (R00-R09);
// R10-R16 live here. Each write acts once, on the master clock where the
// strobe is first seen low. R10 takes effect on the next -HSYNC fall, so the
// last write in a line wins; r10_now already shows it on that fall's clock.

module huc6230_cpu_if #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic       clk,
	input  logic       reset,
	input  logic       ce_master,

	input  logic       cs_n,
	input  logic       wr_n,
	input  logic [4:0] a,
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic [7:0] d_i,
	/* verilator lint_on UNUSEDSIGNAL */
	input  logic       hsync_n,

	output logic       psg_we,
	output logic [3:0] psg_a,
	output logic       hs_fall,
	output logic [5:0] r10_now,
	output logic [3:0][5:0] avol,
	output logic [1:0][5:0] lvol,

	input  logic       ss_shift,
	input  logic       ss_si,
	output logic       ss_so
);

import huc6230_pkg::*;

cpu_state_t st;
logic wr_q;

wire write = !cs_n && !wr_n;
wire wr_go = ce_master && write && !wr_q;

always_ff @(posedge clk)
	if (ce_master)
		wr_q <= write;

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= cpu_state_t'({st[SS_CPU_BITS-2:0], ss_si});
	end else begin
		if (ce_master)
			st.hs_q <= hsync_n;
		if (hs_fall)
			st.r10 <= st.r10_pend;
		if (wr_go && a[4]) begin
			case (a[3:0])
				4'h0:    st.r10_pend <= d_i[5:0];
				4'h1:    st.avol[0]  <= d_i[5:0];
				4'h2:    st.avol[1]  <= d_i[5:0];
				4'h3:    st.avol[2]  <= d_i[5:0];
				4'h4:    st.avol[3]  <= d_i[5:0];
				4'h5:    st.lvol[0]  <= d_i[5:0];
				4'h6:    st.lvol[1]  <= d_i[5:0];
				default: ;
			endcase
		end
	end
end

assign ss_so   = SAVESTATE ? st[SS_CPU_BITS-1] : ss_si;
assign psg_we  = wr_go && !a[4];
assign psg_a   = a[3:0];
assign hs_fall = ce_master && st.hs_q && !hsync_n;
assign r10_now = hs_fall ? st.r10_pend : st.r10;
assign avol    = st.avol;
assign lvol    = st.lvol;

endmodule
