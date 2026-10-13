// HuC6271 CPU registers R00-R05, write only. Chip A3:A1 pick the register,
// A0 the byte (R00 only has a high byte, bit 8 of SCX).
//
// SRT is acted on at the next -HSYNC fall: 0 enters reset and clears every
// register, 1 leaves it. In reset only SRT can be written. SCX and CHR are
// taken into the display at each -HSYNC fall; a write on that clock wins
// and is taken one line later.

module huc6271_cpu_if #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce,

	input  logic        cs_n,
	input  logic        wr_n,
	input  logic  [3:0] a,
	input  logic  [7:0] d_i,

	input  logic        hs_fall,

	output huc6271_pkg::regs_t r,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

import huc6271_pkg::*;

regs_t st;

wire write = !cs_n && !wr_n;

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= regs_t'({st[REGS_BITS-2:0], ss_si});
	end else if (ce) begin
		if (hs_fall) begin
			st.scx_op <= st.scx;
			st.chr_op <= st.cr[1];
			st.run    <= st.cr[0];
			if (st.run && !st.cr[0])
				st <= '0;
		end

		if (write) begin
			if (a[3:1] == 3'd1 && !a[0])
				st.cr[0] <= d_i[0];
			if (st.run) begin
				case (a)
					4'h0: st.scx[7:0] <= d_i;
					4'h1: st.scx[8]   <= d_i[0];
					4'h2: st.cr[2:1]  <= d_i[2:1];
					4'h4: st.nry      <= d_i;
					4'h6: st.nru      <= d_i;
					4'h8: st.nrv      <= d_i;
					4'hA: st.hsr      <= d_i;
					default: ;
				endcase
			end
		end
	end
end

assign ss_so = SAVESTATE ? st[REGS_BITS-1] : ss_si;
assign r     = st;

endmodule
