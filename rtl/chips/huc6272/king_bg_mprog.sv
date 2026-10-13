// KING BG microprogram: 16 nine-bit words, 0-7 for bank A cycles 0-7 and
// 8-F for bank B (C6272_2 3.6.7). Reg 13 sets the load address, each reg 14
// write stores a word and steps the address (wrapping after 16), reg 15
// D0 is MPSW. Words load only while MPSW is 0.
//
//   word  8    7-6  5    4        3        2-0
//         NOP  BG#  rot  BAT/-CG  ind/-dir CG offset

module king_bg_mprog #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_osc,

	/* verilator lint_off UNUSEDSIGNAL */
	input  huc6272_pkg::acc_t acc,
	/* verilator lint_on UNUSEDSIGNAL */
	input  logic  [2:0] cyc,         // cycle of the planned dot

	output logic        mpsw,
	output huc6272_pkg::mpw_t wa,    // bank A word for the planned dot
	output huc6272_pkg::mpw_t wb,
	output logic  [3:0] has_cg,      // BG has a CG slot in the program

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

import huc6272_pkg::*;

mprog_state_t st;

wire wr_reg = ce_osc && acc.wr && !acc.ar && !acc.hi;

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= mprog_state_t'({st[MPROG_BITS-2:0], ss_si});
	end else if (wr_reg) begin
		case (acc.r)
			7'h13: st.mpa <= acc.d[3:0];
			7'h14:
				if (!st.mpsw) begin
					st.mp[st.mpa] <= acc.d[8:0];
					st.mpa <= st.mpa + 4'd1;
				end
			7'h15: st.mpsw <= acc.d[0];
			default: ;
		endcase
	end
end

assign ss_so = SAVESTATE ? st[MPROG_BITS-1] : ss_si;

assign mpsw = st.mpsw;
wire [3:0] ia = {1'b0, cyc};
wire [3:0] ib = {1'b1, cyc};

assign wa   = mpw_t'(st.mp[ia]);
assign wb   = mpw_t'(st.mp[ib]);

always_comb begin
	has_cg = 4'd0;
	for (int i = 0; i < 16; i++)
		if (!st.mp[i][8] && !st.mp[i][4])
			has_cg[st.mp[i][7:6]] = 1'b1;
end

endmodule
