// K-port: one serial pad port.
//
//   +0  W CR  D2 IOS (1 in), D1 MOD (multitap clear), D0 TRG; reset 110.
//             Taken only while TRG = 0 and END = 0.
//       R SR  D3 END, D2 IOS, D1 MOD, D0 TRG
//   +40 DL / +42 DH  data, 1 = line low. Changed only while TRG = 0;
//             any read or write of either half clears END and the IRQ.
//
// A scan in quarter-CLK ticks (ce_kport); TRG waits at pos 0 for a tick:
//
//   pos     1  2  3  4  5  6  7  8  9  10 11 ... 130 131 | END
//   LATCH   \__________/                                    (CLK pulses at 2-3 if MOD)
//   CLK     ~~~~~~~~~~~~~~~\_____/~~~~~\_____/ ...  \_____/~~
//   DATA                 [b0         ][b1   ]       [b31]
//
// Bits go LSB first. Data is sampled on each CLK fall and the next outbound
// bit is set while CLK is high.

module ga_kpc
	import upd65641_pkg::*;
#(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_sys,
	input  logic        ce_kport,

	input  logic        we,
	input  logic        re,
	input  logic        sel_data,
	input  logic        hi,
	input  logic [15:0] wd,
	output logic [15:0] rd,

	output logic        latch_n,
	output logic        kclk,
	output logic        rw,
	input  logic        data_i,
	output logic        data_o,
	output logic        data_oe,

	output logic        irq,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

kpc_state_t st;

wire step    = ce_kport && st.trg;
wire in_lat  = st.pos[7:3] == 5'd0 && st.pos[2:0] != 3'd0 && st.pos[2:0] <= 3'd4;
wire past4   = st.pos[7:2] != 6'd0;
wire fall    = step && past4 && st.pos[1:0] == 2'd1;
wire present = step && (st.pos == 8'd4 || (past4 && st.pos[1:0] == 2'd3));
wire done    = step && st.pos == 8'd131;

always_ff @(posedge clk) begin
	if (reset) begin
		st     <= '0;
		st.ios <= 1'b1;
		st.mod <= 1'b1;
	end else if (SAVESTATE && ss_shift) begin
		st <= kpc_state_t'({st[KPC_BITS-2:0], ss_si});
	end else if (ce_sys) begin
		if (step)
			st.pos <= st.pos + 8'd1;
		if (fall)
			st.sr <= {!data_i, st.sr[31:1]};
		if (present)
			st.dout <= st.sr[0];
		if (done) begin
			st.trg <= 1'b0;
			st.fin <= 1'b1;
		end

		if (we && !sel_data && !st.trg && !st.fin) begin
			st.ios <= wd[2];
			st.mod <= wd[1];
			if (wd[0]) begin
				st.trg <= 1'b1;
				st.pos <= 8'd0;
			end
		end
		if (we && sel_data && !st.trg) begin
			if (hi)
				st.sr[31:16] <= wd;
			else
				st.sr[15:0] <= wd;
		end
		if ((we || re) && sel_data && !st.trg)
			st.fin <= 1'b0;
	end
end

assign ss_so = SAVESTATE ? st[KPC_BITS-1] : ss_si;

assign rd = !sel_data ? {12'd0, st.fin, st.ios, st.mod, st.trg} :
	hi ? st.sr[31:16] : st.sr[15:0];

assign latch_n = !(st.trg && in_lat);
assign kclk    = !(st.trg && st.pos[1] && (!in_lat || st.mod));
assign rw      = !st.ios;
assign data_o  = !st.dout;
assign data_oe = !st.ios;
assign irq     = st.fin;

endmodule
