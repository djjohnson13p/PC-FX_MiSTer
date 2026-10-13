// PC-FX pad on a K-port connector.
//
// LATCH low loads the buttons and the MODE switches and puts frame bit 0 on
// DATA; each CLK rise after LATCH moves to the next bit, LSB first. The
// console samples on CLK falls. DATA is low for a 1 bit and is driven only
// while R/W is low (console reading).
//
//   bit  31-28  14     12     11-8                 7    6    5-0
//        1111   MODE2  MODE1  Left Down Right Up   Run  Sel  VI-I

module kport_pad
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

	input  logic [11:0] buttons,        // Left Down Right Up Run Sel VI V IV III II I
	input  logic        mode1,
	input  logic        mode2,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

pad_state_t st;

always_ff @(posedge clk) begin
	if (reset) begin
		st       <= '0;
		st.clk_q <= 1'b1;
	end else if (SAVESTATE && ss_shift) begin
		st <= pad_state_t'({st[PAD_BITS-2:0], ss_si});
	end else begin
		st.clk_q <= kclk;
		if (!latch_n) begin
			st.snap  <= {mode2, mode1, buttons};
			st.bit_n <= 5'd0;
		end else if (kclk && !st.clk_q) begin
			st.bit_n <= st.bit_n + 5'd1;
		end
	end
end

assign ss_so = SAVESTATE ? st[PAD_BITS-1] : ss_si;

logic bit_v;
always_comb begin
	if (st.bit_n[4:2] == 3'b111)
		bit_v = 1'b1;
	else if (st.bit_n[4:2] < 3'd3)
		bit_v = st.snap[st.bit_n[3:0]];
	else if (st.bit_n == 5'd12)
		bit_v = st.snap[12];
	else if (st.bit_n == 5'd14)
		bit_v = st.snap[13];
	else
		bit_v = 1'b0;
end

assign data_o  = !bit_v;
assign data_oe = !rw;

endmodule
