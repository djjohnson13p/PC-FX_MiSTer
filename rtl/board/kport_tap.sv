// PC-FX multitap (designed, never sold): four K-port connectors behind one.
//
// LATCH, CLK and R/W go to every connector. A CLK fall while LATCH is low
// clears the tap: the scan starting at the LATCH rise reads connector 1.
// Otherwise each scan reads the next connector, and after the fourth the tap
// sends its own frame (ID 1110, rest 0) until cleared.

module kport_tap
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

	output logic        dn_latch_n,
	output logic        dn_kclk,
	output logic        dn_rw,
	input  logic  [3:0] dn_data,        // DATA level on each connector

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

tap_state_t st;

always_ff @(posedge clk) begin
	if (reset) begin
		st         <= '0;
		st.port_n  <= 3'd4;
		st.latch_q <= 1'b1;
		st.clk_q   <= 1'b1;
	end else if (SAVESTATE && ss_shift) begin
		st <= tap_state_t'({st[TAP_BITS-2:0], ss_si});
	end else begin
		st.latch_q <= latch_n;
		st.clk_q   <= kclk;
		if (!latch_n) begin
			st.bit_n <= 5'd0;
			if (st.clk_q && !kclk)
				st.clr <= 1'b1;
		end else if (!st.latch_q) begin
			st.clr    <= 1'b0;
			st.port_n <= st.clr ? 3'd0 : (st.port_n[2] ? 3'd4 : st.port_n + 3'd1);
		end else if (kclk && !st.clk_q) begin
			st.bit_n <= st.bit_n + 5'd1;
		end
	end
end

assign ss_so = SAVESTATE ? st[TAP_BITS-1] : ss_si;

wire own = st.bit_n[4:2] == 3'b111 && st.bit_n[1:0] != 2'b00;

assign dn_latch_n = latch_n;
assign dn_kclk    = kclk;
assign dn_rw      = rw;
assign data_o     = st.port_n[2] ? !own : dn_data[st.port_n[1:0]];
assign data_oe    = !rw;

endmodule
