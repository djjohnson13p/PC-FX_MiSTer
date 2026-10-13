// Subcode port of the CD DSP: one P-W byte per frame on SBSO, P waiting
// when WFCK falls and each rising EXCK edge bringing the next bit. SCOR
// covers the second sync frame of each block; Q (80 bits from the TOC and
// the head, then its inverted CRC) fills frames 2-97, R-W comes from the
// sector's own subcode bytes while audio plays, else zero.

module cd_subcode_gen #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce,

	input  logic        spun,
	input  logic        f_start,
	input  logic  [6:0] f_idx,
	input  logic [79:0] q_now,
	input  logic        rw_ok,
	input  logic [15:0] rw_base,

	input  logic        exck,
	output logic        scor,
	output logic        sbso,

	output logic        rw_rq,
	output logic [15:0] rw_addr,
	input  logic        rw_ack,
	input  logic  [7:0] rw_q,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

import pcfx_cd_pkg::*;

sub_t st;

wire [6:0] k     = f_idx - 7'd2;          // Q bit of this frame
wire       data  = f_idx >= 7'd2;
wire       q_bit = k < 7'd80 ? q_now[7'd79 - k] : !st.crc[15];
wire       p_bit = q_now[63:56] == 8'h00;  // P marks the pause before a track

always_ff @(posedge clk) begin
	if (reset) begin
		st    <= '0;
		rw_rq <= 1'b0;
	end else if (SAVESTATE && ss_shift) begin
		st    <= sub_t'({st[SUB_BITS-2:0], ss_si});
		rw_rq <= 1'b0;
	end else if (ce) begin
		st.exck_d <= exck;
		if (f_start) begin
			st.sh <= data ? {p_bit, q_bit, 6'd0} : 8'd0;
			if (f_idx == 7'd2)
				st.crc <= q_bit ? 16'h1021 : 16'h0000;
			else if (data && k < 7'd80)
				st.crc <= {st.crc[14:0], 1'b0} ^ ((st.crc[15] ^ q_bit) ? 16'h1021 : 16'h0000);
			else if (data)
				st.crc <= {st.crc[14:0], 1'b0};
			if (data && rw_ok) begin
				rw_rq   <= 1'b1;
				rw_addr <= rw_base + 16'd2350 + {9'd0, f_idx};
			end
		end else if (exck && !st.exck_d) begin
			st.sh <= {st.sh[6:0], 1'b0};
		end
		if (rw_rq && rw_ack) begin
			rw_rq <= 1'b0;
			st.sh[5:0] <= rw_q[5:0];
		end
	end
end

assign scor  = spun && f_idx == 7'd1;
assign sbso  = spun && st.sh[7];
assign ss_so = SAVESTATE ? st[SUB_BITS-1] : ss_si;

wire unused_ok = &{1'b0, rw_q[7:6]};

endmodule
