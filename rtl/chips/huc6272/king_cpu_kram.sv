// KING CPU access to KRAM: read and write address registers (0C, 0D), the
// data port (0E), page bits (0F) and the DRAM mode (61).
//
// A write to 0E is taken at once into a one-word buffer and done in the
// next free slot of its bank; a second write, or 0D, waits (BUSY) until the
// buffered one and its recovery are done. A read of 0E then starts
// the KRAM read and waits for the data. Each 0E access steps its address by
// the signed 10-bit increment, wrapping inside A16:A0 (bank and page stay).

module king_cpu_kram #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_osc,
	input  logic        ce_dck,
	input  logic        init,

	input  huc6272_pkg::acc_t acc,
	input  logic        pend_wr,
	input  logic        pend_rd,
	input  logic  [6:0] ar,
	output logic        hold,
	output logic [15:0] rdata,

	output logic        busy_bit,
	output logic  [3:0] pages,       // RAP SOP BGP SCP
	output logic        mode4m,

	output huc6272_pkg::kreq_t req,
	output logic        req_bank,    // 1 = bank B
	input  logic        gnt,         // req takes the dot starting at this DCK
	input  logic [15:0] ka_q,
	input  logic [15:0] kb_q,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

import huc6272_pkg::*;

kram_state_t st;
logic        cap, rd_done, rd_bank;
logic [15:0] rd_q;

function automatic logic [16:0] step(input logic [16:0] a, input logic [9:0] inc);
	step = a + {{7{inc[9]}}, inc};
endfunction

wire sel_c = ar == 7'h0C;
wire sel_d = ar == 7'h0D;
wire sel_e = ar == 7'h0E;
wire recovery_busy = st.recovery_ticks != 6'd0;
wire wbuf  = (st.req_v && st.req_we) || recovery_busy;

assign hold = (pend_wr && sel_e && (st.req_v || init || recovery_busy))
           || (pend_rd && sel_e && !rd_done)
           || ((pend_wr || pend_rd) && sel_d && wbuf);

wire rd_start = pend_rd && sel_e && !st.req_v && !rd_done && !init && !recovery_busy;
wire wr_reg   = acc.wr && !acc.ar;
wire rd_reg   = acc.rd && !acc.ar;

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= kram_state_t'({st[KRAM_BITS-2:0], ss_si});
	end else if (ce_osc) begin
		if (recovery_busy)
			st.recovery_ticks <= st.recovery_ticks - 6'd1;
		if (wr_reg) begin
			case (acc.r)
				7'h0C:
					if (acc.hi) {st.kra.page, st.kra.inc, st.kra.ab, st.kra.a[16]} <= {acc.d[15], acc.d[11:0]};
					else        st.kra.a[15:0] <= acc.d;
				7'h0D:
					if (acc.hi) {st.kwa.page, st.kwa.inc, st.kwa.ab, st.kwa.a[16]} <= {acc.d[15], acc.d[11:0]};
					else        st.kwa.a[15:0] <= acc.d;
				7'h0E: begin
					st.req_v  <= 1'b1;
					st.req_we <= 1'b1;
					st.wdata  <= acc.d;
				end
				7'h0F: if (!acc.hi) st.pages <= {acc.d[12], acc.d[8], acc.d[4], acc.d[0]};
				7'h61: if (!acc.hi) st.mode4m <= acc.d[0];
				default: ;
			endcase
		end

		if (rd_start) begin
			st.req_v  <= 1'b1;
			st.req_we <= 1'b0;
		end
		if (rd_reg && acc.r == 7'h0E)
			st.kra.a <= step(st.kra.a, st.kra.inc);

		if (ce_dck) begin
			if (gnt)
				st.fly <= 1'b1;
			if (st.fly) begin
				st.fly <= 1'b0;
				if (st.req_we) begin
					st.req_v <= 1'b0;
					st.kwa.a <= step(st.kwa.a, st.kwa.inc);
					st.recovery_ticks <= 6'd8;
				end
			end
		end
		if (cap)
			st.req_v <= 1'b0;
	end
end

assign ss_so = SAVESTATE ? st[KRAM_BITS-1] : ss_si;

// Read data is taken in the first clk of the dot after the slot.
always_ff @(posedge clk) begin
	if (reset) begin
		cap     <= 1'b0;
		rd_done <= 1'b0;
	end else if (ce_osc) begin
		cap <= ce_dck && st.fly && !st.req_we;
		if (cap) begin
			rd_q    <= rd_bank ? kb_q : ka_q;
			rd_done <= 1'b1;
		end else if (rd_reg && acc.r == 7'h0E) begin
			rd_done <= 1'b0;
		end
		if (rd_start)
			rd_bank <= st.kra.ab;
	end
end

always_comb begin
	rdata = 16'd0;
	if (sel_c)
		rdata = acc.hi ? {st.kra.page, 3'd0, st.kra.inc, st.kra.ab, st.kra.a[16]} : st.kra.a[15:0];
	else if (sel_d)
		rdata = acc.hi ? {st.kwa.page, 3'd0, st.kwa.inc, st.kwa.ab, st.kwa.a[16]} : st.kwa.a[15:0];
	else if (sel_e)
		rdata = rd_q;
	else if (ar == 7'h0F && !acc.hi)
		rdata = {3'd0, st.pages[3], 3'd0, st.pages[2], 3'd0, st.pages[1], 3'd0, st.pages[0]};
	else if (ar == 7'h61 && !acc.hi)
		rdata = {15'd0, st.mode4m};
end

always_comb begin
	req.v    = st.req_v && !st.fly;
	req.we   = st.req_we;
	req.page = st.req_we ? st.kwa.page : st.kra.page;
	req.a    = st.req_we ? st.kwa.a : st.kra.a;
	req.d    = st.wdata;
end

assign req_bank = st.req_we ? st.kwa.ab : st.kra.ab;
assign busy_bit = wbuf;
assign pages    = st.pages;
assign mode4m   = st.mode4m;

endmodule
