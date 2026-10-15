// KING interrupt flags and -IRQ (C6272_1 2.3.5). A flag is latched when
// its enabled cause occurs and cleared by the read the manual names:
//
//   SCSIBK  read reg 07          SCDMA   read or write reg 0B (low half)
//   SBCODE  read reg 08, or the status upper half
//   LUSTER  read the status low half
//   SOUND   read reg 53
//
// LUSTER is the raster monitor: the raster count reaching reg 44, checked
// as the count changes (within two DCK of -HSYNC). A cause in the same
// clk as its clearing read wins.

module king_irq #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_osc,

	/* verilator lint_off UNUSEDSIGNAL */
	input  huc6272_pkg::acc_t acc,
	/* verilator lint_on UNUSEDSIGNAL */
	input  logic        raster_ev,   // raster count just changed
	input  logic  [8:0] raster,
	input  logic  [8:0] mon,         // reg 44
	input  logic        rint,

	input  logic        set_sound,
	input  logic        set_sbcode,
	input  logic        set_scdma,
	input  logic        set_scsibk,

	output logic  [4:0] flags,       // SCSIBK SCDMA SBCODE LUSTER SOUND
	output logic        irq_n,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

import huc6272_pkg::*;

irq_state_t st;

wire rd_reg  = acc.rd && !acc.ar;
wire wr_reg  = acc.wr && !acc.ar;
wire rd_stat = acc.rd && acc.ar;
wire luster  = raster_ev && rint && raster == mon;

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= irq_state_t'({st[IRQ_BITS-2:0], ss_si});
	end else if (ce_osc) begin
		if (set_scsibk)                            st.scsibk <= 1'b1;
		else if (rd_reg && acc.r == 7'h07)         st.scsibk <= 1'b0;
		if (set_scdma)                             st.scdma <= 1'b1;
		else if ((rd_reg || wr_reg) && acc.r == 7'h0B && !acc.hi)
		                                           st.scdma <= 1'b0;
		if (set_sbcode)                            st.sbcode <= 1'b1;
		else if ((rd_reg && acc.r == 7'h08 && !acc.hi) || (rd_stat && acc.hi))
		                                           st.sbcode <= 1'b0;
		if (luster)                                st.luster <= 1'b1;
		else if (rd_stat && !acc.hi)                st.luster <= 1'b0;
		if (set_sound)                             st.sound <= 1'b1;
		else if (rd_reg && acc.r == 7'h53)         st.sound <= 1'b0;
	end
end

assign ss_so = SAVESTATE ? st[IRQ_BITS-1] : ss_si;

assign flags = {st.scsibk, st.scdma, st.sbcode, st.luster, st.sound};
assign irq_n = !(|flags);

endmodule
