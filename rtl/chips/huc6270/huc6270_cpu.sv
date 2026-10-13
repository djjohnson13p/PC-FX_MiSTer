// HuC6270 CPU interface: bus strobes, address register, display registers
// R05-R0E, status read and BUSY.
//
// Each bus cycle makes one access, at the first strobe cycle BUSY does not
// hold. A strobe held by BUSY waits and then acts once, so a VRR read or a
// VWR write never repeats however long the cycle is.

module huc6270_cpu
	import huc6270_pkg::*;
#(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_dck,

	input  logic        cs_n,
	input  logic        rd_n,
	input  logic        wr_n,
	input  logic        a1,
	input  logic        a0,
	input  logic        mode8,
	input  logic [15:0] d_i,
	output logic [15:0] d_o,
	output logic        d_oe,
	output logic        busy_n,

	output logic        acc_wr,
	output logic        acc_rd,
	output logic        acc_a1,
	output logic [15:0] acc_wd,
	output logic        acc_lo,
	output logic        acc_hi,
	output logic        sr_rd,

	input  logic        vram_busy,
	input  logic [15:0] vrr,
	input  logic  [5:0] irq_flags,
	input  logic        byr_clr,

	output regs_state_t regs,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

regs_state_t st;

wire strobe = !cs_n && (!rd_n || !wr_n);
wire data_reg = st.ar == 5'h00 || st.ar == 5'h01 || st.ar == 5'h02;
wire hold = vram_busy && a1 && data_reg;

logic done;
wire acc = strobe && !done && !hold;

always_ff @(posedge clk) begin
	if (reset || !strobe)
		done <= 1'b0;
	else if (acc)
		done <= 1'b1;
end

assign acc_wr = acc && !wr_n;
assign acc_rd = acc && wr_n;
assign acc_a1 = a1;
assign acc_wd = mode8 ? {d_i[7:0], d_i[7:0]} : d_i;
assign acc_lo = !mode8 || !a0;
assign acc_hi = !mode8 || a0;
assign sr_rd  = acc_rd && !a1 && acc_lo;

wire w = acc_wr && a1;

// Each register as the bus would see it, with this write merged in.
/* verilator lint_off UNUSEDSIGNAL */
wire [15:0] cr_m  = wmerge({3'd0, st.cr_iw, 1'b0, st.cr_te, st.cr_bb, st.cr_sb, st.cr_ex, st.cr_ie},
                           acc_wd, acc_lo, acc_hi);
wire [15:0] rcr_m = wmerge({6'd0, st.rcr}, acc_wd, acc_lo, acc_hi);
wire [15:0] bxr_m = wmerge({6'd0, st.bxr}, acc_wd, acc_lo, acc_hi);
wire [15:0] byr_m = wmerge({7'd0, st.byr}, acc_wd, acc_lo, acc_hi);
wire [15:0] hsr_m = wmerge({1'b0, st.hds, 3'd0, st.hsw}, acc_wd, acc_lo, acc_hi);
wire [15:0] hdr_m = wmerge({1'b0, st.hde, 1'b0, st.hdw}, acc_wd, acc_lo, acc_hi);
wire [15:0] vpr_m = wmerge({st.vds, 3'd0, st.vsw}, acc_wd, acc_lo, acc_hi);
wire [15:0] vdw_m = wmerge({7'd0, st.vdw}, acc_wd, acc_lo, acc_hi);
/* verilator lint_on UNUSEDSIGNAL */

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
		st.hdw <= 7'd31;
	end else if (SAVESTATE && ss_shift) begin
		st <= regs_state_t'({st[REGS_BITS-2:0], ss_si});
	end else begin
		if (ce_dck && byr_clr)
			st.byr_set <= 1'b0;

		if (acc_wr && !a1 && acc_lo)
			st.ar <= acc_wd[4:0];

		if (w) begin
			case (st.ar)
				5'h05: {st.cr_iw, st.cr_te, st.cr_bb, st.cr_sb, st.cr_ex, st.cr_ie} <= {cr_m[12:11], cr_m[9:0]};
				5'h06: st.rcr <= rcr_m[9:0];
				5'h07: st.bxr <= bxr_m[9:0];
				5'h08: begin
					st.byr     <= byr_m[8:0];
					st.byr_set <= 1'b1;
				end
				5'h09: if (acc_lo) st.mwr <= acc_wd[7:0];
				5'h0A: {st.hds, st.hsw} <= {hsr_m[14:8], hsr_m[4:0]};
				5'h0B: {st.hde, st.hdw} <= {hdr_m[14:8], hdr_m[6:0]};
				5'h0C: {st.vds, st.vsw} <= {vpr_m[15:8], vpr_m[4:0]};
				5'h0D: st.vdw <= vdw_m[8:0];
				5'h0E: if (acc_lo) st.vcr <= acc_wd[7:0];
				default: ;
			endcase
		end
	end
end

assign ss_so = SAVESTATE ? st[REGS_BITS-1] : ss_si;
assign regs  = st;

// Status: BSY, VD, DV, DS, RR, OR, CR. The read latch follows the register
// until the access, so data is ready in the clock BUSY lets go.
always_ff @(posedge clk) begin
	if (!cs_n && !rd_n && !done) begin
		if (!a1)
			d_o <= {9'd0, vram_busy, irq_flags};
		else if (mode8 && a0)
			d_o <= {8'd0, vrr[15:8]};
		else
			d_o <= vrr;
	end
end

assign d_oe   = !cs_n && !rd_n;
assign busy_n = !(strobe && !done && hold);

endmodule
