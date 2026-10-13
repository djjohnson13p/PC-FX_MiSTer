// HuC6261 CPU interface: AR, SR, CR, CPA, and the R04-R15 register file,
// which lives in the palette RAM at 512 + register number. Each strobe acts
// once, on the master clock where it is first seen low. R04-R15 and CR
// 14:8 are copied into line_regs once per line, during hblank.
//
// R03 reads a latch: a CPA write loads palette[CPA], an R02 write the word
// written, and each R03 read returns it, steps CPA and loads the next entry.

module huc6261_cpu_if #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic         clk,
	input  logic         reset,
	input  logic         ce_master,

	input  logic         cs_n,
	input  logic         rd_n,
	input  logic         wr_n,
	input  logic         a1,
	input  logic  [15:0] d_i,
	output logic  [15:0] d_o,
	output logic         d_oe,

	input  logic   [8:0] vc,
	input  logic         oe,
	input  logic         disp,

	output logic   [9:0] ram_addr,
	output logic         ram_we,
	input  logic  [15:0] ram_q,

	input  logic         copy,
	input  logic   [4:0] copy_word,
	input  logic  [15:0] copy_q,

	output huc6261_pkg::frame_t     cr_frame,  // CR 7:0
	output huc6261_pkg::line_regs_t lr,

	input  logic         ss_shift,
	input  logic         ss_si,
	output logic         ss_so
);

import huc6261_pkg::*;

cpu_state_t st;
logic rd_q, wr_q;
logic fetch;    // load the R03 latch once the RAM shows palette[CPA]

wire read  = !cs_n && !rd_n;
wire write = !cs_n && !wr_n;
wire rd_go = ce_master && read && !rd_q;
wire wr_go = ce_master && write && !wr_q;

wire ar_pal = st.ar >= 5'h01 && st.ar <= 5'h03;
wire ar_ram = st.ar >= 5'h04 && st.ar <= 5'h15;

always_ff @(posedge clk) begin
	if (ce_master) begin
		rd_q <= read;
		wr_q <= write;
	end
end

// fetch is not saved: a freeze waits for the I/O cycle, which outlasts it.
always_ff @(posedge clk) begin
	if (reset) begin
		st    <= '0;
		fetch <= 1'b0;
	end else if (SAVESTATE && ss_shift) begin
		st <= cpu_state_t'({st[SS_CPU_BITS-2:0], ss_si});
	end else begin
		if (ce_master && fetch) begin
			st.cpr <= ram_q;
			fetch  <= 1'b0;
		end
		if (wr_go) begin
			if (!a1)
				st.ar <= d_i[4:0];
			else case (st.ar)
				5'h00: st.cr <= cr_t'({d_i[14:6], d_i[3:0]});
				5'h01: begin
					st.cpa <= d_i[8:0];
					fetch  <= 1'b1;
				end
				5'h02: begin
					st.cpa <= st.cpa + 9'd1;
					st.cpr <= d_i;
				end
				default: ;
			endcase
		end
		if (rd_go && a1 && st.ar == 5'h03) begin
			st.cpa <= st.cpa + 9'd1;
			fetch  <= 1'b1;
		end
	end
end

assign ss_so = SAVESTATE ? st[SS_CPU_BITS-1] : ss_si;
assign cr_frame = frame_t'({st.cr.sp256, st.cr.bg256, st.cr.dc7, st.cr.dcc});

// Palette entries go through CPA; R04-R15 sit at 512 + AR.
assign ram_addr = ar_pal ? {1'b0, st.cpa} : {5'b10000, st.ar};
assign ram_we   = wr_go && a1 && (st.ar == 5'h02 || ar_ram);

// Bits the chip does not keep read as 0.
logic [15:0] mask;
always_comb begin
	case (st.ar)
		5'h07:   mask = 16'h00FF;
		5'h08:   mask = 16'h0777;
		5'h09:   mask = 16'h7777;
		5'h10, 5'h11, 5'h12, 5'h13, 5'h14, 5'h15:
		         mask = 16'h0FFF;
		default: mask = 16'hFFFF;
	endcase
end

logic [15:0] reg_q;
always_comb begin
	case (st.ar)
		5'h00:   reg_q = {1'b0, st.cr.blank, st.cr.sp256, st.cr.bg256, 2'b00, st.cr.dc7, st.cr.ex, st.cr.dcc};
		5'h01:   reg_q = {7'd0, st.cpa};
		5'h03:   reg_q = st.cpr;
		default: reg_q = ar_ram ? ram_q & mask : 16'd0;
	endcase
end

wire [15:0] sr = {disp, oe, vc, st.ar};

always_ff @(posedge clk) begin
	if (rd_go)
		d_o <= a1 ? reg_q : sr;
end

assign d_oe = read;

// Line latch
always_ff @(posedge clk) begin
	if (reset) begin
		lr.blank <= '0;
	end else if (ce_master && copy) begin
		case (copy_word)
			5'h04: lr.cpo1 <= copy_q;
			5'h05: lr.cpo2 <= copy_q;
			5'h06: lr.cpo3 <= copy_q;
			5'h07: lr.cpo4 <= copy_q[7:0];
			5'h08: lr.pri1 <= {copy_q[10:8], copy_q[6:4], copy_q[2:0]};
			5'h09: lr.pri2 <= {copy_q[14:12], copy_q[10:8], copy_q[6:4], copy_q[2:0]};
			5'h0A: lr.cky <= copy_q;
			5'h0B: lr.cku <= copy_q;
			5'h0C: lr.ckv <= copy_q;
			5'h0D: lr.ccr <= copy_q;
			5'h0E: lr.ble <= copy_q;
			5'h0F: lr.spbl <= copy_q;
			5'h10: lr.bl1a <= copy_q[11:0];
			5'h11: lr.bl1b <= copy_q[11:0];
			5'h12: lr.bl2a <= copy_q[11:0];
			5'h13: lr.bl2b <= copy_q[11:0];
			5'h14: lr.bl3a <= copy_q[11:0];
			5'h15: begin
				lr.bl3b  <= copy_q[11:0];
				lr.blank <= st.cr.blank;
			end
			default: ;
		endcase
	end
end

endmodule
