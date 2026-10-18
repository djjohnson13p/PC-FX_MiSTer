// HuC6270 timing: dot and character counters, line counter, display window,
// raster counter, VRAM slot plan, and the VD and RR interrupts.
//
// The VDC runs as a slave of /HSYN and /VSYN, sampled at each DCK. A fall
// of /HSYN starts a line with a four-character sync period whatever HSW
// says; a line can also end on its own at HDE. A fall of /VSYN restarts the
// line count.
//
//   line:  | sync | HDS |   HDW (display)   | HDE |
//   tile:  0     hsw   hds                 hdisp  hde
//
// The *_end positions are the last character of each period.

module huc6270_timing
	import huc6270_pkg::*;
#(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_dck,

	input  logic        hsyn_n,
	input  logic        vsyn_n,

	/* verilator lint_off UNUSEDSIGNAL */
	input  regs_state_t regs,
	/* verilator lint_on UNUSEDSIGNAL */
	input  logic        sr_rd,

	input  logic        dmas_exec,
	input  logic        dma_exec,
	input  logic        spr_fetch,
	input  logic        spr_fetch_en,
	input  logic        spr_cg,

	output slot_t       slot,
	output logic        dot0,
	output logic        hs_f,
	output logic        at_hds_d3,
	output logic        at_hds_d7,
	output logic        at_hdsm2_d7,
	output logic        at_hdsm3_d7,
	output logic        at_hdisp_d3,
	output logic        at_hdisp_d7,
	output logic        brk_d7,
	output logic        v_ge,
	output logic        v_gt,
	output logic        v_first,
	output logic        v_idle,
	output logic        vbl_start,

	output logic        vdisp,
	output logic        burst,
	output logic        bg_out,
	output logic        bb,
	output logic        sb,
	output logic  [1:0] vm,
	output logic  [1:0] sm,
	output logic  [2:0] screen,
	output logic  [9:0] rc_cnt,
	output logic        irq_vbl,
	output logic        irq_rcr,

	output logic        hsyn_int_n,
	output logic        vsyn_int_n,
	output logic  [9:0] line,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so,
	output logic        ss_rcr_idle
);

timing_state_t st;
// Capture waits for the raster event pipe to drain.
logic [9:0] rcr_pipe;

wire vs_f = !vsyn_n && st.vs_q;
assign hs_f = !hsyn_n && st.hs_q;

wire [6:0] hsw_end   = {2'd0, st.hsw};
wire [6:0] hds_end   = hsw_end + 7'd1 + st.hds;
wire [6:0] hdisp_pos = hds_end + 7'd1 + st.hdw;
wire [6:0] hde_end   = hdisp_pos + 7'd1 + st.hde;

wire [9:0] vsw_end   = {5'd0, st.vsw};
wire [9:0] vds_end   = vsw_end + 10'd2 + {2'd0, st.vds};
wire [9:0] vdisp_end = vds_end + 10'd1 + {1'd0, st.vdw};
wire [9:0] vde_end   = vdisp_end + {2'd0, st.vde};

wire d7 = st.dot == 3'd7;

wire at_hsw_d7   = st.tile == hsw_end && d7;
wire at_hdsm3_d6 = st.tile == hds_end - 7'd3 && st.dot == 3'd6;
wire at_hdisp_d1 = st.tile == hdisp_pos && st.dot == 3'd1;
wire at_hde_d7   = st.tile == hde_end && d7;
wire disp_break  = st.line_inc && hs_f;
wire rcr_match   = at_hdisp_d1 && st.rc_cnt == regs.rcr && regs.cr_ie[2];

assign ss_rcr_idle = rcr_pipe == 10'd0 && !rcr_match;

assign at_hds_d3   = st.tile == hds_end && st.dot == 3'd3;
assign at_hds_d7   = st.tile == hds_end && d7;
assign at_hdsm2_d7 = st.tile == hds_end - 7'd2 && d7;
assign at_hdsm3_d7 = st.tile == hds_end - 7'd3 && d7;
assign at_hdisp_d3 = st.tile == hdisp_pos && st.dot == 3'd3;
assign at_hdisp_d7 = st.tile == hdisp_pos && d7;
assign brk_d7      = st.tile == 7'd0 && d7 && st.break_latch;

wire hdisp_end = at_hdisp_d7 || brk_d7;

assign v_ge    = st.line >= vds_end && st.line < vdisp_end;
assign v_gt    = st.line > vds_end && st.line <= vdisp_end;
assign v_first = st.line == vds_end + 10'd1;
assign v_idle  = st.line < vds_end || st.line > vdisp_end;

assign vbl_start = at_hdsm2_d7 && !st.vdisp && st.vdisp_old;

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
		rcr_pipe <= '0;
		st.hdw    <= 7'd31;
		st.burst  <= 1'b1;
		st.rc_cnt <= 10'd64;
	end else if (SAVESTATE && ss_shift) begin
		st <= timing_state_t'({st[TIMING_BITS-2:0], ss_si});
		rcr_pipe <= '0;
	end else begin
		if (sr_rd) begin
			st.irq_vbl <= 1'b0;
			st.irq_rcr <= 1'b0;
		end

		if (ce_dck) begin
			rcr_pipe <= {rcr_pipe[8:0], rcr_match};
			st.hs_q <= hsyn_n;
			st.vs_q <= vsyn_n;

			// Dot and character counters.
			st.dot <= st.dot + 3'd1;
			if (d7) begin
				st.tile      <= st.tile + 7'd1;
				st.tile_zero <= 1'b0;
			end
			if (at_hde_d7 || hs_f) begin
				st.dot         <= 3'd0;
				st.tile        <= 7'd0;
				st.dots_remain <= st.dot;
				if (hs_f) begin
					st.hsw       <= 5'd3;
					st.tile_zero <= 1'b1;
				end else begin
					st.hsw <= regs.hsw;
				end
				st.hds <= regs.hds;
				st.hdw <= regs.hdw;
				st.hde <= regs.hde;
				st.cm  <= regs.mwr[7];
			end

			// Line counter and vertical window.
			if (vs_f) begin
				st.vdisp  <= 1'b0;
				st.line   <= 10'd0;
				st.vsw    <= regs.vsw;
				st.vds    <= regs.vds;
				st.vdw    <= regs.vdw;
				st.vde    <= regs.vcr;
				st.vm     <= regs.mwr[1:0];
				st.sm     <= regs.mwr[3:2];
				st.screen <= regs.mwr[6:4];
			end else begin
				if (d7)
					st.break_latch <= 1'b0;
				if (at_hsw_d7) begin
					st.line_inc <= 1'b1;
					st.break_en <= 1'b1;
				end
				if (at_hdisp_d7)
					st.break_en <= 1'b0;
				if (disp_break) begin
					st.line     <= st.line + 10'd1;
					st.line_inc <= 1'b0;
					if (st.line == vsw_end)
						st.burst <= !(regs.cr_sb || regs.cr_bb);
					if (st.line == vds_end)
						st.vdisp <= 1'b1;
					if (st.line == vdisp_end)
						st.vdisp <= 1'b0;
					if (st.line == vde_end) begin
						st.line   <= 10'd0;
						st.vsw    <= regs.vsw;
						st.vds    <= regs.vds;
						st.vdw    <= regs.vdw;
						st.vde    <= regs.vcr;
						st.vm     <= regs.mwr[1:0];
						st.sm     <= regs.mwr[3:2];
						st.screen <= regs.mwr[6:4];
					end
					st.break_latch <= st.break_en;
					st.break_en    <= 1'b0;
				end
			end

			if (at_hdsm2_d7 && v_gt)
				st.bg_fetch <= 1'b1;
			else if (hdisp_end)
				st.bg_fetch <= 1'b0;

			if (at_hds_d7 && v_gt)
				st.bg_out <= 1'b1;
			else if (hdisp_end)
				st.bg_out <= 1'b0;

			// Raster counter: 64 on the line before the display starts.
			if (at_hdisp_d7 || (disp_break && !st.rc_upd))
				st.rc_cnt <= st.line == vds_end - 10'd1 ? 10'd64 : st.rc_cnt + 10'd1;
			if (disp_break)
				st.rc_upd <= 1'b0;
			if (at_hdisp_d7)
				st.rc_upd <= 1'b1;

			if (at_hdsm3_d6) begin
				st.bb <= regs.cr_bb;
				st.sb <= regs.cr_sb;
			end

			if (at_hdsm2_d7) begin
				st.vdisp_old <= st.vdisp;
				if (!st.vdisp && st.vdisp_old && regs.cr_ie[3])
					st.irq_vbl <= 1'b1;
			end

			if (rcr_pipe[9])
				st.irq_rcr <= 1'b1;
		end
	end
end

assign ss_so = SAVESTATE ? st[TIMING_BITS-1] : ss_si;

// Slot plan. A line started by /HSYN leaves its first few dots idle.
always_comb begin
	if (st.tile_zero && st.dot <= st.dots_remain) begin
		slot = SL_NOP;
	end else if (dmas_exec) begin
		slot = st.dot[1:0] == 2'd3 ? SL_CPU : SL_NOP;
	end else if (dma_exec) begin
		slot = st.dot[0] ? SL_NOP : SL_CPU;
	end else if (st.bg_fetch) begin
		if (!st.bb) begin
			slot = st.dot[0] ? SL_NOP : SL_CPU;
		end else begin
			case (st.vm)
				2'd0: case (st.dot)
					3'd1:    slot = SL_BAT;
					3'd3:    slot = SL_NOP;
					3'd5:    slot = SL_CG0;
					3'd7:    slot = SL_CG1;
					default: slot = SL_CPU;
				endcase
				2'd1, 2'd2: case (st.dot)
					3'd1:    slot = SL_BAT;
					3'd3:    slot = SL_CPU;
					3'd5:    slot = SL_CG0;
					3'd7:    slot = SL_CG1;
					default: slot = SL_NOP;
				endcase
				default: case (st.dot)
					3'd3:    slot = SL_BAT;
					3'd7:    slot = st.cm ? SL_CG1 : SL_CG0;
					default: slot = SL_NOP;
				endcase
			endcase
		end
	end else if (spr_fetch) begin
		if (!spr_fetch_en) begin
			slot = st.dot[0] ? SL_NOP : SL_CPU;
		end else begin
			case (st.sm)
				2'd0: case (st.dot[1:0])
					2'd0:    slot = SL_SG0;
					2'd1:    slot = SL_SG1;
					2'd2:    slot = SL_SG2;
					default: slot = SL_SG3;
				endcase
				2'd1: case (st.dot[1:0])
					2'd1:    slot = spr_cg ? SL_SG2 : SL_SG0;
					2'd3:    slot = spr_cg ? SL_SG3 : SL_SG1;
					default: slot = SL_NOP;
				endcase
				2'd2: case (st.dot[1:0])
					2'd1:    slot = st.dot[2] ? SL_SG2 : SL_SG0;
					2'd3:    slot = st.dot[2] ? SL_SG3 : SL_SG1;
					default: slot = SL_NOP;
				endcase
				default: case ({st.dot[1:0] == 2'd3, st.tile[0], st.dot[2]})
					3'b100:  slot = SL_SG0;
					3'b101:  slot = SL_SG1;
					3'b110:  slot = SL_SG2;
					3'b111:  slot = SL_SG3;
					default: slot = SL_NOP;
				endcase
			endcase
		end
	end else begin
		slot = st.dot[0] ? SL_NOP : SL_CPU;
	end
end

assign dot0     = st.dot[0];
assign vdisp    = st.vdisp;
assign burst    = st.burst;
assign bg_out   = st.bg_out;
assign bb       = st.bb;
assign sb       = st.sb;
assign vm       = st.vm;
assign sm       = st.sm;
assign screen   = st.screen;
assign rc_cnt   = st.rc_cnt;
assign irq_vbl  = st.irq_vbl;
assign irq_rcr  = st.irq_rcr;
assign line     = st.line;

// Sync as the chip would drive it in the EX output modes.
assign hsyn_int_n = st.tile > hsw_end;
assign vsyn_int_n = st.line > vsw_end;

endmodule
