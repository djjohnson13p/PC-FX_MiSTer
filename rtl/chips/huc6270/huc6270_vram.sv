// HuC6270 VRAM port: CPU reads and writes through MARR/MAWR, VRAM-VRAM and
// VRAM-SATB block transfers, and the MA/MD/MRD/MWR pins.
//
// A CPU access waits two odd dots, then takes the next CPU slot. The pins
// hold one slot's address for the whole dot; read data is taken at the DCK
// that ends it, and a write strobes MWR for the dot.

module huc6270_vram
	import huc6270_pkg::*;
#(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_dck,

	input  logic        acc_wr,
	input  logic        acc_rd,
	input  logic        acc_a1,
	input  logic [15:0] acc_wd,
	input  logic        acc_lo,
	input  logic        acc_hi,
	input  logic        sr_rd,
	input  logic  [4:0] ar,
	input  logic  [1:0] cr_iw,

	input  slot_t       slot,
	input  logic        dot0,
	input  logic        burst,
	input  logic        vdisp,
	input  logic        vbl_start,
	input  logic [15:0] bg_addr,
	input  logic [15:0] spr_addr,

	output logic        busy,
	output logic [15:0] vrr,
	output logic        dma_exec,
	output logic        dmas_exec,
	output logic        irq_dma,
	output logic        irq_dmas,

	output logic  [7:0] sat_addr,
	output logic        sat_we,

	output logic [15:0] ma,
	input  logic [15:0] md_i,
	output logic [15:0] md_o,
	output logic        mrd_n,
	output logic        mwr_n,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

vram_state_t st;

wire w = acc_wr && acc_a1;
/* verilator lint_off UNUSEDSIGNAL */
wire [15:0] dcr_m = wmerge({11'd0, st.dcr}, acc_wd, acc_lo, acc_hi);
/* verilator lint_on UNUSEDSIGNAL */

// MAWR/MARR step after each access: +1, +20h, +40h, +80h.
logic [15:0] step;
always_comb begin
	case (cr_iw)
		2'd0:    step = 16'h0001;
		2'd1:    step = 16'h0020;
		2'd2:    step = 16'h0040;
		default: step = 16'h0080;
	endcase
end
wire cpu_slot = slot == SL_CPU;

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= vram_state_t'({st[VRAM_BITS-2:0], ss_si});
	end else begin
		if (w) begin
			case (ar)
				5'h00: st.mawr <= wmerge(st.mawr, acc_wd, acc_lo, acc_hi);
				5'h01: if (!st.busy) begin
					st.marr <= wmerge(st.marr, acc_wd, acc_lo, acc_hi);
					if (acc_hi) begin
						st.rd_pend <= 1'b1;
						st.busy    <= 1'b1;
					end
				end
				5'h02: if (!st.busy) begin
					st.vwr <= wmerge(st.vwr, acc_wd, acc_lo, acc_hi);
					if (acc_hi) begin
						st.wr_pend <= 1'b1;
						st.busy    <= 1'b1;
					end
				end
				5'h0F: st.dcr  <= dcr_m[4:0];
				5'h10: st.sour <= wmerge(st.sour, acc_wd, acc_lo, acc_hi);
				5'h11: st.desr <= wmerge(st.desr, acc_wd, acc_lo, acc_hi);
				5'h12: begin
					st.lenr <= wmerge(st.lenr, acc_wd, acc_lo, acc_hi);
					if (acc_hi)
						st.dma_pend <= 1'b1;
				end
				5'h13: begin
					st.dvssr <= wmerge(st.dvssr, acc_wd, acc_lo, acc_hi);
					if (acc_hi)
						st.dmas_pend <= 1'b1;
				end
				default: ;
			endcase
		end else if (acc_rd) begin
			if (sr_rd) begin
				st.irq_dma  <= 1'b0;
				st.irq_dmas <= 1'b0;
			end
			if (acc_a1 && acc_hi && ar == 5'h02) begin
				st.rd_pend <= 1'b1;
				st.busy    <= 1'b1;
			end
		end

		if (ce_dck) begin
			if (dot0) begin
				if (st.wr_pend) begin
					st.wr_pend  <= 1'b0;
					st.wr_pend2 <= 1'b1;
				end else if (st.wr_pend2) begin
					st.wr_pend2 <= 1'b0;
					st.cpu_addr <= st.mawr;
					st.cpu_data <= st.vwr;
					st.mawr     <= st.mawr + step;
					st.wr_exec  <= 1'b1;
				end

				if (st.rd_pend) begin
					st.rd_pend  <= 1'b0;
					st.rd_pend2 <= 1'b1;
				end else if (st.rd_pend2) begin
					st.rd_pend2 <= 1'b0;
					st.cpu_addr <= st.marr;
					st.marr     <= st.marr + step;
					st.rd_exec  <= 1'b1;
				end
			end

			if (st.dma_pend && (burst || !vdisp)) begin
				st.dma_pend <= 1'b0;
				st.dma_exec <= 1'b1;
				// A new transfer starts with a read, even after one was cut.
				if (!st.dma_exec)
					st.dma_wr <= 1'b0;
			end else if (st.dma_exec && !burst && vdisp) begin
				st.dma_exec <= 1'b0;
			end

			if (cpu_slot) begin
				if (st.dmas_exec) begin
					st.dmas_sat  <= st.dmas_sat + 8'd1;
					st.dmas_vram <= st.dmas_vram + 16'd1;
					if (st.dmas_sat == 8'hFF) begin
						st.dmas_exec <= 1'b0;
						if (st.dcr[0])
							st.irq_dmas <= 1'b1;
					end
				end else if (st.dma_exec) begin
					if (!st.dma_wr) begin
						st.dma_buf <= md_i;
						st.dma_wr  <= 1'b1;
					end else begin
						st.sour <= st.dcr[2] ? st.sour - 16'd1 : st.sour + 16'd1;
						st.desr <= st.dcr[3] ? st.desr - 16'd1 : st.desr + 16'd1;
						st.lenr <= st.lenr - 16'd1;
						if (st.lenr == 16'd0) begin
							st.dma_exec <= 1'b0;
							if (st.dcr[1])
								st.irq_dma <= 1'b1;
						end
						st.dma_wr <= 1'b0;
					end
				end else if (st.wr_exec) begin
					st.wr_exec  <= 1'b0;
					st.busy_clr <= 1'b1;
				end else if (st.rd_exec) begin
					st.rd_exec  <= 1'b0;
					st.vrr      <= md_i;
					st.busy_clr <= 1'b1;
				end
			end

			if (st.busy_clr) begin
				st.busy_clr <= 1'b0;
				st.busy     <= 1'b0;
			end

			// SATB transfer at the start of vertical blanking.
			if (vbl_start && (st.dcr[4] || st.dmas_pend)) begin
				st.dmas_pend <= 1'b0;
				st.dmas_vram <= st.dvssr;
				st.dmas_sat  <= 8'd0;
				st.dmas_exec <= 1'b1;
			end
		end
	end
end

assign ss_so = SAVESTATE ? st[VRAM_BITS-1] : ss_si;

assign busy      = st.busy;
assign vrr       = st.vrr;
assign dma_exec  = st.dma_exec;
assign dmas_exec = st.dmas_exec;
assign irq_dma   = st.irq_dma;
assign irq_dmas  = st.irq_dmas;

assign sat_addr = st.dmas_sat;
assign sat_we   = ce_dck && st.dmas_exec && cpu_slot;

always_comb begin
	ma    = 16'd0;
	md_o  = 16'd0;
	mrd_n = 1'b1;
	mwr_n = 1'b1;
	case (slot)
		SL_BAT, SL_CG0, SL_CG1: begin
			ma    = bg_addr;
			mrd_n = 1'b0;
		end
		SL_SG0, SL_SG1, SL_SG2, SL_SG3: begin
			ma    = spr_addr;
			mrd_n = 1'b0;
		end
		SL_CPU: begin
			if (st.dmas_exec) begin
				ma    = st.dmas_vram;
				mrd_n = 1'b0;
			end else if (st.dma_exec && st.dma_wr) begin
				ma    = st.desr;
				md_o  = st.dma_buf;
				mwr_n = 1'b0;
			end else if (st.dma_exec) begin
				ma    = st.sour;
				mrd_n = 1'b0;
			end else if (st.wr_exec) begin
				ma    = st.cpu_addr;
				md_o  = st.cpu_data;
				mwr_n = 1'b0;
			end else begin
				ma    = st.cpu_addr;
				mrd_n = 1'b0;
			end
		end
		default: ;
	endcase
end

endmodule
