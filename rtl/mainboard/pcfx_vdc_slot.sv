// One HuC6270 with its HSRM20116 VRAM, wired as on the board: 16-bit bus
// (8/16 pin low), chip A1 on CPU A2, syncs from the HuC6261.
module pcfx_vdc_slot #(
	parameter bit SAVESTATE = 1'b1,
	parameter bit DEBUG     = 1'b0
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_dck,
	input  logic        cs_n,
	input  logic        rd_n,
	input  logic        wr_n,
	input  logic        a1,
	input  logic [15:0] d_i,
	output logic [15:0] d_o,
	output logic        d_oe,
	output logic        busy_n,
	output logic        irq_n,
	input  logic        hsyn_n,
	input  logic        vsyn_n,
	output logic        spbg,
	output logic  [7:0] vd,
	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so,
	input  logic        ss_mem_sel,
	input  logic [16:0] ss_mem_addr,
	input  logic        ss_mem_we,
	input  logic [15:0] ss_mem_wdata,
	output logic [15:0] ss_mem_rdata
);

logic [15:0] ma, md_i, md_o;
logic        mrd_n, mwr_n;

/* verilator lint_off UNUSEDSIGNAL */
logic        hsyn_n_o, hsyn_n_oe, vsyn_n_o, vsyn_n_oe, disp, md_oe, vram_d_oe;
logic  [3:0] dbg_slot;
logic  [9:0] dbg_line;
/* verilator lint_on UNUSEDSIGNAL */

huc6270 #(
	.SAVESTATE (SAVESTATE),
	.DEBUG     (DEBUG)
) u_vdc (
	.clk          (clk),
	.reset        (reset),
	.ce_dck       (ce_dck),
	.cs_n         (cs_n),
	.rd_n         (rd_n),
	.wr_n         (wr_n),
	.a1           (a1),
	.a0           (1'b0),
	.mode8_16     (1'b0),
	.d_i          (d_i),
	.d_o          (d_o),
	.d_oe         (d_oe),
	.busy_n       (busy_n),
	.irq_n        (irq_n),
	.hsyn_n_i     (hsyn_n),
	.hsyn_n_o     (hsyn_n_o),
	.hsyn_n_oe    (hsyn_n_oe),
	.vsyn_n_i     (vsyn_n),
	.vsyn_n_o     (vsyn_n_o),
	.vsyn_n_oe    (vsyn_n_oe),
	.disp         (disp),
	.spbg         (spbg),
	.vd           (vd),
	.ma           (ma),
	.md_i         (md_i),
	.md_o         (md_o),
	.md_oe        (md_oe),
	.mrd_n        (mrd_n),
	.mwr_n        (mwr_n),
	.ss_shift     (ss_shift),
	.ss_si        (ss_si),
	.ss_so        (ss_so),
	.ss_mem_sel   (ss_mem_sel),
	.ss_mem_addr  (ss_mem_addr),
	.ss_mem_we    (ss_mem_we),
	.ss_mem_wdata (ss_mem_wdata),
	.ss_mem_rdata (ss_mem_rdata),
	.dbg_slot     (dbg_slot),
	.dbg_line     (dbg_line)
);

sram_srm20116 u_vram (
	.clk  (clk),
	.cs_n (1'b0),
	.oe_n (mrd_n),
	.we_n (mwr_n),
	.a    (ma),
	.d_i  (md_o),
	.d_o  (md_i),
	.d_oe (vram_d_oe)
);

endmodule
