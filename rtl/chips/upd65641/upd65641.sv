// NEC uPD65641 glue gate array (PC-FX D65641GM107).
//
//              +--------------------------------------------------+
//   V810 bus --| ga_decode -> ga_bus (READY, SZRQ, lanes, I/O bus)|-- HuC I/O bus
//              |              ga_mmc (DRAM, refresh)              |-- 4 x uPD424800
//              |              ga_rom (ROM, SRAM, FX-BMP, EXT)     |-- local 16-bit bus
//              | ga_itc  ga_tmc  ga_kpc x2  ga_ioc                |-- K-ports, IRQs
//              +--------------------------------------------------+
//
// No pinout is known: pin names follow the FXGA manuals where they give one.
// The glue runs on the 42.95 MHz system clock (ce_sys); the V810 CLK is
// half of it (ce_sck).

module upd65641
	import upd65641_pkg::*;
#(
	parameter bit SAVESTATE   = 1'b1,
	parameter bit DEBUG       = 1'b0,
	parameter int REFRESH_SCK = 335
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_sys,
	input  logic        ce_sck,
	input  logic        ce_timer,
	input  logic        ce_kport,

	input  logic [31:2] a,
	input  logic [31:0] d_i,
	output logic [31:0] d_o,
	output logic        d_oe,
	input  logic  [3:0] be_n,
	input  logic        mrq_n,
	input  logic        rw,
	input  logic        bcyst_n,
	output logic        ready_n,
	output logic        szrq_n,
	output logic        intr,
	output logic  [3:0] intv_n,
	output logic        nmi_n,
	input  logic        adrserr_n,

	output logic        ras_n,
	output logic  [3:0] cas_n,
	output logic  [9:0] ma,
	output logic        we_n,
	output logic        oe_n,

	input  logic [15:0] ld_i,
	output logic [15:0] ld_o,
	output logic        ld_oe,
	output logic        la1,
	output logic        lrd_n,
	output logic        lwr_n,
	output logic        rom_ce_n,
	output logic        bram_cs_n,
	output logic        cartsel_n,
	output logic        exb_cs_n,
	input  logic        xeb_n,
	output logic        erst,

	output logic  [5:1] ioa,
	input  logic [15:0] iod_i,
	output logic [15:0] iod_o,
	output logic        iod_oe,
	output logic        iord_n,
	output logic        iowr_n,
	output logic        cs_sb_n,
	output logic        cs_ra_n,
	output logic        cs_nt_n,
	output logic        cs_7a_n,
	output logic        cs_7b_n,
	output logic        cs_72_n,
	input  logic        busy_7a_n,
	input  logic        busy_7b_n,
	input  logic        busy_72_n,

	input  logic        int7a_n,
	input  logic        int72_n,
	input  logic        int7b_n,
	input  logic        int73_n,
	input  logic        intex_n,

	output logic        k0_latch_n,
	output logic        k0_clk,
	output logic        k0_rw,
	input  logic        k0_data_i,
	output logic        k0_data_o,
	output logic        k0_data_oe,
	output logic        k1_latch_n,
	output logic        k1_clk,
	output logic        k1_rw,
	input  logic        k1_data_i,
	output logic        k1_data_o,
	output logic        k1_data_oe,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so,
	output logic        ss_busy,

	output logic  [7:0] dbg_isr,
	output logic  [3:0] dbg_kind
);

kind_t       dec_kind;
logic  [2:0] dec_chip;
logic  [5:1] dec_ioa;
logic  [3:0] dec_wait;
logic  [1:0] bkup;

// A 16-bit device sees the upper half when byte lanes 1:0 are off.
wire hi = &be_n[1:0];

ga_decode u_decode (
	.a_top    (a[31:24]),
	.a22      (a[22]),
	.a_blk    (a[11:8]),
	.a_reg    (a[5:2]),
	.hi       (hi),
	.mrq_n    (mrq_n),
	.rw       (rw),
	.bkup     (bkup),
	.kind     (dec_kind),
	.chip     (dec_chip),
	.ioa      (dec_ioa),
	.wait_cnt (dec_wait)
);

logic ram_rdy, mmc_busy, io_busy;
logic ss_mmc, ss_iob, ss_itc, ss_tmc, ss_kp0, ss_kp1;

ga_mmc #(
	.SAVESTATE   (SAVESTATE),
	.REFRESH_SCK (REFRESH_SCK)
) u_mmc (
	.clk      (clk),
	.reset    (reset),
	.ce_sys   (ce_sys),
	.ce_sck   (ce_sck),
	.a        (a[31:2]),
	.be_n     (be_n),
	.mrq_n    (mrq_n),
	.rw       (rw),
	.bcyst_n  (bcyst_n),
	.ras_n    (ras_n),
	.cas_n    (cas_n),
	.ma       (ma),
	.we_n     (we_n),
	.oe_n     (oe_n),
	.rdy      (ram_rdy),
	.busy     (mmc_busy),
	.ss_shift (ss_shift),
	.ss_si    (ss_si),
	.ss_so    (ss_mmc)
);

logic        cyc;
kind_t       kind;
logic        wlast;
logic  [5:0] cs_n;
logic        io_wr_done;
logic  [2:0] io_chip;
logic        reg_we;
logic        reg_re;
logic        reg_hi;
logic [15:0] reg_wd;
logic [15:0] reg_rd;

ga_bus #(
	.SAVESTATE (SAVESTATE)
) u_bus (
	.clk        (clk),
	.reset      (reset),
	.ce_sys     (ce_sys),
	.ce_sck     (ce_sck),
	.d_i        (d_i),
	.d_o        (d_o),
	.d_oe       (d_oe),
	.hi         (hi),
	.rw         (rw),
	.bcyst_n    (bcyst_n),
	.ready_n    (ready_n),
	.szrq_n     (szrq_n),
	.dec_kind   (dec_kind),
	.dec_chip   (dec_chip),
	.dec_ioa    (dec_ioa),
	.dec_wait   (dec_wait),
	.ram_rdy    (ram_rdy),
	.ld_i       (ld_i),
	.ld_o       (ld_o),
	.ld_oe      (ld_oe),
	.la1        (la1),
	.xeb_n      (xeb_n),
	.cyc        (cyc),
	.kind       (kind),
	.wlast      (wlast),
	.ioa        (ioa),
	.iod_i      (iod_i),
	.iod_o      (iod_o),
	.iod_oe     (iod_oe),
	.iord_n     (iord_n),
	.iowr_n     (iowr_n),
	.cs_n       (cs_n),
	.busy_n     ({busy_72_n, busy_7b_n, busy_7a_n, 3'b111}),
	.io_wr_done (io_wr_done),
	.io_chip    (io_chip),
	.io_busy    (io_busy),
	.reg_we     (reg_we),
	.reg_re     (reg_re),
	.reg_hi     (reg_hi),
	.reg_wd     (reg_wd),
	.reg_rd     (reg_rd),
	.ss_shift   (ss_shift),
	.ss_si      (ss_mmc),
	.ss_so      (ss_iob)
);

assign {cs_72_n, cs_7b_n, cs_7a_n, cs_nt_n, cs_ra_n, cs_sb_n} = cs_n;

// A posted write outlives the CPU's cycle; a freeze waits for its strobe.
assign ss_busy = mmc_busy || io_busy;

ga_rom u_rom (
	.a_rom     (a[31:28] == 4'hF),
	.mrq_n     (mrq_n),
	.bcyst_n   (bcyst_n),
	.cyc       (cyc),
	.kind      (kind),
	.rw        (rw),
	.wlast     (wlast),
	.hi        (hi),
	.be_even_n ({be_n[2], be_n[0]}),
	.rom_ce_n  (rom_ce_n),
	.bram_cs_n (bram_cs_n),
	.cartsel_n (cartsel_n),
	.exb_cs_n  (exb_cs_n),
	.lrd_n     (lrd_n),
	.lwr_n     (lwr_n)
);

// Register blocks share the CPU address: block by A11:8, register by A7:6.
wire blk_kp  = a[11:8] == 4'h0;
// ERST is 700 alone; the rest of the 7xx block is left to board devices.
wire blk_rst = a[11:8] == 4'h7 && a[7:2] == 6'd0;
wire blk_ioc = a[11:8] == 4'hC;
wire blk_itc = a[11:8] == 4'hE;
wire blk_tmc = a[11:8] == 4'hF;

logic [15:0] rd_itc, rd_tmc, rd_kp0, rd_kp1, rd_ioc;
logic        irq_tm, irq_kp0, irq_kp1;
wire  [6:0]  src = {irq_tm, !intex_n, irq_kp0 || irq_kp1, !int7a_n, !int72_n, !int7b_n, !int73_n};

ga_itc #(
	.SAVESTATE (SAVESTATE)
) u_itc (
	.clk       (clk),
	.reset     (reset),
	.ce_sys    (ce_sys),
	.ce_sck    (ce_sck),
	.we        (reg_we && blk_itc),
	.sel       (a[7:6]),
	.wd        (reg_wd[11:0]),
	.rd        (rd_itc),
	.src       (src),
	.adrserr_n (adrserr_n),
	.intr      (intr),
	.intv_n    (intv_n),
	.nmi_n     (nmi_n),
	.ss_shift  (ss_shift),
	.ss_si     (ss_iob),
	.ss_so     (ss_itc)
);

ga_tmc #(
	.SAVESTATE (SAVESTATE)
) u_tmc (
	.clk      (clk),
	.reset    (reset),
	.ce_sys   (ce_sys),
	.ce_timer (ce_timer),
	.we       (reg_we && blk_tmc),
	.sel      (a[7:6]),
	.wd       (reg_wd),
	.rd       (rd_tmc),
	.irq      (irq_tm),
	.ss_shift (ss_shift),
	.ss_si    (ss_itc),
	.ss_so    (ss_tmc)
);

ga_kpc #(
	.SAVESTATE (SAVESTATE)
) u_kp0 (
	.clk      (clk),
	.reset    (reset),
	.ce_sys   (ce_sys),
	.ce_kport (ce_kport),
	.we       (reg_we && blk_kp && !a[7]),
	.re       (reg_re && blk_kp && !a[7]),
	.sel_data (a[6]),
	.hi       (reg_hi),
	.wd       (reg_wd),
	.rd       (rd_kp0),
	.latch_n  (k0_latch_n),
	.kclk     (k0_clk),
	.rw       (k0_rw),
	.data_i   (k0_data_i),
	.data_o   (k0_data_o),
	.data_oe  (k0_data_oe),
	.irq      (irq_kp0),
	.ss_shift (ss_shift),
	.ss_si    (ss_tmc),
	.ss_so    (ss_kp0)
);

ga_kpc #(
	.SAVESTATE (SAVESTATE)
) u_kp1 (
	.clk      (clk),
	.reset    (reset),
	.ce_sys   (ce_sys),
	.ce_kport (ce_kport),
	.we       (reg_we && blk_kp && a[7]),
	.re       (reg_re && blk_kp && a[7]),
	.sel_data (a[6]),
	.hi       (reg_hi),
	.wd       (reg_wd),
	.rd       (rd_kp1),
	.latch_n  (k1_latch_n),
	.kclk     (k1_clk),
	.rw       (k1_rw),
	.data_i   (k1_data_i),
	.data_o   (k1_data_o),
	.data_oe  (k1_data_oe),
	.irq      (irq_kp1),
	.ss_shift (ss_shift),
	.ss_si    (ss_kp0),
	.ss_so    (ss_kp1)
);

ga_ioc #(
	.SAVESTATE (SAVESTATE)
) u_ioc (
	.clk        (clk),
	.reset      (reset),
	.ce_sys     (ce_sys),
	.we_erst    (reg_we && blk_rst),
	.we_ioc     (reg_we && blk_ioc),
	.sel_erst   (blk_rst),
	.sel        (a[7:6]),
	.wd         (reg_wd[1:0]),
	.rd         (rd_ioc),
	.io_wr_done (io_wr_done),
	.io_chip    (io_chip),
	.io_a2      (ioa[2]),
	.io_d       (iod_o[3:0]),
	.erst       (erst),
	.bkup       (bkup),
	.ss_shift   (ss_shift),
	.ss_si      (ss_kp1),
	.ss_so      (ss_so)
);

always_comb begin
	case (a[11:8])
		4'h0:       reg_rd = a[7] ? rd_kp1 : rd_kp0;
		4'h7:       reg_rd = blk_rst ? rd_ioc : 16'd0;
		4'hC:       reg_rd = rd_ioc;
		4'hE:       reg_rd = rd_itc;
		4'hF:       reg_rd = rd_tmc;
		default:    reg_rd = 16'd0;
	endcase
end

assign dbg_isr  = DEBUG ? {!nmi_n, src} : 8'd0;
assign dbg_kind = DEBUG ? kind : 4'd0;

endmodule
