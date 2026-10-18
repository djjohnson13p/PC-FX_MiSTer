`timescale 1ns/1ps
`default_nettype none

// PC-FX SDRAM memories at real speed: main RAM, BIOS ROM, FX-BMP SRAM and
// KRAM A/B on one SDR SDRAM. Every read returns inside the real part's access
// time; nothing waits or stretches a bus cycle.
//
// clk_sdram = 3 x clk_sys, edges aligned. One SCK = 2 clk_sys = 6 SDRAM
// clocks, p0 on the clk_sys edge the V810 launches on (k). Sockets are
// clk_sys levels sampled once per SCK at p1, 7.76 ns after k: anything for a
// cycle starting at k must settle within ~6 ns of k.
//
//   main RAM  uPD424800 pins from the glue. RAS fall opens the row in banks
//             2/3, RAS rise closes it, CAS fall with RAS low is a column
//             access (we_n, lanes from cas_n), CAS low with RAS high a CBR
//             refresh. Write data mr_d is taken at p5, two SDRAM clocks
//             after the CPU's phi1 edge (the SDC allows that), and written
//             in the next SCK. mr_q valid from k+3 for a CAS seen at k, held
//             until the next read.
//   ROM       rom_ce_n & rom_oe_n low at k: a 4-SCK read of rom_a, rom_q
//             valid from k+7. A new read is any low sample 4 SCK later.
//   FX-BMP    fx_cs_n with fx_rd_n / fx_wr_n falling: one byte at fx_a, fx_q
//             valid 2*FX_RD_SCK-1 clk_sys after the sample.
//   KRAM A/B  kx_req in the first clk_sys cycle of a KING dot (ce_dot, at
//             least 8 clk_sys, starting on a ce_sck edge). kx_q valid 8
//             clk_sys after the dot start, held until 8 after the next.
//
// SDRAM map, 9 column bits (1 KB rows) on 32 and 64 MB modules:
//   bank 0  rows 0-511 KRAM A, 512-1535 BIOS
//   bank 1  rows 0-511 KRAM B, 512-1535 BIOS copy, 1536-1663 FX-BMP
//   bank 2  main RAM D15:0, row = RAS address, column = CAS address
//   bank 3  main RAM D31:16, same row and column
// The loader writes the BIOS into both banks. Rows from 2048 up only while
// frozen: A11/A12 double as DQM.
//
// p2 is the CPU's command slot, decoded straight from the p1 sample; p3/p5
// take the rest of the CPU side when needed. Other slots go to KRAM, ROM,
// FX-BMP and row refresh by urgency. Refresh while running is ACT+PRE of
// used rows (banks 2/3 inside the glue's CBR); AUTO REFRESH only while frozen.
//
// Freeze: raise freeze with the board stopped; frozen rises when in-flight
// work has drained, and the bulk port (4-phase req/ack, raw {bank, row,
// column}) works only then. Drop freeze and wait for frozen low before
// restarting the board: the open main-RAM page is reopened first. pg_q is
// that page ({open, row}); a savestate load writes it back with pg_we while
// frozen. reset is power-on only; a machine reset is a freeze.
//
// Read capture: the DQ input cell runs on clk_cap, a copy of clk_sdram whose
// PLL phase this module steps, and feeds two clk_sdram registers per bit:
// dq_cap_q in the controller, which takes every beat, and mr_q's high half.
// Before the memory reports frozen (power-up and every freeze) it writes a
// pattern to bank 0 row 1600, reads it back to back at every phase step on
// both halves of the input cell, and settles on the middle of the widest
// passing run. A step passes only if every beat was in both registers on
// its fixed clk_sdram edge, so a run ends wherever the beat's valid window
// or a cell-to-register hop runs out. cal_width/cal_phase/cal_rise report
// the result.
//
// Background port: one FX-BMP card word (bank 1, row 1536 + bg_addr[15:9])
// read or written (both bytes) while the board runs, 4-phase like the bulk
// port, bg_q valid while bg_ack. It only takes slots no other client wants,
// opens bank 1 after this dot's KRAM is done and closes it before the next
// dot; no deadline. It waits while frozen.
module pcfx_memory #(
	parameter int unsigned CLK_SDRAM_HZ = 128_863_636,
	parameter int unsigned SDRAM_TIMING_GRADE = 7,
	parameter int unsigned DQ_CAPTURE_PIPELINE = 3,
	parameter bit          DQ_CAPTURE_RISING = 1'b0,
	parameter int unsigned FX_RD_SCK = 8,
	parameter int unsigned FROZEN_REFI = 250,
	parameter int unsigned CAP_STEP_PS = 243      // one PLL phase step (VCO / 8)
)
(
	input  wire         clk_sys,
	input  wire         clk_sdram,
	input  wire         clk_cap,
`ifdef PCFX_SIM_CAPTURE
	output wire  [15:0] sim_cell_h,
	output wire  [15:0] sim_cell_l,
	input  wire  [15:0] sim_dq_h,
	input  wire  [15:0] sim_dq_l,
	input  wire  [15:0] sim_mr_h,
	input  wire  [15:0] sim_mr_l,
`endif
	input  wire         reset,
	input  wire         ce_sck,
	input  wire         ce_dot,

	input  wire         mr_ras_n,
	input  wire   [3:0] mr_cas_n,
	input  wire   [9:0] mr_ma,
	input  wire         mr_we_n,
	input  wire  [31:0] mr_d,
	output logic [31:0] mr_q,

	input  wire         rom_ce_n,
	input  wire         rom_oe_n,
	input  wire  [19:1] rom_a,
	output logic [15:0] rom_q,

	input  wire         fx_cs_n,
	input  wire         fx_rd_n,
	input  wire         fx_wr_n,
	input  wire  [16:0] fx_a,
	input  wire   [7:0] fx_d,
	output logic  [7:0] fx_q,

	input  wire         ka_req,
	input  wire         ka_we,
	input  wire   [8:0] ka_row,
	input  wire   [8:0] ka_col,
	input  wire   [1:0] ka_be,
	input  wire  [15:0] ka_d,
	output logic [15:0] ka_q,

	input  wire         kb_req,
	input  wire         kb_we,
	input  wire   [8:0] kb_row,
	input  wire   [8:0] kb_col,
	input  wire   [1:0] kb_be,
	input  wire  [15:0] kb_d,
	output logic [15:0] kb_q,

	input  wire         freeze,
	output logic        frozen,
	input  wire         bk_req,
	input  wire         bk_we,
	input  wire  [23:0] bk_addr,
	input  wire  [15:0] bk_d,
	input  wire   [1:0] bk_be,
	output logic        bk_ack,
	output logic [15:0] bk_q,

	input  wire         bg_req,
	input  wire         bg_we,
	input  wire  [15:0] bg_addr,
	input  wire  [15:0] bg_d,
	output logic        bg_ack,
	output logic [15:0] bg_q,

	output wire  [10:0] pg_q,
	input  wire         pg_we,
	input  wire  [10:0] pg_d,

	// PLL dynamic phase shift for clk_cap (scanclk = clk_sys).
	output logic        dps_phase_en,
	output logic        dps_updn,
	input  wire         dps_phase_done,
	// Calibration result: passing window in phase steps, chosen phase in
	// signed steps from the PLL's static phase, and the input-cell half.
	output logic  [5:0] cal_width,
	output logic  [5:0] cal_phase,
	output logic        cal_rise,
	output logic        cal_ok,             // a window was found and the result read back
	output logic        cal_done,           // clk_sys pulse after each calibration

	output logic        late,

	output wire         SDRAM_CLK,
	output wire         SDRAM_CKE,
	output wire  [12:0] SDRAM_A,
	output wire   [1:0] SDRAM_BA,
	inout  wire  [15:0] SDRAM_DQ,
	output wire         SDRAM_DQML,
	output wire         SDRAM_DQMH,
	output wire         SDRAM_nCS,
	output wire         SDRAM_nCAS,
	output wire         SDRAM_nRAS,
	output wire         SDRAM_nWE
);

	localparam logic [3:0] C_NOP = 4'b0111;
	localparam logic [3:0] C_ACT = 4'b0011;
	localparam logic [3:0] C_RD  = 4'b0101;
	localparam logic [3:0] C_WR  = 4'b0100;
	localparam logic [3:0] C_PRE = 4'b0010;
	localparam logic [3:0] C_REF = 4'b0001;

	localparam int unsigned CL = (CLK_SDRAM_HZ <= 100_000_000) ? 2 : 3;
	localparam int unsigned RD_LAT = CL + DQ_CAPTURE_PIPELINE;

	// Read-data owners carried down the CAS latency pipe.
	localparam logic [2:0] T_NONE = 3'd0;
	localparam logic [2:0] T_MLO  = 3'd1;
	localparam logic [2:0] T_MHI  = 3'd2;
	localparam logic [2:0] T_CP   = 3'd3;
	localparam logic [2:0] T_KA   = 3'd4;
	localparam logic [2:0] T_KB   = 3'd5;
	localparam logic [2:0] T_BK   = 3'd6;
	localparam logic [2:0] T_BG   = 3'd7;

	// Clocks from accepting a request (the p2 after its sample) to the clock
	// before its result loads into the client's clk_sys register.
	localparam logic [5:0] XF_ROM = 6'd18;
	localparam logic [5:0] XF_FX  = 6'(6 * FX_RD_SCK - 6);
	localparam logic [5:0] XF_FXW = 6'd28;     // write done before the next cycle's strobe
	localparam logic [5:0] XF_KR  = 6'd21;

`ifndef SYNTHESIS
	initial begin
		if (CAL_N < 8 || CAL_N > 63)
			$error("pcfx_memory: %0d phase steps per clock; calibration needs 8..63", CAL_N);
		if (RD_LAT > 6 || RD_LAT < 4)
			$error("pcfx_memory: READ-to-data %0d clocks; the page-hit grid needs 4..6", RD_LAT);
		if (CLK_SDRAM_HZ > 143_000_000)
			$error("pcfx_memory: clk_sdram above the -7 part's CL3 limit");
	end
`endif

	// The pin layer clears asynchronously (IOE packing); the scheduler
	// synchronously. Each gets its own copy of the reset.
	logic rst, rst_pins;
	always_ff @(posedge clk_sdram) begin
		rst      <= reset;
		rst_pins <= reset;
	end

	// ---------------------------------------------------------------- pins

	logic  [3:0] cmd, pin_cmd;
	logic  [1:0] cba;
	logic [12:0] ca;
	logic [15:0] cdq;
	logic        coe;
	wire  [15:0] dq_in;                // dq_cap_q: the beat due on the last edge
	wire  [15:0] dq_cell;              // the input cell: the beat due on the next edge
	wire         init_done;

	sdram #(
		.CLK_FREQ_HZ(CLK_SDRAM_HZ),
		.SDRAM_TIMING_GRADE(SDRAM_TIMING_GRADE)
	) u_sdram (
		.clk(clk_sdram),
		.clk_cap(clk_cap),
		.cap_rise(cap_rise_q),
`ifdef PCFX_SIM_CAPTURE
		.sim_cell_h(sim_cell_h),
		.sim_cell_l(sim_cell_l),
		.sim_dq_h(sim_dq_h),
		.sim_dq_l(sim_dq_l),
		.sim_mr_h(sim_mr_h),
		.sim_mr_l(sim_mr_l),
`endif
		.reset(rst_pins),
		.init_done(init_done),
		.cmd(pin_cmd),
		.ba(cba),
		.a(ca),
		.dq_out(cdq),
		.dq_oe(coe),
		.dq_in(dq_in),
		.dq_cell(dq_cell),
		.SDRAM_CLK(SDRAM_CLK),
		.SDRAM_CKE(SDRAM_CKE),
		.SDRAM_A(SDRAM_A),
		.SDRAM_BA(SDRAM_BA),
		.SDRAM_DQ(SDRAM_DQ),
		.SDRAM_DQML(SDRAM_DQML),
		.SDRAM_DQMH(SDRAM_DQMH),
		.SDRAM_nCS(SDRAM_nCS),
		.SDRAM_nCAS(SDRAM_nCAS),
		.SDRAM_nRAS(SDRAM_nRAS),
		.SDRAM_nWE(SDRAM_nWE)
	);


	// ---------------------------------------------------------- phase grid

	// ph and dsl name the edge about to happen. The clk_sdram edge after a
	// ce_sck clk_sys cycle is p1; ce_dot does the same for dot slot 1. dsl
	// stops at 31: a dot can run longer than 24 clocks (one per line does).
	logic       ce_sck_q, ce_dot_q;
	logic [2:0] ph;
	logic [4:0] dsl;
	wire        at_p1 = ce_sck_q && !ce_sck;
	wire        at_d1 = ce_dot_q && !ce_dot;
	// Phase of the slot after the next edge, the one being decided now.
	wire  [2:0] phn = (ph == 3'd5) ? 3'd0 : ph + 3'd1;

	always_ff @(posedge clk_sdram) begin
		ce_sck_q <= ce_sck;
		ce_dot_q <= ce_dot;
		ph  <= at_p1 ? 3'd2 : phn;
		dsl <= at_d1 ? 5'd2 : (dsl == 5'd31) ? 5'd31 : dsl + 5'd1;
	end

	// ------------------------------------------------------------ sampling

	logic        s_ras_n, p_ras_n;
	logic  [3:0] s_cas_n;
	logic  [9:0] s_ma;
	logic        s_we_n;
	logic        s_rom;
	logic [19:1] s_rom_a;
	logic        s_fxr, s_fxw, p_fxr, p_fxw;
	logic [16:0] s_fx_a;
	logic        s_rd;                  // the sample is a main-RAM read column
	logic        p_cas_h;               // the previous p1 sample had CAS high
	logic        fresh;                 // the p1 sample was just taken
	logic        arm;                   // ... and the one before had CAS high

	// A savestate load sets the page while frozen; the kept RAS sample
	// follows it so resuming sees no RAS edge.
	wire         pg_load = pg_we && freeze && frozen;

	always_ff @(posedge clk_sdram) begin
		// The sockets go straight into plain registers on every edge: with no
		// load enable in front, a strobe settling late still makes p1. Only
		// logic between the p1 and p2 edges reads them, while they hold the p1
		// sample; what the previous sample showed is kept from its p2 edge.
		s_ras_n <= mr_ras_n;
		s_cas_n <= mr_cas_n;
		s_rd    <= !mr_ras_n && !(&mr_cas_n) && mr_we_n;
		s_ma    <= mr_ma;
		s_we_n  <= mr_we_n;
		s_rom   <= !rom_ce_n && !rom_oe_n;
		s_rom_a <= rom_a;
		s_fxr   <= !fx_cs_n && !fx_rd_n;
		s_fxw   <= !fx_cs_n && !fx_wr_n;
		s_fx_a  <= fx_a;
		if (fresh) begin
			p_ras_n <= s_ras_n;
			p_cas_h <= &s_cas_n;
			p_fxr   <= s_fxr;
			p_fxw   <= s_fxw;
		end
		if (pg_load) p_ras_n <= !pg_d[10];
		fresh <= at_p1;
		arm   <= at_p1 && p_cas_h;
	end

	// Main-RAM pin events, valid between the p1 and p2 edges.
	logic cbr_q;
	wire  ras_lo   = !s_ras_n;
	wire  cas_lo   = !(&s_cas_n);
	wire  ev_rd    = arm && ras_lo && cas_lo && s_we_n;
	wire  ev_wr    = arm && ras_lo && cas_lo && !s_we_n;
	wire  ev_fall  = fresh && ras_lo && p_ras_n && !cas_lo;
	wire  ev_rise  = fresh && !ras_lo && !p_ras_n && !cbr_q;
	wire  ev_cbr   = arm && !ras_lo && cas_lo;

	// ------------------------------------------------------------- state

	typedef enum logic [2:0] {
		M_INIT,
		M_RUN,
		M_DRAIN,
		M_FROZEN,
		M_RESUME,
		M_CAL
	} mode_e;
	mode_e mode;

	// Commands reach the pins one clock after they are decided: nx holds the
	// next slot's command. The CPU's p2 slot bypasses it (see p2 below).
	logic  [3:0] nx_cmd;
	logic        nx_rd, nx_act, nx_nop;  // nx_cmd is a READ / an ACT / a NOP
	logic  [1:0] nx_ba;
	logic [12:0] nx_a;
	logic [15:0] nx_dq;
	logic        nx_oe;
	logic  [2:0] nx_tag;
	logic        p2_rd_ok;              // the next edge is p2 with no write posted
	// Keep the pin CAS decode off the shared command net.
	(* preserve, dont_merge *) logic p2_pin_ok;
	logic        grid_ok;               // phase grid synced since the last resume

	// CPU side.
	logic        wp_v;                  // posted write for the next p2/p3
	logic        wp_lo;                 // ... with a byte in D15:0
	logic  [3:0] wp_be;
	logic  [8:0] wp_col;
	logic [31:0] wp_d;
	logic        c5_v;                  // bank-3 command booked for p5
	logic        c5_act;                // ACT (else PRE)
	logic  [9:0] c5_row;
	logic        mr_open;
	logic  [9:0] mr_row;
	logic  [1:0] cpq;                   // SCKs before the CPU can start a ROM/FX cycle
	logic  [2:0] rom_hold;              // SCKs left of the current ROM cycle

	// ROM or FX-BMP access, one at a time.
	logic        cp_v, cp_act, cp_col, cp_fx, cp_we, cp_bank, cp_got;
	logic [10:0] cp_row;
	logic  [8:0] cp_ca;
	logic  [1:0] cp_be;
	logic        cp_lane;
	logic [15:0] cp_d;
	logic  [1:0] cp_rcd;
	logic  [5:0] cp_xf;
	logic        cp_xfq;
	logic [15:0] cp_hold;
	logic        bk_land;               // a bulk read beat is in dq_cap_q

	// Background port.
	logic        bgq_req, bgq_we;
	logic [15:0] bgq_addr, bgq_d;
	logic        bg_v;                  // access waiting for its column
	logic        bg_act;                // ... with its row open
	logic  [1:0] bg_rcd;
	logic        bg_fly;                // read beat on its way
	logic        bg_land;               // ... and in dq_cap_q
	logic        rom_pref;

	// KRAM A and B.
	logic        k_v [2], k_act [2], k_col [2], k_we [2], k_got [2];
	logic  [8:0] k_row [2], k_ca [2];
	logic  [1:0] k_be [2];
	logic [15:0] k_d [2];
	logic  [1:0] k_rcd [2];
	logic  [4:0] k_xf [2];
	logic        k_xfq [2];
	logic [15:0] k_hold [2];

	// Row refresh: one bank-0/1 row at a time; banks 2/3 inside a CBR.
	logic        ra_v, ra_act, ra_bank, ra_next, ra_dot;
	logic  [2:0] ra_ras;
	logic [10:0] ra_ptr [2];
	logic  [1:0] rc_left [2];
	logic        rc_act [2];
	logic  [2:0] rc_ras [2];
	logic  [9:0] rc_ptr [2];
	logic  [4:0] cbr_win;

	// Bank and bus history, kept in pin time.
	logic  [3:0] brdy [4];              // clocks until an ACT may be on the pins
	logic  [3:0] bopen;                 // row open, waiting for its column or PRE
	logic  [3:0] bok;                   // ready for an ACT one slot after the next
	logic  [2:0] rd_hist;               // READ on the pins 1..3 clocks ago
	logic        wr_quiet;              // no READ on the pins 1..4 clocks ago
	logic  [2:0] rtag [6];
	logic  [2:0] tag_r;                 // the beat now in dq_cap_q
	logic [15:0] lo_hold;
	logic [15:0] hi_hold;

	// Frozen-mode sequencer, and the bulk port taken into the SDRAM clock.
	logic        bkq_req, bkq_we;
	logic [23:0] bkq_addr;
	logic [15:0] bkq_d;
	logic  [1:0] bkq_be;
	logic  [2:0] fz_st;
	logic  [5:0] fz_w;
	logic [10:0] fz_ref;

	wire sched   = (mode == M_RUN) || (mode == M_DRAIN);
	wire run_q   = (mode == M_RUN);
	wire fz_mode = (mode == M_FROZEN) || (mode == M_RESUME) || (mode == M_CAL);

	// ---------------------------------------------------- CPU p2 command

	// nx holds the write posted from the previous SCK (or a NOP) for p2; a
	// main-RAM READ of D15:0 replaces it straight from the p1 sample. Only
	// nCAS and the column depend on the sample.
	wire         p2_rd  = p2_rd_ok && s_rd;

	// ------------------------------------------------ CPU p3 and p5 command

	// Decided at the p2 edge from the same sample: the D31:16 half of a read
	// or posted write, PRE / ACT bank 2 on RAS rise / fall (bank 3 follows at
	// p5).
	logic  [3:0] c3_cmd;
	logic  [1:0] c3_ba;
	logic [12:0] c3_a;
	logic        c3_oe;
	logic  [2:0] c3_tag;

	always_comb begin
		c3_cmd = C_NOP;
		c3_ba  = 2'd3;
		c3_a   = 13'd0;
		c3_oe  = 1'b0;
		c3_tag = T_NONE;
		if (wp_v) begin
			if (wp_be[3:2] != 2'b00) begin
				c3_cmd = C_WR;
				c3_a   = {~wp_be[3], ~wp_be[2], 2'b00, wp_col};
				c3_oe  = 1'b1;
			end
		end else if (ev_rd) begin
			c3_cmd = C_RD;
			c3_a   = {4'b0000, s_ma[8:0]};
			c3_tag = T_MHI;
		end else if (ev_rise) begin
			c3_cmd = C_PRE;
			c3_ba  = 2'd2;
		end else if (ev_fall) begin
			c3_cmd = C_ACT;
			c3_ba  = 2'd2;
			c3_a   = {3'b000, s_ma};
		end
	end

	// ------------------------------------------------- other-slot choice

	// Every rule below is for the slot after the next edge (phase phn). The
	// slot before it is in nx; older slots are in the pin-time history.
	wire slot_free = (phn == 3'd0) || (phn == 3'd1) || (phn == 3'd4) || ((phn == 3'd5) && !c5_v);

	// DQ turnaround: a WRITE may drive DQ only once the last READ's beat has
	// come back through the board and been captured, RD_LAT clocks after the
	// READ. That is one clock more than the data sheet's idle-cycle rule. A
	// READ must see no WRITE in the next five clocks; the only writes booked
	// that far ahead are the CPU's posted ones.
	wire rd_ok  = !wp_v;
	wire wr_ok  = wr_quiet && !nx_rd;
	// tRRD: no ACT next to another, including the CPU's ACT at p5.
	wire act_ok = !nx_act && !((phn == 3'd4) && c5_v && c5_act);

	// A bank may take an ACT in the decided slot when its countdown ends by
	// then and nx does not touch it.
	wire nx_busy = !nx_nop;
	wire b_rdy0 = bok[0] && !(nx_busy && nx_ba == 2'd0);
	wire b_rdy1 = bok[1] && !(nx_busy && nx_ba == 2'd1);
	wire b_rdy2 = bok[2] && !(nx_busy && nx_ba == 2'd2);
	wire b_rdy3 = bok[3] && !(nx_busy && nx_ba == 2'd3);

	wire rom_new = fresh && s_rom && (rom_hold == 3'd0);
	wire fx_new  = fresh && ((s_fxr && !p_fxr) || (s_fxw && !p_fxw));
	wire cp_wait = cp_v && !cp_act;
	wire cp_open = cp_v && cp_act && !cp_col;
	wire ka_wait = k_v[0] && !k_act[0];
	wire kb_wait = k_v[1] && !k_act[1];
	wire hi_wait = cp_wait || ka_wait || kb_wait;

	// The ROM takes whichever BIOS copy is free, preferring one whose KRAM
	// for this dot is already out of the way.
	wire rom_b0  = b_rdy0 && (!ka_wait || !b_rdy1 || (kb_wait && !rom_pref));
	wire rom_b1  = b_rdy1;
	wire rom_go  = cp_wait && !cp_fx && act_ok && (rom_b0 || rom_b1);
	wire rom_bk  = rom_b0 ? 1'b0 : 1'b1;

	// FX-BMP shares bank 1 with KRAM B and goes once this dot's KRAM B is
	// out of the way. The CPU does nothing else during an FX-BMP cycle, so
	// the next dot's KRAM B has the room to follow it. Close to its deadline
	// it stops waiting for KRAM B.
	wire fx_go   = cp_wait && cp_fx && act_ok && b_rdy1 && !kb_wait;
	wire fx_now  = cp_wait && cp_fx && act_ok && b_rdy1 && (cp_xf < 6'd20);

	wire cp_c    = cp_v && cp_act && !cp_col && (cp_rcd == 2'd3) && (cp_we ? wr_ok : rd_ok);
	wire ka_c    = k_v[0] && k_act[0] && !k_col[0] && (k_rcd[0] == 2'd3) && (k_we[0] ? wr_ok : rd_ok);
	wire kb_c    = k_v[1] && k_act[1] && !k_col[1] && (k_rcd[1] == 2'd3) && (k_we[1] ? wr_ok : rd_ok);
	wire ka_go   = ka_wait && act_ok && b_rdy0;
	wire kb_go   = kb_wait && act_ok && b_rdy1;

	wire ra_pre  = ra_v && ra_act && (ra_ras == 3'd6);
	wire r2_pre  = rc_act[0] && (rc_ras[0] == 3'd6);
	wire r3_pre  = rc_act[1] && (rc_ras[1] == 3'd6);
	// CBR rows: an ACT only while its PRE still fits well inside the window.
	wire r2_go   = (rc_left[0] != 2'd0) && !rc_act[0] && b_rdy2 && act_ok && !hi_wait && (cbr_win >= 5'd11);
	wire r3_go   = (rc_left[1] != 2'd0) && !rc_act[1] && b_rdy3 && act_ok && !hi_wait && (cbr_win >= 5'd11);
	// Bank 0/1 rows: one per dot, after this dot's KRAM on that bank, early
	// enough to be closed again before the next dot's even if its PRE slips
	// a few clocks, and with the other bank (or a CPU still busy) to take a
	// ROM read that shows up meanwhile.
	wire ra_ok0  = b_rdy0 && (!k_v[0] || k_act[0]) && (b_rdy1 || (cpq != 2'd0));
	wire ra_ok1  = b_rdy1 && (!k_v[1] || k_act[1]) && (b_rdy0 || (cpq != 2'd0));
	// The bank whose turn it is, else the other one.
	wire ra_bk   = ra_next ? (ra_ok1 ? 1'b1 : 1'b0) : (ra_ok0 ? 1'b0 : 1'b1);
	wire ra_go   = run_q && !ra_v && !ra_dot && act_ok && !hi_wait && (ra_ok0 || ra_ok1)
		&& (dsl >= 5'd1) && (dsl <= 5'd15) && !bg_act;
	// Background: bank 1 opens only with both of this dot's KRAM columns out,
	// no ROM/FX-BMP access or row refresh going and bank 0 free for a ROM read
	// that shows up meanwhile, early enough to close before the next dot.
	// Near the end of the dot it closes at once, with a PRE if need be.
	wire bg_go   = run_q && bg_v && !bg_act && !cp_v && !ra_v && act_ok && b_rdy0 && b_rdy1
		&& (!k_v[0] || k_col[0]) && (!k_v[1] || k_col[1]) && (dsl >= 5'd2) && (dsl <= 5'd12);
	wire bg_c    = bg_v && bg_act && (bg_rcd == 2'd3) && (bgq_we ? wr_ok : rd_ok);
	wire bg_end  = bg_v && bg_act && (dsl >= 5'd18);

	typedef enum logic [3:0] {
		O_NONE, O_CPC, O_ROM, O_KAC, O_KBC, O_RAP, O_R2P, O_R3P,
		O_KA, O_KB, O_FX, O_R2, O_R3, O_RA, O_BG, O_BGP
	} osel_e;
	osel_e osel;

	// Most urgent first: reads with a deadline, then the ACTs that start
	// them, then work that only has to finish (writes, precharges).
	always_comb begin
		if      (cp_c && !cp_we)       osel = O_CPC;
		else if (rom_go)               osel = O_ROM;
		else if (ka_go)                osel = O_KA;
		else if (kb_go)                osel = O_KB;
		else if (ka_c && !k_we[0])     osel = O_KAC;
		else if (kb_c && !k_we[1])     osel = O_KBC;
		else if (r2_pre)               osel = O_R2P;
		else if (r3_pre)               osel = O_R3P;
		else if (ra_pre)               osel = O_RAP;
		else if (bg_end)               osel = bg_c ? O_BG : O_BGP;
		else if (fx_now)               osel = O_FX;
		else if (cp_c)                 osel = O_CPC;
		else if (ka_c)                 osel = O_KAC;
		else if (kb_c)                 osel = O_KBC;
		else if (fx_go)                osel = O_FX;
		else if (r2_go)                osel = O_R2;
		else if (r3_go)                osel = O_R3;
		else if (ra_go)                osel = O_RA;
		else if (bg_go || bg_c)        osel = O_BG;
		else                           osel = O_NONE;
	end

	logic  [3:0] o_cmd;
	logic  [1:0] o_ba;
	logic [12:0] o_a;
	logic        o_oe;
	logic  [2:0] o_tag;

	always_comb begin
		o_cmd = C_NOP;
		o_ba  = 2'd0;
		o_a   = 13'd0;
		o_oe  = 1'b0;
		o_tag = T_NONE;
		unique case (osel)
			O_CPC: begin
				o_cmd = cp_we ? C_WR : C_RD;
				o_ba  = {1'b0, cp_bank};
				o_a   = {cp_we ? ~cp_be : 2'b00, 2'b10, cp_ca};
				o_oe  = cp_we;
				o_tag = cp_we ? T_NONE : T_CP;
			end
			O_ROM: begin
				o_cmd = C_ACT;
				o_ba  = {1'b0, rom_bk};
				o_a   = {2'b00, cp_row};
			end
			O_FX: begin
				o_cmd = C_ACT;
				o_ba  = 2'd1;
				o_a   = {2'b00, cp_row};
			end
			O_KAC, O_KBC: begin
				o_cmd = k_we[osel == O_KBC] ? C_WR : C_RD;
				o_ba  = {1'b0, osel == O_KBC};
				o_a   = {k_we[osel == O_KBC] ? ~k_be[osel == O_KBC] : 2'b00, 2'b10, k_ca[osel == O_KBC]};
				o_oe  = k_we[osel == O_KBC];
				o_tag = k_we[osel == O_KBC] ? T_NONE : (osel == O_KBC) ? T_KB : T_KA;
			end
			O_KA, O_KB: begin
				o_cmd = C_ACT;
				o_ba  = {1'b0, osel == O_KB};
				o_a   = {4'b0000, k_row[osel == O_KB]};
			end
			O_RAP: begin
				o_cmd = C_PRE;
				o_ba  = {1'b0, ra_bank};
			end
			O_R2P, O_R3P: begin
				o_cmd = C_PRE;
				o_ba  = {1'b1, osel == O_R3P};
			end
			O_RA: begin
				o_cmd = C_ACT;
				o_ba  = {1'b0, ra_bk};
				o_a   = {2'b00, ra_ptr[ra_bk]};
			end
			O_R2, O_R3: begin
				o_cmd = C_ACT;
				o_ba  = {1'b1, osel == O_R3};
				o_a   = {3'b000, rc_ptr[osel == O_R3]};
			end
			O_BG: begin
				o_ba  = 2'd1;
				if (bg_act) begin
					o_cmd = bgq_we ? C_WR : C_RD;
					o_a   = {4'b0010, bgq_addr[8:0]};
					o_oe  = bgq_we;
					o_tag = bgq_we ? T_NONE : T_BG;
				end else begin
					o_cmd = C_ACT;
					o_a   = {6'b001100, bgq_addr[15:9]};
				end
			end
			O_BGP: begin
				o_cmd = C_PRE;
				o_ba  = 2'd1;
			end
			default: ;
		endcase
	end

	// ------------------------------------------------ capture calibration

	localparam longint unsigned CAP_T_PS = 64'd1_000_000_000_000 / 64'(CLK_SDRAM_HZ);
	// Phase steps in one clock period; the sweep covers one period per half.
	localparam int unsigned CAL_N = int'((CAP_T_PS + 64'(CAP_STEP_PS) - 64'd1) / 64'(CAP_STEP_PS));
	localparam logic [12:0] CAL_ROW = 13'd1600;

	typedef enum logic [3:0] {
		CA_WACT, CA_WCOL, CA_WPRE, CA_DOWN, CA_TACT, CA_TRD, CA_TPRE, CA_TEND,
		CA_SCAN, CA_MOVE, CA_DONE
	} cal_e;
	cal_e             ca_st;
	logic       [4:0] ca_col;           // column being written or read
	logic       [5:0] ca_rc;            // beats checked in this pass
	logic             ca_err;
	logic             ca_land;          // a calibration beat is in dq_cap_q and mr_q
	logic       [5:0] ca_i;             // phase step under test
	logic             ca_h;             // half under test: 0 rising, 1 falling
	logic      [63:0] pass_r, pass_f;   // bits CAL_N and up stay clear
	logic       [5:0] ph_idx;           // clk_cap phase in steps above the sweep start
	logic       [6:0] sc_i, run_st, best_st;   // the scan goes round twice
	logic       [5:0] run_len, best_len;
	logic             sc_h, best_h;
	logic             cap_rise_q;
	logic             ca_ver;           // reading back at the chosen setting
	logic             ca_try;           // the read-back already failed once
	logic             cal_tog;          // flips at the end of each calibration
	logic             dps_req, dps_up, dps_ack;
	wire              dps_busy = (dps_req != dps_ack);
	// No bank is open in these states, so an AUTO REFRESH may go first.
	wire              cal_safe = (ca_st == CA_WACT) || (ca_st == CA_DOWN) || (ca_st == CA_TACT)
		|| (ca_st == CA_SCAN) || (ca_st == CA_MOVE) || (ca_st == CA_DONE);
	// A whole write or read pass fits before the next AUTO REFRESH is due.
	wire              cal_room = (fz_ref >= 11'd80);
	// The sweep covers one clock period, so a run may wrap from the last step
	// to the first: the scan goes round twice and runs stop at CAL_N.
	wire        [5:0] sc_idx   = (sc_i >= 7'(CAL_N)) ? 6'(sc_i - 7'(CAL_N)) : sc_i[5:0];
	wire              sc_bit   = sc_h ? pass_f[sc_idx] : pass_r[sc_idx];
	// Middle of the widest run, folded back into one period; no run at all
	// leaves the static phase.
	wire        [7:0] cal_mid  = {1'b0, best_st} + {3'b000, best_len[5:1]};
	wire        [7:0] cal_mid1 = (cal_mid >= 8'(CAL_N)) ? cal_mid - 8'(CAL_N) : cal_mid;
	wire        [5:0] cal_mid2 = (cal_mid1 >= 8'(CAL_N)) ? 6'(cal_mid1 - 8'(CAL_N)) : cal_mid1[5:0];
	wire        [5:0] cal_tgt  = (best_len == 6'd0) ? 6'(CAL_N / 2) : cal_mid2;

	// Consecutive columns differ in every bit, so a beat taken one early or
	// one late never matches.
	function automatic logic [15:0] cal_pat(input logic [4:0] c);
		cal_pat = {4{c[4:1]}} ^ {16{c[0]}};
	endfunction

	// ------------------------------------------------ frozen sequencer

	// One bulk access at a time (ACT, column with auto-precharge), AUTO
	// REFRESH every FROZEN_REFI clocks, and on resume the reopening of the
	// main-RAM page the glue still holds open.
	localparam logic [2:0] F_IDLE  = 3'd0;
	localparam logic [2:0] F_COL   = 3'd1;
	localparam logic [2:0] F_DATA  = 3'd2;
	localparam logic [2:0] F_OPEN3 = 3'd3;
	localparam logic [2:0] F_OPEN  = 3'd4;

	logic tags_idle;
	always_comb begin
		tags_idle = 1'b1;
		for (int i = 0; i < 6; i++)
			if (rtag[i] != T_NONE) tags_idle = 1'b0;
	end

	wire busy_any = wp_v || c5_v || nx_busy || cp_v || k_v[0] || k_v[1] || ra_v || bg_act || bg_fly
		|| (rc_left[0] != 2'd0) || (rc_left[1] != 2'd0) || rc_act[0] || rc_act[1];
	// PRECHARGE ALL closes the CPU's page, so only the timing counts here.
	wire banks_idle = (brdy[0] == 4'd0) && (brdy[1] == 4'd0) && (brdy[2] == 4'd0) && (brdy[3] == 4'd0);

	wire f_go    = fz_mode && (fz_w == 6'd0);
	wire f_ref   = f_go && (fz_st == F_IDLE) && (fz_ref == 11'd0) && ((mode != M_CAL) || cal_safe);
	wire f_cal   = f_go && (mode == M_CAL) && !f_ref;
	wire f_bulk  = f_go && (mode == M_FROZEN) && (fz_st == F_IDLE) && (fz_ref != 11'd0) && bkq_req && !bk_ack;
	wire f_open2 = f_go && (mode == M_RESUME) && (fz_st == F_IDLE) && (fz_ref != 11'd0) && mr_open;
	wire f_col   = f_go && (fz_st == F_COL);
	wire f_open3 = f_go && (fz_st == F_OPEN3);
	wire f_prea  = (mode == M_DRAIN) && ((phn == 3'd0) || (phn == 3'd1) || (phn == 3'd4))
		&& !busy_any && banks_idle && tags_idle && (fz_w == 6'd0);

	logic  [3:0] f_cmd;
	logic  [1:0] f_ba;
	logic [12:0] f_a;
	logic [15:0] f_dq;
	logic        f_oe;
	logic  [2:0] f_tag;

	always_comb begin
		f_cmd = C_NOP;
		f_ba  = bkq_addr[23:22];
		f_a   = 13'd0;
		f_dq  = bkq_d;
		f_oe  = 1'b0;
		f_tag = T_NONE;
		if (f_prea) begin
			f_cmd = C_PRE;
			f_a   = 13'h0400;
		end else if (f_ref) begin
			f_cmd = C_REF;
		end else if (f_cal) begin
			f_ba = 2'd0;
			unique case (ca_st)
				CA_WACT: if (cal_room) begin
					f_cmd = C_ACT;
					f_a   = CAL_ROW;
				end
				CA_TACT: if (cal_room && !dps_busy) begin
					f_cmd = C_ACT;
					f_a   = CAL_ROW;
				end
				CA_WCOL: begin
					f_cmd = C_WR;
					f_a   = {8'd0, ca_col};
					f_dq  = cal_pat(ca_col);
					f_oe  = 1'b1;
				end
				CA_TRD: begin
					f_cmd = C_RD;
					f_a   = {8'd0, ca_col};
					f_tag = T_BK;
				end
				CA_WPRE, CA_TPRE: f_cmd = C_PRE;
				default: ;
			endcase
		end else if (f_bulk) begin
			f_cmd = C_ACT;
			f_a   = bkq_addr[21:9];
		end else if (f_open2) begin
			f_cmd = C_ACT;
			f_ba  = 2'd2;
			f_a   = {3'b000, mr_row};
		end else if (f_open3) begin
			f_cmd = C_ACT;
			f_ba  = 2'd3;
			f_a   = {3'b000, mr_row};
		end else if (f_col) begin
			f_cmd = bkq_we ? C_WR : C_RD;
			f_a   = {bkq_we ? ~bkq_be : 2'b00, 2'b10, bkq_addr[8:0]};
			f_oe  = bkq_we;
			f_tag = bkq_we ? T_NONE : T_BK;
		end
	end

	// ---------------------------------------------------- next command

	wire use_c2 = sched && (phn == 3'd2);
	wire use_c3 = sched && (phn == 3'd3);
	wire use_c5 = sched && (phn == 3'd5) && c5_v;
	// Until the first SCK after a resume the grid phase may still jump, so
	// nothing is planned for the other slots before it.
	wire use_o  = sched && grid_ok && slot_free && (osel != O_NONE);

	// Write data only counts with a WRITE, and the slot choice only ever
	// picks the first ready write in its own order, so the data can follow
	// that shorter choice instead of the slot choice itself.
	wire w_cp = cp_v && cp_act && !cp_col && (cp_rcd == 2'd3) && cp_we;
	wire w_ka = k_v[0] && k_act[0] && !k_col[0] && (k_rcd[0] == 2'd3) && k_we[0];
	wire w_kb = k_v[1] && k_act[1] && !k_col[1] && (k_rcd[1] == 2'd3) && k_we[1];
	wire w_bg = bg_v && bg_act && (bg_rcd == 2'd3) && bgq_we;
	wire [15:0] w_dq = (bg_end && w_bg) ? bgq_d : w_cp ? cp_d : w_ka ? k_d[0] : w_kb ? k_d[1] : bgq_d;

	logic  [3:0] d_cmd;
	logic        d_rd, d_act, d_nop;
	logic  [1:0] d_ba;
	logic [12:0] d_a;
	logic [15:0] d_dq;
	logic        d_oe;
	logic  [2:0] d_tag;

	always_comb begin
		d_cmd = f_cmd;
		d_rd  = (f_cmd == C_RD);
		d_act = (f_cmd == C_ACT);
		d_nop = (f_cmd == C_NOP);
		d_ba  = f_ba;
		d_a   = f_a;
		d_oe  = f_oe;
		d_tag = f_tag;
		if (use_c2) begin
			d_cmd = wp_lo ? C_WR : C_NOP;
			d_rd  = 1'b0;
			d_act = 1'b0;
			d_nop = !wp_lo;
			d_ba  = 2'd2;
			d_a   = wp_lo ? {~wp_be[1], ~wp_be[0], 2'b00, wp_col} : 13'd0;
			d_oe  = wp_lo;
			d_tag = T_NONE;
		end else if (use_c3) begin
			d_cmd = c3_cmd;
			d_rd  = (c3_cmd == C_RD);
			d_act = (c3_cmd == C_ACT);
			d_nop = (c3_cmd == C_NOP);
			d_ba  = c3_ba;
			d_a   = c3_a;
			d_oe  = c3_oe;
			d_tag = c3_tag;
		end else if (use_c5) begin
			d_cmd = c5_act ? C_ACT : C_PRE;
			d_rd  = 1'b0;
			d_act = c5_act;
			d_nop = 1'b0;
			d_ba  = 2'd3;
			d_a   = c5_act ? {3'b000, c5_row} : 13'd0;
			d_oe  = 1'b0;
			d_tag = T_NONE;
		end else if (use_o) begin
			d_cmd = o_cmd;
			d_rd  = (o_cmd == C_RD);
			d_act = (o_cmd == C_ACT);
			d_nop = (o_cmd == C_NOP);
			d_ba  = o_ba;
			d_a   = o_a;
			d_oe  = o_oe;
			d_tag = o_tag;
		end else if (sched && !f_prea) begin
			d_cmd = C_NOP;
			d_rd  = 1'b0;
			d_act = 1'b0;
			d_nop = 1'b1;
			d_a   = 13'd0;
			d_oe  = 1'b0;
			d_tag = T_NONE;
		end
		d_dq  = use_c2 ? wp_d[15:0] : use_c3 ? wp_d[31:16] : sched ? w_dq : f_dq;
	end

	// The pins: what nx decided a clock ago, turned into a READ at p2. nx
	// holds a NOP with a zero address whenever p2_rd can be set, so only
	// nCAS depends on the read itself; the column bits take the sample
	// whenever it could be one (a NOP ignores them).
	assign cmd = {nx_cmd[3:2], nx_cmd[1] && !p2_rd, nx_cmd[0]};
	assign pin_cmd = {nx_cmd[3:2], nx_cmd[1] && !(p2_pin_ok && s_rd), nx_cmd[0]};
`ifndef SYNTHESIS
	always @(posedge clk_sdram) begin
		if (p2_rd_ok && (nx_cmd != C_NOP || nx_a != 13'd0 || nx_ba != 2'd2))
			$error("pcfx_memory: p2 READ over a busy slot");
		if (!rst && (p2_pin_ok != p2_rd_ok || pin_cmd != cmd))
			$error("pcfx_memory: pin command differs from scheduler command");
	end
`endif
	assign pg_q = {mr_open, mr_row};

	assign cba = nx_ba;
	assign ca  = {nx_a[12:9], nx_a[8:0] | ({9{p2_rd_ok}} & s_ma[8:0])};
	assign cdq = nx_dq;
	assign coe = nx_oe;
	wire [2:0] tag_pin = p2_rd ? T_MLO : nx_tag;
	wire [2:0] tag_due = rtag[RD_LAT-1];

	// Clocks until bank b may take an ACT: tRC 9 after ACT; 6 after a column
	// with auto-precharge (row closed once the beat is out, then tRP); tRP 3
	// after PRE; tRFC 9 after AUTO REFRESH.
	function automatic [3:0] at_least;
		input [3:0] cur;
		input [3:0] need;
		begin
			at_least = (cur > need) ? cur - 4'd1 : need - 4'd1;
		end
	endfunction

	// Pin-time bank state after the coming edge.
	logic [3:0] brdy_n [4];
	logic [3:0] bopen_n;
	always_comb begin
		for (int b = 0; b < 4; b++) brdy_n[b] = (brdy[b] != 4'd0) ? brdy[b] - 4'd1 : 4'd0;
		bopen_n = bopen;
		if (cmd == C_ACT) begin
			brdy_n[cba]  = 4'd8;
			bopen_n[cba] = 1'b1;
		end
		if ((cmd == C_RD || cmd == C_WR) && ca[10]) begin
			brdy_n[cba]  = at_least(brdy[cba], 4'd6);
			bopen_n[cba] = 1'b0;
		end
		if (cmd == C_PRE) begin
			if (ca[10]) begin
				for (int b = 0; b < 4; b++) brdy_n[b] = at_least(brdy[b], 4'd3);
				bopen_n = 4'd0;
			end else begin
				brdy_n[cba]  = at_least(brdy[cba], 4'd3);
				bopen_n[cba] = 1'b0;
			end
		end
		if (cmd == C_REF) for (int b = 0; b < 4; b++) brdy_n[b] = 4'd8;
	end

	logic       ra_bump, rc_bump2, rc_bump3;
	logic       ra_bump_bk;

	// ---------------------------------------------------- state update

	always_ff @(posedge clk_sdram) begin
		if (rst) begin
			mode     <= M_INIT;
			frozen   <= 1'b1;
			nx_cmd   <= C_NOP;
			nx_rd    <= 1'b0;
			nx_act   <= 1'b0;
			nx_nop   <= 1'b1;
			nx_ba    <= 2'd0;
			nx_a     <= 13'd0;
			nx_oe    <= 1'b0;
			nx_tag   <= T_NONE;
			p2_rd_ok <= 1'b0;
			p2_pin_ok <= 1'b0;
			grid_ok  <= 1'b0;
			wp_v     <= 1'b0;
			wp_lo    <= 1'b0;
			c5_v     <= 1'b0;
			cbr_q    <= 1'b0;
			mr_open  <= 1'b0;
			mr_row   <= 10'd0;
			cpq      <= 2'd0;
			rom_hold <= 3'd0;
			cp_v     <= 1'b0;
			cp_act   <= 1'b0;
			cp_xf    <= 6'd0;
			cp_xfq   <= 1'b0;
			rom_pref <= 1'b0;
			for (int i = 0; i < 2; i++) begin
				k_v[i]     <= 1'b0;
				k_act[i]   <= 1'b0;
				k_xf[i]    <= 5'd0;
				k_xfq[i]   <= 1'b0;
				ra_ptr[i]  <= 11'd0;
				rc_left[i] <= 2'd0;
				rc_act[i]  <= 1'b0;
				rc_ptr[i]  <= 10'd0;
			end
			ra_v     <= 1'b0;
			ra_act   <= 1'b0;
			ra_next  <= 1'b0;
			ra_dot   <= 1'b0;
			cbr_win  <= 5'd0;
			for (int b = 0; b < 4; b++) brdy[b] <= 4'd0;
			bopen    <= 4'd0;
			bok      <= 4'hF;
			ra_bump  <= 1'b0;
			rc_bump2 <= 1'b0;
			rc_bump3 <= 1'b0;
			rd_hist  <= 3'd0;
			wr_quiet <= 1'b1;
			for (int i = 0; i < 6; i++) rtag[i] <= T_NONE;
			tag_r    <= T_NONE;
			fz_st    <= F_IDLE;
			fz_w     <= 6'd0;
			fz_ref   <= 11'd0;
			bk_ack   <= 1'b0;
			late     <= 1'b0;
			bkq_req  <= 1'b0;
			ca_st    <= CA_WACT;
			ca_col   <= 5'd0;
			ca_rc    <= 6'd0;
			ca_err   <= 1'b0;
			ca_land  <= 1'b0;
			ca_i     <= 6'd0;
			ca_h     <= 1'b0;
			pass_r   <= '0;
			pass_f   <= '0;
			ph_idx   <= 6'(CAL_N / 2);
			sc_i     <= 7'd0;
			sc_h     <= 1'b0;
			run_len  <= 6'd0;
			run_st   <= 7'd0;
			best_len <= 6'd0;
			best_st  <= 7'd0;
			best_h   <= 1'b0;
			cap_rise_q <= DQ_CAPTURE_RISING;
			dps_req  <= 1'b0;
			dps_up   <= 1'b0;
			cal_width <= 6'd0;
			cal_phase <= 6'd0;
			cal_rise <= DQ_CAPTURE_RISING;
			cal_ok   <= 1'b0;
			cal_tog  <= 1'b0;
			ca_ver   <= 1'b0;
			ca_try   <= 1'b0;
			bk_land  <= 1'b0;
			bgq_req  <= 1'b0;
			bg_v     <= 1'b0;
			bg_act   <= 1'b0;
			bg_fly   <= 1'b0;
			bg_land  <= 1'b0;
			bg_ack   <= 1'b0;
		end else begin
			bkq_req  <= bk_req;
			bkq_we   <= bk_we;
			bkq_addr <= bk_addr;
			bkq_d    <= bk_d;
			bkq_be   <= bk_be;
			bgq_req  <= bg_req;
			bgq_we   <= bg_we;
			bgq_addr <= bg_addr;
			bgq_d    <= bg_d;

			nx_cmd <= d_cmd;
			nx_rd  <= d_rd;
			nx_act <= d_act;
			nx_nop <= d_nop;
			nx_ba  <= d_ba;
			nx_a   <= d_a;
			nx_dq  <= d_dq;
			nx_oe  <= d_oe;
			nx_tag <= d_tag;
			// wp_lo only changes on p2, so its value here holds through it.
			p2_rd_ok <= use_c2 && at_p1 && p_cas_h && !wp_lo;
			p2_pin_ok <= use_c2 && at_p1 && p_cas_h && !wp_lo;
			if (at_p1) grid_ok <= 1'b1;
			else if (!sched) grid_ok <= 1'b0;

			// Pin-time history.
			rd_hist  <= {rd_hist[1:0], cmd == C_RD};
			wr_quiet <= (rd_hist == 3'd0) && (cmd != C_RD);
			rtag[0] <= tag_pin;
			for (int i = 1; i < 6; i++) rtag[i] <= rtag[i-1];

			for (int b = 0; b < 4; b++) begin
				brdy[b] <= brdy_n[b];
				bok[b]  <= (brdy_n[b] <= 4'd1) && !bopen_n[b];
			end
			bopen <= bopen_n;
			// Every ACT must find its bank closed and recovered.
			if (cmd == C_ACT && (brdy[cba] != 4'd0 || bopen[cba])) late <= 1'b1;

			// Row pointers move the clock after their ACT was chosen.
			ra_bump  <= 1'b0;
			rc_bump2 <= 1'b0;
			rc_bump3 <= 1'b0;
			if (ra_bump)
				ra_ptr[ra_bump_bk] <= (ra_ptr[ra_bump_bk] == (ra_bump_bk ? 11'd1663 : 11'd1535))
					? 11'd0 : ra_ptr[ra_bump_bk] + 11'd1;
			if (rc_bump2) rc_ptr[0] <= rc_ptr[0] + 10'd1;
			if (rc_bump3) rc_ptr[1] <= rc_ptr[1] + 10'd1;

			// Read data. The beat due on this edge goes into dq_cap_q now; the
			// hold registers copy it from there on the next edge.
			tag_r <= tag_due;
			if (tag_r == T_MLO) lo_hold <= dq_in;
			if (tag_r == T_MHI) hi_hold <= dq_in;
			if (tag_r == T_CP) cp_hold <= dq_in;
			if (tag_r == T_KA) k_hold[0] <= dq_in;
			if (tag_r == T_KB) k_hold[1] <= dq_in;
			if (tag_due == T_CP) cp_got <= 1'b1;
			if (tag_due == T_KA) k_got[0] <= 1'b1;
			if (tag_due == T_KB) k_got[1] <= 1'b1;
			// Calibration checks both registers the input cell feeds.
			ca_land <= (mode == M_CAL) && (tag_due == T_BK);
			if (ca_land) begin
				if (dq_in != cal_pat(ca_rc[4:0]) || mr_q[31:16] != cal_pat(ca_rc[4:0])) ca_err <= 1'b1;
				ca_rc <= ca_rc + 6'd1;
			end
			bk_land <= (tag_due == T_BK);

			// Background handshake: start on a new request, answer once the
			// write is out or the read beat is back.
			bg_land <= (tag_due == T_BG);
			if (bg_land) begin
				bg_q   <= dq_in;
				bg_fly <= 1'b0;
				bg_ack <= 1'b1;
			end
			if (bgq_req && !bg_ack && !bg_v && !bg_fly && !bg_land) bg_v <= 1'b1;
			if (!bgq_req) bg_ack <= 1'b0;
			if (bg_act && bg_rcd != 2'd3) bg_rcd <= bg_rcd + 2'd1;
			if (bk_land) bk_q <= dq_in;

			// ROM / FX-BMP result transfer and deadline. An access that misses
			// its slot still finishes a row it opened.
			if (cp_xf != 6'd0) cp_xf <= cp_xf - 6'd1;
			cp_xfq <= cp_v && (cp_xf == 6'd1) && !cp_we;
			if (cp_v && (cp_xf == 6'd1)) begin
				if (!(cp_we ? cp_col : (cp_got || tag_due == T_CP))) late <= 1'b1;
				if (!cp_act || cp_col) begin
					cp_v   <= 1'b0;
					cp_act <= 1'b0;
				end
			end
			if (cp_act && cp_rcd != 2'd3) cp_rcd <= cp_rcd + 2'd1;

			for (int i = 0; i < 2; i++) begin
				if (k_xf[i] != 5'd0) k_xf[i] <= k_xf[i] - 5'd1;
				k_xfq[i] <= k_v[i] && (k_xf[i] == 5'd1) && !k_we[i];
				if (k_v[i] && (k_xf[i] == 5'd1)) begin
					if (!(k_we[i] ? k_col[i] : (k_got[i] || tag_due == (i == 0 ? T_KA : T_KB)))) late <= 1'b1;
					if (!k_act[i] || k_col[i]) begin
						k_v[i]   <= 1'b0;
						k_act[i] <= 1'b0;
					end
				end
				if (k_act[i] && k_rcd[i] != 2'd3) k_rcd[i] <= k_rcd[i] + 2'd1;
				if (rc_act[i] && rc_ras[i] != 3'd6) rc_ras[i] <= rc_ras[i] + 3'd1;
			end
			if (ra_act && ra_ras != 3'd6) ra_ras <= ra_ras + 3'd1;
			if (cbr_win != 5'd0) cbr_win <= cbr_win - 5'd1;
			if (at_d1) ra_dot <= 1'b0;

			// Once per SCK: how long the CPU is known busy.
			if (at_p1) begin
				if (cpq != 2'd0) cpq <= cpq - 2'd1;
				if (rom_hold != 3'd0) rom_hold <= rom_hold - 3'd1;
			end

			// ---- modes
			if (fz_mode) begin
				if (fz_ref != 11'd0) fz_ref <= fz_ref - 11'd1;
				if (fz_w != 6'd0) fz_w <= fz_w - 6'd1;
			end
			if (!bkq_req) bk_ack <= 1'b0;

			unique case (mode)
				M_INIT: if (init_done) begin
					mode   <= M_CAL;
					ca_st  <= CA_WACT;
					ca_try <= 1'b0;
					fz_ref <= 11'(FROZEN_REFI - 1);
				end

				M_RUN: if (freeze) begin
					mode <= M_DRAIN;
					fz_w <= 6'd3;
				end

				M_DRAIN: begin
					// Let the last write recover (tWR) before PRECHARGE ALL.
					if (busy_any || !banks_idle || !tags_idle) fz_w <= 6'd3;
					else if (fz_w != 6'd0) fz_w <= fz_w - 6'd1;
					if (f_prea) begin
						mode   <= M_CAL;
						ca_st  <= CA_WACT;
						ca_try <= 1'b0;
						fz_st  <= F_IDLE;
						fz_w   <= 6'd3;
						fz_ref <= 11'd0;
					end
				end

				// Write the pattern, step the phase to the bottom of the sweep, read
				// the pattern back at every step on both halves, settle on the
				// middle of the widest passing run and read it back there once more:
				// a step the PLL took twice shows up as a failure and a second sweep.
				M_CAL: begin
					if (f_ref) begin
						fz_w   <= 6'd8;
						fz_ref <= 11'(FROZEN_REFI - 1);
					end else if (f_go) begin
						unique case (ca_st)
							CA_WACT: if (cal_room) begin
								ca_st  <= CA_WCOL;
								ca_col <= 5'd0;
								fz_w   <= 6'd2;
							end
							CA_WCOL: begin
								ca_col <= ca_col + 5'd1;
								if (ca_col == 5'd31) begin
									ca_st <= CA_WPRE;
									fz_w  <= 6'd3;
								end
							end
							CA_WPRE: begin
								ca_st <= CA_DOWN;
								fz_w  <= 6'd3;
							end
							CA_DOWN: if (!dps_busy) begin
								if (ph_idx == 6'd0) begin
									ca_st <= CA_TACT;
									ca_i  <= 6'd0;
									ca_h  <= 1'b0;
								end else begin
									dps_req <= !dps_req;
									dps_up  <= 1'b0;
									ph_idx  <= ph_idx - 6'd1;
								end
							end
							CA_TACT: if (cal_room && !dps_busy) begin
								cap_rise_q <= !ca_h;
								ca_st  <= CA_TRD;
								ca_col <= 5'd0;
								ca_rc  <= 6'd0;
								ca_err <= 1'b0;
								fz_w   <= 6'd2;
							end
							CA_TRD: begin
								ca_col <= ca_col + 5'd1;
								if (ca_col == 5'd31) begin
									ca_st <= CA_TPRE;
									fz_w  <= 6'd2;
								end
							end
							CA_TPRE: begin
								ca_st <= CA_TEND;
								fz_w  <= 6'(RD_LAT + 3);
							end
							CA_TEND: if (ca_ver) begin
								ca_ver <= 1'b0;
								cal_ok <= !ca_err && (ca_rc == 6'd32);
								if (!ca_err && (ca_rc == 6'd32) || ca_try) begin
									ca_st <= CA_DONE;
								end else begin
									ca_try <= 1'b1;
									ca_st  <= CA_DOWN;
								end
							end else begin
								if (ca_h) pass_f[ca_i] <= !ca_err && (ca_rc == 6'd32);
								else      pass_r[ca_i] <= !ca_err && (ca_rc == 6'd32);
								ca_h  <= !ca_h;
								ca_st <= CA_TACT;
								if (ca_h) begin
									if (ca_i == 6'(CAL_N - 1)) begin
										ca_st    <= CA_SCAN;
										sc_i     <= 7'd0;
										sc_h     <= 1'b0;
										run_len  <= 6'd0;
										best_len <= 6'd0;
									end else begin
										ca_i    <= ca_i + 6'd1;
										dps_req <= !dps_req;
										dps_up  <= 1'b1;
										ph_idx  <= ph_idx + 6'd1;
									end
								end
							end
							CA_SCAN: begin
								if (sc_bit && run_len != 6'(CAL_N)) begin
									run_len <= run_len + 6'd1;
									if (run_len == 6'd0) run_st <= sc_i;
									if (run_len + 6'd1 > best_len) begin
										best_len <= run_len + 6'd1;
										best_st  <= (run_len == 6'd0) ? sc_i : run_st;
										best_h   <= sc_h;
									end
								end else if (!sc_bit) begin
									run_len <= 6'd0;
								end
								if (sc_i == 7'(2 * CAL_N - 1)) begin
									sc_i    <= 7'd0;
									run_len <= 6'd0;
									sc_h    <= 1'b1;
									if (sc_h) ca_st <= CA_MOVE;
								end else begin
									sc_i <= sc_i + 7'd1;
								end
							end
							CA_MOVE: if (!dps_busy) begin
								if (ph_idx == cal_tgt && best_len == 6'd0) begin
									cal_ok <= 1'b0;
									ca_st  <= CA_DONE;
								end else if (ph_idx == cal_tgt) begin
									ca_ver <= 1'b1;
									ca_h   <= best_h;
									ca_st  <= CA_TACT;
								end else begin
									dps_req <= !dps_req;
									dps_up  <= 1'b0;
									ph_idx  <= ph_idx - 6'd1;
								end
							end
							default: if (!dps_busy) begin
								cap_rise_q <= (best_len == 6'd0) ? DQ_CAPTURE_RISING : !best_h;
								cal_rise   <= (best_len == 6'd0) ? DQ_CAPTURE_RISING : !best_h;
								cal_width  <= best_len;
								cal_phase  <= ph_idx - 6'(CAL_N / 2);
								cal_tog    <= !cal_tog;
								ca_st  <= CA_WACT;
								mode   <= M_FROZEN;
								frozen <= 1'b1;
								fz_st  <= F_IDLE;
								fz_w   <= 6'd3;
							end
						endcase
					end
				end

				default: begin
					if (f_ref) begin
						fz_w   <= 6'd8;
						fz_ref <= 11'(FROZEN_REFI - 1);
					end else if (f_bulk) begin
						fz_st <= F_COL;
						fz_w  <= 6'd2;
					end else if (f_col) begin
						fz_st <= bkq_we ? F_IDLE : F_DATA;
						fz_w  <= bkq_we ? 6'd5 : 6'(RD_LAT + 1);
						if (bkq_we) bk_ack <= 1'b1;
					end else if (f_go && fz_st == F_DATA) begin
						fz_st  <= F_IDLE;
						bk_ack <= 1'b1;
					end else if (f_open2) begin
						fz_st <= F_OPEN3;
						fz_w  <= 6'd1;
					end else if (f_open3) begin
						fz_st <= F_OPEN;
						fz_w  <= 6'd3;
					end else if (f_go && fz_st == F_OPEN) begin
						fz_st  <= F_IDLE;
						mode   <= M_RUN;
						frozen <= 1'b0;
					end else if (f_go && fz_st == F_IDLE && fz_ref != 11'd0) begin
						if (mode == M_FROZEN && !freeze && !bkq_req && !bk_ack) begin
							mode <= M_RESUME;
						end else if (mode == M_RESUME && !mr_open) begin
							mode   <= M_RUN;
							frozen <= 1'b0;
						end
					end
				end
			endcase

			if (wp_v && ph == 3'd5) wp_d <= mr_d;

			// ---- CPU side, decided at p2 from the p1 sample
			if (sched && phn == 3'd3) begin
				if (wp_v) begin
					wp_v  <= 1'b0;
					wp_lo <= 1'b0;
				end
				if (ev_wr) begin
					wp_v   <= 1'b1;
					wp_lo  <= !(&s_cas_n[1:0]);
					wp_be  <= ~s_cas_n;
					wp_col <= s_ma[8:0];
				end
				if (ev_rd || ev_wr) cpq <= 2'd1;
				if (ev_rise) begin
					mr_open <= 1'b0;
					c5_v    <= 1'b1;
					c5_act  <= 1'b0;
				end
				if (ev_fall) begin
					mr_open <= 1'b1;
					mr_row  <= s_ma;
					c5_v    <= 1'b1;
					c5_act  <= 1'b1;
					c5_row  <= s_ma;
					cpq     <= 2'd2;
				end
				if (ev_cbr) begin
					cbr_q      <= 1'b1;
					rc_left[0] <= 2'd2;
					rc_left[1] <= 2'd2;
					cbr_win    <= 5'd22;
				end else if (fresh && !cas_lo) begin
					cbr_q <= 1'b0;
				end

				// ROM and FX-BMP accesses. A new one never displaces an access
				// still holding a row open (that one is already late).
				if (cp_open && (rom_new || fx_new)) begin
					late <= 1'b1;
				end else if (rom_new) begin
					if (cp_v) late <= 1'b1;
					cp_v     <= 1'b1;
					cp_act   <= 1'b0;
					cp_col   <= 1'b0;
					cp_got   <= 1'b0;
					cp_fx    <= 1'b0;
					cp_we    <= 1'b0;
					cp_row   <= 11'd512 + {1'b0, s_rom_a[19:10]};
					cp_ca    <= s_rom_a[9:1];
					cp_xf    <= XF_ROM;
					rom_hold <= 3'd4;
					cpq      <= 2'd3;
				end else if (fx_new) begin
					if (cp_v) late <= 1'b1;
					cp_v    <= 1'b1;
					cp_act  <= 1'b0;
					cp_col  <= 1'b0;
					cp_got  <= 1'b0;
					cp_fx   <= 1'b1;
					cp_we   <= s_fxw;
					cp_row  <= {4'b1100, s_fx_a[16:10]};
					cp_ca   <= s_fx_a[9:1];
					cp_lane <= s_fx_a[0];
					cp_be   <= s_fx_a[0] ? 2'b10 : 2'b01;
					cp_d    <= {fx_d, fx_d};
					cp_xf   <= s_fxw ? XF_FXW : XF_FX;
					cpq     <= 2'd3;
				end
			end

			if (pg_load) begin
				mr_open <= pg_d[10];
				mr_row  <= pg_d[9:0];
			end

			if (use_c5) c5_v <= 1'b0;

			// KRAM requests, taken on the edge that samples the dot's first
			// clk_sys cycle so every other choice already sees them.
			for (int i = 0; i < 2; i++) begin
				if (at_d1 && (i == 0 ? ka_req : kb_req) && k_v[i] && k_act[i] && !k_col[i]) begin
					late <= 1'b1;
				end else if (at_d1 && (i == 0 ? ka_req : kb_req)) begin
					if (k_v[i]) late <= 1'b1;
					k_v[i]   <= 1'b1;
					k_act[i] <= 1'b0;
					k_col[i] <= 1'b0;
					k_got[i] <= 1'b0;
					k_rcd[i] <= 2'd0;
					k_we[i]  <= (i == 0) ? ka_we  : kb_we;
					k_row[i] <= (i == 0) ? ka_row : kb_row;
					k_ca[i]  <= (i == 0) ? ka_col : kb_col;
					k_be[i]  <= (i == 0) ? ka_be  : kb_be;
					k_d[i]   <= (i == 0) ? ka_d   : kb_d;
					k_xf[i]  <= XF_KR[4:0] + 5'd1;
				end
			end

			// A CBR window that closes with rows left gives them up.
			if (cbr_win == 5'd1) begin
				rc_left[0] <= 2'd0;
				rc_left[1] <= 2'd0;
			end

			// The other slot's pick.
			if (use_o && !use_c3 && !use_c5) begin
				unique case (osel)
					O_CPC: begin
						cp_col <= 1'b1;
						if (cp_xf == 6'd0) cp_v <= 1'b0;
					end
					O_ROM: begin
						cp_act  <= 1'b1;
						cp_rcd  <= 2'd1;
						cp_bank <= rom_bk;
					end
					O_FX: begin
						cp_act  <= 1'b1;
						cp_rcd  <= 2'd1;
						cp_bank <= 1'b1;
					end
					O_KAC, O_KBC: begin
						k_col[osel == O_KBC] <= 1'b1;
						if (k_xf[osel == O_KBC] == 5'd0) k_v[osel == O_KBC] <= 1'b0;
					end
					O_KA: begin
						k_act[0] <= 1'b1;
						k_rcd[0] <= 2'd1;
					end
					O_KB: begin
						k_act[1] <= 1'b1;
						k_rcd[1] <= 2'd1;
					end
					O_RAP: begin
						ra_v   <= 1'b0;
						ra_act <= 1'b0;
					end
					O_R2P, O_R3P: begin
						rc_act[osel == O_R3P]  <= 1'b0;
						if (rc_left[osel == O_R3P] != 2'd0)
							rc_left[osel == O_R3P] <= rc_left[osel == O_R3P] - 2'd1;
					end
					O_RA: begin
						ra_v     <= 1'b1;
						ra_act   <= 1'b1;
						ra_ras   <= 3'd1;
						ra_bank  <= ra_bk;
						ra_dot   <= 1'b1;
						ra_next  <= !ra_bk;
						rom_pref <= ra_bk;
						ra_bump  <= 1'b1;
						ra_bump_bk <= ra_bk;
					end
					O_R2, O_R3: begin
						rc_act[osel == O_R3] <= 1'b1;
						rc_ras[osel == O_R3] <= 3'd1;
						if (osel == O_R3) rc_bump3 <= 1'b1;
						else rc_bump2 <= 1'b1;
					end
					O_BG: begin
						if (!bg_act) begin
							bg_act <= 1'b1;
							bg_rcd <= 2'd1;
						end else begin
							bg_act <= 1'b0;
							bg_v   <= 1'b0;
							if (bgq_we) bg_ack <= 1'b1;
							else bg_fly <= 1'b1;
						end
					end
					O_BGP: bg_act <= 1'b0;
					default: ;
				endcase
			end
		end
	end

	// ------------------------------------------------- clk_sys results

	// Each result register loads on the one clk_sys edge its client reads
	// from; the strobes are SDRAM-side flops set the clock before that edge.
	// mr_q is the exception: a clk_sdram register, loaded on the clk_sdram
	// edge that coincides with clk_sys k+3, the edge the page-hit high half
	// is due on. That half comes straight from the input cell, the low half
	// from dq_cap_q. It changes only on such edges, so its clk_sys readers
	// have three SDRAM periods (sdram.sdc says so). Calibration checks the
	// high half's path too: it takes the calibration beats and then gets its
	// old value back, all while the board is stopped.
	// Its load strobes are flops set the clock before, since the high half
	// sits by the pins, far from the scheduler.
	wire [15:0] mr_hi_src = (RD_LAT == 6) ? dq_cell : (RD_LAT == 5) ? dq_in : hi_hold;
	wire [15:0] mr_lo_src = (RD_LAT == 6) ? dq_in : lo_hold;
	logic        mr_cal, mr_ld, mr_rest, mr_hi_en;
	logic [15:0] mr_keep;
	always_ff @(posedge clk_sdram) begin
		mr_cal   <= (mode == M_CAL);
		mr_ld    <= (rtag[4] == T_MHI);
		mr_rest  <= mr_cal && (mode != M_CAL);
		mr_hi_en <= (rtag[4] == T_MHI) || ((mode == M_CAL) && (rtag[RD_LAT-2] == T_BK))
			|| (mr_cal && (mode != M_CAL));
		if (mode != M_CAL && !mr_cal && !mr_rest) mr_keep <= mr_q[31:16];
		if (mr_ld) mr_q[15:0] <= mr_lo_src;
		if (mr_hi_en) mr_q[31:16] <= mr_rest ? mr_keep : mr_ld ? mr_hi_src : dq_cell;
	end

	// PLL phase steps, one per request: phase_en held until phase_done falls,
	// then phase_done back high. Timeouts keep a missing phase_done from
	// hanging calibration. cal_done is one clk_sys pulse per calibration.
	logic [1:0] dps_st;
	logic [5:0] dps_t;
	logic       cal_tog_s;
	always_ff @(posedge clk_sys) begin
		if (rst) begin
			dps_st       <= 2'd0;
			dps_t        <= 6'd0;
			dps_ack      <= 1'b0;
			dps_phase_en <= 1'b0;
			dps_updn     <= 1'b0;
			cal_tog_s    <= 1'b0;
			cal_done     <= 1'b0;
		end else begin
			cal_tog_s <= cal_tog;
			cal_done  <= (cal_tog != cal_tog_s);
			dps_t     <= dps_t + 6'd1;
			unique case (dps_st)
				2'd0: begin
					dps_t <= 6'd0;
					if (dps_req != dps_ack) begin
						dps_phase_en <= 1'b1;
						dps_updn     <= dps_up;
						dps_st       <= 2'd1;
					end
				end
				2'd1: if (!dps_phase_done || dps_t == 6'd15) begin
					dps_phase_en <= 1'b0;
					dps_t        <= 6'd0;
					dps_st       <= 2'd2;
				end
				default: if (dps_phase_done || dps_t == 6'd63) begin
					dps_ack <= dps_req;
					dps_st  <= 2'd0;
				end
			endcase
		end
	end

	// A beat that came on the deadline edge is still only in dq_cap_q.
	wire [15:0] cp_src = (tag_r == T_CP) ? dq_in : cp_hold;
	always_ff @(posedge clk_sys) begin
		if (cp_xfq) begin
			if (cp_fx) fx_q <= cp_lane ? cp_src[15:8] : cp_src[7:0];
			else rom_q <= cp_src;
		end
		if (k_xfq[0]) ka_q <= (tag_r == T_KA) ? dq_in : k_hold[0];
		if (k_xfq[1]) kb_q <= (tag_r == T_KB) ? dq_in : k_hold[1];
	end

endmodule

`default_nettype wire
