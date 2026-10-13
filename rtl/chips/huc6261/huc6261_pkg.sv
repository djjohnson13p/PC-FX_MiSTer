// HuC6261 timing constants, register layouts and saved state.
//
// hc counts master clocks (OSC/2) through a 1365-clock line; hc 0 is the
// first clock of the 256-dot window (hc 0-1023). A signal said to change
// "at hc X" reads its new value in every clk_sys cycle where hc == X.
//
//   hc      0 ........ 1023 1024 .... 1104 ...... 1138 .... 1316-1333 .. 1364
//           |<- window ->|  hblank  -HSYNC falls  -VSYNC   line latch
//                                   raster +1     edges

package huc6261_pkg;

	localparam logic [10:0] H_LAST    = 11'd1364;
	localparam logic [10:0] H_WINDOW  = 11'd1024;
	localparam logic [10:0] H_RASTER  = 11'd1104;  // raster count steps with -HSYNC
	localparam logic [10:0] H_HS      = 11'd1104;  // -HSYNCA/B/C fall, 256 dots
	localparam logic [10:0] H_HS7     = 11'd1128;  // -HSYNCA fall, DC7 = 1
	localparam logic [10:0] H_HS_END  = 11'd1341;  // -HSYNC rise
	localparam logic [10:0] H_VS      = 11'd1138;  // -VSYNC edges
	localparam logic [10:0] H_COPY    = 11'd1316;  // R04-R15 copied, word = hc[4:0]
	localparam logic [10:0] H_COPY_END = 11'd1333;

	// Board-edge output, in output clocks oc = hc - latency
	localparam logic [10:0] LAT_256   = 11'd14;
	localparam logic [10:0] LAT_7     = 11'd9;
	localparam logic [10:0] TV_HS     = 11'd1108;
	localparam logic [10:0] TV_HS_END = 11'd1209;
	localparam logic [10:0] TV_VS_EV  = 11'd425;   // even field: half a line later

	localparam logic [8:0] V_FIRST = 9'd22;
	localparam logic [8:0] V_LAST  = 9'd261;

	localparam logic [23:0] BLACK = 24'h008080;

	// CR (R00) as stored
	typedef struct packed {
		logic [6:0] blank;  // 14:8 display enables, BG SP BMG0-3 BG71
		logic       sp256;  // 7
		logic       bg256;  // 6
		logic       dc7;    // 3
		logic       ex;     // 2
		logic [1:0] dcc;    // 1:0
	} cr_t;

	// CR 7:0, latched at the start of each field. EX has no input to lock to
	// on this board, so only the register keeps it.
	typedef struct packed {
		logic       sp256;
		logic       bg256;
		logic       dc7;
		logic [1:0] dcc;
	} frame_t;

	// R04-R15 and CR 14:8 as latched at each line
	typedef struct packed {
		logic  [6:0] blank;
		logic [15:0] cpo1;
		logic [15:0] cpo2;
		logic [15:0] cpo3;
		logic  [7:0] cpo4;
		logic  [8:0] pri1;  // 6271, SP, BG
		logic [11:0] pri2;  // BMG3-0
		logic [15:0] cky;
		logic [15:0] cku;
		logic [15:0] ckv;
		logic [15:0] ccr;
		logic [15:0] ble;
		logic [15:0] spbl;
		logic [11:0] bl1a;
		logic [11:0] bl1b;
		logic [11:0] bl2a;
		logic [11:0] bl2b;
		logic [11:0] bl3a;
		logic [11:0] bl3b;
	} line_regs_t;

	// One layer candidate for the current dot
	typedef struct packed {
		logic        pres;    // enabled and not key
		logic        is_pal;
		logic  [1:0] cel;     // cellophane coefficient set, 0 = off
		logic  [7:0] idx;     // palette data
		logic  [7:0] off;     // palette offset of the plane
		logic [23:0] yuv;     // direct colour when not palette
	} cand_t;

	// Saved state, in shift order
	typedef struct packed {
		logic [4:0]  ar;
		cr_t         cr;
		logic [8:0]  cpa;
		logic [15:0] cpr;    // R03 read latch
	} cpu_state_t;

	typedef struct packed {
		logic [10:0] hc;
		logic  [8:0] vc;
		logic  [1:0] p3;     // hc mod 3
		logic        oe;
		frame_t      frm;
		logic        vs_n;
		logic        tv_vs;
	} sync_state_t;

	localparam int SS_CPU_BITS  = $bits(cpu_state_t);
	localparam int SS_SYNC_BITS = $bits(sync_state_t);
	/* verilator lint_off UNUSEDPARAM */
	localparam int SS_BITS      = SS_CPU_BITS + SS_SYNC_BITS;
	/* verilator lint_on UNUSEDPARAM */

	// Palette 0-511, R04-R15 at 512 + register number
	localparam int SS_MEM_AW = 10;

	// 8-bit Y and 4-bit U, V to YUV888
	function automatic logic [23:0] expand(input logic [15:0] c);
		expand = {c[15:8], c[7:4], 4'h0, c[3:0], 4'h0};
	endfunction

endpackage
