// HuC6272 KING: line positions, shared types and saved state.
//
// kx counts KING dots (DCKKR) through the 341-dot line. kx 32 is window
// dot 0, so window dot n is kx n + 32; the -HSYNC fall is seen at kx 309.
//
//   kx   0 .. 7    8 ............ 271 272 .. 279  280-284  309
//        group -1  fetch groups 0-32  group 33   refresh  -HSYNC
//                  <--- window kx 32-287 --->
//
// kx 31 is the line's long dot (10 OSC clocks, the others 8).

package huc6272_pkg;

	localparam logic [8:0] KX_LAST    = 9'd340;
	localparam logic [8:0] KX_HSYNC   = 9'd309;
	localparam logic [8:0] KX_WIN     = 9'd32;
	localparam logic [8:0] KX_WIN_END = 9'd288;
	localparam logic [8:0] KX_SEQ_END = 9'd280;  // microprogram runs kx 0-279
	localparam logic [8:0] KX_REF     = 9'd280;  // five refresh dots
	localparam logic [8:0] KX_REF_END = 9'd285;

	localparam logic [8:0] RASTER_FIRST = 9'd22;
	localparam logic [8:0] RASTER_LAST  = 9'd261;

	// Status register bits
	localparam int ST_BUSY   = 9;
	localparam int ST_SOUND  = 10;
	localparam int ST_LUSTER = 11;
	localparam int ST_SBCODE = 12;
	localparam int ST_SCDMA  = 13;
	localparam int ST_SCSIBK = 14;

	// One register access from the CPU interface.
	typedef struct packed {
		logic        wr;     // write strobe, one clk
		logic        rd;     // read accepted, one clk (clear-on-read effects)
		logic        ar;     // A2 low: address register / status
		logic        hi;     // A1: upper half
		logic  [6:0] r;      // selected register
		logic [15:0] d;
	} acc_t;

	// A KRAM request into a bank arbiter.
	typedef struct packed {
		logic        v;
		logic        we;
		logic        page;
		logic [16:0] a;
		logic [15:0] d;
	} kreq_t;

	// Microprogram word
	typedef struct packed {
		logic       nop;
		logic [1:0] bg;
		logic       rot;
		logic       bat;
		logic       ind;
		logic [2:0] off;
	} mpw_t;

	// Where a BG slot's data goes when it returns from KRAM.
	typedef struct packed {
		logic       v;
		logic       bat;     // into the BG's BAT latch
		logic [1:0] bg;
		logic [1:0] grp;     // ring group
		logic [2:0] widx;    // word in the group
		logic [3:0] shift;   // rotated: bits to shift the dot to the top
		logic [3:0] pbn;     // palette bank from the BAT
		logic       zero;    // outside every screen: store transparent
	} slot_rec_t;

	// BG registers that take effect at the next -HSYNC.
	typedef struct packed {
		logic             rsw;
		logic [3:0][2:0]  prio;
		logic [3:0][6:0]  bat;    // A16:A10; BG0 main
		logic [3:0][6:0]  cg;
		logic      [6:0]  bat0s;  // BG0 sub
		logic      [6:0]  cg0s;
		logic     [10:0]  sx0;
		logic     [10:0]  sy0;
		logic [2:0][9:0]  sx;     // BG1-3
		logic [2:0][9:0]  sy;
		logic     [15:0]  aa;
		logic     [15:0]  ab;
		logic     [15:0]  ac;
		logic     [15:0]  ad;
		logic     [15:0]  x0;
		logic     [15:0]  y0;
	} hregs_t;

	// Saved state, one struct per block, in shift order.
	typedef struct packed {
		logic [6:0] ar;
		logic       done;    // the current strobe has been acted on
	} cpuif_state_t;

	typedef struct packed {
		logic [8:0] kx;
		logic [8:0] lc;      // -HSYNCs since -VSYNC
		logic [1:0] long_f;  // last two fields had 263 lines
		logic [3:0] ph;
		logic       hs_q;
		logic       vs_q;
		logic       hs_ev;
		logic       vs_ev;
		logic [3:0] init;
	} timing_state_t;

	typedef struct packed {
		logic        page;
		logic  [9:0] inc;
		logic        ab;
		logic [16:0] a;
	} kaddr_t;

	typedef struct packed {
		kaddr_t      kra;
		kaddr_t      kwa;
		logic  [3:0] pages;  // RAP SOP BGP SCP
		logic        mode4m;
		logic        req_v;
		logic        req_we;
		logic        fly;
		logic [15:0] wdata;
	} kram_state_t;

	typedef struct packed {
		logic sound;
		logic luster;
		logic sbcode;
		logic scdma;
		logic scsibk;
	} irq_state_t;

	typedef struct packed {
		hregs_t            h;
		logic [15:0]       mode;
		logic [15:0]       size0;
		logic [2:0][7:0]   size;
		logic  [3:0]       cyl;
	} bg_state_t;

	typedef struct packed {
		logic              mpsw;
		logic  [3:0]       mpa;
		logic [15:0][8:0]  mp;
	} mprog_state_t;

	typedef struct packed {
		logic  [7:0] odata;
		logic  [4:0] icmd;   // RST ACK SEL ATN DBUS
		logic  [1:0] mode;   // DMA SED
		logic  [2:0] tcmd;   // MSG C/D I/O
		logic  [1:0] subc;   // INT EN
		logic [17:0] dma_a;
		logic [17:0] dma_n;  // byte count, LSB written as 0
		logic  [2:0] dmacr;  // C/S INT EN
		// transfer engine
		logic        rx;     // initiator receive running
		logic        tx;     // pseudo-DMA send running
		logic        drq;    // pseudo-DMA byte ready / wanted
		logic  [7:0] din;    // last byte taken from the bus
		logic        lo_v;   // low byte of a KRAM word held
		logic  [7:0] lo;
		logic        w_v;    // word waiting for its KRAM slot
		logic        w_fly;  // ... and in its slot this dot
		logic [15:0] w;
		logic        last;   // the count reached 0
		logic        dend;   // reg 0B DMA END
		logic  [1:0] hs;     // handshake step
		logic        hdma;   // ... for a KRAM DMA byte
		logic  [6:0] ht;     // handshake timer
		logic        ack;    // engine ACK
		logic        req_q;
		logic        rst_q;
		// subcode port
		logic        wf_q;
		logic  [9:0] st;     // clocks since -WFCK fell
		logic  [3:0] sk;     // bits taken, 8 = idle
		logic  [7:0] sh;
		logic  [7:0] sub;
	} scsi_state_t;

	typedef struct packed {
		logic  [3:0]      ctl;   // DV1 DV0 EN2 EN1
		logic [1:0][2:0]  buf_m; // HLF END RNG
		logic [1:0][9:0]  sa;
		logic [1:0][17:0] ea;
		logic [1:0][11:0] ha;
		logic  [1:0]      run;   // channels transferring
		logic [1:0][17:0] ptr;   // read pointers {A/B, A16:0}
		logic  [3:0]      stat;  // reg 53: HALF2 END2 HALF1 END1
		logic  [3:0]      lc;    // lines into the transfer cycle
		logic  [1:0]      pend;  // words still to read this blanking
		logic             fly;   // a word read in this dot
		logic             fly_ch;
		logic             fly_bank;
		logic  [1:0]      have;  // words waiting to go out
		logic [1:0][15:0] word;
		logic             bv;    // a byte on the K-R bus this dot
		logic  [1:0]      bsel;  // its {channel, high byte}
	} adpcm_state_t;

	typedef struct packed {
		logic  [1:0] ctl;    // RINT REN
		logic [17:0] sa;
		logic  [8:0] start;
		logic  [4:0] blocks;
		logic  [8:0] mon;
		logic        act;    // transfer running
		logic  [4:0] bcnt;   // blocks left, this one included
		logic  [3:0] lcnt;   // lines of this block gone
		logic        ab;     // bank of this transfer
		logic [16:0] a;      // next word to read
		logic        fly;    // a read holds this dot
		logic        fly_ab;
		logic  [1:0] n;      // words read ahead
		logic [2:0][15:0] w; // w[0] oldest
		logic [15:0] cur;    // word on the K-R bus
		logic        bv;     // it is sent this dot
	} rb_state_t;

	localparam int CPUIF_BITS  = $bits(cpuif_state_t);
	localparam int TIMING_BITS = $bits(timing_state_t);
	localparam int KRAM_BITS   = $bits(kram_state_t);
	localparam int IRQ_BITS    = $bits(irq_state_t);
	localparam int BG_BITS     = $bits(bg_state_t);
	localparam int MPROG_BITS  = $bits(mprog_state_t);
	localparam int SCSI_BITS   = $bits(scsi_state_t);
	localparam int ADPCM_BITS  = $bits(adpcm_state_t);
	localparam int RB_BITS     = $bits(rb_state_t);
	/* verilator lint_off UNUSEDPARAM */
	localparam int SS_BITS = CPUIF_BITS + TIMING_BITS + KRAM_BITS + IRQ_BITS
	                       + BG_BITS + MPROG_BITS + SCSI_BITS + ADPCM_BITS + RB_BITS;
	/* verilator lint_on UNUSEDPARAM */

	// Size code to log2 dots: 3-10 as written, anything else acts as 3.
	function automatic logic [3:0] size_log2(input logic [3:0] c);
		size_log2 = (c >= 4'd3 && c <= 4'd10) ? c : 4'd3;
	endfunction

	function automatic logic size_bad(input logic [3:0] c);
		size_bad = !(c >= 4'd3 && c <= 4'd10);
	endfunction

endpackage
