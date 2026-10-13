// HuC6270 VDC: shared types and saved state.

package huc6270_pkg;

	// VRAM slot owner for the current dot.
	typedef enum logic [3:0] {
		SL_CPU, SL_BAT, SL_CG0, SL_CG1, SL_SG0, SL_SG1, SL_SG2, SL_SG3, SL_NOP
	} slot_t;

	// Display registers R05-R0E and the address register.
	typedef struct packed {
		logic  [4:0] ar;
		logic  [3:0] cr_ie;
		logic  [1:0] cr_ex;
		logic        cr_sb;
		logic        cr_bb;
		logic  [1:0] cr_te;
		logic  [1:0] cr_iw;
		logic  [9:0] rcr;
		logic  [9:0] bxr;
		logic  [8:0] byr;
		logic        byr_set;
		logic  [7:0] mwr;
		logic  [4:0] hsw;
		logic  [6:0] hds;
		logic  [6:0] hdw;
		logic  [6:0] hde;
		logic  [4:0] vsw;
		logic  [7:0] vds;
		logic  [8:0] vdw;
		logic  [7:0] vcr;
	} regs_state_t;

	// CPU VRAM port, block transfers, R00-R02 and R0F-R13.
	typedef struct packed {
		logic [15:0] mawr;
		logic [15:0] marr;
		logic [15:0] vwr;
		logic [15:0] vrr;
		logic  [4:0] dcr;
		logic [15:0] sour;
		logic [15:0] desr;
		logic [15:0] lenr;
		logic [15:0] dvssr;
		logic        rd_pend;
		logic        rd_pend2;
		logic        rd_exec;
		logic        wr_pend;
		logic        wr_pend2;
		logic        wr_exec;
		logic [15:0] cpu_addr;
		logic [15:0] cpu_data;
		logic        busy;
		logic        busy_clr;
		logic        dma_pend;
		logic        dma_exec;
		logic        dma_wr;
		logic [15:0] dma_buf;
		logic        dmas_pend;
		logic        dmas_exec;
		logic  [7:0] dmas_sat;
		logic [15:0] dmas_vram;
		logic        irq_dma;
		logic        irq_dmas;
	} vram_state_t;

	// Dot, character and line counters with their latched geometry.
	typedef struct packed {
		logic        hs_q;
		logic        vs_q;
		logic  [2:0] dot;
		logic  [6:0] tile;
		logic  [2:0] dots_remain;
		logic        tile_zero;
		logic  [4:0] hsw;
		logic  [6:0] hds;
		logic  [6:0] hdw;
		logic  [6:0] hde;
		logic        cm;
		logic  [9:0] line;
		logic        line_inc;
		logic        break_en;
		logic        break_latch;
		logic        vdisp;
		logic        burst;
		logic        bg_fetch;
		logic        bg_out;
		logic  [9:0] rc_cnt;
		logic        rc_upd;
		logic  [4:0] vsw;
		logic  [7:0] vds;
		logic  [8:0] vdw;
		logic  [7:0] vde;
		logic  [1:0] vm;
		logic  [1:0] sm;
		logic  [2:0] screen;
		logic        bb;
		logic        sb;
		logic        vdisp_old;
		logic        irq_vbl;
		logic        irq_rcr;
	} timing_state_t;

	// Sprite status flags; the sprite pipes are rebuilt every line.
	typedef struct packed {
		logic        irq_ovf;
		logic        irq_col;
	} sprite_state_t;

	localparam int REGS_BITS   = $bits(regs_state_t);
	localparam int VRAM_BITS   = $bits(vram_state_t);
	localparam int TIMING_BITS = $bits(timing_state_t);
	localparam int SPRITE_BITS = $bits(sprite_state_t);
	/* verilator lint_off UNUSEDPARAM */
	localparam int SS_BITS     = REGS_BITS + VRAM_BITS + TIMING_BITS + SPRITE_BITS;
	/* verilator lint_on UNUSEDPARAM */

	// A register write lands byte by byte on the 8-bit bus.
	function automatic logic [15:0] wmerge(logic [15:0] old, logic [15:0] wd, logic lo, logic hi);
		return {hi ? wd[15:8] : old[15:8], lo ? wd[7:0] : old[7:0]};
	endfunction

endpackage
