// Savestate slot layout. A slot is a stream of 16-bit elements, four to a
// 64-bit DDR3 word, after four header words:
//
//   w0  {payload dwords, counter}   written last; the HPS saves on a change
//   w1  {"PCFX", layout hash}
//   w2  {BIOS CRC, disc TOC hash}
//   w3  memory open page
//   w4  chain, V810 state and memory, chip RAM ports, SDRAM regions
//       (segments below, 16-bit elements, bytes in the low half)
//
// Every length here feeds the layout hash, so a load of a slot written by
// a different build is refused.

package pcfx_ss_pkg;

	// The clock dividers, the K-port devices (tap, five pads, mouse), then the
	// board (glue, HuC6261, HuC6270 x2, HuC6230, KING, HuC6271, CD drive).
	localparam int CHAIN_BITS = pcfx_clocks_pkg::SS_BITS
		+ kport_pkg::TAP_BITS + 5 * kport_pkg::PAD_BITS + kport_pkg::MOUSE_BITS
		+ upd65641_pkg::SS_BITS
		+ huc6261_pkg::SS_BITS + 2 * huc6270_pkg::SS_BITS + huc6230_pkg::SS_BITS
		+ huc6272_pkg::SS_BITS + huc6271_pkg::SS_BITS + pcfx_cd_pkg::SS_BITS;

	typedef enum logic [3:0] {
		SG_CHAIN, SG_CPU_ST, SG_CPU_MEM,
		SG_NT, SG_VA, SG_VB, SG_BRAM, SG_PSG, SG_RRAM,
		SG_MAIN_LO, SG_MAIN_HI, SG_KRAM_A, SG_KRAM_B, SG_FXBMP
	} seg_t;

	localparam seg_t SG_LAST = SG_FXBMP;

	// Elements per segment.
	function automatic logic [19:0] seg_len(input seg_t s);
		case (s)
			SG_CHAIN:   seg_len = 20'((CHAIN_BITS + 15) / 16);
			SG_CPU_ST:  seg_len = 20'd64;          // 16 x 64-bit V810 words
			SG_CPU_MEM: seg_len = 20'd1664;        // V810 GPRs, cache data, tags
			SG_NT:      seg_len = 20'd534;         // HuC6261 palette and R04-R15
			SG_VA:      seg_len = 20'd65792;       // VRAM, then SATB
			SG_VB:      seg_len = 20'd65792;
			SG_BRAM:    seg_len = 20'd32768;       // backup SRAM bytes
			SG_PSG:     seg_len = 20'd256;         // HuC6230 waves
			SG_RRAM:    seg_len = 20'd16512;       // HuC6271 R-RAM A, B, then Q tables
			SG_MAIN_LO: seg_len = 20'd524288;      // main RAM D15:0
			SG_MAIN_HI: seg_len = 20'd524288;      // main RAM D31:16
			SG_KRAM_A:  seg_len = 20'd262144;
			SG_KRAM_B:  seg_len = 20'd262144;
			default:    seg_len = 20'd65536;       // FX-BMP
		endcase
	endfunction

	function automatic int total_elems();
		int n = 0;
		for (int s = 0; s <= int'(SG_LAST); s++)
			n += int'(seg_len(seg_t'(s)));
		return n;
	endfunction

	localparam int ELEMS     = total_elems();
	localparam int HDR_WORDS = 4;
	localparam int WORDS     = HDR_WORDS + (ELEMS + 3) / 4;
	localparam logic [31:0] SIZE_DWORDS = 32'(2 * (WORDS - 1));

	function automatic logic [31:0] layout_hash();
		logic [31:0] h = 32'h811C9DC5;
		h = (h ^ 32'(CHAIN_BITS)) * 32'h01000193;
		for (int s = 0; s <= int'(SG_LAST); s++)
			h = (h ^ 32'(seg_len(seg_t'(s)))) * 32'h01000193;
		return h;
	endfunction

	localparam logic [31:0] LAYOUT_HASH = layout_hash();
	localparam logic [31:0] MAGIC       = "PCFX";

endpackage
