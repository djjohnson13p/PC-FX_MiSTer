// Shared types and saved state of the uPD65641 glue gate array.

package upd65641_pkg;

	// Kind of the CPU cycle, decoded from the T1 address.
	typedef enum logic [3:0] {
		K_NONE,     // unmapped or disabled: completes, nobody drives the data bus
		K_RAM,
		K_ROM,
		K_BRAM,     // internal backup SRAM
		K_BMP,      // FX-BMP edge connector
		K_EXP,      // expansion bus
		K_IOR,      // HuC chip read
		K_IOW,      // HuC chip write, through the write buffer
		K_GLUE      // the gate array's own registers
	} kind_t;

	// HuC chip numbers on the I/O bus, in I/O address order: 0 HuC6230 (100),
	// 1 HuC6271, 2 HuC6261, 3 HuC6270 #0, 4 HuC6270 #1, 5 HuC6272 (600).
	localparam logic [2:0] CHIP_NT = 3'd2;
	localparam logic [2:0] CHIP_7A = 3'd3;
	localparam logic [2:0] CHIP_7B = 3'd4;

	typedef struct packed {
		logic [12:0] page;      // A23:11 of the open page
		logic        page_ok;
		logic        page_pop;  // page lies in the populated 2 MB
		logic        ras_n;
		logic  [3:0] cas;       // CAS lanes held by the sequencer
		logic  [1:0] op;
		logic  [3:0] step;
		logic        pend;      // RAM cycle waiting for the refresh to end
		logic  [8:0] ref_cnt;
		logic        ref_req;
	} mmc_state_t;

	// I/O bus engine: the write buffer and its bus cycle.
	typedef struct packed {
		logic        busy;
		logic  [2:0] step;
		logic        write;
		logic  [2:0] chip;
		logic  [5:1] ioa;
		logic [15:0] data;
	} iob_state_t;

	typedef struct packed {
		logic        aer;
		logic  [6:0] imr;
		logic [11:0] ilr0;
		logic  [8:0] ilr1;
		logic        intr;
		logic  [2:0] lvl;
	} itc_state_t;

	typedef struct packed {
		logic        tien;
		logic        tce;
		logic        tireq;
		logic [15:0] tmr;
		logic [15:0] dwc;
	} tmc_state_t;

	typedef struct packed {
		logic        ios;
		logic        mod;
		logic        trg;
		logic        fin;       // END
		logic [31:0] sr;
		logic        dout;
		logic  [7:0] pos;
	} kpc_state_t;

	typedef struct packed {
		logic        erst;
		logic  [3:0] ar7a;
		logic  [3:0] ar7b;
		logic  [1:0] bkup;
	} ioc_state_t;

	localparam int MMC_BITS = $bits(mmc_state_t);
	localparam int IOB_BITS = $bits(iob_state_t);
	localparam int ITC_BITS = $bits(itc_state_t);
	localparam int TMC_BITS = $bits(tmc_state_t);
	localparam int KPC_BITS = $bits(kpc_state_t);
	localparam int IOC_BITS = $bits(ioc_state_t);
	// Summed by the board.
	/* verilator lint_off UNUSEDPARAM */
	localparam int SS_BITS  = MMC_BITS + IOB_BITS + ITC_BITS + TMC_BITS +
		2 * KPC_BITS + IOC_BITS;
	/* verilator lint_on UNUSEDPARAM */

endpackage
