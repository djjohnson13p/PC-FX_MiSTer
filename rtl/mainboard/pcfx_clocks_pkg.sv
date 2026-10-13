// Saved state of the board clock dividers.

package pcfx_clocks_pkg;

	typedef struct packed {
		logic       odd;
		logic [4:0] timer;
		logic [7:0] kport;
	} state_t;

	localparam int SS_BITS = $bits(state_t);

endpackage
