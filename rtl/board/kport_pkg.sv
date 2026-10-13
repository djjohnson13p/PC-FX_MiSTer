// Saved state of the K-port devices.

package kport_pkg;

	typedef struct packed {
		logic [13:0] snap;      // MODE2, MODE1, Left Down Right Up Run Sel VI-I
		logic  [4:0] bit_n;     // frame bit on DATA
		logic        clk_q;
	} pad_state_t;

	typedef struct packed {
		logic  [9:0] dx;        // motion not sent yet, signed
		logic  [9:0] dy;
		logic [17:0] snap;      // R, L, X, Y
		logic  [4:0] bit_n;
		logic        latch_q;
		logic        clk_q;
	} mouse_state_t;

	typedef struct packed {
		logic  [2:0] port_n;    // 0-3 pad slots, 4 the tap itself
		logic        clr;       // CLK fell during LATCH
		logic  [4:0] bit_n;
		logic        latch_q;
		logic        clk_q;
	} tap_state_t;

	localparam int PAD_BITS   = $bits(pad_state_t);
	localparam int MOUSE_BITS = $bits(mouse_state_t);
	localparam int TAP_BITS   = $bits(tap_state_t);

endpackage
