// HuC6271 Rainbow: shared constants, the inverse DCT arithmetic and the
// saved state.

package huc6271_pkg;

	// Parser states
	typedef enum logic [2:0] {
		P_HUNT, P_TYPE, P_LENH, P_LENL, P_DATA, P_DONE, P_SKIP
	} pstate_t;

	// Data FIFO between the parser and the decoder
	localparam int FIFO_AW   = 6;
	localparam int FIFO_STOP = 56;  // -REQR goes high at this many bytes

	// Inverse DCT. Rows then columns, each an 8-point even/odd matrix
	// product with 12-bit coefficients C(k) cos((2x+1)k pi/16), C(0) = 1/sqrt 2,
	// so a DC-only block gives DC/2 (four times the JPEG gain). Between the
	// passes two fraction bits are kept. Both scalings round half up. No
	// document gives the real chip's precision; this is the one place it is
	// chosen.
	localparam int IDCT_SH1 = 10;   // pass 1: 12 coefficient bits less 2 kept
	localparam int IDCT_SH2 = 14;   // pass 2: 12 + 2

	// Coefficient for frequency k (0-7) at output x (0-3); x 4-7 follow by
	// symmetry (even k same, odd k negated).
	function automatic logic signed [12:0] idct_k(input logic [2:0] k, input logic [1:0] x);
		case ({k, x})
			5'd0: idct_k = 13'sd2896; 5'd1: idct_k = 13'sd2896; 5'd2: idct_k = 13'sd2896; 5'd3: idct_k = 13'sd2896;
			5'd4: idct_k = 13'sd4017; 5'd5: idct_k = 13'sd3406; 5'd6: idct_k = 13'sd2276; 5'd7: idct_k = 13'sd799;
			5'd8: idct_k = 13'sd3784; 5'd9: idct_k = 13'sd1567; 5'd10: idct_k = -13'sd1567; 5'd11: idct_k = -13'sd3784;
			5'd12: idct_k = 13'sd3406; 5'd13: idct_k = -13'sd799; 5'd14: idct_k = -13'sd4017; 5'd15: idct_k = -13'sd2276;
			5'd16: idct_k = 13'sd2896; 5'd17: idct_k = -13'sd2896; 5'd18: idct_k = -13'sd2896; 5'd19: idct_k = 13'sd2896;
			5'd20: idct_k = 13'sd2276; 5'd21: idct_k = -13'sd4017; 5'd22: idct_k = 13'sd799; 5'd23: idct_k = 13'sd3406;
			5'd24: idct_k = 13'sd1567; 5'd25: idct_k = -13'sd3784; 5'd26: idct_k = 13'sd3784; 5'd27: idct_k = -13'sd1567;
			5'd28: idct_k = 13'sd799; 5'd29: idct_k = -13'sd2276; 5'd30: idct_k = 13'sd3406; 5'd31: idct_k = -13'sd4017;
			default: idct_k = 13'sd0;
		endcase
	endfunction

	// Zigzag position (1-63) to row * 8 + column
	function automatic logic [5:0] zigzag(input logic [5:0] p);
		case (p)
			6'd0: zigzag = 6'h00;  6'd1: zigzag = 6'h01;  6'd2: zigzag = 6'h08;  6'd3: zigzag = 6'h10;
			6'd4: zigzag = 6'h09;  6'd5: zigzag = 6'h02;  6'd6: zigzag = 6'h03;  6'd7: zigzag = 6'h0A;
			6'd8: zigzag = 6'h11;  6'd9: zigzag = 6'h18;  6'd10: zigzag = 6'h20; 6'd11: zigzag = 6'h19;
			6'd12: zigzag = 6'h12; 6'd13: zigzag = 6'h0B; 6'd14: zigzag = 6'h04; 6'd15: zigzag = 6'h05;
			6'd16: zigzag = 6'h0C; 6'd17: zigzag = 6'h13; 6'd18: zigzag = 6'h1A; 6'd19: zigzag = 6'h21;
			6'd20: zigzag = 6'h28; 6'd21: zigzag = 6'h30; 6'd22: zigzag = 6'h29; 6'd23: zigzag = 6'h22;
			6'd24: zigzag = 6'h1B; 6'd25: zigzag = 6'h14; 6'd26: zigzag = 6'h0D; 6'd27: zigzag = 6'h06;
			6'd28: zigzag = 6'h07; 6'd29: zigzag = 6'h0E; 6'd30: zigzag = 6'h15; 6'd31: zigzag = 6'h1C;
			6'd32: zigzag = 6'h23; 6'd33: zigzag = 6'h2A; 6'd34: zigzag = 6'h31; 6'd35: zigzag = 6'h38;
			6'd36: zigzag = 6'h39; 6'd37: zigzag = 6'h32; 6'd38: zigzag = 6'h2B; 6'd39: zigzag = 6'h24;
			6'd40: zigzag = 6'h1D; 6'd41: zigzag = 6'h16; 6'd42: zigzag = 6'h0F; 6'd43: zigzag = 6'h17;
			6'd44: zigzag = 6'h1E; 6'd45: zigzag = 6'h25; 6'd46: zigzag = 6'h2C; 6'd47: zigzag = 6'h33;
			6'd48: zigzag = 6'h3A; 6'd49: zigzag = 6'h3B; 6'd50: zigzag = 6'h34; 6'd51: zigzag = 6'h2D;
			6'd52: zigzag = 6'h26; 6'd53: zigzag = 6'h1F; 6'd54: zigzag = 6'h27; 6'd55: zigzag = 6'h2E;
			6'd56: zigzag = 6'h35; 6'd57: zigzag = 6'h3C; 6'd58: zigzag = 6'h3D; 6'd59: zigzag = 6'h36;
			6'd60: zigzag = 6'h2F; 6'd61: zigzag = 6'h37; 6'd62: zigzag = 6'h3E; 6'd63: zigzag = 6'h3F;
			default: zigzag = 6'h00;
		endcase
	endfunction

	// Block-noise filters: equal weights, half rounds up (the patent only
	// says "weighted average").
	function automatic logic [7:0] fts_avg(input logic [7:0] a, input logic [7:0] b);
		fts_avg = 8'(($signed({a[7], a}) + $signed({b[7], b}) + 9'sd1) >>> 1);
	endfunction

	// Saved state, in shift order.
	typedef struct packed {
		logic [8:0] scx;      // R00 as written
		logic [2:0] cr;       // R01 FTS CHR SRT
		logic [7:0] nry;
		logic [7:0] nru;
		logic [7:0] nrv;
		logic [7:0] hsr;
		logic [8:0] scx_op;   // taken at the -HSYNC fall
		logic       chr_op;
		logic       run;      // out of reset
	} regs_t;

	typedef struct packed {
		logic       hs_q;
		logic [8:0] kx;
		logic       jact;     // counting -HSYNC falls since a header
		logic [3:0] jc;
		logic       disp;     // R-RAM on display: 0 A, 1 B
		logic       dvalid;
		logic [3:0] dl;       // line of the strip on display
		logic       d_dct;
		logic       d_fts;
		logic       w_done;   // strip in the other R-RAM is complete
		logic       w_ready;  // w_done as this line began
		logic       inv;      // INVALID: lines after an incomplete strip
		logic       w_dct;
		logic       w_fts;
		logic [3:0] qcx;
	} strip_t;

	localparam int REGS_BITS  = $bits(regs_t);
	localparam int STRIP_BITS = $bits(strip_t);
	/* verilator lint_off UNUSEDPARAM */
	localparam int SS_BITS    = REGS_BITS + STRIP_BITS + $bits(pstate_t);
	/* verilator lint_on UNUSEDPARAM */

endpackage
