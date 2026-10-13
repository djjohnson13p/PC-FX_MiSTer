// HuC6230 SoundBox: saved state of each block, and the shared tables.

package huc6230_pkg;

	// Registers R10-R16 and the -HSYNC edge detector.
	typedef struct packed {
		logic       hs_q;
		logic [5:0] r10_pend;          // last R10 write, taken at the -HSYNC fall
		logic [5:0] r10;
		logic [3:0][5:0] avol;         // R11-R14: ADPCM#1 L, R, #2 L, R
		logic [1:0][5:0] lvol;         // R15, R16: LINE L, R
	} cpu_state_t;

	typedef struct packed {
		logic [11:0] freq;
		logic        chon;
		logic        dda;
		logic  [4:0] al;
		logic  [3:0] lal;
		logic  [3:0] ral;
		logic  [4:0] wf_addr;
		logic [12:0] wf_cnt;
		logic  [4:0] da_out;
	} psg_ch_t;

	typedef struct packed {
		logic        ne;
		logic  [4:0] ng_freq;
		logic [17:0] lfsr;
		logic [11:0] ng_cnt;
	} psg_noise_t;

	typedef struct packed {
		logic  [1:0] fm_div;           // master / 3 = fmaster
		logic  [2:0] chsel;
		logic  [3:0] lmal;
		logic  [3:0] rmal;
		logic  [7:0] lfo_freq;
		logic  [1:0] lfctl;
		logic        lftrg;
		logic  [7:0] lfo_cnt;
		psg_noise_t [1:0] ng;          // channels 5 and 6
		psg_ch_t    [5:0] ch;
	} psg_state_t;

	// K-R bus receiver: one strobe in progress, one byte pair per channel.
	typedef struct packed {
		logic        wr_q;
		logic  [1:0] h_ch;             // {#2, #1} selected by this strobe
		logic        h_rh;
		logic  [7:0] h_d;
		logic        seen;             // a byte arrived since the cycle was unlocked
		logic  [1:0] got_lo;
		logic  [1:0] got_hi;
		logic [1:0][15:0] pair;        // {high byte, low byte}
	} kr_state_t;

	// 31.47 kHz ticks: two per line, and the transfer cycle position.
	typedef struct packed {
		logic  [9:0] lpos;             // master clocks since the -HSYNC fall
		logic  [3:0] lcount;           // lines since the cycle boundary
		logic        locked;
	} tick_state_t;

	typedef struct packed {
		logic [14:0] p;                // PCM value, 12.3 offset binary
		logic  [5:0] idx;              // scale level 0-48
		logic [15:0] part;             // signed delta added on each tick
		logic [15:0] word;             // four nibbles of this cycle
		logic        valid;            // the cycle has data (else hold)
		logic        rst_q;            // soft reset seen last clock
		logic [23:0] fade;             // reset-click offset, signed 16.8
	} adpcm_ch_t;

	typedef struct packed {
		adpcm_ch_t [1:0] ch;
	} dec_state_t;

	// Smoothed ADPCM volumes, 16.8.
	typedef struct packed {
		logic  [1:0] sel;
		logic [3:0][23:0] g;
	} evol_state_t;

	typedef struct packed {
		logic [14:0] pre;              // master clock prescaler shared by both sides
		logic  [1:0] up;
		logic  [1:0] down;
		logic [1:0][9:0] level;
		logic [1:0][15:0] line;        // CD-DA sample held from its 44.1 kHz enable
	} vca_state_t;

	// Mixer pass phase in fmaster clocks; the pass in flight is not saved.
	typedef struct packed {
		logic [1:0] fmc;
	} mix_state_t;

	localparam int SS_CPU_BITS  = $bits(cpu_state_t);
	localparam int SS_PSG_BITS  = $bits(psg_state_t);
	localparam int SS_KR_BITS   = $bits(kr_state_t);
	localparam int SS_TICK_BITS = $bits(tick_state_t);
	localparam int SS_DEC_BITS  = $bits(dec_state_t);
	localparam int SS_EVOL_BITS = $bits(evol_state_t);
	localparam int SS_VCA_BITS  = $bits(vca_state_t);
	localparam int SS_MIX_BITS  = $bits(mix_state_t);
	/* verilator lint_off UNUSEDPARAM */
	localparam int SS_BITS = SS_CPU_BITS + SS_PSG_BITS + SS_KR_BITS + SS_TICK_BITS
		+ SS_DEC_BITS + SS_EVOL_BITS + SS_VCA_BITS + SS_MIX_BITS;
	/* verilator lint_on UNUSEDPARAM */

	// Linear gain for n steps of -1.5 dB, 65535 = 0 dB.
	function automatic logic [15:0] gain_db15(input logic [5:0] n);
		case (n)
			6'd0:  gain_db15 = 16'd65535;
			6'd1:  gain_db15 = 16'd55141;
			6'd2:  gain_db15 = 16'd46395;
			6'd3:  gain_db15 = 16'd39037;
			6'd4:  gain_db15 = 16'd32845;
			6'd5:  gain_db15 = 16'd27636;
			6'd6:  gain_db15 = 16'd23253;
			6'd7:  gain_db15 = 16'd19565;
			6'd8:  gain_db15 = 16'd16462;
			6'd9:  gain_db15 = 16'd13851;
			6'd10: gain_db15 = 16'd11654;
			6'd11: gain_db15 = 16'd9806;
			6'd12: gain_db15 = 16'd8250;
			6'd13: gain_db15 = 16'd6942;
			6'd14: gain_db15 = 16'd5841;
			6'd15: gain_db15 = 16'd4914;
			6'd16: gain_db15 = 16'd4135;
			6'd17: gain_db15 = 16'd3479;
			6'd18: gain_db15 = 16'd2927;
			6'd19: gain_db15 = 16'd2463;
			6'd20: gain_db15 = 16'd2072;
			6'd21: gain_db15 = 16'd1744;
			6'd22: gain_db15 = 16'd1467;
			6'd23: gain_db15 = 16'd1234;
			6'd24: gain_db15 = 16'd1039;
			6'd25: gain_db15 = 16'd874;
			6'd26: gain_db15 = 16'd735;
			6'd27: gain_db15 = 16'd619;
			6'd28: gain_db15 = 16'd521;
			6'd29: gain_db15 = 16'd438;
			6'd30: gain_db15 = 16'd369;
			6'd31: gain_db15 = 16'd310;
			6'd32: gain_db15 = 16'd261;
			6'd33: gain_db15 = 16'd220;
			6'd34: gain_db15 = 16'd185;
			6'd35: gain_db15 = 16'd155;
			default: gain_db15 = 16'd0;
		endcase
	endfunction

	// ADPCM scale value for each scale level.
	function automatic logic [10:0] adpcm_scale(input logic [5:0] idx);
		case (idx)
			6'd0:  adpcm_scale = 11'd16;
			6'd1:  adpcm_scale = 11'd17;
			6'd2:  adpcm_scale = 11'd19;
			6'd3:  adpcm_scale = 11'd21;
			6'd4:  adpcm_scale = 11'd23;
			6'd5:  adpcm_scale = 11'd25;
			6'd6:  adpcm_scale = 11'd28;
			6'd7:  adpcm_scale = 11'd31;
			6'd8:  adpcm_scale = 11'd34;
			6'd9:  adpcm_scale = 11'd37;
			6'd10: adpcm_scale = 11'd41;
			6'd11: adpcm_scale = 11'd45;
			6'd12: adpcm_scale = 11'd50;
			6'd13: adpcm_scale = 11'd55;
			6'd14: adpcm_scale = 11'd60;
			6'd15: adpcm_scale = 11'd66;
			6'd16: adpcm_scale = 11'd73;
			6'd17: adpcm_scale = 11'd80;
			6'd18: adpcm_scale = 11'd88;
			6'd19: adpcm_scale = 11'd97;
			6'd20: adpcm_scale = 11'd107;
			6'd21: adpcm_scale = 11'd118;
			6'd22: adpcm_scale = 11'd130;
			6'd23: adpcm_scale = 11'd143;
			6'd24: adpcm_scale = 11'd157;
			6'd25: adpcm_scale = 11'd173;
			6'd26: adpcm_scale = 11'd190;
			6'd27: adpcm_scale = 11'd209;
			6'd28: adpcm_scale = 11'd230;
			6'd29: adpcm_scale = 11'd253;
			6'd30: adpcm_scale = 11'd279;
			6'd31: adpcm_scale = 11'd307;
			6'd32: adpcm_scale = 11'd337;
			6'd33: adpcm_scale = 11'd371;
			6'd34: adpcm_scale = 11'd408;
			6'd35: adpcm_scale = 11'd449;
			6'd36: adpcm_scale = 11'd494;
			6'd37: adpcm_scale = 11'd544;
			6'd38: adpcm_scale = 11'd598;
			6'd39: adpcm_scale = 11'd658;
			6'd40: adpcm_scale = 11'd724;
			6'd41: adpcm_scale = 11'd796;
			6'd42: adpcm_scale = 11'd876;
			6'd43: adpcm_scale = 11'd963;
			6'd44: adpcm_scale = 11'd1060;
			6'd45: adpcm_scale = 11'd1166;
			6'd46: adpcm_scale = 11'd1282;
			6'd47: adpcm_scale = 11'd1411;
			default: adpcm_scale = 11'd1552;
		endcase
	endfunction

endpackage
