// HuC6261 priority: builds the three layer candidates (0 VDC, 1 KING,
// 2 Rainbow) and orders them back to front by their 3-bit priority. Equal
// priorities rank 6270 over 6272 over 6271, the order after reset.

module huc6261_prio_sort
(
	input  logic  [9:0] vdc,          // {key, SP/BG, data}
	input  logic        k_key,
	input  logic        k_is_pal,
	input  logic  [1:0] k_plane,
	input  logic  [7:0] k_idx,
	input  logic [23:0] k_yuv,
	input  logic        r_key,
	input  logic        r_is_pal,
	input  logic  [7:0] r_idx,
	input  logic [23:0] r_yuv,

	/* verilator lint_off UNUSEDSIGNAL */
	input  huc6261_pkg::line_regs_t lr,
	/* verilator lint_on UNUSEDSIGNAL */

	output huc6261_pkg::cand_t cand [3],
	output logic  [1:0] order [3]     // 0 = back
);

import huc6261_pkg::*;

wire       v_sp   = vdc[8];
wire [3:0] v_bank = vdc[7:4];

// KING plane fields
logic [2:0] k_pri;
logic [1:0] k_cel;
logic [7:0] k_off;
always_comb begin
	case (k_plane)
		2'd0: begin k_pri = lr.pri2[2:0];   k_cel = lr.ble[5:4];   k_off = lr.cpo2[7:0];  end
		2'd1: begin k_pri = lr.pri2[5:3];   k_cel = lr.ble[7:6];   k_off = lr.cpo2[15:8]; end
		2'd2: begin k_pri = lr.pri2[8:6];   k_cel = lr.ble[9:8];   k_off = lr.cpo3[7:0];  end
		default: begin k_pri = lr.pri2[11:9]; k_cel = lr.ble[11:10]; k_off = lr.cpo3[15:8]; end
	endcase
end

// Sprites in a bank with SPBL clear never mix.
wire [1:0] v_cel = v_sp ? (lr.spbl[v_bank] ? lr.ble[3:2] : 2'd0) : lr.ble[1:0];

always_comb begin
	cand[0].pres   = !vdc[9] && (v_sp ? lr.blank[1] : lr.blank[0]);
	cand[0].is_pal = 1'b1;
	cand[0].cel    = v_cel;
	cand[0].idx    = vdc[7:0];
	cand[0].off    = v_sp ? lr.cpo1[15:8] : lr.cpo1[7:0];
	cand[0].yuv    = 24'd0;

	cand[1].pres   = !k_key && lr.blank[3'd2 + {1'b0, k_plane}];
	cand[1].is_pal = k_is_pal;
	cand[1].cel    = k_cel;
	cand[1].idx    = k_idx;
	cand[1].off    = k_off;
	cand[1].yuv    = k_yuv;

	cand[2].pres   = !r_key && lr.blank[6];
	cand[2].is_pal = r_is_pal;
	cand[2].cel    = lr.ble[13:12];
	cand[2].idx    = r_idx;
	cand[2].off    = lr.cpo4;
	cand[2].yuv    = r_yuv;
end

// Sort keys: priority, then the tie order in the low bits.
wire [4:0] key_v = {v_sp ? lr.pri1[5:3] : lr.pri1[2:0], 2'd2};
wire [4:0] key_k = {k_pri, 2'd1};
wire [4:0] key_r = {lr.pri1[8:6], 2'd0};

wire vk = key_v > key_k;
wire vr = key_v > key_r;
wire kr = key_k > key_r;

wire [1:0] rank_v = {1'b0, vk} + {1'b0, vr};
wire [1:0] rank_k = {1'b0, !vk} + {1'b0, kr};

always_comb begin
	for (int r = 0; r < 3; r++) begin
		if (rank_v == 2'(r))
			order[r] = 2'd0;
		else if (rank_k == 2'(r))
			order[r] = 2'd1;
		else
			order[r] = 2'd2;
	end
end

endmodule
