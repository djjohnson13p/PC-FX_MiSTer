// HuC6270 background: BAT and CG fetch addresses, the pattern shift
// registers, and the background pixel for each display dot.
//
// Each character's BAT word and CG words arrive in its slots; at CG1 (or
// the single CG slot in 4-clock mode) the planes shift in behind the
// previous character, so the pixel select can reach eight dots back for the
// fine X scroll.
//
// Nothing here is saved: the pipe refills two characters before the first
// display dot of every line, BXR reloads each line, and the Y offset
// reloads from BYR on the first display line.

module huc6270_bg
	import huc6270_pkg::*;
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_dck,

	input  slot_t       slot,
	input  logic [15:0] md_i,
	input  logic  [1:0] vm,
	input  logic  [2:0] screen,

	input  logic        hs_f,
	input  logic        at_hdsm2_d7,
	input  logic        at_hdsm3_d7,
	input  logic        v_first,
	input  logic        bg_out,
	input  logic        vdisp,

	input  logic  [9:0] bxr,
	input  logic  [8:0] byr,
	input  logic        byr_set,

	output logic [15:0] bg_addr,
	output logic  [7:0] bg_pix
);

logic  [9:0] bg_x;
logic  [9:0] ofs_x;
logic  [8:0] ofs_y;
logic        bxr_set;
logic [11:0] bat_cc;
logic  [3:0] bat_col;
logic  [7:0] ch0, ch1;
logic [15:0] sr0, sr1, sr2, sr3;
logic  [3:0] src0, src1;
logic  [2:0] out_x;

/* verilator lint_off UNUSEDSIGNAL */
wire [9:0] x = bg_x + ofs_x;
/* verilator lint_on UNUSEDSIGNAL */
wire [8:0] y = screen[2] ? ofs_y : {1'b0, ofs_y[7:0]};

always_comb begin
	case (slot)
		SL_BAT: case (screen[1:0])
			2'd0:    bg_addr = {5'd0, y[8:3], x[7:3]};
			2'd1:    bg_addr = {4'd0, y[8:3], x[8:3]};
			default: bg_addr = {3'd0, y[8:3], x[9:3]};
		endcase
		SL_CG0:  bg_addr = {bat_cc, 1'b0, y[2:0]};
		SL_CG1:  bg_addr = {bat_cc, 1'b1, y[2:0]};
		default: bg_addr = 16'd0;
	endcase
end

wire shift  = slot == SL_CG1 || (slot == SL_CG0 && vm == 2'd3);
wire four_a = slot == SL_CG0 && vm == 2'd3;
wire four_b = slot == SL_CG1 && vm == 2'd3;

always_ff @(posedge clk) begin
	if (reset) begin
		bg_x    <= 10'd0;
		ofs_x   <= 10'd0;
		ofs_y   <= 9'd0;
		bxr_set <= 1'b0;
		bat_cc  <= 12'd0;
		bat_col <= 4'd0;
		ch0     <= 8'd0;
		ch1     <= 8'd0;
		sr0     <= 16'd0;
		sr1     <= 16'd0;
		sr2     <= 16'd0;
		sr3     <= 16'd0;
		src0    <= 4'd0;
		src1    <= 4'd0;
		out_x   <= 3'd0;
	end else if (ce_dck) begin
		case (slot)
			SL_BAT: begin
				bat_cc  <= md_i[11:0];
				bat_col <= md_i[15:12];
			end
			SL_CG0: begin
				ch0 <= md_i[7:0];
				ch1 <= md_i[15:8];
			end
			default: ;
		endcase

		if (shift) begin
			if (four_a) begin
				sr0 <= {sr0[7:0], md_i[7:0]};
				sr1 <= {sr1[7:0], md_i[15:8]};
				sr2 <= 16'd0;
				sr3 <= 16'd0;
			end else if (four_b) begin
				sr0 <= 16'd0;
				sr1 <= 16'd0;
				sr2 <= {sr2[7:0], md_i[7:0]};
				sr3 <= {sr3[7:0], md_i[15:8]};
			end else begin
				sr0 <= {sr0[7:0], ch0};
				sr1 <= {sr1[7:0], ch1};
				sr2 <= {sr2[7:0], md_i[7:0]};
				sr3 <= {sr3[7:0], md_i[15:8]};
			end
			src1 <= src0;
			src0 <= bat_col;
			bg_x <= bg_x + 10'd8;
		end

		if (at_hdsm2_d7)
			bg_x <= 10'd0;

		// BYR applies to the first line; a write during display takes effect
		// on the next line as BYR + 1.
		if (at_hdsm3_d7)
			ofs_y <= v_first ? byr : (byr_set ? byr : ofs_y) + 9'd1;

		bxr_set <= at_hdsm3_d7;
		if (bxr_set)
			ofs_x <= bxr;

		if (hs_f)
			out_x <= 3'd0;
		else if (bg_out && vdisp)
			out_x <= out_x + 3'd1;
	end
end

// Fine X scroll picks one of sixteen dots spanning two characters.
wire [3:0] px = ~({1'b0, out_x} + {1'b0, ofs_x[2:0]});

assign bg_pix = {px[3] ? src1 : src0, sr3[px], sr2[px], sr1[px], sr0[px]};

endmodule
