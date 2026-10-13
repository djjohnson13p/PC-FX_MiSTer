// Address decode of the current CPU cycle.
//
//   memory  0000_0000-00FF_FFFF  main RAM (2 MB populated)
//           8000_0000-80FF_FFFF  I/O space alias
//           A000_0000-AFFF_FFFF  bit-string read windows: 6261, 6270 #0, #1, 6272
//           B000_0000-BFFF_FFFF  bit-string write windows, same order
//           E000_0000-E7FF_FFFF  backup SRAM, while C80 bit 0
//           E800_0000-EFFF_FFFF  FX-BMP, while C80 bit 1
//           F000_0000-FFFF_FFFF  ROM, every 1 MB
//   I/O     000 K-port, 100-600 HuC chips, 700 ERST, C00 IOC, E00 ITC,
//           F00 timer (A11:8 only); 0040_0000-007F_FFFF expansion bus
//
// wait is the clock count minus 2 for kinds timed by the bus counter.

module ga_decode
	import upd65641_pkg::*;
(
	input  logic [31:24] a_top,
	input  logic         a22,
	input  logic [11:8]  a_blk,
	input  logic  [5:2]  a_reg,
	input  logic         hi,
	input  logic         mrq_n,
	input  logic         rw,
	input  logic  [1:0]  bkup,

	output kind_t        kind,
	output logic  [2:0]  chip,
	output logic  [5:1]  ioa,
	output logic  [3:0]  wait_cnt
);

wire io = mrq_n || a_top[31:24] == 8'h80;

always_comb begin
	kind     = K_NONE;
	chip     = 3'(a_blk - 4'd1);
	ioa      = {a_reg, hi};
	wait_cnt = 4'd2;

	if (io) begin
		wait_cnt = 4'd4;
		if (a22) begin
			kind     = K_EXP;
			wait_cnt = 4'd2;
		end else begin
			case (a_blk)
				4'h0, 4'h7, 4'hC, 4'hE, 4'hF: kind = K_GLUE;
				4'h1, 4'h2, 4'h3, 4'h4, 4'h5, 4'h6: kind = rw ? K_IOR : K_IOW;
				default: kind = K_NONE;
			endcase
		end
	end else begin
		case (a_top[31:28])
			4'h0: if (a_top[27:24] == 4'h0) kind = K_RAM;
			4'hA, 4'hB: begin
				// Each window reaches one data port: the chip's A2 is high.
				chip = {1'b0, a_top[27:26]} + CHIP_NT;
				ioa  = {3'b000, 1'b1, hi};
				if (a_top[28] != rw)
					kind = rw ? K_IOR : K_IOW;
			end
			4'hE: begin
				if (!a_top[27] && bkup[0])
					kind = K_BRAM;
				if (a_top[27] && bkup[1]) begin
					kind     = K_BMP;
					wait_cnt = rw ? 4'd7 : 4'd4;
				end
			end
			4'hF: kind = K_ROM;
			default: kind = K_NONE;
		endcase
	end
end

endmodule
