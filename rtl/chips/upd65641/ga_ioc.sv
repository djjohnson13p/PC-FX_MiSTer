// Board registers.
//
//   700 R/W  D0 ERST: expansion and FX-BMP bus reset while 1
//   C00 R    last AR written to HuC6270 #0 (A2 = 0), low 4 bits: 13h reads 03h
//   C40 R    same for HuC6270 #1
//   C80 R/W  D1 CARD (FX-BMP), D0 BRAM (internal backup) access enables
//   CC0 R    gate array version, value unknown: reads 0

module ga_ioc
	import upd65641_pkg::*;
#(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_sys,

	input  logic        we_erst,
	input  logic        we_ioc,
	input  logic        sel_erst,
	input  logic  [1:0] sel,
	input  logic  [1:0] wd,
	output logic [15:0] rd,

	input  logic        io_wr_done,
	input  logic  [2:0] io_chip,
	input  logic        io_a2,
	input  logic  [3:0] io_d,

	output logic        erst,
	output logic  [1:0] bkup,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

ioc_state_t st;

wire ar_write = io_wr_done && !io_a2;

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= ioc_state_t'({st[IOC_BITS-2:0], ss_si});
	end else if (ce_sys) begin
		if (we_erst)
			st.erst <= wd[0];
		if (we_ioc && sel == 2'd2)
			st.bkup <= wd;
		if (ar_write && io_chip == CHIP_7A)
			st.ar7a <= io_d;
		if (ar_write && io_chip == CHIP_7B)
			st.ar7b <= io_d;
	end
end

assign ss_so = SAVESTATE ? st[IOC_BITS-1] : ss_si;

always_comb begin
	if (sel_erst)
		rd = {15'd0, st.erst};
	else case (sel)
		2'd0:    rd = {12'd0, st.ar7a};
		2'd1:    rd = {12'd0, st.ar7b};
		2'd2:    rd = {14'd0, st.bkup};
		default: rd = 16'd0;
	endcase
end

assign erst = st.erst;
assign bkup = st.bkup;

endmodule
