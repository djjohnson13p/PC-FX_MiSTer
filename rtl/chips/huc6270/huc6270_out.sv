// HuC6270 video output: sprite/background priority and the eight-dot
// output pipe to SPBG and VD7-0.
//
// The pixel is chosen as it enters the pipe; BB and SB only change before
// the display starts, so this matches choosing it at the pins. Blanking
// and burst mode give SPBG=1, VD=0.

module huc6270_out
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_dck,

	input  logic        hs_f,
	input  logic        bg_out,
	input  logic        vdisp,
	input  logic        burst,
	input  logic        bb,
	input  logic        sb,
	input  logic  [7:0] bg_pix,
	input  logic  [8:0] spr_pix,

	output logic  [8:0] vd
);

localparam logic [8:0] BLANK = 9'h100;

wire spr_on = spr_pix[3:0] != 4'd0 && sb;
wire bg_on  = bg_pix[3:0] != 4'd0 && bb;

logic [8:0] pix;
always_comb begin
	if (hs_f || !bg_out || !vdisp || burst)
		pix = BLANK;
	else if (spr_on && spr_pix[8])
		pix = {1'b1, spr_pix[7:0]};
	else if (bg_on)
		pix = {1'b0, bg_pix};
	else if (spr_on)
		pix = {1'b1, spr_pix[7:0]};
	else
		pix = 9'h000;
end

logic [8:0] pipe [8];

always_ff @(posedge clk) begin
	if (reset) begin
		for (int i = 0; i < 8; i++)
			pipe[i] <= BLANK;
	end else if (ce_dck) begin
		pipe[7] <= pix;
		for (int i = 0; i < 7; i++)
			pipe[i] <= pipe[i + 1];
	end
end

assign vd = pipe[0];

endmodule
