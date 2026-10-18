// MiSTer video path: the HuC6261's digital Y/U/V through yuv2rgb, gamma,
// scandoubler / HQ2x / scanlines, vertical crop and integer scaling.
//
// The 6261 picks its pixel rate per field: 5.37, 10.74 or 21.48 MHz, with
// lines of 256, 511 or 1024 pixels. CLK_VIDEO runs at twice clk_sys so the
// scandoubler and HQ2x get four clocks per pixel even at 21.48 MHz.
//
// While the board is paused the 6261 makes no pixel strobe. The last strobe
// spacing is repeated so the frozen picture keeps its sync, and HDMI holds
// the last frame. Interlaced fields bypass the scandoubler.

module pcfx_video
(
	input  logic        clk_sys,
	input  logic        CLK_VIDEO,      // 2 x clk_sys, edges aligned
	input  logic        pause,          // board stopped, not in reset

	input  logic  [7:0] y,
	input  logic  [7:0] u,
	input  logic  [7:0] v,
	input  logic        hs,
	input  logic        vs,
	input  logic        hblank,
	input  logic        vblank,
	input  logic        ce_pix,
	input  logic        field,
	input  logic        interlace,

	input  logic        jfif,           // YUV matrix
	input  logic  [1:0] saturation,     // 100 %, 75 %, 50 %, 25 %
	input  logic  [2:0] fx,             // none, HQ2x, CRT 25 %, 50 %, 75 %
	input  logic        forced_scandoubler,
	input  logic  [1:0] overscan,       // 4, 8, 0 lines per edge
	input  logic        vcrop,          // 216 lines on 1080p
	input  logic  [3:0] vcrop_off,      // 0, 2 ... 12, -12 ... -2 lines
	input  logic  [1:0] scale,          // normal, V, narrower HV, wider HV integer
	input  logic  [1:0] ar,             // 128:105, full screen, ARC1, ARC2
	inout  wire  [21:0] gamma_bus,
	input  logic [11:0] HDMI_WIDTH,
	input  logic [11:0] HDMI_HEIGHT,

	output logic        en216p,
	output logic        CE_PIXEL,
	output logic  [7:0] VGA_R,
	output logic  [7:0] VGA_G,
	output logic  [7:0] VGA_B,
	output logic        VGA_HS,
	output logic        VGA_VS,
	output logic        VGA_DE,
	output logic        VGA_F1,
	output logic  [1:0] VGA_SL,
	output logic [12:0] VIDEO_ARX,
	output logic [12:0] VIDEO_ARY,
	output logic        HDMI_FREEZE
);

// Pixel strobe, repeated at its last spacing while paused (5.37 MHz until
// one is seen)
logic [3:0] ce_cnt, ce_gap;
wire  [3:0] gap   = ce_gap == 4'd0 ? 4'd7 : ce_gap;
wire        ce_in = ce_pix || (pause && ce_cnt == gap);

always_ff @(posedge clk_sys) begin
	ce_cnt <= ce_in ? 4'd0 : ce_cnt + 4'd1;
	if (ce_pix)
		ce_gap <= ce_cnt;
end

logic       ce_rgb, hs_rgb, vs_rgb, hb_rgb, vb_rgb;
logic [7:0] r, g, b;

yuv2rgb yuv2rgb
(
	.clk      (clk_sys),
	.jfif     (jfif),
	.ce_pix   (ce_in),
	.hs       (hs),
	.vs       (vs),
	.hblank   (hblank),
	.vblank   (vblank),
	.y        (y),
	.u        (u),
	.v        (v),
	.ce_pix_o (ce_rgb),
	.hs_o     (hs_rgb),
	.vs_o     (vs_rgb),
	.hblank_o (hb_rgb),
	.vblank_o (vb_rgb),
	.r        (r),
	.g        (g),
	.b        (b)
);

wire [7:0] r_sat, g_sat, b_sat;
rgb_saturation saturation_profile
(
	.mode  (saturation),
	.r_in  (r),
	.g_in  (g),
	.b_in  (b),
	.r_out (r_sat),
	.g_out (g_sat),
	.b_out (b_sat)
);

// Register color and pixel timing before the faster gamma clock.
logic [7:0] r_sat_q, g_sat_q, b_sat_q;
logic ce_sat_q, hs_sat_q, vs_sat_q, hb_sat_q, vb_sat_q;
always_ff @(posedge clk_sys) begin
	r_sat_q <= r_sat;
	g_sat_q <= g_sat;
	b_sat_q <= b_sat;
	{ce_sat_q, hs_sat_q, vs_sat_q, hb_sat_q, vb_sat_q} <= {ce_rgb, hs_rgb, vs_rgb, hb_rgb, vb_rgb};
end

// One CLK_VIDEO pulse per pixel; the pixel holds for two clocks after it.
logic ce_old, ce_vid;
always_ff @(posedge CLK_VIDEO) begin
	ce_old <= ce_sat_q;
	ce_vid <= ce_sat_q && !ce_old;
end

wire sd   = !interlace && (fx != 3'd0 || forced_scandoubler);
wire hq2x = fx == 3'd1;
// Scanlines 25, 50, 75 %
wire [1:0] sl = fx < 3'd2 ? 2'd0 : fx[1:0] - 2'd1;

always_ff @(posedge CLK_VIDEO) begin
	VGA_SL      <= interlace ? 2'd0 : sl;
	VGA_F1      <= !field && interlace;   // O/E = 0 is the upper field
	HDMI_FREEZE <= pause;
	en216p      <= HDMI_WIDTH == 12'd1920 && HDMI_HEIGHT == 12'd1080 && !forced_scandoubler && fx == 3'd0;
end

logic de_mix, unused_freeze_sync;

video_mixer #(.LINE_LENGTH(1024), .GAMMA(1)) video_mixer
(
	.CLK_VIDEO   (CLK_VIDEO),
	.CE_PIXEL    (CE_PIXEL),
	.ce_pix      (ce_vid),
	.scandoubler (sd),
	.hq2x        (hq2x),
	.gamma_bus   (gamma_bus),
	.R           (r_sat_q),
	.G           (g_sat_q),
	.B           (b_sat_q),
	.HSync       (hs_sat_q),
	.VSync       (vs_sat_q),
	.HBlank      (hb_sat_q),
	.VBlank      (vb_sat_q),
	.HDMI_FREEZE (HDMI_FREEZE),
	.freeze_sync (unused_freeze_sync),
	.VGA_R       (VGA_R),
	.VGA_G       (VGA_G),
	.VGA_B       (VGA_B),
	.VGA_VS      (VGA_VS),
	.VGA_HS      (VGA_HS),
	.VGA_DE      (de_mix)
);

// The 240-line picture loses 4 or 8 lines per edge; the scandoubler repeats each line.
wire [11:0] overscan_size = overscan == 2'd0 ? 12'd232 : overscan == 2'd1 ? 12'd224 : 12'd0;
wire [11:0] crop_size = en216p && vcrop ? 12'd216 : sd ? {overscan_size[10:0], 1'b0} : overscan_size;

// Crop offset in lines: 0, 2 ... 12, then -12 ... -2
wire [4:0] voff = vcrop_off < 4'd7 ? {vcrop_off, 1'b0} : {vcrop_off, 1'b0} - 5'd26;

video_freak video_freak
(
	.CLK_VIDEO   (CLK_VIDEO),
	.CE_PIXEL    (CE_PIXEL),
	.VGA_VS      (VGA_VS),
	.HDMI_WIDTH  (HDMI_WIDTH),
	.HDMI_HEIGHT (HDMI_HEIGHT),
	.VGA_DE      (VGA_DE),
	.VIDEO_ARX   (VIDEO_ARX),
	.VIDEO_ARY   (VIDEO_ARY),
	.VGA_DE_IN   (de_mix),
	.ARX         (ar == 2'd0 ? 12'd128 : {10'd0, ar - 2'd1}),
	.ARY         (ar == 2'd0 ? 12'd105 : 12'd0),
	.CROP_SIZE   (crop_size),
	.CROP_OFF    (en216p && vcrop ? voff : 5'd0),
	.SCALE       ({1'b0, scale})
);

endmodule
