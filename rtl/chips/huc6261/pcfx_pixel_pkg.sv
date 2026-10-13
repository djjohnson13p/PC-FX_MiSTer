// Pixel buses from KING (HuC6272) and Rainbow (HuC6271) into the HuC6261.
// The real buses are narrower and faster; this carries their content as
// one word per DCKKR dot. Sources load the word on ce_dckkr and hold it
// for the whole dot: window dot n is on the bus during master clocks
// 4n..4n+3 of the line.

package pcfx_pixel_pkg;

	// kind
	localparam logic [1:0] PIX_PAL   = 2'd0;  // data[7:0] palette data
	localparam logic [1:0] PIX_YUV16 = 2'd1;  // data = Y8 U4 V4
	localparam logic [1:0] PIX_YUV_U = 2'd2;  // data = Y8 U8, first dot of a 4:2:2 pair
	localparam logic [1:0] PIX_YUV_V = 2'd3;  // data = Y8 V8, second dot of the pair

	typedef struct packed {
		logic        valid;        // source is drawing this dot
		logic  [1:0] kind;
		logic  [1:0] plane;        // KING BMG number, 0 from Rainbow
		logic        transparent;  // source judged the dot transparent
		logic [15:0] data;
	} pcfx_pixel_t;

endpackage
