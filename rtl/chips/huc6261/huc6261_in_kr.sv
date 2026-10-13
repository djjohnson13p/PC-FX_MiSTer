// HuC6261 receiver for the KING or Rainbow pixel bus. Words are held one
// dot so a 4:2:2 pair has both chroma bytes: the U dot takes V from the
// next word, the V dot takes U from the previous one. Output is the dot
// two DCKKR behind the bus, with its key decided.
//
// Keys: palette data 0, YUV with Y = 0 (KING), or a Rainbow YUV dot whose
// Y, U and V all sit inside the CKY/CKU/CKV ranges (off in 7 MHz mode).

module huc6261_in_kr #(
	parameter bit RAINBOW = 1'b0
)
(
	input  logic        clk,
	input  logic        ce_dckkr,
	input  pcfx_pixel_pkg::pcfx_pixel_t bus,

	input  logic        dc7,
	input  logic [15:0] cky,
	input  logic [15:0] cku,
	input  logic [15:0] ckv,

	output logic        key,
	output logic        is_pal,
	output logic  [1:0] plane,
	output logic  [7:0] idx,
	output logic [23:0] yuv
);

import pcfx_pixel_pkg::*;

pcfx_pixel_t cap, dly;
logic [7:0] prev_c;

always_ff @(posedge clk) begin
	if (ce_dckkr) begin
		cap    <= bus;
		dly    <= cap;
		prev_c <= dly.data[7:0];
	end
end

wire [7:0] y = dly.data[15:8];
logic [7:0] u, v;

always_comb begin
	case (dly.kind)
		PIX_YUV16: begin u = {dly.data[7:4], 4'h0}; v = {dly.data[3:0], 4'h0}; end
		PIX_YUV_U: begin u = dly.data[7:0];         v = cap.data[7:0];          end
		PIX_YUV_V: begin u = prev_c;                v = dly.data[7:0];          end
		default:   begin u = 8'h80;                 v = 8'h80;                  end
	endcase
end

wire in_range = y >= cky[7:0] && y <= cky[15:8]
             && u >= cku[7:0] && u <= cku[15:8]
             && v >= ckv[7:0] && v <= ckv[15:8];

wire yuv_key = RAINBOW ? in_range && !dc7 : y == 8'd0;

assign is_pal = dly.kind == PIX_PAL;
assign idx    = dly.data[7:0];
assign plane  = dly.plane;
assign yuv    = {y, u, v};
assign key    = !dly.valid || dly.transparent || (is_pal ? idx == 8'd0 : yuv_key);

endmodule
