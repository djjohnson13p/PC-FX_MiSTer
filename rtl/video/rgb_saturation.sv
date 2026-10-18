// Copyright (c) 2026 Jamie Blanks
// Adapted 2026-10-16: 100% is the default.

module rgb_saturation
(
	input  wire [1:0] mode,
	input  wire [7:0] r_in,
	input  wire [7:0] g_in,
	input  wire [7:0] b_in,

	output wire [7:0] r_out,
	output wire [7:0] g_out,
	output wire [7:0] b_out
);

// Rounded luma: (2R + 5G + B) / 8, at most 2044 before shifting.
wire [7:0] luma = 8'(({2'b00, r_in, 1'b0} +
	{1'b0, g_in, 2'b00} +
	{3'b000, g_in} +
	{3'b000, b_in} + 11'd4) >> 3);

function automatic [7:0] profile_channel(
	input [7:0] channel,
	input [7:0] gray,
	input [1:0] profile
);
	begin
		case (profile)
			2'd1: profile_channel = 8'(({1'b0, channel, 1'b0} +
			                           {2'b00, channel} +
			                           {2'b00, gray} + 10'd2) >> 2);
			2'd2: profile_channel = 8'(({2'b00, channel} +
			                           {2'b00, gray} + 10'd1) >> 1);
			2'd3: profile_channel = 8'(({2'b00, channel} +
			                           {1'b0, gray, 1'b0} +
			                           {2'b00, gray} + 10'd2) >> 2);
			default: profile_channel = channel;
		endcase
	end
endfunction

assign r_out = profile_channel(r_in, luma, mode);
assign g_out = profile_channel(g_in, luma, mode);
assign b_out = profile_channel(b_in, luma, mode);

endmodule
