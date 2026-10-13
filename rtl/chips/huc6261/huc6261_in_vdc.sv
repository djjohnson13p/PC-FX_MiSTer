// HuC6261 input from the two 6270s. The upper chip (B) shows unless its
// colour is 0; in 256-colour mode an upper pixel with VDB7 set and a
// non-zero colour combines both chips' colours, lower chip high.
// Output is {key, SP/BG, data}, two dots behind the pins so it lines up
// with the KING and Rainbow inputs; in 7 MHz mode two master clocks more.

module huc6261_in_vdc
(
	input  logic       clk,
	input  logic       ce_master,
	input  logic       ce_dck70,
	input  logic       dc7,
	input  logic       bg256,
	input  logic       sp256,

	input  logic [7:0] vda,
	input  logic       spbg_a,
	input  logic [7:0] vdb,
	input  logic       spbg_b,

	output logic [9:0] pix
);

wire up_colour = vdb[3:0] != 4'd0;
wire combine   = up_colour && vdb[7] && (spbg_b ? sp256 : bg256);

logic [9:0] merged;
always_comb begin
	if (combine)
		merged = {1'b0, spbg_b, vda[3:0], vdb[3:0]};
	else if (up_colour)
		merged = {1'b0, spbg_b, vdb};
	else
		merged = {vda[3:0] == 4'd0, spbg_a, vda};
end

logic [9:0] cap, dly, hd1, hd2;

always_ff @(posedge clk) begin
	if (ce_dck70) begin
		cap <= merged;
		dly <= cap;
	end
	if (ce_master) begin
		hd1 <= dly;
		hd2 <= hd1;
	end
end

assign pix = dc7 ? hd2 : dly;

endmodule
