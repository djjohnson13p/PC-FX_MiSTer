// FX-BMP backup cartridge on the front edge connector: the D65612 decoder
// and a 128 KB SRAM whose contents live in SDRAM (bank 1, rows 1536-1663).
//
//   0xE8000000-0xE9FFFFFF  RAM, A26 = A25 = 0; bytes on every second
//                          address (no A0 on the edge), 128 KB mirrored
//   0xEA000000-0xEBFFFFFF  battery status: D0 = 1 when the battery is good,
//                          the other lines read high (FF good, FE low)
//   A26 = 1, or no card    nothing drives LD
//
// RAM read data is on D from 2 * FX_RD_SCK - 1 clk_sys after the select
// (see pcfx_memory). The cart notes only describe the decode of /OE; writes
// are decoded the same way here. Of the edge's address lines only A25, A26
// and A17:1 matter to a 128 KB card.

module fx_bmp
(
	input  logic        inserted,
	input  logic        battery_ok,

	input  logic        cartsel_n,
	input  logic        oe_n,
	input  logic        we_n,
	input  logic        a26,
	input  logic        a25,
	input  logic [17:1] a,
	input  logic  [7:0] d_i,
	output logic  [7:0] d_o,
	output logic        d_oe,

	output logic        mem_cs_n,
	output logic        mem_rd_n,
	output logic        mem_wr_n,
	output logic [16:0] mem_a,
	output logic  [7:0] mem_d,
	input  logic  [7:0] mem_q
);

wire sel  = inserted && !cartsel_n && !a26;
wire ram  = sel && !a25;
wire batt = sel && a25;

assign mem_cs_n = !ram;
assign mem_rd_n = oe_n;
assign mem_wr_n = we_n;
assign mem_a    = a;
assign mem_d    = d_i;

assign d_o  = batt ? {7'h7F, battery_ok} : mem_q;
assign d_oe = sel && !oe_n;

endmodule
