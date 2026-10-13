// What is plugged into the two K-ports, fed from MiSTer's joysticks and
// mouse.
//
//   port 0: pad (joy 0), mouse, or a multitap with pads on joys 0-3
//   port 1: pad (joy 1, or joy 4 behind a tap), or the mouse when port 0
//           has none
//
// DATA is a wired line with a pull-up: the glue and the device each pull it
// low or let it go. Joystick bits: 0 right, 1 left, 2 down, 3 up, 4-9 I-VI,
// 10 Select, 11 Run. Savestate chain: tap, pads 0-4, mouse
// (TAP_BITS + 5 * PAD_BITS + MOUSE_BITS).

module kport_devices
	import kport_pkg::*;
#(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,

	input  logic  [1:0] port0_dev,      // 0 pad, 1 mouse, 2 tap, 3 none
	input  logic  [1:0] port1_dev,      // 0 pad, 1 mouse, 2-3 none
	input  logic        mode1,
	input  logic        mode2,

	input  logic [11:0] joy0,
	input  logic [11:0] joy1,
	input  logic [11:0] joy2,
	input  logic [11:0] joy3,
	input  logic [11:0] joy4,
	input  logic [24:0] ps2_mouse,

	input  logic        k0_latch_n,
	input  logic        k0_clk,
	input  logic        k0_rw,
	input  logic        k0_data_o,
	input  logic        k0_data_oe,
	output logic        k0_data_i,
	input  logic        k1_latch_n,
	input  logic        k1_clk,
	input  logic        k1_rw,
	input  logic        k1_data_o,
	input  logic        k1_data_oe,
	output logic        k1_data_i,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

function automatic logic [11:0] pad_buttons(input logic [11:0] j);
	pad_buttons = {j[1], j[2], j[0], j[3], j[11:4]};
endfunction

wire tap_on   = port0_dev == 2'd2;
wire mouse_on0 = port0_dev == 2'd1;
wire mouse_on1 = port1_dev == 2'd1 && !mouse_on0;

// Pads 0-3 sit behind the tap, which passes LATCH, CLK and R/W through
// unchanged, so pad 0 also serves port 0 directly.
logic       dn_latch_n, dn_kclk, dn_rw;
logic [4:0] pad_o, pad_oe;
logic [3:0] dn_data;
logic       tap_o, tap_oe, mouse_o, mouse_oe;
logic [6:0] ss_c;

kport_tap #(.SAVESTATE(SAVESTATE)) u_tap (
	.clk        (clk),
	.reset      (reset),
	.latch_n    (k0_latch_n),
	.kclk       (k0_clk),
	.rw         (k0_rw),
	.data_o     (tap_o),
	.data_oe    (tap_oe),
	.dn_latch_n (dn_latch_n),
	.dn_kclk    (dn_kclk),
	.dn_rw      (dn_rw),
	.dn_data    (dn_data),
	.ss_shift   (ss_shift),
	.ss_si      (ss_si),
	.ss_so      (ss_c[0])
);

logic [11:0] joy [5];
assign joy[0] = joy0;
assign joy[1] = joy1;
assign joy[2] = joy2;
assign joy[3] = joy3;
assign joy[4] = tap_on ? joy4 : joy1;

genvar i;
generate
for (i = 0; i < 5; i = i + 1) begin : g_pad
	kport_pad #(.SAVESTATE(SAVESTATE)) u_pad (
		.clk      (clk),
		.reset    (reset),
		.latch_n  (i < 4 ? dn_latch_n : k1_latch_n),
		.kclk     (i < 4 ? dn_kclk : k1_clk),
		.rw       (i < 4 ? dn_rw : k1_rw),
		.data_o   (pad_o[i]),
		.data_oe  (pad_oe[i]),
		.buttons  (pad_buttons(joy[i])),
		.mode1    (mode1),
		.mode2    (mode2),
		.ss_shift (ss_shift),
		.ss_si    (ss_c[i]),
		.ss_so    (ss_c[i+1])
	);
end
endgenerate

assign dn_data = pad_o[3:0] | ~pad_oe[3:0];

// PS/2 reports Y up as positive; the PC-FX mouse counts down as positive.
logic       ps2_t;
wire  [8:0] ps2_y = {ps2_mouse[5], ps2_mouse[23:16]};

always_ff @(posedge clk)
	ps2_t <= ps2_mouse[24];

wire unused_ps2 = &{1'b0, ps2_mouse[7:6], ps2_mouse[3:2]};

kport_mouse #(.SAVESTATE(SAVESTATE)) u_mouse (
	.clk      (clk),
	.reset    (reset),
	.latch_n  (mouse_on1 ? k1_latch_n : k0_latch_n),
	.kclk     (mouse_on1 ? k1_clk : k0_clk),
	.rw       (mouse_on1 ? k1_rw : k0_rw),
	.data_o   (mouse_o),
	.data_oe  (mouse_oe),
	.move     (ps2_mouse[24] != ps2_t),
	.dx       ({ps2_mouse[4], ps2_mouse[15:8]}),
	.dy       (ps2_y == 9'h100 ? 9'h0FF : 9'd0 - ps2_y),
	.btn_l    (ps2_mouse[0]),
	.btn_r    (ps2_mouse[1]),
	.ss_shift (ss_shift),
	.ss_si    (ss_c[5]),
	.ss_so    (ss_c[6])
);

assign ss_so = ss_c[6];

// What each device pulls on its port's DATA (1 = released).
logic dev0, dev1;
always_comb begin
	case (port0_dev)
		2'd0:    dev0 = pad_o[0] | ~pad_oe[0];
		2'd1:    dev0 = mouse_o | ~mouse_oe;
		2'd2:    dev0 = tap_o | ~tap_oe;
		default: dev0 = 1'b1;
	endcase
	if (port1_dev == 2'd0)
		dev1 = pad_o[4] | ~pad_oe[4];
	else if (mouse_on1)
		dev1 = mouse_o | ~mouse_oe;
	else
		dev1 = 1'b1;
end

assign k0_data_i = dev0 & (k0_data_o | ~k0_data_oe);
assign k1_data_i = dev1 & (k1_data_o | ~k1_data_oe);

endmodule
