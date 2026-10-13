// HuC6230 ADPCM decoders #1 and #2, one datapath: channel 1 the clock after
// a tick, channel 2 the clock after that. Each nibble is a sign and three
// data bits; delta = (data + 1) * scale / 8, the scale level steps by
// -1 -1 -1 -1 +2 +4 +6 +8 within 0-48, and the 12.3 PCM value saturates at
// 0 and 4095.875. With interpolation the delta is shifted down by DIV and
// added on every tick of the nibble; without it, the whole delta goes in on
// the first tick. The DAC takes the top 10 bits. A cycle without data, or a
// channel in soft reset, holds; reset puts the PCM value at 800h.
//
// Two OSD options, not hardware:
//   enc_compat  decode as the faulty official encoder assumed: delta doubled,
//               top scale 1522, output at half amplitude around 800h
//   no_click    a soft reset moves the jump to centre into an offset that
//               fades by 1/256 per 31.47 kHz tick (about 8 ms)

module huc6230_adpcm_dec #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic       clk,
	input  logic       reset,

	input  logic       tick,
	input  logic       boundary,
	input  logic [1:0] nib,
	input  logic       first,
	input  logic [5:0] r10,
	input  logic [1:0] pair_ok,
	input  logic [1:0][15:0] pair,

	input  logic       enc_compat,
	input  logic       no_click,

	output logic [1:0][9:0] dac,

	input  logic       ss_shift,
	input  logic       ss_si,
	output logic       ss_so
);

import huc6230_pkg::*;

localparam logic [14:0] CENTRE = 15'h4000;

dec_state_t st;

// Tick details held for the two processing clocks.
logic [1:0] go;
logic [1:0] nib_r;
logic       first_r;

always_ff @(posedge clk) begin
	go <= {go[0], tick};
	if (tick) begin
		nib_r   <= nib;
		first_r <= first;
	end
end

wire [1:0] div = r10[1:0];
wire       c   = go[1];

/* verilator lint_off UNUSEDSIGNAL */
adpcm_ch_t   ch;          // the channel being decoded (its fade is not)
/* verilator lint_on UNUSEDSIGNAL */
logic  [3:0] n;
logic [10:0] scale;
logic [14:0] mag, slice;
logic [15:0] delta, add;
logic [16:0] sum;
logic  [6:0] idx_sum;
logic  [5:0] idx_next;
logic        interp;

always_comb begin
	ch     = st.ch[c];
	n      = 4'(ch.word >> {nib_r, 2'b00});
	scale  = enc_compat && ch.idx == 6'd48 ? 11'd1522 : adpcm_scale(ch.idx);
	mag    = {4'd0, scale} + (n[0] ? {4'd0, scale} : 15'd0)
		+ (n[1] ? {3'd0, scale, 1'b0} : 15'd0) + (n[2] ? {2'd0, scale, 2'b00} : 15'd0);
	if (enc_compat)
		mag = {mag[13:0], 1'b0};
	interp = (c ? r10[3] : r10[2]) && div != 2'd0;
	slice  = interp ? mag >> div : mag;
	delta  = n[3] ? 16'(-{1'b0, slice}) : {1'b0, slice};
	add    = first_r ? delta : interp ? ch.part : 16'd0;
	sum    = {2'b00, ch.p} + {add[15], add};

	case (n[2:0])
		3'd4:    idx_sum = {1'b0, ch.idx} + 7'd2;
		3'd5:    idx_sum = {1'b0, ch.idx} + 7'd4;
		3'd6:    idx_sum = {1'b0, ch.idx} + 7'd6;
		3'd7:    idx_sum = {1'b0, ch.idx} + 7'd8;
		default: idx_sum = {1'b0, ch.idx} - 7'd1;
	endcase
	if (idx_sum[6])
		idx_next = 6'd0;
	else if (idx_sum > 7'd48)
		idx_next = 6'd48;
	else
		idx_next = idx_sum[5:0];
end

// R10 changes on a tick's clock, so a reset starting there adds its jump to
// the offset after that tick's fade step.
logic [1:0][23:0] fade_dec;

always_comb
	for (int i = 0; i < 2; i++)
		fade_dec[i] = tick ? st.ch[i].fade - {{8{st.ch[i].fade[23]}}, st.ch[i].fade[23:8]} : st.ch[i].fade;

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
		st.ch[0].p <= CENTRE;
		st.ch[1].p <= CENTRE;
	end else if (SAVESTATE && ss_shift) begin
		st <= dec_state_t'({st[SS_DEC_BITS-2:0], ss_si});
	end else begin
		if (boundary) begin
			st.ch[0].word  <= pair[0];
			st.ch[1].word  <= pair[1];
			st.ch[0].valid <= pair_ok[0];
			st.ch[1].valid <= pair_ok[1];
		end

		if ((go[0] || go[1]) && ch.valid) begin
			if (first_r) begin
				st.ch[c].idx  <= idx_next;
				st.ch[c].part <= delta;
			end
			if (sum[16])
				st.ch[c].p <= 15'd0;
			else if (sum[15])
				st.ch[c].p <= 15'h7FFF;
			else
				st.ch[c].p <= sum[14:0];
		end

		for (int i = 0; i < 2; i++) begin
			st.ch[i].rst_q <= r10[4 + i];
			st.ch[i].fade  <= fade_dec[i];
			if (r10[4 + i]) begin
				st.ch[i].p     <= CENTRE;
				st.ch[i].idx   <= 6'd0;
				st.ch[i].part  <= 16'd0;
				st.ch[i].valid <= 1'b0;
				if (no_click && !st.ch[i].rst_q)
					st.ch[i].fade <= fade_add(fade_dec[i], st.ch[i].p);
			end
		end
	end
end

assign ss_so  = SAVESTATE ? st[SS_DEC_BITS-1] : ss_si;

// The offset grows by the jump the reset makes, within +-4000h.
function automatic logic [23:0] fade_add(input logic [23:0] f, input logic [14:0] pcm);
	logic [24:0] s;
	s = {f[23], f} + {{3{~pcm[14]}}, pcm[13:0], 8'd0};
	if (!s[24] && s[23:22] != 2'b00)
		fade_add = 24'h3FFFFF;
	else if (s[24] && s[23:22] != 2'b11)
		fade_add = 24'hC00000;
	else
		fade_add = s[23:0];
endfunction

// What the DAC sees: the PCM value plus the fading offset, then halved
// around centre in encoder-compatible mode.
logic [1:0][16:0] v;
logic [1:0][14:0] o;

always_comb begin
	for (int i = 0; i < 2; i++) begin
		v[i] = {2'b00, st.ch[i].p} + {{3{st.ch[i].fade[23]}}, st.ch[i].fade[21:8]};
		if (v[i][16])
			o[i] = 15'd0;
		else if (v[i][15])
			o[i] = 15'h7FFF;
		else
			o[i] = v[i][14:0];
		if (enc_compat)
			o[i] = {1'b0, o[i][14:1]} + 15'h2000;
		dac[i] = o[i][14:5];
	end
end

endmodule
