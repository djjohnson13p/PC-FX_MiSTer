// HuC6230 PSG: the HuC6280 PSG, six wave channels, noise on 5 and 6, LFO
// from channel 2 onto channel 1. Counters step on fmaster = master / 3.
// The 6 x 32 x 5 wave memory is one RAM: port A takes CPU writes and
// otherwise reads channel 2's current sample for the LFO; port B is the
// mixer's voice read, one channel per clock.
//
// HuC6280A behaviour: a channel with ON = DDA = 0 is silent, and samples
// are centred as 2s - 31. Noise, while the channel is on, takes priority
// over D/A. Writes with CHSEL 6 or 7 are dropped.

module huc6230_psg #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_master,
	output logic        ce_fm,

	input  logic        we,
	input  logic  [3:0] a,
	input  logic  [7:0] d,

	input  logic  [2:0] v_ch,      // voice asked for at t, answered at t + 1
	output logic signed [5:0] v_samp,
	output logic  [4:0] v_att_l,   // -1.5 dB steps
	output logic  [4:0] v_att_r,
	output logic        v_sil_l,   // 45 dB down or more: no output
	output logic        v_sil_r,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so,
	input  logic        ss_mem_sel,
	input  logic  [7:0] ss_mem_addr,
	input  logic        ss_mem_we,
	input  logic  [4:0] ss_mem_wdata,
	output logic  [4:0] ss_mem_rdata
);

import huc6230_pkg::*;

psg_state_t st;
logic [5:0] wf_res, wf_inc;

wire       chv      = st.chsel < 3'd6;
wire [2:0] cs       = chv ? st.chsel : 3'd0;
wire       cur_chon = st.ch[cs].chon;
wire       cur_dda  = st.ch[cs].dda;

// Channel 2's wave is not written while LF TRG is set.
wire wave_we = we && a == 4'h6 && chv && !cur_dda && !(cs == 3'd1 && st.lftrg);

assign ce_fm = ce_master && st.fm_div == 2'd2;

// ---------------------------------------------------------------- wave RAM

logic [7:0] pa_addr, pb_addr;
logic       pa_we;
logic [4:0] pa_wd, qa, qb;

always_comb begin
	if (ss_mem_sel) begin
		pa_addr = ss_mem_addr;
		pa_we   = ss_mem_we;
		pa_wd   = ss_mem_wdata;
	end else begin
		pa_addr = wave_we ? {cs, st.ch[cs].wf_addr} : {3'd1, st.ch[1].wf_addr};
		pa_we   = wave_we;
		pa_wd   = d[4:0];
	end
end

assign pb_addr = {v_ch, st.ch[v_ch].wf_addr};

cache_ram_dp #(
	.ADDR_WIDTH (8),
	.DATA_WIDTH (5)
) u_wave (
	.clk_i     (clk),
	.addr_a_i  (pa_addr),
	.wren_a_i  (pa_we),
	.wdata_a_i (pa_wd),
	.q_a_o     (qa),
	.addr_b_i  (pb_addr),
	.wren_b_i  (1'b0),
	.wdata_b_i (5'd0),
	.q_b_o     (qb)
);

assign ss_mem_rdata = qa;

// Channel 2's current sample: port A answers the clock after a read of it
// or a write to it; otherwise keep the last answer.
logic       lfo_ok;
logic [4:0] lfo_hold, lfo_src;

always_ff @(posedge clk) begin
	lfo_ok   <= !ss_mem_sel && (!wave_we || cs == 3'd1);
	lfo_hold <= lfo_src;
end

assign lfo_src = lfo_ok ? qa : lfo_hold;

// Channel 1's frequency offset: channel 2's sample less 10h, scaled by LF
// CTL, only while channel 2 is on.
logic [11:0] lfo_add;
wire  [4:0] lfo_s = lfo_src ^ 5'h10;

always_comb begin
	case (st.lfctl)
		2'd1:    lfo_add = {{7{lfo_s[4]}}, lfo_s};
		2'd2:    lfo_add = {{5{lfo_s[4]}}, lfo_s, 2'b00};
		2'd3:    lfo_add = {{4{lfo_s[4]}}, lfo_s[3:0], 4'b0000};
		default: lfo_add = 12'd0;
	endcase
	if (!st.ch[1].chon)
		lfo_add = 12'd0;
end

// ---------------------------------------------------------------- registers and counters

function automatic logic [12:0] wf_reload(input logic [11:0] freq, input logic [11:0] add);
	wf_reload = {freq - 12'd1 + add, 1'b1};
endfunction

function automatic logic [11:0] ng_reload(input logic [4:0] f);
	ng_reload = f == 5'h1F ? 12'h03F : {~f - 5'd1, 7'h7F};
endfunction

always_ff @(posedge clk) begin
	wf_res <= '0;
	wf_inc <= '0;
	if (we && chv) begin
		if (a == 4'h4 && cur_dda && !d[6])
			wf_res[cs] <= 1'b1;
		if (a == 4'h6 && !cur_dda && !cur_chon)
			wf_inc[cs] <= 1'b1;
	end
end

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= psg_state_t'({st[SS_PSG_BITS-2:0], ss_si});
	end else begin
		if (ce_master)
			st.fm_div <= st.fm_div == 2'd2 ? 2'd0 : st.fm_div + 2'd1;

		if (we) begin
			case (a)
				4'h0: st.chsel <= d[2:0];
				4'h1: begin
					st.lmal <= d[7:4];
					st.rmal <= d[3:0];
				end
				4'h2: if (chv) st.ch[cs].freq[7:0]  <= d;
				4'h3: if (chv) st.ch[cs].freq[11:8] <= d[3:0];
				4'h4: if (chv) begin
					st.ch[cs].chon <= d[7];
					st.ch[cs].dda  <= d[6];
					st.ch[cs].al   <= d[4:0];
				end
				4'h5: if (chv) begin
					st.ch[cs].lal <= d[7:4];
					st.ch[cs].ral <= d[3:0];
				end
				4'h6: if (chv && cur_chon) st.ch[cs].da_out <= d[4:0];
				4'h7: if (cs[2]) begin
					st.ng[cs[0]].ne      <= d[7];
					st.ng[cs[0]].ng_freq <= d[4:0];
				end
				4'h8: st.lfo_freq <= d;
				4'h9: begin
					st.lfctl <= d[1:0];
					st.lftrg <= d[7];
				end
				default: ;
			endcase
		end

		for (int i = 0; i < 6; i++) begin
			if (wf_res[i])
				st.ch[i].wf_addr <= 5'd0;
			if (wf_inc[i])
				st.ch[i].wf_addr <= st.ch[i].wf_addr + 5'd1;

			if (i == 1 && (st.lfctl != 2'd0 || st.lftrg)) begin
				// Channel 2 as the LFO: its wave steps at fmaster / LFO FRQ.
				// LF TRG holds it at address 0 whatever LF CTL.
				if (st.lftrg) begin
					st.ch[i].wf_addr <= 5'd0;
					st.ch[i].wf_cnt  <= wf_reload(st.ch[i].freq, 12'd0);
					st.lfo_cnt       <= st.lfo_freq - 8'd1;
				end else if (ce_fm) begin
					st.lfo_cnt <= st.lfo_cnt - 8'd1;
					if (st.lfo_cnt == 8'd0) begin
						st.lfo_cnt      <= st.lfo_freq - 8'd1;
						st.ch[i].wf_cnt <= st.ch[i].wf_cnt - 13'd1;
						if (st.ch[i].wf_cnt == 13'd0) begin
							st.ch[i].wf_cnt  <= wf_reload(st.ch[i].freq, 12'd0);
							st.ch[i].wf_addr <= st.ch[i].wf_addr + 5'd1;
						end
					end
				end
			end else if (!st.ch[i].chon) begin
				st.ch[i].wf_cnt <= wf_reload(st.ch[i].freq, i == 0 ? lfo_add : 12'd0);
			end else if (!st.ch[i].dda && ce_fm) begin
				st.ch[i].wf_cnt <= st.ch[i].wf_cnt - 13'd1;
				if (st.ch[i].wf_cnt == 13'd0) begin
					st.ch[i].wf_cnt  <= wf_reload(st.ch[i].freq, i == 0 ? lfo_add : 12'd0);
					st.ch[i].wf_addr <= st.ch[i].wf_addr + 5'd1;
				end
			end
		end

		for (int n = 0; n < 2; n++) begin
			if (!st.ng[n].ne) begin
				st.ng[n].ng_cnt <= ng_reload(st.ng[n].ng_freq);
			end else if (ce_fm) begin
				st.ng[n].ng_cnt <= st.ng[n].ng_cnt - 12'd1;
				if (st.ng[n].ng_cnt == 12'd0) begin
					st.ng[n].ng_cnt <= ng_reload(st.ng[n].ng_freq);
					if (st.ng[n].lfsr == 18'd0)
						st.ng[n].lfsr[0] <= 1'b1;
					else
						st.ng[n].lfsr <= {st.ng[n].lfsr[0] ^ st.ng[n].lfsr[1] ^ st.ng[n].lfsr[11]
							^ st.ng[n].lfsr[12] ^ st.ng[n].lfsr[17], st.ng[n].lfsr[17:1]};
				end
			end
		end
	end
end

assign ss_so = SAVESTATE ? st[SS_PSG_BITS-1] : ss_si;

// ---------------------------------------------------------------- voice port

logic [2:0] v_ch_d;

always_ff @(posedge clk)
	v_ch_d <= v_ch;

wire       vc_chon = st.ch[v_ch_d].chon;
wire       vc_dda  = st.ch[v_ch_d].dda;
wire [4:0] vc_al   = st.ch[v_ch_d].al;
wire [3:0] vc_lal  = st.ch[v_ch_d].lal;
wire [3:0] vc_ral  = st.ch[v_ch_d].ral;
logic [4:0] v_lvl;
logic [5:0] sum_l, sum_r;

always_comb begin
	if (v_ch_d[2] && st.ng[v_ch_d[0]].ne && vc_chon)
		v_lvl = {5{st.ng[v_ch_d[0]].lfsr[0]}};
	else if (vc_dda)
		v_lvl = st.ch[v_ch_d].da_out;
	else
		v_lvl = qb;

	v_samp = (vc_chon || vc_dda) ? $signed({v_lvl, 1'b0} - 6'd31) : 6'sd0;

	// Attenuation in 3 dB units from LMAL, LAL and AL's top four bits.
	sum_l   = {2'b00, ~st.lmal} + {2'b00, ~vc_lal} + {2'b00, ~vc_al[4:1]};
	sum_r   = {2'b00, ~st.rmal} + {2'b00, ~vc_ral} + {2'b00, ~vc_al[4:1]};
	v_sil_l = sum_l >= 6'd15;
	v_sil_r = sum_r >= 6'd15;
	v_att_l = {sum_l[3:0], ~vc_al[0]};
	v_att_r = {sum_r[3:0], ~vc_al[0]};
end

endmodule
