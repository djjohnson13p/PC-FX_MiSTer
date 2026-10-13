// HuC6271 variable-length decoder: strip data bytes from the FIFO to
// coefficients, null runs and palette runs.
//
// DCT strip (FF: 128 Q-table bytes first, Y then C): 16 macroblocks, each
// Y top-left, bottom-left, top-right, bottom-right, U, V. A block is a DC
// difference (Y or UV table, then that many bits) and AC run/size codes in
// zigzag order up to EOB or position 63. The DC-Y table also carries QCx
// (the scale for the next blocks) and NRL: an AC-Y code follows, run + 1
// macroblocks are filled with the NRL colour and the DC predictors restart.
// QCx codes right after the last macroblock set up the next strip.
// Run-length strip (F0-F3): a byte is a palette index over a count of
// 4 - kind bits; count 0 means the next byte holds count - 1.
//
// The bit reader decodes as if missing bits were 0 and only consumes a
// code once all of its bits are in, so a strip that is short of data
// waits and is never completed.

module huc6271_vlc
	import huc6271_pkg::*;
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce,
	input  logic        init,
	input  logic        start,
	input  logic  [7:0] kind,

	input  logic        empty,
	output logic        pop,
	input  logic  [7:0] q,
	input  logic        in_end,     // every byte of the strip is in the FIFO or used

	output logic        qcx_wr,
	output logic  [3:0] qcx_d,
	output logic        qw_v,       // Q-table byte
	output logic  [6:0] qw_a,
	output logic  [7:0] qw_d,

	input  logic  [1:0] cb_full,    // coefficient buffers waiting for the IDCT
	output logic        cbuf,       // buffer of the block being decoded
	output logic        blk_start,
	output logic        blk_end,
	output logic  [3:0] mb,
	output logic  [2:0] blk,
	output logic        cw_v,
	output logic  [5:0] cw_idx,     // row * 8 + column
	output logic [11:0] cw_val,

	input  logic        pipe_idle,  // nothing left in the dequant, IDCT and writes
	output logic        nrl_v,
	output logic  [4:0] nrl_n,      // macroblocks from mb

	input  logic        run_rdy,
	input  logic        rle_full,
	output logic        run_v,
	output logic  [6:0] run_idx,
	output logic  [8:0] run_len,

	output logic        busy,
	output logic        done
);

typedef enum logic [4:0] {
	S_IDLE, S_QTAB, S_BLK, S_DCY, S_DCUV, S_DCMAG, S_AC, S_ACMAG,
	S_NRL, S_NRLMAG, S_NRLGO, S_NEXT, S_RLE, S_RLECNT, S_RLEOUT, S_TAIL, S_END
} vstate_t;

vstate_t     vs;
logic [31:0] bb;        // next bit at bb[31]
logic  [5:0] nb;
logic        inflight;
logic  [6:0] pos;       // zigzag position
logic  [3:0] msize;
logic        drop;
logic        nrl_fill;
logic [11:0] pred_y, pred_u, pred_v;
logic  [2:0] shift;
logic  [1:0] kbits;
logic        vbuf;
logic  [6:0] qn;        // Q-table byte count

// ------------------------------------------------------------ tables

// DC-Y: 0-9 difference size, 0F null run, 10-1F QCx. {length, symbol}
function automatic logic [11:0] dcy_tab(input logic [8:0] c);
	logic [11:0] d;
	casez (c)
		9'b000??????: d = {4'd3, 8'h00};
		9'b001??????: d = {4'd3, 8'h02};
		9'b010??????: d = {4'd3, 8'h03};
		9'b011??????: d = {4'd3, 8'h04};
		9'b100??????: d = {4'd3, 8'h05};
		9'b101??????: d = {4'd3, 8'h06};
		9'b110??????: d = {4'd3, 8'h07};
		9'b1110?????: d = {4'd4, 8'h01};
		9'b111100???: d = {4'd6, 8'h08};
		9'b1111010??: d = {4'd7, 8'h09};
		9'b1111011??: d = {4'd7, 8'h0F};
		default:      d = {4'd9, 4'h1, c[3:0]};
	endcase
	return d;
endfunction

function automatic logic [11:0] dcuv_tab(input logic [7:0] c);
	logic [11:0] d;
	casez (c)
		8'b00??????: d = {4'd2, 8'h00};
		8'b01??????: d = {4'd2, 8'h01};
		8'b10??????: d = {4'd2, 8'h02};
		8'b110?????: d = {4'd3, 8'h03};
		8'b1110????: d = {4'd4, 8'h04};
		8'b11110???: d = {4'd5, 8'h05};
		8'b111110??: d = {4'd6, 8'h06};
		8'b1111110?: d = {4'd7, 8'h07};
		8'b11111110: d = {4'd8, 8'h08};
		default:     d = {4'd8, 8'h09};
	endcase
	return d;
endfunction

// 12-bit codes count up from 1111_0001_0000.
function automatic logic [7:0] ac_long(input logic [6:0] i, input logic uv);
	logic [7:0] s;
	s = 8'h10;
	if (!uv) begin
		case (i)
			7'd0: s = 8'h08; 7'd1: s = 8'h14; 7'd2: s = 8'h15; 7'd3: s = 8'h16; 7'd4: s = 8'h17; 7'd5: s = 8'h18;
			7'd6: s = 8'h23; 7'd7: s = 8'h24; 7'd8: s = 8'h25; 7'd9: s = 8'h26; 7'd10: s = 8'h27; 7'd11: s = 8'h28;
			7'd12: s = 8'h32; 7'd13: s = 8'h33; 7'd14: s = 8'h34; 7'd15: s = 8'h35; 7'd16: s = 8'h36; 7'd17: s = 8'h37;
			7'd18: s = 8'h38; 7'd19: s = 8'h42; 7'd20: s = 8'h43; 7'd21: s = 8'h44; 7'd22: s = 8'h45; 7'd23: s = 8'h46;
			7'd24: s = 8'h47; 7'd25: s = 8'h48; 7'd26: s = 8'h52; 7'd27: s = 8'h53; 7'd28: s = 8'h54; 7'd29: s = 8'h55;
			7'd30: s = 8'h56; 7'd31: s = 8'h57; 7'd32: s = 8'h58; 7'd33: s = 8'h62; 7'd34: s = 8'h63; 7'd35: s = 8'h64;
			7'd36: s = 8'h65; 7'd37: s = 8'h66; 7'd38: s = 8'h67; 7'd39: s = 8'h68; 7'd40: s = 8'h72; 7'd41: s = 8'h73;
			7'd42: s = 8'h74; 7'd43: s = 8'h75; 7'd44: s = 8'h76; 7'd45: s = 8'h77; 7'd46: s = 8'h78; 7'd47: s = 8'h81;
			7'd48: s = 8'h82; 7'd49: s = 8'h83; 7'd50: s = 8'h84; 7'd51: s = 8'h85; 7'd52: s = 8'h86; 7'd53: s = 8'h87;
			7'd54: s = 8'h88;
			default: ;
		endcase
	end else begin
		case (i)
			7'd0: s = 8'h06; 7'd1: s = 8'h07; 7'd2: s = 8'h08; 7'd3: s = 8'h14; 7'd4: s = 8'h15; 7'd5: s = 8'h16;
			7'd6: s = 8'h17; 7'd7: s = 8'h18; 7'd8: s = 8'h23; 7'd9: s = 8'h24; 7'd10: s = 8'h25; 7'd11: s = 8'h26;
			7'd12: s = 8'h27; 7'd13: s = 8'h28; 7'd14: s = 8'h33; 7'd15: s = 8'h34; 7'd16: s = 8'h35; 7'd17: s = 8'h36;
			7'd18: s = 8'h37; 7'd19: s = 8'h38; 7'd20: s = 8'h42; 7'd21: s = 8'h43; 7'd22: s = 8'h44; 7'd23: s = 8'h45;
			7'd24: s = 8'h46; 7'd25: s = 8'h47; 7'd26: s = 8'h48; 7'd27: s = 8'h52; 7'd28: s = 8'h53; 7'd29: s = 8'h54;
			7'd30: s = 8'h55; 7'd31: s = 8'h56; 7'd32: s = 8'h57; 7'd33: s = 8'h58; 7'd34: s = 8'h62; 7'd35: s = 8'h63;
			7'd36: s = 8'h64; 7'd37: s = 8'h65; 7'd38: s = 8'h66; 7'd39: s = 8'h67; 7'd40: s = 8'h68; 7'd41: s = 8'h72;
			7'd42: s = 8'h73; 7'd43: s = 8'h74; 7'd44: s = 8'h75; 7'd45: s = 8'h76; 7'd46: s = 8'h77; 7'd47: s = 8'h78;
			7'd48: s = 8'h82; 7'd49: s = 8'h83; 7'd50: s = 8'h84; 7'd51: s = 8'h85; 7'd52: s = 8'h86; 7'd53: s = 8'h87;
			7'd54: s = 8'h88;
			default: ;
		endcase
	end
	// From index 55 both tables list runs 9-15 with sizes 1-8, then ZRL.
	if (i >= 7'd55 && i < 7'd111)
		s = {4'(7'd9 + ((i - 7'd55) >> 3)), 4'((i - 7'd55) & 7'd7) + 4'd1};
	return s;
endfunction

// AC: run << 4 | size; 00 end of block, 10 sixteen zeros.
function automatic logic [11:0] ac_tab(input logic [11:0] c, input logic uv);
	logic [11:0] d;
	casez (c[11:3])
		9'b00???????: d = {4'd2, 8'h01};
		9'b01???????: d = {4'd2, 8'h02};
		9'b100??????: d = {4'd3, uv ? 8'h11 : 8'h03};
		9'b1010?????: d = {4'd4, uv ? 8'h03 : 8'h04};
		9'b1011?????: d = {4'd4, uv ? 8'h21 : 8'h11};
		9'b11000????: d = {4'd5, uv ? 8'h04 : 8'h05};
		9'b11001????: d = {4'd5, 8'h12};
		9'b11010????: d = {4'd5, uv ? 8'h31 : 8'h21};
		9'b11011????: d = uv ? {4'd5, 8'h41} : (c[6] ? {4'd6, 8'h31} : {4'd6, 8'h06});
		9'b11111????: d = {4'd5, 8'h00};
		9'b111000???: d = {4'd6, uv ? 8'h51 : 8'h41};
		9'b1110010??: d = uv ? {4'd7, 8'h05} : {4'd6, 8'h51};
		9'b1110011??: d = uv ? {4'd7, 8'h13} : {4'd6, 8'h51};
		9'b1110100??: d = {4'd7, uv ? 8'h22 : 8'h13};
		9'b1110101??: d = {4'd7, uv ? 8'h61 : 8'h22};
		9'b1110110??: d = {4'd7, uv ? 8'h71 : 8'h61};
		9'b111011100: d = {4'd9, uv ? 8'h32 : 8'h07};
		9'b111011101: d = {4'd9, uv ? 8'h81 : 8'h71};
		default:
			if (c[11:7] == 5'b11110 && c[6:4] != 3'b000)
				d = {4'd12, ac_long(c[6:0] - 7'd16, uv)};
			else
				d = {4'd11, !c[4], c[3:1], 4'h9};   // run = code - 111_0111_1000
	endcase
	return d;
endfunction

// ------------------------------------------------------------ bit reader

wire [11:0] t_dcy  = dcy_tab(bb[31:23]);
wire [11:0] t_dcuv = dcuv_tab(bb[31:24]);
wire [11:0] t_ac   = ac_tab(bb[31:20], blk[2]);

// Magnitude bits: n bits, a leading 0 means negative (v - 2^n + 1).
function automatic logic [11:0] mag(input logic [8:0] top, input logic [3:0] n);
	logic [8:0]  raw;
	logic [11:0] m;
	raw = n == 4'd0 ? 9'd0 : top >> (4'd9 - n);
	m   = {3'd0, raw};
	if (n != 4'd0 && !raw[n - 4'd1])
		m = m + 12'd1 - (12'd1 << n);
	return m;
endfunction

wire [11:0] mval = mag(bb[31:23], msize);

logic [4:0] take;     // bits consumed this clock

always_comb begin
	take = 5'd0;
	case (vs)
		S_QTAB, S_RLE, S_RLECNT: if (nb >= 6'd8) take = 5'd8;
		S_DCY:    if (nb >= {2'd0, t_dcy[11:8]}) take = {1'b0, t_dcy[11:8]};
		S_TAIL:   if (nb >= {2'd0, t_dcy[11:8]} && t_dcy[4]) take = {1'b0, t_dcy[11:8]};
		S_DCUV:   if (nb >= {2'd0, t_dcuv[11:8]}) take = {1'b0, t_dcuv[11:8]};
		S_AC, S_NRL: if (nb >= {2'd0, t_ac[11:8]}) take = {1'b0, t_ac[11:8]};
		S_DCMAG, S_ACMAG, S_NRLMAG: if (nb >= {2'd0, msize}) take = {1'b0, msize};
		default: ;
	endcase
end

// A code of length 0 never exists, so "took" is nb >= length here.
wire took = (vs == S_DCMAG || vs == S_ACMAG || vs == S_NRLMAG) ? nb >= {2'd0, msize} : take != 5'd0;

wire want_bits = vs != S_IDLE && vs != S_END;
wire [5:0] nb_left = nb - {1'b0, take};
assign pop = ce && want_bits && !empty && (nb + (inflight ? 6'd8 : 6'd0)) <= 6'd24;

always_ff @(posedge clk) begin
	if (reset || init || start) begin
		bb       <= '0;
		nb       <= '0;
		inflight <= 1'b0;
	end else if (ce) begin
		inflight <= pop;
		bb       <= (bb << take) | (inflight ? {q, 24'd0} >> nb_left : 32'd0);
		nb       <= nb_left + (inflight ? 6'd8 : 6'd0);
	end
end

// ------------------------------------------------------------ sequencer

wire [7:0] sym = vs == S_DCY || vs == S_TAIL ? t_dcy[7:0] : vs == S_DCUV ? t_dcuv[7:0] : t_ac[7:0];
wire [6:0] pos_run = pos + {3'd0, sym[7:4]};
wire [6:0] pos_zrl = pos + (sym[7:4] == 4'd1 ? 7'd16 : {3'd0, sym[7:4]} + 7'd1);
wire [7:0] rbyte   = bb[31:24];
wire [7:0] rmask   = 8'hFF >> (3'd4 + {1'b0, kbits});
wire       last_mb = mb == 4'd15;

always_ff @(posedge clk) begin
	if (reset) begin
		vs   <= S_IDLE;
		vbuf <= 1'b0;
	end else if (ce) begin
		qcx_wr    <= 1'b0;
		qw_v      <= 1'b0;
		blk_start <= 1'b0;
		blk_end   <= 1'b0;
		cw_v      <= 1'b0;
		nrl_v     <= 1'b0;
		run_v     <= 1'b0;

		if (init) begin
			vs       <= S_IDLE;
			vbuf     <= 1'b0;
			nrl_fill <= 1'b0;
		end else if (start) begin
			mb     <= 4'd0;
			blk    <= 3'd0;
			qn     <= 7'd0;
			pred_y <= 12'd0;
			pred_u <= 12'd0;
			pred_v <= 12'd0;
			shift  <= 3'd4 - {1'b0, kind[1:0]};
			kbits  <= kind[1:0];
			nrl_fill <= 1'b0;
			vs     <= kind == 8'hFF ? S_QTAB : kind == 8'hF8 ? S_BLK : S_RLE;
		end else begin
			case (vs)
				S_QTAB: if (took) begin
					qw_v <= 1'b1;
					qw_d <= rbyte;
					qw_a <= qn;
					qn   <= qn + 7'd1;
					if (qn == 7'd127) begin
						qcx_wr <= 1'b1;
						qcx_d  <= 4'd4;
						vs     <= S_BLK;
					end
				end

				S_BLK: if (!cb_full[vbuf]) begin
					blk_start <= 1'b1;
					cbuf      <= vbuf;
					pos       <= 7'd1;
					vs        <= blk[2] ? S_DCUV : S_DCY;
				end

				S_DCY: if (took) begin
					if (sym[4]) begin
						qcx_wr <= 1'b1;
						qcx_d  <= sym[3:0];
					end else if (sym[3:0] == 4'hF) begin
						nrl_fill <= blk == 3'd0;
						vs       <= S_NRL;
					end else begin
						msize <= sym[3:0];
						vs    <= S_DCMAG;
					end
				end

				S_DCUV: if (took) begin
					msize <= sym[3:0];
					vs    <= S_DCMAG;
				end

				S_DCMAG: if (took) begin
					cw_v   <= 1'b1;
					cw_idx <= 6'd0;
					case (blk)
						3'd4: begin pred_u <= pred_u + mval; cw_val <= pred_u + mval; end
						3'd5: begin pred_v <= pred_v + mval; cw_val <= pred_v + mval; end
						default: begin pred_y <= pred_y + mval; cw_val <= pred_y + mval; end
					endcase
					vs <= S_AC;
				end

				S_AC: if (took) begin
					if (sym[3:0] == 4'd0) begin
						if (sym[7:4] == 4'd0 || pos_zrl >= 7'd64)
							vs <= S_NEXT;     // end of block
						else
							pos <= pos_zrl;
					end else begin
						pos   <= pos_run;
						drop  <= pos_run >= 7'd64;
						msize <= sym[3:0];
						vs    <= S_ACMAG;
					end
				end

				S_ACMAG: if (took) begin
					if (!drop) begin
						cw_v   <= 1'b1;
						cw_idx <= zigzag(pos[5:0]);
						cw_val <= mval;
					end
					pos <= pos + 7'd1;
					vs  <= drop || pos == 7'd63 ? S_NEXT : S_AC;
				end

				// A block is done, or a null run has been handed over.
				S_NEXT: begin
					if (nrl_fill) begin
						if (pipe_idle) begin
							nrl_fill <= 1'b0;
							pred_y   <= 12'd0;
							pred_u   <= 12'd0;
							pred_v   <= 12'd0;
							if (mb + nrl_n[3:0] == 4'd0 || nrl_n[4])
								vs <= S_TAIL;
							else begin
								mb <= mb + nrl_n[3:0];
								vs <= S_BLK;
							end
						end
					end else begin
						blk_end <= 1'b1;
						vbuf    <= !vbuf;
						if (blk == 3'd5) begin
							blk <= 3'd0;
							mb  <= mb + 4'd1;
							vs  <= last_mb ? S_TAIL : S_BLK;
						end else begin
							blk <= blk + 3'd1;
							vs  <= S_BLK;
						end
					end
				end

				S_NRL: if (took) begin
					msize <= sym[3:0];
					nrl_n <= {1'b0, sym[7:4]} + 5'd1;
					vs    <= S_NRLMAG;
				end

				S_NRLMAG: if (took) begin
					if (nrl_fill) begin
						if ({1'b0, mb} + nrl_n > 5'd16)
							nrl_n <= 5'd16 - {1'b0, mb};
						vs <= S_NRLGO;
					end else begin
						// Inside a macroblock the null run reads as difference 0.
						cw_v   <= 1'b1;
						cw_idx <= 6'd0;
						cw_val <= pred_y;
						vs     <= S_AC;
					end
				end

				S_NRLGO: if (pipe_idle) begin
					nrl_v <= 1'b1;
					vs    <= S_NEXT;
				end

				// Only whole QCx codes count; any other code, or a code cut
				// off by the end of the strip's bytes, ends the strip.
				S_TAIL: if (took) begin
					qcx_wr <= 1'b1;
					qcx_d  <= sym[3:0];
				end else if (nb >= {2'd0, t_dcy[11:8]} || (in_end && !inflight)) begin
					vs <= S_END;
				end

				S_RLE: if (rle_full) begin
					vs <= S_END;
				end else if (took) begin
					run_idx <= 7'(rbyte >> shift);
					run_len <= {1'b0, rbyte & rmask};
					vs      <= (rbyte & rmask) == 8'd0 ? S_RLECNT : S_RLEOUT;
				end

				S_RLECNT: if (rle_full) begin
					vs <= S_END;
				end else if (took) begin
					run_len <= {1'b0, rbyte} + 9'd1;
					vs      <= S_RLEOUT;
				end

				S_RLEOUT: if (rle_full) begin
					vs <= S_END;
				end else if (run_rdy) begin
					run_v <= 1'b1;
					vs    <= S_RLE;
				end

				default: ;
			endcase
		end
	end
end

assign busy = vs != S_IDLE && vs != S_END;
assign done = vs == S_TAIL || vs == S_END;

endmodule
