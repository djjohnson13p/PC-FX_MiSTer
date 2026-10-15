// The CD board's SCSI side as target ID 2: selection, the bus phases, the
// NEC command set, sense data, mode pages and the replies.
//
// The board's controller is a slow MCU and the bus shows it: about 0.5 ms
// to answer selection, 135 us per command byte, 221 us per reply byte and
// 0.7-5 ms per command. Sector data goes through the buffer hardware at
// the initiator's pace, REQ 160 ns after ACK. These times are the medians
// of logic-analyser captures of a real drive.
//
// Replies are built a byte ahead of the bus. Positions are absolute frame
// numbers (LBA + 150).

module cd_scsi_target #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce,
	input  logic        ce_x,       // 16.9344 MHz crystal
	input  logic        ce_t,       // crystal / 192
	input  logic        fast,       // fast drive: no gap between sectors

	// bus, asserted = 1, data true
	input  logic        sel,
	input  logic        atn,
	input  logic        ack,
	input  logic        rst,
	input  logic  [7:0] db_i,
	output logic        bsy,
	output logic        req,
	output logic        cd,
	output logic        io,
	output logic        msg,
	output logic  [7:0] db_o,
	output logic        db_oe,
	output logic        bus_rst,

	// mechanism; op and addresses hold until m_done
	output logic        m_go,
	output logic  [3:0] m_op,
	output logic [18:0] m_lba,
	output logic [18:0] m_end,
	output logic  [2:0] m_mode,
	input  logic        m_done,
	input  logic  [3:0] m_err,
	input  logic        m_ready,
	input  logic        m_nodisc,
	input  logic        m_change,
	input  logic  [1:0] m_play,
	input  logic        m_busy_end,
	input  logic [18:0] m_leadout,

	input  logic        p_ready,
	input  logic        p_eov,
	input  logic        p_slow,
	input  logic [15:0] p_base,
	output logic        p_next,

	input  logic [79:0] q_now,
	output logic        q_hold,
	input  logic        q_busy,
	input  logic  [6:0] toc_first,
	input  logic  [6:0] toc_last,

	output logic        mr_rq,
	output logic [15:0] mr_addr,
	input  logic        mr_ack,
	input  logic  [7:0] mr_q,

	output logic  [1:0] port0_sel,
	output logic  [1:0] port1_sel,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

import pcfx_cd_pkg::*;

scsi_t st;

function automatic scsi_mode_t mode_defaults();
	scsi_mode_t m;
	begin
		m = '0;
		m.p0e_immed = 1'b1;
		m.p0e_sel0  = 2'd1;
		m.p0e_sel1  = 2'd2;
		m.p2a       = 16'h0011;
		m.vretry    = 4'hF;
		m.block_len = 12'd2048;
		mode_defaults = m;
	end
endfunction

function automatic logic descriptor_valid(input logic [7:0] density, input logic [11:0] n);
	begin
		descriptor_valid = density == 8'h00 ? (n == 12'd2048 || n == 12'd2336 || n == 12'd2340) :
		                   density == 8'h81 && (n == 12'd2048 || n == 12'd2056 || n == 12'd2324 ||
		                   n == 12'd2332 || n == 12'd2336 || n == 12'd2340);
	end
endfunction

function automatic logic [11:0] vendor_size(input logic [3:0] flags, input logic mode2, input logic form2);
	begin
		case (flags[1:0])
		2'd2: vendor_size = 12'd2336;
		2'd3: vendor_size = 12'd2340;
		default: begin
			if (!flags[3]) vendor_size = flags[0] && mode2 ? 12'd2336 : 12'd2048;
			else if (flags[0] && form2) vendor_size = flags[2] ? 12'd2324 : 12'd2332;
			else vendor_size = flags[2] ? 12'd2048 : 12'd2056;
		end
		endcase
	end
endfunction

function automatic logic [11:0] descriptor_source(input logic [7:0] density, input logic [11:0] n);
	begin
		if (n == 12'd2340) descriptor_source = 12'd12;
		else if (density == 8'h81 && (n == 12'd2048 || n == 12'd2324)) descriptor_source = 12'd24;
		else descriptor_source = 12'd16;
	end
endfunction

// ---- bus states ----
localparam logic [3:0] F_FREE = 4'd0, F_SELW = 4'd1, F_SELD = 4'd2, F_CMD = 4'd3, F_EXEC = 4'd4,
                       F_DIN = 4'd5, F_DOUT = 4'd6, F_STAT = 4'd7, F_MIN = 4'd8, F_MOUT = 4'd9,
                       F_HOLD = 4'd10, F_ATN = 4'd11, F_DROP = 4'd12;
localparam logic [1:0] M_NONE = 2'd0, M_CMD = 2'd1, M_DOUT = 2'd2, M_STAT = 2'd3;

// phase lines {MSG, C/D, I/O}
localparam logic [2:0] L_DOUT = 3'b000, L_DIN = 3'b001, L_CMD = 3'b010, L_STAT = 3'b011,
                       L_MOUT = 3'b110, L_MIN = 3'b111;

// ---- replies ----
localparam logic [3:0] R_SENSE = 4'd1, R_INQ = 4'd2, R_MSENSE = 4'd3, R_CAP = 4'd4, R_HDR = 4'd5,
                       R_SUBCH = 4'd6, R_SUBQ = 4'd7, R_TOC = 4'd8, R_DE0 = 4'd9, R_DE1 = 4'd10,
                       R_DE2 = 4'd11, R_DE3 = 4'd12, R_SECT = 4'd13;

// ---- bus timing, crystal ticks ----
localparam logic [15:0] B_SEL    = 16'd9517;    // 562 us
localparam logic [15:0] B_CMD0   = 16'd1795;    // 106 us
localparam logic [15:0] B_CMD    = 16'd2290;    // 135 us
localparam logic [15:0] B_REPLY  = 16'd3751;    // 221 us
localparam logic [15:0] B_DIN    = 16'd2;       // 160 ns with the pipeline
localparam logic [15:0] B_SECGAP = 16'd11278;   // 666 us
// last data byte to status, and status to message, by command (medians)
localparam logic [15:0] B_E2S    = 16'd11025;   // 651 us, READ TOC and most replies
localparam logic [15:0] B_E2S_RD = 16'd19542;   // 1.15 ms after buffered sector data
localparam logic [15:0] B_E2S_SK = 16'd42337;   // 2.5 ms when the first sector came off the disc
localparam logic [15:0] B_E2S_RS = 16'd26943;   // 1.59 ms after sense data
localparam logic [15:0] B_E2S_DE = 16'd28213;   // 1.67 ms, NEC TOC
localparam logic [15:0] X_E2S_HD = 16'd559;     // 6.3 ms, READ HEADER, slow ticks
localparam logic [15:0] B_S2M    = 16'd21592;   // 1.28 ms
localparam logic [15:0] B_S2M_RD = 16'd3421;    // 202 us after READ
localparam logic [15:0] B_S2M_RS = 16'd7638;    // 451 us after REQUEST SENSE
localparam logic [15:0] B_S2M_LG = 16'd56262;   // 3.3 ms after READ TOC and MODE SELECT
localparam logic [15:0] B_FREE   = 16'd2540;    // 150 us
// command times, slow ticks
localparam logic [15:0] X_ERR    = 16'd88;      // 1.0 ms to CHECK CONDITION
localparam logic [15:0] X_TUR    = 16'd309;     // 3.5 ms
localparam logic [15:0] X_SENSE  = 16'd63;      // 0.71 ms
localparam logic [15:0] X_MSENSE = 16'd189;     // 2.1 ms
localparam logic [15:0] X_MSEL   = 16'd56;      // 0.63 ms
localparam logic [15:0] X_TOC0   = 16'd271;     // 3.1 ms, READ TOC header only
localparam logic [15:0] X_TOCN   = 16'd300;     // 3.4 ms, READ TOC from a track
localparam logic [15:0] X_DE0    = 16'd150;
localparam logic [15:0] X_DE2    = 16'd433;
localparam logic [15:0] X_DE3    = 16'd123;
localparam logic [15:0] X_AUDIO  = 16'd417;     // 4.7 ms
localparam logic [15:0] X_SEARCH = 16'd1447;    // 16.4 ms, AUDIO TRACK SEARCH at the pause point
localparam logic [15:0] X_RD     = 16'd112;     // 1.27 ms from a buffered sector
localparam logic [15:0] X_RESUME = 16'd488;     // 5.53 ms when the read-ahead had stopped
localparam logic [15:0] X_ATN    = 16'd441;     // 5 ms from the sector's end to MESSAGE OUT
localparam logic [15:0] X_ABORT  = 16'd476;     // 5.4 ms from ABORT to bus free

// ---- helpers ----

function automatic logic [6:0] bin(input logic [7:0] b);
	bin = {b[7:4], 3'b000} + {2'b00, b[7:4], 1'b0} + {3'd0, b[3:0]};
endfunction

function automatic logic [5:0] sec_bin(input logic [7:0] b);
	sec_bin = 6'(bin(b));
endfunction

function automatic logic bcd_ok(input logic [7:0] b);
	bcd_ok = b[7:4] <= 4'd9 && b[3:0] <= 4'd9;
endfunction

function automatic logic [7:0] bcd(input logic [6:0] bin7);
	logic [3:0] t;
	logic [6:0] r;
	begin
		r = bin7;
		t = 4'd0;
		if (r >= 7'd80) begin r = r - 7'd80; t = t + 4'd8; end
		if (r >= 7'd40) begin r = r - 7'd40; t = t + 4'd4; end
		if (r >= 7'd20) begin r = r - 7'd20; t = t + 4'd2; end
		if (r >= 7'd10) begin r = r - 7'd10; t = t + 4'd1; end
		bcd = {t, r[3:0]};
	end
endfunction

function automatic logic [7:0] be32(input logic [19:0] val, input logic [1:0] i);
	logic [31:0] w;
	begin
		w = {{12{val[19]}}, val};
		case (i)
		2'd0:    be32 = w[31:24];
		2'd1:    be32 = w[23:16];
		2'd2:    be32 = w[15:8];
		default: be32 = w[7:0];
		endcase
	end
endfunction


wire [7:0] op  = st.cdb[95:88];
wire [7:0] c1  = st.cdb[87:80];
wire [7:0] c2  = st.cdb[79:72];
wire [7:0] c3  = st.cdb[71:64];
wire [7:0] c4  = st.cdb[63:56];
wire [7:0] c5  = st.cdb[55:48];
wire [7:0] c6  = st.cdb[47:40];
wire [7:0] c7  = st.cdb[39:32];
wire [7:0] c8  = st.cdb[31:24];
wire [7:0] c9  = st.cdb[23:16];
wire [7:0] c10 = st.cdb[15:8];

wire [3:0] cdb_len = op[7:5] == 3'd0 ? 4'd6 : op[7:5] == 3'd5 ? 4'd12 : 4'd10;

wire verify_zero = st.mode.density == 8'h00 &&
                   ((op == 8'h2F && {c7, c8} == 16'd0) || (op == 8'hAF && {c6, c7, c8, c9} == 32'd0));
wire known = verify_zero || op == 8'h00 || op == 8'h01 || op == 8'h03 || op == 8'h08 || op == 8'h0B || op == 8'h0D ||
             op == 8'h12 || op == 8'h15 || op == 8'h1A || op == 8'h1B || op == 8'h1E || op == 8'h25 ||
             op == 8'h28 || op == 8'h2B || op == 8'h34 || op == 8'h42 || op == 8'h43 || op == 8'h44 ||
             op == 8'h45 || op == 8'h47 || op == 8'h48 || op == 8'h49 || op == 8'h4B || op == 8'hA5 ||
             op == 8'hA8 || op == 8'hA9 || op == 8'hD2 || (op >= 8'hD8 && op <= 8'hDE);
wire needs_disc = !(op == 8'h01 || op == 8'h03 || op == 8'h0D || op == 8'h12 || op == 8'h15 ||
                op == 8'h1A || op == 8'h1E);

// ---- mode pages ----
// MODE SENSE walks one image: 28 (0-5), 29 (6-8), 2A (9-12), 2B (13-15), 0E (16-31)
function automatic logic [7:0] page_byte(input logic [69:0] s, input logic [4:0] i, input logic [1:0] pc);
	logic [7:0] cur, msk, dfl;
	begin
		cur = 8'h00; msk = 8'h00; dfl = 8'h00;
		case (i)
		5'd0:  begin cur = 8'h28; msk = 8'h28; dfl = 8'h28; end
		5'd1:  begin cur = 8'h04; msk = 8'h04; dfl = 8'h04; end
		5'd2:  cur = s[63:56];
		5'd3:  cur = s[55:48];
		5'd4:  cur = s[47:40];
		5'd5:  cur = s[39:32];
		5'd6:  begin cur = 8'h29; msk = 8'h29; dfl = 8'h29; end
		5'd7:  begin cur = 8'h01; msk = 8'h01; dfl = 8'h01; end
		5'd8:  cur = s[31:24];
		5'd9:  begin cur = 8'h2A; msk = 8'h2A; dfl = 8'h2A; end
		5'd10: begin cur = 8'h02; msk = 8'h02; dfl = 8'h02; end
		5'd11: cur = s[23:16];
		5'd12: begin cur = s[15:8]; dfl = 8'h11; end
		5'd13: begin cur = 8'h2B; msk = 8'h2B; dfl = 8'h2B; end
		5'd14: begin cur = 8'h01; msk = 8'h01; dfl = 8'h01; end
		5'd15: cur = s[7:0];
		5'd16: begin cur = 8'h0E; msk = 8'h0E; dfl = 8'h0E; end
		5'd17: begin cur = 8'h0E; msk = 8'h0E; dfl = 8'h0E; end
		5'd18: begin cur = {5'd0, s[69], 2'd0}; msk = 8'h04; dfl = 8'h04; end
		5'd21: begin cur = {7'd0, s[68]}; msk = 8'h01; end
		5'd24: begin cur = {6'd0, s[67:66]}; msk = 8'h01; dfl = 8'h01; end
		5'd25: begin cur = 8'hFF; dfl = 8'hFF; end
		5'd26: begin cur = {6'd0, s[65:64]}; msk = 8'h02; dfl = 8'h02; end
		5'd27: begin cur = 8'hFF; dfl = 8'hFF; end
		default: ;
		endcase
		page_byte = pc == 2'd1 ? msk : pc == 2'd2 ? dfl : cur;
	end
endfunction

wire [69:0] modes = {st.mode.p0e_immed, st.mode.p0e_b3, st.mode.p0e_sel0, st.mode.p0e_sel1, st.mode.p28, st.mode.p29, st.mode.p2a, st.mode.p2b};
wire [5:0] ms_page = c2[5:0];
wire [1:0] ms_pc   = c2[7:6];
wire       ms_dbd  = c1[3];
reg  [4:0] ms_lo;
reg  [5:0] ms_n;
always_comb begin
	ms_lo = 5'd0;
	ms_n  = 6'd0;
	case (ms_page)
	6'h3F: begin ms_lo = 5'd0;  ms_n = 6'd32; end
	6'h28: begin ms_lo = 5'd0;  ms_n = 6'd6;  end
	6'h29: begin ms_lo = 5'd6;  ms_n = 6'd3;  end
	6'h2A: begin ms_lo = 5'd9;  ms_n = 6'd4;  end
	6'h2B: begin ms_lo = 5'd13; ms_n = 6'd3;  end
	6'h0E: begin ms_lo = 5'd16; ms_n = 6'd16; end
	default: ;
	endcase
end
wire [5:0] ms_hdr = ms_dbd ? 6'd4 : 6'd12;

// ---- reply bytes that need no TOC entry ----

wire [7:0]  q_ca   = q_now[79:72];
wire [7:0]  q_tno  = q_now[71:64];
wire [7:0]  q_idx  = q_now[63:56];
wire [23:0] q_rel  = q_now[55:32];
wire [23:0] q_abs  = q_now[23:0];
wire [7:0]  sc_st  = m_play == PS_PLAY ? 8'h11 : m_play == PS_STOP ? 8'h13 : 8'h12;
wire        want_q = c2[6];
wire [7:0]  sc_fmt = c3;
wire [5:0]  sc_len = !want_q ? 6'd4 : sc_fmt == 8'd0 ? 6'd48 : sc_fmt == 8'd1 ? 6'd16 : 6'd24;
wire [18:0] last_lba = m_leadout - 19'd151;

function automatic logic [7:0] inq_byte(input logic [5:0] i);
	case (i)
	6'd0:  inq_byte = 8'h05;
	6'd1:  inq_byte = 8'h80;
	6'd2:  inq_byte = 8'h02;
	6'd4:  inq_byte = 8'h1F;
	6'd8:  inq_byte = "N";
	6'd9:  inq_byte = "E";
	6'd10: inq_byte = "C";
	6'd16: inq_byte = "C";
	6'd17: inq_byte = "D";
	6'd18: inq_byte = "-";
	6'd19: inq_byte = "R";
	6'd20: inq_byte = "O";
	6'd21: inq_byte = "M";
	6'd23: inq_byte = "D";
	6'd24: inq_byte = "R";
	6'd25: inq_byte = "I";
	6'd26: inq_byte = "V";
	6'd27: inq_byte = "E";
	6'd28: inq_byte = ":";
	6'd29: inq_byte = "F";
	6'd30: inq_byte = "X";
	6'd32: inq_byte = "1";
	6'd33: inq_byte = ".";
	6'd34: inq_byte = "0";
	// last byte: a space on the PC-FX (assumed), 'E' on the FX board and FXGA
	6'd11, 6'd12, 6'd13, 6'd14, 6'd15, 6'd22, 6'd31, 6'd35: inq_byte = 8'h20;
	default: inq_byte = 8'h00;
	endcase
endfunction

wire [11:0] skc = sense_kc(st.sense);

// one BCD and one binary converter shared by all reply bytes
reg [6:0] bcd_in;
reg [7:0] bin_in;
wire [7:0] bcd_o = bcd(bcd_in);
wire [6:0] bin_o = bin(bin_in);

reg [7:0] hb;    // reply byte st.pidx of the replies without TOC entries
always_comb begin
	logic [5:0] i;
	logic [4:0] r;
	i  = st.pidx[5:0];
	r  = 5'(i - ms_hdr) + ms_lo;
	hb = 8'h00;
	case (st.rt)
	R_SENSE: case (i)
		6'd0:        hb = 8'h70;
		6'd2:        hb = {4'd0, skc[11:8]};
		6'd7:        hb = 8'h0A;
		6'd8:        hb = 8'h10;
		6'd9, 6'd12: hb = skc[7:0];
		default: ;
		endcase
	R_INQ: hb = (i == 6'd0 && c1[7:5] != 3'd0) ? 8'h7F : inq_byte(i);
	R_MSENSE: begin
		if (ms_page == 6'd0)
			hb = i == 6'd0 ? 8'h09 : i == 6'd2 ? 8'h80 : i == 6'd4 ? st.mode.vflags :
			     i == 6'd5 ? st.mode.vrecovery : i == 6'd9 ? {4'd0, st.mode.vretry} : 8'h00;
		else if (i == 6'd0)
			hb = 8'(st.len - 11'd1);
		else if (i == 6'd3)
			hb = ms_dbd ? 8'h00 : 8'h08;
		else if (i >= ms_hdr)
			hb = page_byte(modes, r, ms_pc);
		else
			hb = i == 6'd4 ? (ms_pc == 2'd0 ? st.mode.density : 8'h00) : i == 6'd7 ? 8'h6E :
			     i == 6'd10 ? (ms_pc == 2'd0 ? {4'd0, st.mode.block_len[11:8]} : 8'h08) :
			     i == 6'd11 && ms_pc == 2'd0 ? st.mode.block_len[7:0] : 8'h00;
	end
	R_CAP: case (i)
		6'd0, 6'd1, 6'd2, 6'd3: hb = be32({1'b0, last_lba}, i[1:0]);
		6'd6:                   hb = {4'd0, capacity_len[11:8]};
		6'd7:                   hb = capacity_len[7:0];
		default: ;
		endcase
	R_HDR: case (i)
		6'd0: hb = st.e_c;
		6'd4, 6'd5, 6'd6, 6'd7:
			if (!c1[1])         hb = be32(st.xr, i[1:0]);
			else if (i != 6'd4) hb = {1'b0, bin_o};
		default: ;
		endcase
	R_SUBCH: case (i)
		6'd1: hb = sc_st;
		6'd3: hb = {2'd0, sc_len} - 8'd4;
		6'd4: hb = want_q ? sc_fmt : 8'h00;
		6'd5: hb = sc_fmt == 8'd2 ? 8'h00 : {q_ca[3:0], q_ca[7:4]};
		6'd6: hb = sc_fmt == 8'd3 ? c6 : sc_fmt == 8'd2 ? 8'h00 : {1'b0, bin_o};
		6'd7: hb = sc_fmt <= 8'd1 ? {1'b0, bin_o} : 8'h00;
		6'd8, 6'd9, 6'd10, 6'd11:
			if (sc_fmt > 8'd1)   hb = 8'h00;
			else if (!c1[1])     hb = be32(st.xr, i[1:0]);
			else if (i != 6'd8)  hb = {1'b0, bin_o};
		6'd12, 6'd13, 6'd14, 6'd15:
			if (sc_fmt > 8'd1)   hb = 8'h00;
			else if (!c1[1])     hb = be32(st.xr, i[1:0]);
			else if (i != 6'd12) hb = {1'b0, bin_o};
		default: ;
		endcase
	R_SUBQ: case (i)
		6'd0: hb = {6'd0, m_play};
		6'd1: hb = q_ca;
		6'd2: hb = q_tno;
		6'd3: hb = q_idx;
		6'd4: hb = q_rel[23:16];
		6'd5: hb = q_rel[15:8];
		6'd6: hb = q_rel[7:0];
		6'd7: hb = q_abs[23:16];
		6'd8: hb = q_abs[15:8];
		6'd9: hb = q_abs[7:0];
		default: ;
		endcase
	R_DE0: hb = i <= 6'd1 ? bcd_o : 8'h00;
	R_DE1, R_DE2: case (i)
		6'd0, 6'd1, 6'd2: hb = bcd_o;
		6'd3: hb = st.rt == R_DE2 ? st.e_c : 8'h00;
		default: ;
		endcase
	default: ;
	endcase
end

// ---- READ TOC and the NEC TOC, entry by entry ----

// NEC TOC entry kinds: A0 first track, A1 last track, A2 lead-out, else a track
wire       de_all = c2 == 8'h00;
wire [1:0] de_kind = (de_all && st.ent < 7'd3) ? st.ent[1:0] : c2 == 8'hA0 ? 2'd0 : c2 == 8'hA1 ? 2'd1 :
                     c2 == 8'hA2 ? 2'd2 : 2'd3;
wire [6:0] toc_start = c6 == 8'd0 ? 7'd1 : c6 == 8'hAA ? toc_last + 7'd1 : c6[6:0];
wire [6:0] trk_43    = toc_start + st.ent > toc_last ? 7'd100 : toc_start + st.ent;
wire [6:0] trk_de    = de_kind == 2'd0 ? toc_first : de_kind == 2'd1 ? toc_last :
                       de_kind == 2'd2 ? 7'd100 : toc_first + st.ent - 7'd3;
wire       toc_rep   = st.rt == R_TOC || st.rt == R_DE3;
wire [3:0] ent_last  = st.rt == R_TOC ? 4'd7 : 4'd9;
wire       in_ent    = st.rt == R_TOC ? st.pidx >= 11'd4 : st.pidx >= 11'd2;
wire [10:0] ent_bytes = {1'b0, toc_last - toc_start + 7'd2, 3'b000};
wire [10:0] lenm2    = st.rt == R_TOC ? 11'd2 + ent_bytes : st.len - 11'd2;
wire [6:0] lin       = st.ent >= 7'd75 ? st.ent - 7'd75 : st.ent;

reg [7:0] tb;
always_comb begin
	tb = 8'h00;
	if (st.rt == R_TOC) begin
		if (!in_ent) case (st.pidx[1:0])
			2'd0:    tb = {5'd0, lenm2[10:8]};
			2'd1:    tb = lenm2[7:0];
			2'd2:    tb = {1'b0, toc_first};
			default: tb = {1'b0, toc_last};
			endcase
		else case (st.fld)
			4'd1: tb = {4'h1, st.e_c[3:0]};
			4'd2: tb = st.trk == 7'd100 ? 8'hAA : {1'b0, st.trk};
			4'd4: tb = c1[1] ? 8'h00 : be32(st.xr, 2'd0);
			4'd5: tb = c1[1] ? st.e_m : be32(st.xr, 2'd1);
			4'd6: tb = c1[1] ? st.e_s : be32(st.xr, 2'd2);
			4'd7: tb = c1[1] ? st.e_f : be32(st.xr, 2'd3);
			default: ;
			endcase
	end else begin
		if (!in_ent) tb = st.pidx[0] ? lenm2[7:0] : {5'd0, lenm2[10:8]};
		else if (c2 != 8'hB0) case (st.fld)
			4'd0: tb = {st.e_c[3:0], 4'h0};
			4'd2: tb = de_kind == 2'd0 ? 8'hA0 : de_kind == 2'd1 ? 8'hA1 : de_kind == 2'd2 ? 8'hA2 : bcd_o;
			4'd4: tb = st.ent >= 7'd75 ? 8'h01 : 8'h00;
			4'd5, 4'd7: tb = bcd_o;
			4'd8, 4'd9: tb = de_kind[1] ? bcd_o : 8'h00;
			default: ;
			endcase
	end
end

always_comb begin
	logic [3:0] k;
	k = st.rt == R_DE3 ? st.fld : st.pidx[3:0];
	bcd_in = st.e_f[6:0];
	if (st.rt == R_DE0)
		bcd_in = st.pidx[0] ? toc_last : toc_first;
	else if (k == 4'd0 || (st.rt == R_DE3 && k == 4'd7 && de_kind[1]))
		bcd_in = st.e_m[6:0];
	else if (st.rt == R_DE3 && k == 4'd5)
		bcd_in = lin;
	else if (st.rt == R_DE3 && (k == 4'd2 || k == 4'd7))
		bcd_in = st.trk;
	else if (k == 4'd1 || k == 4'd8)
		bcd_in = st.e_s[6:0];

	case (st.pidx[3:0])
	4'd5:          bin_in = st.e_m;
	4'd6:          bin_in = st.rt == R_SUBCH ? q_tno : st.e_s;
	4'd7:          bin_in = st.rt == R_SUBCH ? q_idx : st.e_f;
	4'd9:          bin_in = q_abs[23:16];
	4'd10:         bin_in = q_abs[15:8];
	4'd11:         bin_in = q_abs[7:0];
	4'd13:         bin_in = q_rel[23:16];
	4'd14:         bin_in = q_rel[15:8];
	4'd15:         bin_in = q_rel[7:0];
	default:       bin_in = st.e_f;
	endcase
end

// reply length: what the initiator allows, or the whole reply if shorter
reg [15:0] r_alloc;
reg [10:0] r_real;
always_comb begin
	r_alloc = {c7, c8};
	r_real  = 11'd8;
	case (op)
	8'h03: begin r_alloc = c4 == 8'd0 ? 16'd4 : {8'd0, c4}; r_real = 11'd18; end
	8'h12: begin r_alloc = {8'd0, c4}; r_real = 11'd36; end
	8'h1A: begin r_alloc = {8'd0, c4}; r_real = ms_page == 6'd0 ? 11'd10 : {5'd0, ms_hdr} + {5'd0, ms_n}; end
	8'h42: r_real = {5'd0, sc_len};
	8'h43: r_real = 11'd4 + ent_bytes;
	8'hDD: begin r_alloc = {8'd0, c1}; r_real = 11'd10; end
	default: ;
	endcase
end
wire [10:0] r_len = r_alloc < {5'd0, r_real} ? r_alloc[10:0] : r_real;

// ---- the controller ----

localparam logic [4:0] X_START = 5'd0, X_ADDR = 5'd1, X_ENT = 5'd2, X_CONV = 5'd3, X_ADONE = 5'd4,
                       X_MECH = 5'd5, X_MWAIT = 5'd6, X_RWAIT = 5'd7, X_HDR = 5'd8, X_HCONV = 5'd9,
                       X_REPLY = 5'd10, X_STAT = 5'd11, X_SUBC = 5'd12, X_ADD2 = 5'd13, X_DOUT = 5'd14,
                       X_FETCH = 5'd15;

localparam logic [2:0] A_LBA = 3'd0, A_BMSF = 3'd1, A_BTRK = 3'd2, A_MSF = 3'd3, A_TRK = 3'd4, A_BAD = 3'd5;

function automatic logic [2:0] typed(input logic [1:0] t);
	typed = t == 2'd0 ? A_LBA : t == 2'd1 ? A_BMSF : t == 2'd2 ? A_BTRK : A_BAD;
endfunction

// what the address unit converts: kind and four bytes, start or end address
reg  [2:0] au_kind;
reg [31:0] au_b;
always_comb begin
	au_kind = A_LBA;
	au_b    = {c2, c3, c4, c5};
	case (op)
	8'h08, 8'h0B: au_b = {11'd0, c1[4:0], c2, c3};
	8'h28, 8'h2B, 8'hD2, 8'hD8, 8'hD9: au_kind = typed(c9[7:6]);
	8'h47: begin au_kind = A_MSF; au_b = st.second ? {c6, c7, c8, 8'd0} : {c3, c4, c5, 8'd0}; end
	8'h48: begin au_kind = A_TRK; au_b = {st.second ? c7 : c4, 24'd0}; end
	8'h49: begin au_kind = A_TRK; au_b = {c6, 24'd0}; end
	8'hA9: begin au_kind = A_TRK; au_b = {c10, 24'd0}; end
	default: ;
	endcase
end

wire [6:0]  au_tr  = au_kind == A_BTRK ? bin(au_b[31:24]) : au_b[30:24];
wire [18:0] rel19  = {c3[2:0], c4, c5};
wire [18:0] tgt_sum;
wire [18:0] len19  = op == 8'hA5 || op == 8'hA9 ? {c7[2:0], c8, c9} : {3'd0, c7, c8};
wire [23:0] rcount = op == 8'h08 ? (c4 == 8'd0 ? 24'd256 : {16'd0, c4}) :
                     op == 8'hA8 ? {c7, c8, c9} : {8'd0, c7, c8};
wire        is_read  = op == 8'h08 || op == 8'h28 || op == 8'hA8;
wire        is_seek  = op == 8'h0B || op == 8'h2B || op == 8'h34;
wire        is_play  = op == 8'h45 || op == 8'h47 || op == 8'h48 || op == 8'h49 || op == 8'hA5 || op == 8'hA9;
wire        two_addr = op == 8'h47 || op == 8'h48;

wire [15:0] ent_adr = TOC_BASE + {7'd0, st.trk - 7'd1, 2'b00} + {13'd0, st.fs};
wire [15:0] sec_adr = p_base + {4'd0, st.sect_src} + {4'd0, st.sk};
wire [11:0] capacity_len = st.mode.custom ? st.mode.block_len : vendor_size({st.mode.vflags[7:6], st.mode.vflags[1:0]}, 1'b0, 1'b0);
wire [11:0] custom_origin = st.mode.density == 8'h00 && st.mode.block_len == 12'd2048 ? 12'd16 : 12'd12;
wire [11:0] ml_origin = st.ml_mode.density == 8'h00 && st.ml_mode.block_len == 12'd2048 ? 12'd16 : 12'd12;

wire tick = st.dslow ? ce_t : ce_x;
wire read_error = st.rt == R_SECT && (m_err != E_NONE || (st.xs == X_STAT && st.err != E_NONE));
assign tgt_sum = st.tgt + (st.xs == X_ADD2 ? len19 : rel19);

wire [15:0] e2s = st.rt == R_SECT ? (st.waited ? B_E2S_SK : B_E2S_RD) : op == 8'h03 ? B_E2S_RS : op == 8'h44 ? X_E2S_HD :
                  op == 8'hDE ? B_E2S_DE : B_E2S;
wire [15:0] s2m = st.dbo == 8'h02 ? B_S2M_RS : is_read ? B_S2M_RD : op == 8'h03 ? B_S2M_RS :
                  (op == 8'h43 || op == 8'h15) ? B_S2M_LG : B_S2M;
wire dly0 = st.dly == 16'd0;

always_ff @(posedge clk) begin
	if (reset) begin
		st           <= '0;
		st.ua        <= 2'd1;
		st.mode      <= mode_defaults();
		mr_rq        <= 1'b0;
	end else if (SAVESTATE && ss_shift) begin
		st    <= scsi_t'({st[SCSI_BITS-2:0], ss_si});
		mr_rq <= 1'b0;
	end else if (ce) begin
		// The receiver consumes each event on this enabled edge.
		st.m_go    <= 1'b0;
		st.p_next  <= 1'b0;
		st.bus_rst <= 1'b0;
		st.rst_d   <= rst;
		if (tick && !dly0) st.dly <= st.dly - 16'd1;

		// m2f: minutes and seconds counted onto the frames
		if (st.cv_busy) begin
			if (st.cv_m != 7'd0) begin
				st.cv   <= st.cv + 19'd4500;
				st.cv_m <= st.cv_m - 7'd1;
			end else if (st.cv_s != 6'd0) begin
				st.cv   <= st.cv + 19'd75;
				st.cv_s <= st.cv_s - 6'd1;
			end else begin
				st.cv_busy <= 1'b0;
			end
		end

		if (m_change) st.ua <= 2'd2;

		if (rst) begin
			// SCSI bus reset: back to power-on settings
			st.ph        <= F_FREE;
			st.hs        <= 2'd0;
			st.ua        <= 2'd1;
			st.mode       <= mode_defaults();
			st.ml_mode    <= '0;
			st.msg_return <= M_NONE;
			mr_rq         <= 1'b0;
			st.bus_rst   <= !st.rst_d;
		end else case (st.ph)

		// ---- selection: our ID bit with SEL ----
		F_FREE: begin
			st.hs <= 2'd0;
			if (sel && db_i[2]) begin
				st.ph    <= F_SELW;
				st.dly   <= B_SEL;
				st.dslow <= 1'b0;
			end
		end
		F_SELW: begin
			if (!sel)     st.ph <= F_FREE;
			else if (dly0) st.ph <= F_SELD;
		end
		F_SELD: if (!sel) begin
			st.ph    <= atn ? F_MOUT : F_CMD;
			st.pl    <= atn ? L_MOUT : L_CMD;
			st.hs    <= 2'd0;
			st.cdb_n <= 4'd0;
			st.dly   <= B_CMD0;
			st.msg_return <= atn ? M_CMD : M_NONE;
		end

		// ---- command bytes ----
		F_CMD: begin
			if (st.hs == 2'd0) begin
				if (atn) begin
					st.ph <= F_MOUT;
					st.pl <= L_MOUT;
					st.msg_return <= M_CMD;
				end else if (dly0) begin
					st.hs <= 2'd1;
				end
			end else if (st.hs == 2'd1) begin
				if (ack) begin
					case (st.cdb_n)
					4'd0:  st.cdb[95:88] <= db_i;
					4'd1:  st.cdb[87:80] <= db_i;
					4'd2:  st.cdb[79:72] <= db_i;
					4'd3:  st.cdb[71:64] <= db_i;
					4'd4:  st.cdb[63:56] <= db_i;
					4'd5:  st.cdb[55:48] <= db_i;
					4'd6:  st.cdb[47:40] <= db_i;
					4'd7:  st.cdb[39:32] <= db_i;
					4'd8:  st.cdb[31:24] <= db_i;
					4'd9:  st.cdb[23:16] <= db_i;
					4'd10: st.cdb[15:8]  <= db_i;
					default: st.cdb[7:0] <= db_i;
					endcase
					st.cdb_n <= st.cdb_n + 4'd1;
					st.hs    <= 2'd2;
				end
			end else if (!ack) begin
				st.hs <= 2'd0;
				if (st.cdb_n == cdb_len) begin
					st.ph <= F_EXEC;
					st.xs <= X_START;
				end else begin
					st.dly <= B_CMD;
				end
			end
		end

		// ---- command execution ----
		F_EXEC: begin
			case (st.xs)
			X_START: if (!(q_hold && q_busy)) begin
				st.msg_return <= M_NONE;
				st.dslow  <= 1'b1;
				st.second <= 1'b0;
				st.err    <= E_NONE;
				st.dly    <= X_ERR;
				st.xs     <= X_STAT;
				if (op != 8'h03 && op != 8'h0D) st.sense <= E_NONE;
				if (st.ua != 2'd0 && op != 8'h03 && op != 8'h0D && op != 8'h12) begin
					st.err   <= st.ua == 2'd1 ? E_RESET : E_CHANGED;
					st.sense <= st.ua == 2'd1 ? E_RESET : E_CHANGED;
					st.ua    <= 2'd0;
				end else if (!known) begin
					st.err   <= E_BAD_CMD;
					st.sense <= E_BAD_CMD;
				end else if (needs_disc && m_nodisc) begin
					st.err   <= E_NO_DISC;
					st.sense <= E_NO_DISC;
				end else if (needs_disc && !m_ready) begin
					st.err   <= E_NOT_READY;
					st.sense <= E_NOT_READY;
				end else case (op)
				8'h00: st.dly <= X_TUR;
				8'h01: begin
					st.mop <= MO_REZERO;
					st.xs  <= X_MECH;
				end
				8'h2F, 8'hAF: st.xs <= X_STAT;
				8'h03: begin
					if (st.ua != 2'd0) begin
						st.sense <= st.ua == 2'd1 ? E_RESET : E_CHANGED;
						st.ua    <= 2'd0;
					end
					st.rt  <= R_SENSE;
					st.len <= r_len;
					st.dly <= X_SENSE;
					st.xs  <= X_REPLY;
				end
				8'h12: begin
					st.rt  <= R_INQ;
					st.len <= r_len;
					st.dly <= X_SENSE;
					st.xs  <= c4 == 8'd0 ? X_STAT : X_REPLY;
				end
				8'h15: begin
					st.dly     <= X_MSEL;
					st.ml_mode <= st.mode;
					st.ml_left <= !c1[4] && c4 != 8'd0 ? 8'd10 : c4;
					st.ml_st   <= 3'd0;
					st.ml_n    <= 8'd0;
					st.ml_i    <= 8'd0;
					st.ml_err  <= 1'b0;
					st.ml_bd   <= 1'b0;
					st.xs      <= c4 == 8'd0 ? X_STAT : X_DOUT;
					if (!c1[4] && c4 == 8'd0) begin
						st.mode.vflags     <= 8'd0;
						st.mode.vrecovery  <= 8'd0;
						st.mode.vretry     <= 4'hF;
						st.mode.custom     <= 1'b0;
						st.mode.p28_applied <= 1'b0;
					end
				end
				8'h1A: begin
					st.rt  <= R_MSENSE;
					st.dly <= X_MSENSE;
					st.len <= r_len;
					if (ms_pc == 2'd3 || (ms_page == 6'd0 && (ms_dbd || ms_pc != 2'd0)) || (ms_page != 6'd0 && ms_n == 6'd0)) begin
						st.err   <= E_BAD_PARAM;
						st.sense <= E_BAD_PARAM;
						st.dly   <= X_ERR;
					end else begin
						st.xs <= c4 == 8'd0 ? X_STAT : X_REPLY;
					end
				end
				8'h1E, 8'hDC: begin
					st.err   <= E_BAD_CDB;
					st.sense <= E_BAD_CDB;
				end
				8'h1B: begin
					if (c4[1]) begin
						st.err   <= E_BAD_CDB;
						st.sense <= E_BAD_CDB;
					end else begin
						st.mop <= MO_STOP;
						st.xs  <= c4[0] ? X_STAT : X_MECH;
					end
				end
				8'h25: begin
					st.rt  <= R_CAP;
					st.len <= 11'd8;
					st.mop <= MO_STOP;
					st.dly <= X_MSENSE;
					st.xs  <= X_MECH;
				end
				8'h42: begin
					st.rt  <= R_SUBCH;
					st.len <= r_len;
					st.dly <= X_MSENSE;
					if ((c2 & 8'hBF) != 8'd0 || sc_fmt > 8'd3 ||
					    (sc_fmt == 8'd3 && (c6[6:0] < toc_first || c6 > {1'b0, toc_last}))) begin
						st.err   <= E_BAD_PARAM;
						st.sense <= E_BAD_PARAM;
						st.dly   <= X_ERR;
					end else if ({c7, c8} == 16'd0) begin
						st.xs <= X_STAT;
					end else if (want_q && sc_fmt <= 8'd1 && !c1[1]) begin
						// LBA form: abs - 150, then rel
						st.cv      <= {12'd0, bin(q_abs[7:0])};
						st.cv_m    <= bin(q_abs[23:16]);
						st.cv_s    <= sec_bin(q_abs[15:8]);
						st.cv_busy <= 1'b1;
						st.xs      <= X_SUBC;
					end else begin
						st.xs <= X_REPLY;
					end
				end
				8'h43: begin
					st.rt  <= R_TOC;
					st.len <= r_len;
					st.dly <= {c7, c8} <= 16'd4 ? X_TOC0 : X_TOCN;
					if ((c1 & 8'hFD) != 8'd0 || c2 != 8'd0 || c3 != 8'd0 || c4 != 8'd0 || c5 != 8'd0 || c9 != 8'd0 ||
					    (c6 > {1'b0, toc_last} && c6 != 8'hAA)) begin
						st.err   <= E_BAD_PARAM;
						st.sense <= E_BAD_PARAM;
						st.dly   <= X_ERR;
					end else begin
						st.xs <= {c7, c8} == 16'd0 ? X_STAT : X_REPLY;
					end
				end
				8'h44: begin
					st.rt  <= R_HDR;
					st.len <= r_len;
					st.mop <= MO_READ;
					st.dly <= X_RD;
					st.xs  <= {c7, c8} == 16'd0 ? X_STAT : X_ADDR;
				end
				8'h4B, 8'hDA: begin
					st.dly <= X_AUDIO;
					st.mop <= (op == 8'h4B && c8[0]) ? MO_RESUME : MO_PAUSE;
					if (m_play == PS_STOP) begin
						st.err   <= E_NOT_PLAY;
						st.sense <= E_NOT_PLAY;
						st.dly   <= X_ERR;
					end else begin
						st.xs <= X_MECH;
					end
				end
				8'hD8: begin
					st.mop   <= MO_SEARCH;
					st.mmode <= {2'd0, c1[0]};
					st.dly   <= X_SEARCH;
					st.xs    <= X_ADDR;
				end
				8'hD9: begin
					st.mop   <= MO_SETEND;
					st.mmode <= c1[2:0];
					st.dly   <= X_AUDIO;
					if (m_play == PS_STOP) begin
						st.err   <= E_NOT_PLAY;
						st.sense <= E_NOT_PLAY;
						st.dly   <= X_ERR;
					end else begin
						st.second <= 1'b1;
						st.xs     <= X_ADDR;
					end
				end
				8'hDD: begin
					st.rt  <= R_SUBQ;
					st.len <= r_len;
					st.dly <= X_SENSE;
					st.xs  <= c1 == 8'd0 ? X_STAT : X_REPLY;
				end
				8'hDE: begin
					st.len <= 11'd4;
					st.dly <= X_DE0;
					case (c1[1:0])
					2'd0: begin st.rt <= R_DE0; st.xs <= X_REPLY; end
					2'd1: begin st.rt <= R_DE1; st.trk <= 7'd100; st.fs <= 3'd0; st.xs <= X_FETCH; end
					2'd2: begin
						st.rt  <= R_DE2;
						st.dly <= X_DE2;
						st.trk <= bin(c2);
						st.fs  <= 3'd0;
						if (!bcd_ok(c2) || bin(c2) < toc_first || bin(c2) > toc_last) begin
							st.err   <= E_BAD_ADDR;
							st.sense <= E_BAD_ADDR;
							st.dly   <= X_ERR;
						end else begin
							st.xs <= X_FETCH;
						end
					end
					default: begin
						st.rt  <= R_DE3;
						st.dly <= X_DE3;
						st.len <= c2 == 8'hB0 ? 11'd22 : c2 == 8'h00 ?
						          11'd2 + {1'b0, toc_last - toc_first + 7'd4, 3'd0} + {2'd0, toc_last - toc_first + 7'd4, 1'b0} : 11'd12;
						if (c2 != 8'h00 && c2 != 8'hA0 && c2 != 8'hA1 && c2 != 8'hA2 && c2 != 8'hB0) begin
							st.err   <= E_BAD_ADDR;
							st.sense <= E_BAD_ADDR;
							st.dly   <= X_ERR;
						end else begin
							st.xs <= X_REPLY;
						end
					end
					endcase
				end
				default: begin
					if ((op == 8'h28 && c1 != 8'd0) || (op == 8'hA8 && c6 != 8'd0)) begin
						st.err   <= E_BAD_PARAM;
						st.sense <= E_BAD_PARAM;
					end else begin
						// reads, seeks and plays go through the address unit
						st.mop   <= is_read ? MO_READ : is_seek ? MO_SEEK : MO_PLAY;
						st.mmode <= 3'd3;
						st.dly   <= is_read || is_seek ? X_RD : X_AUDIO;
						st.scnt  <= rcount;
						if (is_read || is_seek || is_play) st.xs <= X_ADDR;
					end
				end
				endcase
			end

			// ---- address unit ----
			X_ADDR: begin
				case (au_kind)
				A_LBA: begin
					if (au_b[31:19] != 13'd0 || au_b[18:0] > 19'h7FF68) begin
						if (st.second) st.tgt2 <= 19'h7FFFF; else st.tgt <= 19'h7FFFF;
					end else begin
						if (st.second) st.tgt2 <= au_b[18:0] + 19'd150; else st.tgt <= au_b[18:0] + 19'd150;
					end
					st.xs <= X_ADONE;
				end
				A_BMSF, A_MSF: begin
					if (au_kind == A_BMSF && !(bcd_ok(au_b[31:24]) && bcd_ok(au_b[23:16]) && bcd_ok(au_b[15:8]))) begin
						st.err   <= E_BAD_PARAM;
						st.sense <= E_BAD_PARAM;
						st.dly   <= X_ERR;
						st.xs    <= X_STAT;
					end else begin
						st.cv      <= {12'd0, au_kind == A_BMSF ? bin(au_b[15:8]) : au_b[14:8]};
						st.cv_m    <= au_kind == A_BMSF ? bin(au_b[31:24]) : au_b[30:24];
						st.cv_s    <= au_kind == A_BMSF ? sec_bin(au_b[23:16]) : au_b[21:16];
						st.cv_busy <= 1'b1;
						st.xs      <= X_CONV;
					end
				end
				A_BTRK, A_TRK: begin
					st.fs <= 3'd0;
					if ((au_kind == A_BTRK && !bcd_ok(au_b[31:24])) || au_tr == 7'd0 || au_tr < toc_first) begin
						st.err   <= E_BAD_PARAM;
						st.sense <= E_BAD_PARAM;
						st.dly   <= X_ERR;
						st.xs    <= X_STAT;
					end else if (au_tr == toc_last + 7'd1 || (op == 8'h48 && st.second && au_tr > toc_last)) begin
						st.trk <= 7'd100;
						st.xs  <= X_ENT;
					end else if (au_tr > toc_last) begin
						st.err   <= E_EOV;
						st.sense <= E_EOV;
						st.dly   <= X_ERR;
						st.xs    <= X_STAT;
					end else begin
						st.trk <= au_tr;
						st.xs  <= X_ENT;
					end
				end
				default: begin
					st.err   <= E_BAD_PARAM;
					st.sense <= E_BAD_PARAM;
					st.dly   <= X_ERR;
					st.xs    <= X_STAT;
				end
				endcase
			end
			X_ENT: begin
				if (!mr_rq) begin
					mr_rq   <= 1'b1;
					mr_addr <= ent_adr;
				end else if (mr_ack) begin
					mr_rq <= 1'b0;
					st.fs <= st.fs + 3'd1;
					case (st.fs)
					3'd0: st.cv <= {12'd0, mr_q[6:0]};
					3'd1: st.cv_s <= mr_q[5:0];
					default: begin
						st.cv_m    <= mr_q[6:0];
						st.cv_busy <= 1'b1;
						st.xs      <= X_CONV;
					end
					endcase
				end
			end
			X_CONV: if (!st.cv_busy) begin
				if (st.second) st.tgt2 <= st.cv; else st.tgt <= st.cv;
				st.xs <= X_ADONE;
			end
			X_ADONE: begin
				if (two_addr && !st.second) begin
					st.second <= 1'b1;
					st.xs     <= X_ADDR;
				end else begin
					if (op == 8'h49 || op == 8'hA9) st.tgt <= tgt_sum;
					if (op == 8'h47 && st.tgt > st.tgt2) begin
						st.err   <= E_BAD_ADDR;
						st.sense <= E_BAD_ADDR;
						st.dly   <= X_ERR;
						st.xs    <= X_STAT;
					end else if (op == 8'hD2) begin
						st.xs <= X_STAT;
					end else begin
						st.xs <= op == 8'h45 || op == 8'hA5 || op == 8'h49 || op == 8'hA9 ? X_ADD2 : X_MECH;
					end
				end
			end
			// the end of a play given as a length
			X_ADD2: begin
				st.tgt2 <= tgt_sum;
				st.xs   <= X_MECH;
			end

			// ---- mechanism ----
			X_MECH: begin
				st.m_go <= 1'b1;
				st.xs <= X_MWAIT;
			end
			X_MWAIT: if (m_done) begin
				if (m_err != E_NONE) begin
					st.err   <= op == 8'h44 ? (m_err == E_NOT_DATA ? E_HEADER : E_BAD_PARAM) : m_err;
					st.sense <= op == 8'h44 ? (m_err == E_NOT_DATA ? E_HEADER : E_BAD_PARAM) : m_err;
					st.xs    <= X_STAT;
				end else if (is_read || op == 8'h44) begin
					if (st.scnt == 24'd0 && is_read) begin
						st.xs <= X_STAT;
					end else begin
						if (p_slow) st.dly <= X_RESUME;
						st.waited <= 1'b0;
						st.xs <= X_RWAIT;
					end
				end else if (op == 8'h25) begin
					st.xs <= X_REPLY;
				end else if (op == 8'hD9 && st.mmode == 3'd5) begin
					st.ph <= F_HOLD;
				end else begin
					st.xs <= X_STAT;
				end
			end
			// the first sector: the controller's time runs from when it is in
			X_RWAIT: begin
				if (m_err != E_NONE) begin
					st.err   <= m_err;
					st.sense <= m_err;
					st.xs    <= X_STAT;
				end else if (p_eov) begin
					st.err   <= E_EOV;
					st.sense <= E_EOV;
					st.xs    <= X_STAT;
				end else if (!p_ready) begin
					st.dly    <= X_RD;
					st.waited <= 1'b1;
				end else if (dly0) begin
					if (op == 8'h44) begin
						st.fs <= 3'd0;
						st.xs <= X_HDR;
					end else begin
						st.ph     <= F_DIN;
						st.pl     <= L_DIN;
						st.hs     <= 2'd0;
						st.rt     <= R_SECT;
						st.sec_init <= 2'd0;
						st.sfirst   <= 1'b1;
						st.nb_last  <= 1'b0;
						st.db_last  <= 1'b0;
						st.nbv    <= 1'b0;
						st.dbv    <= 1'b0;
						st.dfirst <= 1'b1;
						st.pdone  <= 1'b0;
						st.dly    <= 16'd0;
						st.dslow  <= 1'b0;
					end
				end
			end
			// READ HEADER: the sector's header bytes, then the sector is used up
			X_HDR: begin
				if (!mr_rq) begin
					mr_rq   <= 1'b1;
					mr_addr <= p_base + 16'd12 + {13'd0, st.fs};
				end else if (mr_ack) begin
					mr_rq <= 1'b0;
					st.fs <= st.fs + 3'd1;
					case (st.fs)
					3'd0: st.e_m <= mr_q;
					3'd1: st.e_s <= mr_q;
					3'd2: st.e_f <= mr_q;
					default: begin
						st.e_c     <= mr_q;
						st.p_next  <= 1'b1;
						st.cv      <= {12'd0, bin(st.e_f)};
						st.cv_m    <= bin(st.e_m);
						st.cv_s    <= sec_bin(st.e_s);
						st.cv_busy <= 1'b1;
						st.xs      <= X_HCONV;
					end
					endcase
				end
			end
			X_HCONV: if (!st.cv_busy) begin
				st.xr <= {1'b0, st.cv} - 20'd150;
				st.xs <= X_REPLY;
			end

			// READ SUB-CHANNEL in LBA form: absolute, then relative
			X_SUBC: if (!st.cv_busy) begin
				st.xr <= {1'b0, st.cv} - 20'd150;
				st.xs <= X_REPLY;
			end

			// one TOC entry for the NEC TOC's lead-out and track forms
			X_FETCH: begin
				if (!mr_rq) begin
					mr_rq   <= 1'b1;
					mr_addr <= ent_adr;
				end else if (mr_ack) begin
					mr_rq <= 1'b0;
					st.fs <= st.fs + 3'd1;
					case (st.fs)
					3'd0: st.e_f <= mr_q;
					3'd1: st.e_s <= mr_q;
					3'd2: st.e_m <= mr_q;
					default: begin
						st.e_c <= mr_q;
						st.xs  <= X_REPLY;
					end
					endcase
				end
			end

			X_REPLY: if (dly0) begin
				st.ph       <= F_DIN;
				st.pl       <= L_DIN;
				st.hs       <= 2'd0;
				st.pidx     <= 11'd0;
				st.fld      <= 4'd0;
				st.ent      <= 7'd0;
				st.efetched <= 1'b0;
				st.fs       <= 3'd0;
				st.nbv      <= 1'b0;
				st.dbv      <= 1'b0;
				st.dfirst   <= 1'b1;
				st.pdone    <= st.len == 11'd0;
				st.dslow    <= 1'b0;
			end

			X_DOUT: if (dly0) begin
				st.ph    <= F_DOUT;
				st.pl    <= L_DOUT;
				st.hs    <= 2'd0;
				st.dslow <= 1'b0;
			end

			default: begin
				// X_STAT: status once the command's time is up
				if (atn) begin
					st.ph    <= F_ATN;
					st.dly   <= 16'd0;
				end else if (dly0) begin
					st.ph    <= F_STAT;
					st.pl    <= L_STAT;
					st.hs    <= 2'd0;
					st.dbo   <= st.err != E_NONE ? 8'h02 : op == 8'h34 ? 8'h04 : 8'h00;
					st.dslow <= 1'b0;
					if (op == 8'h03 && st.err == E_NONE) st.sense <= E_NONE;
				end
			end
			endcase
		end

		// ---- data in: replies and sectors ----
		F_DIN: begin
			if (read_error) begin
				if (st.err == E_NONE) begin
					st.err   <= m_err;
					st.sense <= m_err;
				end
				st.xs       <= X_STAT;
				st.nbv      <= 1'b0;
				st.dbv      <= 1'b0;
				st.sec_init <= 2'd0;
				st.pdone  <= 1'b1;
				mr_rq     <= 1'b0;
			// build the next byte
			end else if (!st.nbv && !st.pdone) begin
				if (st.rt == R_SECT) begin
					case (st.sec_init)
					2'd0: begin
						if (st.scnt == 24'd0) begin
							st.pdone <= 1'b1;
						end else if (p_eov) begin
							st.err   <= E_EOV;
							st.sense <= E_EOV;
							st.pdone <= 1'b1;
						end else if (p_ready && !mr_rq) begin
							mr_rq   <= 1'b1;
							mr_addr <= p_base + 16'd15;
							st.sk   <= 12'd0;
							st.sec_init <= 2'd1;
							if (st.mode.custom) begin
								st.sect_src <= st.mode.p28_applied ? custom_origin : descriptor_source(st.mode.density, st.mode.block_len);
								st.sect_len <= st.mode.p28_applied ? st.mode.p28[11:0] : st.mode.block_len;
							end else begin
								st.sect_src <= st.mode.vflags[1:0] == 2'd3 ? 12'd12 :
								               st.mode.vflags[1:0] == 2'd2 ? 12'd16 :
								               st.mode.vflags[7:6] == 2'b11 ? 12'd24 : 12'd16;
								st.sect_len <= vendor_size({st.mode.vflags[7:6], st.mode.vflags[1:0]}, 1'b0, 1'b0);
							end
						end
					end
					2'd1: begin
						if (!mr_rq) begin
							mr_rq   <= 1'b1;
							mr_addr <= p_base + 16'd15;
						end else if (mr_ack) begin
							mr_rq   <= 1'b0;
							st.mode2 <= mr_q == 8'h02;
							if (!st.mode.custom && st.mode.vflags[1:0] == 2'd1) begin
								if (st.mode.vflags[7] && mr_q == 8'h02) st.sec_init <= 2'd2;
								else begin
									st.sect_len <= vendor_size({st.mode.vflags[7:6], st.mode.vflags[1:0]}, mr_q == 8'h02, 1'b0);
									st.sec_init <= 2'd3;
								end
							end else st.sec_init <= 2'd3;
						end
					end
					2'd2: begin
						if (!mr_rq) begin
							mr_rq   <= 1'b1;
							mr_addr <= p_base + 16'd20;
						end else if (mr_ack) begin
							mr_rq       <= 1'b0;
							st.sect_len <= vendor_size({st.mode.vflags[7:6], st.mode.vflags[1:0]}, 1'b1, mr_q[5]);
							st.sec_init <= 2'd3;
						end
					end
					default: begin
						if (st.sect_len == 12'd0) begin
							st.sec_init <= 2'd0;
							st.sfirst <= 1'b0;
							st.scnt   <= st.scnt - 24'd1;
							st.p_next <= 1'b1;
						end else if (!mr_rq) begin
							mr_rq   <= 1'b1;
							mr_addr <= sec_adr;
						end else if (mr_ack) begin
							mr_rq       <= 1'b0;
							st.nb       <= mr_q;
							st.nbv      <= 1'b1;
							st.nb_first <= st.sk == 12'd0 && !st.sfirst;
							st.nb_last  <= st.sk == st.sect_len - 12'd1;
							st.sk       <= st.sk + 12'd1;
							if (st.sk == st.sect_len - 12'd1) begin
								st.sec_init <= 2'd0;
								st.sfirst <= 1'b0;
								st.scnt   <= st.scnt - 24'd1;
								st.p_next <= 1'b1;
							end
						end
					end
					endcase
				end else if (st.pidx == st.len) begin
					st.pdone <= 1'b1;
				end else if (st.rt == R_SUBCH && !c1[1] && st.pidx == 11'd12 && !st.efetched) begin
					// the relative time in LBA form, into xr once the absolute bytes are out
					if (st.fs == 3'd0) begin
						st.cv      <= {12'd0, bin(q_rel[7:0])};
						st.cv_m    <= bin(q_rel[23:16]);
						st.cv_s    <= sec_bin(q_rel[15:8]);
						st.cv_busy <= 1'b1;
						st.fs      <= 3'd1;
					end else if (!st.cv_busy) begin
						st.xr       <= {1'b0, st.cv};
						st.efetched <= 1'b1;
					end
				end else if (toc_rep && in_ent && st.fld == 4'd0 && !st.efetched && !(st.rt == R_DE3 && c2 == 8'hB0)) begin
					// fetch the entry: F, S, M, control, then its LBA if wanted
					if (st.fs == 3'd0 && !mr_rq) st.trk <= st.rt == R_TOC ? trk_43 : trk_de;
					if (st.fs == 3'd4) begin
						if (!st.cv_busy) begin
							st.xr       <= {1'b0, st.cv} - 20'd150;
							st.efetched <= 1'b1;
							st.fs       <= 3'd0;
						end
					end else if (!mr_rq) begin
						mr_rq   <= 1'b1;
						mr_addr <= TOC_BASE + {7'd0, (st.rt == R_TOC ? trk_43 : trk_de) - 7'd1, 2'b00} + {13'd0, st.fs};
					end else if (mr_ack) begin
						mr_rq <= 1'b0;
						st.fs <= st.fs + 3'd1;
						case (st.fs)
						3'd0: st.e_f <= mr_q;
						3'd1: st.e_s <= mr_q;
						3'd2: st.e_m <= mr_q;
						default: begin
							st.e_c     <= mr_q;
							st.cv      <= {12'd0, st.e_f[6:0]};
							st.cv_m    <= st.e_m[6:0];
							st.cv_s    <= st.e_s[5:0];
							st.cv_busy <= 1'b1;
						end
						endcase
					end
				end else begin
					st.nb   <= toc_rep ? tb : hb;
					st.nbv  <= 1'b1;
					st.pidx <= st.pidx + 11'd1;
					if (toc_rep && in_ent) begin
						if (st.fld == ent_last) begin
							st.fld      <= 4'd0;
							st.ent      <= st.ent + 7'd1;
							st.efetched <= 1'b0;
						end else begin
							st.fld <= st.fld + 4'd1;
						end
					end
				end
			end

			// put it on the bus
			if (read_error) begin
				// Finish an advertised byte before changing phase.
				if (st.hs == 2'd1) begin
					if (ack) st.hs <= 2'd2;
				end else if (!ack) begin
					st.ph    <= F_EXEC;
					st.hs    <= 2'd0;
					st.dly   <= e2s;
					st.dslow <= 1'b0;
				end
			end else if (st.hs == 2'd0) begin
				if (!st.dbv && st.nbv) begin
					st.dbo     <= st.nb;
					st.db_last <= st.nb_last;
					st.dbv     <= 1'b1;
					st.nbv    <= 1'b0;
					st.dfirst <= 1'b0;
					st.dly    <= st.dfirst ? 16'd0 : st.rt != R_SECT ? B_REPLY : st.nb_first && !fast ? B_SECGAP : B_DIN;
				end else if (st.dbv && dly0) begin
					st.hs <= 2'd1;
				end else if (!st.dbv && !st.nbv && st.pdone) begin
					st.ph  <= F_STAT;
					st.pl  <= L_STAT;
					st.dbo <= st.err != E_NONE ? 8'h02 : 8'h00;
					st.dly <= e2s;
					st.dslow <= op == 8'h44;
					if (op == 8'h03) st.sense <= E_NONE;
				end else if (!st.dbv && atn) begin
					st.ph    <= F_ATN;
					st.dly   <= X_ATN;
					st.dslow <= 1'b1;
				end
			end else if (st.hs == 2'd1) begin
				if (ack) st.hs <= 2'd2;
			end else if (!ack) begin
				st.hs  <= 2'd0;
				st.dbv <= 1'b0;
				// ATN: a sector is finished first
				if (atn && (st.rt != R_SECT || st.db_last)) begin
					st.ph    <= F_ATN;
					st.dly   <= X_ATN;
					st.dslow <= 1'b1;
				end
			end
		end

		// ---- MODE SELECT parameter list ----
		F_DOUT: begin
			if (st.hs == 2'd0) begin
				if (dly0) st.hs <= 2'd1;
			end else if (st.hs == 2'd1) begin
				if (ack) begin
					st.hs      <= 2'd2;
					st.ml_left <= st.ml_left - 8'd1;
					if (!st.ml_err && !c1[4]) begin
						st.ml_i <= st.ml_i + 8'd1;
						case (st.ml_i)
						8'd4: st.ml_mode.vflags <= db_i;
						8'd5: begin
							st.ml_mode.vrecovery <= db_i;
							st.ml_err <= db_i[5:4] != 2'd0;
						end
						8'd9: begin
							st.ml_mode.vretry <= db_i[3:0];
							st.ml_mode.custom <= 1'b0;
							st.ml_mode.p28_applied <= 1'b0;
							st.ml_err <= db_i[7:4] != 4'd0;
							st.ml_st <= 3'd2;
						end
						default: st.ml_err <= db_i != 8'd0;
						endcase
					end else if (!st.ml_err) case (st.ml_st)
					3'd0: begin
						st.ml_n <= st.ml_n + 8'd1;
						if (st.ml_n == 8'd3) begin
							st.ml_err <= db_i != 8'd0 && db_i != 8'd8;
							st.ml_bd  <= db_i == 8'd8;
							st.ml_n   <= db_i;
							st.ml_i   <= 8'd0;
							st.ml_st  <= db_i == 8'd8 ? 3'd1 : 3'd2;
						end else st.ml_err <= db_i != 8'd0;
					end
					3'd1: begin
						st.ml_i <= st.ml_i + 8'd1;
						st.ml_n <= st.ml_n - 8'd1;
						case (st.ml_i)
						8'd0: begin
							st.ml_mode.density <= db_i;
							st.ml_err <= db_i != 8'h00 && db_i != 8'h81;
						end
						8'd5: st.ml_err <= db_i != 8'd0;
						8'd6: begin
							st.ml_mode.block_len[11:8] <= db_i[3:0];
							st.ml_err <= db_i[7:4] != 4'd0;
						end
						8'd7: begin
							st.ml_mode.block_len[7:0] <= db_i;
							st.ml_mode.custom <= 1'b1;
							st.ml_mode.p28_applied <= 1'b0;
							st.ml_err <= !descriptor_valid(st.ml_mode.density, {st.ml_mode.block_len[11:8], db_i});
						end
						default: ;
						endcase
						if (st.ml_n == 8'd1) st.ml_st <= 3'd2;
					end
					3'd2: begin
						st.ml_page <= db_i;
						st.ml_st   <= 3'd4;
					end
					3'd4: begin
						st.ml_i  <= 8'd0;
						st.ml_n  <= db_i;
						st.ml_st <= 3'd5;
						case (st.ml_page)
						8'h28:        st.ml_err <= db_i != 8'h04;
						8'h29, 8'h2B: st.ml_err <= db_i != 8'h01;
						8'h2A:        st.ml_err <= db_i != 8'h02;
						8'h0E:        st.ml_err <= db_i != 8'h0E;
						default:      st.ml_err <= 1'b1;
						endcase
					end
					default: begin
						st.ml_i <= st.ml_i + 8'd1;
						st.ml_n <= st.ml_n - 8'd1;
						if (st.ml_n == 8'd1) st.ml_st <= 3'd2;
						case (st.ml_page)
						8'h28: case (st.ml_i[1:0])
							2'd0: begin st.ml_mode.p28[31:24] <= db_i; st.ml_err <= db_i != 8'd0; end
							2'd1: begin st.ml_mode.p28[23:16] <= db_i; st.ml_err <= db_i != 8'd0; end
							2'd2: st.ml_mode.p28[15:8] <= db_i;
							default: begin
								st.ml_mode.p28[7:0] <= db_i;
								st.ml_mode.p28_applied <= 1'b1;
								st.ml_err <= {st.ml_mode.p28[15:8], db_i} > 16'd2352;
							end
							endcase
						8'h29: st.ml_mode.p29 <= db_i;
						8'h2A: if (st.ml_i[0]) st.ml_mode.p2a[7:0] <= db_i; else st.ml_mode.p2a[15:8] <= db_i;
						8'h2B: st.ml_mode.p2b <= db_i;
						default: case (st.ml_i)
							8'd0: st.ml_mode.p0e_immed <= db_i[2];
							8'd3: st.ml_mode.p0e_b3    <= db_i[0];
							8'd6: st.ml_mode.p0e_sel0  <= db_i[1:0];
							8'd8: st.ml_mode.p0e_sel1  <= db_i[1:0];
							default: ;
							endcase
						endcase
					end
					endcase
				end
			end else if (!ack) begin
				st.hs  <= 2'd0;
				st.dly <= B_CMD;
				if (atn) begin
					st.ph         <= F_MOUT;
					st.pl         <= L_MOUT;
					st.dly        <= 16'd0;
					st.dslow      <= 1'b0;
					st.msg_return <= M_DOUT;
				end else if (st.ml_left == 8'd0) begin
					st.ph    <= F_EXEC;
					st.mop   <= MO_FLUSH;
					st.xs    <= X_STAT;
					st.dly   <= X_MSEL;
					st.dslow <= 1'b1;
					if (st.ml_err || st.ml_st != 3'd2 ||
					    (st.ml_mode.custom && st.ml_mode.p28_applied &&
					    {1'b0, ml_origin} + {1'b0, st.ml_mode.p28[11:0]} > 13'd2352)) begin
						st.err   <= E_BAD_PARAM;
						st.sense <= E_BAD_PARAM;
					end else begin
						st.mode  <= st.ml_mode;
						st.err   <= E_NONE;
						st.sense <= E_NONE;
						if (st.ml_bd) st.xs <= X_MECH;
					end
				end
			end
		end

		// ---- status and message ----
		F_STAT, F_MIN: begin
			if (st.hs == 2'd0) begin
				if (dly0) st.hs <= 2'd1;
			end else if (st.hs == 2'd1) begin
				if (ack) st.hs <= 2'd2;
			end else if (!ack) begin
				st.hs    <= 2'd0;
				st.dslow <= 1'b0;
				if (st.ph == F_STAT && atn) begin
					st.ph         <= F_MOUT;
					st.pl         <= L_MOUT;
					st.dly        <= 16'd0;
					st.msg_return <= M_STAT;
				end else if (st.ph == F_STAT) begin
					st.ph  <= F_MIN;
					st.pl  <= L_MIN;
					st.dbo <= 8'h00;
					st.dly <= s2m;
				end else begin
					st.ph  <= F_DROP;
					st.dly <= B_FREE;
				end
			end
		end

		F_DROP: if (dly0) st.ph <= F_FREE;

		// ---- busy play keeps the bus until the end address ----
		F_HOLD: begin
			if (atn) begin
				st.ph  <= F_ATN;
				st.dly <= 16'd0;
			end else if (m_busy_end || m_play == PS_STOP) begin
				st.ph    <= F_STAT;
				st.pl    <= L_STAT;
				st.hs    <= 2'd0;
				st.dbo   <= 8'h00;
				st.dly   <= 16'd0;
				st.dslow <= 1'b0;
			end
		end

		F_ATN: if (dly0) begin
			st.ph <= F_MOUT;
			st.pl <= L_MOUT;
			st.hs <= 2'd0;
		end

		F_MOUT: begin
			if (st.hs == 2'd0) begin
				if (dly0) st.hs <= 2'd1;
			end else if (st.hs == 2'd1) begin
				if (ack) begin
					st.hs  <= 2'd2;
					st.dbo <= db_i;
				end
			end else if (!ack) begin
				st.hs <= 2'd0;
				if (st.dbo[7]) begin
					// IDENTIFY: on to the command
					st.ph    <= atn ? F_MOUT : F_CMD;
					st.pl    <= atn ? L_MOUT : L_CMD;
					st.cdb_n <= 4'd0;
					st.dly   <= B_CMD;
					st.dslow <= 1'b0;
				end else if (st.dbo == 8'h08 && st.msg_return != M_NONE) begin
					if (!atn) begin
						st.dslow <= 1'b0;
						case (st.msg_return)
						M_CMD: begin
							st.ph  <= F_CMD;
							st.pl  <= L_CMD;
							st.dly <= B_CMD;
						end
						M_DOUT, M_STAT: begin
							// Resume the release step of the acknowledged byte.
							st.ph <= st.msg_return == M_DOUT ? F_DOUT : F_STAT;
							st.pl <= st.msg_return == M_DOUT ? L_DOUT : L_STAT;
							st.hs <= 2'd2;
							if (st.msg_return == M_STAT) st.dbo <= st.err == E_NONE ? 8'h00 : 8'h02;
						end
						default: ;
						endcase
					end
				end else if (st.dbo == 8'h0C) begin
					// BUS DEVICE RESET
					st.bus_rst    <= 1'b1;
					st.ua         <= 2'd1;
					st.ph         <= F_FREE;
					st.mode       <= mode_defaults();
					st.ml_mode    <= '0;
					st.msg_return <= M_NONE;
					mr_rq         <= 1'b0;
				end else begin
					// ABORT and the rest: the command is dropped, audio stops
					st.mop   <= MO_ABORT;
					st.m_go  <= 1'b1;
					st.ph    <= F_DROP;
					st.dly   <= X_ABORT;
					st.dslow <= 1'b1;
				end
			end
		end

		default: st.ph <= F_FREE;
		endcase
	end
end

wire on_bus = st.ph != F_FREE && st.ph != F_SELW;
wire info   = st.ph == F_CMD || st.ph == F_DIN || st.ph == F_DOUT || st.ph == F_STAT || st.ph == F_MIN || st.ph == F_MOUT;

assign bsy    = on_bus;
assign req    = info && st.hs == 2'd1 && !ack;
assign msg    = on_bus && st.ph != F_SELD && st.pl[2];
assign cd     = on_bus && st.ph != F_SELD && st.pl[1];
assign io     = on_bus && st.ph != F_SELD && st.pl[0];
assign db_o   = st.dbo;
assign db_oe  = io;

// one sub-channel snapshot per reply, as the controller copies it
assign q_hold = (st.ph == F_EXEC || st.ph == F_DIN) && (op == 8'h42 || op == 8'hDD);

assign bus_rst = st.bus_rst;
assign p_next = st.p_next;
assign m_go   = st.m_go;
assign m_op   = st.mop;
assign m_lba  = st.tgt;
assign m_end  = st.tgt2;
assign m_mode = st.mmode;

assign port0_sel = st.mode.p0e_sel0;
assign port1_sel = st.mode.p0e_sel1;

assign ss_so = SAVESTATE ? st[SCSI_BITS-1] : ss_si;

wire unused_ok = &{1'b0, q_now[31:24]};

endmodule
