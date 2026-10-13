// KING SCSI initiator and CD subcode port, registers 00-0B (C6272_1 2.3.5,
// C6272_2 3.3). The register set follows the 5380 pattern: output data,
// initiator command (RST ACK SEL ATN data bus), mode (DMA, sequential),
// target command (phase to match), bus status, input data.
//
// Transfers, in DMA mode, started by writing reg 07 (receive) or reg 05
// (send). A byte moves when REQ is up and the bus phase matches reg 03:
//
//   receive, reg 0B EN = 1   bytes packed low then high into KRAM words at
//                            the reg 09 address (SCP page), one slot each,
//                            until the reg 0A count runs out: EN drops,
//                            END sets, SCDMA if reg 0B INT
//   receive, reg 0B EN = 0   pseudo-DMA: the byte waits in reg 05 D23:16
//                            (D6 = ready) until the CPU reads it
//   send                     pseudo-DMA: reg 05 D23:16 writes go out
//
//   handshake   REQ seen with a byte to take or give -> ACK 2 clocks later.
//               Once REQ falls, a DMA byte holds ACK 81 clocks; a pseudo-DMA
//               byte holds it until the CPU has read it (receive) or next
//               touches reg 05 (send). Captures: ACK 40 ns after REQ, let go
//               1.6-3.1 us after REQ falls, at the CPU's pace.
//
// Interrupts: SCSIBK on a bus reset and when REQ rises in DMA mode with the
// phase not matching; SCDMA at DMA end; SBCODE per subcode byte (reg 08 INT).
// Writing RST in reg 01 also clears regs 00, 02, 03 and the transfer, and
// holds off writes to 00-07 until RST is released.
//
// Subcode (reg 08 EN): after each -WFCK (SBCF) fall, 8 EXCK (SBCK) pulses,
// the first 528 clocks (12.3 us) in, every 64 clocks, high for 32; SBDT is taken as
// each pulse rises, SBSC in place of P.

module king_scsi #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_osc,
	input  logic        ce_dck,

	input  huc6272_pkg::acc_t acc,
	input  logic  [6:0] ar,
	output logic [15:0] rdata,
	output logic  [7:0] bus_st,      // status register D23:16
	output logic  [7:0] subcode,     // status register D31:24
	input  logic        scsibk,      // SCSIBK pending, reg 05 D4

	input  logic        page,        // SCP
	output huc6272_pkg::kreq_t req,
	output logic        req_bank,
	input  logic        gnt,

	input  logic  [7:0] db_n_i,
	output logic  [7:0] db_n_o,
	output logic        db_n_oe,
	input  logic        rst_n_i,
	output logic        rst_n_o,
	output logic        rst_n_oe,
	input  logic        ack_n_i,
	output logic        ack_n_o,
	output logic        ack_n_oe,
	input  logic        sel_n_i,
	output logic        sel_n_o,
	output logic        sel_n_oe,
	input  logic        atn_n_i,
	output logic        atn_n_o,
	output logic        atn_n_oe,
	input  logic        bsy_n_i,
	input  logic        req_n_i,
	input  logic        cd_n_i,
	input  logic        io_n_i,
	input  logic        msg_n_i,

	input  logic        sbcf_i,
	input  logic        sbsc_i,
	input  logic        sbdt_i,
	output logic        sbck_o,

	output logic        set_scsibk,
	output logic        set_scdma,
	output logic        set_sbcode,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

import huc6272_pkg::*;

localparam logic [6:0] ACK_DLY  = 7'd2;
localparam logic [6:0] ACK_HOLD = 7'd81;
localparam logic [9:0] SB_FIRST = 10'd527;
localparam logic [9:0] SB_END   = 10'd1007;  // last pulse falls

scsi_state_t st;

// Bus lines, asserted = 1.
wire       bsy = !bsy_n_i, req_b = !req_n_i, cd = !cd_n_i, io = !io_n_i, msg = !msg_n_i;
wire       sel = !sel_n_i, rst = !rst_n_i, ack_b = !ack_n_i, atn_b = !atn_n_i;
wire [7:0] db  = st.dmacr[2] ? db_n_i : ~db_n_i;
wire       pmatch = {msg, cd, io} == st.tcmd;

wire wr_reg = ce_osc && acc.wr && !acc.ar;
wire rd_reg = ce_osc && acc.rd && !acc.ar;
wire in_rst = st.icmd[4];
wire en     = st.dmacr[0];

// Byte events
wire take_dma  = st.rx && en && req_b && pmatch && st.hs == 2'd0 && !st.last && (!st.lo_v || !st.w_v);
wire take_pdma = st.rx && !en && req_b && pmatch && st.hs == 2'd0 && !st.drq;
wire cpu_rd    = rd_reg && acc.r == 7'h05 && acc.hi && st.rx && !en && st.drq;
wire send      = st.tx && !st.drq && req_b && pmatch && st.hs == 2'd0;
wire dma_done  = st.last && !st.w_v;
wire pdma_rel  = st.rx ? !st.drq : (rd_reg || wr_reg) && acc.r == 7'h05;
wire bk_ev     = (req_b && !st.req_q && st.mode[1] && !pmatch) || (rst && !st.rst_q);

// Subcode pulse position; sk counts bits taken, F = idle
wire [5:0] sp    = 6'(st.st - SB_FIRST);
wire       sb_on = st.sk != 4'hF && st.st >= SB_FIRST && st.st < SB_END;
wire       sb_up = sb_on && sp[5:0] == 6'd0 && !st.sk[3];
wire       sb_done = !(st.wf_q && !sbcf_i && st.subc[0]) && st.sk != 4'hF && st.st == SB_END;

// In the clk the reported state changes, so a pause or a clearing read
// cannot come between them.
assign set_scsibk = ce_osc && bk_ev;
assign set_scdma  = ce_osc && dma_done && st.dmacr[1];
assign set_sbcode = ce_osc && sb_done && st.subc[1];

always_ff @(posedge clk) begin
	if (reset) begin
		st       <= '0;
		st.dma_n <= 18'h3FFFF;
		st.sk    <= 4'hF;
	end else if (SAVESTATE && ss_shift) begin
		st <= scsi_state_t'({st[SCSI_BITS-2:0], ss_si});
	end else if (ce_osc) begin
		st.req_q <= req_b;
		st.rst_q <= rst;

		// ---- handshake
		case (st.hs)
			2'd0:
				if (take_dma || take_pdma || send) begin
					st.hs   <= 2'd1;
					st.ht   <= ACK_DLY - 7'd1;
					st.hdma <= take_dma;
				end
			2'd1:
				if (st.ht == 7'd0) begin
					st.ack <= 1'b1;
					st.hs  <= 2'd2;
				end else begin
					st.ht <= st.ht - 7'd1;
				end
			2'd2:
				if (!req_b) begin
					st.hs <= 2'd3;
					st.ht <= ACK_HOLD - 7'd1;
				end
			default:
				if (st.hdma ? st.ht == 7'd0 : pdma_rel) begin
					st.ack <= 1'b0;
					st.hs  <= 2'd0;
					if (st.tx) st.drq <= 1'b1;
				end else begin
					st.ht <= st.ht - 7'd1;
				end
		endcase

		if (take_dma) begin
			st.din   <= db;
			st.dma_n <= st.dma_n - 18'd1;
			if (st.dma_n == 18'd1) st.last <= 1'b1;
			if (!st.lo_v) begin
				st.lo   <= db;
				st.lo_v <= 1'b1;
			end else begin
				st.w    <= {db, st.lo};
				st.w_v  <= 1'b1;
				st.lo_v <= 1'b0;
			end
		end
		if (take_pdma) begin
			st.din <= db;
			st.drq <= 1'b1;
		end
		if (cpu_rd)
			st.drq <= 1'b0;

		// ---- KRAM word: one slot, then the address steps
		if (ce_dck) begin
			if (gnt)
				st.w_fly <= 1'b1;
			if (st.w_fly) begin
				st.w_fly    <= 1'b0;
				st.w_v      <= 1'b0;
				st.dma_a[16:0] <= st.dma_a[16:0] + 17'd1;
			end
		end

		if (dma_done) begin
			st.last     <= 1'b0;
			st.rx       <= 1'b0;
			st.dmacr[0] <= 1'b0;
		end

		// ---- registers
		if (rd_reg && acc.r == 7'h0B && !acc.hi)
			st.dend <= 1'b0;

		if (wr_reg) begin
			case (acc.r)
				7'h00: if (!in_rst && !acc.hi) st.odata <= acc.d[7:0];
				7'h01:
					if (!acc.hi) begin
						if (acc.d[7]) begin
							st.icmd  <= 5'b10000;
							st.odata <= 8'd0;
							st.mode  <= 2'd0;
							st.tcmd  <= 3'd0;
							st.din   <= 8'd0;
							st.rx    <= 1'b0;
							st.tx    <= 1'b0;
							st.drq   <= 1'b0;
							st.ack   <= 1'b0;
							st.hs    <= 2'd0;
						end else begin
							st.icmd <= {1'b0, acc.d[4], acc.d[2:0]};
						end
					end
				7'h02:
					if (!in_rst && !acc.hi) begin
						st.mode <= acc.d[1:0];
						if (!acc.d[1] || acc.d[0]) begin
							// leaving DMA mode, or sequential: ACK let go
							st.ack <= 1'b0;
							st.hs  <= 2'd0;
						end
						if (!acc.d[1]) begin
							st.rx   <= 1'b0;
							st.tx   <= 1'b0;
							st.drq  <= 1'b0;
							st.lo_v <= 1'b0;
						end
					end
				7'h03: if (!in_rst && !acc.hi) st.tcmd <= acc.d[2:0];
				7'h05:
					if (!in_rst) begin
						if (acc.hi) begin
							if (st.tx && st.drq) begin
								st.odata <= acc.d[7:0];
								st.drq   <= 1'b0;
							end
						end else if (st.mode[1]) begin
							st.tx  <= 1'b1;
							st.rx  <= 1'b0;
							st.drq <= 1'b1;
						end
					end
				7'h07:
					if (!in_rst && st.mode[1]) begin
						st.rx   <= 1'b1;
						st.tx   <= 1'b0;
						st.drq  <= 1'b0;
						st.lo_v <= 1'b0;
						st.last <= 1'b0;
					end
				7'h08: if (!acc.hi) st.subc <= acc.d[1:0];
				7'h09: if (acc.hi) st.dma_a[17:16] <= acc.d[1:0]; else st.dma_a[15:0] <= acc.d;
				7'h0A: if (acc.hi) st.dma_n[17:16] <= acc.d[1:0]; else st.dma_n[15:0] <= {acc.d[15:1], 1'b0};
				7'h0B:
					if (!acc.hi) begin
						st.dmacr <= acc.d[2:0];
						st.dend  <= 1'b0;
					end
				default: ;
			endcase
		end

		// After the register writes: END sets even against a clearing
		// access in the same clk, as SCDMA does.
		if (dma_done)
			st.dend <= 1'b1;

		// ---- subcode port
		st.wf_q <= sbcf_i;
		if (st.wf_q && !sbcf_i && st.subc[0]) begin
			st.st <= 10'd0;
			st.sk <= 4'd0;
		end else if (st.sk != 4'hF) begin
			st.st <= st.st + 10'd1;
			if (sb_up) begin
				st.sh <= {st.sh[6:0], st.sk == 4'd0 ? sbsc_i : sbdt_i};
				st.sk <= st.sk + 4'd1;
			end
			if (st.st == SB_END) begin
				st.sub <= st.sh;
				st.sk  <= 4'hF;
			end
		end
	end
end

assign ss_so = SAVESTATE ? st[SCSI_BITS-1] : ss_si;

assign bus_st  = {rst, bsy, req_b, msg, cd, io, sel, 1'b0};
assign subcode = st.sub;

always_comb begin
	rdata = 16'd0;
	case (ar)
		7'h00: rdata = acc.hi ? 16'd0 : {8'd0, db};
		7'h01: rdata = acc.hi ? 16'd0 : {8'd0, st.icmd[4], 2'd0, st.icmd[3], 1'b0, st.icmd[2:0]};
		7'h02: rdata = acc.hi ? 16'd0 : {14'd0, st.mode};
		7'h03: rdata = acc.hi ? 16'd0 : {13'd0, st.tcmd};
		7'h04: rdata = acc.hi ? 16'd0 : {8'd0, bus_st};
		7'h05: rdata = acc.hi ? {8'd0, st.din} : {9'd0, st.drq, 1'b0, scsibk, pmatch, 1'b0, atn_b, ack_b};
		7'h06: rdata = acc.hi ? 16'd0 : {8'd0, st.din};
		7'h07: rdata = acc.hi ? 16'd0 : 16'h00FF;
		7'h08: rdata = acc.hi ? 16'd0 : {8'd0, st.sub};
		7'h09: rdata = acc.hi ? {14'd0, st.dma_a[17:16]} : st.dma_a[15:0];
		7'h0A: rdata = acc.hi ? {14'd0, st.dma_n[17:16]} : st.dma_n[15:0];
		7'h0B: rdata = acc.hi ? 16'd0 : {15'd0, st.dend};
		default: ;
	endcase
end

always_comb begin
	req      = '0;
	req.v    = st.w_v && !st.w_fly;
	req.we   = 1'b1;
	req.page = page;
	req.a    = st.dma_a[16:0];
	req.d    = st.w;
end

assign req_bank = st.dma_a[17];

assign rst_n_o  = 1'b0;
assign rst_n_oe = st.icmd[4];
assign ack_n_o  = 1'b0;
assign ack_n_oe = (st.icmd[3] || st.ack) && !st.mode[0];
assign sel_n_o  = 1'b0;
assign sel_n_oe = st.icmd[2];
assign atn_n_o  = 1'b0;
assign atn_n_oe = st.icmd[1];
assign db_n_oe  = st.icmd[0] && !io;
assign db_n_o   = st.dmacr[2] ? st.odata : ~st.odata;
assign sbck_o   = sb_on && !sp[5];

endmodule
