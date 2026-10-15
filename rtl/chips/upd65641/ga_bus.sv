// CPU bus control: cycle tracking, READY, SZRQ, byte lanes, and the HuC I/O
// bus with its one-deep write buffer.
//
// A cycle is taken at the CE edge that ends T1 and finishes at the CE edge
// after its last T-state. READY is a flop that is high for that whole last
// state, so it holds at the mid-state CLK fall (the data sheet's sample) and
// at the CE edge. Every kind but main RAM asks for 16-bit sizing.
//
// HuC I/O bus cycle, in CPU clocks from the end of T1:
//
//   step     1     2     3     4     5
//   CS, IOA  ======================== (IOD too on writes)
//   RD/WR          \_______________/
//                              ^ BUSY low holds step 4; read data latched
//
// A write is buffered: the CPU sees READY in step 1 (2 clocks in all) while
// the bus finishes on its own. A HuC cycle arriving while the buffer is busy
// waits for it. A read sees READY in step 5 (6 clocks).

module ga_bus
	import upd65641_pkg::*;
#(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_sys,
	input  logic        ce_sck,

	input  logic [31:0] d_i,
	output logic [31:0] d_o,
	output logic        d_oe,
	input  logic        hi,
	input  logic        rw,
	input  logic        bcyst_n,
	output logic        ready_n,
	output logic        szrq_n,

	input  kind_t       dec_kind,
	input  logic  [2:0] dec_chip,
	input  logic  [5:1] dec_ioa,
	input  logic  [3:0] dec_wait,

	input  logic        ram_rdy,

	input  logic [15:0] ld_i,
	output logic [15:0] ld_o,
	output logic        ld_oe,
	output logic        la1,
	input  logic        xeb_n,

	output logic        cyc,
	output kind_t       kind,
	output logic        wlast,

	output logic  [5:1] ioa,
	input  logic [15:0] iod_i,
	output logic [15:0] iod_o,
	output logic        iod_oe,
	output logic        iord_n,
	output logic        iowr_n,
	output logic  [5:0] cs_n,
	input  logic  [5:0] busy_n,
	output logic        io_wr_done,
	output logic  [2:0] io_chip,
	output logic        io_busy,      // an I/O cycle is still on the pins

	output logic        reg_we,
	output logic        reg_re,
	output logic        reg_hi,
	output logic [15:0] reg_wd,
	input  logic [15:0] reg_rd,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

iob_state_t st;

// The CPU cycle itself is not saved: the bus is idle at the capture point.
logic        cyc_q;
kind_t       kind_q;
logic  [3:0] wcnt_q;
logic        owner_q;
logic [15:0] rd_q;

wire [15:0] wd = d_i[15:0];
wire unused_d_hi = ^d_i[31:16];

wire t1_end  = ce_sck && !bcyst_n;
wire io_kind = kind_q == K_IOR || kind_q == K_IOW;
wire local_k = kind_q == K_ROM || kind_q == K_BRAM || kind_q == K_BMP || kind_q == K_EXP;

wire [7:0] busy_v = {2'b11, busy_n};
wire busy_hold = !busy_v[st.chip];
wire io_last   = st.busy && st.step == 3'd5;
wire strobe_up = ce_sck && st.busy && st.step == 3'd4 && !busy_hold;
wire io_start  = ce_sck && (!st.busy || io_last) &&
	((t1_end && (dec_kind == K_IOR || dec_kind == K_IOW)) || (cyc_q && io_kind && !owner_q));

always_comb begin
	case (kind_q)
		K_RAM:   wlast = ram_rdy;
		K_IOW:   wlast = owner_q && st.busy && st.step == 3'd1;
		K_IOR:   wlast = owner_q && io_last;
		default: wlast = wcnt_q == 4'd0;
	endcase
end

wire io_free = !st.busy || io_last;
wire wait_ok = !(kind_q == K_EXP && !xeb_n);

// At a CE edge: does the cycle enter its last T-state here?
logic ready_q, ready_next;
always_comb begin
	ready_next = 1'b0;
	if (t1_end) begin
		case (dec_kind)
			K_RAM:   ready_next = ram_rdy;
			K_IOW:   ready_next = io_free;
			K_IOR:   ready_next = 1'b0;
			default: ready_next = dec_wait == 4'd0;
		endcase
	end else if (cyc_q && !ready_q) begin
		case (kind_q)
			K_RAM:   ready_next = ram_rdy;
			K_IOW:   ready_next = !owner_q && io_free;
			K_IOR:   ready_next = owner_q && st.step == 3'd4 && !busy_hold;
			default: ready_next = wcnt_q == 4'd1 && wait_ok;
		endcase
	end
end

wire accept = ce_sck && ready_q;

always_ff @(posedge clk) begin
	if (reset) begin
		st <= '0;
	end else if (SAVESTATE && ss_shift) begin
		st <= iob_state_t'({st[IOB_BITS-2:0], ss_si});
	end else if (ce_sys && ce_sck) begin
		if (io_start) begin
			st.busy  <= 1'b1;
			st.step  <= 3'd1;
			st.write <= !rw;
			st.chip  <= dec_chip;
			st.ioa   <= dec_ioa;
			st.data  <= wd;
		end else if (io_last) begin
			st.busy <= 1'b0;
			st.step <= 3'd0;
		end else if (st.busy && !(st.step == 3'd4 && busy_hold)) begin
			st.step <= st.step + 3'd1;
		end
	end
end

assign ss_so = SAVESTATE ? st[IOB_BITS-1] : ss_si;

always_ff @(posedge clk) begin
	if (reset) begin
		cyc_q   <= 1'b0;
		kind_q  <= K_NONE;
		wcnt_q  <= 4'd0;
		owner_q <= 1'b0;
		ready_q <= 1'b0;
		rd_q    <= 16'd0;
	end else if (ce_sys) begin
		if (ce_sck)
			ready_q <= ready_next;

		if (kind_q == K_GLUE)
			rd_q <= reg_rd;
		if (strobe_up && !st.write)
			rd_q <= iod_i;

		if (ce_sck) begin
			if (t1_end) begin
				cyc_q  <= 1'b1;
				kind_q <= dec_kind;
				wcnt_q <= dec_wait;
			end else if (accept) begin
				cyc_q <= 1'b0;
			end else if (wcnt_q != 4'd0 && (wcnt_q != 4'd1 || wait_ok)) begin
				wcnt_q <= wcnt_q - 4'd1;
			end

			if (io_start)
				owner_q <= 1'b1;
			else if (accept)
				owner_q <= 1'b0;
		end
	end
end

assign cyc  = cyc_q;
assign kind = kind_q;

assign ready_n = !ready_q;
assign szrq_n  = !(cyc_q && kind_q != K_RAM);

// 16-bit devices use D15:0 for either half of a sized word.
wire [15:0] rd16 = local_k ? ld_i : rd_q;
assign d_o  = {16'hFFFF, rd16};
assign d_oe = cyc_q && rw && (local_k || kind_q == K_IOR || kind_q == K_GLUE);

assign ld_o  = wd;
assign ld_oe = cyc_q && !rw && local_k;
assign la1   = hi;

assign ioa        = st.ioa;
assign iod_o      = st.data;
assign iod_oe     = st.busy && st.write;
assign iord_n     = !(st.busy && !st.write && st.step >= 3'd2 && st.step <= 3'd4);
assign iowr_n     = !(st.busy && st.write && st.step >= 3'd2 && st.step <= 3'd4);
assign io_wr_done = strobe_up && st.write;
assign io_chip    = st.chip;
assign io_busy    = st.busy;

always_comb begin
	for (int i = 0; i < 6; i++)
		cs_n[i] = !(st.busy && st.chip == 3'(i));
end

assign reg_we = accept && kind_q == K_GLUE && !rw;
assign reg_re = accept && kind_q == K_GLUE && rw;
assign reg_hi = hi;
assign reg_wd = wd;

endmodule
