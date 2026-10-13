// Main-RAM controller for 4 x uPD424800 (512K x 8, fast page, 2 KB page).
//
// Times in clk_sys edges; k is the CPU CE edge that starts T1, j a CE edge.
//
//   page hit   CAS low from k (combinational on the T1 address), high at k+2,
//              CPU samples at k+4                                  2 clocks
//   page miss  RAS high at k+1, row on MA, RAS low at k+4, column and CAS
//              low at k+6, CAS high at k+8, CPU samples at k+10      5 clocks
//
// CAS changes only on CE edges, so each CAS pulse spans a CE edge; RAS keeps
// 3-clock precharge (tRP 60 ns) and 4-clock CBR (tRAS 70 ns).
//   refresh    every REFRESH_SCK clocks, at the first free CE edge j: RAS
//              high at j, all CAS low at j+2, RAS low at j+3, both high at
//              j+7. Closes the page; a RAM cycle meanwhile waits, then misses.
//
// Row = A20:11, column = A10:2. 0x200000-0xFFFFFF keeps page timing but
// drives no RAS or CAS.

module ga_mmc
	import upd65641_pkg::*;
#(
	parameter bit SAVESTATE   = 1'b1,
	parameter int REFRESH_SCK = 335
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_sys,
	input  logic        ce_sck,

	input  logic [31:2] a,
	input  logic  [3:0] be_n,
	input  logic        mrq_n,
	input  logic        rw,
	input  logic        bcyst_n,

	output logic        ras_n,
	output logic  [3:0] cas_n,
	output logic  [9:0] ma,
	output logic        we_n,
	output logic        oe_n,

	output logic        rdy,
	output logic        busy,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

localparam logic [1:0] OP_IDLE = 2'd0;
localparam logic [1:0] OP_HIT  = 2'd1;
localparam logic [1:0] OP_MISS = 2'd2;
localparam logic [1:0] OP_REF  = 2'd3;

localparam logic [8:0] REF_LAST = 9'(REFRESH_SCK - 1);

mmc_state_t st;

wire ram_t1 = !bcyst_n && !mrq_n && a[31:24] == 8'h00;
wire hit    = st.page_ok && a[23:11] == st.page;
wire idle   = st.op == OP_IDLE;

wire hit_last  = st.op == OP_HIT  && st.step == 4'd3;
wire miss_last = st.op == OP_MISS && st.step == 4'd9;
wire ref_last  = st.op == OP_REF  && st.step == 4'd7;
wire free      = idle || hit_last || miss_last;
wire ram_op    = (st.op == OP_HIT || st.op == OP_MISS) && st.page_pop;

// The page-hit CAS falls in the first clock of T1.
wire cas_now = idle && ram_t1 && hit && st.page_pop;

always_ff @(posedge clk) begin
	if (reset) begin
		st       <= '0;
		st.ras_n <= 1'b1;
	end else if (SAVESTATE && ss_shift) begin
		st <= mmc_state_t'({st[MMC_BITS-2:0], ss_si});
	end else if (ce_sys) begin
		if (ce_sck) begin
			if (st.ref_cnt == REF_LAST) begin
				st.ref_cnt <= 9'd0;
				st.ref_req <= 1'b1;
			end else begin
				st.ref_cnt <= st.ref_cnt + 9'd1;
			end
		end

		if (ce_sck && free && st.ref_req) begin
			st.op      <= OP_REF;
			st.step    <= 4'd1;
			st.ras_n   <= 1'b1;
			st.cas     <= 4'h0;
			st.page_ok <= 1'b0;
			st.ref_req <= 1'b0;
		end else if (ref_last) begin
			st.ras_n <= 1'b1;
			st.cas   <= 4'h0;
			st.op    <= OP_IDLE;
			if (st.pend || ram_t1) begin
				st.op       <= OP_MISS;
				st.step     <= 4'd1;
				st.pend     <= 1'b0;
				st.page     <= a[23:11];
				st.page_ok  <= 1'b1;
				st.page_pop <= a[23:21] == 3'd0;
			end
		end else if (idle) begin
			// T1 is first seen on the edge after the CPU's CE edge.
			if (ram_t1 && !ce_sck) begin
				st.step <= 4'd1;
				if (hit) begin
					st.op  <= OP_HIT;
					st.cas <= st.page_pop ? ~be_n : 4'h0;
				end else begin
					st.op       <= OP_MISS;
					st.ras_n    <= 1'b1;
					st.page     <= a[23:11];
					st.page_ok  <= 1'b1;
					st.page_pop <= a[23:21] == 3'd0;
				end
			end
		end else begin
			st.step <= st.step + 4'd1;
			case (st.op)
				OP_HIT: begin
					if (st.step == 4'd1)
						st.cas <= 4'h0;
					if (hit_last)
						st.op <= OP_IDLE;
				end
				OP_MISS: begin
					if (st.step == 4'd3)
						st.ras_n <= !st.page_pop;
					if (st.step == 4'd5)
						st.cas <= st.page_pop ? ~be_n : 4'h0;
					if (st.step == 4'd7)
						st.cas <= 4'h0;
					if (miss_last)
						st.op <= OP_IDLE;
				end
				default: begin
					// CAS before RAS.
					if (st.step == 4'd2)
						st.cas <= 4'hF;
					if (st.step == 4'd3)
						st.ras_n <= 1'b0;
					if (ram_t1)
						st.pend <= 1'b1;
				end
			endcase
		end
	end
end

assign ss_so = SAVESTATE ? st[MMC_BITS-1] : ss_si;

assign ras_n = st.ras_n;
assign cas_n = ~(st.cas | ({4{cas_now}} & ~be_n));
assign ma    = (st.op == OP_MISS && st.step < 4'd5) ? a[20:11] : {1'b0, a[10:2]};
assign we_n  = rw || !(cas_now || ram_op);
assign oe_n  = !rw || !ram_op;

// A sequence is under way: no freeze for a savestate until it ends.
assign busy = st.op != OP_IDLE;

// Read on a CE edge: the access enters its last T-state there.
assign rdy = (st.op == OP_HIT && st.step == 4'd1) ||
	(st.op == OP_MISS && st.step == 4'd7);

endmodule
