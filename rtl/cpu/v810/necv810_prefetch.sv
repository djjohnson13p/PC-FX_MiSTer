// NEC V810 instruction prefetch unit (used with T2_PINS).
//
// Uncached, it fetches aligned 32-bit words, low half first, as T1/T2 bus
// cycles: two 16-bit beats in fixed-16 mode, one beat on the 32-bit bus (two
// when SZRQ asks). Words stream back to back into a three-word queue; the
// chain stops when the queue is full or after a data access, and restarts
// when the decoder needs a word it does not hold. A redirect flushes the
// queue and refetches from the target's word, even one already held. The
// next word's address is fixed at the edge that starts the in-flight word's
// last beat, so a redirect after that point lets one more sequential word go
// out first.
//
//   fixed-16 word:  T1 T2 .. | T1 T2 ..   (low beat, high beat)
//                              ^ next address fixed here
//
// With the cache on, it reads one word per clock from the cache, starting
// with the target in the clock the redirect is seen. A missing word is
// fetched on the bus and written into its cache line half.
//
// Pins come from registers loaded at the CE edge that starts T1. READY and
// SZRQ are sampled at phi1 in T2, DIN at the CE that ends T2.
module necv810_prefetch (
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,
	input  wire        phi1_i,
	input  wire        fixed_16_i,
	input  wire        ready_i,
	input  wire        szrq_i,
	input  wire [31:0] din_i,
	// A data access waits or runs: no new word starts.
	input  wire        data_wait_i,
	// The core needs the instruction at want_pc. Redirect marks a flow change
	// the core took at the previous CE edge.
	input  wire        want_valid_i,
	input  wire [31:1] want_pc_i,
	input  wire        redirect_i,
	input  wire        bubble_i,        // the target starts one clock late
	input  wire        flush_i,
	// Instruction cache: a read address is sampled at phi1, so its line is
	// here by the next CE. Writes take effect at the CE after ic_wr_o.
	input  wire        cache_on_i,
	input  wire        ic_hold_i,       // cache maintenance runs
	input  wire [63:0] ic_data_i,
	input  wire [31:0] ic_tag_i,
	output wire        ic_rd_o,
	output wire [6:0]  ic_rd_index_o,
	output reg         ic_wr_o,
	output reg  [6:0]  ic_wr_index_o,
	output reg  [63:0] ic_wr_data_o,
	output reg  [31:0] ic_tag_wr_o,
	output wire        busy_o,
	output wire        pin_own_o,       // copies of busy_o for the pin multiplexers:
	output wire        pin_own_c_o,     // one for A, one for the other pins
	output wire        done_o,          // the word on the bus completes at this CE
	output wire        ready_o,
	output wire [31:0] iw_o,
	output wire [31:1] a_o,
	output wire [3:0]  be_o,
	output wire [1:0]  st_o,
	output wire        bcyst_o,
	output wire        da_o
);

	reg        q_valid_q [0:2];
	reg        q_aged_q  [0:2];
	reg [29:0] q_addr_q  [0:2];
	reg [31:0] q_data_q  [0:2];

	reg        f_active_q;      // a word is on the bus
	reg [29:0] f_addr_q;
	reg        f_waste_q;       // its data is not wanted any more
	reg        f_fill_q;        // it fills a cache line half
	reg        f_beat_q;        // second beat of the word
	reg        f_t1_q;
	reg        f_branch_q;      // first word after a redirect
	reg [15:0] f_lo_q;
	reg        rdy_q;
	reg        split_q;         // SZRQ asked for a second beat
	reg        lb_started_q;    // the in-flight word's last beat has begun
	reg        lb_now_q;        // ... at the latest CE edge
	reg [29:0] next_addr_q;     // word that follows the in-flight one
	reg        next_set_q;      // fixed by a redirect: start it regardless
	reg        next_waste_q;
	reg        next_branch_q;
	reg        next_bubble_q;
	reg        tgt_valid_q;     // target still to start
	reg [29:0] tgt_addr_q;
	reg        stopped_q;
	reg        c_run_q;         // cache reads one word per clock
	reg [29:0] c_addr_q;
	reg [63:0] fill_line_q;     // line read at the miss
	reg [31:0] fill_tag_q;
	// Copies of the bus-cycle state that feed only the pins, so they can sit
	// next to the logic that decodes the bus.
	(* preserve *) reg        pin_own_q;
	(* preserve *) reg        pin_own_c_q;
	(* preserve *) reg [31:1] pin_a_q;
	(* preserve *) reg [3:0]  pin_be_q;
	reg        fill_ok_q;       // no maintenance since the line was read

	integer    i;

	wire [29:0] want_word_w = want_pc_i[31:2];
	wire [29:0] want_next_w = want_pc_i[31:2] + 30'd1;

	// Look up the wanted instruction: first halfword, and the second one of
	// a 32-bit format.
	reg        hit0_v, hit1_v, aged0_v, aged1_v;
	reg [31:0] word0_v, word1_v;
	reg [1:0]  count_v;             // words kept past this CE
	reg [2:0]  keep_v;
	reg [29:0] behind_v;
	always @* begin
		hit0_v = 1'b0; hit1_v = 1'b0; aged0_v = 1'b0; aged1_v = 1'b0;
		word0_v = 32'd0; word1_v = 32'd0; count_v = 2'd0;
		for (i = 0; i < 3; i = i + 1) begin
			// Words the decoder has moved past are dropped at this CE.
			behind_v = want_word_w - q_addr_q[i];
			keep_v[i] = q_valid_q[i] &&
				!(want_valid_i && (behind_v != 30'd0) && (behind_v < 30'd8));
			count_v = count_v + {1'b0, keep_v[i]};
			if (q_valid_q[i] && (q_addr_q[i] == want_word_w)) begin
				hit0_v = 1'b1; aged0_v = q_aged_q[i]; word0_v = q_data_q[i];
			end
			if (q_valid_q[i] && (q_addr_q[i] == want_next_w)) begin
				hit1_v = 1'b1; aged1_v = q_aged_q[i]; word1_v = q_data_q[i];
			end
		end
	end
	wire [15:0] half0_w = want_pc_i[1] ? word0_v[31:16] : word0_v[15:0];
	wire [15:0] half1_w = want_pc_i[1] ? word1_v[15:0] : word0_v[31:16];
	wire        long_w = (half0_w[15:13] == 3'b101) || (half0_w[15:13] == 3'b110) ||
		(half0_w[15:13] == 3'b111) || (half0_w[15:10] == 6'b111110);
	wire        need_next_w = long_w && want_pc_i[1];
	// An instruction is ready a full clock after its last word arrived.
	assign ready_o = want_valid_i && !redirect_i && hit0_v && aged0_v &&
		(!need_next_w || (hit1_v && aged1_v));
	assign iw_o = {half0_w, long_w ? half1_w : 16'd0};

	// The decoder needs a word that is neither queued nor on its way.
	wire [29:0] need_word_w = (hit0_v && need_next_w) ? want_next_w : want_word_w;
	wire        starve_w = want_valid_i && !(hit0_v && (!need_next_w || hit1_v)) &&
		!(f_active_q && !f_waste_q && (f_addr_q == need_word_w));
	wire        beat_done_w = f_active_q && !f_t1_q && rdy_q;
	wire        word_done_w = beat_done_w && (fixed_16_i ? f_beat_q : (f_beat_q || !split_q));
	wire        in_time_w = !lb_started_q || lb_now_q;

	// Cache read of this clock. A line written at the coming CE is taken from
	// the write registers.
	wire [29:0] rd_word_w = redirect_i ? want_word_w : c_addr_q;
	wire        rd_fwd_w = ic_wr_o && (ic_wr_index_o == rd_word_w[7:1]);
	wire [63:0] rd_line_w = rd_fwd_w ? ic_wr_data_o : ic_data_i;
	wire [31:0] rd_tag_w = rd_fwd_w ? ic_tag_wr_o : ic_tag_i;
	wire        rd_hit_w = (rd_tag_w[21:0] == rd_word_w[29:8]) &&
		(rd_word_w[0] ? rd_tag_w[23] : rd_tag_w[22]);
	wire [31:0] rd_data_w = rd_word_w[0] ? {rd_line_w[15:0], rd_line_w[31:16]} :
		{rd_line_w[47:32], rd_line_w[63:48]};
	assign ic_rd_o = cache_on_i && (redirect_i || c_run_q);
	assign ic_rd_index_o = rd_word_w[7:1];

	assign busy_o = f_active_q;
	assign done_o = word_done_w;
	assign pin_own_o = pin_own_q;
	assign pin_own_c_o = pin_own_c_q;
	assign a_o = pin_a_q;
	assign be_o = pin_be_q;
	assign st_o = !pin_own_c_q ? 2'b00 : (f_branch_q && !f_beat_q) ? 2'b01 : 2'b11;
	assign bcyst_o = pin_own_c_q && f_t1_q;
	assign da_o = pin_own_c_q && !f_t1_q;

	task start_word_task;
		input [29:0] addr;
		input        waste;
		input        branch;
		input        fill;
		begin
			f_active_q <= 1'b1;
			pin_own_q <= 1'b1;
			pin_own_c_q <= 1'b1;
			f_addr_q <= addr;
			pin_a_q <= {addr, 1'b0};
			pin_be_q <= fixed_16_i ? 4'b1100 : 4'b0000;
			f_waste_q <= waste;
			f_fill_q <= fill;
			f_beat_q <= 1'b0;
			f_t1_q <= 1'b1;
			f_branch_q <= branch;
			lb_started_q <= !fixed_16_i;
			lb_now_q <= !fixed_16_i;
			next_addr_q <= addr + 30'd1;
			next_set_q <= 1'b0;
			next_waste_q <= 1'b0;
			next_branch_q <= 1'b0;
			next_bubble_q <= 1'b0;
		end
	endtask

	// Start a cache fill of the word just read, keeping its line and tag.
	// With the cache on no fetch shows as a fetch after branch.
	task start_fill_task;
		input [29:0] addr;
		begin
			c_run_q <= 1'b0;
			c_addr_q <= addr;
			fill_line_q <= rd_line_w;
			fill_tag_q <= rd_tag_w;
			fill_ok_q <= 1'b1;
			if (!f_active_q && !data_wait_i) begin
				start_word_task(addr, 1'b0, 1'b0, 1'b1);
			end else begin
				tgt_valid_q <= 1'b1;
				tgt_addr_q <= addr;
			end
		end
	endtask

	reg [31:0] word_v;
	reg        push_v;
	reg        push_aged_v;
	reg [29:0] push_addr_v;
	reg        start_ok_v;
	reg [31:0] fill_tag_v;
	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i) begin
			for (i = 0; i < 3; i = i + 1) begin
				q_valid_q[i] <= 1'b0; q_aged_q[i] <= 1'b0; q_addr_q[i] <= 30'd0; q_data_q[i] <= 32'd0;
			end
			f_active_q <= 1'b0; f_addr_q <= 30'd0; f_waste_q <= 1'b0; f_fill_q <= 1'b0;
			pin_own_q <= 1'b0; pin_own_c_q <= 1'b0; pin_a_q <= 31'd0; pin_be_q <= 4'b0000;
			f_beat_q <= 1'b0; f_t1_q <= 1'b0; f_branch_q <= 1'b0; f_lo_q <= 16'd0;
			rdy_q <= 1'b0; split_q <= 1'b0; lb_started_q <= 1'b0; lb_now_q <= 1'b0;
			next_addr_q <= 30'd0; next_set_q <= 1'b0; next_waste_q <= 1'b0; next_branch_q <= 1'b0;
			next_bubble_q <= 1'b0;
			tgt_valid_q <= 1'b0; tgt_addr_q <= 30'd0; stopped_q <= 1'b1;
			c_run_q <= 1'b0; c_addr_q <= 30'd0; fill_line_q <= 64'd0; fill_tag_q <= 32'd0;
			fill_ok_q <= 1'b0;
			ic_wr_o <= 1'b0; ic_wr_index_o <= 7'd0; ic_wr_data_o <= 64'd0; ic_tag_wr_o <= 32'd0;
		end else begin
			// READY and SZRQ count at the mid-T2 edge.
			if (phi1_i && f_active_q && !f_t1_q) begin
				rdy_q <= ready_i;
				if (!f_beat_q) split_q <= !fixed_16_i && !szrq_i;
			end
			if (ce_i) begin
				rdy_q <= 1'b0;
				lb_now_q <= 1'b0;
				ic_wr_o <= 1'b0;
				for (i = 0; i < 3; i = i + 1) begin
					if (q_valid_q[i]) q_aged_q[i] <= 1'b1;
					q_valid_q[i] <= keep_v[i];
				end

				// Beat and word progress.
				push_v = 1'b0;
				push_aged_v = 1'b0;
				push_addr_v = f_addr_q;
				word_v = fixed_16_i ? {din_i[15:0], f_lo_q} :
					(f_beat_q ? {din_i[31:16], f_lo_q} : din_i);
				if (f_active_q) begin
					if (f_t1_q) begin
						f_t1_q <= 1'b0;
					end else if (beat_done_w) begin
						if (!word_done_w) begin
							f_lo_q <= din_i[15:0];
							f_beat_q <= 1'b1;
							pin_a_q <= {f_addr_q, 1'b1};
							pin_be_q <= fixed_16_i ? 4'b1100 : 4'b0011;
							f_t1_q <= 1'b1;
							lb_started_q <= 1'b1;
							lb_now_q <= 1'b1;
						end else begin
							push_v = !f_waste_q;
							f_active_q <= 1'b0;
							pin_own_q <= 1'b0;
							pin_own_c_q <= 1'b0;
							// A wasted fill is dropped: its line copy may belong to a later miss.
							if (f_fill_q && !f_waste_q) begin
								// Write the word into its line half.
								fill_tag_v = (fill_tag_q[21:0] == f_addr_q[29:8]) ?
									fill_tag_q : {10'd0, f_addr_q[29:8]};
								ic_wr_o <= fill_ok_q && !ic_hold_i;
								ic_wr_index_o <= f_addr_q[7:1];
								ic_wr_data_o <= f_addr_q[0] ?
									{fill_line_q[63:32], word_v[15:0], word_v[31:16]} :
									{word_v[15:0], word_v[31:16], fill_line_q[31:0]};
								ic_tag_wr_o <= f_addr_q[0] ? (fill_tag_v | 32'h0080_0000) :
									(fill_tag_v | 32'h0040_0000);
								if (!f_waste_q) begin
									c_run_q <= 1'b1;
									c_addr_q <= f_addr_q + 30'd1;
								end
							end
						end
					end
				end
				start_ok_v = (!f_active_q || word_done_w) && !data_wait_i;

				if (flush_i) begin
					for (i = 0; i < 3; i = i + 1) q_valid_q[i] <= 1'b0;
					if (f_active_q) f_waste_q <= 1'b1;
					next_set_q <= 1'b0;
					tgt_valid_q <= 1'b0;
					stopped_q <= 1'b1;
					c_run_q <= 1'b0;
				end else if (redirect_i && cache_on_i) begin
					// The target was read this clock.
					for (i = 0; i < 3; i = i + 1) q_valid_q[i] <= 1'b0;
					if (f_active_q) f_waste_q <= 1'b1;
					next_set_q <= 1'b0;
					tgt_valid_q <= 1'b0;
					if (rd_hit_w) begin
						q_valid_q[0] <= 1'b1; q_aged_q[0] <= 1'b1;
						q_addr_q[0] <= want_word_w; q_data_q[0] <= rd_data_w;
						c_run_q <= 1'b1;
						c_addr_q <= want_word_w + 30'd1;
					end else begin
						start_fill_task(want_word_w);
					end
				end else if (redirect_i) begin
					for (i = 0; i < 3; i = i + 1) q_valid_q[i] <= 1'b0;
					stopped_q <= 1'b0;
					c_run_q <= 1'b0;
					if (!f_active_q) begin
						if (!data_wait_i && !bubble_i) begin
							start_word_task(want_word_w, 1'b0, 1'b1, 1'b0);
							tgt_valid_q <= 1'b0;
						end else begin
							tgt_valid_q <= 1'b1;
							tgt_addr_q <= want_word_w;
						end
					end else if (word_done_w || !in_time_w) begin
						// Too late: one more sequential word goes out first.
						if (!word_done_w) f_waste_q <= 1'b1;
						if (start_ok_v) begin
							start_word_task(next_addr_q, 1'b1, 1'b0, 1'b0);
						end else begin
							next_set_q <= 1'b1;
							next_waste_q <= 1'b1;
						end
						tgt_valid_q <= 1'b1;
						tgt_addr_q <= want_word_w;
					end else begin
						f_waste_q <= 1'b1;
						next_addr_q <= want_word_w;
						next_set_q <= 1'b1;
						next_waste_q <= 1'b0;
						next_branch_q <= 1'b1;
						next_bubble_q <= bubble_i;
						tgt_valid_q <= 1'b0;
					end
				end else begin
					// Cache hits come in one word per clock while there is room.
					if (cache_on_i && c_run_q && !push_v) begin
						if ((count_v != 2'd3) && rd_hit_w) begin
							push_v = 1'b1;
							push_aged_v = 1'b1;
							push_addr_v = c_addr_q;
							word_v = rd_data_w;
							c_addr_q <= c_addr_q + 30'd1;
						end else if (!rd_hit_w && (count_v != 2'd3)) begin
							// A fill only starts with a queue slot free for its word.
							start_fill_task(c_addr_q);
						end
					end
					if (push_v) begin
						if (!keep_v[0]) begin
							q_valid_q[0] <= 1'b1; q_aged_q[0] <= push_aged_v; q_addr_q[0] <= push_addr_v; q_data_q[0] <= word_v;
						end else if (!keep_v[1]) begin
							q_valid_q[1] <= 1'b1; q_aged_q[1] <= push_aged_v; q_addr_q[1] <= push_addr_v; q_data_q[1] <= word_v;
						end else begin
							q_valid_q[2] <= 1'b1; q_aged_q[2] <= push_aged_v; q_addr_q[2] <= push_addr_v; q_data_q[2] <= word_v;
						end
					end
					if (cache_on_i) begin
						if (start_ok_v && tgt_valid_q) begin
							start_word_task(tgt_addr_q, 1'b0, 1'b0, 1'b1);
							tgt_valid_q <= 1'b0;
						end else if (!c_run_q && !f_active_q && !tgt_valid_q && starve_w) begin
							c_run_q <= 1'b1;
							c_addr_q <= need_word_w;
						end
					end else if (start_ok_v) begin
						if (next_set_q && next_bubble_q) begin
							next_set_q <= 1'b0;
							next_bubble_q <= 1'b0;
							tgt_valid_q <= 1'b1;
							tgt_addr_q <= next_addr_q;
						end else if (next_set_q) begin
							start_word_task(next_addr_q, next_waste_q, next_branch_q, 1'b0);
						end else if (tgt_valid_q) begin
							start_word_task(tgt_addr_q, 1'b0, 1'b1, 1'b0);
							tgt_valid_q <= 1'b0;
						end else if (word_done_w && !stopped_q &&
							((count_v + {1'b0, push_v}) < 2'd3)) begin
							start_word_task(next_addr_q, 1'b0, 1'b0, 1'b0);
						end else if (starve_w) begin
							start_word_task(need_word_w, 1'b0, 1'b0, 1'b0);
							stopped_q <= 1'b0;
						end else if (word_done_w) begin
							stopped_q <= 1'b1;
						end
					end else if (data_wait_i && (!f_active_q || word_done_w)) begin
						// A data access breaks the chain; it restarts on demand.
						stopped_q <= 1'b1;
					end
				end
				// Maintenance may change the line a pending fill copied.
				if (ic_hold_i) fill_ok_q <= 1'b0;
			end
		end
	end
`ifndef SYNTHESIS
	// synthesis translate_off
	always @(posedge clk_i) begin
		if ((pin_own_q !== f_active_q) || (pin_own_c_q !== f_active_q) ||
			(pin_a_q !== {f_addr_q, f_beat_q}) ||
			(f_active_q && (pin_be_q !== (fixed_16_i ? 4'b1100 : (f_beat_q ? 4'b0011 : 4'b0000))))) begin
			$display("FAIL prefetch pin copy differs own=%b/%b a=%h/%h be=%b", pin_own_q, f_active_q,
				pin_a_q, {f_addr_q, f_beat_q}, pin_be_q);
			$fatal(1);
		end
	end
	// synthesis translate_on
`endif
endmodule
