`timescale 1ns/1ps
`default_nettype none

// MiSTer SDR SDRAM pin layer: JEDEC power-up, the command/address/data pin
// registers, the inverted DDIO clock forward and the DDIO DQ capture. It
// makes no scheduling decisions. `reset` is asynchronous; init_done rises
// once the power-up sequence and mode load are finished.
//
// After init_done, whatever the scheduler presents on cmd/ba/a/dq_out/dq_oe
// is registered onto the pins at the next clk edge, so a command decided in
// cycle t is on the pins from edge t and sampled by the chip half a clock
// later (SDRAM_CLK is ~clk). Read data for a READ registered at edge T is on
// dq_cell for the edge T + CL + the capture pipeline the scheduler counts,
// and on dq_in (dq_cap_q) for the edge after (see below).
//
// MiSTer boards have no DQM pins, so DQML/DQMH alias SDRAM_A[11]/A[12].
// Mode: burst length 1, write single, CL2 up to 100 MHz, CL3 above.
// SDRAM_TIMING_GRADE 7 = -7TIN 143 MHz part (default), 6 = -6TIN 166 MHz.
module sdram #(
	parameter int unsigned CLK_FREQ_HZ = 100_000_000,
	parameter int unsigned SDRAM_TIMING_GRADE = 7
) (
	input  wire        clk,
	input  wire        clk_cap,          // clk's rate, phase set by calibration
	input  wire        cap_rise,         // read the rising half of the input cell
`ifdef PCFX_SIM_CAPTURE
	// Bench: the input cell's outputs, and what each capture register's
	// route delivers of them on the coming clk edge.
	output wire [15:0] sim_cell_h,
	output wire [15:0] sim_cell_l,
	input  wire [15:0] sim_dq_h,
	input  wire [15:0] sim_dq_l,
	input  wire [15:0] sim_mr_h,
	input  wire [15:0] sim_mr_l,
`endif
	input  wire        reset,
	output logic       init_done,

	input  wire  [3:0] cmd,              // {nCS, nRAS, nCAS, nWE}
	input  wire  [1:0] ba,
	input  wire [12:0] a,
	input  wire [15:0] dq_out,
	input  wire        dq_oe,
	output wire [15:0] dq_in,
	output wire [15:0] dq_cell,          // the input cell's chosen half, unregistered

	output wire        SDRAM_CLK,
	output logic       SDRAM_CKE,
	output logic [12:0] SDRAM_A,
	output logic [1:0] SDRAM_BA,
	inout  wire [15:0] SDRAM_DQ,
	output wire        SDRAM_DQML,
	output wire        SDRAM_DQMH,
	output logic       SDRAM_nCS,
	output logic       SDRAM_nCAS,
	output logic       SDRAM_nRAS,
	output logic       SDRAM_nWE
);

	localparam logic [3:0] CMD_DESELECT      = 4'b1111;
	localparam logic [3:0] CMD_NOP           = 4'b0111;
	localparam logic [3:0] CMD_PRECHARGE     = 4'b0010;
	localparam logic [3:0] CMD_AUTO_REFRESH  = 4'b0001;
	localparam logic [3:0] CMD_LOAD_MODE     = 4'b0000;

	// Elaboration-time timing helper; the division constant-folds into
	// localparams.
	function automatic [15:0] cycles_for_ns;
		input [31:0] freq_hz;
		input [31:0] ns;
		reg [63:0] num;
		reg [63:0] cycles;
		begin
			num = ({32'd0, freq_hz} * {32'd0, ns}) + 64'd999_999_999;
			cycles = num / 64'd1_000_000_000;
			if (cycles < 64'd1) cycles_for_ns = 16'd1;
			else cycles_for_ns = cycles[15:0];
		end
	endfunction

	// After issuing a command in the current enabled cycle, wait this many
	// additional enabled cycles before the next dependent command/action.
	function automatic [15:0] count_after_command;
		input [15:0] cycles;
		begin
			if (cycles > 16'd1) count_after_command = cycles - 16'd1;
			else count_after_command = 16'd0;
		end
	endfunction

	// AS4C32M16SB supports CL2 or CL3. For both -6TIN and -7TIN, CL2 is
	// valid through 100 MHz; above that CL3 is programmed.
	localparam logic [2:0] CAS_LATENCY = (CLK_FREQ_HZ <= 100_000_000) ? 3'd2 : 3'd3;
	localparam logic MODE_WRITE_SINGLE = 1'b1;
	localparam logic [12:0] MODE_REG = {3'b000, MODE_WRITE_SINGLE, 2'b00, CAS_LATENCY, 1'b0, 3'b000};
	// Read capture pipeline (the scheduler's DQ_CAPTURE_PIPELINE), in clock
	// cycles between the edge that registers the READ command and the edge on
	// which the beat may be consumed:
	//
	//   T0        READ registered
	//   T0+0.5    SDRAM samples it on the rising pin edge (SDRAM_CLK is ~clk)
	//   T0+0.5+CL SDRAM launches the beat
	//   T0+CL+pipeline the beat is on dq_cell and goes into dq_cap_q
	//
	// Where in between the input cell takes the beat depends on the board;
	// the scheduler's calibration moves clk_cap and picks the cell's half so
	// the beat is there by that edge.

	// AS4C32M16SB timing values in nanoseconds. Grade 7 is the default
	// -7TIN/-7TCN board, 6 the faster -6TIN; any other value falls back to
	// the -7 profile.
	localparam logic TIMING_GRADE_6 = (SDRAM_TIMING_GRADE == 6);
	localparam logic [31:0] T_RP_NS = TIMING_GRADE_6 ? 32'd18 : 32'd21;
	localparam logic [31:0] T_RFC_NS = TIMING_GRADE_6 ? 32'd60 : 32'd63;
	localparam logic [31:0] T_MRD_NS = TIMING_GRADE_6 ? 32'd12 : 32'd14;

`ifndef SYNTHESIS
	// Elaboration guards: catch a clock the part cannot sustain at the chosen
	// CAS latency here rather than as intermittent read corruption on a board.
	localparam int unsigned CL2_MAX_HZ = 100_000_000;
	localparam int unsigned CL3_MAX_HZ = TIMING_GRADE_6 ? 166_000_000 : 143_000_000;
	initial begin
		if ((CAS_LATENCY == 3'd2) && (CLK_FREQ_HZ > CL2_MAX_HZ)) begin
			$error("sdram: CL2 is not valid above %0d Hz", CL2_MAX_HZ);
		end
		if ((CAS_LATENCY == 3'd3) && (CLK_FREQ_HZ > CL3_MAX_HZ)) begin
			$error("sdram: CLK_FREQ_HZ %0d exceeds the CL3 ceiling %0d for the configured speed grade",
				CLK_FREQ_HZ, CL3_MAX_HZ);
		end
		if ((SDRAM_TIMING_GRADE != 6) && (SDRAM_TIMING_GRADE != 7)) begin
			$display("sdram: SDRAM_TIMING_GRADE %0d is unknown; using the conservative -7 profile",
				SDRAM_TIMING_GRADE);
		end
	end
`endif

	// CKE stays low for the full 200 us datasheet startup interval; one NOP
	// cycle after CKE rises is sufficient before PRECHARGE ALL.
	localparam logic [15:0] T_POWER_WAIT = cycles_for_ns(CLK_FREQ_HZ[31:0], 32'd200_000);
	localparam logic [15:0] T_CKE_NOP = 16'd1;
	localparam logic [15:0] T_RP = cycles_for_ns(CLK_FREQ_HZ[31:0], T_RP_NS);
	localparam logic [15:0] T_RFC = cycles_for_ns(CLK_FREQ_HZ[31:0], T_RFC_NS);
	localparam logic [15:0] T_MRD = cycles_for_ns(CLK_FREQ_HZ[31:0], T_MRD_NS);
	localparam int unsigned WAIT_W = 6;

	function automatic [WAIT_W-1:0] wait_after;
		input [15:0] cycles;
		begin
			wait_after = WAIT_W'(count_after_command(cycles));
		end
	endfunction

	typedef enum logic [2:0] {
		ST_POWER_WAIT,
		ST_NOP_WAIT,
		ST_INIT_TRP,
		ST_INIT_RFC1,
		ST_INIT_RFC2,
		ST_INIT_MRD,
		ST_RUN
	} state_e;

	state_e state_q;
	logic run_q;                        // the pins follow the command port
	logic [3:0] i_cmd;                  // init sequence, a clock ahead of the pins
	logic [12:0] i_a;
	logic [WAIT_W-1:0] wait_q;
	logic [15:0] init_wait_q;
	logic [15:0] dq_out_q;
	logic dq_oe_q;
	wire [15:0] dq_in_w;

	// The output DDIO cell drives an exact inverted copy of clk to the pad. The
	// input DDIO cell runs on clk_cap and samples DQ on both of its edges;
	// cap_rise picks which half goes on. dq_cap_q, one register per bit
	// beside its cell, takes every beat on the edge it is due; readers copy
	// it from there. dq_cell is for the one reader that cannot wait a clock.
	// Each register gets its own half-select LUT, packed beside it: a shared
	// one reaches the second register through a feeder and makes its hop
	// 1.1-1.7 ns longer than its twin's.
	wire [15:0] dq_capture_h_w, dq_capture_l_w;   // the cell's halves, as dq_cap_q sees them
	wire [15:0] mr_capture_h_w, mr_capture_l_w;   // ... and as mr_q sees them
	(* keep *) wire [15:0] dq_cap_d = cap_rise ? dq_capture_h_w : dq_capture_l_w;
	(* keep *) wire [15:0] dq_cell_d = cap_rise ? mr_capture_h_w : mr_capture_l_w;
	(* preserve *) reg [15:0] dq_cap_q;
	assign dq_cell = dq_cell_d;
	always @(posedge clk) dq_cap_q <= dq_cap_d;
	assign dq_in = dq_cap_q;
`ifdef SYNTHESIS
	altddio_out #(
		.extend_oe_disable("OFF"),
		.intended_device_family("Cyclone V"),
		.invert_output("OFF"),
		.lpm_hint("UNUSED"),
		.lpm_type("altddio_out"),
		.oe_reg("UNREGISTERED"),
		.power_up_high("OFF"),
		.width(1)
	) u_sdram_clock (
		.datain_h(1'b0),
		.datain_l(1'b1),
		.outclock(clk),
		.dataout(SDRAM_CLK),
		.aclr(1'b0),
		.aset(1'b0),
		.oe(1'b1),
		.outclocken(1'b1),
		.sclr(1'b0),
		.sset(1'b0)
	);

	altddio_in #(
		.intended_device_family("Cyclone V"),
		.invert_input_clocks("OFF"),
		.lpm_hint("UNUSED"),
		.lpm_type("altddio_in"),
		.power_up_high("OFF"),
		.width(16)
	) u_sdram_dq_capture (
		.datain(dq_in_w),
		.inclock(clk_cap),
		.inclocken(1'b1),
		.aclr(1'b0),
		.aset(1'b0),
		.sclr(1'b0),
		.sset(1'b0),
		.dataout_h(dq_capture_h_w),
		.dataout_l(dq_capture_l_w)
	);
	assign mr_capture_h_w = dq_capture_h_w;
	assign mr_capture_l_w = dq_capture_l_w;
`else
	// Simulation only: the input cell as the device builds it. The falling
	// half is moved onto the next rising clk_cap edge (the cell's alignment
	// register); the rising half comes straight out. With PCFX_SIM_CAPTURE the
	// bench plays each route from the cell into the core, bit by bit.
	reg [15:0] dq_capture_l_q, dq_capture_h_q, dq_align_l_q;

	assign SDRAM_CLK = ~clk;

	always @(negedge clk_cap) dq_capture_l_q <= dq_in_w;
	always @(posedge clk_cap) begin
		dq_capture_h_q <= dq_in_w;
		dq_align_l_q   <= dq_capture_l_q;
	end
`ifdef PCFX_SIM_CAPTURE
	assign sim_cell_h     = dq_capture_h_q;
	assign sim_cell_l     = dq_align_l_q;
	assign dq_capture_h_w = sim_dq_h;
	assign dq_capture_l_w = sim_dq_l;
	assign mr_capture_h_w = sim_mr_h;
	assign mr_capture_l_w = sim_mr_l;
`else
	assign dq_capture_l_w = dq_align_l_q;
	assign dq_capture_h_w = dq_capture_h_q;
	assign mr_capture_l_w = dq_align_l_q;
	assign mr_capture_h_w = dq_capture_h_q;
`endif
`endif

	// DQM outputs are exact aliases of A11/A12, matching the board wiring.
	assign SDRAM_DQML = SDRAM_A[11];
	assign SDRAM_DQMH = SDRAM_A[12];

	// Data and OE are both registered so Quartus can place the final stage in
	// the SDRAM IOE.
	assign dq_in_w = SDRAM_DQ;
	assign SDRAM_DQ = dq_oe_q ? dq_out_q : 16'hZZZZ;

	assign init_done = run_q;

	// Reset is asynchronous on purpose: Cyclone V SDRAM output and
	// output-enable registers cannot pack into the IOE with a synchronous
	// clear. The init sequence is built in fabric registers a clock ahead, so
	// each pin register only chooses between them and the command port.
	always_ff @(posedge clk or posedge reset) begin
		if (reset) begin
			run_q <= 1'b0;
			SDRAM_A <= 13'd0;
			SDRAM_BA <= 2'd0;
			{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_DESELECT;
			dq_out_q <= 16'd0;
			dq_oe_q <= 1'b0;
		end else begin
			run_q <= (state_q == ST_RUN);
			{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= run_q ? cmd : i_cmd;
			SDRAM_BA <= run_q ? ba : 2'd0;
			SDRAM_A <= run_q ? a : i_a;
			dq_out_q <= dq_out;
			dq_oe_q <= run_q && dq_oe;
		end
	end

	always_ff @(posedge clk or posedge reset) begin
		if (reset) begin
			state_q <= ST_POWER_WAIT;
			wait_q <= {WAIT_W{1'b0}};
			init_wait_q <= count_after_command(T_POWER_WAIT);
			SDRAM_CKE <= 1'b0;
			i_a <= 13'd0;
			i_cmd <= CMD_DESELECT;
		end else begin
			if (wait_q != {WAIT_W{1'b0}})
				wait_q <= wait_q - {{(WAIT_W-1){1'b0}}, 1'b1};
			i_cmd <= CMD_NOP;

			unique case (state_q)
				ST_POWER_WAIT: begin
					// JEDEC init: hold CKE low, then CKE high with NOPs,
					// precharge all banks, issue two refreshes, load mode.
					i_cmd <= CMD_DESELECT;
					if (init_wait_q != 16'd0) begin
						init_wait_q <= init_wait_q - 16'd1;
					end else begin
						SDRAM_CKE <= 1'b1;
						state_q <= ST_NOP_WAIT;
						wait_q <= wait_after(T_CKE_NOP);
					end
				end

				ST_NOP_WAIT: begin
					if (wait_q == {WAIT_W{1'b0}}) begin
						i_a <= 13'h0400;
						i_cmd <= CMD_PRECHARGE;
						state_q <= ST_INIT_TRP;
						wait_q <= wait_after(T_RP);
					end
				end

				ST_INIT_TRP: begin
					if (wait_q == {WAIT_W{1'b0}}) begin
						i_a <= 13'd0;
						i_cmd <= CMD_AUTO_REFRESH;
						state_q <= ST_INIT_RFC1;
						wait_q <= wait_after(T_RFC);
					end
				end

				ST_INIT_RFC1: begin
					if (wait_q == {WAIT_W{1'b0}}) begin
						i_cmd <= CMD_AUTO_REFRESH;
						state_q <= ST_INIT_RFC2;
						wait_q <= wait_after(T_RFC);
					end
				end

				ST_INIT_RFC2: begin
					if (wait_q == {WAIT_W{1'b0}}) begin
						i_a <= MODE_REG;
						i_cmd <= CMD_LOAD_MODE;
						state_q <= ST_INIT_MRD;
						wait_q <= wait_after(T_MRD);
					end
				end

				ST_INIT_MRD: begin
					i_a <= 13'd0;
					if (wait_q == {WAIT_W{1'b0}})
						state_q <= ST_RUN;
				end

				default: ;
			endcase
		end
	end

endmodule

`default_nettype wire
