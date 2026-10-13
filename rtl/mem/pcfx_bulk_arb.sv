// The memory ports that work only with the board stopped, shared by the
// BIOS loader (0), the backup-file service (1) and savestates (2):
// pcfx_memory's bulk port and the mainboard's ss_mem port.
//
// A client asks with own (the loader's bk_req serves as its own) and keeps
// the grant until own drops; the owner changes only with bk_req and bk_ack
// both low, priority 0 > 1 > 2. Only savestates stop the board: pause_req
// goes to the run/freeze control, which answers on paused, and the engine
// starts once paused && gnt (a backup copy holding the grant never waits on
// the board). Read data (bk_q, ss_mem_rdata) goes to every client; only the
// owner sees bk_ack.

module pcfx_bulk_arb
(
	input  logic        clk,
	input  logic        reset,

	input  logic        c0_req,
	input  logic        c0_we,
	input  logic [23:0] c0_addr,
	input  logic [15:0] c0_d,
	input  logic  [1:0] c0_be,
	output logic        c0_ack,

	input  logic        c1_own,
	output logic        c1_gnt,
	input  logic        c1_req,
	input  logic        c1_we,
	input  logic [23:0] c1_addr,
	input  logic [15:0] c1_d,
	input  logic  [1:0] c1_be,
	output logic        c1_ack,
	input  logic  [2:0] c1_ssm_chip,
	input  logic        c1_ssm_sel,
	input  logic [16:0] c1_ssm_addr,
	input  logic        c1_ssm_we,
	input  logic [15:0] c1_ssm_wdata,

	input  logic        c2_own,
	output logic        c2_gnt,
	input  logic        c2_req,
	input  logic        c2_we,
	input  logic [23:0] c2_addr,
	input  logic [15:0] c2_d,
	input  logic  [1:0] c2_be,
	output logic        c2_ack,
	input  logic  [2:0] c2_ssm_chip,
	input  logic        c2_ssm_sel,
	input  logic [16:0] c2_ssm_addr,
	input  logic        c2_ssm_we,
	input  logic [15:0] c2_ssm_wdata,

	output logic        bk_req,
	output logic        bk_we,
	output logic [23:0] bk_addr,
	output logic [15:0] bk_d,
	output logic  [1:0] bk_be,
	input  logic        bk_ack,

	output logic  [2:0] ss_mem_chip,
	output logic        ss_mem_sel,
	output logic [16:0] ss_mem_addr,
	output logic        ss_mem_we,
	output logic [15:0] ss_mem_wdata,

	output logic        pause_req
);

typedef enum logic [1:0] {NONE, LOADER, SAVES, STATES} owner_t;
owner_t owner;

wire idle = !bk_req && !bk_ack;
wire held = (owner == LOADER && c0_req) || (owner == SAVES && c1_own) || (owner == STATES && c2_own);

always_ff @(posedge clk) begin
	if (reset)
		owner <= NONE;
	else if (idle && !held)
		owner <= c0_req ? LOADER : c1_own ? SAVES : c2_own ? STATES : NONE;
end

always_comb begin
	{bk_req, bk_we, bk_addr, bk_d, bk_be} = '0;
	{ss_mem_chip, ss_mem_sel, ss_mem_addr, ss_mem_we, ss_mem_wdata} = '0;
	case (owner)
		LOADER: {bk_req, bk_we, bk_addr, bk_d, bk_be} = {c0_req, c0_we, c0_addr, c0_d, c0_be};
		SAVES: begin
			{bk_req, bk_we, bk_addr, bk_d, bk_be} = {c1_req, c1_we, c1_addr, c1_d, c1_be};
			{ss_mem_chip, ss_mem_sel, ss_mem_addr, ss_mem_we, ss_mem_wdata} =
				{c1_ssm_chip, c1_ssm_sel, c1_ssm_addr, c1_ssm_we, c1_ssm_wdata};
		end
		STATES: begin
			{bk_req, bk_we, bk_addr, bk_d, bk_be} = {c2_req, c2_we, c2_addr, c2_d, c2_be};
			{ss_mem_chip, ss_mem_sel, ss_mem_addr, ss_mem_we, ss_mem_wdata} =
				{c2_ssm_chip, c2_ssm_sel, c2_ssm_addr, c2_ssm_we, c2_ssm_wdata};
		end
		default: ;
	endcase
end

assign c0_ack    = bk_ack && owner == LOADER;
assign c1_ack    = bk_ack && owner == SAVES;
assign c2_ack    = bk_ack && owner == STATES;
assign c1_gnt    = owner == SAVES;
assign c2_gnt    = owner == STATES;
assign pause_req = c2_own;

endmodule
