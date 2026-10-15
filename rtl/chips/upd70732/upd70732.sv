// NEC uPD70732 (V810) at package polarity around the NECv810 core.
//
// Pins follow the data sheet pin table; a bar there is _n here. MRQ is
// active only on memory cycles, so an I/O cycle has MRQ inactive. Every pin
// comes from a flop: a bus cycle shows T1 (BCYST) and T2 (DA) on the pins,
// READY and SZRQ are sampled at the CLK fall in T2 (phi1), data at the CLK
// rise that ends it, as in the data sheet's AC tables.

module upd70732 #(
	parameter bit DEBUG = 1'b0
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_clk,
	input  logic        phi1,

	output logic [31:1] a,
	output logic        a_oe,
	input  logic [31:0] d_i,
	output logic [31:0] d_o,
	output logic  [3:0] d_oe,
	output logic  [3:0] be_n,
	output logic  [3:0] be_oe,
	output logic  [1:0] st,
	output logic        st_oe,
	output logic        da_n,
	output logic        da_oe,
	output logic        mrq_n,
	output logic        mrq_oe,
	output logic        rw,
	output logic        rw_oe,
	output logic        bcyst_n,
	output logic        bcyst_oe,
	input  logic        ready_n,
	input  logic        hldrq_n,
	output logic        hldak_n,
	input  logic        szrq_n,
	input  logic        siz16b,
	output logic        block,
	input  logic        icheen_n,
	input  logic        intr,
	input  logic  [3:0] intv_n,
	input  logic        nmi_n,
	output logic        adrserr_n,

	input  logic        ss_pause_req,
	output logic        ss_pause_ready,
	input  logic        ss_restore,
	input  logic  [3:0] ss_state_addr,
	input  logic [63:0] ss_state_wdata,
	input  logic        ss_state_wren,
	output logic [63:0] ss_state_rdata,
	input  logic        ss_mem_active,
	input  logic [10:0] ss_mem_addr,
	input  logic        ss_mem_rden,
	input  logic        ss_mem_wren,
	input  logic  [7:0] ss_mem_wdata,
	output logic  [7:0] ss_mem_rdata,

	output logic [31:0] dbg_pc,
	output logic [31:0] dbg_psw,
	output logic        dbg_retire,
	output logic [31:0] dbg_retire_pc,
	output logic [15:0] dbg_retire_insn,
	output logic        dbg_halted,
	output logic        dbg_illegal
);

logic        da, mrq, bcyst, hldak;
logic [31:0] pc, psw, retire_pc;
logic        retire, halted, illegal;
logic [15:0] retire_insn;

/* verilator lint_off UNUSEDSIGNAL */
logic        dout_oe, cycle_tag;
logic [31:0] reg_data;
/* verilator lint_on UNUSEDSIGNAL */

NECv810 #(
	.IO_MRQ     (1),
	.T2_PINS    (1),
	.VB_EXT     (0)
) u_cpu (
	.clk_i                   (clk),
	.reset_i                 (reset),
	.ce_i                    (ce_clk),
	.phi1_i                  (phi1),
	.ready_i                 (!ready_n),
	.din_i                   (d_i),
	.hldrq_i                 (!hldrq_n),
	.icheen_i                (!icheen_n),
	.siz16b_i                (siz16b),
	.szrq_i                  (szrq_n),
	.irq_valid_i             (intr),
	.irq_level_i             (~intv_n),
	.nmi_i                   (!nmi_n),
	.savestate_pause_req_i   (ss_pause_req),
	.savestate_pause_ready_o (ss_pause_ready),
	.savestate_restore_i     (ss_restore),
	.savestate_state_addr_i  (ss_state_addr),
	.savestate_state_wdata_i (ss_state_wdata),
	.savestate_state_wren_i  (ss_state_wren),
	.savestate_state_rdata_o (ss_state_rdata),
	.savestate_mem_active_i  (ss_mem_active),
	.savestate_mem_addr_i    (ss_mem_addr),
	.savestate_mem_rden_i    (ss_mem_rden),
	.savestate_mem_wren_i    (ss_mem_wren),
	.savestate_mem_wdata_i   (ss_mem_wdata),
	.savestate_mem_rdata_o   (ss_mem_rdata),
	.a_o                     (a),
	.a_oe_o                  (a_oe),
	.dout_o                  (d_o),
	.dout_oe_o               (dout_oe),
	.dout_lane_oe_o          (d_oe),
	.be_o                    (be_n),
	.be_oe_o                 (be_oe),
	.st_o                    (st),
	.st_oe_o                 (st_oe),
	.da_o                    (da),
	.da_oe_o                 (da_oe),
	.mrq_o                   (mrq),
	.mrq_oe_o                (mrq_oe),
	.rw_o                    (rw),
	.rw_oe_o                 (rw_oe),
	.bcyst_o                 (bcyst),
	.bcyst_oe_o              (bcyst_oe),
	.adrs_err_n_o            (adrserr_n),
	.cycle_tag_o             (cycle_tag),
	.block_o                 (block),
	.hldak_o                 (hldak),
	.dbg_reg_addr_i          (5'd0),
	.dbg_reg_data_o          (reg_data),
	.dbg_pc_o                (pc),
	.dbg_psw_o               (psw),
	.trace_valid_o           (retire),
	.trace_pc_o              (retire_pc),
	.trace_insn_o            (retire_insn),
	.halted_o                (halted),
	.illegal_o               (illegal)
);

assign da_n    = !da;
assign mrq_n   = !mrq;
assign bcyst_n = !bcyst;
assign hldak_n = !hldak;

assign dbg_pc          = DEBUG ? pc : 32'd0;
assign dbg_psw         = DEBUG ? psw : 32'd0;
assign dbg_retire      = DEBUG ? retire : 1'b0;
assign dbg_retire_pc   = DEBUG ? retire_pc : 32'd0;
assign dbg_retire_insn = DEBUG ? retire_insn : 16'd0;
assign dbg_halted      = DEBUG ? halted : 1'b0;
assign dbg_illegal     = DEBUG ? illegal : 1'b0;

endmodule
