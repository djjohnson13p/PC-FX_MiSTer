// HuC6271 R-RAM A/B control. One chip is read for display while the other
// takes the strip being decoded; they swap at each judgment. Filter B also
// reads the decode chip, only once its strip is complete. While frozen the
// savestate port drives either chip (address bit 13: 0 A, 1 B).

module huc6271_rram_ctl
(
	input  logic        disp,

	input  logic [12:0] rd_a,
	output logic  [7:0] rd_q,
	input  logic        fb_rd,
	input  logic [12:0] fb_a,
	output logic  [7:0] fb_q,
	input  logic        wr_v,
	input  logic [12:0] wr_a,
	input  logic  [7:0] wr_d,

	input  logic        ss_sel,
	input  logic [13:0] ss_addr,
	input  logic        ss_we,
	input  logic  [7:0] ss_wdata,
	output logic  [7:0] ss_rdata,

	output logic        rma_cs_n,
	output logic        rma_oe_n,
	output logic        rma_we_n,
	output logic [12:0] rma_a,
	input  logic  [7:0] rma_d_i,
	output logic  [7:0] rma_d_o,
	output logic        rma_d_oe,

	output logic        rmb_cs_n,
	output logic        rmb_oe_n,
	output logic        rmb_we_n,
	output logic [12:0] rmb_a,
	input  logic  [7:0] rmb_d_i,
	output logic  [7:0] rmb_d_o,
	output logic        rmb_d_oe
);

logic [1:0]       we;
logic [1:0][12:0] a;

always_comb begin
	for (int i = 0; i < 2; i++) begin
		if (ss_sel && ss_addr[13] == i[0]) begin
			a[i]  = ss_addr[12:0];
			we[i] = ss_we;
		end else if (disp == i[0]) begin
			a[i]  = rd_a;
			we[i] = 1'b0;
		end else begin
			a[i]  = fb_rd ? fb_a : wr_a;
			we[i] = wr_v && !fb_rd;
		end
	end
end

wire [7:0] wd = ss_sel ? ss_wdata : wr_d;

assign rma_cs_n = 1'b0;
assign rma_oe_n = we[0];
assign rma_we_n = !we[0];
assign rma_a    = a[0];
assign rma_d_o  = wd;
assign rma_d_oe = we[0];

assign rmb_cs_n = 1'b0;
assign rmb_oe_n = we[1];
assign rmb_we_n = !we[1];
assign rmb_a    = a[1];
assign rmb_d_o  = wd;
assign rmb_d_oe = we[1];

assign rd_q     = disp ? rmb_d_i : rma_d_i;
assign fb_q     = disp ? rma_d_i : rmb_d_i;
assign ss_rdata = ss_addr[13] ? rmb_d_i : rma_d_i;

endmodule
