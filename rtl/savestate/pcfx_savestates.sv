// Savestates for the top level: OSD and hotkey control and the slot manager
// (both as in VirtualBoy_MiSTer), the engine and its DDR3 port. Four slots
// of 8 MB from DDR3 0x3800_0000 (CONF_STR SS38000000:800000).

module pcfx_savestates
(
	input  logic        clk,
	input  logic        reset,

	input  logic [10:0] ps2_key,
	input  logic        allow_ss,
	input  logic        joy_ss,          // hold for pad control
	input  logic        joy_right,
	input  logic        joy_left,
	input  logic        joy_down,
	input  logic        joy_up,
	input  logic        joy_start,
	input  logic        joy_save,
	input  logic  [1:0] status_slot,
	input  logic  [1:0] osd_saveload,    // {load, save} OSD toggles
	input  logic        count_save,      // off: the HPS does not write the file
	output logic        info_req,
	output logic  [7:0] info,
	output logic        status_update,
	output logic  [1:0] slot,
	output logic        busy,
	output logic        load_bad,

	input  logic        ss_window,
	input  logic        ss_busy,
	input  logic        cd_ss_busy,
	output logic        cpu_pause_req,
	input  logic        cpu_pause_ready,
	output logic        cpu_restore,
	output logic  [3:0] cpu_state_addr,
	output logic [63:0] cpu_state_wdata,
	output logic        cpu_state_wren,
	input  logic [63:0] cpu_state_rdata,
	output logic        cpu_mem_active,
	output logic [10:0] cpu_mem_addr,
	output logic        cpu_mem_rden,
	output logic        cpu_mem_wren,
	output logic  [7:0] cpu_mem_wdata,
	input  logic  [7:0] cpu_mem_rdata,
	output logic        own,
	input  logic        gnt,
	input  logic        paused,
	output logic        ss_shift,
	output logic        ss_out,
	input  logic        ss_in,
	output logic  [2:0] ssm_chip,
	output logic        ssm_sel,
	output logic [16:0] ssm_addr,
	output logic        ssm_we,
	output logic [15:0] ssm_wdata,
	input  logic [15:0] ssm_rdata,
	output logic        bk_req,
	output logic        bk_we,
	output logic [23:0] bk_addr,
	output logic [15:0] bk_d,
	output logic  [1:0] bk_be,
	input  logic        bk_ack,
	input  logic [15:0] bk_q,
	input  logic [10:0] pg_q,
	output logic        pg_we,
	output logic [10:0] pg_d,
	input  logic [31:0] bios_crc,
	input  logic [31:0] toc_hash,

	input  logic        DDRAM_BUSY,
	output logic  [7:0] DDRAM_BURSTCNT,
	output logic [28:0] DDRAM_ADDR,
	input  logic [63:0] DDRAM_DOUT,
	input  logic        DDRAM_DOUT_READY,
	output logic        DDRAM_RD,
	output logic [63:0] DDRAM_DIN,
	output logic  [7:0] DDRAM_BE,
	output logic        DDRAM_WE
);

logic   ui_save, ui_load, req_save, req_load;
/* verilator lint_off UNUSEDSIGNAL */
integer req_addr;            // word offset, 25 bits used
logic   no_rewind;
/* verilator lint_on UNUSEDSIGNAL */

savestate_ui ui (
	.clk           (clk),
	.ps2_key       (ps2_key),
	.allow_ss      (allow_ss),
	.joySS         (joy_ss),
	.joyRight      (joy_right),
	.joyLeft       (joy_left),
	.joyDown       (joy_down),
	.joyUp         (joy_up),
	.joyStart      (joy_start),
	.joySaveState  (joy_save),
	.status_slot   (status_slot),
	.OSD_saveload  (osd_saveload),
	.ss_save       (ui_save),
	.ss_load       (ui_load),
	.ss_info_req   (info_req),
	.ss_info       (info),
	.statusUpdate  (status_update),
	.selected_slot (slot)
);

// Slot n at 64-bit word 0x100_0000 + n << 20 of the 0x3000_0000 window.
statemanager #(
	.Softmap_SaveState_ADDR (32'h0100_0000),
	.Softmap_Rewind_ADDR    (32'h0100_0000),
	.SAVESTATE_SHIFT        (20)
) slots (
	.clk               (clk),
	.reset             (reset),
	.rewind_on         (1'b0),
	.rewind_active     (1'b0),
	.savestate_number  ({30'd0, slot}),
	.save              (ui_save),
	.load              (ui_load),
	.sleep_rewind      (no_rewind),
	.vsync             (1'b0),
	.request_savestate (req_save),
	.request_loadstate (req_load),
	.request_address   (req_addr),
	.request_busy      (busy)
);

logic        dd_req, dd_we, dd_ack;
logic [24:0] dd_addr;
logic [63:0] dd_wdata, dd_rdata;
logic  [7:0] dd_be;

pcfx_ss_engine engine (
	.clk, .reset,
	.save (req_save), .load (req_load), .slot_addr (25'(req_addr)), .count_save,
	.busy, .load_bad,
	.ss_window, .ss_busy, .cd_ss_busy,
	.cpu_pause_req, .cpu_pause_ready, .cpu_restore,
	.cpu_state_addr, .cpu_state_wdata, .cpu_state_wren, .cpu_state_rdata,
	.cpu_mem_active, .cpu_mem_addr, .cpu_mem_rden, .cpu_mem_wren, .cpu_mem_wdata, .cpu_mem_rdata,
	.own, .gnt, .paused,
	.ss_shift, .ss_out, .ss_in,
	.ssm_chip, .ssm_sel, .ssm_addr, .ssm_we, .ssm_wdata, .ssm_rdata,
	.bk_req, .bk_we, .bk_addr, .bk_d, .bk_be, .bk_ack, .bk_q,
	.pg_q, .pg_we, .pg_d,
	.bios_crc, .toc_hash,
	.dd_req, .dd_we, .dd_addr, .dd_wdata, .dd_be, .dd_ack, .dd_rdata
);

pcfx_ss_ddram ddram (
	.clk, .reset, .req (dd_req), .we (dd_we), .addr (dd_addr), .wdata (dd_wdata), .be (dd_be),
	.ack (dd_ack), .rdata (dd_rdata),
	.DDRAM_BUSY, .DDRAM_BURSTCNT, .DDRAM_ADDR, .DDRAM_DOUT, .DDRAM_DOUT_READY,
	.DDRAM_RD, .DDRAM_DIN, .DDRAM_BE, .DDRAM_WE
);

endmodule
