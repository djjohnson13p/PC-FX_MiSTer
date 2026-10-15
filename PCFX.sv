//============================================================================
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S = 1;
assign AUDIO_MIX = status[26:25];

assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

// Status bits; add new menu items here first.
//   0 reset (R)           2 eject FX-BMP card (R)  3-4 port 1     5-6 port 2
//   7-8 pad modes         9-10 savestate slot      11-12 save/restore (R)
//   13 savestates to SD   14-16 scandoubler fx     17 vertical crop
//   18-21 crop offset     22-23 scale              24 YUV matrix  25-26 stereo mix
//   28 ADPCM compat off   29 ADPCM smoothing       30 new FX-BMP card (R)
//   31-32 CD speed        33-38 capture window     39-44 capture phase
//   45 capture edge       46 capture found         121-122 aspect ratio
//   free: 1, 27, 47-120, 123-127
`include "build_id.v"
localparam CONF_STR = {
	"PCFX;SS38000000:800000;",
	"-;",
	"S0,CUECHDISO,CD-ROM;",
	"SC2,BMPFXB,FX-BMP card;",
	"R[30],Create new FX-BMP card;",
	"R[2],Eject FX-BMP card;",
	"-;",
	"P1,Audio & Video;",
	"P1-;",
	"P1O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"P1O[23:22],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
	"P1O[16:14],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"P1-;",
	"d0P1O[17],Vertical Crop,Disabled,216p(5x);",
	"d0P1O[21:18],Crop Offset,0,2,4,6,8,10,12,-12,-10,-8,-6,-4,-2;",
	"P1-;",
	"P1O[24],YUV Matrix,Analog (BT.601),JFIF;",
	"P1O[26:25],Stereo Mix,None,25%,50%,100%;",
	"-;",
	"O[4:3],Port 1,Pad,Mouse,Multitap,None;",
	"O[6:5],Port 2,Pad,Mouse,None;",
	"O[7],Pad MODE 1,A,B;",
	"O[8],Pad MODE 2,A,B;",
	"-;",
	"O[10:9],Savestate Slot,1,2,3,4;",
	"R[11],Save State (Alt+F1-F4);",
	"R[12],Restore State (F1-F4);",
	"O[13],Savestates to SD,On,Off;",
	"-;",
	"O[28],ADPCM Encoder Compat,On,Off;",
	"O[29],ADPCM Reset Clicks,Hardware,Smoothed;",
	"O[32:31],CD Speed,Original,Fast (4x),Faster (8x);",
	"P2,SDRAM Capture;",
	"P2-;",
	"D1P2O[46],Window found,No,Yes;",
	"D1P2O[38:33],Window (steps),0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63;",
	"D1P2O[44:39],Phase (steps),0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"D1P2O[45],Capture edge,Falling,Rising;",
	"-;",
	"R[0],Reset;",
	"I,",
	"-,",
	"-,",
	"-,",
	"-,",
	"Slot=DPAD|Save/Load=Start+DPAD,",
	"Active Slot 1,",
	"Active Slot 2,",
	"Active Slot 3,",
	"Active Slot 4,",
	"Save to state 1,",
	"Restore state 1,",
	"Save to state 2,",
	"Restore state 2,",
	"Save to state 3,",
	"Restore state 3,",
	"Save to state 4,",
	"Restore state 4;",
	"J1,I,II,III,IV,V,VI,Select,Run,Save State;",
	"jn,A,B,Y,X,L,R,Select,Start;",
	"v,1;",
	"V,v",`BUILD_DATE
};

wire forced_scandoubler;
wire  [21:0] gamma_bus;
wire         en216p;
wire   [1:0] buttons;
wire [127:0] status;
wire         ioctl_download;
wire  [15:0] ioctl_index;
wire         ioctl_wr;
wire  [26:0] ioctl_addr;
wire   [7:0] ioctl_dout;
wire         ioctl_wait;
wire   [5:0] cd_sd_blk_cnt;
wire   [2:0] dbg_led;
// Slots: 0 CD, 1 backup.sav, 2 fxbmp.sav (Main mounts 1 and 2 itself).
wire  [31:0] sd_lba[3];
wire   [2:0] sd_rd, sd_wr, sd_ack;
wire  [13:0] sd_buff_addr;
wire   [7:0] sd_buff_dout;
wire   [7:0] sd_buff_din[3];
wire         sd_buff_wr;
wire   [2:0] img_mounted;
wire         img_readonly;
wire  [63:0] img_size;
wire  [31:0] joy0, joy1, joy2, joy3, joy4;
wire  [24:0] ps2_mouse;
wire  [10:0] ps2_key;
wire   [1:0] ss_slot;
wire   [5:0] cal_width, cal_phase;
wire         cal_rise, cal_ok, cal_done;
wire         ss_status_set, ss_info_req;
wire   [7:0] ss_info;

hps_io #(.CONF_STR(CONF_STR), .VDNUM(3)) hps_io
(
	.clk_sys            (clk_sys),
	.HPS_BUS            (HPS_BUS),
	.EXT_BUS            (),
	.gamma_bus          (gamma_bus),

	.forced_scandoubler (forced_scandoubler),

	.buttons            (buttons),
	.status             (status),
	.status_menumask    ({14'd0, 1'b1, en216p}),
	.status_in          ({status[127:47], cal_ok, cal_rise, cal_phase, cal_width, status[32:11], ss_slot, status[8:0]}),
	.status_set         (ss_status_set | cal_done),
	.info_req           (ss_info_req),
	.info               (ss_info),

	.joystick_0         (joy0),
	.joystick_1         (joy1),
	.joystick_2         (joy2),
	.joystick_3         (joy3),
	.joystick_4         (joy4),
	.ps2_mouse          (ps2_mouse),
	.ps2_key            (ps2_key),

	.ps2_kbd_led_use    (3'b111),
	.ps2_kbd_led_status (dbg_led),

	.ioctl_download     (ioctl_download),
	.ioctl_index        (ioctl_index),
	.ioctl_wr           (ioctl_wr),
	.ioctl_addr         (ioctl_addr),
	.ioctl_dout         (ioctl_dout),
	.ioctl_wait         (ioctl_wait),

	.sd_lba             (sd_lba),
	.sd_blk_cnt         ('{cd_sd_blk_cnt, 6'd0, 6'd0}),
	.sd_rd              (sd_rd),
	.sd_wr              (sd_wr),
	.sd_ack             (sd_ack),
	.sd_buff_addr       (sd_buff_addr),
	.sd_buff_dout       (sd_buff_dout),
	.sd_buff_din        (sd_buff_din),
	.sd_buff_wr         (sd_buff_wr),
	.img_mounted        (img_mounted),
	.img_readonly       (img_readonly),
	.img_size           (img_size)
);

///////////////////////   CLOCKS   ///////////////////////////////

// clk_sys is the board oscillator; clk_sdram is 3x it and clk_vid 2x it,
// edges aligned. clk_cap runs with clk_sdram; its phase is stepped at run
// time (one step is 1/8 of the 515.45 MHz VCO period, about 242 ps).
wire clk_sys, clk_sdram, clk_vid, clk_cap, pll_locked;
wire dps_phase_en, dps_updn, dps_phase_done;
pll pll
(
	.refclk     (CLK_50M),
	.rst        (0),
	.outclk_0   (clk_sys),        // 42.954545 MHz
	.outclk_1   (clk_sdram),      // 128.863636 MHz
	.outclk_2   (clk_vid),        // 85.909091 MHz
	.outclk_3   (clk_cap),        // 128.863636 MHz, phase stepped
	.locked     (pll_locked),
	.scanclk    (clk_sys),
	.cntsel     (5'd3),           // outclk_3
	.phase_en   (dps_phase_en),
	.updn       (dps_updn),
	.phase_done (dps_phase_done)
);

///////////////////////   BOOT   /////////////////////////////////

// The board stays in reset with memory frozen until the SDRAM is up and a
// BIOS is loaded. Memory thaws first; the board runs once it has.
wire bios_loaded, frozen;
wire bios_dl = ioctl_download && ioctl_index == 16'd0;
wire bk_loading, pause_req;
wire hold    = RESET | status[0] | buttons[1] | bios_dl | ~bios_loaded | bk_loading;

reg       board_reset = 1'b1;
reg       run         = 1'b0;
reg       freeze      = 1'b1;
reg [19:0] hold_cnt   = 20'hFFFFF;

always @(posedge clk_sys) begin
	if (hold) begin
		board_reset <= 1'b1;
		run         <= 1'b0;
		freeze      <= 1'b1;
		hold_cnt    <= 20'hFFFFF;
	end else if (hold_cnt != 20'd0) begin
		// Any reset lasts about 24 ms, like the power switch.
		hold_cnt <= hold_cnt - 20'd1;
	end else if (pause_req) begin
		// Stopped, not reset, while a savestate is copied.
		run    <= 1'b0;
		freeze <= 1'b1;
	end else if (freeze) begin
		if (frozen) freeze <= 1'b0;
	end else if (!frozen) begin
		// Together with run, so the first clock out of reset carries ce_sck.
		board_reset <= 1'b0;
		run         <= 1'b1;
	end
end

wire        paused = freeze && frozen && !run;

// Boot hold state on the keyboard LEDs: caps = hold (blinks for a pause),
// num = freeze, scroll = frozen.
reg  [23:0] led_blink = 24'd0;
always @(posedge clk_sys) led_blink <= led_blink + 24'd1;
assign dbg_led = {frozen, freeze, hold | (pause_req & led_blink[23])};
wire        bk_req, bk_we, bk_ack, ld_req, ld_we, ld_ack, sv_req, sv_we, sv_ack;
wire [23:0] bk_addr, ld_addr, sv_addr;
wire [15:0] bk_d, ld_d, sv_d, bk_q;
wire  [1:0] bk_be, ld_be, sv_be;
wire [31:0] bios_crc;

bios_loader bios_loader
(
	.clk            (clk_sys),
	.reset          (~pll_locked),
	.ioctl_download (ioctl_download),
	.ioctl_index    (ioctl_index),
	.ioctl_wr       (ioctl_wr),
	.ioctl_addr     (ioctl_addr),
	.ioctl_dout     (ioctl_dout),
	.ioctl_wait     (ioctl_wait),
	.bk_req         (ld_req),
	.bk_we          (ld_we),
	.bk_addr        (ld_addr),
	.bk_d           (ld_d),
	.bk_be          (ld_be),
	.bk_ack         (ld_ack),
	.loaded         (bios_loaded),
	.crc            (bios_crc)
);

// Internal backup SRAM on slot 1 and the FX-BMP card on slot 2.
wire        sv_mem_sel, sv_mem_we, sv_own, sv_gnt, bram_we;
wire        fx_cs_n, fx_wr_n;
wire [14:0] sv_mem_addr;
wire  [7:0] sv_mem_wdata, sv_din;
wire [15:0] ss_mem_rdata;
wire [31:0] sv_lba;
wire        bg_req, bg_we, bg_ack, card_in;
wire [15:0] bg_addr, bg_d, bg_q;

save_slot save_slot
(
	.clk          (clk_sys),
	.reset        (~pll_locked),
	.img_mounted  (img_mounted[2:1]),
	.img_readonly (img_readonly),
	.img_size     (img_size),
	.sd_lba       (sv_lba),
	.sd_rd        (sd_rd[2:1]),
	.sd_wr        (sd_wr[2:1]),
	.sd_ack       (sd_ack[2:1]),
	.sd_buff_addr (sd_buff_addr[8:0]),
	.sd_buff_dout (sd_buff_dout),
	.sd_buff_din  (sv_din),
	.sd_buff_wr   (sd_buff_wr),
	.osd_status   (OSD_STATUS),
	.bram_we      (bram_we),
	.fx_we        (!fx_cs_n && !fx_wr_n),
	.running      (run),
	.loading      (bk_loading),
	.card_in      (card_in),
	.paused       (paused),
	.own_req      (sv_own),
	.gnt          (sv_gnt),
	.mem_sel      (sv_mem_sel),
	.mem_addr     (sv_mem_addr),
	.mem_we       (sv_mem_we),
	.mem_wdata    (sv_mem_wdata),
	.mem_rdata    (ss_mem_rdata[7:0]),
	.bk_req       (sv_req),
	.bk_we        (sv_we),
	.bk_addr      (sv_addr),
	.bk_d         (sv_d),
	.bk_be        (sv_be),
	.bk_ack       (sv_ack),
	.bg_req       (bg_req),
	.bg_we        (bg_we),
	.bg_addr      (bg_addr),
	.bg_d         (bg_d),
	.bg_ack       (bg_ack),
	.bg_q         (bg_q)
);

assign sd_lba[1]      = sv_lba;
assign sd_lba[2]      = sv_lba;
assign sd_buff_din[1] = sv_din;
assign sd_buff_din[2] = sv_din;

// The memory ports that need the board stopped: loader, backup files,
// savestates.
wire        st_own, st_gnt, st_req, st_we, st_ack, st_mem_sel, st_mem_we;
wire [23:0] st_addr;
wire [15:0] st_d, st_mem_wdata;
wire  [1:0] st_be;
wire  [2:0] st_mem_chip, ss_mem_chip;
wire [16:0] st_mem_addr, ss_mem_addr;
wire        ss_mem_sel, ss_mem_we;
wire [15:0] ss_mem_wdata;

pcfx_bulk_arb pcfx_bulk_arb
(
	.clk          (clk_sys),
	.reset        (~pll_locked),
	.c0_req       (ld_req),
	.c0_we        (ld_we),
	.c0_addr      (ld_addr),
	.c0_d         (ld_d),
	.c0_be        (ld_be),
	.c0_ack       (ld_ack),
	.c1_own       (sv_own),
	.c1_gnt       (sv_gnt),
	.c1_req       (sv_req),
	.c1_we        (sv_we),
	.c1_addr      (sv_addr),
	.c1_d         (sv_d),
	.c1_be        (sv_be),
	.c1_ack       (sv_ack),
	.c1_ssm_chip  (3'd3),
	.c1_ssm_sel   (sv_mem_sel),
	.c1_ssm_addr  ({2'd0, sv_mem_addr}),
	.c1_ssm_we    (sv_mem_we),
	.c1_ssm_wdata ({8'd0, sv_mem_wdata}),
	.c2_own       (st_own),
	.c2_gnt       (st_gnt),
	.c2_req       (st_req),
	.c2_we        (st_we),
	.c2_addr      (st_addr),
	.c2_d         (st_d),
	.c2_be        (st_be),
	.c2_ack       (st_ack),
	.c2_ssm_chip  (st_mem_chip),
	.c2_ssm_sel   (st_mem_sel),
	.c2_ssm_addr  (st_mem_addr),
	.c2_ssm_we    (st_mem_we),
	.c2_ssm_wdata (st_mem_wdata),
	.bk_req       (bk_req),
	.bk_we        (bk_we),
	.bk_addr      (bk_addr),
	.bk_d         (bk_d),
	.bk_be        (bk_be),
	.bk_ack       (bk_ack),
	.ss_mem_chip  (ss_mem_chip),
	.ss_mem_sel   (ss_mem_sel),
	.ss_mem_addr  (ss_mem_addr),
	.ss_mem_we    (ss_mem_we),
	.ss_mem_wdata (ss_mem_wdata),
	.pause_req    (pause_req)
);

///////////////////////   BOARD   ////////////////////////////////

wire        ce_sys, ce_master, ce_sck, phi1, ce_timer, ce_kport, ce_dot;
wire        mr_ras_n, mr_we_n;
wire  [3:0] mr_cas_n;
wire  [9:0] mr_ma;
wire [31:0] mr_d, mr_q;
wire        rom_ce_n, rom_oe_n;
wire [19:1] rom_a;
wire [15:0] rom_q;
wire        fx_rd_n;
wire [16:0] fx_a;
wire  [7:0] fx_d, fx_q;
wire  [7:0] y, u, v;
wire        tv_hs, tv_vs, tv_hblank, tv_vblank, tv_ce_pix, tv_field, tv_interlace;
wire        ka_req, ka_we, kb_req, kb_we;
wire  [8:0] ka_row, ka_col, kb_row, kb_col;
wire  [1:0] ka_be, kb_be;
wire [15:0] ka_d, ka_q, kb_d, kb_q;
wire [19:0] cd_blk_lba;
wire [15:0] cd_blk_base, cd_buf_addr;
wire  [2:0] cd_blk_cnt;
wire        cd_blk_rd, cd_blk_done, cd_blk_err, cd_blk_busy, cd_blk_fault;
wire        cd_disc, cd_toc_busy, cd_toc_new;
wire  [7:0] cd_buf_dout;
wire  [6:0] cd_toc_first, cd_toc_last;
wire [15:0] audio_l, audio_r;
wire        late;
wire        ss_shift, ss_out, clk_ss_so, kport_ss_so, board_ss_so;
wire        ss_window, ss_busy, cd_ss_busy;
wire [10:0] pg_q, pg_d;
wire        pg_we;
wire        cpu_ss_pause_req, cpu_ss_pause_ready, cpu_ss_restore, cpu_ss_state_wren;
wire  [3:0] cpu_ss_state_addr;
wire [63:0] cpu_ss_state_wdata, cpu_ss_state_rdata;
wire        cpu_ss_mem_active, cpu_ss_mem_rden, cpu_ss_mem_wren;
wire [10:0] cpu_ss_mem_addr;
wire  [7:0] cpu_ss_mem_wdata, cpu_ss_mem_rdata;
wire [31:0] toc_hash;

pcfx_clocks pcfx_clocks
(
	.clk       (clk_sys),
	.reset     (board_reset),
	.run       (run),
	.ce_sys    (ce_sys),
	.ce_master (ce_master),
	.ce_sck    (ce_sck),
	.phi1      (phi1),
	.ce_timer  (ce_timer),
	.ce_kport  (ce_kport),
	.ss_shift  (ss_shift),
	.ss_si     (ss_out),
	.ss_so     (clk_ss_so)
);

pcfx_memory pcfx_memory
(
	.clk_sys    (clk_sys),
	.clk_sdram  (clk_sdram),
	.clk_cap    (clk_cap),
	.reset      (~pll_locked),
	.ce_sck     (ce_sck),
	.ce_dot     (ce_dot),
	.mr_ras_n   (mr_ras_n),
	.mr_cas_n   (mr_cas_n),
	.mr_ma      (mr_ma),
	.mr_we_n    (mr_we_n),
	.mr_d       (mr_d),
	.mr_q       (mr_q),
	.rom_ce_n   (rom_ce_n),
	.rom_oe_n   (rom_oe_n),
	.rom_a      (rom_a),
	.rom_q      (rom_q),
	.fx_cs_n    (fx_cs_n),
	.fx_rd_n    (fx_rd_n),
	.fx_wr_n    (fx_wr_n),
	.fx_a       (fx_a),
	.fx_d       (fx_d),
	.fx_q       (fx_q),
	.ka_req     (ka_req),
	.ka_we      (ka_we),
	.ka_row     (ka_row),
	.ka_col     (ka_col),
	.ka_be      (ka_be),
	.ka_d       (ka_d),
	.ka_q       (ka_q),
	.kb_req     (kb_req),
	.kb_we      (kb_we),
	.kb_row     (kb_row),
	.kb_col     (kb_col),
	.kb_be      (kb_be),
	.kb_d       (kb_d),
	.kb_q       (kb_q),
	.freeze     (freeze),
	.frozen     (frozen),
	.bk_req     (bk_req),
	.bk_we      (bk_we),
	.bk_addr    (bk_addr),
	.bk_d       (bk_d),
	.bk_be      (bk_be),
	.bk_ack     (bk_ack),
	.bk_q       (bk_q),
	.bg_req     (bg_req),
	.bg_we      (bg_we),
	.bg_addr    (bg_addr),
	.bg_d       (bg_d),
	.bg_ack     (bg_ack),
	.bg_q       (bg_q),
	.pg_q       (pg_q),
	.pg_we      (pg_we),
	.pg_d       (pg_d),
	.late       (late),
	.dps_phase_en   (dps_phase_en),
	.dps_updn       (dps_updn),
	.dps_phase_done (dps_phase_done),
	.cal_width  (cal_width),
	.cal_phase  (cal_phase),
	.cal_rise   (cal_rise),
	.cal_ok     (cal_ok),
	.cal_done   (cal_done),
	.SDRAM_CLK  (SDRAM_CLK),
	.SDRAM_CKE  (SDRAM_CKE),
	.SDRAM_A    (SDRAM_A),
	.SDRAM_BA   (SDRAM_BA),
	.SDRAM_DQ   (SDRAM_DQ),
	.SDRAM_DQML (SDRAM_DQML),
	.SDRAM_DQMH (SDRAM_DQMH),
	.SDRAM_nCS  (SDRAM_nCS),
	.SDRAM_nCAS (SDRAM_nCAS),
	.SDRAM_nRAS (SDRAM_nRAS),
	.SDRAM_nWE  (SDRAM_nWE)
);

// Pads, mouse and multitap on the two K-ports.
wire k0_latch_n, k0_clk, k0_rw, k0_data_o, k0_data_oe, k0_data_i;
wire k1_latch_n, k1_clk, k1_rw, k1_data_o, k1_data_oe, k1_data_i;

kport_devices #(.SAVESTATE(1'b1)) kport_devices
(
	.clk        (clk_sys),
	.reset      (~pll_locked),
	.port0_dev  (status[4:3]),
	.port1_dev  (status[6:5]),
	.mode1      (status[7]),
	.mode2      (status[8]),
	.joy0       (joy0[11:0]),
	.joy1       (joy1[11:0]),
	.joy2       (joy2[11:0]),
	.joy3       (joy3[11:0]),
	.joy4       (joy4[11:0]),
	.ps2_mouse  (ps2_mouse),
	.k0_latch_n (k0_latch_n),
	.k0_clk     (k0_clk),
	.k0_rw      (k0_rw),
	.k0_data_o  (k0_data_o),
	.k0_data_oe (k0_data_oe),
	.k0_data_i  (k0_data_i),
	.k1_latch_n (k1_latch_n),
	.k1_clk     (k1_clk),
	.k1_rw      (k1_rw),
	.k1_data_o  (k1_data_o),
	.k1_data_oe (k1_data_oe),
	.k1_data_i  (k1_data_i),
	.ss_shift   (ss_shift),
	.ss_si      (clk_ss_so),
	.ss_so      (kport_ss_so)
);

pcfx_mainboard #(
	.SAVESTATE (1'b1),
	.DEBUG     (1'b0)
) pcfx_mainboard (
	.clk                (clk_sys),
	.reset              (board_reset),
	.ce_sys             (ce_sys),
	.ce_sck             (ce_sck),
	.phi1               (phi1),
	.ce_timer           (ce_timer),
	.ce_kport           (ce_kport),
	.mr_ras_n           (mr_ras_n),
	.mr_cas_n           (mr_cas_n),
	.mr_ma              (mr_ma),
	.mr_we_n            (mr_we_n),
	.mr_d               (mr_d),
	.mr_q               (mr_q),
	.rom_ce_n           (rom_ce_n),
	.rom_oe_n           (rom_oe_n),
	.rom_a              (rom_a),
	.rom_q              (rom_q),
	.fx_cs_n            (fx_cs_n),
	.fx_rd_n            (fx_rd_n),
	.fx_wr_n            (fx_wr_n),
	.fx_a               (fx_a),
	.fx_d               (fx_d),
	.fx_q               (fx_q),
	.fx_inserted        (card_in),
	.ka_req             (ka_req),
	.ka_we              (ka_we),
	.ka_row             (ka_row),
	.ka_col             (ka_col),
	.ka_be              (ka_be),
	.ka_d               (ka_d),
	.ka_q               (ka_q),
	.kb_req             (kb_req),
	.kb_we              (kb_we),
	.kb_row             (kb_row),
	.kb_col             (kb_col),
	.kb_be              (kb_be),
	.kb_d               (kb_d),
	.kb_q               (kb_q),
	.fx_battery_ok      (1'b1),
	.ce_dot             (ce_dot),
	.y                  (y),
	.u                  (u),
	.v                  (v),
	.tv_hs              (tv_hs),
	.tv_vs              (tv_vs),
	.tv_hblank          (tv_hblank),
	.tv_vblank          (tv_vblank),
	.tv_ce_pix          (tv_ce_pix),
	.tv_field           (tv_field),
	.tv_interlace       (tv_interlace),
	.audio_l            (audio_l),
	.audio_r            (audio_r),
	.adpcm_enc_compat   (!status[28]),
	.adpcm_no_click     (status[29]),
	.cd_speed           (status[32:31]),
	.k0_latch_n         (k0_latch_n),
	.k0_clk             (k0_clk),
	.k0_rw              (k0_rw),
	.k0_data_i          (k0_data_i),
	.k0_data_o          (k0_data_o),
	.k0_data_oe         (k0_data_oe),
	.k1_latch_n         (k1_latch_n),
	.k1_clk             (k1_clk),
	.k1_rw              (k1_rw),
	.k1_data_i          (k1_data_i),
	.k1_data_o          (k1_data_o),
	.k1_data_oe         (k1_data_oe),
	.cd_blk_lba         (cd_blk_lba),
	.cd_blk_base        (cd_blk_base),
	.cd_blk_cnt         (cd_blk_cnt),
	.cd_blk_rd          (cd_blk_rd),
	.cd_blk_done        (cd_blk_done),
	.cd_blk_err         (cd_blk_err),
	.cd_blk_busy        (cd_blk_busy),
	.cd_blk_fault       (cd_blk_fault),
	.cd_buf_addr        (cd_buf_addr),
	.cd_buf_dout        (cd_buf_dout),
	.cd_disc            (cd_disc),
	.cd_toc_busy        (cd_toc_busy),
	.cd_toc_new         (cd_toc_new),
	.cd_toc_first       (cd_toc_first),
	.cd_toc_last        (cd_toc_last),
	.cd_ss_busy         (cd_ss_busy),
	.bram_we            (bram_we),
	.ss_window          (ss_window),
	.ss_busy            (ss_busy),
	.ss_shift           (ss_shift),
	.ss_si              (kport_ss_so),
	.ss_so              (board_ss_so),
	.ss_mem_chip        (ss_mem_chip),
	.ss_mem_sel         (ss_mem_sel),
	.ss_mem_addr        (ss_mem_addr),
	.ss_mem_we          (ss_mem_we),
	.ss_mem_wdata       (ss_mem_wdata),
	.ss_mem_rdata       (ss_mem_rdata),
	.cpu_ss_pause_req   (cpu_ss_pause_req),
	.cpu_ss_pause_ready (cpu_ss_pause_ready),
	.cpu_ss_restore     (cpu_ss_restore),
	.cpu_ss_state_addr  (cpu_ss_state_addr),
	.cpu_ss_state_wdata (cpu_ss_state_wdata),
	.cpu_ss_state_wren  (cpu_ss_state_wren),
	.cpu_ss_state_rdata (cpu_ss_state_rdata),
	.cpu_ss_mem_active  (cpu_ss_mem_active),
	.cpu_ss_mem_addr    (cpu_ss_mem_addr),
	.cpu_ss_mem_rden    (cpu_ss_mem_rden),
	.cpu_ss_mem_wren    (cpu_ss_mem_wren),
	.cpu_ss_mem_wdata   (cpu_ss_mem_wdata),
	.cpu_ss_mem_rdata   (cpu_ss_mem_rdata),
	.dbg_retire         (),
	.dbg_retire_pc      (),
	.dbg_a              (),
	.dbg_d              (),
	.dbg_be_n           (),
	.dbg_mrq_n          (),
	.dbg_rw             (),
	.dbg_bcyst_n        (),
	.dbg_ready_n        ()
);

// CD sectors and TOC from the HPS, slot 0 and ioctl index 250.
pcfx_cd_host pcfx_cd_host
(
	.clk            (clk_sys),
	.reset          (~pll_locked),
	.sd_lba         (sd_lba[0]),
	.sd_blk_cnt     (cd_sd_blk_cnt),
	.sd_rd          (sd_rd[0]),
	.sd_wr          (sd_wr[0]),
	.sd_ack         (sd_ack[0]),
	.sd_buff_addr   (sd_buff_addr),
	.sd_buff_dout   (sd_buff_dout),
	.sd_buff_din    (sd_buff_din[0]),
	.sd_buff_wr     (sd_buff_wr),
	.img_mounted    (img_mounted[0]),
	.img_readonly   (img_readonly),
	.img_size       (img_size),
	.ioctl_download (ioctl_download),
	.ioctl_index    (ioctl_index),
	.ioctl_wr       (ioctl_wr),
	.ioctl_addr     (ioctl_addr),
	.ioctl_dout     (ioctl_dout),
	.blk_lba        (cd_blk_lba),
	.blk_base       (cd_blk_base),
	.blk_cnt        (cd_blk_cnt),
	.blk_rd         (cd_blk_rd),
	.blk_done       (cd_blk_done),
	.blk_err        (cd_blk_err),
	.blk_busy       (cd_blk_busy),
	.blk_fault      (cd_blk_fault),
	.buf_addr       (cd_buf_addr),
	.buf_dout       (cd_buf_dout),
	.toc_busy       (cd_toc_busy),
	.toc_new        (cd_toc_new),
	.disc           (cd_disc),
	.toc_first      (cd_toc_first),
	.toc_last       (cd_toc_last),
	.toc_hash       (toc_hash)
);

///////////////////////   SAVESTATES   ///////////////////////////

// Four 8 MB slots at DDR3 0x3800_0000 (CONF_STR SS); Alt+F1-F4 save,
// F1-F4 load, or hold Save State with Down / Up and Left / Right for the slot.
assign DDRAM_CLK = clk_sys;

pcfx_savestates pcfx_savestates
(
	.clk              (clk_sys),
	.reset            (~pll_locked),
	.ps2_key          (ps2_key),
	.allow_ss         (bios_loaded && !hold),
	.joy_ss           (1'b0),
	.joy_right        (joy0[0]),
	.joy_left         (joy0[1]),
	.joy_down         (joy0[2]),
	.joy_up           (joy0[3]),
	.joy_start        (joy0[11]),
	.joy_save         (joy0[12]),
	.status_slot      (status[10:9]),
	.osd_saveload     (status[12:11]),
	.count_save       (!status[13]),
	.info_req         (ss_info_req),
	.info             (ss_info),
	.status_update    (ss_status_set),
	.slot             (ss_slot),
	.busy             (),
	.load_bad         (),
	.ss_window        (ss_window),
	.ss_busy          (ss_busy),
	.cd_ss_busy       (cd_ss_busy),
	.cpu_pause_req    (cpu_ss_pause_req),
	.cpu_pause_ready  (cpu_ss_pause_ready),
	.cpu_restore      (cpu_ss_restore),
	.cpu_state_addr   (cpu_ss_state_addr),
	.cpu_state_wdata  (cpu_ss_state_wdata),
	.cpu_state_wren   (cpu_ss_state_wren),
	.cpu_state_rdata  (cpu_ss_state_rdata),
	.cpu_mem_active   (cpu_ss_mem_active),
	.cpu_mem_addr     (cpu_ss_mem_addr),
	.cpu_mem_rden     (cpu_ss_mem_rden),
	.cpu_mem_wren     (cpu_ss_mem_wren),
	.cpu_mem_wdata    (cpu_ss_mem_wdata),
	.cpu_mem_rdata    (cpu_ss_mem_rdata),
	.own              (st_own),
	.gnt              (st_gnt),
	.paused           (paused),
	.ss_shift         (ss_shift),
	.ss_out           (ss_out),
	.ss_in            (board_ss_so),
	.ssm_chip         (st_mem_chip),
	.ssm_sel          (st_mem_sel),
	.ssm_addr         (st_mem_addr),
	.ssm_we           (st_mem_we),
	.ssm_wdata        (st_mem_wdata),
	.ssm_rdata        (ss_mem_rdata),
	.bk_req           (st_req),
	.bk_we            (st_we),
	.bk_addr          (st_addr),
	.bk_d             (st_d),
	.bk_be            (st_be),
	.bk_ack           (st_ack),
	.bk_q             (bk_q),
	.pg_q             (pg_q),
	.pg_we            (pg_we),
	.pg_d             (pg_d),
	.bios_crc         (bios_crc),
	.toc_hash         (toc_hash),
	.DDRAM_BUSY       (DDRAM_BUSY),
	.DDRAM_BURSTCNT   (DDRAM_BURSTCNT),
	.DDRAM_ADDR       (DDRAM_ADDR),
	.DDRAM_DOUT       (DDRAM_DOUT),
	.DDRAM_DOUT_READY (DDRAM_DOUT_READY),
	.DDRAM_RD         (DDRAM_RD),
	.DDRAM_DIN        (DDRAM_DIN),
	.DDRAM_BE         (DDRAM_BE),
	.DDRAM_WE         (DDRAM_WE)
);

// Lit once the memory schedule has ever missed a deadline.
reg late_seen = 1'b0;
always @(posedge clk_sys) begin
	if (~pll_locked) late_seen <= 1'b0;
	else if (late)   late_seen <= 1'b1;
end

assign LED_USER = late_seen;

assign AUDIO_L = audio_l;
assign AUDIO_R = audio_r;

///////////////////////   VIDEO   ////////////////////////////////

assign CLK_VIDEO = clk_vid;

pcfx_video pcfx_video
(
	.clk_sys            (clk_sys),
	.CLK_VIDEO          (CLK_VIDEO),
	.pause              (!board_reset && !run),
	.y                  (y),
	.u                  (u),
	.v                  (v),
	.hs                 (tv_hs),
	.vs                 (tv_vs),
	.hblank             (tv_hblank),
	.vblank             (tv_vblank),
	.ce_pix             (tv_ce_pix),
	.field              (tv_field),
	.interlace          (tv_interlace),
	.jfif               (status[24]),
	.fx                 (status[16:14]),
	.forced_scandoubler (forced_scandoubler),
	.vcrop              (status[17]),
	.vcrop_off          (status[21:18]),
	.scale              (status[23:22]),
	.ar                 (status[122:121]),
	.gamma_bus          (gamma_bus),
	.HDMI_WIDTH         (HDMI_WIDTH),
	.HDMI_HEIGHT        (HDMI_HEIGHT),
	.en216p             (en216p),
	.CE_PIXEL           (CE_PIXEL),
	.VGA_R              (VGA_R),
	.VGA_G              (VGA_G),
	.VGA_B              (VGA_B),
	.VGA_HS             (VGA_HS),
	.VGA_VS             (VGA_VS),
	.VGA_DE             (VGA_DE),
	.VGA_F1             (VGA_F1),
	.VGA_SL             (VGA_SL),
	.VIDEO_ARX          (VIDEO_ARX),
	.VIDEO_ARY          (VIDEO_ARY),
	.HDMI_FREEZE        (HDMI_FREEZE)
);

endmodule
