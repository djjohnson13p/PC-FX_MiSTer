// Copyright (c) 2026 Jamie Blanks
//
// Linux side of the CD drive: the disc on OSD slot 0 as 2448-byte blocks
// (2352 raw bytes, then 96 P-W subcode bytes), and the table of contents as
// a 1 KB record on download index 250 at every mount and eject.
//
//   buffer (64 KB)  0 .. 58751   a ring of 24 blocks, placed by the
//                                drive, six to a request
//                   58752 ..     track t at +4(t-1): F, S, M, control
//                                (t = 100 is the lead-out)
//                   59152 ..     track t pregap start at +4(t-1): F, S, M
//
// Record bytes 0-2 are first track, last track and disc present. While a
// record arrives the drive's buffer port is taken for the writes. The
// record and the slot are as agreed with the Main_MiSTer side.

module pcfx_cd_host #(parameter TIMEOUT = 429545454)
(
	input             clk,
	input             reset,

	// hps_io, slot 0
	output     [31:0] sd_lba,
	output      [5:0] sd_blk_cnt,
	output            sd_rd,
	output            sd_wr,
	input             sd_ack,
	input      [13:0] sd_buff_addr,
	input       [7:0] sd_buff_dout,
	output      [7:0] sd_buff_din,
	input             sd_buff_wr,
	input             img_mounted,
	input             img_readonly,
	input      [63:0] img_size,

	input             ioctl_download,
	input      [15:0] ioctl_index,
	input             ioctl_wr,
	input      [26:0] ioctl_addr,
	input       [7:0] ioctl_dout,

	// drive side
	input      [19:0] blk_lba,
	input      [15:0] blk_base,
	input       [2:0] blk_cnt,
	input             blk_rd,
	output            blk_done,
	output            blk_err,
	output            blk_busy,
	output            blk_fault,
	input      [15:0] buf_addr,
	output      [7:0] buf_dout,

	output            toc_busy,
	output reg        toc_new,
	output reg        disc,
	output reg  [6:0] toc_first,
	output reg  [6:0] toc_last,
	output reg [31:0] toc_hash          // record digest for the savestate header, 0 without a disc
);

wire toc_load = ioctl_download && ioctl_index == 16'd250;
wire [9:0] a = ioctl_addr[9:0];

// record byte to buffer address; pregap starts of tracks 1-99 only
wire        in_trk = a >= 10'd8   && a < 10'd408;
wire        in_idx = a >= 10'd516 && a < 10'd912;
wire [15:0] toc_waddr = in_trk ? 16'd58744 + {6'd0, a} : 16'd58636 + {6'd0, a};
wire        toc_we = toc_load && ioctl_wr && ioctl_addr[26:10] == 17'd0 && (in_trk || in_idx);

reg toc_load_d;
always @(posedge clk) begin
	toc_load_d <= toc_load;
	toc_new    <= toc_load_d && !toc_load;
	if (toc_load && !toc_load_d) toc_hash <= 32'd0;
	else if (toc_load && ioctl_wr) toc_hash <= {toc_hash[24:0], toc_hash[31:25]} + {24'd0, ioctl_dout};
	if (toc_load && ioctl_wr && ioctl_addr[26:10] == 17'd0) begin
		case (a)
		10'd0: toc_first <= ioctl_dout[6:0];
		10'd1: toc_last  <= ioctl_dout[6:0];
		10'd2: disc      <= ioctl_dout[0];
		default: ;
		endcase
	end
end

assign toc_busy = toc_load;

wire [31:0] lba32;
wire [19:0] blocks_unused;
wire        present_unused, wp_unused, mounted_unused;

hps_blk_slot #(.BUF_AW(16), .LBA_W(20), .TIMEOUT(TIMEOUT), .BASED(1), .RING(58752)) slot
(
	.clk(clk), .reset(reset),
	.sd_lba(lba32), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr),
	.img_mounted(img_mounted), .img_readonly(img_readonly), .img_size(img_size),
	.present(present_unused), .wp(wp_unused), .blocks(blocks_unused), .mounted(mounted_unused),
	.req_lba(blk_lba), .req_bank(1'b0), .req_base(blk_base), .req_rd(blk_rd), .req_wr(1'b0),
	.busy(blk_busy), .done(blk_done), .err(blk_err),
	.buf_addr(toc_we ? toc_waddr : buf_addr), .buf_we(toc_we), .buf_din(ioctl_dout), .buf_dout(buf_dout)
);

assign blk_fault = blk_busy && blk_err;
assign sd_lba = lba32;
assign sd_blk_cnt = {3'd0, blk_cnt};

endmodule
