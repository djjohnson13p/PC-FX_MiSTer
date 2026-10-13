// BIOS loader: boot.rom (hps_io ioctl index 0, byte wide) into both BIOS
// copies in SDRAM through pcfx_memory's bulk port, which only works while the
// machine is frozen. Bytes pair up little-endian into ROM words; each word is
// written to bank 0 and then bank 1 (rows 512-1535) while ioctl_wait holds the
// next byte back. crc is the CRC-32 (IEEE, as zlib) of every byte of the
// image, for the savestate header; loaded rises when a download has ended
// and its last word is in place. reset is power-on only: the HPS sends the
// file while it holds the core in reset, and ioctl_wait would hold it there.

module bios_loader
(
	input  logic        clk,
	input  logic        reset,

	input  logic        ioctl_download,
	input  logic [15:0] ioctl_index,
	input  logic        ioctl_wr,
	input  logic [26:0] ioctl_addr,
	input  logic  [7:0] ioctl_dout,
	output logic        ioctl_wait,

	output logic        bk_req,
	output logic        bk_we,
	output logic [23:0] bk_addr,
	output logic [15:0] bk_d,
	output logic  [1:0] bk_be,
	input  logic        bk_ack,

	output wire         loaded,
	output logic [31:0] crc
);

	function automatic [31:0] crc32_byte;
		input [31:0] c;
		input  [7:0] d;
		reg   [31:0] x;
		begin
			x = c ^ {24'd0, d};
			for (int i = 0; i < 8; i++)
				x = x[0] ? (x >> 1) ^ 32'hEDB88320 : (x >> 1);
			crc32_byte = x;
		end
	endfunction

	wire mine = ioctl_download && (ioctl_index == 16'd0);

	logic        busy;          // a word is going out
	logic        bank;          // copy being written
	logic        acked;         // waiting for bk_ack to drop
	logic  [7:0] lo;
	logic [18:0] word;
	logic [31:0] c;
	logic        dl_q;
	logic        ended;

	assign ioctl_wait = busy;
	assign bk_we      = 1'b1;
	assign bk_be      = 2'b11;
	assign bk_addr    = {1'b0, bank, 13'd512 + {3'd0, word[18:9]}, word[8:0]};
	assign crc        = ~c;
	assign loaded     = ended && !busy;

	always_ff @(posedge clk) begin
		if (reset) begin
			busy   <= 1'b0;
			bank   <= 1'b0;
			acked  <= 1'b0;
			bk_req <= 1'b0;
			c      <= 32'hFFFFFFFF;
			ended  <= 1'b0;
			dl_q   <= 1'b0;
		end else begin
			dl_q <= mine;
			if (mine && !dl_q) begin
				c     <= 32'hFFFFFFFF;
				ended <= 1'b0;
			end
			if (!mine && dl_q) ended <= 1'b1;

			if (mine && ioctl_wr) begin
				c <= crc32_byte((mine && !dl_q) ? 32'hFFFFFFFF : c, ioctl_dout);
				if (!ioctl_addr[0]) begin
					lo <= ioctl_dout;
				end else if (ioctl_addr[26:20] == 7'd0) begin
					word   <= ioctl_addr[19:1];
					bk_d   <= {ioctl_dout, lo};
					bank   <= 1'b0;
					busy   <= 1'b1;
					bk_req <= 1'b1;
				end
			end

			// Four-phase handshake per copy: req until ack, then wait for
			// ack to drop before the next request.
			if (busy) begin
				if (bk_req && bk_ack) begin
					bk_req <= 1'b0;
					acked  <= 1'b1;
				end else if (acked && !bk_ack) begin
					acked <= 1'b0;
					if (!bank) begin
						bank   <= 1'b1;
						bk_req <= 1'b1;
					end else begin
						busy <= 1'b0;
					end
				end
			end
		end
	end

endmodule
