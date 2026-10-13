// One 64-bit DDR3 port for the savestate engine, a word at a time, in the
// HPS-shared window at 0x3000_0000 (addr is the 64-bit word offset in it).
// req is held until the ack pulse and dropped before the next request.

module pcfx_ss_ddram
(
	input  logic        clk,
	input  logic        reset,

	input  logic        req,
	input  logic        we,
	input  logic [24:0] addr,
	input  logic [63:0] wdata,
	input  logic  [7:0] be,
	output logic        ack,
	output logic [63:0] rdata,

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

typedef enum logic [1:0] {IDLE, CMD, READ, DONE} state_t;
state_t st;

assign DDRAM_BURSTCNT = 8'd1;

always_ff @(posedge clk) begin
	ack <= 1'b0;
	if (reset) begin
		st       <= IDLE;
		DDRAM_WE <= 1'b0;
		DDRAM_RD <= 1'b0;
	end else case (st)
		IDLE: if (req) begin
			DDRAM_ADDR <= {4'b0011, addr};
			DDRAM_DIN  <= wdata;
			DDRAM_BE   <= we ? be : 8'hFF;
			DDRAM_WE   <= we;
			DDRAM_RD   <= !we;
			st         <= CMD;
		end

		// The command is taken on a clock with BUSY low.
		CMD: if (!DDRAM_BUSY) begin
			DDRAM_WE <= 1'b0;
			DDRAM_RD <= 1'b0;
			if (DDRAM_WE) begin
				ack <= 1'b1;
				st  <= DONE;
			end else begin
				st <= READ;
			end
		end

		READ: if (DDRAM_DOUT_READY) begin
			rdata <= DDRAM_DOUT;
			ack   <= 1'b1;
			st    <= DONE;
		end

		default: if (!req) st <= IDLE;
	endcase
end

endmodule
