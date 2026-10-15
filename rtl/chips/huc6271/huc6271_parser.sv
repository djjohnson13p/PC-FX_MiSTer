// HuC6271 stream parser, on bytes as they arrive from KING.
//
//   FF  kind  len_hi  len_lo  data ...      kind FF (Q tables + DCT), F8
//                                           (DCT), F0-F3 (run length)
//
// Anything else is skipped while hunting for FF. The length counts data
// and alignment padding after the header, stuffing included; when it runs
// out the request to KING is dropped until the judgment re-arms the parser.
// In the data a byte after FF is stuffing and is dropped.

module huc6271_parser #(
	parameter bit SAVESTATE = 1'b1
)
(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce,
	input  logic        init,       // judgment or reset: hunt again
	input  logic        run,

	input  logic        rx_v,
	input  logic  [7:0] rx_d,

	output logic        hdr,        // one clk: a strip starts
	output logic  [7:0] kind,
	output logic        push,
	output logic  [7:0] push_d,
	output huc6271_pkg::pstate_t ps,

	input  logic        ss_shift,
	input  logic        ss_si,
	output logic        ss_so
);

import huc6271_pkg::*;

logic [15:0] cnt;
logic  [7:0] len_hi;
logic        stuff;

wire valid_kind = rx_d == 8'hFF || rx_d == 8'hF8 || rx_d[7:2] == 6'b111100;
wire [15:0] len = {len_hi, rx_d};

always_ff @(posedge clk) begin
	if (reset) begin
		ps   <= P_HUNT;
		hdr  <= 1'b0;
		push <= 1'b0;
	end else if (SAVESTATE && ss_shift) begin
		ps <= pstate_t'({ps[1:0], ss_si});
	end else if (ce) begin
		hdr  <= 1'b0;
		push <= 1'b0;
		if (init || !run) begin
			ps <= P_HUNT;
		end else if (rx_v) begin
			case (ps)
				P_HUNT: if (rx_d == 8'hFF) ps <= P_TYPE;
				P_TYPE: begin
					kind <= rx_d;
					ps   <= valid_kind ? P_LENH : P_HUNT;
				end
				P_LENH: begin
					len_hi <= rx_d;
					ps     <= P_LENL;
				end
				P_LENL: begin
					cnt   <= len;
					stuff <= 1'b0;
					if (!len[15] && len > 16'd2) begin
						hdr <= 1'b1;
						ps  <= P_DATA;
					end else if (kind == 8'hFF) begin
						// Rejected table headers still carry 128 raw bytes.
						cnt <= 16'd128;
						ps  <= P_SKIP;
					end else begin
						ps  <= P_HUNT;
					end
				end
				P_SKIP: begin
					cnt <= cnt - 16'd1;
					if (cnt == 16'd1)
						ps <= P_HUNT;
				end
				P_DATA: begin
					cnt    <= cnt - 16'd1;
					stuff  <= !stuff && rx_d == 8'hFF;
					push   <= !stuff;
					push_d <= rx_d;
					if (cnt == 16'd1)
						ps <= P_DONE;
				end
				default: ;
			endcase
		end
	end
end

assign ss_so = SAVESTATE ? ps[2] : ss_si;

endmodule
