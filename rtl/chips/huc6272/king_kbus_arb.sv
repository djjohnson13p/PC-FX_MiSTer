// KING K-bus arbiter for one KRAM bank: one access per dot, decided in the
// DCK clock that starts the dot. Priorities (C6272_2 3.2.2):
//
//   display  BG > HuC6271 > SCSI > CPU
//   blanking refresh > ADPCM > HuC6271 > SCSI > CPU
//
// A request that loses is held by its owner and judged again next dot.
// Refresh takes the slot without a KRAM request (the memory system keeps
// its own rows alive).

module king_kbus_arb
	import huc6272_pkg::*;
(
	input  logic  display,
	input  logic  refresh,
	input  kreq_t bg,
	input  kreq_t adpcm,
	input  kreq_t rb,
	input  kreq_t scsi,
	input  kreq_t cpu,

	output kreq_t sel,
	output logic  gnt_bg,
	output logic  gnt_adpcm,
	output logic  gnt_rb,
	output logic  gnt_scsi,
	output logic  gnt_cpu
);

always_comb begin
	gnt_bg    = 1'b0;
	gnt_adpcm = 1'b0;
	gnt_rb    = 1'b0;
	gnt_scsi  = 1'b0;
	gnt_cpu   = 1'b0;
	if (refresh)
		;
	else if (display && bg.v)
		gnt_bg = 1'b1;
	else if (!display && adpcm.v)
		gnt_adpcm = 1'b1;
	else if (rb.v)
		gnt_rb = 1'b1;
	else if (scsi.v)
		gnt_scsi = 1'b1;
	else if (cpu.v)
		gnt_cpu = 1'b1;

	sel = '0;
	if (gnt_bg)    sel = bg;
	if (gnt_adpcm) sel = adpcm;
	if (gnt_rb)    sel = rb;
	if (gnt_scsi)  sel = scsi;
	if (gnt_cpu)   sel = cpu;
end

endmodule
