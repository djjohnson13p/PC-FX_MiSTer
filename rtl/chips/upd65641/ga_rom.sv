// ROM control and the other slow memory-side devices on the local 16-bit
// bus: BIOS ROM, backup SRAM and FX-BMP (both on LD7:0), expansion bus.
// Selects and read strobe run from T2 to the end of the cycle; the write
// strobe rises one clock before the end. A ROM read selects already in T1,
// straight from the address, so the ROM socket sees it at the T1 CE edge.

module ga_rom
	import upd65641_pkg::*;
(
	input  logic       a_rom,
	input  logic       mrq_n,
	input  logic       bcyst_n,
	input  logic       cyc,
	input  kind_t      kind,
	input  logic       rw,
	input  logic       wlast,
	input  logic       hi,
	input  logic [1:0] be_even_n,

	output logic       rom_ce_n,
	output logic       bram_cs_n,
	output logic       cartsel_n,
	output logic       exb_cs_n,
	output logic       lrd_n,
	output logic       lwr_n
);

wire local_k = kind == K_ROM || kind == K_BRAM || kind == K_BMP || kind == K_EXP;

// 8-bit devices take only the even byte of the half being accessed.
wire narrow  = kind == K_BRAM || kind == K_BMP;
wire even_on = !(hi ? be_even_n[1] : be_even_n[0]);

wire rom_t1 = !bcyst_n && !mrq_n && a_rom && rw;

assign rom_ce_n  = !(cyc && kind == K_ROM || rom_t1);
assign bram_cs_n = !(cyc && kind == K_BRAM);
assign cartsel_n = !(cyc && kind == K_BMP);
assign exb_cs_n  = !(cyc && kind == K_EXP);
assign lrd_n     = !(cyc && local_k && rw || rom_t1);
assign lwr_n     = !(cyc && local_k && !rw && !wlast && (!narrow || even_on));

endmodule
