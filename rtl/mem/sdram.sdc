# SDRAM controller pin timing. Paired with rtl/mem/sdram.sv.
#
# These constraints describe the physical pin relationship the controller owns:
# the inverted DDIO-forwarded SDRAM_CLK and the command/address/write-data
# launch. The read capture is calibrated in hardware (see below). That
# relationship is part of the
# controller's contract, so it is constrained here beside the controller
# rather than in the core SDC.
#
# Integration requirements:
# - Read this file AFTER the core SDC that runs `derive_pll_clocks`, so the
#   controller clock already exists when the generated pin clock is derived.
#   `files.qip` lists it after the core SDC for that reason.
# - Set `sdram_ctrl_clk_pin` below to the PLL output pin driving the controller.
#   That single line is the only core-specific coupling in this file.
# - Crossings between the controller wrapper and the core's slower clock stay in
#   the core SDC. Those are integration timing, not pin timing.

set sdram_ctrl_clk_pin {emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[1].output_counter|divclk}

# A miss here stops the Fitter about a minute in with a generic "Can't fit
# design in device" (Error 11802); check this name first when that appears.
set sdram_ctrl_clk [get_clocks $sdram_ctrl_clk_pin]
if {[get_collection_size $sdram_ctrl_clk] == 0} {
	post_message -type error "sdram.sdc: no controller clock matched the configured pin"
}

# SDRAM_CLK is an inverted copy of the controller clock forwarded through the
# output DDIO cell (altddio_out, datain_h=0 / datain_l=1). Modeling it as a
# generated clock ON the pin accounts for the forward path's insertion delay.
# Without this the SDRAM output paths are analyzed against the raw PLL
# reference, which lacks the ~10 ns clock-network insertion that the data path
# actually sees, and every output pin reports a large fictitious clock skew
# (measured ~6 ns at 120 MHz).
create_generated_clock -name sdram_clk_pin -invert \
	-source [get_pins $sdram_ctrl_clk_pin] \
	[get_ports {SDRAM_CLK}]

if {[get_collection_size [get_clocks sdram_clk_pin]] == 0} {
	post_message -type error "sdram.sdc: no SDRAM_CLK generated pin clock matched"
}

set sdram_out_ports [get_ports {SDRAM_CKE SDRAM_A[*] SDRAM_BA[*] SDRAM_DQML SDRAM_DQMH SDRAM_nCS SDRAM_nCAS SDRAM_nRAS SDRAM_nWE SDRAM_DQ[*]}]

# Outputs launch on the controller clock rising edge and are sampled by the
# SDRAM on the SDRAM_CLK edge a half period later. The inversion lives in the
# generated clock, so no -clock_fall qualifier is used here.
set_output_delay -clock sdram_clk_pin -max 1.500 $sdram_out_ports
set_output_delay -clock sdram_clk_pin -min -0.800 $sdram_out_ports

# DQ reads: the input cell runs on clk_cap, whose phase pcfx_memory sets at
# run time by sweeping it over a test pattern, so no static check of the pin
# to the cell holds; the calibration window it reports is the real margin.
set_false_path -from [get_ports {SDRAM_DQ[*]}] -to [get_registers {*|altddio_in:u_sdram_dq_capture|*}]

# The cell feeds two clk_sdram registers per bit, dq_cap_q and the high
# half of pcfx_memory's mr_q, and calibration checks both, so a hop that is
# too slow or too fast for a phase just fails there. The hops only decide
# where the window can sit: the longest sets its late end at the slow
# corner, the shortest its early end at the fast corner. Hence an upper
# bound and no hold check (which would only pad them). Keeping the window
# wide enough also needs every hop's fast-corner setup slack under this
# bound to be at most 4.6 ns (a hop of at least 3.2 ns); TimeQuest cannot
# enforce that, so it is read from the report.
set sdram_cap_cells [get_registers -nowarn {*|altddio_in:u_sdram_dq_capture|*}]
set sdram_cap_regs  [get_registers -nowarn {*|sdram:u_sdram|dq_cap_q[*]}]
for {set i 16} {$i < 32} {incr i} {
	set sdram_cap_regs [add_to_collection $sdram_cap_regs [get_registers -nowarn "*|pcfx_memory:*|mr_q\[$i\]"]]
}
# More than 32 means a capture register was copied, and the copy's route
# is one calibration does not check.
set sdram_cap_n [get_collection_size $sdram_cap_regs]
if {$sdram_cap_n > 32} {
	post_message -type critical_warning "sdram.sdc: $sdram_cap_n capture registers, expected 32"
}
if {[get_collection_size $sdram_cap_cells] == 0 || $sdram_cap_n < 32} {
	post_message -type error "sdram.sdc: input cell or its 32 capture registers not found"
} else {
	set_max_delay -from $sdram_cap_cells -to $sdram_cap_regs 7.800
	set_min_delay -from $sdram_cap_cells -to $sdram_cap_regs -15.000
}

# pcfx_memory's main-RAM result register is clocked by the controller clock
# but loads only on edges that coincide with a clk_sys edge (k+3 of a CPU
# cycle) and holds for at least two clk_sys periods, so its clk_sys readers
# get three controller periods. Its controller-clock readers (calibration)
# keep the usual one.
set board_clks [remove_from_collection [all_clocks] $sdram_ctrl_clk]
set mr_q_regs [get_registers -nowarn {*|pcfx_memory:*|mr_q[*]}]
if {[get_collection_size $mr_q_regs] > 0} {
	set_multicycle_path -setup -start 3 -from $mr_q_regs -to $board_clks
	set_multicycle_path -hold -start 2 -from $mr_q_regs -to $board_clks
}

# Write data goes from the board straight into wp_d (main RAM, loads only
# on p5, the second controller edge after the CPU's phi1 edge) and cp_d
# (FX-BMP, loads only on p2, the second edge after the strobe's CE edge).
set wp_d_regs [get_registers -nowarn {*|pcfx_memory:*|wp_d[*] *|pcfx_memory:*|cp_d[*]}]
if {[get_collection_size $wp_d_regs] > 0} {
	set_multicycle_path -setup -end 2 -from $board_clks -to $wp_d_regs
	set_multicycle_path -hold -end 1 -from $board_clks -to $wp_d_regs
}

# ROM, FX-BMP and KRAM answers load into clk_sys registers from a capture
# register that took the beat at least one controller clock earlier and
# does not change again until well after the load, so a value launched on
# the load edge itself never reaches them.
set ans_src [get_registers -nowarn {*|pcfx_memory:*|cp_hold[*] *|pcfx_memory:*|k_hold[*]}]
set ans_dst [get_registers -nowarn {*|pcfx_memory:*|rom_q[*] *|pcfx_memory:*|fx_q[*] *|pcfx_memory:*|ka_q[*] *|pcfx_memory:*|kb_q[*]}]
if {[get_collection_size $ans_src] > 0 && [get_collection_size $ans_dst] > 0} {
	set_multicycle_path -hold -start 1 -from $ans_src -to $ans_dst
}
