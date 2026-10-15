derive_pll_clocks
derive_clock_uncertainty

# core specific constraints

# The column bit is sampled again before the column command.
set bulk_col_src [get_registers {emu:emu|pcfx_savestates:pcfx_savestates|pcfx_ss_engine:engine|bk_addr[0]}]
set bulk_col_dst [get_registers {emu:emu|pcfx_memory:pcfx_memory|bkq_addr[0]}]
if {[get_collection_size $bulk_col_src] != 1 || [get_collection_size $bulk_col_dst] != 1} {
	error "Bulk column hold endpoints must each match one register"
}
set_false_path -hold -from $bulk_col_src -to $bulk_col_dst
