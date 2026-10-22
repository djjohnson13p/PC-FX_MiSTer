# PC-FX development branch

This experimental branch starts from the newer Kitrinx implementation at
`b9480625951afd96988533d252847bdb74bc30bd` in
[MiSTer-devel/PC-FX_MiSTer](https://github.com/MiSTer-devel/PC-FX_MiSTer).
The older `PCFX_MiSTer` repository is a separate implementation. Its controller
and framework forks are not interchangeable with this core.

Changes on this branch were prepared with AI-assisted development. They have
not been submitted upstream and are not a claim of complete hardware accuracy.

## Changes under evaluation

- The framework selects a version-specific PLL include. The added Quartus19
  and Quartus25 includes reference the existing `pll_q17.qip` sources without
  regenerating their parameters or changing dynamic phase adjustment.
- A dual-SDRAM build explicitly parks the unused second module and releases its
  pins when `SDRAM2_EN` is low. Emulation still uses only the first module.
- The RAINBOW receive FIFO retains its pending RAM response while `ce` is low.
  Continuous-ce read latency is unchanged. Synthetic actual-RTL tests cover the
  reproduced failure and regressions; see [tests/rainbow](tests/rainbow/README.md).

The FIFO change is a module clock-enable robustness fix. Current normal runtime
keeps this clock enable high, and savestate pause waits for RAINBOW to become
idle. It has not been shown to fix the Queen of Queens corruption report.

Three additional [nonzero-coefficient controls](tests/rainbow/nonzero/README.md)
passed against both upstream and candidate FIFOs with identical timing and all
714 coefficient events matching their fixed expectations. They exercise signed
DC prediction, Y/UV AC runs and long-token alignment using software-reference
expectations. They do not establish chip accuracy; the portable runner's
packaging has only been syntax/dry-run checked.

## Build configuration

The default project uses the analog-I/O pin map. For a dual-SDRAM board, use a
separate build checkout and replace exactly the following line in `PCFX.qsf`:

```tcl
source sys/sys_analog.tcl
```

with:

```tcl
source sys/sys_dual_sdram.tcl
```

The dual pin Tcl also defines `MISTER_DUAL_SDRAM`. Do not combine that macro
with the analog pin map: the two maps assign the same physical connector pins
to different functions. Do not alter generated PLL parameters just to select
another compiler version.

Validation builds target Cyclone V `5CSEBA6U23I7`, use one compiler worker and
enable multicorner timing analysis. The relevant project assignments are:

```tcl
set_global_assignment -name NUM_PARALLEL_PROCESSORS 1
set_global_assignment -name TIMEQUEST_MULTICORNER_ANALYSIS ON
```

Quartus Lite25.1std Build1129 recognized the device and synthesized the initial
PLL-include baseline. The corrected dual-board/FIFO candidate hit an internal
optimizer assertion in `opt_op_add.cpp:2110` before fitting. It is not a completed
Quartus25 build. The same candidate completed synthesis, fitting, assembly and
standard timing analysis with Quartus Lite19.1 Build670 as a controlled fallback.
The resulting bitstream is held from hardware testing for the capture-window
check below; ordinary timing success is not sufficient for this controller.

### SDRAM capture-window acceptance

The inherited `rtl/mem/sdram.sdc` deliberately leaves pin-to-DDIO read capture
to runtime phase calibration. It constrains the following DDIO-to-register paths
with a 7.800 ns maximum and a relaxed -15.000 ns minimum, and additionally
requires every fast-corner setup slack to be at most 4.600 ns. This upper-slack
limit protects the available calibration window; a larger positive slack is
not automatically better here.

The first Quartus19 fit passes ordinary constrained timing in all eight models,
but fails this separate upper-slack gate on 249 of 256 fast-corner path/model
observations (maximum 5.937 ns). The review covers all 64 direct source/target
pairs and 32 calibrated destinations per corner. All 64 intended register and
mux LAB placements were honored. These results establish a source-contract
failure in this fit, not an observed malfunction on a running MiSTer.

A private routing experiment is being prepared. It must preserve the original
7.800/-15.000 constraints and 4.600 acceptance limit, constrain actual physical
nets, and pass a fresh full fit and all-corner review before device testing.
The source comment's nominal 3.200 ns hop is not a raw net-delay budget: the
reported slack also includes clock skew, uncertainty and cell delays. No guest
clock or memory-cycle slowdown is justified by this finding.

## Host software requirement

This implementation requires the PCFX support introduced in official
[Main_MiSTer commit88d2d107](https://github.com/MiSTer-devel/Main_MiSTer/commit/88d2d107630d05d52a2defb42fb7a449a8098827).
The official `MiSTer_20261010` binary predates that support. A BIOS screen alone
does not establish working CD loading. Do not use a custom Main intended for
the older no-hyphen PCFX core as a substitute.

The development host build is pinned to official Main source
`0c98d24e1b548b8de4cd875300810d8214540151`, with no C/C++ source edits. Preserve a
verified rollback binary before replacing Main. An updater may replace a local
development Main with the last published binary; check versions after updates.

## Acceptance still required

Before presenting any fix upstream, complete FPGA fit/timing review (including
the special capture-window gate and runtime calibration), isolated
on-device boot/CD/input checks, relevant game regression checks and repeated
save/restore tests. Characterize the reported Queen of Queens and3dtank issues
separately. Emulator agreement or a single benchmark score is not sufficient
evidence for changing poorly documented hardware timing.

The reported Queen of Queens effect may last only one or two frames. Existing
one-shot screenshots cannot establish that it is absent; that acceptance needs
reliable consecutive-frame capture or a live observation of the reproduction.

BIOS, commercial games, compiled tools, licenses and private device settings
must remain outside this public source repository.
