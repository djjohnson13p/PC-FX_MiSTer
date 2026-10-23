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
set_global_assignment -name TIMING_ANALYZER_MULTICORNER_ANALYSIS ON
```

Quartus Lite25.1std Build1129 originally hit an optimizer assertion in
`opt_op_add.cpp:2110`. A later controlled build completed synthesis, fitting,
assembly and standard timing analysis with these synthesis assignments:

```tcl
set_global_assignment -name SYNTH_TIMING_DRIVEN_SYNTHESIS OFF
set_global_assignment -name SYNTH_PROTECT_SDC_CONSTRAINT ON
```

This establishes a working compiler route, not a qualified core. The October22
experiment used the RTL at `d7a75abcbb73d18edb18be276e3593e7e7200305`, the dual-board
pin map, and additional local capture-routing constraints described below.
The full physical flow took39m49s. It fit the device at76% ALMs and87% RAM blocks,
but failed the final capture timing checks. No candidate from this work has been
loaded onto a MiSTer.

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

The later local experiment added64 reviewed per-pair minimum requirements and
26 Fitter-only maximum hints. It retained the original final-analysis maximum
and capture-window acceptance limit, clock parameters and RTL. Genuine mapped
checks verified that the implementation-only hints did not replace the final
timing requirements. The source comment's nominal3.200ns hop is not a raw net
delay budget: reported slack also includes clock skew, uncertainty and cells.

### October22 physical result and feasibility decision

The completed Quartus25 experiment was checked across all eight available
operating models. The native query had zero structural coverage failures and
covered512 capture source/target/model observations. Independent comparisons
confirmed all512 capture-hold full reports and272 ordinary-path full reports
against their exported results.

The numerical result failed:

- Capture setup failed149 observations across52 distinct source/target pairs.
  The Slow100C/-40C/85C/0C models had52/24/49/24 failing pairs respectively.
- Worst capture setup slack was-0.708ns at Slow85C, from DDIO high bit11 to
  `mr_q[27]`, with6.680ns data delay under the original7.800ns relationship.
- All capture hold observations remained positive; minimum margin was0.084ns.
- Maximum Fast capture setup slack was4.070ns, leaving0.530ns below the4.600ns
  upper limit. These margins occur on different paths and are not a proof that
  any specific lane is impossible to fix.
- All eleven reviewed ordinary CPU/memory families passed setup and hold.
  Minimum reviewed HDMI same-clock setup margin was0.032ns.
- Fifty-six additional manual-review entries for low-half result-register copies
  remain recorded. This failed candidate was not given complete timing acceptance.

The inspected worst capture path already uses an adjacent capture register and
a separate preserved half-select LUT; much of its delay is routing. No specific
RTL defect or small correction with demonstrated margin was identified. A second
physical build was therefore not justified under the bounded plan. This timing
tuning cycle has ended; additional work needs a deeper capture-interface or
placement investigation. This does not establish that timing closure is
impossible on CycloneV.

The experimental RBF is retained locally, unqualified for hardware use or release.
Its SHA-256 is
`feea0e81962b89241b451754dfb4d5a7241b3c01590d5bc6b8794cd780ba8883`.
The original build reports, mapped/fitted databases, local constraint package and
failed/successful receipts are preserved. A compiler-generated pin-model dump
caused an output-classification guard failure after native compilation succeeded;
the exact file was separately traced to the vendor Assembler and adjudicated,
without rewriting the original failed receipt or rerunning compilation.

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
