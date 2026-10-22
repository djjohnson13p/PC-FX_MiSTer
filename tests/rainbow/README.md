# RAINBOW receive-FIFO clock-enable robustness tests

Original synthetic source-only tests for `MiSTer-devel/PC-FX_MiSTer`. Prepared
with AI-assisted development; review before integration. No BIOS, game data,
compiled libraries, tool binaries, licenses, private configuration or logs are
included. Core source is read externally and never modified by this runner.

## Run

Requires Python 3.10+ and an existing working Questa installation/license.
Execution is one sequential local job. No installation or license setup occurs.

```text
python run_questa.py --source-root path/to/PC-FX_MiSTer --questa-bin path/to/questa/bin --output-dir path/to/results
python run_questa.py --source-root path/to/PC-FX_MiSTer --questa-bin path/to/questa/bin --bench fifo --output-dir path/to/results
```

Without `--source-root`, the runner searches its parent directories and the
current directory for `rtl/chips/huc6271/huc6271_pkg.sv`. Without `--questa-bin`,
it uses `QUESTA_BIN`, `MODELSIM_BIN` or the directory of `vlog` on PATH. Existing
license environment is inherited but not read, recorded or changed. Logs and a
receipt with input SHA-256 and exact sequential argv are generated under
`--output-dir` (default `runs/` beside the runner). Review generated local paths
before publishing any logs. `--dry-run` prints input hashes and commands without
running a compiler or simulator.

Default mode uses the actual `huc6271_kr_rx` and `cache_ram_dp` portable RAM model,
plus actual `huc6271_vlc` for the VLC bench. It does not force or read internal
DUT state. `in_end` matches the real top level: producer finished and FIFO empty.
All pulse counts use enabled edges, as the downstream design does.

On the unchanged pinned baseline the default VLC suite **fails**. To reproduce
that expected result as an explicit experiment:

```text
python run_questa.py --source-root path/to/exact-baseline --questa-bin path/to/questa/bin --expect-baseline-failures
```

That mode checks canonical LF source hashes against commit
`b9480625951afd96988533d252847bdb74bc30bd`, the exact v3 bench, the recorded seed,
all fourteen cases and exactly the five known failing case names / 415 reported
assertion errors. It returns success only for reproducing that known failure;
the receipt still records `suite_passed=false` and
`expected_baseline_failure_reproduced=true`. It cannot accept arbitrary candidate
failures. Default mode never silently accepts the expected baseline failures.

`--held-q-model` is a separate diagnostic with a response register held between
enabled pops. It bypasses the real receive FIFO and must not be counted as
current-core qualification. The runner labels it explicitly.

## Cases and bounded results

The VLC bench independently encodes sixteen macroblocks, each four Y and two UV
DC-zero/EOB blocks (736 bits / 92 bytes), then optional QCx. Expected outputs are
96 ordered block starts, 96 block ends, 96 DC writes at index0/value0, no other
coefficients or unexpected null/palette/Q-table outputs, and the specified QCx
sequence. Cases cover zero/one/three trailing QCx, a final QCx byte delayed until
100 enabled cycles after the 96th block, incomplete tail code at true end,
truncated entropy, input/coefficient-buffer stalls, idle pause and five forms of
midstream ce gaps. One recorded deterministic LFSR seed is used, not a claim of
exhaustive entropy or schedule coverage.

The direct FIFO bench has **530 cases**, not 191,067 independent tests:

- One continuous-ce streaming/wrap case.
- 128 read addresses times four pause lengths (1, 2, 7, 33 clocks): 512 cases.
- Reset/flush during pending replies with ce high/low and 0–3 preceding gap
  clocks: sixteen cases.
- One 50,000-cycle seeded random traffic case (fixed LFSR seed `0x7a91c42d`).

Its independent queue scoreboard checks accepted byte order, no extra/lost
responses, next-clock response latency, response retention during ce0, empty
status, full-FIFO rejection, simultaneous push/pop and 128-address wrap.
Reset/flush deliberately discard pending replies, matching VLC init. The
existing full-FIFO policy rejects a push at count128 even if that edge pops.

| Result | Baseline | CE-hold candidate |
| --- | --- | --- |
| VLC actual FIFO, portable RAM | 9 pass / 5 artificial-gap failures | 14 pass / 0 failures |
| VLC held-response diagnostic | 14 pass; alternate model only | Not required |
| Direct actual FIFO, portable RAM | Not separately run | 530 pass / 0 errors |
| Direct actual FIFO, vendor model | Not separately run | 530 pass / 0 assertion errors, initialization/port warnings below |
| Vendor model, explicit init-disabled test profile | Not separately run | 530 pass / 0 errors; fifteen optional-port warnings |

Candidate direct-FIFO coverage: 191,067 steps; 95,540 accepted pushes; 95,090 pops
and immediate-response checks; 95,046 consumed responses; 11,871 pause checks;
729 read-pointer wraps; 22,426 simultaneous accepted push/pop; 512 full-FIFO push
rejections; nineteen resets and twenty-five flushes with a pending reply.
These are correlated operations within the 530 cases, not independent
qualifications. All candidate normal-ce completion cycle counts equal baseline.
Source pins are in `source-pins.json`; normalized LF hashes permit CRLF checkouts.

## Vendor-model option and its warning

When precompiled libraries are already mapped by the installed simulator:

```text
python run_questa.py --source-root path/to/core --questa-bin path/to/questa/bin --bench fifo --vendor-ram
```

This defines `ALTERA_RESERVED_QIS` and loads `altera_mf_ver`, `altera_lnsim_ver`
and `cyclonev_ver`, exercising the actual wrapper's `altsyncram` branch. It does
not download or compile vendor libraries. The tested Questa Altera Starter FPGA
Edition 2025.2 libraries were bundled with Quartus Lite 25.1.

The current wrapper supplies a single-space `MEM_INIT_FILE`. Its vendor model
reports an invalid initialization filename/readmem warning at time0, plus
fifteen omitted optional-port warnings. Every checked FIFO byte was explicitly
written by the test before read, and all 530 transport cases passed, but this is
**not a clean initialization-model result or power-up-memory proof**. Default
runner mode treats an `ERROR:` line as failure. To explicitly reproduce the
transport-only result while accepting precisely this known initialization error:

```text
python run_questa.py --source-root path/to/core --questa-bin path/to/questa/bin --bench fifo --vendor-ram --allow-vendor-noinit-warning
```

That allowance is explicit in the receipt and does not suppress other error
lines or any scoreboard failure. The original vendor-warning run did not
override a wrapper parameter or change source to conceal the warning.

A later, separately labelled test profile disables the nonexistent file load:

```text
python run_questa.py --source-root path/to/core --questa-bin path/to/questa/bin --bench fifo --vendor-ram --disable-ram-init
```

This uses the testbench parameter `DISABLE_RAM_INIT=1`, which sets only
`dut.u_fifo.MEM_INIT_FILE` to the empty string with a top-level `defparam`.
Default profile0 retains the original single-space value. The core file and its
default are unchanged. This profile passed the same 530 cases with identical
coverage, no initialization error and fifteen retained optional-port warnings.
Its bytes were initialized by test writes; it still makes no claim about
power-up memory contents. The original warning-bearing result is retained
separately. The FIFO source pin includes this optional profile; the pin file
also records the original default-only bench used in the first portable run.

## Finding and production scope

The receive FIFO pointer/count and VLC inflight flag advance under `ce`; the
RAM continues reading every clock. On baseline, a pause just after pop five
changes public `q` from `3e` to `7c` without another pop. The VLC then emits wrong
coefficients and does not complete. The candidate retains `3e` across the pause
with a local prior-ce/data-hold mux; continuous-ce read latency is unchanged.

This is **generic module clock-enable robustness**, not an established fix for
the Queen of Queens corruption report. Production RAINBOW ce is normally high
every oscillator clock. Reset/hold resets the board, and the current savestate
engine waits for idle `ss_busy` before requesting board pause; that signal
includes active VLC/nonempty FIFO. The vulnerable midstrip pause therefore
appears excluded by the current production path. Full-top-level reachability,
FPGA fit/timing and hardware/game acceptance remain separate requirements.

The tests do not prove chip accuracy, general nonzero entropy, IDCT/dequant
precision, palette/null modes, Q-table load, scale persistence, display-buffer
commit, all interleavings, synthesis-primitive equivalence or power-loss safety.
Vendor simulation adds evidence about configured RAM semantics; it is not a
substitute for actual synthesis and on-device testing. No gameplay improvement
is claimed from these bounded interface results alone.

## Additional nonzero controls

The separate [nonzero suite](nonzero/README.md) adds three literal controls for
signed DC prediction, Y/UV AC runs and long-token alignment. The original
private execution passed all three against both upstream and candidate FIFOs,
with 714 checked coefficient events and identical completion timing. Its
portable packaging has only been syntax/dry-run checked. Expected values come
from a separate software reference, with the limits documented in that suite;
this is not a hardware or Queen of Queens acceptance result.
