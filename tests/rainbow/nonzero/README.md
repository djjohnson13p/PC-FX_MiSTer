# Three nonzero RAINBOW FIFO/VLC controls

Original synthetic source-only tests, prepared with AI-assisted development.
Licensed GPL-2.0-only, matching the repository's GPL version 2 license; see
`LICENSE`. Review before integration. No BIOS, game data, reference emulator
source/tables, tools, compiler output, private configuration or logs are included.

Requires Python 3.10+ and an existing working Questa installation/license.
The runner inherits the environment without reading, recording or changing
license settings. No downloads or tool installation occur. The four core RTL
inputs are read externally, copied into a new results directory and hash-pinned;
the core checkout is never edited. One sequential worker runs vlib/vlog/vsim.

```text
python run_nonzero.py --source-root path/to/PC-FX_MiSTer --questa-bin path/to/questa/bin --output-dir path/to/new-results
python run_nonzero.py --source-root path/to/candidate --baseline-root path/to/upstream --questa-bin path/to/questa/bin --output-dir path/to/new-comparison
```

`--baseline-root` is an optional already-present second source tree; the runner
does not run Git or download a baseline. An existing output directory is refused.
Without `--questa-bin`, `vlib` is found on PATH. `--dry-run` validates fixture/source
inputs and prints commands without creating files or executing a tool.

Exactly three cases run per source tree. No old fourteen-case VLC or 530-case FIFO
suite is run, and no alternate held-response FIFO is offered. The actual
`huc6271_kr_rx`, `cache_ram_dp` portable RAM and `huc6271_vlc` modules run with CE
continuously high. The parser-push/FIFO/VLC boundary is tested; raw K-R strobes,
parser stuffing/length rules, dequantization, IDCT and final pixels are outside it.
`in_end` matches the top-level conjunction of producer completion, no current
push and an empty FIFO. Every payload is shorter than the 128-byte FIFO.

| Fixture | Payload | Expected coefficients | Purpose |
| --- | ---: | ---: | --- |
| `nz_dc_predictors` | 94 bytes | 96 | Signed differences, independent Y/U/V predictors, cross-macroblock accumulation. |
| `nz_ac_yuv_runs` | 97 bytes | 104 | Nonzero AC signs, different Y/UV tables, zero runs and zigzag addresses. |
| `nz_ac_overrun_alignment` | 117 bytes | 157 | Long overrun token consumes its magnitude without wrapping; following DC blocks remain aligned. |

Each requires 96 ordered block starts and ends, exact ordered coefficient
events, one completion transition, all bytes transferred, no FIFO overflow and
no unexpected QCx/table/null/palette output. Limits are 12,000 cycles per case,
one millisecond simulation watchdog and 180 wall-clock seconds per tool command.
Full stdout/stderr and input hashes remain in the chosen results directory.
Review generated paths before publishing results. Questa 2025.2 may print two
macro-command warnings for the `onerror`/`onbreak` batch commands; warnings are
retained in receipts. Errors, assertion failures and incomplete suites fail.

## Fixture derivation and limits

`vectors.json` is the readable authoritative fixture: literal MSB-first block
bitstrings, packed payload hex, original synthetic payload SHA-256, and every
expected `(macroblock, block, index, signed-value)` event. The checked-in `.hex`
files provide a second readable view; the runner regenerates simulator files
from the pinned JSON. No oracle uses DUT functions or internal state.

There is no located manufacturer-verified entropy-code specification for these
controls. Codewords were corroborated against separately pinned software table
data in [PaulDanielGaming/QueenofQueens commit
786fc9dcbaa6cdb8bfbe421f3f43c2f202c4bbac](https://github.com/PaulDanielGaming/QueenofQueens/tree/786fc9dcbaa6cdb8bfbe421f3f43c2f202c4bbac/mednafen/src/pcfx).
Its `rainbow.cpp` credits David Michel's MagicEngine/FX algorithm and tables.
That is reverse-engineering provenance, not a silicon oracle. No heuristic image
repair, downloaded table or reference decoder runs as part of this suite.

Hand derivation:

- DC Y category0=`000`, category1=`1110`, category2=`001`; UV category0=`00`,
  category2=`10`. Signed magnitude with leading zero is `raw + 1 - 2^n`.
  First macroblock differences are `+1,-1,+3,-3,+2,-2`; next U/V differences
  `-2,+2` return those predictors to zero. Only four DC outputs are nonzero.
- Y AC tokens are `00/1`, `01/01`, `11001/11`, `11010/0`: output indices
  `1,8,9,10`, values `+1,-2,+3,-1`. U tokens `100/1`, `1011/0`, `1010/101`
  yield indices `8,2,3`, values `+1,-1,+5`; V `01/00` yields index1/value-3.
  EOB=`11111`. Slashes separate code and magnitude, not stored bits.
- First Y block of the overrun fixture has DC0, 61 `00/1` AC+1 tokens,
  then `111101110111/1` (run15/size1), landing past coefficient63. The
  next Y DC differences are +1 and -1. This is a software-reference clipping
  control, not a hardware-proven rule for malformed or oversized runs.

`provenance.json` retains the independent source/table hashes and source links;
`known-source-pins.json` identifies the originally tested upstream/candidate
inputs. The runner records actual hashes and does not silently substitute known
files for the provided source tree. Future source revisions can be tested; a pass
still covers only these three small interfaces.

The initial bounded run passed 3/3 on each of pinned upstream and the CE-hold
candidate, with identical completion counts/timing. That statement is a historical
result, not a fresh run of this portable packaging. The portable runner has been
syntax/dry-run checked only; source integration/publication requires review.
No Queen of Queens fix, chip accuracy, FPGA timing or hardware acceptance is
claimed. The separate calibrated SDRAM timing hold remains unrelated.
