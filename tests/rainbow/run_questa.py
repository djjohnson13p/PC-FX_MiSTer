"""Source-only RAINBOW interface tests; Python 3.10+ and Questa required.

No core edits, downloads, network/device/UI access or license configuration.
All compiler/simulator steps run sequentially as one local job.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

HERE = Path(__file__).resolve().parent
INPUTS = ["rtl/chips/huc6271/huc6271_pkg.sv", "rtl/mem/cache_ram.v",
          "rtl/chips/huc6271/huc6271_kr_rx.sv", "rtl/chips/huc6271/huc6271_vlc.sv"]
BASELINE_FAILED_CASES = {
    "one_pause_after_fifth_pop", "deterministic_stalls", "pseudorandom_stalls",
    "isolated_ce_gaps", "alternating_ce",
}
KNOWN_VENDOR_INIT_ERROR = "ERROR: Invalid input file name  . Expecting file with .hex extension (with Intel-hex data format) or .mif extension (with Altera-mif data format)."

def sha(path: Path, normalize_newlines: bool = False) -> str:
    content = path.read_bytes()
    if normalize_newlines:
        content = content.replace(b"\r\n", b"\n")
    return hashlib.sha256(content).hexdigest()

def find_core() -> Path | None:
    for candidate in [Path.cwd(), HERE, *HERE.parents]:
        if (candidate / INPUTS[0]).is_file():
            return candidate
    return None

def find_sim_bin(argument: Path | None) -> Path | None:
    if argument is not None:
        return argument.resolve()
    configured = os.environ.get("QUESTA_BIN") or os.environ.get("MODELSIM_BIN")
    if configured:
        return Path(configured).resolve()
    found = shutil.which("vlog")
    return Path(found).resolve().parent if found else None

def evaluate_simulation(stdout: str, expected_baseline: bool,
                        allow_vendor_init_error: bool) -> dict:
    """Interpret explicit scoreboard results, retaining failure status honestly."""
    suite = re.search(r"(?:FIFO_)?SUITE_RESULT cases=(\d+) errors=(\d+)", stdout)
    case_matches = re.findall(r"CASE_RESULT name=(\S+).*? errors=(\d+)", stdout)
    failing = {name for name, errors in case_matches if int(errors)}
    bad_error_lines = []
    known_init_seen = False
    for line in stdout.splitlines():
        text = line.lstrip("# ").strip()
        if re.match(r"(?:\*\*\s*)?(?:ERROR|Error|Fatal):", text):
            if text == KNOWN_VENDOR_INIT_ERROR and allow_vendor_init_error:
                known_init_seen = True
            elif expected_baseline and text == "** Fatal: RAINBOW baseline failures recorded":
                pass  # accepted only with all pinned expected-case checks below
            else:
                bad_error_lines.append(text)
    result = {
        "suite_found": suite is not None,
        "suite_cases": int(suite.group(1)) if suite else None,
        "suite_errors": int(suite.group(2)) if suite else None,
        "failing_case_names": sorted(failing),
        "case_results": [line.lstrip("# ") for line in stdout.splitlines() if "CASE_RESULT" in line],
        "known_vendor_init_error_explicitly_allowed": known_init_seen,
        "unexpected_error_lines": bad_error_lines,
        "expected_baseline_failure_reproduced": False,
        "suite_passed": bool(suite and int(suite.group(2)) == 0 and not bad_error_lines),
    }
    if expected_baseline:
        result["expected_baseline_failure_reproduced"] = bool(
            suite and int(suite.group(1)) == 14 and int(suite.group(2)) == 415
            and len(case_matches) == 14 and failing == BASELINE_FAILED_CASES
            and not bad_error_lines)
    result["accepted"] = (result["expected_baseline_failure_reproduced"]
                          if expected_baseline else result["suite_passed"])
    return result

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, help="PC-FX source checkout; otherwise search cwd/test parents")
    parser.add_argument("--questa-bin", type=Path, help="folder containing vlib/vlog/vsim; alternatively QUESTA_BIN/MODELSIM_BIN/PATH")
    parser.add_argument("--output-dir", type=Path, default=HERE / "runs")
    parser.add_argument("--bench", choices=["vlc", "fifo"], default="vlc")
    parser.add_argument("--held-q-model", action="store_true", help="diagnostic alternate FIFO; NOT current core integration")
    parser.add_argument("--vendor-ram", action="store_true", help="ALTERA_RESERVED_QIS plus bundled precompiled vendor libraries")
    parser.add_argument("--disable-ram-init", action="store_true", help="explicit FIFO test-only parameter profile: MEM_INIT_FILE empty; core defaults unchanged")
    parser.add_argument("--allow-vendor-noinit-warning", action="store_true", help="explicitly allow known single-space init_file model error; initialization remains unqualified")
    parser.add_argument("--expect-baseline-failures", action="store_true", help="require pinned baseline and reproduce exactly its five known failing VLC cases; never silently treat them as passes")
    parser.add_argument("--seed", type=int, default=1654779917, help="VLC deterministic LFSR seed; FIFO bench has its separately recorded fixed seed")
    parser.add_argument("--tag", default="rainbow")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    core = args.source_root.resolve() if args.source_root else find_core()
    if core is None:
        parser.error("Cannot find core source. Supply --source-root.")
    sim = find_sim_bin(args.questa_bin)
    if sim is None:
        parser.error("Cannot find Questa. Supply --questa-bin or QUESTA_BIN/MODELSIM_BIN/PATH.")
    if args.held_q_model and args.bench != "vlc":
        parser.error("--held-q-model applies only to the VLC diagnostic bench")
    if args.held_q_model and args.vendor_ram:
        parser.error("held model bypasses the actual RAM; do not label it vendor qualification")
    if args.expect_baseline_failures and (args.bench != "vlc" or args.held_q_model or args.vendor_ram or args.seed != 1654779917):
        parser.error("expected baseline mode requires portable-RAM VLC, actual FIFO and recorded seed")
    if args.allow_vendor_noinit_warning and not args.vendor_ram:
        parser.error("--allow-vendor-noinit-warning requires --vendor-ram")
    if args.disable_ram_init and (not args.vendor_ram or args.bench != "fifo"):
        parser.error("--disable-ram-init is only the vendor FIFO test profile")
    top = "tb_rainbow_fifo" if args.bench == "fifo" else "tb_rainbow_vlc"
    parts = INPUTS[:3] if args.bench == "fifo" else INPUTS
    sources = [core / p for p in parts] + [HERE / (top + ".sv")]
    if not all(p.is_file() for p in sources):
        parser.error("One or more source inputs are missing")
    if args.expect_baseline_failures:
        pins = json.loads((HERE / "source-pins.json").read_text())["baseline_lf_sha256"]
        if any(sha(core / p, True) != pins[p] for p in INPUTS):
            parser.error("Expected-failure mode requires the exact pinned b948 baseline, allowing CRLF/LF only")
        if sha(HERE / "tb_rainbow_vlc.sv", True) != json.loads((HERE / "source-pins.json").read_text())["vlc_testbench_sha256"]:
            parser.error("Expected-failure mode requires the exact reviewed v3 testbench")
    suffix = ".exe" if os.name == "nt" else ""
    executable = lambda name: str(sim / (name + suffix))
    commands = [
        [executable("vlib"), "work"],
        [executable("vlog"), "-sv", "-work", "work",
         *(["+define+ALTERA_RESERVED_QIS"] if args.vendor_ram else []), *map(str, sources)],
        [executable("vsim"), "-c", "-l", "transcript.log",
         *([f"-gHELD_Q_MODEL={int(args.held_q_model)}"] if args.bench == "vlc" else []),
         *(["-gDISABLE_RAM_INIT=1"] if args.disable_ram_init else []),
         *(["-L", "altera_mf_ver", "-L", "altera_lnsim_ver", "-L", "cyclonev_ver"] if args.vendor_ram else []),
         f"work.{top}", f"+SEED={args.seed}", "-do",
         "onerror {quit -code 2}; onbreak {quit -code 1}; run -all; quit -code 0"],
    ]
    if args.dry_run:
        print(json.dumps({"commands": commands, "source_sha256": {p.name: sha(p) for p in sources}}, indent=2))
        return 0
    run_dir = args.output_dir.resolve() / f"{args.tag}-{args.bench}-{time.time_ns()}"
    run_dir.mkdir(parents=True, exist_ok=False)
    receipt = {"bench": args.bench, "held_q_model": args.held_q_model,
               "vendor_ram": args.vendor_ram, "seed": args.seed,
               "disable_ram_init_test_profile": args.disable_ram_init,
               "scope": "External source pinned by hashes; synthetic interface tests, not hardware/game accuracy proof",
               "source_sha256": {str(p): sha(p) for p in sources}, "steps": []}
    interpreted_exit = 0
    for index, command in enumerate(commands):
        began = time.monotonic()
        try:
            result = subprocess.run(command, cwd=run_dir, text=True, capture_output=True, timeout=180)
        except subprocess.TimeoutExpired as exc:
            out, err = exc.stdout or "", exc.stderr or ""
            if isinstance(out, bytes): out = out.decode(errors="replace")
            if isinstance(err, bytes): err = err.decode(errors="replace")
            result = subprocess.CompletedProcess(command, 124, out, err+"\nRunner timeout after 180 seconds.\n")
        (run_dir / f"step{index}.stdout.log").write_text(result.stdout, encoding="utf-8")
        (run_dir / f"step{index}.stderr.log").write_text(result.stderr, encoding="utf-8")
        receipt["steps"].append({"argv": command, "native_returncode": result.returncode,
                                 "elapsed_seconds": round(time.monotonic()-began, 3)})
        print(result.stdout)
        if result.stderr:
            print(result.stderr)
        if index == 2:
            evaluation = evaluate_simulation(result.stdout, args.expect_baseline_failures,
                                             args.allow_vendor_noinit_warning)
            receipt["evaluation"] = evaluation
            interpreted_exit = 0 if evaluation["accepted"] and result.returncode == 0 else 1
        else:
            interpreted_exit = result.returncode
        receipt["interpreted_returncode"] = interpreted_exit
        (run_dir / "receipt.json").write_text(json.dumps(receipt, indent=2)+"\n", encoding="utf-8")
        if interpreted_exit:
            break
    print(f"RECEIPT {run_dir / 'receipt.json'}")
    if args.expect_baseline_failures:
        print("MODE expected baseline failures; consult evaluation.suite_passed (remains false)")
    if args.held_q_model:
        print("MODE held-q diagnostic; not actual current-core receive FIFO qualification")
    return interpreted_exit

if __name__ == "__main__":
    raise SystemExit(main())
