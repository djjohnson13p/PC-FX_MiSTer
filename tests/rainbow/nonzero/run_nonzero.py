# SPDX-License-Identifier: GPL-2.0-only
"""Three synthetic nonzero FIFO/VLC controls; Python 3.10+ and Questa required.

No downloads, core edits, Quartus, device access, or license configuration.
All vlib/vlog/vsim steps execute sequentially. See README.md for oracle limits.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
INPUTS=['rtl/chips/huc6271/huc6271_pkg.sv','rtl/mem/cache_ram.v',
        'rtl/chips/huc6271/huc6271_kr_rx.sv','rtl/chips/huc6271/huc6271_vlc.sv']
NAMES=['nz_dc_predictors','nz_ac_yuv_runs','nz_ac_overrun_alignment']
sha=lambda data:hashlib.sha256(data).hexdigest()

def write_json(path: Path, value: dict) -> None:
    with path.open('x',encoding='utf-8',newline='\n') as f:
        json.dump(value,f,indent=2);f.write('\n')

def evaluate(out: str, err: str, code: int) -> dict:
    cases=re.findall(r'CASE_RESULT name=(\S+).*? errors=(\d+)',out)
    suite=re.search(r'NONZERO_SUITE_RESULT cases=(\d+) errors=(\d+) continuous_ce=1 actual_fifo=1',out)
    problems=[line for line in (out+'\n'+err).splitlines()
              if re.match(r'^(?:#\s*)?(?:\*\*\s*)?(?:Error|ERROR|Fatal):',line.strip())]
    warnings=[line for line in (out+'\n'+err).splitlines()
              if re.match(r'^(?:#\s*)?(?:\*\*\s*)?Warning:',line.strip())]
    return dict(passed=bool(code==0 and suite and tuple(map(int,suite.groups()))==(3,0)
                           and cases==[(n,'0') for n in NAMES] and not problems),
                cases=[line.lstrip('# ') for line in out.splitlines() if 'CASE_RESULT' in line],
                errors=problems,warnings=warnings)

def main() -> int:
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-root',type=Path,required=True,help='Core checkout under test (read only).')
    p.add_argument('--baseline-root',type=Path,help='Optional already-present comparison checkout; no git/network operations.')
    p.add_argument('--questa-bin',type=Path,help='Directory of vlib/vlog/vsim; otherwise discover vlib on PATH.')
    p.add_argument('--output-dir',type=Path,required=True,help='New output directory; existing directories are refused.')
    p.add_argument('--dry-run',action='store_true',help='Validate inputs and print planned commands without writes or tool execution.')
    a=p.parse_args()
    sim=a.questa_bin
    if sim is None:
        found=shutil.which('vlib')
        if not found:p.error('Cannot find vlib; supply --questa-bin.')
        sim=Path(found).parent
    def executable(name: str) -> str:
        path=sim/name
        if not path.is_file():path=sim/(name+'.exe')
        if not path.is_file():p.error('Cannot find '+name+' under --questa-bin.')
        return str(path.resolve())
    tools={name:executable(name) for name in ['vlib','vlog','vsim']}
    pins=json.loads((HERE/'fixture-pins.json').read_text())
    for relative,expected in pins['files'].items():
        if sha((HERE/relative).read_bytes())!=expected:p.error('Handoff input hash differs: '+relative)
    vectors=json.loads((HERE/'vectors.json').read_text())['vectors']
    if [v['name'] for v in vectors]!=NAMES:p.error('Unexpected fixture names/order.')
    for v in vectors:
        raw=bytes.fromhex(v['payload_hex'])
        if len(raw)!=v['bytes'] or sha(raw)!=v['sha256'] or len(raw)>=128:p.error('Fixture payload identity/bound differs.')
        if len(v['expected_events_mb_blk_index_signed12'])!=v['expected_coefficients']:p.error('Expected-event count differs.')
    sources=[('source',a.source_root.resolve())]
    if a.baseline_root is not None:sources.append(('baseline',a.baseline_root.resolve()))
    frozen={}
    for name,root in sources:
        frozen[name]={relative:(root/relative).read_bytes() for relative in INPUTS}
    output=a.output_dir.resolve()
    if output.exists():p.error('Output directory already exists; no overwrite or cleanup.')
    plans=[]
    for name,_ in sources:
        folder=output/name
        commands=[
            [tools['vlib'],'work'],
            [tools['vlog'],'-sv','-work','work',*[str(folder/'source'/n) for n in INPUTS],str(folder/'tb_rainbow_nonzero.sv')],
            [tools['vsim'],'-c','-l','transcript.log','work.tb_rainbow_nonzero',
             '+VECTOR_DIR='+str(output/'vectors').replace('\\','/'),'-do',
             'onerror {quit -code 2}; onbreak {quit -code 1}; run -all; quit -code 0'],
        ]
        plans.append((name,folder,commands))
    if a.dry_run:
        print(json.dumps(dict(dry_run=True,source_hashes={name:{n:sha(b) for n,b in files.items()} for name,files in frozen.items()},
                              commands={name:commands for name,_,commands in plans}),indent=2))
        return 0
    output.mkdir(parents=True,exist_ok=False)
    data_dir=output/'vectors';data_dir.mkdir()
    for v in vectors:
        (data_dir/(v['name']+'.payload.hex')).write_text(''.join(f'{b:02x}\n' for b in bytes.fromhex(v['payload_hex'])),encoding='ascii')
        words=[(mb<<21)|(blk<<18)|(index<<12)|(value&0xfff) for mb,blk,index,value in v['expected_events_mb_blk_index_signed12']]
        (data_dir/(v['name']+'.expected.hex')).write_text(''.join(f'{w:08x}\n' for w in words),encoding='ascii')
    receipt=dict(scope='Three software-reference sparse coefficient-interface controls; not silicon/game accuracy proof',
                 fixture_pins=pins,runner_sha256=sha(Path(__file__).read_bytes()),
                 sequential_worker=True,continuous_ce=True,actual_fifo=True,vendor_ram=False,
                 max_cycles_per_case=12000,max_seconds_per_command=180,profiles=[])
    passed=True
    for name,folder,commands in plans:
        folder.mkdir()
        for relative,raw in frozen[name].items():
            target=folder/'source'/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(raw)
        (folder/'tb_rainbow_nonzero.sv').write_bytes((HERE/'tb_rainbow_nonzero.sv').read_bytes())
        record=dict(profile=name,source_hashes={n:sha(b) for n,b in frozen[name].items()},steps=[],passed=False)
        for index,command in enumerate(commands):
            began=time.monotonic()
            try:
                r=subprocess.run(command,cwd=folder,text=True,capture_output=True,timeout=180)
                code,out,err=r.returncode,r.stdout,r.stderr
            except subprocess.TimeoutExpired as e:
                code=124;out=e.stdout or '';err=e.stderr or ''
                if isinstance(out,bytes):out=out.decode(errors='replace')
                if isinstance(err,bytes):err=err.decode(errors='replace')
                err+='\nFixed subprocess budget exhausted.\n'
            (folder/f'step{index}.stdout.log').write_text(out,encoding='utf-8')
            (folder/f'step{index}.stderr.log').write_text(err,encoding='utf-8')
            record['steps'].append(dict(argv=command,returncode=code,seconds=round(time.monotonic()-began,3),
                                       stdout_sha256=sha(out.encode()),stderr_sha256=sha(err.encode())))
            if index==2:record.update(evaluate(out,err,code))
            if code!=0:break
        receipt['profiles'].append(record);passed=passed and record['passed']
        write_json(folder/'receipt.json',record)
        print(json.dumps(dict(profile=name,passed=record['passed'],cases=record.get('cases',[]))),flush=True)
    receipt['source_inputs_unchanged']=all((root/relative).read_bytes()==frozen[name][relative] for name,root in sources for relative in INPUTS)
    receipt['all_profiles_passed']=passed and receipt['source_inputs_unchanged']
    write_json(output/'receipt.json',receipt)
    print(json.dumps(dict(receipt=str(output/'receipt.json'),all_profiles_passed=receipt['all_profiles_passed'])),flush=True)
    return 0 if receipt['all_profiles_passed'] else 1

if __name__=='__main__':sys.exit(main())
