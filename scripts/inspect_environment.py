#!/usr/bin/env python3
"""Read-only version manifest; no dependency installation or environment changes."""
import json
import platform
import shutil
import subprocess
from pathlib import Path
from common import ROOT,config

def run(cmd):
    try:
        p=subprocess.run(cmd,capture_output=True,text=True,timeout=15)
        return {'command':cmd,'returncode':p.returncode,'stdout':p.stdout.strip(),'stderr':p.stderr.strip()}
    except (OSError,subprocess.TimeoutExpired) as e:return {'command':cmd,'error':str(e)}
px4=Path(config('simulation')['px4_root']).expanduser()
report={'os':platform.platform(),'architecture':platform.machine(),'python':platform.python_version(),'px4':run(['git','-C',str(px4),'describe','--tags','--always','--dirty']),'px4_submodule':run(['git','-C',str(px4/'Tools/simulation/gz'),'rev-parse','HEAD'])}
for tool,args in [('gz',['sim','--versions']),('cmake',['--version']),('ninja',['--version']),('ros2',['--help'])]:
    report[tool]=run([tool]+args) if shutil.which(tool) else {'available':False}
(ROOT/'docs/environment.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
