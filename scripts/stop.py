import fcntl
import json
import os
import signal
import time
from common import ROOT
state=ROOT/'.runtime/state.json'
if not state.exists():
    print('No active agriculture simulation.'); raise SystemExit(0)
with (ROOT/'.runtime/lock').open('a') as lock:
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        data=json.loads(state.read_text()); os.kill(data['pid'],signal.SIGTERM)
        for _ in range(900):
            if not state.exists(): print('Simulation stopped; logs finalized.');break
            time.sleep(.2)
        else: raise SystemExit('Shutdown is still in progress; inspect the supervisor.')
    else:
        raise SystemExit('Stale runtime state; no active supervisor. No process was signaled.')
