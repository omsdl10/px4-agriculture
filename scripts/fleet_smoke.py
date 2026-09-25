import os
os.environ["MAVLINK20"]="1"
import json
import time
from pymavlink import mavutil
from common import ROOT
links=[mavutil.mavlink_connection(f'udpin:127.0.0.1:{14540+i}',source_system=250) for i in range(4)]
data=[{} for _ in links]; lastbeat=0

def pump(seconds):
    global lastbeat
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        beat=time.monotonic()-lastbeat>.5
        for i,m in enumerate(links):
            if beat: m.mav.heartbeat_send(6,8,0,0,0)
            for _ in range(100):
                msg=m.recv_match()
                if not msg:break
                if msg.get_type().startswith(('UNKNOWN','BAD_DATA')) or msg.get_srcSystem()==250: continue
                if msg.get_srcSystem()!=i+1:raise RuntimeError(f'Unexpected system ID on port {14540+i}: {msg.get_srcSystem()} {msg}')
                data[i][msg.get_type()]=msg
                if msg.get_type() in ('COMMAND_ACK','STATUSTEXT'):print(i+1,msg,flush=True)
        if beat:lastbeat=time.monotonic()
        time.sleep(.01)
def command(i,cmd,*args):links[i].mav.command_long_send(i+1,1,cmd,0,*(list(args)+[0]*(7-len(args))))
for _ in range(90):
    pump(1)
    if all('LOCAL_POSITION_NED' in d and 'HEARTBEAT' in d for d in data):break
else:raise SystemExit('Missing telemetry')
pump(8)
for i in range(4): command(i,400,1)
pump(4)
if not all(d['HEARTBEAT'].base_mode &128 for d in data):raise SystemExit('Some vehicles failed to arm')
for i in range(4): command(i,22,0,0,0,float('nan'),float('nan'),float('nan'),498)
maxalt=[0]*4
for _ in range(80):
    pump(.5)
    for i,d in enumerate(data):maxalt[i]=max(maxalt[i],-d['LOCAL_POSITION_NED'].z)
    if min(maxalt)>5:break
for i in range(4):command(i,21,0,0,0,float('nan'),float('nan'),float('nan'),0)
for _ in range(120):
    pump(.5)
    if all(not d['HEARTBEAT'].base_mode &128 for d in data):break
result={'system_ids':[d['HEARTBEAT'].get_srcSystem() for d in data],'max_altitudes_m':maxalt,'all_landed_disarmed':all(not d['HEARTBEAT'].base_mode &128 for d in data)}
result['passed']=min(maxalt)>5 and result['all_landed_disarmed']
(ROOT/'docs/phase3-result.json').write_text(json.dumps(result,indent=2));print(result)
if not result['passed']:raise SystemExit(1)
