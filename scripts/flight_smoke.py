"""One-vehicle acceptance: heartbeat, normal arm, takeoff, land, disarm."""
import json
import time
from pathlib import Path
from pymavlink import mavutil
from common import ROOT
m=mavutil.mavlink_connection('udpin:127.0.0.1:14540',source_system=250)
start=time.monotonic(); lastbeat=0; data={}; events=[]
def pump(seconds=1):
    global lastbeat
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if time.monotonic()-lastbeat> .5:
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0);lastbeat=time.monotonic()
        msg=m.recv_match(blocking=True,timeout=.1)
        if msg:
            data[msg.get_type()]=msg
            if msg.get_type() in ['COMMAND_ACK','STATUSTEXT']: print(msg,flush=True)
def command(cmd,*args):
    m.mav.command_long_send(1,1,cmd,0,*(list(args)+[0]*(7-len(args))))
for _ in range(30):
    pump()
    if 'HEARTBEAT' in data and 'LOCAL_POSITION_NED' in data:break
else: raise SystemExit('No telemetry')
print('Connected',data['HEARTBEAT'],data['LOCAL_POSITION_NED'],flush=True)
pump(4)
command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
pump(3)
if not data['HEARTBEAT'].base_mode & 128: raise SystemExit('ARM FAILED')
command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,float('nan'),float('nan'),float('nan'),503)
maxalt=0
for _ in range(45):
    pump(.5); maxalt=max(maxalt,-data['LOCAL_POSITION_NED'].z)
    if maxalt>8: break
command(mavutil.mavlink.MAV_CMD_NAV_LAND,0,0,0,float('nan'),float('nan'),float('nan'),0)
landed=False
for _ in range(90):
    pump(.5)
    if not data['HEARTBEAT'].base_mode &128: landed=True;break
result={'system_id':data['HEARTBEAT'].get_srcSystem(),'max_altitude_local_m':maxalt,'disarmed_after_land':landed,'passed':maxalt>8 and landed}
(ROOT/'docs/phase2-result.json').write_text(json.dumps(result,indent=2));print(result)
if not result['passed']:raise SystemExit(1)
