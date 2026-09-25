#!/usr/bin/env python3
"""PX4 autonomous missions over MAVLink 2; Gazebo is the only simulator."""
import argparse
import csv
import json
import math
import os
import signal
import time
import itertools
from pathlib import Path
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
from common import ROOT, config
from planner import mission
M=mavutil.mavlink

def global_xy(x,y):
    return 47.397742+math.degrees(y/6378137),8.545594+math.degrees(x/(6378137*math.cos(math.radians(47.397742))))

def items(drone,compact):
    cfg=config('simulation'); sx,sy,_=drone['spawn']; alt=cfg['mission_altitude_agl_m']
    result=[(M.MAV_CMD_NAV_TAKEOFF,sx,sy,alt),(M.MAV_CMD_DO_CHANGE_SPEED,0,0,0)]
    result += [(M.MAV_CMD_NAV_WAYPOINT,*p) for p in mission(drone,compact)]
    result += [(M.MAV_CMD_NAV_WAYPOINT,sx,sy,alt),(M.MAV_CMD_NAV_LAND,sx,sy,0)]
    msgs=[]
    for seq,(cmd,x,y,z) in enumerate(result):
        lat,lon=global_xy(x,y)
        p1,p2,p3,p4=0,2,0,float('nan')
        if cmd==M.MAV_CMD_DO_CHANGE_SPEED:p1,p2,p3,p4=1,cfg['mission_speed_m_s'],-1,0
        frame=M.MAV_FRAME_MISSION if cmd==M.MAV_CMD_DO_CHANGE_SPEED else M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        if frame==M.MAV_FRAME_MISSION:lat,lon=0,0
        msgs.append(M.MAVLink_mission_item_int_message(drone['system_id'],1,seq,frame,cmd,0,1,p1,p2,p3,p4,int(lat*1e7),int(lon*1e7),z,M.MAV_MISSION_TYPE_MISSION))
    return msgs

class Fleet:
    def __init__(self,drones,output):
        self.drones=drones; self.output=output; self.data=[{} for _ in drones];self.last=[{} for _ in drones]
        self.links=[mavutil.mavlink_connection(f'udpin:127.0.0.1:{14540+d["instance"]}',source_system=250,source_component=190) for d in drones]
        self.last_beat=0; self.stop=False; self.start=time.monotonic(); self.events=[];self.last_log=0
        self.files=[];self.writers=[]
        self.columns=['sim_time_s','host_monotonic_s','host_unix_ns','px4_boot_ms','uav_id','x','y','z','latitude','longitude','altitude','vx','vy','vz','roll','pitch','yaw','roll_rate','pitch_rate','yaw_rate','ax','ay','az','battery_voltage','battery_current','battery_remaining','armed','flight_mode','mission_waypoint','wind_speed','wind_direction','temperature','humidity','nearest_uav','nearest_uav_distance']
        for d in drones:
            folder=output/d['name'].lower();folder.mkdir(parents=True,exist_ok=True)
            f=(folder/'telemetry.csv').open('w',newline='');self.files.append(f)
            writer=csv.DictWriter(f,fieldnames=self.columns);writer.writeheader();self.writers.append(writer)
    def command(self,i,cmd,*args):
        self.links[i].mav.command_long_send(self.drones[i]['system_id'],1,cmd,0,*(list(args)+[0]*(7-len(args))))
    def request_stream(self,i,message_id,rate_hz):
        self.command(i,M.MAV_CMD_SET_MESSAGE_INTERVAL,message_id,1_000_000/rate_hz)
    def world_snapshot(self):
        try:
            state=json.loads((ROOT/'.runtime/gz_state.json').read_text())
            if time.monotonic()-state['host_monotonic_s']>1:return None
            return state
        except (OSError,ValueError,KeyError):return None
    def pump(self,seconds=.1,callback=None):
        end=time.monotonic()+seconds
        while time.monotonic()<end and not self.stop:
            now=time.monotonic();beat=now-self.last_beat>.5
            for i,m in enumerate(self.links):
                if beat:m.mav.heartbeat_send(M.MAV_TYPE_GCS,M.MAV_AUTOPILOT_INVALID,0,0,0)
                for _ in range(200):
                    msg=m.recv_match()
                    if not msg:break
                    typ=msg.get_type()
                    if typ.startswith(('UNKNOWN','BAD_DATA')) or msg.get_srcSystem()!=self.drones[i]['system_id']:continue
                    self.data[i][typ]=msg;self.last[i][typ]=now
                    if typ in ('STATUSTEXT','COMMAND_ACK','MISSION_ACK','MISSION_ITEM_REACHED'):
                        self.events.append({'time':now,'uav':self.drones[i]['name'],'message':msg.to_dict()})
                        print(self.drones[i]['name'],msg,flush=True)
                    if callback:callback(i,msg)
            if beat:self.last_beat=now
            if now-self.last_log>=1/config('simulation')['logging_rate_hz']:
                self.log(now);self.last_log=now
            time.sleep(.005)
    def log(self,now):
        for i,(d,writer) in enumerate(zip(self.data,self.writers)):
            pos=d.get('GLOBAL_POSITION_INT'); local=d.get('LOCAL_POSITION_NED');att=d.get('ATTITUDE');imu=d.get('HIGHRES_IMU');hb=d.get('HEARTBEAT');bat=d.get('SYS_STATUS');cur=d.get('MISSION_CURRENT')
            if not pos or not hb or now-self.last[i].get('GLOBAL_POSITION_INT',0)>1:continue
            x=math.radians(pos.lon/1e7-8.545594)*6378137*math.cos(math.radians(47.397742)); y=math.radians(pos.lat/1e7-47.397742)*6378137
            weather=config('weather')[config('simulation')['weather_profile']]
            snapshot=self.world_snapshot();nearest_name='';nearest_distance=''
            if snapshot and self.drones[i]['name'] in snapshot['poses']:
                own=snapshot['poses'][self.drones[i]['name']][1:4]
                choices=[(math.dist(own,p[1:4]),name) for name,p in snapshot['poses'].items() if name!=self.drones[i]['name']]
                if choices:nearest_distance,nearest_name=min(choices)
            row=dict(sim_time_s=snapshot['sim_time_s'] if snapshot else '',host_monotonic_s=now,host_unix_ns=time.time_ns(),px4_boot_ms=pos.time_boot_ms,uav_id=self.drones[i]['name'],x=x,y=y,z=pos.alt/1000-488,latitude=pos.lat/1e7,longitude=pos.lon/1e7,altitude=pos.alt/1000,vx=pos.vy/100,vy=pos.vx/100,vz=-pos.vz/100,armed=bool(hb.base_mode&128),flight_mode=mavutil.mode_string_v10(hb),mission_waypoint=cur.seq if cur else '',wind_speed=weather['wind_speed'],wind_direction=weather['wind_direction'],temperature=weather['temperature'],humidity=weather['humidity'],nearest_uav=nearest_name,nearest_uav_distance=nearest_distance)
            if att:row.update(roll=att.roll,pitch=att.pitch,yaw=att.yaw,roll_rate=att.rollspeed,pitch_rate=att.pitchspeed,yaw_rate=att.yawspeed)
            if imu:row.update(ax=imu.xacc,ay=imu.yacc,az=imu.zacc)
            if bat:row.update(battery_voltage=bat.voltage_battery/1000 if bat.voltage_battery!=65535 else '',battery_current=bat.current_battery/100 if bat.current_battery!=-1 else '',battery_remaining=bat.battery_remaining if bat.battery_remaining!=-1 else '')
            writer.writerow(row)
    def wait(self,predicate,timeout,label):
        deadline=time.monotonic()+timeout
        while not self.stop and time.monotonic()<deadline:
            self.pump(.1)
            if predicate():return
        raise RuntimeError('Timeout/stopped: '+label)
    def close(self):
        for f in self.files:f.close()
        (self.output/'mission_events.json').write_text(json.dumps(self.events,indent=2))
        for m in self.links:m.close()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--compact',action='store_true');ap.add_argument('--count',type=int,default=4);ap.add_argument('--timeout',type=float,default=7200);args=ap.parse_args()
    state=json.loads((ROOT/'.runtime/state.json').read_text());output=Path(state['experiment'])
    drones=config('drones')['drones'][:args.count];f=Fleet(drones,output)
    def stop(*_):f.stop=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    result={'passed':False,'compact':args.compact};armed=False
    try:
        f.wait(lambda:all('GLOBAL_POSITION_INT' in d and 'LOCAL_POSITION_NED' in d and 'HEARTBEAT' in d for d in f.data),180,'telemetry readiness')
        for i in range(len(drones)):
            f.request_stream(i,M.MAVLINK_MSG_ID_HIGHRES_IMU,config('simulation')['logging_rate_hz'])
        f.pump(6)
        for i,drone in enumerate(drones):
            plan=items(drone,args.compact);ack=[]
            def upload(j,msg):
                if i!=j:return
                if msg.get_type()=='MISSION_REQUEST_INT':
                    if not 0<=msg.seq<len(plan):raise RuntimeError('Invalid requested mission sequence')
                    f.links[i].mav.send(plan[msg.seq])
                elif msg.get_type()=='MISSION_ACK':ack.append(msg.type)
            for attempt in range(3):
                f.links[i].mav.mission_count_send(drone['system_id'],1,len(plan),M.MAV_MISSION_TYPE_MISSION)
                end=time.monotonic()+15
                while not ack and time.monotonic()<end:f.pump(.1,upload)
                if ack:break
            if not ack or ack[-1]!=M.MAV_MISSION_ACCEPTED:raise RuntimeError(f'Mission upload failed for {drone["name"]}: {ack}')
            (output/drone['name'].lower()/'mission.json').write_text(json.dumps([m.to_dict() for m in plan],indent=2))
        f.wait(lambda:all('SYS_STATUS' in d and d['SYS_STATUS'].onboard_control_sensors_health & M.MAV_SYS_STATUS_PREARM_CHECK for d in f.data),120,'PX4 prearm health checks')
        for i in range(len(drones)):f.command(i,M.MAV_CMD_COMPONENT_ARM_DISARM,1)
        armed=True
        f.wait(lambda:all(d['HEARTBEAT'].base_mode&128 for d in f.data),15,'arm')
        for i in range(len(drones)):f.command(i,M.MAV_CMD_MISSION_START,0,0)
        f.pump(5)
        airborne=[False]*len(drones); reached=[set() for _ in drones]
        deadline=time.monotonic()+args.timeout;paused=set();safety_events=[]
        def progress(i,msg):
            if msg.get_type()=='MISSION_ITEM_REACHED':reached[i].add(msg.seq)
        while not f.stop and time.monotonic()<deadline:
            f.pump(.1,progress)
            for i,d in enumerate(f.data):
                if -d['LOCAL_POSITION_NED'].z>5:airborne[i]=True
                if time.monotonic()-f.last[i].get('HEARTBEAT',0)>5:raise RuntimeError('Lost vehicle heartbeat')
            snapshot=f.world_snapshot()
            if snapshot:
                poses=snapshot['poses']; unsafe=set()
                for a,b in itertools.combinations(range(len(drones)),2):
                    na,nb=drones[a]['name'],drones[b]['name']
                    if na not in poses or nb not in poses:continue
                    distance=math.dist(poses[na][1:4],poses[nb][1:4])
                    if distance<config('simulation')['safety_separation_m']:
                        hold=max(a,b);unsafe.add(hold)
                        if hold not in paused:
                            f.command(hold,M.MAV_CMD_DO_PAUSE_CONTINUE,0)
                            paused.add(hold);safety_events.append({'sim_time_s':snapshot['sim_time_s'],'action':'pause','uav':drones[hold]['name'],'distance_m':distance})
                if not unsafe and paused:
                    for hold in sorted(paused):
                        f.command(hold,M.MAV_CMD_DO_PAUSE_CONTINUE,1)
                        safety_events.append({'sim_time_s':snapshot['sim_time_s'],'action':'resume','uav':drones[hold]['name']})
                    paused.clear()
            if all(airborne) and all(not d['HEARTBEAT'].base_mode&128 for d in f.data):break
        complete=[len(items(d,args.compact))-2 in reached[i] for i,d in enumerate(drones)]
        result.update(airborne=airborne,returned_home_waypoint_reached=complete,disarmed=[not bool(d['HEARTBEAT'].base_mode&128) for d in f.data],reached_sequences=[sorted(x) for x in reached],safety_events=safety_events)
        result['passed']=all(airborne) and all(complete) and all(result['disarmed'])
        if not result['passed']:raise RuntimeError('Mission incomplete; see mission_result.json')
    except Exception as exc:
        result['error']=str(exc);print('MISSION FAILED:',exc,flush=True)
        if armed:
            f.stop=False
            for i in range(len(drones)):f.command(i,M.MAV_CMD_NAV_RETURN_TO_LAUNCH)
            # Keep GCS heartbeat active while PX4 attempts a controlled recovery.
            try:f.wait(lambda:all(not d.get('HEARTBEAT') or not d['HEARTBEAT'].base_mode&128 for d in f.data),90,'recovery landing')
            except RuntimeError:pass
    finally:
        (output/'mission_result.json').write_text(json.dumps(result,indent=2));f.close()
    if not result['passed']:raise SystemExit(1)
    print('All missions completed and landed.',flush=True)
if __name__=='__main__':main()
