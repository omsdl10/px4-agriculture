#!/usr/bin/env python3
"""Own all child processes; never kill other projects' simulators."""
import argparse
import datetime
import fcntl
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from common import ROOT, config
from generate_world import generate

RUNTIME=ROOT/'.runtime'

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--count',type=int,default=4,choices=range(1,5))
    ap.add_argument('--world-only',action='store_true')
    ap.add_argument('--mission',choices=['none','compact','full'],default='full')
    ap.add_argument('--headless',action='store_true')
    ap.add_argument('--pov',choices=['UAV1','UAV2','UAV3','UAV4'],help='Open a forward-facing onboard viewpoint after PX4 startup')
    ap.add_argument('--sensors',action=argparse.BooleanOptionalAction,default=True)
    ap.add_argument('--wind',action=argparse.BooleanOptionalAction,default=True)
    ap.add_argument('--duration',type=float,default=0,help='Wall seconds; 0 runs until stopped')
    args=ap.parse_args()
    RUNTIME.mkdir(exist_ok=True)
    lock=(RUNTIME/'lock').open('w')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: raise SystemExit('This project is already running. Use stop_simulation.sh.')
    cfg=config('simulation'); drones=config('drones')['drones'][:args.count]
    if args.pov and (args.world_only or args.pov not in [d['name'] for d in drones]):
        raise SystemExit('POV requires the selected drone to be spawned')
    px4=Path(os.environ.get('PX4_ROOT',cfg['px4_root'])).expanduser(); build=px4/'build/px4_sitl_default'
    if not args.world_only:
        if not (build/'bin/px4').is_file(): raise SystemExit(f'No compiled PX4: {build}/bin/px4')
        for d in drones:
            if d['system_id']!=d['instance']+1: raise SystemExit('PX4 rcS requires system_id=instance+1')
            for port in [14540+d['instance'],14580+d['instance'],18570+d['instance']]:
                with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
                    try: s.bind(('127.0.0.1',port))
                    except OSError: raise SystemExit(f'Port {port} is occupied; refusing to interfere.')
    generate()
    if args.sensors or args.wind:
        from generate_vehicle import generate_vehicles
        import xml.etree.ElementTree as ET
        generate_vehicles(px4,sensors=args.sensors,wind=args.wind)
        world_path=ROOT/'worlds/agriculture_world.sdf'
        tree=ET.parse(world_path)
        world=tree.getroot().find('world')
        if args.sensors:
            plugin=ET.SubElement(world,'plugin',filename='gz-sim-sensors-system',name='gz::sim::systems::Sensors')
            ET.SubElement(plugin,'render_engine').text='ogre2'
        if args.wind:
            import math
            w=config('weather')[cfg['weather_profile']]
            direction=math.radians(w['wind_direction'])
            wind=ET.SubElement(world,'wind');ET.SubElement(wind,'linear_velocity').text=f"{w['wind_speed']*math.cos(direction)} {w['wind_speed']*math.sin(direction)} 0"
            plugin=ET.SubElement(world,'plugin',filename='gz-sim-wind-effects-system',name='gz::sim::systems::WindEffects')
            ET.SubElement(plugin,'force_approximation_scaling_factor').text='0.1'
            horizontal=ET.SubElement(plugin,'horizontal');magnitude=ET.SubElement(horizontal,'magnitude')
            ET.SubElement(magnitude,'time_for_rise').text='1'
            wave=ET.SubElement(magnitude,'sin');ET.SubElement(wave,'amplitude_percent').text=str((w['wind_gust']-w['wind_speed'])/max(w['wind_speed'],.01))
            ET.SubElement(wave,'period').text='20'
        tree.write(world_path,encoding='unicode')
    experiment=ROOT/'experiments'/datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    experiment.mkdir(); (experiment/'logs').mkdir()
    shutil.copytree(ROOT/'config',experiment/'config')
    env=os.environ.copy(); env.update(GZ_PARTITION='agriculture_'+str(os.getpid()),GZ_SIM_RESOURCE_PATH=str(ROOT/'models')+':'+str(px4/'Tools/simulation/gz/models'),GZ_SIM_SYSTEM_PLUGIN_PATH=str(build/'src/modules/simulation/gz_plugins'),PX4_GZ_STANDALONE='1',PX4_GZ_NO_FOLLOW='1',PX4_GZ_WORLD=cfg['world_name'],PX4_GZ_MODELS=str(px4/'Tools/simulation/gz/models'),PX4_GZ_WORLDS=str(ROOT/'worlds'))
    # PX4 shell startup does not support spaces in its working directory.
    work=Path(tempfile.mkdtemp(prefix='agri_px4_'))
    state={'pid':os.getpid(),'experiment':str(experiment),'workdir':str(work),'partition':env['GZ_PARTITION']}
    (RUNTIME/'state.json').write_text(json.dumps(state)); (RUNTIME/'latest').write_text(str(experiment))
    children=[]; logs=[]; stopping=False
    def stop(signum=None,frame=None):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    def launch(name,cmd,cwd=None,extra=None):
        log=(experiment/'logs'/f'{name}.log').open('w'); logs.append(log)
        proc=subprocess.Popen(cmd,cwd=cwd or ROOT,env=env| (extra or {}),stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
        children.append((name,proc)); return proc
    failed=None
    try:
        gz=launch('gazebo',['gz','sim','-r','-v','3']+(['-s'] if args.headless or args.pov else [])+['--seed',str(cfg['random_seed']),str(ROOT/'worlds/agriculture_world.sdf')])
        deadline=time.monotonic()+45
        while True:
            if stopping: return
            if gz.poll() is not None: raise RuntimeError('Gazebo exited; inspect logs/gazebo.log')
            result=subprocess.run(['gz','service','-l'],env=env,capture_output=True,text=True,timeout=8)
            if f'/world/{cfg["world_name"]}/create' in result.stdout: break
            if time.monotonic()>deadline: raise RuntimeError('Gazebo startup timed out')
            time.sleep(.5)
        if not args.world_only:
            for d in drones:
                wd=work/d['name'];wd.mkdir()
                x,y,z=d['spawn']
                model='agri_'+d['name'].lower() if args.sensors or args.wind else 'x500'
                sdf=f'<sdf version="1.9"><include><uri>model://{model}</uri><pose>{x} {y} {z} 0 0 0</pose></include></sdf>'
                req=f'name: "{d["name"]}", allow_renaming: false, sdf: {json.dumps(sdf)}'
                out=subprocess.run(['gz','service','-s',f'/world/{cfg["world_name"]}/create','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','5000','--req',req],env=env,capture_output=True,text=True,timeout=10)
                if out.returncode or 'data: true' not in out.stdout: raise RuntimeError('Spawn failed: '+out.stdout+out.stderr)
                vehicle=launch(d['name'],[str(build/'bin/px4'),str(build/'etc'),'-s','etc/init.d-posix/rcS','-i',str(d['instance']),'-d'],wd,dict(PX4_SYS_AUTOSTART='4001',PX4_SIM_MODEL='gz_x500',PX4_GZ_MODEL_NAME=d['name'],PX4_UXRCE_DDS_NS='px4_'+str(d['system_id'])))
                startup_log=experiment/'logs'/f"{d['name']}.log"
                startup_deadline=time.monotonic()+120
                while time.monotonic()<startup_deadline:
                    if vehicle.poll() is not None:raise RuntimeError(f"{d['name']} exited during startup; inspect logs/{d['name']}.log")
                    if 'Startup script returned successfully' in startup_log.read_text(errors='replace'):break
                    time.sleep(.25)
                else:raise RuntimeError(f"{d['name']} startup timed out; inspect logs/{d['name']}.log")
        if args.pov:
            launch('pov_gui',['gz','sim','-g','-v','3'])
            gui_deadline=time.monotonic()+45
            while time.monotonic()<gui_deadline:
                services=subprocess.run(['gz','service','-l'],env=env,capture_output=True,text=True,timeout=8)
                if '/gui/move_to/pose' in services.stdout:break
                time.sleep(.5)
            else:raise RuntimeError('Gazebo POV window did not become ready')
            track=f'track_mode: FOLLOW_LOOK_AT, follow_target: {{name: "{args.pov}"}}, track_target: {{name: "{args.pov}"}}, follow_offset: {{x: 0.4, y: 0, z: 0.15}}, track_offset: {{x: 15, y: 0, z: 0.15}}, follow_pgain: 1.0, track_pgain: 1.0'
            for _ in range(3):
                subprocess.run(['gz','topic','-t','/gui/track','-m','gz.msgs.CameraTrack','-p',track],env=env,check=True,timeout=8)
                time.sleep(.3)
            print(f'{args.pov} forward-facing POV ready',flush=True)
        if not args.world_only:
            launch('data_logger',[sys.executable,str(ROOT/'scripts/data_logger.py'),'--count',str(args.count)])
        if not args.world_only and args.mission!='none':
            launch('mission',[sys.executable,str(ROOT/'scripts/mission_controller.py'),'--count',str(args.count)]+(['--compact'] if args.mission=='compact' else []))
        print('Started '+str(experiment),flush=True)
        (experiment/'metadata.json').write_text(json.dumps(state|{'platform':sys.platform,'seed':cfg['random_seed'],'count':0 if args.world_only else len(drones),'status':'running','sensors_enabled':args.sensors,'wind_physics_enabled':args.wind,'mission':args.mission},indent=2))
        start=time.monotonic()
        while not stopping and (not args.duration or time.monotonic()-start<args.duration):
            for name,proc in children:
                if proc.poll() is not None:
                    if name=='mission' and proc.returncode==0: stopping=True;break
                    raise RuntimeError(f'{name} exited with {proc.returncode}; inspect logs')
            time.sleep(.2)
    except Exception as exc:
        failed=str(exc); print(failed,file=sys.stderr)
    finally:
        # Reverse startup order: clients/PX4 before Gazebo. Signal only owned process groups.
        for name,proc in reversed(children):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try: proc.wait(timeout=120 if name=='mission' else 12)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try: proc.wait(timeout=5)
                    except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for log in logs: log.close()
        for d in drones:
            src=work/d['name']/'log'
            if src.exists(): shutil.copytree(src,experiment/d['name'].lower()/'ulog',dirs_exist_ok=True)
        if any((experiment/d['name'].lower()/'telemetry.csv').is_file() for d in drones):
            with (experiment/'logs/analysis.log').open('w') as analysis_log:
                analysis_result=subprocess.run([sys.executable,str(ROOT/'analysis/analyze_experiment.py'),str(experiment)],cwd=ROOT,stdout=analysis_log,stderr=subprocess.STDOUT)
            if analysis_result.returncode and not failed:failed='Analysis failed; inspect logs/analysis.log'
        abnormal=[n for n,p in children if p.returncode in (-signal.SIGSEGV,-signal.SIGABRT,-signal.SIGKILL)]
        if abnormal and not failed:failed='Abnormal shutdown: '+','.join(abnormal)
        (experiment/'shutdown.json').write_text(json.dumps({'clean':not bool(abnormal),'error':failed,'children':[{ 'name':n,'returncode':p.returncode} for n,p in children]},indent=2))
        metadata=experiment/'metadata.json'
        if metadata.exists():
            info=json.loads(metadata.read_text());info['status']='failed' if failed else 'stopped';metadata.write_text(json.dumps(info,indent=2))
        if metadata.exists() and not failed and not args.world_only:
            with (experiment/'logs/acceptance.log').open('w') as acceptance_log:
                acceptance_result=subprocess.run([sys.executable,str(ROOT/'scripts/validate_experiment.py'),str(experiment)],cwd=ROOT,stdout=acceptance_log,stderr=subprocess.STDOUT)
            if acceptance_result.returncode:
                failed='Acceptance validation failed; inspect logs/acceptance.log'
                shutdown=json.loads((experiment/'shutdown.json').read_text());shutdown['error']=failed;(experiment/'shutdown.json').write_text(json.dumps(shutdown,indent=2))
                info=json.loads(metadata.read_text());info['status']='failed';metadata.write_text(json.dumps(info,indent=2))
        (RUNTIME/'state.json').unlink(missing_ok=True)
        (RUNTIME/'gz_state.json').unlink(missing_ok=True)
        shutil.rmtree(work)
    if failed: raise SystemExit(1)
if __name__=='__main__': main()
