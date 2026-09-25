#!/usr/bin/env python3
"""Gazebo clock/world-pose and sensor recorder. Never manufactures sensor samples."""
import csv
import argparse
import json
import os
import queue
import signal
import threading
import time
from collections import deque
from pathlib import Path
import numpy as np
from PIL import Image as PILImage
from common import ROOT,config
from environment_manager import weather_at
from crop_ground_truth import regions,sample_region
from soil_sensor_manager import samples
from multi_uav_monitor import SeparationMonitor
from gazebo_api import Node,Clock,Pose_V,Image,stamp

class Recorder:
    def __init__(self,experiment,partition,count):
        os.environ['GZ_PARTITION']=partition
        self.path=experiment;self.node=Node();self.lock=threading.Lock();self.sim=0;self.poses={};self.buffers={};self.queue=queue.Queue(64);self.dropped=0;self.errors=[];self.counts={};self.stop=False;self.callbacks=[]
        self.files=[];self.manifests={};self.last_pose=-1;self.last_environment=-1
        self.monitor=SeparationMonitor(config('simulation')['safety_separation_m'])
        self.regions=regions()
        envdir=experiment/'environment';envdir.mkdir(exist_ok=True)
        def csvwriter(path,columns):
            f=path.open('w',newline='');self.files.append(f);w=csv.DictWriter(f,fieldnames=columns);w.writeheader();return w
        self.weatherwriter=csvwriter(envdir/'weather.csv',['sim_time_s',*weather_at(0)])
        self.soilwriter=csvwriter(envdir/'soil.csv',['sim_time_s',*next(samples())])
        self.separationwriter=csvwriter(experiment/'separation.csv',['sim_time_s','uav_a','uav_b','distance_m','minimum_m','warning'])
        self.spectralwriter=csvwriter(envdir/'synthetic_observations.csv',['sim_time_s','uav_id','region_id','red','green','blue','red_edge','nir','NDVI'])
        columns=['sim_time_s',*self.regions[0]]
        groundwriter=csvwriter(envdir/'crop_ground_truth.csv',columns)
        for r in self.regions:groundwriter.writerow({'sim_time_s':0,**r})
        (envdir/'crop_ground_truth.json').write_text(json.dumps(self.regions,indent=2))
        names=[d['name'] for d in config('drones')['drones'][:count]]
        self.names=set(names)
        for name in names:
            self.buffers[name]=deque(maxlen=1000)
            for kind in ('rgb','depth'):
                folder=experiment/name.lower()/('camera' if kind=='rgb' else 'depth');folder.mkdir(parents=True,exist_ok=True)
                file=(folder/'manifest.csv').open('w',newline='');self.files.append(file)
                writer=csv.writer(file);writer.writerow(['sim_time_s','host_unix_ns','file','pose_sim_time_s','pose_delta_s','x','y','z','qx','qy','qz','qw','width','height','pixel_format']);self.manifests[name,kind]=(writer,folder)
                topic=f'/{name.lower()}/{kind}'
                callback=lambda msg,n=name,k=kind:self.enqueue(n,k,msg)
                self.callbacks.append(callback);self.node.subscribe(Image,topic,callback)
        posefile=(experiment/'ground_truth_poses.csv').open('w',newline='');self.files.append(posefile);self.posewriter=csv.writer(posefile)
        self.posewriter.writerow(['sim_time_s','host_unix_ns','uav_id','x','y','z','qx','qy','qz','qw'])
        world=config('simulation')['world_name']
        self.node.subscribe(Clock,f'/world/{world}/clock',self.clock)
        self.node.subscribe(Pose_V,f'/world/{world}/pose/info',self.pose)
    def clock(self,msg):
        with self.lock:self.sim=msg.sim.sec+msg.sim.nsec/1e9
    def pose(self,msg):
        t=stamp(msg.header)
        with self.lock:
            for p in msg.pose:
                if p.name in self.names:
                    values=[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
                    self.poses[p.name]=[t,*values];self.buffers[p.name].append([t,*values])
    def enqueue(self,name,kind,msg):
        try:self.queue.put_nowait((name,kind,msg,time.time_ns()))
        except queue.Full:self.dropped+=1
    def save(self,name,kind,msg,host):
        t=stamp(msg.header);writer,folder=self.manifests[name,kind]
        with self.lock:
            candidates=list(self.buffers[name])
        pose=min(candidates,key=lambda p:abs(p[0]-t)) if candidates else None
        if pose and abs(pose[0]-t)>.1:pose=None
        basename=f'{int(round(t*1e9)):019d}'
        if kind=='rgb':
            formats={3:('RGB',3),4:('RGBA',4),8:('BGR',3)}
            if msg.pixel_format_type not in formats:raise ValueError(f'Unsupported RGB format {msg.pixel_format_type}')
            mode,channels=formats[msg.pixel_format_type]
            raw=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*channels].reshape(msg.height,msg.width,channels)
            if mode=='BGR':raw=raw[:,:,::-1];mode='RGB'
            namefile=basename+'.png';PILImage.fromarray(raw,mode=mode).save(folder/namefile)
        else:
            if msg.pixel_format_type!=13:raise ValueError(f'Expected R_FLOAT32 depth; received {msg.pixel_format_type}')
            raw=np.frombuffer(msg.data,dtype='<f4').reshape(msg.height,msg.step//4)[:,:msg.width]
            namefile=basename+'.npy';np.save(folder/namefile,raw)
        writer.writerow([t,host,namefile,pose[0] if pose else '',abs(pose[0]-t) if pose else '',*(pose[1:] if pose else ['']*7),msg.width,msg.height,msg.pixel_format_type])
        key=name+'/'+kind;self.counts[key]=self.counts.get(key,0)+1
    def run(self):
        last=0
        try:
            while not self.stop or not self.queue.empty():
                try:self.save(*self.queue.get(timeout=.05))
                except queue.Empty:pass
                except Exception as exc:self.errors.append(str(exc))
                if time.monotonic()-last>.1:
                    with self.lock:sim=self.sim;poses=dict(self.poses)
                    for name,p in poses.items():
                        if p[0]>self.last_pose:self.posewriter.writerow([p[0],time.time_ns(),name,*p[1:]])
                    if poses:self.last_pose=max(p[0] for p in poses.values())
                    for row in self.monitor.update(poses,sim):self.separationwriter.writerow(row)
                    if sim-self.last_environment>=1:
                        self.weatherwriter.writerow({'sim_time_s':sim,**weather_at(sim)})
                        for row in samples():self.soilwriter.writerow({'sim_time_s':sim,**row})
                        for name,p in poses.items():
                            r=sample_region(p[1],p[2],self.regions)
                            if r:self.spectralwriter.writerow(dict(sim_time_s=p[0],uav_id=name,region_id=r['region_id'],red=r['red_value'],green=r['green_value'],blue=r['blue_value'],red_edge=r['red_edge_value'],nir=r['nir_value'],NDVI=r['NDVI']))
                        self.last_environment=sim
                    snapshot={'sim_time_s':sim,'host_monotonic_s':time.monotonic(),'poses':poses}
                    tmp=ROOT/'.runtime/gz_state.tmp';tmp.write_text(json.dumps(snapshot));tmp.replace(ROOT/'.runtime/gz_state.json')
                    for f in self.files:f.flush()
                    last=time.monotonic()
        finally:
            for topic in self.node.subscribed_topics():self.node.unsubscribe(topic)
            del self.node
            for f in self.files:f.close()
            (self.path/'sensor_recording.json').write_text(json.dumps({'frames':self.counts,'dropped_frames':self.dropped,'errors':self.errors[:30],'minimum_separation_m':min(self.monitor.minimum.values()) if self.monitor.minimum else None,'conflict_events':self.monitor.conflicts},indent=2))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--count',type=int,default=4,choices=range(1,5));args=ap.parse_args()
    state=json.loads((ROOT/'.runtime/state.json').read_text());rec=Recorder(Path(state['experiment']),state['partition'],args.count)
    def stop(*_):rec.stop=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop);rec.run()
if __name__=='__main__':main()
