"""Pair distances use fresh Gazebo world ENU poses, never per-UAV local origins."""
import itertools
import math
class SeparationMonitor:
    def __init__(self,threshold):
        if threshold<=0:raise ValueError('Separation threshold must be positive')
        self.threshold=threshold;self.minimum={};self.active=set();self.conflicts=0
    def update(self,poses,sim_time,max_age=.3):
        fresh={name:p for name,p in poses.items() if 0<=sim_time-p[0]<=max_age}
        rows=[];active=set()
        for a,b in itertools.combinations(sorted(fresh),2):
            p,q=fresh[a],fresh[b]
            if abs(p[0]-q[0])>.05:continue
            d=math.dist(p[1:4],q[1:4]);key=(a,b);warn=d<self.threshold
            self.minimum[key]=min(self.minimum.get(key,float('inf')),d)
            if warn:
                active.add(key)
                if key not in self.active:self.conflicts+=1
            rows.append(dict(sim_time_s=sim_time,uav_a=a,uav_b=b,distance_m=d,minimum_m=self.minimum[key],warning=warn))
        self.active=active
        return rows
