#!/usr/bin/env python3
"""Offline deterministic farm generation. Coordinates: ENU, meters."""
import json
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path
from common import ROOT, config, elevation

def child(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, attrs)
    if text is not None: e.text = str(text)
    return e

def generate():
    cfg = config('simulation'); rng = random.Random(cfg['random_seed'])
    size = cfg['map_size_m']; scale = size/1000
    if size < 200 or cfg['quality'] not in ('low','medium','high'):
        raise ValueError('Map must be >=200m and quality low/medium/high')
    sdf = ET.Element('sdf', version='1.9'); world = child(sdf, 'world', name=cfg['world_name'])
    p = child(world,'physics',name='fixed_step',type='ignored')
    child(p,'max_step_size',0.004); child(p,'real_time_factor',1)
    child(world,'gravity','0 0 -9.80665')
    for lib, cls in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('imu','Imu'),('navsat','NavSat'),('air-pressure','AirPressure'),('magnetometer','Magnetometer')]:
        child(world,'plugin',filename=f'gz-sim-{lib}-system',name=f'gz::sim::systems::{cls}')
    light = child(world,'light',name='sun',type='directional')
    child(light,'pose','0 0 100 0 0 0'); child(light,'direction','-0.5 0.3 -0.9')
    child(light,'diffuse','0.95 0.91 0.80 1'); child(light,'cast_shadows','true')
    scene = child(world,'scene'); child(scene,'ambient','0.55 0.55 0.55 1'); child(scene,'background','0.65 0.8 0.92 1')
    sc=child(world,'spherical_coordinates'); child(sc,'surface_model','EARTH_WGS84')
    for tag,val in [('latitude_deg',47.397742),('longitude_deg',8.545594),('elevation',488),('heading_deg',0)]: child(sc,tag,val)

    def model(name):
        m=child(world,'model',name=name); child(m,'static','true'); return child(m,'link',name='link')
    def geom(link, name, pose, shape, dimensions, color, collision=True):
        for kind in (['visual','collision'] if collision else ['visual']):
            e=child(link,kind,name=name+'_'+kind); child(e,'pose',' '.join(map(str,pose)))
            g=child(child(e,'geometry'),shape)
            if shape=='box': child(g,'size',' '.join(map(str,dimensions)))
            elif shape=='cylinder': child(g,'radius',dimensions[0]); child(g,'length',dimensions[1])
            else: child(g,'uri',dimensions)
            if kind=='visual':
                mat=child(e,'material'); child(mat,'ambient',color); child(mat,'diffuse',color)
    # One triangle mesh shared by physical and visual terrain; no remote resources.
    n={'low':51,'medium':81,'high':121}[cfg['quality']]
    verts=[]; faces=[]
    for j in range(n):
        y=-size/2+j*size/(n-1)
        for i in range(n):
            x=-size/2+i*size/(n-1); verts.append((x,y,elevation(x,y,cfg)))
    for j in range(n-1):
        for i in range(n-1):
            a=j*n+i+1; faces += [(a,a+1,a+n),(a+1,a+n+1,a+n)]
    mesh=ROOT/'models/terrain/terrain.obj'
    normals=[]
    for x,y,z in verts:
        dx=(elevation(x+.01,y,cfg)-elevation(x-.01,y,cfg))/.02
        dy=(elevation(x,y+.01,cfg)-elevation(x,y-.01,cfg))/.02
        length=math.sqrt(dx*dx+dy*dy+1)
        normals.append((-dx/length,-dy/length,1/length))
    (mesh.parent/'terrain.mtl').write_text('newmtl soil\nKa 0.35 0.42 0.20\nKd 0.35 0.42 0.20\n')
    mesh.write_text('\n'.join(['mtllib terrain.mtl','o terrain','usemtl soil']+['v '+' '.join(f'{v:.5f}' for v in xyz) for xyz in verts]+['vn '+' '.join(f'{v:.6f}' for v in xyz) for xyz in normals]+['f '+' '.join(f'{i}//{i}' for i in f) for f in faces])+'\n')
    geom(model('terrain'),'terrain',[0]*6,'mesh','../models/terrain/terrain.obj','0.35 0.42 0.20 1')
    roads=model('farm_roads')
    for x,y,sx,sy in [(0,0,size,12),(0,0,12,size)]: geom(roads,'road'+str(sx),[x,y,0.035,0,0,0],'box',[sx,sy,0.05],'0.43 0.34 0.22 1',False)
    truth=[]
    for field in config('fields')['fields']:
        xmin,xmax,ymin,ymax=[v*scale for v in field['bounds']]
        link=model('crop_field_'+field['id'])
        pitch={'low':8,'medium':5,'high':3}[cfg['quality']]*scale
        # Tiled short row segments conform to the gentle terrain and vary in health.
        for region in range(4):
            xa=xmin+(xmax-xmin)*(region%2)/2; xb=xa+(xmax-xmin)/2
            ya=ymin+(ymax-ymin)*(region//2)/2; yb=ya+(ymax-ymin)/2
            unhealthy = field['id'] in ('B','D') and region==2
            red=field['red']*(1.4 if unhealthy else 1); nir=field['nir']*(0.7 if unhealthy else 1)
            truth.append(dict(region_id=f"Field_{field['id']}_Region_{region+1:02}",field_id=field['id'],crop_type=field['crop_type'],bounds=[xa,xb,ya,yb],health_state='stressed' if unhealthy else field['health'],vegetation_density=field['density'],soil_moisture=field['moisture'],water_stress=1-field['moisture']/50,disease_state='suspected' if field['id']=='D' and unhealthy else 'none',red_value=red,green_value=.18,blue_value=.06,red_edge_value=(red+nir)/2,nir_value=nir,NDVI=(nir-red)/(nir+red)))
            row=0; x=xa+pitch/2
            while x<xb:
                for seg in range(8):
                    if field['id']=='D' and rng.random()>.8: continue
                    y=ya+(seg+.5)*(yb-ya)/8
                    h=(.7 if field['id'] in ('A','B') else 1.6)*(rng.uniform(.45,1) if field['id']=='C' else 1)
                    color='0.22 0.43 0.08 1' if field['id']=='A' else ('0.55 0.48 0.13 1' if unhealthy else '0.36 0.46 0.11 1')
                    geom(link,f'r{region}_{row}_{seg}',[x,y,elevation(x,y,cfg)+h/2,0,0,0],'box',[pitch*.35,(yb-ya)/8*.95,h],color,False)
                row+=1; x+=pitch
    structures=model('farm_structures')
    for name,x,y,sx,sy,h,color in [('farmhouse',-22,20,16,12,7,'0.8 0.72 0.57 1'),('barn',22,24,22,16,9,'0.55 0.18 0.12 1'),('storage',-25,-25,18,12,5,'0.55 0.58 0.6 1'),('base_station',15,-22,12,8,3,'0.25 0.35 0.44 1'),('greenhouse',-440,60,30,65,6,'0.60 0.81 0.76 0.55'),('pond',440,130,65,100,.15,'0.12 0.35 0.48 1'),('water_tank',430,20,12,12,7,'0.55 0.64 0.7 1'),('tractor',-25,-9,5,3,2.5,'0.1 0.45 0.13 1')]:
        x*=scale;y*=scale
        geom(structures,name,[x,y,elevation(x,y,cfg)+h/2,0,0,0],'box',[sx*scale,sy*scale,h],color,name!='pond')
    orchard=model('orchard')
    for i in range(6):
        for j in range(12):
            x=(405+i*12)*scale;y=(-320+j*22)*scale; z=elevation(x,y,cfg)
            geom(orchard,f'trunk{i}_{j}',[x,y,z+2,0,0,0],'cylinder',[.4,4],'0.3 0.19 0.1 1')
            geom(orchard,f'canopy{i}_{j}',[x,y,z+5,0,0,0],'cylinder',[4,4],'0.18 0.36 0.10 1',False)
    infrastructure=model('irrigation_fences_weather_solar')
    for idx in range(20):
        x=(-475+idx*50)*scale
        for y in [-490*scale,490*scale]:
            if y < 0 and idx in (9,10):
                continue
            geom(infrastructure,f'fence{idx}_{y}',[x,y,elevation(x,y,cfg)+.8,0,0,0],'box',[49*scale,.15,1.6],'0.52 0.47 0.35 1')
    # Perimeter fence: the south side leaves a 100 m road entrance.
    for idx in range(20):
        y=(-475+idx*50)*scale
        for x in [-490*scale,490*scale]:
            geom(infrastructure,f'fence_side{idx}_{x}',[x,y,elevation(x,y,cfg)+.8,0,0,0],'box',[.15,49*scale,1.6],'0.52 0.47 0.35 1')
    for idx in (9,10):
        x=(-475+idx*50)*scale;y=-490*scale
        geom(infrastructure,f'gate_post{idx}',[x,y,elevation(x,y,cfg)+1.5,0,0,0],'box',[.35,.35,3],'0.40 0.36 0.28 1')
        geom(infrastructure,f'open_gate{idx}',[x+(25 if idx==9 else -25)*scale,y,elevation(x,y,cfg)+1.2,0,0,(.45 if idx==9 else -.45)],'box',[48*scale,.12,2.4],'0.52 0.47 0.35 1',False)
    for i in range(4):
        x=(-6+i*4)*scale
        geom(infrastructure,f'pad{i}',[x,-6*scale,.05,0,0,0],'box',[3*scale,3*scale,.1],'0.8 0.8 0.8 1')
    for i in range(8):
        x=(-350+i*100)*scale; y=-22*scale
        geom(infrastructure,f'pipe{i}',[x,y,elevation(x,y,cfg)+.15,0,math.pi/2,0],'cylinder',[.15,95*scale],'0.24 0.32 0.37 1')
        geom(infrastructure,f'sprinkler{i}',[x,y,elevation(x,y,cfg)+.5,0,0,0],'cylinder',[.1,1],'0.25 0.3 0.3 1')
    geom(infrastructure,'irrigation_channel',[-210*scale,-30*scale,.08,0,0,0],'box',[340*scale,3*scale,.12],'0.10 0.31 0.42 1',False)
    for i in range(7):
        x=(-300+i*100)*scale;y=18*scale;z=elevation(x,y,cfg)
        geom(infrastructure,f'electric_pole{i}',[x,y,z+4,0,0,0],'cylinder',[.16,8],'0.35 0.27 0.18 1')
        geom(infrastructure,f'electric_crossarm{i}',[x,y,z+7.5,0,0,0],'box',[.3,4,.2],'0.30 0.23 0.16 1')
    geom(infrastructure,'weather_mast',[-12,12,4,0,0,0],'cylinder',[.13,8],'0.7 0.7 0.7 1')
    geom(infrastructure,'weather_head',[-12,12,8,0,0,0],'box',[2,1,.3],'0.9 0.9 0.9 1')
    for i in range(5): geom(infrastructure,f'solar{i}',[12+i*3,-32,2,0,.35,0],'box',[2.5,5,.15],'0.08 0.15 0.27 1')
    ET.indent(sdf); target=ROOT/'worlds/agriculture_world.sdf'; ET.ElementTree(sdf).write(target,encoding='unicode',xml_declaration=True)
    (ROOT/'models/crop_regions.json').write_text(json.dumps(truth,indent=2)+'\n')
    print(f'Generated {target}: {len(verts)} terrain vertices; {len(world.findall("model"))} static models; {len(truth)} crop regions')
if __name__=='__main__': generate()
