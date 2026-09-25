"""Replace this policy for future AI controllers; output is world ENU waypoints."""
import math
from common import config, elevation

def lawnmower(bounds,spacing):
    xmin,xmax,ymin,ymax=bounds
    if not xmin<xmax or not ymin<ymax or spacing<=0: raise ValueError('Invalid survey bounds/spacing')
    count=max(1,math.ceil((xmax-xmin)/spacing))
    result=[]
    for row in range(count):
        x=xmin+(row+.5)*(xmax-xmin)/count
        ends=[ymin,ymax] if row%2==0 else [ymax,ymin]
        result.extend((x,y) for y in ends)
    return result

def mission(drone,compact=False):
    cfg=config('simulation');scale=cfg['map_size_m']/1000
    field=next(f for f in config('fields')['fields'] if f['id']==drone['field'])
    bounds=[v*scale for v in field['bounds']]
    if compact:
        # A small real subset inside each assigned field for rapid regression flights.
        x=-60 if drone['field'] in ('A','C') else 60
        y=60 if drone['field'] in ('A','B') else -60
        bounds=[(x-10)*scale,(x+10)*scale,(y-10)*scale,(y+10)*scale]
    spacing=cfg.get('survey_spacing_m',12)
    xy=lawnmower(bounds,spacing)
    if not compact and drone['role']=='irrigation': xy += [(-100*scale,-22*scale),(0,-22*scale),(440*scale,130*scale)]
    if not compact and drone['role']=='anomaly_inspection': xy += [(435*scale,-200*scale),(455*scale,-100*scale)]
    alt=cfg['mission_altitude_agl_m']
    return [(x,y,elevation(x,y,cfg)+alt) for x,y in xy]
