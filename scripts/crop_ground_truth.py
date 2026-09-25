"""Synthetic region labels and idealized spectral bands; no spectral ray tracing."""
import json
from common import ROOT,config

def regions():
    data=json.loads((ROOT/'models/crop_regions.json').read_text())
    weather=config('weather')[config('simulation')['weather_profile']]
    for row in data:row['soil_moisture']=min(100,row['soil_moisture']*weather.get('moisture_scale',1))
    return data

def sample_region(x,y,data=None):
    for r in data if data is not None else regions():
        xmin,xmax,ymin,ymax=r['bounds']
        if xmin<=x<xmax and ymin<=y<ymax:return r
    return None

def ndvi(red,nir):
    if red<0 or nir<0:raise ValueError('Reflectance cannot be negative')
    return (nir-red)/(nir+red) if nir+red>0 else None
