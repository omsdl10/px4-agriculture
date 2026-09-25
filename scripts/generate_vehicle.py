"""Derive local sensor-equipped models from the installed PX4 X500, preserving dynamics."""
import copy
import math
import xml.etree.ElementTree as ET
from common import ROOT,config
from generate_world import child

def generate_vehicles(px4,sensors=False,wind=False):
    models=px4/'Tools/simulation/gz/models'
    settings=config('sensors')
    for d in config('drones')['drones']:
        name='agri_'+d['name'].lower();folder=ROOT/'models'/name;folder.mkdir(exist_ok=True)
        base=ET.parse(models/'x500_base/model.sdf').getroot()
        model=base.find('model');model.set('name',name)
        for plugin in ET.parse(models/'x500/model.sdf').findall('model/plugin'):model.append(copy.deepcopy(plugin))
        link=model.find("link[@name='base_link']")
        if wind:child(link,'enable_wind','true')
        if sensors:
            for kind in ('rgb','depth'):
                c=settings[kind]
                if not c['enabled']:continue
                sensor=child(link,'sensor',name=kind,type='camera' if kind=='rgb' else 'depth_camera')
                child(sensor,'pose',f'0 0 -0.08 0 {math.pi/2} 0');child(sensor,'always_on','true');child(sensor,'update_rate',c['rate_hz'])
                child(sensor,'topic',f'/{d["name"].lower()}/{kind}')
                camera=child(sensor,'camera');child(camera,'horizontal_fov',c['horizontal_fov_rad'])
                image=child(camera,'image');child(image,'width',c['width']);child(image,'height',c['height']);child(image,'format','R8G8B8')
                clip=child(camera,'clip');child(clip,'near',.1);child(clip,'far',100)
        # Retain the upstream sensor definitions and motor coefficients unchanged.
        ET.indent(base);ET.ElementTree(base).write(folder/'model.sdf',encoding='unicode',xml_declaration=True)
        (folder/'model.config').write_text(f'<model><name>{name}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><author><name>Agriculture simulation</name></author><description>Derived PX4 X500 with downward agricultural sensors</description></model>')
        license_path=models/'x500_base/LICENSE'
        if license_path.exists():(folder/'UPSTREAM_LICENSE').write_text(license_path.read_text())
