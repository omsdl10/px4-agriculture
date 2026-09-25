import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common import config,elevation,ROOT
from planner import lawnmower,mission
class CoreTests(unittest.TestCase):
    def test_lawnmower(self):
        points=lawnmower([0,100,0,50],12)
        self.assertEqual(len(points),18)
        for x,y in points:self.assertTrue(0<x<100 and y in (0,50))
        for a,b in zip(points[1::2],points[2::2]):self.assertEqual(a[1],b[1])
        with self.assertRaises(ValueError):lawnmower([0,10,0,10],0)
    def test_unique_ids_ports_and_spawn_separation(self):
        drones=config('drones')['drones']
        self.assertEqual(len({d['instance'] for d in drones}),4)
        self.assertEqual(len({d['system_id'] for d in drones}),4)
        for i,d in enumerate(drones):
            self.assertEqual(d['system_id'],d['instance']+1)
            for other in drones[i+1:]:self.assertGreater(math.dist(d['spawn'],other['spawn']),config('simulation')['safety_separation_m'])
    def test_terrain_and_agl(self):
        self.assertEqual(elevation(0,0),0)
        self.assertGreater(elevation(300,300),0)
        for d in config('drones')['drones']:
            for x,y,z in mission(d,True):self.assertAlmostEqual(z-elevation(x,y),15)
    def test_local_world_resources(self):
        world=ET.parse(ROOT/'worlds/agriculture_world.sdf')
        names=[x.attrib['name'] for x in world.findall('.//world/model')]
        self.assertEqual(len(names),len(set(names)))
        for uri in world.findall('.//uri'):
            self.assertNotIn('http',uri.text)
            self.assertTrue((ROOT/'worlds'/uri.text).is_file())
if __name__=='__main__':unittest.main()
