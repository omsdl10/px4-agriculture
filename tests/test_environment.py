import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from crop_ground_truth import ndvi,regions,sample_region
from multi_uav_monitor import SeparationMonitor
from soil_sensor_manager import samples
class EnvironmentTests(unittest.TestCase):
    def test_ndvi(self):
        self.assertAlmostEqual(ndvi(.1,.9),.8)
        self.assertIsNone(ndvi(0,0))
        with self.assertRaises(ValueError):ndvi(-1,1)
    def test_truth_regions(self):
        r=regions();self.assertEqual(len(r),16)
        for row in r:
            a,b,c,d=row['bounds'];self.assertEqual(sample_region((a+b)/2,(c+d)/2,r)['region_id'],row['region_id'])
            self.assertAlmostEqual(row['NDVI'],ndvi(row['red_value'],row['nir_value']))
        self.assertEqual(len(list(samples())),9)
    def test_distances_and_stale_data(self):
        m=SeparationMonitor(3)
        poses={f'UAV{i}':[1,i*4,0,0] for i in range(4)}
        self.assertEqual(len(m.update(poses,1)),6)
        self.assertEqual(m.update(poses,2),[])
        poses={'A':[3,0,0,0],'B':[3,1,0,0]}
        self.assertTrue(m.update(poses,3)[0]['warning']);m.update(poses,3)
        self.assertEqual(m.conflicts,1)
        poses['B']=[4,5,0,0];poses['A'][0]=4;m.update(poses,4)
        poses['B']=[5,1,0,0];poses['A'][0]=5;m.update(poses,5)
        self.assertEqual(m.conflicts,2)
if __name__=='__main__':unittest.main()
