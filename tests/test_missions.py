import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission_controller import items, M
from common import config
class MissionWireTests(unittest.TestCase):
    def test_px4_command_frames(self):
        for d in config('drones')['drones']:
            plan=items(d,True)
            self.assertEqual(plan[0].command,M.MAV_CMD_NAV_TAKEOFF)
            self.assertEqual(plan[-1].command,M.MAV_CMD_NAV_LAND)
            for seq,msg in enumerate(plan):
                self.assertEqual(msg.seq,seq)
                self.assertEqual(msg.target_system,d['system_id'])
                expected=M.MAV_FRAME_MISSION if msg.command==M.MAV_CMD_DO_CHANGE_SPEED else M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                self.assertEqual(msg.frame,expected)
            self.assertEqual((plan[0].x,plan[0].y),(plan[-1].x,plan[-1].y))
if __name__=='__main__':unittest.main()
