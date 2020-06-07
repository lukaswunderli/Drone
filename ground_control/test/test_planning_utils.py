import unittest
import math
from planning_utils import midpoint_heading_between_3_waypoints

class TestPlanningUtils(unittest.TestCase):

    def test_midpoint_heading_between_3_waypoints_1(self):
        wp = [[2,0],[0,0],[0,2]]
        angle = midpoint_heading_between_3_waypoints(wp)
        self.assertEqual(angle, math.pi*3/4)
    def test_midpoint_heading_between_3_waypoints_2(self):
        wp = [[-1,1],[1,1],[1,0]]
        angle = midpoint_heading_between_3_waypoints(wp)
        self.assertEqual(angle, -math.pi/4)

if __name__ == '__main__':
    unittest.main()