# -*- coding: utf-8 -*-
"""
Created on Mon Jun  1 14:39:06 2020

@author: lukas
"""


import unittest
import sys
sys.path.append('../src')

from backyard_flyer import BackyardFlyer
from dronekit import connect

class TestBackyardFlyer(unittest.TestCase):

    def test_sequence(self):
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()
        drone = connect(connection_string, wait_ready=False, vehicle_class=BackyardFlyer)
        drone.wait_ready(True, raise_exception=False)
        drone.arming_transition()
        drone.local_position_callback('','','')
        drone.set_box_waypoints()
        drone.waypoint_transition()
        drone.local_position_callback('','','')
        drone.landing_transition()
        drone.local_position_callback('','','')
        drone.disarming_transition()
        drone.local_position_callback('','','')
        drone.manual_transition()
        drone.local_position_callback('','','')
        
    def test_LocationGlobalRelative(self):
        from dronekit import LocationLocal
        loc = LocationLocal(0.0,0.0,0.0)
        print(type(loc.down))

if __name__ == '__main__':
    unittest.main()