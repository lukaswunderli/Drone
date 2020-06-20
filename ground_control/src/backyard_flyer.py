# -*- coding: utf-8 -*-
"""
Created on Mon Jun  1 12:11:08 2020

@author: lukas
"""

from enum import Enum
import numpy as np

from drone import Drone
from dronekit import LocationGlobalRelative
from planning_utils import (local_to_global,LocationGlobal_to_LonLatAlt,LocationLocal_to_NorthEastUp)

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

class BackyardFlyer(Drone):
    def __init__(self, *args):
        super(BackyardFlyer, self).__init__(*args)
        
        self.target_position = np.array([0.0, 0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL
        #self.add_message_listener('LOCAL_POSITION_NED',self.local_position_callback)
        #self.add_message_listener('GLOBAL_POSITION_INT',self.local_position_callback)
        
    def __del__(self):
        self.close()
        
    def local_position_callback(self, dummy, name, message):
        print('message: {0}'.format(message))
        print(self.mode, ' ', self.flight_state)
        if self.flight_state == States.MANUAL:
            #if not self.is_armable:
            #    print(" Waiting for vehicle to initialise...")
            #else:
                self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position.down > 0.95 * self.target_position[2]:
                self.set_box_waypoints()
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            local_p = np.array(LocationLocal_to_NorthEastUp(self.local_position))
            dist = np.linalg.norm(self.target_position[0:2] - local_p[0:2])
            print("distance: {0}".format(dist))
            if dist < 1.0:
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()
        elif self.flight_state == States.LANDING:
            if self.global_position.alt - self.global_home.alt < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()
        elif self.flight_state == States.DISARMING:
            if ~self.armed & ~self.guided:
                self.manual_transition()
    
    def set_box_waypoints(self):
        print("Setting Home")
        squaresize = 3.0
        altitude = 1.0
        local_waypoints = [
            [squaresize, 0.0, altitude, 0],
            [squaresize, squaresize, altitude, 90],
            [0.0, squaresize, altitude, 180],
            [0.0, 0.0, altitude, 270]]
        self.all_waypoints = local_waypoints

    def arming_transition(self):
        print("arming transition")
        self.arm()
        self.set_home_position(self.global_position)  # set the current location to be the home position
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.simple_takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        self.target_position = self.all_waypoints.pop(0)
        print('target position', self.target_position)
        self.set_heading(self.target_position[3])
        global_home = LocationGlobal_to_LonLatAlt(self.home_location)
        local = [self.target_position[0],self.target_position[1],self.target_position[2]]
        target = local_to_global(local,global_home);
        globalLoc = LocationGlobalRelative(target[0],target[1],target[2])
        print(globalLoc)
        self.simple_goto(globalLoc)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.release_control()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        print("starting connection")
        self.add_message_listener('GLOBAL_POSITION_INT',self.local_position_callback)
        self.flight_state = States.MANUAL
        
    #def stop(self):
    #    print("starting connection")
    #    #Create a message listener for all messages.
    #    self.remove_message_listener('LOCAL_POSITION_NED',self.local_position_callback)