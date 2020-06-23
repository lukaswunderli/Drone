# -*- coding: utf-8 -*-
"""
Created on Mon Jun  1 12:11:08 2020

@author: lukas
"""

from enum import Enum
import numpy as np

from drone import Drone, States
from dronekit import LocationGlobalRelative
from planning_utils import (local_to_global,LocationGlobal_to_LonLatAlt,LocationLocal_to_NorthEastUp)


class Mission(Enum):
    NONE = 0
    CHECK_ARM = 1
    CHECK_TAKEOFF = 2
    WAYPOINT = 3
    FLIP = 4

class BackyardFlyer(Drone):
    def __init__(self, *args):
        super(BackyardFlyer, self).__init__(*args)
        
        self.target_position = np.array([0.0, 0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.check_state = {}

        # initial states
        self.flight_state = States.MANUAL
        self.flight_mission = Mission.CHECK_ARM
        #self.add_message_listener('LOCAL_POSITION_NED',self.local_position_callback)
        #self.add_message_listener('GLOBAL_POSITION_INT',self.local_position_callback)
        
    def __del__(self):
        self.close()
        
    def local_position_callback(self, dummy, name, message):
        print('message: {0}'.format(message))
        print(self.mode, ' ', self.flight_state)
        if self.flight_state == States.MANUAL && self.flight_mission != Mission.NONE:
            if not self.is_armable:
                print(" Waiting for vehicle to initialise...")
            else:
                self.mode = VehicleMode("GUIDED")
                self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.armed:
                if self.flight_mission == Mission.CHECK_ARM:
                    self.flight_mission = Mission.NONE
                    self.disarming_transition()
                elif self.flight_mission == Mission.WAYPOINT:
                    target_altitude = 3.0
                else:
                    target_altitude = 0.5
                self.takeoff_transition(target_altitude)
        elif self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position.down > 0.95 * self.target_position[2]:
                if self.flight_mission == Mission.WAYPOINT:
                    self.set_box_waypoints()
                    self.waypoint_transition()
                elif self.flight_mission == Mission.FLIP:
                    self.flip_transition()
                else
                    self.landing_transition()
                    
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
        elif self.flight_state == States.FLIP:
            if self.mode == VehicleMode("ACRO")
                if self.flight_mission = Mission.FLIP
                    self.mode = VehicleMode("FLIP")
                    self.flight_mission = Mission.NONE
                else self.mode == VehicleMode("ACRO"):
                    self.landing_transistion();
        elif self.flight_state == States.LANDING:
            if self.global_position.alt - self.global_home.alt < 0.1:
                if abs(self.local_position[2]) < 0.1:
                    self.flight_mission = Mission.NONE
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

    def flip_transition(self):
        print("flip transition")
        self.mode = VehicleMode("ACRO")
        self.flight_state = States.FLIP

    def start(self):
        print("starting connection")
        self.add_message_listener('GLOBAL_POSITION_INT',self.local_position_callback)
        self.flight_state = States.MANUAL
    
    def startArmCheck(self):
        self.flight_mission = Mission.CHECK_ARM
    def startTakeoffCheck(self):
        self.flight_mission = Mission.CHECK_TAKEOFF
    def startWaypointMission(self):
        self.flight_mission = Mission.WAYPOINT
    def startFlipMission(self):
        self.flight_mission = Mission.FLIP
        
    #def stop(self):
    #    print("starting connection")
    #    #Create a message listener for all messages.
    #    self.remove_message_listener('LOCAL_POSITION_NED',self.local_position_callback)
