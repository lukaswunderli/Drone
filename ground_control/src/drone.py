# -*- coding: utf-8 -*-
"""
Created on Mon Jun  1 12:11:08 2020

@author: lukas
"""

from enum import Enum
import numpy as np
import pandas as pd

from dronekit import (Vehicle,VehicleMode)
from pymavlink import mavutil

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5
    FLIP = 6

class Drone(Vehicle):
    def __init__(self, *args):
        self.in_mission = True
        super(Drone, self).__init__(*args)
    
    @property
    def local_position(self):
        return self.location.local_frame
    
    @property
    def global_position(self):
        return self.location.global_frame
    
    def land(self):
        self.mode = VehicleMode("LAND")
        
    def set_home_position(self, global_position):
        self.home_location = global_position
        
    def set_heading(self, heading, relative=False):
        if relative:
            is_relative=1 #yaw relative to direction of travel
        else:
            is_relative=0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.send_mavlink(msg)

    def arming_transition(self):
        print("arming transition")
        self.armed = True
        self.set_home_position(self.global_position)  # set the current location to be the home position
        self.flight_state = States.ARMING

    def takeoff_transition(self,target_altitude):
        print("takeoff transition")
        self.target_position[2] = target_altitude
        self.simple_takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

    def manual_transition(self):
        print("manual transition")
        self.in_mission = False
        self.flight_state = States.MANUAL

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.armed = False
        #self.release_control()
        self.flight_state = States.DISARMING
    
    def print_parameter(self, name):
        print("Param[{0}]: {1}".format(name, self.parameters[name]))
        
    def set_parameter(self, name, value):
        self.parameters[name] = value
        self.print_parameter(name)
    
    def import_from_parameter_file(self,filename):
        df = pd.read_csv("parameters.csv")
        for index, row in df.iterrows():
            self.set_parameter(row['Parameter'],row['Value'])
            
            
            
            
            
