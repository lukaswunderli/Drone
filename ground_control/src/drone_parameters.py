# -*- coding: utf-8 -*-
"""
Created on Mon Jun  1 12:11:08 2020

@author: lukas
"""

from drone_infrastructure import setup_drone


drone = setup_drone("SkyVipe","Drone")
drone.import_from_parameter_file("parameters.csv")

drone.close()
