# -*- coding: utf-8 -*-
"""
Created on Mon Jun  1 18:50:27 2020

@author: lukas
"""


def setup_drone(hardware, program):
    from drone import Drone
    from backyard_flyer import BackyardFlyer
    from dronekit import connect
    
    if hardware=="SkyViper":
        connection_string = 'udp:0.0.0.0:14550'
    else:
        connection_string = ''
        connection_string = 'tcp:127.0.0.1:5760'
    sitl = None
    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()
    
    
    # Connect to the Vehicle
    print('Connecting to vehicle on: {0}'.format(connection_string))
    if program=="BackyardFlyer":
        drone = connect(connection_string, wait_ready=False, vehicle_class=BackyardFlyer)
    else:
        drone = connect(connection_string, wait_ready=False, vehicle_class=Drone)
    drone.wait_ready(True, raise_exception=False)
    print('Connected to vehicle on: {0}'.format(connection_string))

    return drone