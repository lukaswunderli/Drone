
from drone_infrastructure import setup_drone
import time

drone = setup_drone("SkyViper","BackyardFlyer")
drone.arm()
time.sleep(1)
drone.disarm()
#drone.start()
#drone.close()
