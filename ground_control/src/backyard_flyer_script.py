
from drone_infrastructure import setup_drone


drone = setup_drone("SkyVipe","BackyardFlyer")
drone.start()
drone.close()