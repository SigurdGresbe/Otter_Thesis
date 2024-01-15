import numpy as np
import math


target = [-150, -150]



# Calculates the distance to the target
north_distance = target[0]
east_distance = target[1]
distance_to_target = math.sqrt(north_distance ** 2 + east_distance ** 2)

# Calculates the angle to the target in degrees. If this must be changed to radians, remember to change the controller aswell as this is set up for degrees!

yaw_setpoint = math.atan2(east_distance, north_distance)
yaw_setpoint = yaw_setpoint * (180 / math.pi)


print(yaw_setpoint)