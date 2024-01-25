import Otter_api
import Controller_test
import Otter_simulator
from lib.plotTimeSeries import *


# Simulation options
N = 25000                  # Number of simulation samples
sampleTime = 0.02           # Simulation time

# Targeting
use_target_coordinates = False           # To use coordinates as a target or to use a linear path
use_moving_target = True               # To use moving target instead of target list (path following)

#target_list = [[-20, 0], [-30, 10], [-20, 20], [-10, 10], [-10, 60], [0, 70], [10, 60], [10, 10], [20, 20], [30, 10], [20, 0], [0, 0]] # Coordinates of target
target_list = [[100, 100], [200, -100], [300, 100], [400, -100], [-500, -100], [-500, 400], [0, 0]]
moving_target_start = [400, 400]

moving_target_increase = [-0.25, 0.15]       # Movement of the moving target each second


surge_target_radius = 0.1                     # Radius of the target or the distance from the target that counts as target reached
always_face_target = True              # Does the Otter have to face directly at the center of the target when inside the target radius (causes instabillity when reaching the target)
#TODO Lag noe bedre greier enn dette (always face target), funker sånn halvveis



# Creates Otter object
otter = Otter_api.otter()
simulator = Otter_simulator.otter_simulator(target_list, use_target_coordinates, surge_target_radius, always_face_target, use_moving_target, moving_target_start, moving_target_increase)


# Some values needed for the plotting
otter.controls = ["Left propeller shaft speed (rad/s)", "Right propeller shaft speed (rad/s)"]
otter.dimU = len(otter.controls)

# 3D plot and animation parameters where browser = {firefox,chrome,safari,etc.}
numDataPoints = 50                  # number of 3D data points
FPS = 10                            # frames per second (animated GIF)
filename = '3D_animation.gif'       # data file for animated GIF
browser = 'chrome'                  # browser for visualization of animated GIF



# Creating two controller objects for surge and yaw
#TODO Må tunes!!!
surge_kp = 50
surge_ki = 7
surge_kd = 10
yaw_kp = 12
yaw_ki = 5
yaw_kd = 5

surge_PID = Controller_test.PIDController(surge_kp, surge_ki, surge_kd)
yaw_PID = Controller_test.PIDController(yaw_kp, yaw_ki, yaw_kd)



# Introduction part
print("Welcome to the Otter controller simulator")

print("(1): Simulate Otter")
print("(2): Connect and use live Otter")

try:
    # Uncomment to choose between simulating and using a live connection

  #  option = float(input("Enter option: "))
    option = 1
    pass
except:
    print("You entered an invalid option so a simulation will be run!")
    option = 1


print(len(target_list))

# Main:
def main(option):

    if option == 1:
        [simTime, simData, targetData] = simulator.simulate(N, sampleTime, otter, surge_PID, yaw_PID)

        plotVehicleStates(simTime, simData, 1)
        plotControls(simTime, simData, otter, 2)
        plot3D(simData, numDataPoints, FPS, filename, 3)
        plotPosTar(simTime, simData, 4, targetData)
        # Saves a GIF for 3d animation in the same folder as main



        plt.show()
        plt.close()


if __name__ == "__main__":
    main(option)












