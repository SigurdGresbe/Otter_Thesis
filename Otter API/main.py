import Otter_api
import Controller_test
import Otter_simulator
from lib.plotTimeSeries import *


# Simulation options
N = 100000                  # Number of simulation samples
sampleTime = 0.02           # Simulation time

use_target_coordinates = True           # To use coordinates as a target or to use a linear path

target_list = [[100, 100], [200, -100], [300, 100], [400, -100], [-500, -500], [-500, 500], [0, 0]]          # Coordinates of target

surge_target_radius = 5                      # Radius of the target or the distance from the target that counts as target reached
always_face_target = False              # Does the Otter have to face directly at the center of the target when inside the target radius (causes instabillity when reaching the target)
#TODO Lag noe bedre greier enn dette (always face target), funker sånn halvveis



# Creates Otter object
otter = Otter_api.otter()
simulator = Otter_simulator.otter_simulator(target_list, use_target_coordinates, surge_target_radius, always_face_target)


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
surge_kp = 8
surge_ki = 2
surge_kd = 5
yaw_kp = 1
yaw_ki = 0
yaw_kd = 0

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
        [simTime, simData] = simulator.simulate(N, sampleTime, otter, surge_PID, yaw_PID)

        plotVehicleStates(simTime, simData, 1)
        plotControls(simTime, simData, otter, 2)
        plot3D(simData, numDataPoints, FPS, filename, 3)

        """ Ucomment the line below for 3D animation in the web browswer. 
        Alternatively, open the animated GIF file manually in your preferred browser. """
        # webbrowser.get(browser).open_new_tab('file://' + os.path.abspath(filename))

        plt.show()
        plt.close()


if __name__ == "__main__":
    main(option)












