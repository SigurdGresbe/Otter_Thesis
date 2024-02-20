import Otter_api
import Controller_test_v2
import Otter_simulator
from lib.plotTimeSeries import *


##########################################################################################################################################################
#                                                                      OPTIONS                                                                           #
##########################################################################################################################################################


N = 25000                                                                                               # Number of simulation samples
sampleTime = 0.02                                                                                       # Simulation time per sample. Usually at 0.02, other values could cause instabillity in the simulation
use_target_coordinates = False                                                                          # To use coordinates as a target or to use a linear path
use_moving_target = True                                                                                # To use moving target instead of target list (path following)
target_list = [[100, 100], [200, -100], [300, 100], [400, -100]]                                        # List of targets to use if use_target_coordinates is set to True
end_when_last_target_reached = False                                                                    # Ends the simulation when the final target is reached
moving_target_start = [100, 100]                                                                          # Start point of the moving target if use_moving_target is set to True
moving_target_increase = [0, 0.3]                                                                      # Movement of the moving target each second
target_radius = 5                                                                                       # Radius from center of target that counts as target reached, change this depending on the complete size of the run. Very low values causes instabillity
verbose = True                                                                                          # Enable verbose printing
store_force_file = True                                                                                # Store the simulated control forces in a .csv file

#############################################################################################################################################################################################################################################################
#                                                                                                                                                                                                                                                           #
#                                                                                                                                                                                                                                                           #
#                                                                                                               MAIN CODE BELOW!                                                                                                                            #
#                                                                                                                                                                                                                                                           #
#                                                                                                                                                                                                                                                           #
#############################################################################################################################################################################################################################################################




otter = Otter_api.otter()                                                                                                                                                                                                          # Creates Otter object from the API
simulator = Otter_simulator.otter_simulator(target_list, use_target_coordinates, target_radius, use_moving_target, moving_target_start, moving_target_increase, end_when_last_target_reached, verbose, store_force_file)           # Creates Simulator object


otter.controls = ["Left propeller shaft speed (rad/s)", "Right propeller shaft speed (rad/s)"]          # Some values needed for the plotting
otter.dimU = len(otter.controls)                                                                        #

numDataPoints = 50                                                                                      # number of 3D data points
FPS = 10                                                                                                # frames per second (animated GIF)
filename = '3D_animation.gif'                                                                           # data file for animated GIF
browser = 'chrome'                                                                                      # browser for visualization of animated GIF




surge_kp = 86                                                                                            #
surge_ki = 8.6                                                                                           # Surge PID controller values
surge_kd = 70                                                                                            #

yaw_kp = 90                                                                                              #
yaw_ki = 9                                                                                               # Yaw PID controller values
yaw_kd = 55                                                                                              #



surge_PID = Controller_test_v2.PIDController(surge_kp, surge_ki, surge_kd)                              # Surge PID object
yaw_PID = Controller_test_v2.PIDController(yaw_kp, yaw_ki, yaw_kd)                                      # Yaw PID object



print("Welcome to the Otter controller simulator")                                                      #
print("(1): Simulate Otter")                                                                            # Main introduction part
print("(2): Connect and use live Otter")                                                                #

try:
    # Uncomment to choose between simulating and using a live connection

  #  option = float(input("Enter option: "))
    option = 1
    pass
except:
    print("You entered an invalid option so a simulation will be run!")
    option = 1




# Main:
def main(option):

    if option == 1:
        [simTime, simData, targetData] = simulator.simulate(N, sampleTime, otter, surge_PID, yaw_PID)   # This runs the whole simulation

        plotVehicleStates(simTime, simData, 1)                                                          #
        plotControls(simTime, simData, otter, 2)                                                        #
        plot3D(simData, numDataPoints, FPS, filename, 3)                                                #
        plotPosTar(simTime, simData, 4, targetData)                                                     # Plotting......
        # Saves a GIF for 3d animation in the same folder as main



        plt.show()
        plt.close()


if __name__ == "__main__":
    main(option)












