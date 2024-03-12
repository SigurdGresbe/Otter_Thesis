import Otter_api
import Controller_test_v2
import Otter_simulator
import Live_guidance
import Live_plotter
from lib.plotTimeSeries import *
import threading
import atexit
import time



##########################################################################################################################################################
#                                                                      OPTIONS                                                                           #
##########################################################################################################################################################


N = 15000                                                                                               # Number of simulation samples
sampleTime = 0.02                                                                                       # Simulation time per sample. Usually at 0.02, other values could cause instabillity in the simulation
use_target_coordinates = False                                                                          # To use coordinates as a target or to use a linear path
use_moving_target = True                                                                                # To use moving target instead of target list (path following)
target_list = [[100, 100], [200, -100], [300, 100], [400, -100]]                                        # List of targets to use if use_target_coordinates is set to True
end_when_last_target_reached = False                                                                    # Ends the simulation when the final target is reached
moving_target_start = [10, 0]                                                                        # Start point of the moving target if use_moving_target is set to True
moving_target_increase = [-1.0, 0.0]                                                                    # Movement of the moving target each second
target_radius = 1                                                                                       # Radius from center of target that counts as target reached, change this depending on the complete size of the run. Very low values causes instabillity
verbose = True                                                                                          # Enable verbose printing
store_force_file = False                                                                                # Store the simulated control forces in a .csv file
circular_target = False                                                                                  # Make the moving target a circle
enable_live_plot = True                                                                                  # Enables live plotting

# When connecting to live otter and using target tracking
ip = "localhost"
port = 2009
start_north = 0                                                                                          # Target north position from referance point
start_east = 0                                                                                           # Target east position from referance point
v_north = 2.0                                                                                             # Moving target speed north (m/s)
v_east = 0.0                                                                                              # Moving target speed east (m/s)
radius = 8                                                                                                # If tracking a circular motion
v_circle = 1.2                                                                                            # Angular velocity (m/s)



#############################################################################################################################################################################################################################################################
#                                                                                                                                                                                                                                                           #
#                                                                                                                                                                                                                                                           #
#                                                                                                               MAIN CODE BELOW!                                                                                                                            #
#                                                                                                                                                                                                                                                           #
#                                                                                                                                                                                                                                                           #
#############################################################################################################################################################################################################################################################




otter = Otter_api.otter()                                                                                                                                                                                                          # Creates Otter object from the API
simulator = Otter_simulator.otter_simulator(target_list, use_target_coordinates, target_radius, use_moving_target, moving_target_start, moving_target_increase, end_when_last_target_reached, verbose, store_force_file, circular_target)           # Creates Simulator object



otter.controls = ["Left propeller shaft speed (rad/s)", "Right propeller shaft speed (rad/s)"]          # Some values needed for the plotting
otter.dimU = len(otter.controls)                                                                        #

numDataPoints = 50                                                                                      # number of 3D data points
FPS = 10                                                                                                # frames per second (animated GIF)
filename = '3D_animation.gif'                                                                           # data file for animated GIF
browser = 'chrome'                                                                                      # browser for visualization of animated GIF




surge_kp = 2.86                                                                                         #
surge_ki = 0.25                                                                                         # Surge PID controller values
surge_kd = 0                                                                                            #

yaw_kp = 6                                                                                              #
yaw_ki = 0.6                                                                                            # Yaw PID controller values
yaw_kd = 0                                                                                              #



surge_PID = Controller_test_v2.PIDController(surge_kp, surge_ki, surge_kd)                              # Surge PID object
yaw_PID = Controller_test_v2.PIDController(yaw_kp, yaw_ki, yaw_kd)                                      # Yaw PID object

live_guidance = Live_guidance.live_guidance(ip, port, surge_PID, yaw_PID, target_radius, otter)                # Live guidance object




print("Welcome to the Otter controller simulator")                                                      #
print("(1): Simulate Otter")                                                                            # Main introduction part
print("(2): Connect and use live Otter")                                                                #

try:
    option = float(input("Enter option: "))
    pass
except:
    print("You entered an invalid option so a simulation will be run!")
    option = 1


def _target_tracking():
    live_guidance.target_tracking(start_north, start_east, v_north, v_east)

def _circular_tracking():
    live_guidance.circular_tracking(start_north, start_east, radius, v_circle)

def exit_handler():
    live_guidance.save_log()



# Main:
def main(option):

    if option == 1:
        [simTime, simData, targetData] = simulator.simulate(N, sampleTime, otter, surge_PID, yaw_PID)   # This runs the whole simulation

        plotVehicleStates(simTime, simData, 1)                                                          #
        plotControls(simTime, simData, otter, 2)                                                        #
        plot3D(simData, numDataPoints, FPS, filename, 3)                                                #
        plotPosTar(simTime, simData, 4, targetData)                                                     # Plotting
        # Saves a GIF for 3d animation in the same folder as main



        plt.show()
        plt.close()

    elif option == 2:

        _target_thread = threading.Thread(target=_target_tracking, args=())
        _target_thread.daemon = True
        _circle_thread = threading.Thread(target=_circular_tracking, args=())
        _circle_thread.daemon = True


        option = float(input("Enter 1 for target tracking with moving target or 2 for circular motion: "))


        if option == 1:

            if enable_live_plot:
                _target_thread.start()
                print(" Waiting for data")
                time.sleep(6)
                p1 = Live_plotter.live_plotter(otter)
                atexit.register(exit_handler)
            else:
                live_guidance.target_tracking(start_north, start_east, v_north, v_east)

        elif option == 2:
            if enable_live_plot:
                _circle_thread.start()
                print(" Waiting for data")
                time.sleep(6)
                p1 = Live_plotter.live_plotter(otter)
                atexit.register(exit_handler)
            else:
                live_guidance.circular_tracking(start_north, start_east, radius, v_circle)

        else:
            print("Error")




if __name__ == "__main__":
    main(option)













