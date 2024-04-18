import Otter_api
import lib.PID_Controller_test_v2 as PID_Controller_test_v2
import Otter_simulator
import lib.Live_guidance as Live_guidance
import lib.Live_plotter as Live_plotter
from lib.plotTimeSeries import *
import threading
import atexit
import time
from numba import jit, cuda


##########################################################################################################################################################
#                                                                      OPTIONS                                                                           #
##########################################################################################################################################################


N = 51000                                                                                               # Number of simulation samples
sampleTime = 0.02                                                                                       # Simulation time per sample. Usually at 0.02, other values could cause instabillity in the simulation
use_target_coordinates = False                                                                          # To use coordinates as a target or to use a linear path
use_moving_target = True                                                                                # To use moving target instead of target list (path following)
target_list = [[0, 10000]]                                                                           # List of targets to use if use_target_coordinates is set to True
end_when_last_target_reached = False                                                                    # Ends the simulation when the final target is reached
moving_target_start = [5, 0]                                                                        # Start point of the moving target if use_moving_target is set to True
moving_target_increase = [0.4, 0.8]                                                                    # Movement of the moving target each second
target_radius = 2                                                                                       # Radius from center of target that counts as target reached, change this depending on the complete size of the run. Very low values causes instabillity
verbose = True                                                                                          # Enable verbose printing
store_force_file = False                                                                                # Store the simulated control forces in a .csv file
circular_target = True                                                                                  # Make the moving target a circle in the simulation


# When connecting to live otter and using target tracking:
ip = "localhost"
port = 2009
start_north = 0                                                                                          # Target north position from referance point
start_east = 0                                                                                          # Target east position from referance point
v_north = 0                                                                                             # Moving target speed north (m/s)
v_east = 0                                                                                              # Moving target speed east (m/s)
radius = 20                                                                                                # If tracking a circular motion
v_circle = 2                                                                                            # Angular velocity (m/s)
enable_live_plot = True                                                                                  # Enables live plotting


parameter_list = 3                                     # Tuning parameters, 1 for trial and error, 2 for pole placement wb = 0.5, and 3 for pole placement wb = 0.4


trial_and_error_parameters = {"surge_kp" : 12, "surge_ki" : 0.7, "surge_kd" : 0, "yaw_kp" : 37, "yaw_ki" : 4, "yaw_kd" : 8}
pp_05 = {"surge_kp" : 22.48, "surge_ki" : 3.92, "surge_kd" : 11.62, "yaw_kp" : 23.72, "yaw_ki" : 4.13, "yaw_kd" : 15.08}
pp_04 = {"surge_kp" : 14.39, "surge_ki" : 3.13, "surge_kd" : 0, "yaw_kp" : 15.21, "yaw_ki" : 0.7, "yaw_kd" : 1.86}
test_pdi = {"surge_kp" : 14.39, "surge_ki" : 3.13, "surge_kd" : 0, "yaw_kp" : 15.21, "yaw_ki" : 0.7, "yaw_kd" : 0}

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



if parameter_list == 1:
    pdi = trial_and_error_parameters
elif parameter_list == 2:
    pdi = pp_05
elif parameter_list == 3:
    pdi = pp_04
elif parameter_list == 4:
    pdi = test_pdi



surge_kp = pdi["surge_kp"]                                                                                              #
surge_ki = pdi["surge_ki"]                                                                                              # Surge PID controller values
surge_kd = pdi["surge_kd"]                                                                                              #

yaw_kp = pdi["yaw_kp"]                                                                                                  #
yaw_ki = pdi["yaw_ki"]                                                                                                  # Yaw PID controller values
yaw_kd = pdi["yaw_kd"]                                                                                                  #



surge_PID = PID_Controller_test_v2.PIDController(surge_kp, surge_ki, surge_kd)                                  # Surge PID object
yaw_PID = PID_Controller_test_v2.PIDController(yaw_kp, yaw_ki, yaw_kd)                                          # Yaw PID object

live_guidance = Live_guidance.live_guidance(ip, port, surge_PID, yaw_PID, target_radius, otter)                 # Live guidance object




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

        #plotVehicleStates(simTime, simData, 1)                                                          #
        plotControls(simTime, simData, otter, 2)                                                        #
        plot3D(simData, numDataPoints, FPS, filename, 3)                                                #
        plotPosTar(simTime, simData, 4, targetData)                                                     # Plotting
        plotSpeed(simTime, simData, 5)                                                                  #
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
                atexit.register(exit_handler)
        elif option == 2:
            if enable_live_plot:
                _circle_thread.start()
                print(" Waiting for data")
                time.sleep(6)
                p1 = Live_plotter.live_plotter(otter)
                atexit.register(exit_handler)
            else:
                live_guidance.circular_tracking(start_north, start_east, radius, v_circle)
                atexit.register(exit_handler)
        else:
            print("Error")




if __name__ == "__main__":
    main(option)













