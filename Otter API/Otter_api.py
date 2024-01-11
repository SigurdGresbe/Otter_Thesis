import Control
import Connector
import time
import pymap3d as pm
import math
import numpy as np

#
#   This is the complete API for the Otter. This can be imported in custom programs and then the functions can be called to communicate with the otter.
#

class otter():

    def __init__(self):

        # Creates instances of the connector and control classes
        self.otter_connector = Connector.otter_connector()
        self.otter_control = Control.otter_control()


        # The observer coordinates for the geodetic to ned conversion. This can be changed manually
        self.observer_coordinates = [59.908642666666665, 10.71945885, 0.0]


        # Creates an empty dictionary for the values
        self.values = {}
        self.sorted_values = {}


        # Variables and lists
        self.geo2ned_from_observer = [0.0, 0.0, 0.0]


        # Data for one pontoon
        self.B_pont = 0.25  # Beam of one pontoon (m)
        y_pont = 0.395      # Distance from centerline to waterline centroid (m)


        # Propeller configuration/input matrix
        self.l1 = -y_pont  # Lever arm, left propeller (m)
        self.l2 = y_pont  # Lever arm, right propeller (m)
        self.k_pos = 0.02216 / 2  # Positive Bollard, one propeller
        self.k_neg = 0.01289 / 2  # Negative Bollard, one propeller
        B = self.k_pos * np.array([[1, 1], [-self.l1, -self.l2]])
        self.Binv = np.linalg.inv(B)


    # Tries to establish connection to the otter. Default values are in place for testing on a local machine with a test server. Returns boolean
    def establish_connection(self, ip="localhost", port=2009):
        return self.otter_connector.establish_connection(ip, port)


    # Tries to close the socket connection to the Otter. Returns boolean
    def close_connection(self):
        return self.otter_connector.close_connection()


    # Checks the current connection status to the Otter. Returns boolean
    def check_connection(self):
        return self.otter_connector.check_connection()


    # Reads a message from the Otter and returns it and stores it in "last_message_recieved". Returns the last message recieved.
    def read_message(self):
        return self.otter_connector.read_message()


    # Sends a message to the Otter, with the option of adding checksum (defaults to False). Returns boolean
    def send_message(self, message, checksum_needed = False):
        return self.otter_connector.send_message(message, checksum_needed)


    # Tries to update all the values in the dictionary "values" with the current values from the Otter. Requires connection established. Returns updated dictionary "values"
    def update_values(self):
        self.otter_connector.update_values(timeout = 0.1)
        self.values["current_position"] = self.otter_connector.current_position
        self.values["previous_position"] = self.otter_connector.previous_position
        self.values["last_speed_update"] = self.otter_connector.last_speed_update
        self.values["current_course_over_ground"] = self.otter_connector.current_course_over_ground
        self.values["current_speed"] = self.otter_connector.current_speed
        self.values["current_fuel_capacity"] = self.otter_connector.current_fuel_capacity
        self.values["current_orientation"] = self.otter_connector.current_orientation
        self.values["current_rotational_velocities"] = self.otter_connector.current_rotational_velocities
        self.values["observer_coordinates"] = self.observer_coordinates
        self.values["geo2ned_from_observer"] = self.geo2ned_from_observer
        #self.values["yaw"] = self.otter_connector.yaw


        # Sorts all the data into individuall int and float values in a dictionary sorted_values
        self.sorted_values["lat"] = self.values["current_position"][0]
        self.sorted_values["lon"] = self.values["current_position"][1]
        self.sorted_values["height"] = self.values["current_position"][2]
        self.sorted_values["previous_lat"] = self.values["previous_position"][0]
        self.sorted_values["previous_lon"] = self.values["previous_position"][1]
        self.sorted_values["previous_height"] = self.values["previous_position"][2]
        self.sorted_values["last_speed_update"] = self.values["last_speed_update"]
        self.sorted_values["current_course_over_ground"] = self.values["current_course_over_ground"]
        self.sorted_values["current_speed"] = self.values["current_speed"]
        self.sorted_values["current_fuel_capacity"] = self.values["current_fuel_capacity"]
        self.sorted_values["current_orientation_1"] = self.values["current_orientation"][0]
        self.sorted_values["current_orientation_2"] = self.values["current_orientation"][1]
        self.sorted_values["current_orientation_3"] = self.values["current_orientation"][2]
        self.sorted_values["current_rotational_velocities_1"] = self.values["current_rotational_velocities"][0]
        self.sorted_values["current_rotational_velocities_2"] = self.values["current_rotational_velocities"][1]
        self.sorted_values["current_rotational_velocities_3"] = self.values["current_rotational_velocities"][2]
        self.sorted_values["observer_lat"] = self.values["observer_coordinates"][0]
        self.sorted_values["observer_lon"] = self.values["observer_coordinates"][1]
        self.sorted_values["observer_height"] = self.values["observer_coordinates"][2]

        self.geo2ned_position()

        self.sorted_values["north_from_observer"] = self.values["geo2ned_from_observer"][0]
        self.sorted_values["east_from_observer"] = self.values["geo2ned_from_observer"][1]
        self.sorted_values["down_from_observer"] = self.values["geo2ned_from_observer"][2]

        return self.values

    # Takes the otter coordinates and converts it to north east down observed from the observer coordinates
    def geo2ned_position(self):
        n, e, d = pm.geodetic2ned(self.sorted_values["lat"], self.sorted_values["lon"], self.sorted_values["height"], self.sorted_values["observer_lat"], self.sorted_values["observer_lon"], self.sorted_values["observer_height"])
        self.geo2ned_from_observer = [n, e, d]

    # Tries to set the Otter in manual control mode, controlling the x, y and torques.
    def set_manual_control_mode(self, force_x, force_y, torque_z):
        if self.check_connection():
            return self.otter_control.set_manual_control_mode(force_x, force_y, torque_z, self.otter_connector)

        else:
            "No connection to Otter"
            return False


    # Tries to make the Otter enter drift mode. Returns boolean
    def drift(self):
        if self.check_connection():
            return self.otter_control.drift(self.otter_connector)

        else:
            "No connection to Otter"
            return False


    # Takes inputs tau_x and tau_y and returns the control forces n1 and n2
    def controlAllocation(self, tau_X, tau_N):
        """
        [n1, n2] = controlAllocation(tau_X, tau_N)
        """
        tau = np.array([tau_X, tau_N])  # tau = B * u_alloc
        u_alloc = np.matmul(self.Binv, tau)  # u_alloc = inv(B) * tau

        # u_alloc = abs(n) * n --> n = sign(u_alloc) * sqrt(u_alloc)
        n1 = np.sign(u_alloc[0]) * math.sqrt(abs(u_alloc[0]))
        n2 = np.sign(u_alloc[1]) * math.sqrt(abs(u_alloc[1]))

        return n1, n2


    # Tries to set the trusters manually. a and b are individual thrusters and their values range from -1 to 1
    def set_thrusters(self, a, b):
        return self.otter_control.set_thrusters(a, b, self.otter_connector)







# This runs if this script is run by itself and not imported into another program. Make sure to have a connection available to the Otter or the test server running.
if __name__ == "__main__":


    otter = otter()


    # Establishes a socket connection to the Otter with IP and the PORT'
#    otter.establish_connection("10.0.5.1", 2009)


    # Write test commands under here:

#    otter.set_manual_control_mode(0.2, 0, 0.0)
#    otter.update_values()

#    time.sleep(5)
#    otter.close_connection()





