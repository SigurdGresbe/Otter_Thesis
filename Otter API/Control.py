import numpy as np
from numpy import round, pi
import math




class otter_control():

    def __init__(self):


        # This enables the printing of messages. Used for debugging. Slows down the software a bit.
        self.verbose = True

        # Scaling values for the engines
        self.scale_x_pos = 0.42
        self.scale_x_neg = 0.45
        self.scale_z_pos = 0.95
        self.scale_z_neg = 0.95

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


        # Propeller speed values
        self.max_rpm = 1108
        self.min_rpm = 73
        self.max_radS = (self.max_rpm * (2*pi) / 60)
        self.min_radS = (self.min_rpm * (2*pi) / 60)
        self.radS_spectrum = self.max_radS - self.min_radS
        # Assuming linear throttle
        self.radS_per_percent = self.radS_spectrum / 100



    # Sets the Otter in drift mode with zero trust
    def drift(self, otter_connector):
        if self.verbose:
            print("Otter entering drift mode")
        message_to_send = "$PMARABT"
        return otter_connector.send_message(message_to_send, False)


    # Sets the Otter in manual mode with specific forces and torques
    # The message must be repeated every 3 seconds or else the otter will enter drift mode.
    def set_manual_control_mode(self, force_x, force_y, torque_z, otter_connector):
        if self.verbose:
            print("Otter entering manual control mode with X force:", force_x, "Y force:", force_y, "Z torque:", torque_z)

        # Scaling for the values since 0 rpm is a field of values (not only x=0)
        if force_x > 0:
            force_x = (force_x * self.scale_x_pos) + 0.18
        elif force_x < 0:
            force_x = (force_x * self.scale_x_neg) - 0.15
        else:
            force_x = 0.0

        if torque_z > 0.05:
            torque_z = (torque_z * self.scale_z_pos) + 0.05
        elif torque_z < -0.05:
            torque_z = (torque_z * self.scale_z_neg) - 0.05
        else:
            torque_z = 0.0


        force_x = str(force_x)
        force_y = str(force_y)
        torque_z = str(torque_z)


        message_to_send = "$PMARMAN," + force_x + "," + force_y + "," + torque_z

        return otter_connector.send_message(message_to_send, True)


    # Sets the trusters manually
    # Calculates the force_x (surge) and torque_z (yaw) "backwards"

    def set_thrusters(self, a, b, otter_connector):

        if self.verbose:
            print("Setting Otter thrusters to", a, b)

        # Calculate linear force
        CG = np.array([0.2, 0, -0.2])
        l_y = 0.395
        l_x = 2.0/2 + CG[0]
        alpha_a = pi/2 - np.arctan2(l_x, l_y)
        alpha_b = pi/2 - np.arctan2(l_x, -l_y)
        c_x = 1/(np.cos(alpha_a)+np.cos(alpha_b))

        F_x = c_x * (a * np.cos(alpha_a) + b * np.cos(alpha_b))

        # Calculate rotational force
        beta_a = pi/2 - alpha_a
        beta_b = pi/2 - alpha_b
        c_t = 1/(np.cos(beta_a)-np.cos(beta_b))

        F_z = c_t * (a * np.cos(beta_a) + b * (np.cos(beta_b)))

        return self.set_manual_control_mode(F_x, 0.0, F_z, otter_connector)


    # Takes inputs tau_X and tau_Y (N) and returns the control speeds n1 and n2 (rad/s)
    def controlAllocation(self, tau_X, tau_N):
        tau = np.array([tau_X, tau_N])  # tau = B * u_alloc
        u_alloc = np.matmul(self.Binv, tau)  # u_alloc = inv(B) * tau

        # u_alloc = abs(n) * n --> n = sign(u_alloc) * sqrt(u_alloc)
        n1 = np.sign(u_alloc[0]) * math.sqrt(abs(u_alloc[0]))
        n2 = np.sign(u_alloc[1]) * math.sqrt(abs(u_alloc[1]))

        return n1, n2

    def radS_to_throttle(self, n1, n2):
        throttle_left = n1 / self.radS_per_percent
        throttle_right = n2 / self.radS_per_percent

        throttle_left = throttle_left / 100
        throttle_right = throttle_right / 100

        return throttle_left, throttle_right



    # Applies emergency brakes using reverse trusting until the speed of the Otter is below zero
    def EMERGENCY_BRAKES(self, otter_connector):
        print("APPLYING EMERGENCY BRAKES")

        self.set_thrusters(-1, -1)

        # Checks if the speed is above zero. If above zero then it will update the values until zero speed or lower is reached.
        while otter_connector.current_speed > 0:
            otter_connector.update_values()

        print("Otter stopped")
        print("Entering drift mode")

        # Enters drift mode when the speed is zero or lower.
        self.drift()