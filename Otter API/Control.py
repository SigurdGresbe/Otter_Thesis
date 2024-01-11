import numpy as np
from numpy import round, pi




class otter_control():

    def __init__(self):


        # This enables the printing of messages. Used for debugging. Slows down the software a bit.
        self.verbose = True



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
        force_x = str(np.round(force_x, 2))
        force_y = str(np.round(force_y, 2))
        torque_z = str(np.round(torque_z, 2))


        message_to_send = "$PMARMAN," + force_x + "," + force_y + "," + torque_z

        return otter_connector.send_message(message_to_send, True)


    # Sets the trusters manually
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