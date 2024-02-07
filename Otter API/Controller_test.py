import Otter_api
import time
import numpy as np
import math


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        self.previous_time = time.time()


        self.TauX_max = 250           # Max forward thrust
        self.TauX_min = 0.1          # Min forward thrust
        self.Decel_distance = 10   # Distance to start deceleration

        self.negative_thrust_distance = 10  # Distance from target when negative thrust is acceptable


    def calculate_surge(self, surge_radius, distance_to_target, yaw_setpoint, yaw_measured):
        current_time = time.time()
        sample_time = current_time - self.previous_time
        self.previous_time = current_time

        error = surge_radius - distance_to_target
        if distance_to_target < surge_radius:
            error = 0
            self.integral = 0


        self.integral += error * sample_time

        # Caps the integral
        self.integral = max(min(self.integral, 25), -25)

        derivative = (error - self.previous_error) / sample_time if sample_time > 0 else 0
        self.previous_error = error

        # Scalar that scales the output down if the yaw error is big
        yaw_error = (yaw_setpoint - yaw_measured + math.pi) % (2 * math.pi) - math.pi
        scalar = 1 - abs(yaw_error) / math.pi


        # Calculate the PID output
        #surge_output = -(self.kp * error + self.ki * self.integral + self.kd * derivative) * scalar
        surge_output = -(self.kp * error + self.ki * self.integral + self.kd * derivative)

        if (yaw_error > (math.pi/2) or yaw_error < -(math.pi/2)) and distance_to_target < self.negative_thrust_distance:    # Allows the thrusters to go in reverse if the target is passed
            surge_output = -(surge_output)


        return surge_output


    def calculate_yaw(self, setpoint, measured_value, surge_setpoint, distance_to_target, always_face_target):
        current_time = time.time()
        sample_time = current_time - self.previous_time
        self.previous_time = current_time

        #error = (setpoint - measured_value + 180) % 360 - 180
        error = (setpoint - measured_value + math.pi) % (2 * math.pi) - math.pi
        if not always_face_target and (distance_to_target - surge_setpoint) < 0:
            error = 0
            self.integral = 0


        self.integral += error * sample_time

        # Caps the integral
        self.integral = max(min(self.integral, 25), -25)

        derivative = (error - self.previous_error) / sample_time if sample_time > 0 else 0
        self.previous_error = error


        yaw_output = (self.kp * error + self.ki * self.integral + self.kd * derivative)
        return yaw_output





if __name__ == "__main__":

    otter = Otter_api.otter()

    pid_distance = PIDController(kp=1, ki=0, kd=0)
    pid_heading = PIDController(kp=1, ki=0, kd=0)

    # Test waypoint
    waypoint = [100, 200]


    i = 0

    current_heading = float(input("Current start heading: "))
    distance_to_waypoint = float(input("Start distance to waypoint: "))
    desired_heading = float(input("Desired heading: "))

    ls = [[], []]


    while i < 100:
        i += 1

        surge_force = pid_distance.calculate_surge(0, distance_to_waypoint)
        yaw_force = pid_heading.calculate_yaw(desired_heading, current_heading)


        otter.controller_inputs_torque(surge_force, yaw_force)


        ls[0].append(otter.otter_control.F_x)
        ls[1].append(otter.otter_control.F_z)

        time.sleep(0)

    # Dette er for debuging for å se verdier
    ls = np.array(ls)
    print("ok")
