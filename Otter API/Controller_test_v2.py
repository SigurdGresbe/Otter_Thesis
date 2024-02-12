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
        self.previous_time = None


        self.integrator_limits = [0, 5]     # Limits the integrator in both regulators

        self.previous_distance = None


    def calculate_surge(self, surge_radius, distance_to_target, yaw_setpoint, yaw_measured):
        current_time = time.time()
        sample_time = current_time - self.previous_time if self.previous_time else 0
        self.previous_time = current_time


        error = distance_to_target - surge_radius
        yaw_error = (yaw_setpoint - yaw_measured + math.pi) % (2 * math.pi) - math.pi


        if (yaw_error > (math.pi/2) or yaw_error < -(math.pi/2)):       # Allows the thrusters to go in reverse if the target is passed
            error = -(error)

        self.integral += error * sample_time if sample_time > 0 else 0
        self.integral = max(min(self.integral, self.integrator_limits[1]), self.integrator_limits[0])

        if distance_to_target - surge_radius < 0:
            self.integral = 0
            error = 0

        derivative = (error - self.previous_error) / sample_time if sample_time > 0 else 0
        self.previous_error = error

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)


        return output


    def calculate_yaw(self, setpoint, measured_value, surge_radius, distance_to_target):
        current_time = time.time()
        sample_time = current_time - self.previous_time if self.previous_time else 0
        self.previous_time = current_time


        error = (setpoint - measured_value + math.pi) % (2 * math.pi) - math.pi

        if ((distance_to_target - (surge_radius + 2)) < 0):
            self.integral = 0                                           # Resets the integral when the target is reached. Probably is a better method than this.
            error = 0

        self.integral += error * sample_time if sample_time > 0 else 0
        self.integral = max(min(self.integral, self.integrator_limits[1]), self.integrator_limits[0])
        derivative = (error - self.previous_error) / sample_time if sample_time > 0 else 0
        self.previous_error = error


        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)


        return output





