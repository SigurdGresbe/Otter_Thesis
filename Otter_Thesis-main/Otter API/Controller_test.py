import Otter_api
import time
import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        self.outputA = 0
        self.previous_time = time.time()

    # Not working as intended... yet!... Should make faster or slower responsiveness for sharper or smoother curves 
    def lowpass_filter(self, dt, inputA):
        self.outputA
        f_c = 0.02 # larger value = faster response, lower for smoother curve
        RC = 1 / (2 * np.pi * f_c)
        alpha = dt / (RC + dt)
    
        self.outputA = alpha * inputA + (1 - alpha) * self.outputA

        return self.outputA
        
    def calculate_surge(self, setpoint, measured_value):
        current_time = time.time()
        sample_time = current_time - self.previous_time
        self.previous_time = current_time

        error = setpoint - measured_value

        self.integral += error * sample_time

        # Caps the integral
        if self.integral > 25:
            self.integral = 25
        if self.integral < -25:
            self.integral = -25



        
        raw_derivative = (error - self.previous_error) / sample_time if sample_time > 0 else 0
        derivative = self.lowpass_filter(sample_time, raw_derivative)
        
        self.previous_error = error


        surge_output = -(self.kp * error + self.ki * self.integral + self.kd * derivative)
        return surge_output

    def calculate_yaw(self, setpoint, measured_value):
        current_time = time.time()
        sample_time = current_time - self.previous_time
        self.previous_time = current_time

        error = (setpoint - measured_value + 180) % 360 - 180
        self.integral += error * sample_time

        # Caps the integral
        if self.integral > 25:
            self.integral = 25
        if self.integral < -25:
            self.integral = -25
        
        raw_derivative = -(error - self.previous_error)
        derivative = self.lowpass_filter(sample_time,raw_derivative)


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

        # TODO legg til dynamikken så man kan simulere dette realistisk?


        ls[0].append(otter.otter_control.F_x)
        ls[1].append(otter.otter_control.F_z)

        time.sleep(0)

    # Dette er for debuging for å se verdier
    ls = np.array(ls)
    print("ok")
