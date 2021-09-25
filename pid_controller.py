# from collections import namedtuple
# from math import exp
from matplotlib import pyplot as plt
import numpy as np
import time
from simulator import Simulator, motor

# def method():

#     return 4000 + np.random.randn() * 1000
#     # return 4000 + int(str(time.time())[-3:])


class Pid(object):

    def __init__(self, desired_value, kp, ki, kd, sim:Simulator) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.desired_value = desired_value
        self.current_error = desired_value
        self.sum_error = 0
        self.previous_error = desired_value
        self.sample_time = 0.3
        self.sim = sim
        self.u_t = 0

    def read_current_value(self):
        return self.sim.output

    def set_integral_limit(self, limit):
        self.integral_limit = limit

    def set_sample_time(self, sample_time):
        self.sample_time = sample_time

    def integral_limit_method(self, sum_error):
        self.sum_error = sum_error
        if self.sum_error > self.integral_limit:
            self.sum_error = self.integral_limit

        if self.sum_error < -1 * self.integral_limit:
            self.sum_error = -1 * self.integral_limit

        return self.sum_error

    def __next__(self):
        self.sim.input_update(self.u_t)
        self.sim.output_update(motor)
        self.sum_error = self.integral_limit_method(
            self.current_error + self.sum_error)
        self.current_error = self.desired_value - self.sim.output
        self.difference = (self.current_error -
                           self.previous_error) / self.sample_time
        self.previous_error = self.current_error
        self.u_t = self.kp * self.current_error + self.ki * self.sample_time * \
                    self.sum_error + self.kd * self.difference
        self.read_current_value()        
        return self

    def __iter__(self):
        return self


if __name__ == "__main__":
    desired = 2000
    iterations = 200
    sim = Simulator(0, desired, iterations, noise_flag=True, emergence=False)
    pid_controller = Pid(desired, 0.8, 3, 0., sim)
    pid_controller.set_sample_time(0.3)
    pid_controller.set_integral_limit(10000)
    for i in range(iterations):
        next(pid_controller)
    pid_controller.sim.animation()
