# from collections import namedtuple
# from math import exp
from matplotlib import pyplot as plt
import numpy as np
import time


def method():

    return 4000 + np.random.randn() * 10
    # return 4000 + int(str(time.time())[-3:])


class Pid(object):

    def __init__(self, desired_value, kp, ki, kd) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.desired_value = desired_value
        self.current_value = 0
        self.current_error = desired_value
        self.sum_error = 0
        self.previous_error = desired_value
        self.u_t = 0
        self.sample_time = 0.3

    def read_current_value(self, method=method):
        self.current_value = method() + self.u_t

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
        self.sum_error = self.integral_limit_method(
            self.current_error + self.sum_error)
        self.previous_error = self.current_error
        self.read_current_value()
        self.current_error = self.desired_value - self.current_value
        self.difference = (self.current_error -
                           self.previous_error) / self.sample_time
        self.u_t = self.sample_time * ((self.kp * self.current_error + self.ki * self.sample_time *
                    self.sum_error + self.kd * self.difference))
        # u_t表示通过执行器后的输出结果，其中执行器的传递函数为1/s，也就是通过一个积分器
        return self

    def __iter__(self):
        return self


if __name__ == "__main__":
    pid_controller = Pid(4000, 0.9, 3, 0)
    pid_controller.set_sample_time(0.3)
    pid_controller.set_integral_limit(10000)
    controller_output = []
    for i in range(1000):
        controller_output.append(next(pid_controller).current_value)
        print(pid_controller.sum_error)
    x = np.arange(len(controller_output[1:]))
    plt.plot(x, controller_output[1:])
    plt.show()
