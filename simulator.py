import random
import numpy as np
from collections import namedtuple
from matplotlib import pyplot as plt, animation as ani, transforms
from numpy.lib.arraysetops import intersect1d

Noise = namedtuple("Noise", ["miu","sigma"])


def motor(*arg):
    """
    该执行器为一个直流减速电机，PWM12位分辨率(4096级)  
    经过PID控制器后得到一个PWM占空比参数，假设输入与输出存在一定的线性关系
    """    
    input = arg[0] if arg[0] < 4096 else 4096

    return input 


class Simulator():
    """
    该仿真器用于模拟控制系统中的执行器、噪声、突发事件等等
    """
    def __init__(self, input, desired_value, iterations=100, noise_flag=False, noise: Noise=None, emergence=True, coefficient=1, emergence_times=1) -> None:
        """
        # Parameters
        - input 为执行器的输入
        - desired_value 表示期望值
        - iterations 表示迭代次数
        - noise_flag 表示是否开启高斯噪声默认不开启  
        - noise 设置高斯噪声的参数如果不存在则自动设置
        - emergence 是否开启突发事件模拟默认开启
        - coefficient 突发事件的剧烈程度用于测试系统的鲁棒性
        - emergence_times 表示突发事件发生的次数
        """
        self.input = input
        self.desired_value = desired_value
        self.iterations = iterations
        self.noise = (0, 0) if not noise_flag else noise if noise != None else (0, self.desired_value * 0.01)
        self.coefficient = coefficient
        self.emergence = self.coefficient * self.desired_value if emergence else 0
        self.emergence_list = [random.randint(0, int(iterations * 0.8)) for i in range(emergence_times)]  #iterations * 0.8是为了避免出现过后的突发事件
        self.count = 0
        self.output = desired_value
        self.pid_y_list = []


    def time2emergence(self) -> bool:
        if self.count in self.emergence_list:
            self.emergence_list.remove(self.count)
            return True
        return False


    def input_update(self, input):
        self.input = input


    def output_update(self, Actuators):
        """
        Actuators为执行器
        """ 

        self.count += 1

        if self.time2emergence():
            self.output = np.random.normal(*self.noise) + Actuators(self.input) + self.emergence
        else:
            self.output = np.random.normal(*self.noise) + Actuators(self.input)

        self.pid_y_list.append(self.output)
    
    def _animation_init(self):
        """
        初始化动画
        """
        self.fig = plt.figure("PID Controller Simulator Gif")
        self.ax = plt.gca()
        self.line = plt.plot(0, 0, "red")[0]
        self.current_value_line = plt.plot(0, 0, "blue")[0]
        self.ax = plt.gca() 
        self.N_text = plt.text(0.95, 0.95, "N=0", horizontalalignment="right", verticalalignment="top", transform=self.ax.transAxes, fontsize=12) #固定文本框
        self.output_text = plt.text(0.95, 0.9, "output={:.3f}".format(self.pid_y_list[0]), horizontalalignment="right", verticalalignment="top", transform=self.ax.transAxes, fontsize=12)

    def _update(self, num):
        """
        数据以及文本框的更新
        """
        plt.xlim((0, int(num * 1.1)))
        plt.ylim((0, 1.2 * self.pid_y_list[num]))
        self.line.set_data(np.arange(num), [self.desired_value] * num)
        self.current_value_line.set_data(range(num), self.pid_y_list[:num])
        self.N_text.set_text("N= {}".format(num))
        self.output_text.set_text("output= {:.3f}".format(self.pid_y_list[num]))        
        return self.current_value_line, self.line, self.N_text, self.output_text



    def animation(self):
        self._animation_init()
        animer = ani.FuncAnimation(fig=self.fig, func=self._update, frames=np.arange(0, self.iterations), interval=10)
        animer.save("PID Controller.gif") #如果先show再save将报错，我也不知道为什么
        plt.show()
        
        


# if __name__ == "__main__":
#     sim = Simulator(2000, 800, noise_flag=True)
#     for i in range(100):
#         sim.output_update(motor)
#     sim.animation(100)