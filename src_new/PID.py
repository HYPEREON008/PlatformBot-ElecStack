import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8,Float32MultiArray

class Compute:  # this is to compute throttle value
    def __init__(self,thr_limit,kp,ki,kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.deriv_filter = 0.7 # making derivative smooth
        self.thr_min = thr_limit[0] # minimum throttle
        self.thr_max = thr_limit[1] # maximum throttle
        self.error = 0
        self.prev_error = 0
        self.integral = 0
        self.derivative = 0
        self.prev_derivative = 0
        self.setpoint = 0
        self.throttle = 0
    def hard_reset(self): # this is for estop
        self.error = 0
        self.prev_error = 0
        self.integral = 0
        self.derivative = 0
        self.setpoint = 0
        self.throttle = 0
    def soft_reset(self): # resetting the error
        self.error = 0
        self.prev_error = 0
        self.integral = 0
        self.derivative = 0
    def compute(self,cur_vel,dt):
        self.prev_error = self.error
        self.prev_derivative = self.derivaive
        self.error = self.setpoint - cur_vel
        if self.ki:
            max_integral = (self.thr_max - self.kp * self.error) / self.ki
            min_integral = (self.thr_min - self.kp * self.error) / self.ki
        else:
            max_integral=min_integral=0.0
        self.integral += self.error * dt
        self.integral = max(min_integral,min(max_integral,self.integral))
        self.derivative = self.deriv_filter * self.prev_derivative + (1 - self.deriv_filter) * (self.error - self.prev_error) / dt if dt > 0 else 0.0
        self.throttle = max(self.thr_min,min(self.thr_max,self.kp * self.error + self.ki * self.integral + self.kd * self.derivative))
