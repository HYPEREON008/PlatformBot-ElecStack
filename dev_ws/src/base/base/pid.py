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
    def compute(self,curr_rpm,dt):
        self.prev_error = self.error
        self.prev_derivative = self.derivative
        self.error = self.setpoint - curr_rpm
        if self.ki:
            max_integral = (self.thr_max - self.kp * self.error) / self.ki
            min_integral = (self.thr_min - self.kp * self.error) / self.ki
        else:
            max_integral=min_integral=0.0
        self.integral += self.error * dt
        self.integral = max(min_integral,min(max_integral,self.integral))
        self.derivative = self.deriv_filter * self.prev_derivative + (1 - self.deriv_filter) * (self.error - self.prev_error) / dt if dt > 0 else 0.0
        self.throttle = max(self.thr_min,min(self.thr_max,self.kp * self.error + self.ki * self.integral + self.kd * self.derivative))
        return self.throttle

class PID_Node(Node):
    def __init__(self):
        super().__init__('pid_control')
        self.wheel_radius = 0.1
        self.wheel_separation = 0.4
        self.rpm_left = 0.0
        self.rpm_right = 0.0
        self.estop = 0 # 1 if estop is triggered
        self.left_pid = Compute([-500,500],0.0,0.0,0.0)
        self.right_pid = Compute([-500,500],0.0,0.0,0.0)
        self.prev_time = self.get_clock().now().nanoseconds/1e9
        self.estop_sub = self.create_subscription(Int8,'estop',self.estop_callback,10)
        self.monitor_sub = self.create_subscription(Float32MultiArray,'k_vals',self.monitor_callback,10)
        self.cmd_sub = self.create_subscription(Twist,'/cmd_vel_nav',self.cmd_callback,10)
        self.rpm_sub = self.create_subscription(Float32MultiArray,'rpm',self.rpm_callback,10)
        self.mode_sub = self.create_subscription(Int8,'mode',self.mode_callback,10) # 0 for software,1 for keystroke, 2 for joystick
        self.keystroke_sub = self.create_subscription(Float32MultiArray,'vel_keystroke',self.keystroke_callback,10)
        self.joystick_sub = self.create_subscription(Twist,'vel_joystick',self.joystick_callback,10)
        self.throttle_pub = self.create_publisher(Float32MultiArray,'/throttle',10)
        self.monitor_pub = self.create_publisher(Float32MultiArray,'/monitor',10)
        self.mode = 0
        self.joystick_resolution = 10
        self.joystick_counter = 0 # will consider every joystick_resolutionth joystick command
        self.control_timer = self.create_timer(0.01,self.control_loop)
    def estop_callback(self,msg):
        self.estop = msg.data
        if self.estop:
            self.left_pid.hard_reset()
            self.right_pid.hard_reset()
            throttle = Float32MultiArray()
            throttle.data = [float(0),float(0)]
            self.throttle_pub.publish(throttle)
            self.get_logger().info("Estop engaged")
        elif not self.estop:
            self.left_pid.soft_reset()
            self.right_pid.soft_reset()
            self.get_logger().info("Estop disengaged")
    def monitor_callback(self,msg):
        self.left_pid.kp = msg.data[0]
        self.left_pid.ki = msg.data[1]
        self.left_pid.kd = msg.data[2]
        self.right_pid.kp = msg.data[0]
        self.right_pid.ki = msg.data[1]
        self.right_pid.kd = msg.data[2]
        self.left_pid.soft_reset()
        self.right_pid.soft_reset()
    def cmd_callback(self,msg):
        if self.mode == 0:
            v = msg.linear.x
            w = msg.angular.z
            v_left = v - w * self.wheel_separation / 2
            v_right = v + w * self.wheel_separation / 2
            self.left_pid.setpoint = v_left * 30 / math.pi / self.wheel_radius
            self.right_pid.setpoint = v_right * 30 / math.pi / self.wheel_radius
    def rpm_callback(self,msg):
        self.rpm_left = msg.data[0]
        self.rpm_right = msg.data[1]
    def mode_callback(self,msg):
        self.mode = msg.data
    def keystroke_callback(self,msg):
        if self.mode == 1:
            v_left = msg.data[0]
            v_right = msg.data[1]
            self.left_pid.setpoint = v_left
            self.right_pid.setpoint = v_right
    def joystick_callback(self,msg):
        self.joystick_counter += 1
        if self.mode == 2 and self.joystick_counter == self.joystick_resolution:
            v = msg.linear.x
            w = msg.angular.z
            v_left = v - w * self.wheel_separation / 2
            v_right = v + w * self.wheel_separation / 2
            self.left_pid.setpoint = v_left * 30 / math.pi / self.wheel_radius
            self.right_pid.setpoint = v_right * 30 / math.pi / self.wheel_radius
            self.joystick_counter = 0
    def control_loop(self):
        now_time = self.get_clock().now().nanoseconds/1e9
        dt = now_time - self.prev_time
        self.prev_time = now_time
        if not self.estop:
            left_throttle = self.left_pid.compute(self.rpm_left,dt)
            right_throttle = self.right_pid.compute(self.rpm_right,dt)
            msg = Float32MultiArray()
            msg.data = [float(left_throttle),float(right_throttle)]
            self.throttle_pub.publish(msg)
            value = Float32MultiArray()
            value.data = [self.left_pid.setpoint,self.right_pid.setpoint,self.rpm_left,self.rpm_right,left_throttle,right_throttle]
            self.monitor_pub.publish(value)

def main(args=None):
    rclpy.init(args=args)
    node = PID_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()