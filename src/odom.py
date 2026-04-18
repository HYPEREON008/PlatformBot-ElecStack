import rclpy
from rclpy.node import Node
import math
import queue
import time
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray,Bool
from geometry_msgs.msg import Quaternion


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


class Odom(Node):
    wheel_dist = 0.395
    wheel_radius = 0.1016

    #Enc params
    pulses_per_rev= 1024
    gear_ratio = 1
    counts_per_rev = pulses_per_rev * 4 * gear_ratio

    flip_motor_channels = False

    def __init__(self):
        super().__init__('odometry')

        
        self.odom_topic = "Odom"
        self.wheel_vel_topic = "wheel_vel"
        self.reset_topic = "O_reset"
        self.odom_reset = self.create_subscription(Bool, self.reset_topic,self.reset,10)
        self.odom_pub = self.create_publisher(Odometry , self.odom_topic , 10)
        #self.vel_pub = self.create_publisher(WheelVel , self.wheel_vel_topic , 10)
        ## below line for odom reset
        #Odometry and Wheel Velocity
        self.odom: Odometry = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_footprint"

        #parameters of vehicle
        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = 0.0

        self.vx: float = 0.0
        self.vy: float = 0.0
        self.wz: float = 0.0

        self.time_now = time.time()
        self.time_prev = time.time()

        self.dt = 0.0

        self.rpm = Float64MultiArray()
        self.pulses_msg = Float64MultiArray()
        self.pulses = [0,0]
        self.ntot = [0,0]
        # timer to publish odometry data
        self.timer_period = 0.1 # frequency = 10 Hertz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        #Subscribe to encoder pulses and rpm
        self.subscription = self.create_subscription(Float64MultiArray,'/enc_pulses',self.pulses_callback,10)
        self.subscription = self.create_subscription(Float64MultiArray,'/enc_RPM',self.rpm_callback,10)
        
        self.counter = 0 #counter for when to update the Odometry values
        self.updated = False #To monitor the state of odometry

    def reset(self,msg):
        if(msg.data == True):
            self.x = self.y = self.z = 0.0
            self.vx = self.vy = self.wz = 0.0
            self.yaw = 0.0
            self.updated = False

    # timer_callback for publishing data
    def timer_callback(self):
        # If data is updated , publish self.odom , otherwise , update self.odom and then publish.
        if(not self.updated):
            self.odom.header.stamp = Node.get_clock(self).now().to_msg()
            self.odom.header.frame_id = "odom"
            self.odom.pose.pose.position.x = self.x
            self.odom.pose.pose.position.y = self.y
            self.odom.pose.pose.orientation = quaternion_from_euler(0,0,self.yaw)
            self.odom.twist.twist.linear.x = self.vx
            self.odom.twist.twist.linear.y = self.vy
            self.odom.twist.twist.angular.z = self.wz
            self.updated = True
        self.odom_pub.publish(self.odom)
        self.get_logger().debug("Published")
    
    # update odometry values
    def compute(self):
        self.counter = 0
        [n1,n2] = self.pulses
        self.time_now = time.time()
        dt = self.time_now- self.time_prev
        self.time_prev =self.time_now
        self.ntot[0] += n1
        self.ntot[1] += n2

        # calculate theta1 and theta2
        th1 , th2 = 2 * math.pi * n1 / self.counts_per_rev , 2 * math.pi * n2 / self.counts_per_rev
        # calculate distance travelled by the wheels
        s1,s2 = th1 * self.wheel_radius , th2 * self.wheel_radius

        #Odometry calculations for the current design
        ds = (s2 + s1)/2
        dyaw = 2*(s2 - s1) / (2 * self.wheel_dist)
        cos_yaw = math.cos(self.yaw + (dyaw))
        sin_yaw = math.sin(self.yaw + (dyaw))
        dx = ds * cos_yaw
        dy = ds * sin_yaw
        self.vx = dx / dt
        self.vy = dy / dt
        self.wz = dyaw / dt

        self.yaw += dyaw
        self.x += dx
        self.y += dy
        self.pulses = [0,0]
        self.updated = False # self.odom now needs to be updates

    def rpm_callback(self,msg):
        self.rpm = msg
    def pulses_callback(self,msg):
        self.pulses_msg = msg
        self.pulses[0] += msg.data[0]
        self.pulses[1] += msg.data[1]
        self.counter += 1
        if(self.counter == 5):
            self.compute()
   


def main(args = None):
    rclpy.init(args = args)
    odometry = Odom()
    rclpy.spin(odometry)
    odometry.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
