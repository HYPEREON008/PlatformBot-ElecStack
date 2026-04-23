import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
import threading

class Monitor_node(Node):
    def __init__(self):
        super().__init__("monitor")
        self.sub = self.create_subscription(Float32MultiArray,"/monitor",self.callback_function,10)
        self.pub = self.create_publisher(Float32MultiArray,"k_vals",10)
        threading.Thread(target=self.input_loop,daemon = True).start()
        plt.ion()
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.left_throttle_values = np.zeros(10)
        self.left_rpm_values = np.zeros(10)
        self.left_setpoint_values = np.zeros(10)
        self.right_throttle_values = np.zeros(10)
        self.right_rpm_values = np.zeros(10)
        self.right_setpoint_values = np.zeros(10)
        self.figure,self.axes = plt.subplots(nrows=1,ncols=2,figsize=(12,12))
        self.left_throttle_line, = self.axes[0].plot(self.left_throttle_values,color = "red",marker ="o",label = "throttle")
        self.left_rpm_line, = self.axes[0].plot(self.left_rpm_values,color = "blue",marker ="o", label ="RPM")
        self.left_setpoint_line, = self.axes[0].plot(self.left_setpoint_values,color = "green",marker ="o",label ="setpoint")
        self.right_throttle_line, = self.axes[1].plot(self.right_throttle_values,color = "red",marker ="o",label = "throttle")
        self.right_rpm_line, = self.axes[1].plot(self.right_rpm_values,color = "blue",marker ="o", label ="RPM")
        self.right_setpoint_line, = self.axes[1].plot(self.right_setpoint_values,color = "green",marker ="o",label ="setpoint")
        self.axes[0].set_title("Left RPM")
        self.axes[0].grid(True)
        self.axes[0].set_ylim(-200,200)
        self.axes[0].legend()
        self.axes[1].set_title("Right RPM")
        self.axes[1].grid(True)
        self.axes[1].set_ylim(-200,200)
        self.axes[1].legend()
        self.k_text = self.figure.text(0.5,0.05,"",ha = 'center')
    def callback_function(self,msg):
        left_wheel_setpoint = msg.data[0]
        right_wheel_setpoint = msg.data[1]
        left_wheel_rpm_value = msg.data[2]
        right_wheel_rpm_value = msg.data[3]
        left_wheel_throttle = msg.data[4]
        right_wheel_throttle = msg.data[5]
        self.k_text.set_text(f"kp={self.kp},ki={self.ki},kd={self.kd}")
        self.left_throttle_values = np.roll(self.left_throttle_values,-1)
        self.left_throttle_values[-1] = left_wheel_throttle
        self.left_throttle_line.set_ydata(self.left_throttle_values)
        self.left_rpm_values = np.roll(self.left_rpm_values,-1)
        self.left_rpm_values[-1] = left_wheel_rpm_value
        self.left_rpm_line.set_ydata(self.left_rpm_values)
        self.left_setpoint_values = np.roll(self.left_setpoint_values,-1)
        self.left_setpoint_values[-1] = left_wheel_setpoint
        self.left_setpoint_line.set_ydata(self.left_setpoint_values)
        self.right_throttle_values = np.roll(self.right_throttle_values,-1)
        self.right_throttle_values[-1] = right_wheel_throttle
        self.right_throttle_line.set_ydata(self.right_throttle_values)
        self.right_rpm_values = np.roll(self.right_rpm_values,-1)
        self.right_rpm_values[-1] = right_wheel_rpm_value
        self.right_rpm_line.set_ydata(self.right_rpm_values)
        self.right_setpoint_values = np.roll(self.right_setpoint_values,-1)
        self.right_setpoint_values[-1] = right_wheel_setpoint
        self.right_setpoint_line.set_ydata(self.right_setpoint_values)
        self.axes[0].set_xlim(0,len(self.left_rpm_values) - 1)
        self.axes[1].set_xlim(0,len(self.right_rpm_values) - 1)
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
        plt.pause(0.001)
    def input_loop(self):
        while True:
            try:
                kp,ki,kd = input().split("") # kp ki kd is inout format
                self.kp = float(kp)
                self.ki = float(ki)
                self.kd = float(kd)
                msg = Float32MultiArray()
                msg.data = [self.kp,self.ki,self.kd]
                self.pub.publish(msg)
            except Exception as e:
                print("Invalid input")

def main(args = None):
    rclpy.init(args = args)
    node = Monitor_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()