import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Int8
import matplotlib.pyplot as plt
import numpy as np
import threading

class Monitor_node(Node):
    def __init__(self):
        super().__init__("monitor")
        self.sub = self.create_subscription(Float32MultiArray,"/monitor",self.callback_function,10)
        self.k_pub = self.create_publisher(Float32MultiArray,"k_vals",10)
        self.mode_pub = self.create_publisher(Int8,"/mode",10)
        threading.Thread(target = self.input_loop,daemon = True).start()
        plt.ion()
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.mode = 0
        self.modes = ['software','keystroke','joystick']
        self.estop_sub = self.create_subscription(Int8,"estop",self.estop_callback,10)
        self.estop = 1
        self.left_rpm_values = np.zeros(10)
        self.left_setpoint_values = np.zeros(10)
        self.right_rpm_values = np.zeros(10)
        self.right_setpoint_values = np.zeros(10)
        self.figure,self.axes = plt.subplots(nrows = 1,ncols = 2,figsize = (24,12))
        self.left_rpm_line, = self.axes[0].plot(self.left_rpm_values,color = "blue",label ="RPM")
        self.left_setpoint_line, = self.axes[0].plot(self.left_setpoint_values,color = "green",label = "setpoint",drawstyle = "steps-post")
        self.right_rpm_line, = self.axes[1].plot(self.right_rpm_values,color = "blue",label = "RPM")
        self.right_setpoint_line, = self.axes[1].plot(self.right_setpoint_values,color = "green",label = "setpoint",drawstyle = "steps-post")
        self.axes[0].set_title("Left RPM")
        self.axes[0].grid(True)
        self.axes[0].set_ylim(-300,300)
        self.axes[0].legend()
        self.axes[1].set_title("Right RPM")
        self.axes[1].grid(True)
        self.axes[1].set_ylim(-300,300)
        self.axes[1].legend()
        self.k_text = self.figure.text(0.5,0.02,"",ha = 'center')
        self.mode_text = self.figure.text(0.5,0.06,"",ha = 'center')
        self.estop_text = self.figure.text(0.5,0.04,"",ha = 'center')
    def estop_callback(self,msg):
        self.estop = msg.data
    def callback_function(self,msg):
        left_wheel_setpoint = msg.data[0]
        right_wheel_setpoint = msg.data[1]
        left_wheel_rpm_value = msg.data[2]
        right_wheel_rpm_value = msg.data[3]
        self.k_text.set_text(f"kp={self.kp},ki={self.ki},kd={self.kd}")
        self.mode_text.set_text(f"Mode: {self.modes[self.mode]}")
        self.estop_text.set_text(f"Estop: {bool(self.estop)}")
        self.left_rpm_values = np.roll(self.left_rpm_values,-1)
        self.left_rpm_values[-1] = left_wheel_rpm_value
        self.left_rpm_line.set_ydata(self.left_rpm_values)
        self.left_setpoint_values = np.roll(self.left_setpoint_values,-1)
        self.left_setpoint_values[-1] = left_wheel_setpoint
        self.left_setpoint_line.set_ydata(self.left_setpoint_values)
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
                vals = input().strip().split()
                if len(vals) == 2:
                    if vals[0] in 'Mm':
                        if vals[1] in ['0','1','2']:
                            msg = Int8()
                            self.mode = msg.data = int(vals[1])
                            self.mode_pub.publish(msg)
                        else:
                            raise ValueError
                    else:
                        raise ValueError
                elif len(vals) == 4:
                    if vals[0] in 'kK':
                        msg = Float32MultiArray()
                        self.kp,self.ki,self.kd = msg.data = [float(i) for i in vals[1:]]
                        self.k_pub.publish(msg)
                    else:
                        raise ValueError
                else:
                    raise ValueError
            except Exception as e:
                self.get_logger().warn("Invalid input")

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