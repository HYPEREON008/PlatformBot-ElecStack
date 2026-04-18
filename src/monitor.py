import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from matplotlib import pyplot as plt
import numpy as np
#monitor Code

#displays the throttle value, set point value and the encoder value.

class myNode(Node):

    def __init__ (self):
        super().__init__("monitor")
        self.subscriber_object = self.create_subscription(Float32MultiArray,"/monitor",self.callback_function,10)
        plt.ion()
        
        #Setting the starting array to be 0s
        self.left_throttle_values = np.zeros(10)
        self.left_encoder_values = np.zeros(10)
        self.left_setpoint_values = np.zeros(10)


        self.right_throttle_values = np.zeros(10)
        self.right_encoder_values = np.zeros(10)
        self.right_setpoint_values = np.zeros(10)
        
        # Creating a matpoltlib fig
        self.figure,self.axes = plt.subplots(nrows=1,ncols=2,figsize=(12,12))

        self.left_throttle_line, = self.axes[0].plot(self.left_throttle_values,color = "red",marker ="o",label = "throttle")
        self.left_encoder_line, = self.axes[0].plot(self.left_encoder_values,color = "blue",marker ="o", label ="encoder")
        self.left_setpoint_line, = self.axes[0].plot(self.left_setpoint_values,color = "green",marker ="o",label ="setpoint")


        self.right_throttle_line, = self.axes[1].plot(self.right_throttle_values,color = "red",marker ="o",label = "throttle")
        self.right_encoder_line, = self.axes[1].plot(self.right_encoder_values,color = "blue",marker ="o", label ="encoder")
        self.right_setpoint_line, = self.axes[1].plot(self.right_setpoint_values,color = "green",marker ="o",label ="setpoint")

        self.axes[0].set_title("Left Encoder")
        self.axes[0].grid(True)
        self.axes[0].set_ylim(-200,200)
        self.axes[0].legend() 


        self.axes[1].set_title("Right Encoder")
        self.axes[1].grid(True)
        self.axes[1].set_ylim(-200,200)
        self.axes[1].legend()




    def callback_function(self, msg : Float32MultiArray):
        
        left_wheel_setpoint = msg.data[0]
        right_wheel_setpoint = msg.data[1]
        left_wheel_encoder_value = msg.data[2]
        right_wheel_encoder = msg.data[3]
        left_wheel_throttle = msg.data[4]
        right_wheel_throttle = msg.data[5]

        self.left_throttle_values = np.roll(self.left_throttle_values,-1)
        self.left_throttle_values[-1] = left_wheel_throttle
        self.left_throttle_line.set_ydata(self.left_throttle_values)

        self.left_encoder_values = np.roll(self.left_encoder_values,-1)
        self.left_encoder_values[-1]=left_wheel_encoder_value
        self.left_encoder_line.set_ydata(self.left_encoder_values)

        self.left_setpoint_values = np.roll(self.left_setpoint_values,-1)
        self.left_setpoint_values[-1] = left_wheel_setpoint
        self.left_setpoint_line.set_ydata(self.left_setpoint_values)

        self.right_throttle_values = np.roll(self.right_throttle_values,-1)
        self.right_throttle_values[-1] = right_wheel_throttle
        self.right_throttle_line.set_ydata(self.right_throttle_values)

        self.right_encoder_values = np.roll(self.right_encoder_values,-1)
        self.right_encoder_values[-1] = right_wheel_encoder
        self.right_encoder_line.set_ydata(self.right_encoder_values)

        self.right_setpoint_values = np.roll(self.right_setpoint_values,-1)
        self.right_setpoint_values[-1] = right_wheel_setpoint
        self.right_setpoint_line.set_ydata(self.right_setpoint_values)

        self.axes[0].set_xlim(0, len(self.left_encoder_values)-1)
        self.axes[1].set_xlim(0, len(self.right_encoder_values)-1)


        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
        plt.pause(0.001)
        
def main(args = None):
    rclpy.init(args=args)
    node = myNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
