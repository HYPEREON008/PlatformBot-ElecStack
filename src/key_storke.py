#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8
import sys,termios,tty,select
#importing important Libraries
class key_board_node(Node):
    
    def __init__(self):
        super().__init__("keyboard_control")
        self.get_logger().info("Key controls")
        self.get_logger().info("Press w to move forward ADD[+10,+10]")
        self.get_logger().info("Press s to move backwards ADD[-10,-10]")
        self.get_logger().info("Press a to move left ADD[-10,+10]")
        self.get_logger().info("Press d to move right ADD[+10,-10]")
        self.get_logger().info("Press z to reset to [0,0]")
        # self.get_logger().info("Press b to send [-10,0]")
        # self.get_logger().info("Press n to send [+10,-10]")
        # self.get_logger().info("Press m to send [0,-10]")
        # self.get_logger().info("Press j to send [-10,+10]")
        self.get_logger().warn("Press esc or Ctrl+C to kill the node and activate e-stop")
        self.get_logger().warn("Press p to kill the node and **NOT** activate e-stop")
        self.get_logger().warn("Press e to trigger e-stop")
        self.get_logger().warn("Press r to reset e-stop")
        self.get_logger().warn("Pressing any other key will initiate e-stop")
        
        self.key_stroke = self.create_publisher(Float32MultiArray,"keystroke",10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.timer = self.create_timer(0.1,self.key_loop)
        self.left_velocity = 0
        self.right_velocity = 0
        self.e_stop_pub = self.create_publisher(Int8, 'estop', 10)
        
    
    def key_loop(self):
        msg = Float32MultiArray()
        msg.data = [0,0]

        key = self.getkey()
    

 
        if key is None:
            return  # no key pressed 

        if key == "w":

            msg.data = [self.left_velocity + 10.0 , self.right_velocity + 10.0]
            


        elif key == "s":

            msg.data = [self.left_velocity - 10.0 , self.right_velocity - 10.0]
            
        
        
        elif key == "a":
            msg.data = [self.left_velocity - 10.0 , self.right_velocity + 10.0]
            

        
        elif key == "d":
            msg.data = [self.left_velocity+ 10.0 , self.right_velocity - 10.0]
            


        # elif key == "b":
        #     msg.data = [self.left_velocity - 10.0 ,self.right_velocity]


        # elif key == "m":
        #     msg.data = [self.left_velocity , self.right_velocity - 10.0 ]


        elif key == "z":
            msg.data = [0.0,0.0]


        # elif key == "j":
        #     msg.data = [-10.0 ,10.0]


        # elif key == "n":
        #     msg.data = [10.0 , -10.0]

        elif key == "e":
            e_stop_msg = Int8()
            e_stop_msg.data = 1
            self.e_stop_pub.publish(e_stop_msg)


        elif key == "r":
            e_stop_msg = Int8()
            e_stop_msg.data = 0
            self.e_stop_pub.publish(e_stop_msg)

        elif key == "\x03" or key == "\x1b":
            e_stop_msg = Int8()
            e_stop_msg.data = 1
            self.e_stop_pub.publish(e_stop_msg) 
            rclpy.shutdown()

        elif key == "p":
            rclpy.shutdown()

        else:
            self.get_logger().info("Error e-stop intialized")
            e_stop_msg = Int8()
            e_stop_msg.data = 1
            self.e_stop_pub.publish(e_stop_msg) 
            return
        
        

        if (key == 'a' or key == 'd'):
            self.left_velocity = 0
            self.right_velocity = 0

        else:
            
            self.left_velocity = msg.data[0]
            self.right_velocity = msg.data[1]

       # self.prev_key = key
        self.key_stroke.publish(msg)
        # only publish if valid key
        self.get_logger().info(f"Sent {msg.data}")
            
    def getkey(self):
        tty.setraw(sys.stdin.fileno())
        rlist,_,_ = select.select([sys.stdin],[],[],0.1)
        key = sys.stdin.read(1) if rlist else None
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,self.settings)
        return key


def main(args=None):
    rclpy.init(args=args)
    node = key_board_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Exiting")
    finally:
        rclpy.shutdown()

if __name__ =="main":
    main()
