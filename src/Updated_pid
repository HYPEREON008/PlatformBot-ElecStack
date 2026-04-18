import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Float32MultiArray
import math
import time

class Compute_PID:
    def __init__(self,thr_limit,kp,ki,kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        #Min and max throttle
        self.thrmin = thr_limit[0]
        self.thrmax = thr_limit[1]

        self.error = 0
        self.error_prev = 0
        self.error_sum = 0
        #self.deriv_prev=0.0

        self.setpoint = 0 #Desired velocity
        self.thr = 0      #Output throttle
    # To reset the PID state variables when E-Stop is triggered
    def reset(self):
        self.error = 0
        self.error_prev = 0
        self.error_sum = 0
        #self.deriv_prev=0.0

        self.setpoint = 0
        self.thr = 0
    
    #Just resets the accumulated error
    def soft_reset(self):
        self.error_prev = 0
        self.error_sum = 0
        #self.deriv_prev=0.0


    def compute(self,enc,dt):
        self.error_prev = self.error
        self.error = self.setpoint - enc
        if self.setpoint == 0:  
            self.error_sum = 0
        else:
            self.error_sum += self.error

        # max_error_sum = 200
        # self.error_sum = max(-max_error_sum,min(self.error_sum,max_error_sum))

        # deriv = (self.error - self.error_prev)/dt if dt>0 else 0.0
        #deriv = 0.7*self.deriv_prev + 0.3*rawderiv
        #self.deriv_prev = deriv

        self.output = self.kp*self.error + self.ki*self.error_sum

        self.output = max(self.thrmin,min(self.output,self.thrmax))

        return self.output
    
class PID_node(Node):
    def __init__(self,PID_f_scale):

        super().__init__ ('pid_control')

        self.r = 0.124 #Wheel radius
        self.l = 1.12  #Distance between wheels

        #Frequency scaling according to encoder serial frequency
        self.counter = 1
        self.counter_limit = PID_f_scale

        # Encoder readings
        self.left_enc =0
        self.right_enc =0

        # E-stop initialized to False
        self.estop = False
        # Wheel PIDs initialized
        self.lpid = Compute_PID(thr_limit =[-500,500],kp = 2,ki=0.03,kd=0.028)  # 1.3, 0.02, 0.003
        #0.65, 0.05, 0.007 has crazy delay, but reaches setpont perfectly
        # 2.5 0.12 0.0
        # 2 0.013 0.0
        self.rpid = Compute_PID(thr_limit =[-500,500],kp = 2,ki=0.03,kd=0.028)

        self.prev_time = self.get_clock().now().nanoseconds/1e9

        #Subs
        self.cmd_sub = self.create_subscription(Twist,'/cmd_vel_nav',self.cmd_callback,10)
        self.enc_sub = self.create_subscription(Float32MultiArray,'rpm',self.rpm_callback,10)
        self.estop_sub = self.create_subscription(Int8,'estop',self.estop_callback,10)
        self.key_sub = self.create_subscription(Float32MultiArray,'keystroke',self.keystroke_callback,10)

        #Pubs
        self.thr_pub = self.create_publisher(Float32MultiArray,'/thr',10)
        self.mon_pub = self.create_publisher(Float32MultiArray,'/monitor',10)
        self.estop_pub = self.create_publisher(Int8, 'e_stop', 10)

       

        self.active ="software"
        self.last_key_time = None
        self.timeout = 15.0

        #Timers
        # self.timer = self.create_timer(0.1,self.control_loop)
        # self.timer_2 = self.create_timer(0.1,self.check_key)


    #Check if keystroke should be deactivated and switch back to software
    def check_key(self):
        if self.active =="keystroke" and  self.last_key_time is not None:
            if time.time() - self.last_key_time>self.timeout:
                
                self.get_logger().info("Keystroke deactivated")
                self.active = "software"


    #E-Stop
    def estop_callback(self,msg = Int8):
        self.estop = bool(msg.data)
        if self.estop:
            self.lpid.reset()
            self.rpid.reset()
            thr = Float32MultiArray()
            thr.data = [float(0),float(0)]
            self.thr_pub.publish(thr)

            estop_msg = Int8()
            estop_msg.data = 1
            self.estop_pub.publish(estop_msg)
            
            self.get_logger().info("Estop has been engaged")
        
        elif not self.estop:
            self.lpid.reset()
            self.rpid.reset()

            estop_msg = Int8()
            estop_msg.data = 0
            self.estop_pub.publish(estop_msg)

            self.get_logger().info("Estop has been disengaged")

       
    def cmd_callback(self,msg: Twist):
        if self.active == "software":
            v =msg.linear.x
            w = msg.angular.z
            #Converting bot velocity to wheel velocity
            v_l = v - (w*self.l/2)
            v_r = v + (w*self.l/2)

            self.lpid.setpoint = v_l*30/(math.pi*self.r)
            self.rpid.setpoint = v_r*30/(math.pi*self.r)

    def rpm_callback(self,msg: Float32MultiArray):
        
        self.left_enc = msg.data[0]
        self.right_enc = msg.data[1]

        self.control_loop()
        #Trigger PID compute only if counter limit is reached
        # self.counter += 1
        # if self.counter == self.counter_limit:
        #     self.control_loop()
        #     self.counter = 1

    def keystroke_callback(self,msg :Float32MultiArray):
            if self.active !="keystroke":
                self.lpid.soft_reset()
                self.rpid.soft_reset()
                
            self.active ="keystroke"
            self.last_key_time = time.time()

            vl = msg.data[0]
            vr = msg.data[1]

            self.lpid.setpoint = vl
            self.rpid.setpoint = vr


    def control_loop(self):
        #Compute dt
        now = self.get_clock().now().nanoseconds/1e9
        dt = now - self.prev_time
        self.prev_time = now

        if not self.estop:

            left_thr  = self.lpid.compute(self.left_enc,dt)
            right_thr = self.rpid.compute(self.right_enc,dt)

            msg = Float32MultiArray()

            msg.data = [float(left_thr), float(right_thr)]
            # msg.data[0]= left_thr
            # msg.data[1]= right_thr
            self.thr_pub.publish(msg)

            value = Float32MultiArray()
    
            value.data = [self.lpid.setpoint,self.rpid.setpoint,
                          self.left_enc,self.right_enc,
                          left_thr,right_thr
                          ]
            self.mon_pub.publish(value)

            self.get_logger().info(f"[{self.active.upper()} MODE] SP: {self.lpid.setpoint:.2f}, {self.rpid.setpoint:.2f} | ENC: {self.left_enc:.2f}, {self.right_enc:.2f} | THR: {left_thr:.2f}, {right_thr:.2f}")




def main(args=None):
    rclpy.init(args=args)
    node = PID_node(PID_f_scale=2)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


         
