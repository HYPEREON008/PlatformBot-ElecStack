#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from serial import * 
from time import sleep

class encoder_serial_reading(Node):
    
    def __init__(self,serPort): 
        
        super().__init__("encoder_serial")
        self.get_logger().info("initing node")

        #initilaizing the variables which we will be using 
        self.serPort = serPort # Port Address
        self.rpm = self.create_publisher(Float32MultiArray,"rpm",10) # Publisher 1
        self.pulse = self.create_publisher(Float32MultiArray,"pulse",10) #Publisher 2
        self.ser = 0 
        self.open_serial() #tries to establish Serial connection 
        self.timer = self.create_timer(0.01,self.callback) # Timer 
        self.started = False #acts as a flag
        
    def callback(self):
            try:
                raw = self.ser.readline() 
                
                #reading the output from serial in the form of b"Started \r \n" and it will remove the unwanted parts 
                #Later the same code will remove the unwanted parts of the encoder data 
                
                # The data is of the form $ pulse1 pulse2 rpm1 rpm2 seq_no & -> Final data
                
                enc = raw.decode("utf-8").strip()
                if enc == "Started":
                    
                    # Proper connection acheived (handshake)
                    self.started = True
                    self.get_logger().info("The Connection has been established Properly",once = True)
                    self.prev_sequence = 0
                    self.ser.write(b"++\n") # For acknowledging Tiva (ack msg)
                
                elif(not self.started):
                    
                    if enc[0] == "$" and enc[-1] =="&":
                        
                        self.ser.write(b"--\n") # to start the Tiva code again (failure msg)
                        self.get_logger().info("Improper Handshake, restart Required")
                    
                    self.get_logger().info("Connection is not Proper -> Waiting for Tiva",once = True)                
                
                if enc[0] == "$" and enc[-1] == "&":
                    
                    enc = enc[1:-1].split() # This will split them into list ["pulse1","pulse2","rpm1","rpm2","sequence"]
                    print(enc)
                    self.counter = int(enc[-1])
                    
                    if ((self.counter - self.prev_sequence+1000)%1000!=1): # 1000 is taken as that is the max_sequence from the Tiva code
                        self.get_logger().warning("DATA LOSS FROM TIVA",once = True)    
                        self.started = False
                           
                    self.prev_sequence = self.counter     
                    
                    
                    if len(enc) != 5: 
                        
                        #if enc doesnt have all the required readings
                        
                        self.get_logger().warning("Error in reading the data",once = True )
                        self.started = False

                    else:
                        # Getting the values for the rpm and the pulse and then publishing them       
                        rpm = Float32MultiArray()
                        pulse = Float32MultiArray()
                        
                        try:
                            
                            rpm.data = [float(enc[2]),float(enc[3])]
                            self.rpm.publish(rpm)
                            pulse.data = [float(enc[0]),float(enc[1])]
                            self.pulse.publish(pulse)
                            self.get_logger().info(f"Successfully Published {rpm.data}  {pulse.data}")   
                            
                        except:
                            
                            self.get_logger().warning("Value Error",once = True)
                            self.started = False

            except:
                
                while True:
                    
                    try:
                        
                        self.open_serial()
                        break
                    
                    except SerialException:
                        sleep(0.01)
                        
    # function to open the serial monitor at the port address
    def open_serial(self):
        try:
            self.ser = Serial(
                port = self.serPort,
                baudrate = 921600,
                parity = PARITY_NONE,
                stopbits = STOPBITS_ONE,
                bytesize=EIGHTBITS,
                timeout = 0.01
            )
            if self.ser.isOpen():
                self.get_logger().info(f"Serial has been established and Port has been opened {self.serPort}",once = True)
        
    # Port having error issues due to it being busy or someother issue        
        except IOError:
            self.ser = Serial(
                port = self.serPort,
                baudrate = 921600,
                parity = PARITY_NONE,
                stopbits = STOPBITS_ONE,
                bytesize=EIGHTBITS,
                timeout = 0.01
            )
            # Trying to wait for the port to open if its not working due to port being busy or something 
            self.get_logger().info("Waiting for the port to open")
            self.ser.close()
            self.ser.open()
            if self.ser.isOpen():
                self.get_logger().info(f"Serial has been established and Port has been opened {self.serPort}",once = True)  
        



def main(args = None):
    rclpy.init(args = args)
    node = None
    # try block to make sure that the node is properly initialized and 
    # if any error -> The cause is printed out.
    try:
        node = encoder_serial_reading("/dev/serial/by-id/usb-Teensyduino_USB_Serial_15612700-if00")
        if node is not None:
            rclpy.spin(node)
            
    except Exception as e :
        print(f"Failed Initializing the node due to {e}")
        
    finally:
        if node is not None:
            node.destroy_node()
            
    rclpy.shutdown()

#executing the script from the terminal 

if __name__ == "__main__":
    main()
