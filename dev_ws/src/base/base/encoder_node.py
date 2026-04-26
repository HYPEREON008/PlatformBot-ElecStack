import rclpy
from serial import *
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from time import sleep

class encoder(Node):
    def __init__(self,port):
        super().__init__("encoder_node")
        self.publisher = self.create_publisher(Float32MultiArray, 'rpm',10)
        self.port = port
        self.started = False
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.serial = None
        self.open_serial()
        self.started = False
        self.previous = 0

    def open_serial(self):
        
        while True:
            try:
                self.serial = Serial(
                port = self.port,
                baudrate = 115200,
                parity = PARITY_NONE,
                stopbits = STOPBITS_ONE,
                bytesize=EIGHTBITS,
                timeout = 0.01
            )
            
                if self.serial.isOpen():
                    self.get_logger().info(f"Serial connection successful: {self.port}")
                    break

            except IOError:
                self.get_logger().info(f"Waiting for seiral port to open")
                sleep(0.05)

                    



    def timer_callback(self):
        try:
            raw = self.serial.readline()
            enc = raw.decode("utf-8").strip()
            if len(enc) == 0:
                return
            

            if enc == "Started":
                self.started = True
                self.get_logger().info("Succsessful handshake with teensy")
                self.serial.write(b"++\n")
                return

            elif(not self.started):
                if (enc[0] =="$" and enc[-1] == "&"):
                    self.serial.write(b"--\n")
                    self.get_logger().info("Improper handshake with teensy, restart again")
                    self.started = False
                    self.previous = 0
                    return

            if enc[0] == "$" and enc[-1] == "&":
                    
                    enc = enc[1:-1].split() 
                    if len(enc) != 3:
                        self.get_logger().info("Data sent in wrong format.")
                        self.started = False
                        self.serial.write(b"--\n")

                    else:

                        if ((int(enc[2]) - self.previous+1000)%1000!=1):
                            self.get_logger().info("Sequence number has exceeded 1000 counts")
                            self.started = False
                            self.serial.write(b"--\n")
                        self.previous = int(enc[2])

                        rpm = Float32MultiArray()

                        rpm.data = [float(enc[0]), float(enc[1])]
                        self.publisher.publish(rpm)
                        self.get_logger().info(f"Published enc data: {rpm.data}")

            else:
                self.get_logger().info("Data sent in wrong format")
                self.started = False
                self.serial.write(b"--\n")

        except:
            self.get_logger().info("Couldnt read from serial")


def main(args = None):
        rclpy.init(args=args)
        node = None
        try:
            node = encoder("portname") ## put port name inside the brackets
            if node is not None:
                rclpy.spin(node)
                print("Port has been connected successfully")

        except:
            print("Port has not bee found")

        finally:
            if node is not None:
                node.destroy_node()
            
        rclpy.shutdown()            





if __name__ == "__main__":
    main()

