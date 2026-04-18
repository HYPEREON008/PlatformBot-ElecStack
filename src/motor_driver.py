
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8
import time


class motor_driver(Node):
    def __init__(self, serial_port_1, serial_port_2):
        super().__init__('motor_driver')

        # mode 0 says that it is running in open loop
        self.mode = "0"
        self.current_vel_1 = 0
        self.current_vel_2 = 0

        self.ack_error_count = 0
        self.echo_error_count = 0

        self.ack_error_limit = 10
        self.echo_error_limit = 10

        self.serial_port_1 = serial_port_1
        self.serial_port_2 = serial_port_2

        # opening the serial port
        try:
            self.ser_1 = serial.Serial(
                port = self.serial_port_1,
                baudrate = 115200,
                parity = "N",
                stopbits = 1,
                bytesize = 8,
                timeout = 0.1
            )    
            self.ser_1.isOpen()
            print("Opened port", self.ser_1, "!")

        # if the port is occupied and didnt open, close it and open it again
        except IOError:
            self.ser_1 = serial.Serial(
                port = self.serial_port_1,
                baudrate = 115200,
                parity = "N",
                stopbits = 1,
                bytesize = 8,
                timeout = 0.1
            )
            print("Waiting for port to close")
            self.ser_1.close()
            self.ser_1.open()


        # opening the serial port
        try:
            self.ser_2 = serial.Serial(
                port = self.serial_port_2,
                baudrate = 115200,
                parity = "N",
                stopbits = 1,
                bytesize = 8,
                timeout = 0.1
            )    
            self.ser_2.isOpen()
            print("Opened port", self.ser_2, "!")

        # if the port is occupied and didnt open, close it and open it again
        except IOError:
            self.ser_2 = serial.Serial(
                port = self.serial_port_1,
                baudrate = 115200,
                parity = "N",
                stopbits = 1,
                bytesize = 8,
                timeout = 0.1
            )
            print("Waiting for port to close")
            self.ser_2.close()
            self.ser_2.open()

            
        # checking if port is actually open

        if self.ser_1.is_open:
            self.get_logger().info("Opened Serial Port 1")
        else:
            self.get_logger().info("Unable to Open Serial Port 1")
            rclpy.shutdown()


        if self.ser_2.is_open:
            self.get_logger().info("Opened Serial Port 2")
        else:
            self.get_logger().info("Unable to Open Serial Port 2")
            rclpy.shutdown()
        
        
        self.set_mode(self.ser_1)
        self.set_mode(self.ser_2)
        self.config(self.ser_1)
        self.config(self.ser_2)
        
        # the throttle values come in as an array of two integers
        self.thr_sub = self.create_subscription(
            Float32MultiArray,
            'thr',
            self.thr_callback,
            10
        )

        # subscribing to the e-stop topic
        self.e_stop_sub = self.create_subscription(
            Int8,
            'e_stop',
            self.e_stop_callback,
            10
        )

        # we implement e-stop if there are more than 5 acknowledgemnt or echo errors
        # self.publisher = self.create_publisher(Int8, 'e_stop', 10)

        # self.timer = self.create_timer(0.2, self.timer_callback)


    def mcSerialRead(self, ser):
        line = ser.readline().decode().strip()
        return line
        # x = ''
        # line = ''

        # # this is the command to read bytewise data and stop when it is \r
        # while x != b'\r':

        #     # the bytes are read as b'<char>'
        #     line += str(x)[2:-1]
        #     x = ser.read()
        
        # #self.get_logger().info("Read " + str(line) + " from serial" + str(ser))

        # return line


    # to set the drivers at open loop mode 0
    def set_mode(self, ser):

        x = f'^MMOD 1 {str(self.mode)}\r'
        ser.write(x.encode('utf-8'))

        if (self.mcSerialRead(ser) != x[:-1]):
            self.get_logger().warn("EchoErr in setting mode in " + str(ser))
            self.echo_error_count += 1

        if (self.mcSerialRead(ser) != "+"):
            self.get_logger().warn("AckErr in setting mode in " + str(ser))
            self.ack_error_count += 1


    # this command is to save the updates status in the ROM
    def config(self, ser):

        x = f'%EESAV\r'
        ser.write(x.encode('utf-8'))

        if (self.mcSerialRead(ser) != x[:-1]):
            self.get_logger().warn("EchoErr in setting configuration in " + str(ser))
            self.echo_error_count += 1

        if (self.mcSerialRead(ser) != "+"):
            self.get_logger().warn("AckErr in setting configuration in " + str(ser))
            self.ack_error_count += 1


    def thr_callback(self, msg: Float32MultiArray):
        self.get_logger().info("Reading from topic Thr"+str(msg.data[0])+str(msg.data[1]))
        self.current_vel_1 = msg.data[0]
        self.current_vel_2 = msg.data[1]
        self.send_setpoint(self.current_vel_1, self.ser_1) 
        self.send_setpoint(self.current_vel_2, self.ser_2)

        #self.get_logger().info("Sent setpoint " + str(self.current_vel_1) + " to " + str(self.ser_1))
        #self.send_setpoint(msg.data[0], self.ser_1)
        #self.send_setpoint(msg.data[1], self.ser_2)


    def send_setpoint(self, setpoint, ser):

        # command to give velocity setpoints
        x = f'!G 1 {str(setpoint)}\r'
        ser.write(x.encode('utf-8'))

        #self.get_logger().info("Sending setpoint " + str(setpoint) + " to " + str(ser))

        if (self.mcSerialRead(ser) != x[:-1]):
            self.get_logger().warn("EchoErr in sending setpoint in " + str(ser))
            self.echo_error_count += 1
        
        if (self.mcSerialRead(ser) != "+"):
            self.get_logger().warn("AckErr in sending setpoint in " + str(ser))
            self.ack_error_count += 1
        


    
    # keeps sending the velocity setpoint to the wheel non-stop
    # def timer_callback(self):
    #     self.get_logger().info("Setpoint sent 1 : " + str(self.current_vel_1))
    #     self.get_logger().info("Setpoint sent 2 : " + str(self.current_vel_2))
    #     self.send_setpoint(self.current_vel_1, self.ser_1)
    #     self.send_setpoint(self.current_vel_2, self.ser_2)

        # implementing e-stop if ack or echo errors exceed limit
        # if (self.ack_error_count > self.ack_error_limit) or (self.echo_error_count > self.echo_error_limit):
        #     msg = Int8
        #     msg.data = 1
        #     self.publisher.publish(msg)

    # implementing e-stop
    def e_stop_callback(self, msg: Int8):
        
        # for e-stop
        if msg.data == 1:
            x = f'!EX\r'
            self.get_logger().warn("E-Stop implemented")
            self.ser_1.write(x.encode('utf-8'))
            self.ser_2.write(x.encode('utf-8'))

        # to restart an e-stopped motor
        elif msg.data == 0:
            x = f"!MG\r"
            self.get_logger().info("E-Stop removed")
            self.ser_1.write(x.encode('utf-8'))
            self.ser_2.write(x.encode('utf-8'))

def main(args=None):
    while True:
        rclpy.init(args=args)
        node = motor_driver(serial_port_1 = '/dev/left_controller', serial_port_2 = '/dev/right_controller')        # The serial port goes here
        rclpy.spin(node)

        node.destroy_node()
        rclpy.shutdown()

        time.sleep(2)


if __name__ == '__main__':
    main()
