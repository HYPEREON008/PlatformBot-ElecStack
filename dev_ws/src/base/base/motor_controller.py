import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8
import time

class motor_controller(Node):
    def __init__(self, serial_port_1, serial_port_2):
        super().__init__('motor_controller') ## name of the node
        self.mode = "0" ## mode 0 says that it is running in open loop
        self.serial_port_1 = serial_port_1
        self.serial_port_2 = serial_port_2

        self.ack_error_count = 0
        self.echo_error_count = 0
        self.ack_error_limit = 10
        self.echo_error_limit = 10

        self.ser_1 = serial.Serial(
            port=self.serial_port_1,
            baudrate=115200,
            parity=serial.PARITY_NONE, ## stands for no parity
            stopbits=1,
            bytesize=8,
            timeout=0.1
        )
        self.ser_2 = serial.Serial(
            port=self.serial_port_2,
            baudrate=115200,
            parity=serial.PARITY_NONE, ## stands for no parity
            stopbits=1,
            bytesize=8,
            timeout=0.1
        )

        ## opening the serial port
        for ser in [self.ser_1, self.ser_2]:
            for attempt in range(3):
                try:
                    if not ser.in_open:
                        self.get_logger().info(f"Attempting to open port: {ser.port} (Attempt {attempt + 1}/3)")
                        ser.open()
                    if ser.is_open:
                        self.get_logger().info(f"Successfully opened port: {ser.port}!")
                        break
                except serial.SerialException as e:
                    self.get_logger().error(f"Error opening port: {ser.port}: {e}")
                    if attempt < 2:
                        continue
                    else:
                        self.get_logger().error(f"Failed to open port: {ser.port} after 3 attempts. Shutting down.")
                        rclpy.shutdown()
        
        ## setmode and config commands to set the mode and save the config in the EEPROM of the motor controller

        for ser in [self.ser_1, self.ser_2]:
            command = f'^MMOD {str(self.mode)}\r'
            self.ser_write_w_error_check(command, ser)
            command = f'%EESAV\r'
            self.ser_write_w_error_check(command, ser)

        throttle_subscription_ = self.create_subscription(
            Float32MultiArray,
            'throttle',
            self.throttle_callback,
            10
        )
        e_stop_subscription_ = self.create_subscription(
            Int8,
            'e_stop',
            self.e_stop_callback,
            10
        )

        def serial_read(self, ser):
            data = ser.readline().decode('utf-8').strip()
            return data
    
        def set_velocity(self, velocity, ser):
            command = f'!G 1 {velocity}\r'
            ser_write_w_error_check(command, ser)

        def throttle_callback(self, msg):
            pass

        def e_stop_callback(self, msg):
            pass
        
        def error_limit_check(self, error):
            if self.ack_error_count >= self.ack_error_limit:
                self.get_logger().info(f"ACK error limit[{self.ack_error_limit}] reached. Stopping motor controller.")
                self.ser_1.close()
                self.ser_2.close()
                rclpy.shutdown()
            elif self.echo_error_count >= self.echo_error_limit:
                self.get_logger().info(f"Echo error limit[{self.echo_error_limit}] reached. Stopping motor controller.")
                self.ser_1.close()
                self.ser_2.close()
                rclpy.shutdown()

        def ser_write_w_error_check(self, command, ser):
            ser.write(command.encode('utf-8'))
            
            if (self.serial_read(ser) != command[:-1]):
                self.echo_error_count += 1
                self.get_logger().info(f"Echo error while: {command}")
                self.error_limit_check(self.echo_error_count)
            elif (self.serial_read(ser) != "ACK"):
                self.ack_error_count += 1
                self.get_logger().info(f"ACK error while: {command}")
                self.error_limit_check(self.ack_error_count)
            
                
def main(Args=None):
    rclpy.init(args=Args)
    motor_controller_node = motor_controller("/dev/ttyUSB0", "/dev/ttyUSB1")
    rclpy.spin(motor_controller_node)
    motor_controller_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

