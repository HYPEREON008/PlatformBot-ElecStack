#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8
from pynput import keyboard

class key_board_node(Node):
    def __init__(self):
        super().__init__("keyboard_control")
        self.get_logger().info("--- Key Controls ---")
        self.get_logger().info("Hold W/S : Move Forward/Backward at Max RPM")
        self.get_logger().info("Hold A/D : Turn Left/Right at Max Turn RPM")
        self.get_logger().info("Up/Down Arrows : Increase/Decrease Max RPM by 10")
        self.get_logger().info("Right/Left Arrows : Increase/Decrease Max Turn RPM by 10")
        self.get_logger().info("Press z to reset RPMs to 0")
        self.get_logger().warn("Press e to trigger e-stop | Press r to reset e-stop")
        self.get_logger().warn("Press esc to kill the node and activate e-stop")
        self.get_logger().warn("Press p to kill the node and NOT activate e-stop")
        self.get_logger().warn("Pressing any other unmapped key will initiate e-stop")

        self.key_stroke = self.create_publisher(Float32MultiArray, "keystroke", 10)
        self.e_stop_pub = self.create_publisher(Int8, 'estop', 10)
        
        # Timer for continuous velocity publishing (10Hz)
        self.timer = self.create_timer(0.1, self.key_loop)
        
        # Variables to track velocities and max RPM states
        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.max_rpm = 10.0       # Default starting RPM
        self.max_turn_rpm = 10.0  # Default starting Turn RPM

        # Set to hold all keys currently being held down
        self.pressed_keys = set()
        
        # Define allowed keys to trigger the "any other key e-stop" logic safely
        self.valid_keys = {'w', 'a', 's', 'd', 'e', 'r', 'z', 'p', 'up', 'down', 'left', 'right'}

        # Start the pynput keyboard listener in a non-blocking background thread
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        """Callback for when a key is pressed down."""
        # 1. Track standard held keys
        try:
            # Force lowercase so Shift/CapsLock doesn't trigger the unmapped key E-Stop
            char = key.char.lower()
            self.pressed_keys.add(char)
        except AttributeError:
            # Handle special keys (arrows, esc, etc.)
            self.pressed_keys.add(key.name)

        # 2. Handle discrete toggle inputs (Arrow keys) instantly on press
        if key == keyboard.Key.up:
            self.max_rpm += 10.0
            self.get_logger().info(f"Max Forward RPM increased to: {self.max_rpm}")
        elif key == keyboard.Key.down:
            self.max_rpm -= 10.0
            self.get_logger().info(f"Max Forward RPM decreased to: {self.max_rpm}")
        elif key == keyboard.Key.right:
            self.max_turn_rpm += 10.0
            self.get_logger().info(f"Max Turn RPM increased to: {self.max_turn_rpm}")
        elif key == keyboard.Key.left:
            self.max_turn_rpm -= 10.0
            self.get_logger().info(f"Max Turn RPM decreased to: {self.max_turn_rpm}")

    def on_release(self, key):
        """Callback for when a key is released."""
        try:
            char = key.char.lower()
            if char in self.pressed_keys:
                self.pressed_keys.remove(char)
        except AttributeError:
            if key.name in self.pressed_keys:
                self.pressed_keys.remove(key.name)
        
        # Exit condition: Stop the listener and node if 'Esc' is pressed
        if key == keyboard.Key.esc:
            e_stop_msg = Int8()
            e_stop_msg.data = 1
            self.e_stop_pub.publish(e_stop_msg)
            self.get_logger().warn("ESC pressed. E-Stop activated. Shutting down...")
            rclpy.shutdown()
            return False

    def key_loop(self):
        """Timer callback running at 10Hz to calculate and publish velocities."""
        msg = Float32MultiArray()
        
        # 1. Handle Single-Press Utility Keys
        if "p" in self.pressed_keys:
            self.get_logger().info("P pressed. Shutting down without E-stop.")
            rclpy.shutdown()
            return
            
        if "e" in self.pressed_keys:
            e_stop_msg = Int8()
            e_stop_msg.data = 1
            self.e_stop_pub.publish(e_stop_msg)
            self.get_logger().warn("E-STOP Activated manually!")
        
        elif "r" in self.pressed_keys:
            e_stop_msg = Int8()
            e_stop_msg.data = 0
            self.e_stop_pub.publish(e_stop_msg)
            self.get_logger().info("E-STOP Reset!")
            
        if "z" in self.pressed_keys:
            self.max_rpm = 0.0
            self.max_turn_rpm = 0.0
            self.get_logger().info("Max RPMs reset to 0.0")

        # 2. Check for invalid keys to trigger "Any other key e-stop"
        invalid_keys = self.pressed_keys - self.valid_keys
        if invalid_keys:
            e_stop_msg = Int8()
            e_stop_msg.data = 1
            self.e_stop_pub.publish(e_stop_msg)
            self.get_logger().error(f"Unmapped key(s) {invalid_keys} pressed! E-STOP ACTIVATED.")
            # Force velocity to 0 if an invalid key triggers estop
            self.left_velocity = 0.0
            self.right_velocity = 0.0
            msg.data = [self.left_velocity, self.right_velocity]
            self.key_stroke.publish(msg)
            return

        # 3. Calculate Kinematics using Superposition (Allows W+A combinations)
        self.left_velocity = 0.0
        self.right_velocity = 0.0

        if "w" in self.pressed_keys:
            self.left_velocity += self.max_rpm
            self.right_velocity += self.max_rpm
        
        if "s" in self.pressed_keys:
            self.left_velocity -= self.max_rpm
            self.right_velocity -= self.max_rpm
            
        if "a" in self.pressed_keys:
            self.left_velocity -= self.max_turn_rpm
            self.right_velocity += self.max_turn_rpm
            
        if "d" in self.pressed_keys:
            self.left_velocity += self.max_turn_rpm
            self.right_velocity -= self.max_turn_rpm

        # 4. Publish Velocities
        msg.data = [float(self.left_velocity), float(self.right_velocity)]
        self.key_stroke.publish(msg)

        # Log the published data only if keys are actively pressed to reduce log spam
        if self.pressed_keys:
            self.get_logger().info(f"Sent Velocities [Left, Right]: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = key_board_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle standard Ctrl+C safely to trigger E-Stop
        e_stop_msg = Int8()
        e_stop_msg.data = 1
        node.e_stop_pub.publish(e_stop_msg)
        node.get_logger().warn("Ctrl+C detected. E-Stop activated. Exiting.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()