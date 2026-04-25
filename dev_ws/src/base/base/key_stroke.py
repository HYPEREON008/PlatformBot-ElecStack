#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8
import evdev
from evdev import ecodes
import threading
import select
import sys

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
        
        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.max_rpm = 10.0       
        self.max_turn_rpm = 10.0  
        self.estop_active = False

        self.pressed_keys = set()
        
        # Evdev keycodes map directly to these string names
        self.valid_keys = {
            'KEY_W', 'KEY_A', 'KEY_S', 'KEY_D', 
            'KEY_UP', 'KEY_DOWN', 'KEY_LEFT', 'KEY_RIGHT', 
            'KEY_Z', 'KEY_E', 'KEY_R', 'KEY_P', 'KEY_ESC'
        }

        # 1. Hardware Setup: Find and grab the keyboard
        self.device = self.find_keyboard_device()
        if not self.device:
            self.get_logger().error("No keyboard found in /dev/input/. Did you add your user to the 'input' group and log out/in?")
            sys.exit(1)
            
        self.get_logger().info(f"Successfully connected to hardware: {self.device.name}")
        
        # Grabbing the device prevents keystrokes from leaking into the terminal
        # This completely fixes the ^[[A visual clutter problem!
        try:
            self.device.grab()
        except IOError:
            self.get_logger().error("Failed to grab device. Is another program using it?")
            sys.exit(1)

        # 2. Start the background thread to safely read hardware events
        self.running = True
        self.input_thread = threading.Thread(target=self._read_hardware_events)
        self.input_thread.daemon = True
        self.input_thread.start()

        # 3. Timer for continuous velocity publishing (10Hz)
        self.timer = self.create_timer(0.1, self.key_loop)

    def find_keyboard_device(self):
        """Scans Linux input devices to find the primary keyboard."""
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for dev in devices:
            caps = dev.capabilities()
            # If the device supports EV_KEY events and has the 'A' key registered, it's a keyboard
            if ecodes.EV_KEY in caps and ecodes.KEY_A in caps[ecodes.EV_KEY]:
                return dev
        return None

    def _read_hardware_events(self):
        """Background thread loop that reads physical electrical state changes."""
        while self.running:
            # select prevents read() from blocking permanently, allowing clean shutdown
            r, _, _ = select.select([self.device.fd], [], [], 0.1)
            if r:
                for event in self.device.read():
                    if event.type == ecodes.EV_KEY:
                        key_event = evdev.categorize(event)
                        keycode = key_event.keycode
                        
                        # Some hardware maps one electrical signal to multiple key strings. 
                        # We just take the primary one.
                        if isinstance(keycode, list):
                            keycode = keycode[0]

                        # keystate 1 = Key Down, 0 = Key Up (we ignore 2 = Key Hold Repeat)
                        if key_event.keystate == 1: 
                            self.on_press(keycode)
                        elif key_event.keystate == 0:
                            self.on_release(keycode)

    def on_press(self, keycode):
        """Logic executed exactly once when a key is pressed down."""
        # Check invalid key E-Stop condition
        if keycode not in self.valid_keys:
            self.get_logger().error(f"Unmapped key '{keycode}' pressed! E-STOP ACTIVATED.")
            self.estop_active = True
            msg = Int8()
            msg.data = 1
            self.e_stop_pub.publish(msg)
            return

        # Add to currently held keys set
        self.pressed_keys.add(keycode)

        # Handle instantaneous toggle keys
        if keycode == 'KEY_UP':
            self.max_rpm = min(100.0, self.max_rpm + 10.0)
            self.get_logger().info(f"Max Forward RPM increased to: {self.max_rpm}")
        elif keycode == 'KEY_DOWN':
            self.max_rpm = max(0.0, self.max_rpm - 10.0)
            self.get_logger().info(f"Max Forward RPM decreased to: {self.max_rpm}")
        elif keycode == 'KEY_RIGHT':
            self.max_turn_rpm = min(100.0, self.max_turn_rpm + 10.0)
            self.get_logger().info(f"Max Turn RPM increased to: {self.max_turn_rpm}")
        elif keycode == 'KEY_LEFT':
            self.max_turn_rpm = max(0.0, self.max_turn_rpm - 10.0)
            self.get_logger().info(f"Max Turn RPM decreased to: {self.max_turn_rpm}")

    def on_release(self, keycode):
        """Remove key from the set when physically released."""
        if keycode in self.pressed_keys:
            self.pressed_keys.remove(keycode)

    def key_loop(self):
        """Timer callback running at 10Hz to calculate and publish velocities."""
        
        # 1. Handle Single-Press Utility Keys
        if 'KEY_P' in self.pressed_keys:
            self.get_logger().info("P pressed. Shutting down without E-stop.")
            rclpy.shutdown()
            return
            
        if 'KEY_ESC' in self.pressed_keys or 'KEY_E' in self.pressed_keys:
            self.estop_active = True
            e_stop_msg = Int8()
            e_stop_msg.data = 1
            self.e_stop_pub.publish(e_stop_msg)
            if 'KEY_ESC' in self.pressed_keys:
                self.get_logger().warn("ESC pressed. Shutting down...")
                rclpy.shutdown()
                return
        
        if 'KEY_R' in self.pressed_keys:
            self.estop_active = False
            e_stop_msg = Int8()
            e_stop_msg.data = 0
            self.e_stop_pub.publish(e_stop_msg)
            self.get_logger().info("E-STOP Reset!")
            
        if 'KEY_Z' in self.pressed_keys:
            self.max_rpm = 0.0
            self.max_turn_rpm = 0.0
            self.get_logger().info("Max RPMs reset to 0.0")

        # 2. Block movement if E-Stop is active
        if self.estop_active:
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0]
            self.key_stroke.publish(msg)
            return

        # 3. Calculate Kinematics using physical hardware states
        self.left_velocity = 0.0
        self.right_velocity = 0.0

        if 'KEY_W' in self.pressed_keys:
            self.left_velocity += self.max_rpm
            self.right_velocity += self.max_rpm
        
        if 'KEY_S' in self.pressed_keys:
            self.left_velocity -= self.max_rpm
            self.right_velocity -= self.max_rpm
            
        if 'KEY_A' in self.pressed_keys:
            self.left_velocity -= self.max_turn_rpm
            self.right_velocity += self.max_turn_rpm
            
        if 'KEY_D' in self.pressed_keys:
            self.left_velocity += self.max_turn_rpm
            self.right_velocity -= self.max_turn_rpm

        self.left_velocity = max(-100.0, min(100.0, self.left_velocity))
        self.right_velocity = max(-100.0, min(100.0, self.right_velocity))

        # 4. Publish Velocities
        msg = Float32MultiArray()
        msg.data = [float(self.left_velocity), float(self.right_velocity)]
        self.key_stroke.publish(msg)

        if self.left_velocity != 0.0 or self.right_velocity != 0.0:
            self.get_logger().info(f"Sent Velocities [Left, Right]: {msg.data}")

    def destroy_node(self):
        """Cleanup hardware bindings gracefully on exit."""
        self.running = False
        if self.input_thread.is_alive():
            self.input_thread.join(timeout=1.0)
        try:
            self.device.ungrab()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = key_board_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        e_stop_msg = Int8()
        e_stop_msg.data = 1
        node.e_stop_pub.publish(e_stop_msg)
        node.get_logger().warn("Ctrl+C detected. E-Stop activated. Exiting.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()