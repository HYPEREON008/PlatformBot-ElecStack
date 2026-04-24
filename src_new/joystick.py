# 
# CONTROLLER MANUAL:
#
# 1. SAFETY & E-STOP:
#    - STARTUP: The bot boots in LOCKED mode (E-Stop Engaged).
#    - UNLOCK: Press 'OPTIONS' button to enable movement.
#    - PANIC STOP: Press ANY Face Button (Plus, Cube, Pyramid, Cylinder) to 
#      instantly cut all velocity to 0.0 and lock the bot.
#
# 2. DRIVE CONTROLS (Split Arcade):
#    - LEFT STICK (Up/Down): Linear Velocity (Forward / Reverse).
#    - RIGHT STICK (Left/Right): Angular Velocity (Turn Left / Turn Right).
#
# 3. GEAR SHIFTING (Speed Limits):
#    - R1 (Right Bumper): Shift UP (Increase max speed).
#    - L1 (Left Bumper): Shift DOWN (Decrease max speed).
#    - GEARS: 
#        - LOW:    0.5 m/s 
#        - MEDIUM: 1.0 m/s
#        - HIGH:   1.5 m/s
#        - TURBO:  2.0 m/s 
#
# 4. TOPICS:
#    - Velocity: /cmd_vel_nav (geometry_msgs/Twist)
#    - E-Stop:   /estop (std_msgs/Int8)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8  
import evdev
from evdev import ecodes
from enum import Enum

class Gear(Enum):
    # Max speed and its gears
    LOW = 0.5
    MEDIUM = 1.0
    HIGH = 1.5
    TURBO = 2.0

class Action(Enum):
    # identifier strings for buttons
    SHIFT_UP = "shift_up"
    SHIFT_DOWN = "shift_down"
    ESTOP_ENGAGE = "estop_engage"
    ESTOP_DISENGAGE = "estop_disengage"

BUTTON_MAPPING = {
    311: Action.SHIFT_UP,          # R1 Bumper
    310: Action.SHIFT_DOWN,        # L1 Bumper
    304: Action.ESTOP_ENGAGE,      # Face Button (Plus)
    305: Action.ESTOP_ENGAGE,      # Face Button (Cylinder)
    307: Action.ESTOP_ENGAGE,      # Face Button (Pyramid)
    308: Action.ESTOP_ENGAGE,      # Face Button (Cube)
    315: Action.ESTOP_DISENGAGE    # Options Button
}

def get_controller(keyword="Xbox"):
    #Scans for device and connects with it
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for dev in devices:
        if keyword.lower() in dev.name.lower():
            print(f"Successfully connected to: {dev.name} at {dev.path}")
            return dev
            
    print(f"Could not find a controller with '{keyword}' in the name.")
    return None

def normalize_axis(raw_value, center=128, max_val=255, deadzone=0.15):
    # Step 1: Center the value (0 to 255 becomes -128 to +127)
    centered = raw_value - center
    
    # Step 2: Normalize to a decimal between -1.0 and 1.0
    normalized = centered / (max_val / 2.0)
    
    # Step 3: Apply the Deadzone
    if abs(normalized) < deadzone:
        return 0.0
        
    return normalized

class JoystickNode(Node):
    def __init__(self):
        super().__init__('kreon_joystick_node')

        # 1. Publishers
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.estop_pub = self.create_publisher(Int8, 'estop', 10)

        # 2. State Variables
        self.vel_msg = Twist()
        self.current_gear = Gear.LOW
        self.is_estopped = True  # Always start safely (E-Stopped)

        # 3. Hardware Setup
        self.dev = get_controller("Wireless Controller") # Change to your controller's keyword

        # 4. Map the Actions to actual class methods
        self.action_methods = {
            Action.SHIFT_UP: self.shift_up,
            Action.SHIFT_DOWN: self.shift_down,
            Action.ESTOP_ENGAGE: self.engage_estop,
            Action.ESTOP_DISENGAGE: self.disengage_estop
        }

        # 5. The Non-Blocking Event Loop (Runs 100 times per second)
        timer_period = 0.01 
        self.timer = self.create_timer(timer_period, self.process_controller_events)

        self.get_logger().info("Joystick Node Initialized. E-Stop is ENGAGED by default.")
        self.publish_estop()
    
    def process_controller_events(self):
        #reading the inputs
        if self.dev is None:
            return

        try: 
            # If nothing happened, it throws a BlockingIOError (which we ignore).
            for event in self.dev.read():
                if event.type == ecodes.EV_KEY:
                    self.handle_button(event)
                elif event.type == ecodes.EV_ABS:
                    self.handle_axis(event)
                    
        except BlockingIOError:
            pass  # No new inputs rn
        except OSError:
            self.get_logger().error("Controller disconnected abruptly!")
            self.dev = None
            self.engage_estop()

    def handle_button(self, event):
        #For button presses
        # event.value == 1 is a press down. 0 is a release. We only care about presses.
        if event.value == 1 and event.code in BUTTON_MAPPING:
            action_name = BUTTON_MAPPING[event.code]
            
            # Execute the method linked to this action
            if action_name in self.action_methods:
                self.action_methods[action_name]()

    def handle_axis(self, event):
        #for joystick movements
        #Left Stick Y-Axis (Forward/Reverse)
        if event.code == evdev.ecodes.ABS_Y:
            multiplier = normalize_axis(event.value)
            # Invert so pushing UP yields positive linear velocity
            self.vel_msg.linear.x = -1.0 * multiplier * self.current_gear.value
            self.publish_velocity()

        #Right Stick X-Axis (Steering)
        elif event.code == evdev.ecodes.ABS_RX:
            multiplier = normalize_axis(event.value)
            # Invert so pushing RIGHT yields negative angular velocity (standard ROS format)
            self.vel_msg.angular.z = -1.0 * multiplier * self.current_gear.value
            self.publish_velocity()

    def publish_velocity(self):
        #for publishing velocity
        if self.is_estopped:
            # If estopped, force zero velocity regardless of stick position
            safe_vel = Twist()
            self.vel_pub.publish(safe_vel)
        else:
            # Publish actual stick position
            self.vel_pub.publish(self.vel_msg)
            self.get_logger().info(f"{self.vel_msg} m/s")
    
    def publish_estop(self):
        msg = Int8()
        msg.data = 1 if self.is_estopped else 0
        self.estop_pub.publish(msg)

    def engage_estop(self):
        self.is_estopped = True
        self.publish_estop()
        self.publish_velocity() # Instantly send 0,0 velocity
        self.get_logger().warn("E-STOP engaged. Press option button to exit")

    def disengage_estop(self):
        self.is_estopped = False
        self.publish_estop()
        self.get_logger().info("E-Stop Disengaged. Ready to drive")

    def shift_up(self):
        gears = list(Gear)
        current_idx = gears.index(self.current_gear)
        if current_idx < len(gears) - 1:
            self.current_gear = gears[current_idx + 1]
            self.get_logger().info(f"Upshifted to {self.current_gear.name} (limit = {self.current_gear.value} m/s)")
        else:
            self.get_logger().warn("Already in highest gear")

    def shift_down(self):
        gears = list(Gear)
        current_idx = gears.index(self.current_gear)
        if current_idx > 0:
            self.current_gear = gears[current_idx - 1]
            self.get_logger().info(f"Downshifted to {self.current_gear.name} (limit = {self.current_gear.value} m/s)")
        else:
            self.get_logger().warn("Already in lowest gear")

    def shutdown_safely(self):
        #called when bot is turned off
        self.engage_estop()
        if self.dev:
            self.dev.close()

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down...")
    finally:
        node.shutdown_safely()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()