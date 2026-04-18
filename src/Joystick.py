from evdev import InputDevice, categorize, ecodes, list_devices
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from enum import Enum
from time import sleep

class Kreo(Enum):
    # Top Buttonsyh
    R1 = 311
    L1 = 310

    # Right hand buttons
    pyramid = 307
    cylinder = 305
    cube = 308
    plus = 304

    #Left hand buttons
    upDown = 17
    leftRight = 16

    #other buttons
    power = 316
    share = 314
    options = 315
    backleft = 304
    backright = 305

    #Hall Triggers max value
    HALL_TRIG_MAX = 255

    #Thumbstick Max
    THUMBSTICK_MAX = 255


class Gear(Enum):
    REVERSE = -1.0
    LOW = 1.0
    MEDIUM = 1.5
    HIGH = 2.0
    TURBO = 3.0   # 👈 add more here easily

    @classmethod
    def list_gears(cls):
        return list(cls)


def select_device(name_hint=None):
    devices = [InputDevice(path) for path in list_devices()]
    print("Available input devices:")
    for idx, dev in enumerate(devices):
        print(f"{idx}: {dev.path} ({dev.name})")

    if name_hint:
        for dev in devices:
            if name_hint in dev.name:
                print(f"Selected {dev.path} ({dev.name})")
                return InputDevice(dev.path)

    choice = int(input("Select device number: "))
    dev = devices[choice]
    print(f"Selected {dev.path} ({dev.name})")
    return dev


class Joystick(Node):

    def _init_(self, controller_hint=None):
        super()._init_('joystick')

        # Publishers
        self.vel_setpoint_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.pub_Estop = self.create_publisher(Int8, 'estop', 10)

        # Twist and estop messages
        self.vels = Twist()
        self.reset_vels()
        self.estop = Int8()
        self.estop.data = 1

        # Params
        self.declare_parameter('controller', controller_hint or "")
        self.controllerName = self.get_parameter('controller').get_parameter_value().string_value

        # Controller selection
        self.dev = select_device(self.controllerName)

        # States
        self.brake_status = 1
        self.speed = 0
        self.gear = Gear.LOW 

        # Track previous thumbstick/trigger values
        self.LTX_old = (Kreo.THUMBSTICK_MAX.value+1)//2
        self.RT_old = 0

        # Input loop
        try:
            for event in self.dev.read_loop():
                # --- handle key press ---
                if (event.type == ecodes.EV_KEY and event.value == 1):
                    self.handle_key(event)

                # --- handle analog inputs ---
                elif (event.type == ecodes.EV_ABS):
                    self.handle_abs(event)

        except OSError:
            self.reset_vels()
            self.vel_setpoint_pub.publish(self.vels)
            print("Controller disconnected!")

    def reset_vels(self):
        self.vels.linear.x = 0.0
        self.vels.linear.y = 0.0
        self.vels.linear.z = 0.0
        self.vels.angular.x = 0.0
        self.vels.angular.y = 0.0
        self.vels.angular.z = 0.0

    def estopEngage(self):
        self.reset_vels()
        self.vel_setpoint_pub.publish(self.vels)
        self.estop.data = 1
        self.pub_Estop.publish(self.estop)
        self.estop.data = 0
        print("Estop engaged. Press Options/Share to exit.")

    def estopDisengage(self):
        self.pub_Estop.publish(self.estop)
        self.estop.data = 1
        print("Estop disengaged.")

    def upshift(self):
        gears = Gear.list_gears()
        idx = gears.index(self.gear)
        if idx < len(gears) - 1:
            self.gear = gears[idx + 1]
            print(f"Upshifted to {self.gear.name} gear (limit = {self.gear.value} m/s)")
        else:
            print("Already in highest gear")

    def downshift(self):
        gears = Gear.list_gears()
        idx = gears.index(self.gear)
        if idx > 0:
            self.gear = gears[idx - 1]
            print(f"Downshifted to {self.gear.name} gear (limit = {self.gear.value} m/s)")
        else:
            print("Already in lowest gear")

    def trigger_to_velocity(self, raw_value, gear_max_speed, deadzone=10, smooth_fraction=0.2):
        norm = max(0, raw_value - deadzone) / (Kreo.HALL_TRIG_MAX.value - deadzone)
        norm = min(norm, 1.0)
        if norm < smooth_fraction:
            progress = norm / smooth_fraction
            velocity = gear_max_speed * (progress**2) * smooth_fraction
        else:
            progress = (norm - smooth_fraction) / (1 - smooth_fraction)
            velocity = gear_max_speed * (smooth_fraction + progress * (1 - smooth_fraction))
        return velocity

    def handle_key(self, event):
        if (event.code in (Kreo.pyramid.value, Kreo.cylinder.value, Kreo.plus.value, Kreo.cube.value)):
            self.estopEngage()
        elif (event.code == Kreo.L1.value):
            self.downshift()
        elif (event.code == Kreo.R1.value):
            self.upshift()
        elif (event.code in (Kreo.share.value, Kreo.options.value)) and not self.estop.data:
            self.estopDisengage()

    def handle_abs(self, event):
        NEUTRAL = (Kreo.THUMBSTICK_MAX.value+1)//2
        MAX_VAL = Kreo.THUMBSTICK_MAX.value

        # Left Thumbstick X
        if event.code == 0:
            if abs(event.value - NEUTRAL) > 5 and self.brake_status:
                LeftThumbstickX = event.value
                if self.LTX_old != LeftThumbstickX:
                    centered_x = LeftThumbstickX - NEUTRAL
                    self.vels.angular.z = -float(centered_x * pow(abs(centered_x), 0.8) * 0.015625 / 8)
                    print("Setpoint: V =", self.vels.linear.x, "W =", self.vels.angular.z, LeftThumbstickX)
                    self.vel_setpoint_pub.publish(self.vels)
                self.LTX_old = LeftThumbstickX
            else:
                LeftThumbstickX = NEUTRAL
                if self.LTX_old != LeftThumbstickX:
                    self.vels.angular.z = 0.0
                    print("Setpoint: V =", self.vels.linear.x, "W =", self.vels.angular.z)
                    self.vel_setpoint_pub.publish(self.vels)
                self.LTX_old = LeftThumbstickX

        # Left Trigger = brake
        elif event.code == 2:
            if event.value > Kreo.HALL_TRIG_MAX.value/2 and self.brake_status:
                self.reset_vels()
                self.vel_setpoint_pub.publish(self.vels)
                self.brake_status = 0
                print("Brakes engaged.", event.value)
            elif event.value <= Kreo.HALL_TRIG_MAX.value/2 and not self.brake_status:
                self.brake_status = 1
                print("Brakes disengaged.", event.value)

        # Right Trigger = throttle
        elif event.code == 5:
            if event.value not in range(0, 10) and self.brake_status:
                velocity = self.trigger_to_velocity(event.value, gear_max_speed=self.gear.value)
                if self.RT_old != velocity:
                    self.vels.linear.x = velocity
                    print("Setpoint: V =", self.vels.linear.x, "W =", self.vels.angular.z, event.value)
                    self.vel_setpoint_pub.publish(self.vels)
                self.RT_old = velocity
            else:
                self.vels.linear.x = 0.0
                self.vel_setpoint_pub.publish(self.vels)


def main():
    rclpy.init()
    js = Joystick("Xbox Wireless Controller")
    rclpy.spin(js)
    js.destroy_node()
    rclpy.shutdown()

if _name_ == "_main_":
    main()
