import evdev
import can
import sys
import time
import threading
import struct
import math
import socket
import dataclasses
import collections

CONTROL_LOOP_PERIOD = 1/200
WHEEL_PERIMETER = 52.42 * math.pi
WHEEL_SPACING = 258.5

CANID_MOTOR_ALIVE = 0x100
CANID_MOTOR_SETPOINT = 0x101
CANID_MOTOR_STATUS = 0x102
CANID_MOTOR_SETPOINT_ERROR = 0x103
CANID_MOTOR_RESET_SETPOINT_ERROR = 0x104

CANID_RASPI_ALIVE = 0x200

bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitrate=500000)

teleplot_addr = ("192.168.0.10", 47269)
teleplot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_telemetry(name, value):
    now = time.time() * 1000
    msg = name+":"+str(now)+":"+str(value)
    teleplot_sock.sendto(msg.encode(), teleplot_addr)

def get_controller_device():
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for device in devices:
        print(device)
        if "8BitDo" in device.name:
            print(device)
            return device

    print("no controller found")
    sys.exit(1)

def can_alive_thread():
    while True:
        msg = can.Message(arbitration_id=CANID_RASPI_ALIVE)
        bus.send(msg)
        time.sleep(1)

@dataclasses.dataclass
class ControllerUpdate:
    x: float
    y: float
    rx: float
    ry: float
    z: float
    rz: float
    keys_active: list[int]
    keys_pressed: list[int]
    keys_released: list[int]

class Controller:
    def __init__(self):
        self.last_keys_active = []
        self.device = None

    def init(self, device_name: str):
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            print(device)
            if device_name in device.name:
                print("Found controller:", device)
                self.device = device
                return True

        print("No controller found")
        return False
    
    def update(self):
        keys_active = self.device.active_keys()
        keys_pressed = []
        keys_released = []
        # BTN_A BTN_B BTN_X BTN_Y
        # BTN_TR BTN_TR
        # BTN_THUMBL BTN_THUMBR
        # BTN_SELECT BTN_START
        # BTN_DPAD_UP BTN_DPAD_DOWN BTN_DPAD_RIGHT BTN_DPAD_LEFT

        # dpad is an axis for some reason, append it to keys_active
        x = self.device.absinfo(evdev.ecodes.ABS_HAT0X) # dpad 
        y = self.device.absinfo(evdev.ecodes.ABS_HAT0Y) # dpad
        if x.value == 1:
            keys_active.append(evdev.ecodes.BTN_DPAD_RIGHT)
        elif x.value == -1:
            keys_active.append(evdev.ecodes.BTN_DPAD_LEFT)
        if y.value == 1:
            keys_active.append(evdev.ecodes.BTN_DPAD_DOWN)
        elif y.value == -1:
            keys_active.append(evdev.ecodes.BTN_DPAD_UP)

        for k in keys_active:
            if k not in self.last_keys_active:
                keys_pressed.append(k)
        for k in self.last_keys_active:
            if k not in keys_active:
                keys_released.append(k)

        x = self.device.absinfo(evdev.ecodes.ABS_X) # joystick gauche horizontal
        y = self.device.absinfo(evdev.ecodes.ABS_Y) # joystick gauche vertical
        rx = self.device.absinfo(evdev.ecodes.ABS_RX) # joystick droite horizontal
        ry = self.device.absinfo(evdev.ecodes.ABS_RY) # joystick droite vertical
        z = self.device.absinfo(evdev.ecodes.ABS_Z) # gachette gauche
        rz = self.device.absinfo(evdev.ecodes.ABS_RZ) # gachette droite

        self.last_keys_active = keys_active

        return ControllerUpdate(
            x = x.value/x.max,
            y = y.value/y.max,
            rx = rx.value/rx.max,
            ry = ry.value/ry.max,
            z = z.value/z.max,
            rz = rz.value/rz.max,
            keys_active = keys_active,
            keys_pressed = keys_pressed,
            keys_released = keys_released,
        )

class PID:
    def __init__(self, kp: float, ki: float, kd: float, integrator_max: float = 10000):
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd
        self.integrator_max: float = integrator_max

        self.last_error: float = 0
        self.integrator: float = 0
    
    def update(self, error: float) -> float:
        self.integrator += error

        if self.integrator > self.integrator_max:
                self.integrator = self.integrator_max
        elif self.integrator < -self.integrator_max:
            self.integrator = -self.integrator_max

        output = self.kp * error
        output += self.ki * self.integrator
        output += self.kd * (error - self.last_error)

        self.last_error = error

        return output

class MovingAverageFilter:
    def __init__(self, window_size: int = 2):
        self.window_size = window_size
        self.values = collections.deque()
        self.values.append(0)
        self.sum = 0
    
    def update(self, value: int):
        self.values.append(value)
        self.sum += value

        if len(self.values) > self.window_size:
            removed = self.values.popleft()
            self.sum -= removed
        
    def get(self) -> float:
        return self.sum / len(self.values)

class Odometry:
    def __init__(self, control_loop_period: float, wheel_perimeter: float, ticks_per_rev: int, wheel_spacing: float):
        """
        :param control_loop_period: in seconds
        :param wheel_perimeter: odometry wheel perimeter, in mm
        :param ticks_per_rev: in ticks
        :param wheel_spacing: spacing between the two odometry wheels, in mm
        """
        self.control_loop_period = control_loop_period
        self.wheel_spacing = wheel_spacing
        self.k_wheel = wheel_perimeter / ticks_per_rev

        self.x_mm = 0.0
        self.y_mm = 0.0
        self.theta_rad = 0.0
        self.distance_mm = 0.0

        self.vel_dist_filter = MovingAverageFilter(5)
        self.vel_theta_filter = MovingAverageFilter(5)

        self.last_ticks_left = 0
        self.last_ticks_right = 0
    
    def update(self, ticks_left: int, ticks_right: int):
        delta_left = (ticks_left - self.last_ticks_left) * self.k_wheel
        delta_right = (ticks_right - self.last_ticks_right) * self.k_wheel

        delta_distance = (delta_right + delta_left) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheel_spacing

        self.distance_mm += delta_distance
        self.theta_rad += delta_theta

        self.vel_dist_filter.update(delta_distance / self.control_loop_period)
        self.vel_theta_filter.update(delta_theta / self.control_loop_period)

        self.x_mm += delta_distance * math.cos(self.theta_rad)
        self.y_mm += delta_distance * math.sin(self.theta_rad)

        self.last_ticks_left = ticks_left
        self.last_ticks_right = ticks_right

    def get_x(self) -> float:
        return self.x_mm

    def get_y(self) -> float:
        return self.y_mm

    def get_theta(self) -> float:
        return self.theta_rad * 180.0 / math.pi
    
    def get_velocity_theta(self) -> float:
        return self.vel_theta_filter.get() * 180.0 / math.pi

    def get_distance(self) -> float:
        return self.distance_mm

    def get_velocity_distance(self) -> float:
        return self.vel_dist_filter.get()

class ThresholdFilter:
    def __init__(self, threshold: int):
        self.threshold = threshold
        self.filtered_value: int = 0

    def update(self, value: int):
        if abs(value - self.filtered_value) > self.threshold:
            self.filtered_value = value
    
    def get(self):
        return self.filtered_value

class HallEncoder:
    TICKS_PER_REVOLUTION = 16384
    TICKS_PER_REVOLUTION_FIRST_QUARTER = TICKS_PER_REVOLUTION / 4
    TICKS_PER_REVOLUTION_LAST_QUARTER = 3 * TICKS_PER_REVOLUTION / 4

    def __init__(self):
        self.last_reading_ticks: int = 0
        self.total_ticks: int = 0
        self.lock = threading.Lock()
        self.first_measure = True
        self.filter = ThresholdFilter(3)
    
    def update(self, current_reading_ticks: int):
        delta = current_reading_ticks - self.last_reading_ticks

        # handle encoder rollover/rolldown
        if self.last_reading_ticks < self.TICKS_PER_REVOLUTION_FIRST_QUARTER and current_reading_ticks > self.TICKS_PER_REVOLUTION_LAST_QUARTER:
            delta -= self.TICKS_PER_REVOLUTION
        elif self.last_reading_ticks > self.TICKS_PER_REVOLUTION_LAST_QUARTER and current_reading_ticks < self.TICKS_PER_REVOLUTION_FIRST_QUARTER:
            delta += self.TICKS_PER_REVOLUTION
        
        self.last_reading_ticks = current_reading_ticks

        if self.first_measure:
            self.first_measure = False
            return 0

        with self.lock:
            self.total_ticks += delta
            self.filter.update(self.total_ticks)

        return delta

    def get(self):
        return self.filter.get()

class RampFilter:
    def __init__(self, control_loop_period: float, accel_rate: float, decel_rate: float):
        self.accel_rate = accel_rate * control_loop_period
        self.decel_rate = decel_rate * control_loop_period
        self.current_value = 0.0

    def update(self, target: float):
        delta = target - self.current_value

        if delta == 0:
            return self.current_value

        direction = 1 if delta > 0 else -1

        if (self.current_value * direction) < 0:
            change = self.decel_rate * direction
        elif abs(delta) > self.accel_rate:
            if (self.current_value < target and direction > 0) or (self.current_value > target and direction < 0):
                change = self.accel_rate * direction
            else:
                change = self.decel_rate * direction
        else:
            self.current_value = target
            return self.current_value

        self.current_value += change

        if (direction > 0 and self.current_value > target) or (direction < 0 and self.current_value < target):
            self.current_value = target

        return self.current_value

encoder_left = HallEncoder()
encoder_right = HallEncoder()
odometry = Odometry(CONTROL_LOOP_PERIOD, WHEEL_PERIMETER, HallEncoder.TICKS_PER_REVOLUTION, WHEEL_SPACING)
# pid_dist = 30, 1.7, 0
pid_dist = PID(30, 1.7, 0, integrator_max=8000)
# pid_theta = 60, 4.0, 0
pid_theta = PID(60, 4.0, 0, integrator_max=3000)
limit_ramp_theta = RampFilter(CONTROL_LOOP_PERIOD, 1080, 1080)
limit_ramp_dist = RampFilter(CONTROL_LOOP_PERIOD, 1000, 2100)

con = Controller()
if not con.init("8BitDo"):
    quit()

def can_read_thread():
    while True:
        for msg in bus:
            if msg.arbitration_id == CANID_MOTOR_ALIVE:
                error = struct.unpack(">?", msg.data)
                print("[CAN] MotorBoard IsAlive error:", error)
            
            elif msg.arbitration_id == CANID_MOTOR_STATUS:
                enc_left, enc_right, timeout_remaining = struct.unpack(">HHh", msg.data)

                encoder_left.update(enc_left)
                encoder_right.update(enc_right)
                odometry.update(encoder_left.get(), -encoder_right.get())

                send_telemetry("vel_theta", odometry.get_velocity_theta())
                send_telemetry("vel_dist", odometry.get_velocity_distance())
                send_telemetry("timeout_remaining", timeout_remaining)
                # send_telemetry("dist_integrator", pid_dist.integrator)
                # send_telemetry("theta_integrator", pid_theta.integrator)
            
            elif msg.arbitration_id == CANID_MOTOR_SETPOINT_ERROR:
                timeout_remaining = struct.unpack(">i", msg.data)

                print("SETPOINT ERROR! timeout_remaining:", timeout_remaining)

def main():
    dev = get_controller_device()

    t_can_alive = threading.Thread(target=can_alive_thread)
    t_can_alive.start()

    t_can_read = threading.Thread(target=can_read_thread)
    t_can_read.start()

    while True:
        joy = con.update()

        if evdev.ecodes.BTN_SELECT in joy.keys_pressed:
            print("sleeping a bit to trigger motorboard error")
            time.sleep(0.005)
        if evdev.ecodes.BTN_START in joy.keys_pressed:
            msg = can.Message(arbitration_id=CANID_MOTOR_RESET_SETPOINT_ERROR)
            bus.send(msg)
            print("sent error reset")
        
        joy_theta = -joy.x * 90
        joy_speed = (joy.rz - joy.z) * 200

        # boost mode
        if evdev.ecodes.BTN_Y in joy.keys_active:
            joy_theta = -joy.x * 90
            joy_speed = (joy.rz - joy.z) * 600

        # joy_theta = limit_ramp_theta.update(joytheta)
        # joy_speed = limit_ramp_dist.update(joy_speed)

        err_theta = joy_theta - odometry.get_velocity_theta()
        err_speed = joy_speed - odometry.get_velocity_distance()

        pidtheta = pid_theta.update(err_theta)
        pidspeed = pid_dist.update(err_speed)

        pwm_right = pidspeed + pidtheta
        pwm_left = -pidspeed + pidtheta

        pwm_right = int(pwm_right)
        pwm_left = int(pwm_left)

        # clamp values to signed 16 bit integer
        if pwm_right > 32767:
            pwm_right = 32767;
        elif pwm_right < -32768:
            pwm_right = -32768;
        
        if pwm_left > 32767:
            pwm_left = 32767;
        elif pwm_left < -32768:
            pwm_left = -32768;

        data = bytearray(struct.pack(">hh", pwm_right, pwm_left))
        msg = can.Message(arbitration_id=CANID_MOTOR_SETPOINT, data=data)
        bus.send(msg)

        # TODO: this code should be execute each 5ms, not 5ms sleep between each execution
        time.sleep(CONTROL_LOOP_PERIOD)

if __name__ == "__main__":
    main()
