import evdev
import can
import sys
import time
import threading
import struct
import math
import socket

CONTROL_LOOP_PERIOD = 1/200
WHEEL_PERIMETER = 52.42 * math.pi
WHEEL_SPACING = 258.5

CANID_MOTOR_ALIVE = 0x100
CANID_MOTOR_SETPOINT = 0x101
CANID_MOTOR_SETPOINT_RESPONSE = 0x102
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

class PID:
    def __init__(self, kp, ki, kd):
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd

        self.last_error: float = 0
        self.integrator: float = 0
    
    def process(self, error: float) -> float:
        self.integrator += error

        output = self.kp * error
        output += self.ki * self.integrator
        output += self.kd * (error - self.last_error)

        self.last_error = error

        return output

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
        self.vel_distance_mm = 0.0
        self.vel_theta_rad = 0.0

        self.last_ticks_left = 0
        self.last_ticks_right = 0
    
    def update(self, ticks_left: int, ticks_right: int):
        delta_left: float = (ticks_left - self.last_ticks_left) * self.k_wheel
        delta_right: float = (ticks_right - self.last_ticks_right) * self.k_wheel

        delta_distance: float = (delta_right + delta_left) / 2.0
        delta_theta: float = (delta_right - delta_left) / self.wheel_spacing

        self.distance_mm += delta_distance
        self.theta_rad += delta_theta

        self.vel_distance_mm = delta_distance / self.control_loop_period
        self.vel_theta_rad = delta_theta / self.control_loop_period

        self.x_mm += delta_distance * math.cos(self.theta_rad)
        self.y_mm += delta_distance * math.sin(self.theta_rad)

        self.last_ticks_left = ticks_left
        self.last_ticks_right = ticks_right

    def get_x(self):
        return self.x_mm

    def get_y(self):
        return self.y_mm
    
    def get_theta_rad(self):
        return self.theta_rad

    def get_theta_deg(self):
        return self.theta_rad * 180.0 / math.pi
    
    def get_velocity_theta(self):
        """
        :returns: theta velocity in deg/s
        """
        return self.vel_theta_rad * 180.0 / math.pi

    def get_distance(self):
        return self.distance_mm

    def get_velocity_distance(self):
        return self.vel_distance_mm

class HallEncoder:
    TICKS_PER_REVOLUTION = 16384
    TICKS_PER_REVOLUTION_FIRST_QUARTER = TICKS_PER_REVOLUTION / 4
    TICKS_PER_REVOLUTION_LAST_QUARTER = 3 * TICKS_PER_REVOLUTION / 4

    def __init__(self):
        self.last_reading_ticks: int = 0
        self.total_ticks: int = 0
        self.lock = threading.Lock()
        self.first_measure = True
    
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

        return delta

    def get(self):
        with self.lock:
            return self.total_ticks

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

encoder1 = HallEncoder()
encoder2 = HallEncoder()
odometry = Odometry(CONTROL_LOOP_PERIOD, WHEEL_PERIMETER, HallEncoder.TICKS_PER_REVOLUTION, WHEEL_SPACING)
pid_dist = PID(0.45, 0.01, 0)
pid_theta = PID(1.5, 0.0, 0)
limit_ramp_theta = RampFilter(CONTROL_LOOP_PERIOD, 1080, 1080)
limit_ramp_dist = RampFilter(CONTROL_LOOP_PERIOD, 1000, 2500)

def can_read_thread():
    while True:
        for msg in bus:
            if msg.arbitration_id == CANID_MOTOR_ALIVE:
                print("[CAN] MotorBoard IsAlive ", msg.data[0])
            elif msg.arbitration_id == CANID_MOTOR_SETPOINT_RESPONSE:
                enc1, enc2, timeout_remaining = struct.unpack(">HHb", msg.data)
                encoder1.update(enc1) #left
                encoder2.update(enc2) #right
                odometry.update(encoder1.get(), -encoder2.get())
                # print("[CAN] MotorBoard SetpointRes dist:{:.2f} theta:{:.2f} x:{:.2f} y:{:.2f}".format(odometry.get_distance(), odometry.get_theta_deg(), odometry.get_x(), odometry.get_y()))
                # print("[CAN] ", enc1, enc2)

def main():
    dev = get_controller_device()

    t_can_alive = threading.Thread(target=can_alive_thread)
    t_can_alive.start()

    t_can_read = threading.Thread(target=can_read_thread)
    t_can_read.start()

    # for event in dev.read_loop():
    #     print(evdev.categorize(event))

    msg = can.Message(arbitration_id=CANID_MOTOR_RESET_SETPOINT_ERROR)
    bus.send(msg)

    # joytheta_integration = 0.0
    # last_state_y = False
    # last_state_x = False

    while True:
        keys = dev.active_keys()
        # LB = BTN_TR
        # RB = BTN_TR
        # A = BTN_A
        # B = BTN_B
        # X = BTN_X
        # Y = BTN_Y
        # print(keys)
    
        # is_btn_y_pressed = evdev.ecodes.BTN_Y in keys
        # if is_btn_y_pressed and not last_state_y:
        #     joytheta_integration += 45

        # is_btn_x_pressed = evdev.ecodes.BTN_X in keys
        # if is_btn_x_pressed and not last_state_x:
        #     joytheta_integration -= 45
        
        # last_state_y = is_btn_y_pressed
        # last_state_x = is_btn_x_pressed

        x = dev.absinfo(evdev.ecodes.ABS_X) # joystick gauche horizontal
        y = dev.absinfo(evdev.ecodes.ABS_Y) # joystick gauche vertical
        rx = dev.absinfo(evdev.ecodes.ABS_RX) # joystick droite horizontal
        ry = dev.absinfo(evdev.ecodes.ABS_RY) # joystick droite vertical
        z = dev.absinfo(evdev.ecodes.ABS_Z) # gachette gauche
        rz = dev.absinfo(evdev.ecodes.ABS_RZ) # gachette droite

        joytheta = (-x.value/x.max) * 180
        # joytheta = (-x.value/x.max) / 2.0
        # joytheta_integration += joytheta
        joyspeed = (rz.value/rz.max - z.value/z.max) * 800

        joyspeed_ramped = limit_ramp_dist.update(joyspeed)
        # joytheta_ramped = limit_ramp_theta.update(joytheta_integration)
        joytheta_ramped = limit_ramp_theta.update(joytheta)

        err_theta = joytheta_ramped - odometry.get_velocity_theta()
        # err_theta = joytheta_ramped - odometry.get_theta_deg()
        err_speed = joyspeed_ramped - odometry.get_velocity_distance()

        pidtheta = pid_theta.process(err_theta)
        pidspeed = pid_dist.process(err_speed)

        cmdtheta = int(pidtheta)
        cmdspeed = int(pidspeed)

        send_telemetry("vel_theta", odometry.get_velocity_theta())
        send_telemetry("vel_dist", odometry.get_velocity_distance())

        if cmdtheta > 255:
            cmdtheta = 255;
        elif cmdtheta < -255:
            cmdtheta = -255;
        
        if cmdspeed > 255:
            cmdspeed = 255;
        elif cmdspeed < -255:
            cmdspeed = -255;

        print("{:.2f} {:.2f} {} {}".format(joytheta, joyspeed, cmdtheta, cmdspeed))

        data = bytearray(struct.pack(">hh", cmdtheta, cmdspeed))
        msg = can.Message(arbitration_id=CANID_MOTOR_SETPOINT, data=data)
        bus.send(msg)

        time.sleep(CONTROL_LOOP_PERIOD)

if __name__ == "__main__":
    main()