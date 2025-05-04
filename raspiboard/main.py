import collections
import dataclasses
import datetime
import gc
import math
import os
import queue
import select
import socket
import struct
import threading
import time

import can
import can_utils as can_utils
import evdev
import gpiod
from boards.ioboard import IOBoard
from boards.motorboard import MotorBoard
from boards.servoboard import ServoBoard
from canids import CANIDS


@dataclasses.dataclass
class GamepadUpdateData:
    x: float
    y: float
    rx: float
    ry: float
    z: float
    rz: float
    keys_active: list[int]
    keys_pressed: list[int]
    keys_released: list[int]


class Gamepad:
    def __init__(self):
        self.last_keys_active = []
        self.device = None

    def init(self, device_name: str):
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            print(device)
            if device_name in device.name:
                print(f"Found gamepad:{device}")
                self.device = device
                return True

        print("No gamepad found")
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
        x = self.device.absinfo(evdev.ecodes.ABS_HAT0X)
        y = self.device.absinfo(evdev.ecodes.ABS_HAT0Y)
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

        x = self.device.absinfo(evdev.ecodes.ABS_X)  # joystick left horizontal
        y = self.device.absinfo(evdev.ecodes.ABS_Y)  # joystick left vertical
        rx = self.device.absinfo(evdev.ecodes.ABS_RX)  # joystick right horizontal
        ry = self.device.absinfo(evdev.ecodes.ABS_RY)  # joystick right vertical
        z = self.device.absinfo(evdev.ecodes.ABS_Z)  # gachette left
        rz = self.device.absinfo(evdev.ecodes.ABS_RZ)  # gachette right

        self.last_keys_active = keys_active

        return GamepadUpdateData(
            x=x.value / x.max,
            y=y.value / y.max,
            rx=rx.value / rx.max,
            ry=ry.value / ry.max,
            z=z.value / z.max,
            rz=rz.value / rz.max,
            keys_active=keys_active,
            keys_pressed=keys_pressed,
            keys_released=keys_released,
        )


class PID:
    def __init__(self, kp: float, ki: float, kd: float, integrator_max: float = 10000):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integrator_max = integrator_max

        self.last_error = 0.0
        self.integrator = 0.0

    def reset(self):
        self.integrator = 0.0
        self.last_error = 0.0

    def compute(self, error: float) -> float:
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

    def reset(self):
        self.values.clear()
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
    def __init__(
        self,
        control_loop_period: float,
        wheel_perimeter: float,
        ticks_per_rev: int,
        wheel_spacing: float,
    ):
        """
        :param control_loop_period: in seconds
        :param wheel_perimeter: odometry wheel perimeter, in mm
        :param ticks_per_rev: in ticks
        :param wheel_spacing: spacing between the two odometry wheels, in mm
        """
        self.control_loop_period = control_loop_period
        self.wheel_spacing = wheel_spacing
        self.k_wheel = wheel_perimeter / ticks_per_rev

        self.last_ticks_left = 0
        self.last_ticks_right = 0
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.theta_rad = 0.0
        self.distance_mm = 0.0
        self.filter_vel_dist = MovingAverageFilter(5)
        self.filter_vel_theta = MovingAverageFilter(5)
        self.lock = threading.Lock()

    def reset(self):
        self.last_ticks_left = 0
        self.last_ticks_right = 0
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.theta_rad = 0.0
        self.distance_mm = 0.0
        self.filter_vel_dist.reset()
        self.filter_vel_theta.reset()

    def update(self, ticks_left: int, ticks_right: int):
        delta_left = (ticks_left - self.last_ticks_left) * self.k_wheel
        delta_right = (ticks_right - self.last_ticks_right) * self.k_wheel

        delta_theta = (delta_right - delta_left) / self.wheel_spacing
        delta_distance = (delta_right + delta_left) / 2.0

        delta_distance_x = delta_distance * math.cos(self.theta_rad)
        delta_distance_y = delta_distance * math.sin(self.theta_rad)

        vel_theta = delta_theta / self.control_loop_period
        vel_dist = delta_distance / self.control_loop_period

        with self.lock:
            self.last_ticks_left = ticks_left
            self.last_ticks_right = ticks_right
            self.x_mm += delta_distance_x
            self.y_mm += delta_distance_y
            self.theta_rad += delta_theta
            self.distance_mm += delta_distance
            self.filter_vel_dist.update(vel_dist)
            self.filter_vel_theta.update(vel_theta)

    def get_x(self) -> float:
        with self.lock:
            return self.x_mm

    def get_y(self) -> float:
        with self.lock:
            return self.y_mm

    def get_theta(self) -> float:
        with self.lock:
            return self.theta_rad * 180.0 / math.pi

    def get_velocity_theta(self) -> float:
        with self.lock:
            return self.filter_vel_theta.get() * 180.0 / math.pi

    def get_distance(self) -> float:
        with self.lock:
            return self.distance_mm

    def get_velocity_distance(self) -> float:
        with self.lock:
            return self.filter_vel_dist.get()


class ThresholdFilter:
    def __init__(self, threshold: int):
        self.threshold = threshold
        self.filtered_value: int = 0

    def reset(self):
        self.filtered_value = 0

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
        self.last_reading_ticks = 0
        self.total_ticks = 0
        self.lock = threading.Lock()
        self.first_measure = True
        self.filter = ThresholdFilter(3)

    def reset(self):
        self.last_reading_ticks = 0
        self.total_ticks = 0
        self.first_measure = True
        self.filter.reset()

    def update(self, current_reading_ticks: int):
        delta = current_reading_ticks - self.last_reading_ticks

        # handle encoder rollover/rolldown
        if (
            self.last_reading_ticks < self.TICKS_PER_REVOLUTION_FIRST_QUARTER
            and current_reading_ticks > self.TICKS_PER_REVOLUTION_LAST_QUARTER
        ):
            delta -= self.TICKS_PER_REVOLUTION
        elif (
            self.last_reading_ticks > self.TICKS_PER_REVOLUTION_LAST_QUARTER
            and current_reading_ticks < self.TICKS_PER_REVOLUTION_FIRST_QUARTER
        ):
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
    def __init__(
        self, control_loop_period: float, accel_rate: float, decel_rate: float
    ):
        self.accel_rate = accel_rate * control_loop_period
        self.decel_rate = decel_rate * control_loop_period
        self.current_value = 0.0

    def reset(self):
        self.current_value = 0.0

    def update(self, target: float):
        delta = target - self.current_value

        if delta == 0:
            return self.current_value

        direction = 1 if delta > 0 else -1

        if (self.current_value * direction) < 0:
            change = self.decel_rate * direction
        elif abs(delta) > self.accel_rate:
            if (self.current_value < target and direction > 0) or (
                self.current_value > target and direction < 0
            ):
                change = self.accel_rate * direction
            else:
                change = self.decel_rate * direction
        else:
            self.current_value = target
            return self.current_value

        self.current_value += change

        if (direction > 0 and self.current_value > target) or (
            direction < 0 and self.current_value < target
        ):
            self.current_value = target

        return self.current_value


def can_alive_thread():
    while True:
        msg = can.Message(arbitration_id=CANIDS.CANID_RASPI_ALIVE, is_extended_id=False)
        can_utils.send(bus, msg)
        time.sleep(1)


def can_read_thread():
    while True:
        for msg in bus:
            if msg.arbitration_id == CANIDS.CANID_MOTOR_STATUS:
                state_error, enc_left, enc_right = struct.unpack(">?HH", msg.data)
                encoder_left.update(enc_left)
                encoder_right.update(enc_right)
                odometry.update(encoder_left.get(), -encoder_right.get())

            elif msg.arbitration_id == CANIDS.CANID_MOTOR_STATE_ERROR:
                print("MOTORBOARD STATE_ERROR")

            elif msg.is_error_frame:
                print("CAN ERROR FRAME:", msg)


def watch_gpios_thread(chip_path: str = "/dev/gpiochip0", line_offsets: tuple = (5, 6)):
    with gpiod.request_lines(
        chip_path,
        consumer="gpiod_consumer",
        config={
            line_offsets: gpiod.LineSettings(
                direction=gpiod.line.Direction.INPUT,
                bias=gpiod.line.Bias.DISABLED,
                edge_detection=gpiod.line.Edge.BOTH,
                debounce_period=datetime.timedelta(milliseconds=10),
            )
        },
    ) as request:
        while True:
            for event in request.read_edge_events():
                print(event)


CONTROL_LOOP_PERIOD = 1 / 200
WHEEL_PERIMETER = 52.42 * math.pi  # diameter in mm
WHEEL_SPACING = 258.5  # in mm

bus = can_utils.get_can_interface()
ioboard = IOBoard(bus)
motorboard = MotorBoard(bus)
servoboard = ServoBoard(bus)

encoder_left = HallEncoder()
encoder_right = HallEncoder()
odometry = Odometry(
    CONTROL_LOOP_PERIOD,
    WHEEL_PERIMETER,
    HallEncoder.TICKS_PER_REVOLUTION,
    WHEEL_SPACING,
)

GPIO_START = 5
GPIO_SHUTDOWN = 6

teleplot_addr = ("192.168.0.10", 47269)
teleplot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
telemetry_queue = queue.Queue()


def send_telemetry(name, value):
    t = time.time() * 1000
    telemetry_queue.put_nowait((t, name, value))


def telemetry_write_thread():
    while True:
        t, name, value = telemetry_queue.get()
        data = f"{name}:{t}:{value}"
        teleplot_sock.sendto(data.encode(), teleplot_addr)


def setup_realtime():
    # I didn't observed any spike difference when disabling or enabling the gc (when measuring main loop time execution). But still prefer to disable it just in case :P
    gc.disable()

    # use CPU3 (isolcpus=3 in the kernel boot parameters)
    os.sched_setaffinity(0, {3})

    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(10))
    except PermissionError:
        print("cap_sys_nice+ep not enabled on python3 bin")
        quit()


def main():
    setup_realtime()

    pid_dist = PID(30, 1.7, 0, integrator_max=8000)  # good enough values: p=30, i=1.7
    pid_theta = PID(60, 4.0, 0, integrator_max=3000)  # good enough values: p=60  i=4.0

    limit_ramp_theta = RampFilter(CONTROL_LOOP_PERIOD, 2160, 2160)
    limit_ramp_dist = RampFilter(CONTROL_LOOP_PERIOD, 1000, 2100)

    gamepad = Gamepad()
    if not gamepad.init("8BitDo"):
        quit()

    thread_can_alive = threading.Thread(target=can_alive_thread)
    thread_can_alive.start()

    thread_can_read = threading.Thread(target=can_read_thread)
    thread_can_read.start()

    thread_watch_gpio = threading.Thread(
        target=watch_gpios_thread, kwargs={"line_offsets": (GPIO_START, GPIO_SHUTDOWN)}
    )
    thread_watch_gpio.start()

    thread_telemetry_write = threading.Thread(target=telemetry_write_thread)
    thread_telemetry_write.start()

    last_elapsed_time = 0.0

    led_pattern_id = 0
    servo8_angle = 0
    servo15_angle = 0

    while True:
        start_time = time.monotonic()

        gp = gamepad.update()

        if evdev.ecodes.BTN_START in gp.keys_pressed:
            print("let's reset all errors!")

            # because we totally disabled the gc, let's collect it now before we restart the critical application
            gc.collect()

            servoboard.enable_power(False, True, True)

            motorboard.reset_error()
            encoder_left.reset()
            encoder_right.reset()
            odometry.reset()
            pid_dist.reset()
            pid_theta.reset()

        elif evdev.ecodes.BTN_SELECT in gp.keys_pressed:
            print("ok let's go to error mode")
            time.sleep(CONTROL_LOOP_PERIOD * 2)
            servoboard.enable_power(False, False, False)

        gp_theta = -gp.x * 130  # deg/s
        gp_speed = (gp.rz - gp.z) * 400  # mm/s

        # boost mode when holding Y
        if evdev.ecodes.BTN_Y in gp.keys_active:
            gp_theta = -gp.x * 180
            gp_speed = (gp.rz - gp.z) * 800

        if evdev.ecodes.BTN_DPAD_DOWN in gp.keys_pressed:
            print("setting led pattern:", led_pattern_id)
            for i in range(4):
                servoboard.set_led_pattern(i, led_pattern_id)
            led_pattern_id += 1
            if led_pattern_id >= 7:
                led_pattern_id = 0

        if evdev.ecodes.BTN_A in gp.keys_pressed:
            print("write servo angle:", servo8_angle)
            servoboard.servo_write_angle(8, servo8_angle)
            servoboard.servo_write_angle(9, servo8_angle)
            servoboard.servo_write_angle(15, servo15_angle)

            if servo8_angle == 0:
                servo8_angle = 180
            else:
                servo8_angle = 0

            if servo15_angle == 0:
                servo15_angle = 270
            else:
                servo15_angle = 0

        gp_theta = limit_ramp_theta.update(gp_theta)
        gp_speed = limit_ramp_dist.update(gp_speed)

        err_theta = gp_theta - odometry.get_velocity_theta()
        err_dist = gp_speed - odometry.get_velocity_distance()

        cmd_theta = pid_theta.compute(err_theta)
        cmd_dist = pid_dist.compute(err_dist)

        pwm_right = cmd_dist + cmd_theta
        pwm_left = -cmd_dist + cmd_theta

        pwm_right = int(pwm_right)
        pwm_left = int(pwm_left)

        motorboard.pwm_write(pwm_right, pwm_left)

        # send_telemetry("vel_theta", odometry.get_velocity_theta())
        # send_telemetry("err_theta", err_theta)
        # send_telemetry("vel_dist", odometry.get_velocity_distance())
        # send_telemetry("err_dist", err_dist)
        send_telemetry("elapsed_time", last_elapsed_time)

        elapsed_time = time.monotonic() - start_time
        last_elapsed_time = elapsed_time * 1000.0
        timeout = max(0, CONTROL_LOOP_PERIOD - elapsed_time)
        select.select([], [], [], timeout)


if __name__ == "__main__":
    main()
