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
import signal

import can
from evdev import ecodes
import gpiod
import robot.can_utils as can_utils
from robot.boards.ioboard import IOBoard
from robot.boards.motorboard import MotorBoard
from robot.boards.servoboard import ServoBoard
from robot.canids import CANIDS
from robot.filters import RampFilter
from robot.gamepad import Gamepad
from robot.hall_encoder import HallEncoder
from robot.odometry import Odometry
from robot.pid import PID


def send_telemetry(name, value):
    t = time.time() * 1000
    telemetry_queue.put_nowait((t, name, value))


def telemetry_write_thread():
    while not stop_event.is_set():
        try:
            t, name, value = telemetry_queue.get(timeout=0.1)
            data = f"{name}:{t}:{value}"
            teleplot_sock.sendto(data.encode(), teleplot_addr)
        except queue.Empty:
            continue


def can_alive_thread():
    while not stop_event.is_set():
        msg = can.Message(arbitration_id=CANIDS.CANID_RASPI_ALIVE, is_extended_id=False)
        can_utils.send(bus, msg)
        time.sleep(1)


def can_read_thread():
    while not stop_event.is_set():
        try:
            msg = bus.recv(timeout=0.1)
            if not msg:
                continue

            if msg.arbitration_id == CANIDS.CANID_MOTOR_STATUS:
                state_error, enc_left, enc_right = struct.unpack(">?HH", msg.data)
                encoder_left.update(enc_left)
                encoder_right.update(enc_right)
                odometry.update(encoder_left.get(), -encoder_right.get())

            elif msg.arbitration_id == CANIDS.CANID_MOTOR_STATE_ERROR:
                print("MOTORBOARD STATE_ERROR")

            elif msg.is_error_frame:
                print("CAN ERROR FRAME:", msg)

        except can.CanError:
            continue


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
        while not stop_event.is_set():
            if not request.wait_edge_events(timeout=0.5):
                continue

            for event in request.read_edge_events():
                print(event)


def setup_realtime():
    # I didn't observed any spike difference when disabling or enabling the gc (when measuring main loop time execution). But still prefer to disable it just in case :P
    gc.disable()

    # use CPU3 (isolcpus=3 in the kernel boot parameters)
    os.sched_setaffinity(0, {3})

    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(10))
    except PermissionError:
        print("cap_sys_nice+ep not enabled on python3 bin !")


def disable_motors():
    motorboard.reboot()
    ioboard.reboot()
    servoboard.reboot()


CONTROL_LOOP_PERIOD = 1 / 200
WHEEL_PERIMETER = 52.42 * math.pi  # diameter in mm
WHEEL_SPACING = 258.5  # in mm

stop_event = threading.Event()

bus = can_utils.get_can_interface()
if bus is None:
    print("no can bus found")
    quit()

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


def signal_handler(sig, frame):
    print("bye")
    stop_event.set()


def main():
    setup_realtime()

    gamepad = Gamepad()

    pid_dist = PID(30, 1.7, 0, integrator_max=8000)  # good enough values: p=30, i=1.7
    pid_theta = PID(60, 4.0, 0, integrator_max=3000)  # good enough values: p=60  i=4.0

    limit_ramp_theta = RampFilter(CONTROL_LOOP_PERIOD, 2160, 2160)
    limit_ramp_dist = RampFilter(CONTROL_LOOP_PERIOD, 1000, 2100)

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

    signal.signal(signal.SIGINT, signal_handler)

    while not stop_event.is_set():
        if not gamepad.is_connected():
            print("Waiting for gamepad...")
            if not gamepad.connect():
                time.sleep(1)
                continue

        start_time = time.monotonic()

        if not gamepad.update():
            # power_off_motors()
            break

        if ecodes.BTN_START in gamepad.keys_pressed:
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

        elif ecodes.BTN_SELECT in gamepad.keys_pressed:
            print("ok let's go to error mode")
            time.sleep(CONTROL_LOOP_PERIOD * 2)
            servoboard.enable_power(False, False, False)

        gp_theta = -gamepad.x * 130  # deg/s
        gp_speed = (gamepad.rz - gamepad.z) * 400  # mm/s

        # boost mode when holding Y
        if ecodes.BTN_Y in gamepad.keys_active:
            gp_theta = -gamepad.x * 180
            gp_speed = (gamepad.rz - gamepad.z) * 800

        if ecodes.BTN_DPAD_DOWN in gamepad.keys_pressed:
            print("setting led pattern:", led_pattern_id)
            for i in range(4):
                servoboard.set_led_pattern(i, led_pattern_id)
            led_pattern_id += 1
            if led_pattern_id >= 7:
                led_pattern_id = 0

        if ecodes.BTN_A in gamepad.keys_pressed:
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

    # power_off_motors()
    gamepad.disconnect()

    thread_can_alive.join()
    thread_can_read.join()
    thread_watch_gpio.join()
    thread_telemetry_write.join()

    bus.shutdown()


if __name__ == "__main__":
    main()
