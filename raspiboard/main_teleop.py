import datetime
import gc
import select
import threading
import time
import signal
import can
from evdev import ecodes
import gpiod
import logging

import robot.can_utils as can_utils
from robot import MotorBoard, ServoBoard, IOBoard
from robot import CANIDS, Gamepad, RobotController, RobotParameters
from robot.telemetry import telemetry
import robot.utils as utils


class Robot:
    def __init__(self):
        self.logger = logging.getLogger(__name__ + "." + self.__class__.__name__)

        self.bus = can_utils.get_can_interface()
        if self.bus is None:
            self.logger.error("no CAN bus found, quitting..")
            quit()

        self.params = RobotParameters()

        self.stop_event = threading.Event()
        self.t_watch_gpios = threading.Thread(target=self.thread_watch_gpios)
        self.t_can_alive = threading.Thread(target=self.thread_can_alive)

        self.motorboard = MotorBoard(self.bus)
        self.servoboard = ServoBoard(self.bus)
        self.ioboard = IOBoard(self.bus)
        self.robotcontroller = RobotController(self.params)
        self.motorboard.set_status_callback(self.robotcontroller.on_status)

        self.notifier = can.Notifier(self.bus, [self.motorboard, self.servoboard, self.ioboard])

        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, sig, frame):
        self.logger.info("SIGINT received!")
        self.stop_event.set()

    def thread_can_alive(self):
        while not self.stop_event.is_set():
            msg = can.Message(arbitration_id=CANIDS.CANID_RASPI_ALIVE, is_extended_id=False)
            can_utils.send(self.bus, msg)
            time.sleep(1)

    def thread_watch_gpios(self):
        with gpiod.request_lines(
            "/dev/gpiochip0",
            consumer="gpiod_consumer",
            config={
                (5, 6): gpiod.LineSettings(
                    direction=gpiod.line.Direction.INPUT,
                    bias=gpiod.line.Bias.DISABLED,
                    edge_detection=gpiod.line.Edge.BOTH,
                    debounce_period=datetime.timedelta(milliseconds=10),
                )
            },
        ) as request:
            while not self.stop_event.is_set():
                if not request.wait_edge_events(timeout=0.5):
                    continue

                for event in request.read_edge_events():
                    print(event)

    def start(self):
        telemetry.start(("192.168.0.10", 47269))
        utils.setup_logging()
        utils.setup_realtime()

        self.t_watch_gpios.start()
        self.t_can_alive.start()

        self.motorboard.reboot()
        self.servoboard.reboot()
        self.ioboard.reboot()

    def stop(self):
        self.notifier.stop()

        self.t_watch_gpios.join()
        self.t_can_alive.join()
        self.bus.shutdown()
        telemetry.stop()


class TeleopRobot(Robot):
    def __init__(self):
        super().__init__()
        self.gamepad = Gamepad()
        self.logger = logging.getLogger(__name__ + "." + self.__class__.__name__)

    def run(self):
        self.start()

        ELEVATOR_ACCEL = self.params.STEPPER_STEPS_PER_REV * 40
        ELEVATOR_MAXVEL = self.params.STEPPER_STEPS_PER_REV * 4
        ELEVATOR_POS_LOW = 0
        ELEVATOR_POS_HIGH = self.params.STEPPER_STEPS_PER_REV * 3
        ELEVATOR_HOMING_MAX_STEPS = self.params.STEPPER_STEPS_PER_REV * 10

        last_elapsed_time = 0.0

        led_pattern_id = 0
        servo_angle = 0

        while not self.stop_event.is_set():
            start_time = time.monotonic()

            if not self.gamepad.is_connected():
                self.logger.info("Waiting for gamepad...")
                if not self.gamepad.connect():
                    time.sleep(1)
                    continue

            if not self.gamepad.update():
                self.robotcontroller.reset_pid()
                continue

            # handle gamepad START
            if ecodes.BTN_START in self.gamepad.keys_pressed:
                self.logger.info("RESUME")

                # because we totally disabled the gc, let's collect it now before we restart the critical application
                gc.collect()

                self.ioboard.enable(True)
                self.servoboard.enable_power(False, True, True)
                self.robotcontroller.reset_pid()
                self.motorboard.reset_error()

            # handle gamepad SELECT
            elif ecodes.BTN_SELECT in self.gamepad.keys_pressed:
                self.logger.info("PAUSE")
                time.sleep(self.params.CONTROLLOOP_PERIOD_S * 2)
                self.ioboard.enable(False)
                self.servoboard.enable_power(False, False, False)

            # handle gamepad sticks
            gp_theta = -self.gamepad.x * 130  # deg/s
            gp_speed = (self.gamepad.rz - self.gamepad.z) * 400  # mm/s

            # handle gamepad Y
            if ecodes.BTN_Y in self.gamepad.keys_active:
                gp_theta = -self.gamepad.x * 180
                gp_speed = (self.gamepad.rz - self.gamepad.z) * 800

            # handle gamepad X
            if ecodes.BTN_X in self.gamepad.keys_pressed:
                self.logger.info("setting led pattern:%d", led_pattern_id)
                for i in range(4):
                    self.servoboard.set_led_pattern(i, led_pattern_id)
                led_pattern_id += 1
                if led_pattern_id >= 7:
                    led_pattern_id = 0

            # handle gamepad A
            if ecodes.BTN_A in self.gamepad.keys_pressed:
                self.servoboard.servo_write_angle(8, servo_angle)
                self.servoboard.servo_write_angle(9, servo_angle)
                self.servoboard.servo_write_angle(15, servo_angle)

                if servo_angle == 0:
                    servo_angle = 180
                else:
                    servo_angle = 0

            # handle gamepad DPAD
            if ecodes.BTN_DPAD_UP in self.gamepad.keys_pressed:
                self.ioboard.goto_abs(4, ELEVATOR_POS_HIGH, ELEVATOR_ACCEL, ELEVATOR_MAXVEL)
            elif ecodes.BTN_DPAD_DOWN in self.gamepad.keys_pressed:
                self.ioboard.goto_abs(4, ELEVATOR_POS_LOW, ELEVATOR_ACCEL, ELEVATOR_MAXVEL)
            elif ecodes.BTN_DPAD_RIGHT in self.gamepad.keys_pressed:
                self.ioboard.home(4, ELEVATOR_HOMING_MAX_STEPS, 15, False)

            pwm_left, pwm_right = self.robotcontroller.compute(gp_theta, gp_speed)
            self.motorboard.pwm_write(pwm_left, pwm_right)

            # telemetry.send("vel_theta", odometry.get_velocity_theta())
            # telemetry.send("err_theta", err_theta)
            # telemetry.send("vel_dist", odometry.get_velocity_distance())
            # telemetry.send("err_dist", err_dist)
            telemetry.send("elapsed_time", last_elapsed_time)

            # try our best to execute the function exactly at CONTROLLOOP_FREQ_HZ
            elapsed_time = time.monotonic() - start_time
            last_elapsed_time = elapsed_time * 1000.0
            timeout = max(0, self.params.CONTROLLOOP_PERIOD_S - elapsed_time)
            select.select([], [], [], timeout)

        self.gamepad.disconnect()


def main():
    robot = TeleopRobot()
    robot.run()


if __name__ == "__main__":
    main()
