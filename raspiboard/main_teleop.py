import gc
import select
import threading
import time
from evdev import ecodes
import queue
import logging

from robot import Robot, Gamepad, RobotParameters, HallEncoder, Odometry, PID, RampFilter, telemetry


class TeleopController:
    def __init__(self, params: RobotParameters):
        self.encoder_left = HallEncoder(params)
        self.encoder_right = HallEncoder(params)
        self.odometry = Odometry(params)

        self.pid_dist = PID(30, 1.7, 0, integrator_max=8000)  # good enough values: p=30, i=1.7 d=0
        self.pid_theta = PID(60, 4, 0, integrator_max=3000)  # good enough values: p=60  i=4 d=0

        self.ramp_theta = RampFilter(params.CONTROLLOOP_PERIOD_S, 360 * 6, 360 * 6)
        self.ramp_dist = RampFilter(params.CONTROLLOOP_PERIOD_S, 1000, 2100)

        self.lock = threading.Lock()

    def on_status(self, state_error: bool, enc_left: int, enc_right: int) -> None:
        """
        This must be called by the motorboard callback (from on_message_received from the python-can Notifier thread)
        """
        with self.lock:
            self.encoder_left.update(enc_left)
            self.encoder_right.update(enc_right)
            self.odometry.update(self.encoder_left.get(), -self.encoder_right.get())

    def compute(self, velocity_theta: float, velocity_dist: float) -> tuple[int, int]:
        velocity_theta_ramped = self.ramp_theta.update(velocity_theta)
        velocity_dist_ramped = self.ramp_dist.update(velocity_dist)

        with self.lock:
            error_velocity_theta = velocity_theta_ramped - self.odometry.get_velocity_theta()
            error_velocity_dist = velocity_dist_ramped - self.odometry.get_velocity_distance()

        pwm_theta = self.pid_theta.compute(error_velocity_theta)
        pwm_dist = self.pid_dist.compute(error_velocity_dist)

        pwm_left = int(-pwm_dist + pwm_theta)
        pwm_right = int(pwm_dist + pwm_theta)

        return (pwm_left, pwm_right)

    def reset_odometry(self):
        with self.lock:
            self.encoder_left.reset()
            self.encoder_right.reset()
            self.odometry.reset()

    def reset_pid(self):
        self.pid_dist.reset()
        self.pid_theta.reset()


class TeleopRobot(Robot):
    def __init__(self):
        super().__init__()
        self.gamepad = Gamepad()
        self.gamepad_updates = queue.Queue()

        self.controller = TeleopController(self.params)
        self.motorboard.set_status_callback(self.controller.on_status)

        self.logger = logging.getLogger(self.__class__.__name__)

    def set_grabber(self, grab: bool):
        self.servoboard.servo_write_angle(8, 0 if grab else 180)
        self.servoboard.servo_write_angle(9, 180 if grab else 0)
        self.servoboard.servo_write_angle(10, 0 if grab else 180)
        self.servoboard.servo_write_angle(11, 180 if grab else 0)

    def thread_gamepad_actions(self):
        # servo_write_angle, goto_abs, home are blocking for a duration of can.send each (approx 1ms), ence why this thread is required

        ELEVATOR_ACCEL = int(self.params.STEPPER_STEPS_PER_REV * 10)
        ELEVATOR_MAXVEL = int(self.params.STEPPER_STEPS_PER_REV * 2)
        ELEVATOR_MAXVEL_LOWER = int(self.params.STEPPER_STEPS_PER_REV * 4)
        ELEVATOR_POS_LOW = -7900
        ELEVATOR_POS_HIGH = -1100
        ELEVATOR_POS_HIGH_WITH_MARGIN = -800

        ELEVATOR_HOMING_MAX_STEPS = int(self.params.STEPPER_STEPS_PER_REV * 10)

        led_pattern_id = 0
        grabber_state = False

        # step_counter = 0

        while not self.stop_event.is_set():
            try:
                gp = self.gamepad_updates.get(timeout=1)
            except queue.Empty:
                continue

            # handle gamepad X
            if ecodes.BTN_X in gp.keys_pressed:
                self.logger.info("setting led pattern:%d", led_pattern_id)
                for i in range(4):
                    self.servoboard.set_led_pattern(i, led_pattern_id)
                led_pattern_id += 1
                if led_pattern_id >= 7:
                    led_pattern_id = 0

            # handle gamepad A
            if ecodes.BTN_A in gp.keys_pressed:
                self.set_grabber(grabber_state)
                grabber_state = not grabber_state

            # if ecodes.BTN_B in gp.keys_pressed:
            #     step_counter -= 20
            #     self.logger.debug("step_counter:%d", step_counter)
            #     self.ioboard.goto_abs(4, step_counter, ELEVATOR_ACCEL, ELEVATOR_MAXVEL)
            # if ecodes.BTN_X in gp.keys_pressed:
            #     step_counter += 20
            #     self.logger.debug("step_counter:%d", step_counter)
            #     self.ioboard.goto_abs(4, step_counter, ELEVATOR_ACCEL, ELEVATOR_MAXVEL)

            # handle gamepad DPAD
            if ecodes.BTN_DPAD_UP in gp.keys_pressed:
                self.ioboard.goto_abs(4, ELEVATOR_POS_HIGH_WITH_MARGIN, ELEVATOR_ACCEL, ELEVATOR_MAXVEL)
                self.logger.debug("goto pos high with margin")
            elif ecodes.BTN_DPAD_LEFT in gp.keys_pressed:
                self.ioboard.goto_abs(4, ELEVATOR_POS_HIGH, ELEVATOR_ACCEL, ELEVATOR_MAXVEL)
                self.logger.debug("goto pos high")
            elif ecodes.BTN_DPAD_DOWN in gp.keys_pressed:
                self.logger.debug("goto pos low")
                self.ioboard.goto_abs(4, ELEVATOR_POS_LOW, ELEVATOR_ACCEL, ELEVATOR_MAXVEL_LOWER)
            elif ecodes.BTN_DPAD_RIGHT in gp.keys_pressed:
                self.logger.debug("starting homing")
                self.ioboard.home(4, ELEVATOR_HOMING_MAX_STEPS, 15, False)

    def run(self):
        self.start()

        t_gamepad_actions = threading.Thread(target=self.thread_gamepad_actions)
        t_gamepad_actions.start()

        last_elapsed_time = 0.0

        while not self.stop_event.is_set():
            # main critical loop, the elapsed_time for this loop MUST strictly be under 5ms

            start_time = time.monotonic()

            if not self.gamepad.is_connected():
                self.logger.info("Waiting for gamepad...")
                if not self.gamepad.connect():
                    time.sleep(1)
                    continue

            gp = self.gamepad.update()
            if gp is None:
                self.controller.reset_pid()
                continue
            self.gamepad_updates.put(gp)

            # handle gamepad START
            if ecodes.BTN_START in gp.keys_pressed:
                self.logger.info("START")

                # because we totally disabled the gc, let's collect it now before we restart the critical application
                gc.collect()

                self.ioboard.enable(True)
                self.servoboard.enable_power(False, True, True)
                self.controller.reset_pid()
                self.motorboard.reset_error()

            # handle gamepad SELECT
            elif ecodes.BTN_SELECT in gp.keys_pressed:
                self.logger.info("PAUSE")
                time.sleep(self.params.CONTROLLOOP_PERIOD_S * 2)
                self.ioboard.enable(False)
                self.servoboard.enable_power(False, False, False)

            # handle gamepad sticks
            gp_theta = -gp.x * 130  # deg/s
            gp_speed = (gp.rz - gp.z) * 400  # mm/s

            # handle gamepad Y
            if ecodes.BTN_Y in gp.keys_active:
                gp_theta = -gp.x * 180
                gp_speed = (gp.rz - gp.z) * 800

            pwm_left, pwm_right = self.controller.compute(gp_theta, gp_speed)
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


if __name__ == "__main__":
    robot = TeleopRobot()
    robot.run()
