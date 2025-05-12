import gc
import select
import threading
import time
from evdev import ecodes
import queue
import logging

from robot import Robot, Gamepad, telemetry, Servo


class TeleopRobot(Robot):
    def __init__(self):
        super().__init__()
        self.servoboard.servos = {
            8: Servo(min_us=675, max_us=2125),  # ON / OFF
            9: Servo(min_us=900, max_us=2225),  # OFF / ON
            10: Servo(min_us=500, max_us=1650),  # ON / OFF
            11: Servo(min_us=525, max_us=1750),  # OFF / ON
        }
        self.gamepad = Gamepad()
        self.gamepad_updates = queue.Queue()
        self.logger = logging.getLogger(self.__class__.__name__)

    def set_grabber(self, grab: bool):
        self.servoboard.servo_write_angle(8, 0 if grab else 180)
        self.servoboard.servo_write_angle(9, 180 if grab else 0)
        self.servoboard.servo_write_angle(10, 0 if grab else 180)
        self.servoboard.servo_write_angle(11, 180 if grab else 0)

    def thread_gamepad_actions(self):
        # servo_write_angle, goto_abs, home are blocking for a duration of can.send each (approx 1ms), ence why this thread is required

        ELEVATOR_ACCEL = self.params.STEPPER_STEPS_PER_REV * 40
        ELEVATOR_MAXVEL = self.params.STEPPER_STEPS_PER_REV * 4
        ELEVATOR_POS_LOW = 0
        ELEVATOR_POS_HIGH = self.params.STEPPER_STEPS_PER_REV * 3
        ELEVATOR_HOMING_MAX_STEPS = self.params.STEPPER_STEPS_PER_REV * 10

        led_pattern_id = 0
        grabber_state = False

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

            # handle gamepad DPAD
            elif ecodes.BTN_DPAD_UP in gp.keys_pressed:
                self.ioboard.goto_abs(4, ELEVATOR_POS_HIGH, ELEVATOR_ACCEL, ELEVATOR_MAXVEL)
            elif ecodes.BTN_DPAD_DOWN in gp.keys_pressed:
                self.ioboard.goto_abs(4, ELEVATOR_POS_LOW, ELEVATOR_ACCEL, ELEVATOR_MAXVEL)
            elif ecodes.BTN_DPAD_RIGHT in gp.keys_pressed:
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
                self.robotcontroller.reset_pid()
                continue
            self.gamepad_updates.put(gp)

            # handle gamepad START
            if ecodes.BTN_START in gp.keys_pressed:
                self.logger.info("START")

                # because we totally disabled the gc, let's collect it now before we restart the critical application
                gc.collect()

                self.ioboard.enable(True)
                self.servoboard.enable_power(False, True, True)
                self.robotcontroller.reset_pid()
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
