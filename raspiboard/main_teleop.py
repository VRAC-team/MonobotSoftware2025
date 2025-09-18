import gc
import select
import threading
import time
from evdev import ecodes
import logging

from robot import Robot, Gamepad, GamepadState, RobotConfig, HallEncoder, Odometry, PID, RampFilter, telemetry
from robot_config_2025 import RobotConfig2025
from robot_2025 import Robot2025, RobotFace, BaguettePosition, AimantPosition, BrasExterieurAscenseurPosition


class TeleopController:
    def __init__(self, params: RobotConfig):
        self.encoder_left = HallEncoder(params)
        self.encoder_right = HallEncoder(params)
        self.odometry = Odometry(params)

        self.pid_dist = PID(50, 200, 0, params.CONTROLLOOP_FREQUENCY, integrator_max=10000)  # good enough values: p=30, i=1.7 d=0
        self.pid_theta = PID(120, 1000, 0, params.CONTROLLOOP_FREQUENCY, integrator_max=5000)  # good enough values: p=60  i=4 d=0

        self.ramp_theta = RampFilter(params.get_controlloop_period(), 360 * 6, 360 * 6)
        self.ramp_dist = RampFilter(params.get_controlloop_period(), 2300, 2300)

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
            error_velocity_theta = velocity_theta_ramped - self.odometry.get_theta_vel()
            error_velocity_dist = velocity_dist_ramped - self.odometry.get_dist_vel()

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
        self.config = RobotConfig2025()
        super().__init__(self.config)

        self.robot2025 = Robot2025(self.servoboard, self.ioboard, self.pumpboard)

        self.gamepad = Gamepad()

        self.controller = TeleopController(self.config)
        self.motorboard.set_status_callback(self.controller.on_status)

        self.control_loop_period = self.config.get_controlloop_period()

        self.logger = logging.getLogger(self.__class__.__name__)

        self.front_grabber_state = False
        self.front_magnet_state = False
        self.front_bras_ext = False

    def process_gamepad(self, gs: GamepadState) -> tuple[float, float]:
        # handle gamepad A (baguettes avant)
        if ecodes.BTN_A in gs.keys_pressed:
            self.logger.debug("baguettes avant %s", self.front_grabber_state)
            if self.front_grabber_state:
                self.robot2025.set_base_baguettes(RobotFace.FRONT, BaguettePosition.PRISE)
            else:
                self.robot2025.set_base_baguettes(RobotFace.FRONT, BaguettePosition.BAS)
            self.front_grabber_state = not self.front_grabber_state

        # handle gamepad B (aimants avant)
        if ecodes.BTN_B in gs.keys_pressed:
            self.logger.debug("aimants avant %s", self.front_magnet_state)
            if self.front_magnet_state:
                self.robot2025.set_base_aimants(RobotFace.FRONT, AimantPosition.ON)
            else:
                self.robot2025.set_base_aimants(RobotFace.FRONT, AimantPosition.OFF)
            self.front_magnet_state = not self.front_magnet_state

        # handle gamepad X (bras ext avant)
        if ecodes.BTN_X in gs.keys_pressed:
            self.logger.debug("bras ext avant %s", self.front_bras_ext)
            if self.front_bras_ext:
                self.robot2025.set_ascenseur_bras_exterieur(RobotFace.FRONT, BrasExterieurAscenseurPosition.PRISE)
            else:
                self.robot2025.set_ascenseur_bras_exterieur(RobotFace.FRONT, BrasExterieurAscenseurPosition.EXTERIEUR)
            self.front_bras_ext = not self.front_bras_ext

        # handle gamepad DPAD
        # if ecodes.BTN_DPAD_UP in gs.keys_pressed:
        #     self.ioboard.goto_abs(4, ELEVATOR_POS_HIGH_WITH_MARGIN, ELEVATOR_ACCEL, ELEVATOR_MAXVEL)
        #     self.logger.debug("goto pos high with margin")
        # elif ecodes.BTN_DPAD_LEFT in gs.keys_pressed:
        #     self.ioboard.goto_abs(4, ELEVATOR_POS_HIGH, ELEVATOR_ACCEL, ELEVATOR_MAXVEL)
        #     self.logger.debug("goto pos high")
        # elif ecodes.BTN_DPAD_DOWN in gs.keys_pressed:
        #     self.logger.debug("goto pos low")
        #     self.ioboard.goto_abs(4, ELEVATOR_POS_LOW, ELEVATOR_ACCEL, ELEVATOR_MAXVEL_LOWER)
        # elif ecodes.BTN_DPAD_RIGHT in gs.keys_pressed:
        #     self.logger.debug("starting homing")
        #     self.ioboard.home(4, ELEVATOR_HOMING_MAX_STEPS, 15, False)

        # handle gamepad START
        if ecodes.BTN_START in gs.keys_pressed:
            self.logger.info("START")

            # because we totally disabled the gc, let's collect it now before we restart the critical application
            gc.collect()

            self.ioboard.enable(True)
            self.servoboard.enable_power(False, True, True)
            self.controller.reset_pid()
            self.motorboard.reset_error()

        # handle gamepad SELECT
        elif ecodes.BTN_SELECT in gs.keys_pressed:
            self.logger.info("PAUSE")
            time.sleep(self.control_loop_period * 2)
            self.ioboard.enable(False)
            self.servoboard.enable_power(False, False, False)

        # handle gamepad sticks
        cmd_vel_theta = -gs.x * 130  # deg/s
        cmd_vel_dist = (gs.rz - gs.z) * 400  # mm/s

        # handle gamepad Y
        if ecodes.BTN_Y in gs.keys_active:
            cmd_vel_theta = -gs.x * 200
            cmd_vel_dist = (gs.rz - gs.z) * 1300

        return (cmd_vel_theta, cmd_vel_dist)

    def run(self):
        self.start()

        last_elapsed_time = 0.0

        while not self.stop_event.is_set():
            # main critical loop, the elapsed_time for this loop MUST strictly be under 5ms
            t = time.monotonic()

            if not self.gamepad.is_connected():
                self.logger.info("Waiting for gamepad...")
                if not self.gamepad.connect():
                    time.sleep(1)
                    continue

            gamepad_state = self.gamepad.update()
            if gamepad_state is None:
                self.controller.reset_pid()
                continue
            cmd_vel_theta, cmd_vel_dist = self.process_gamepad(gamepad_state)

            pwm_left, pwm_right = self.controller.compute(cmd_vel_theta, cmd_vel_dist)
            self.motorboard.pwm_write(pwm_left, pwm_right)

            self.process(t)

            telemetry.send("cmd_vel_dist", cmd_vel_dist)
            telemetry.send("cmd_vel_theta", cmd_vel_theta)
            telemetry.send("get_dist_vel", self.controller.odometry.get_dist_vel())
            telemetry.send("get_theta_vel", self.controller.odometry.get_theta_vel())
            telemetry.send("elapsed_time", last_elapsed_time)

            if last_elapsed_time > 5.0:
                self.logger.warning("elapsed time moar than 5ms! danger zone!")

            # try our best to execute the function exactly at CONTROLLOOP_FREQ_HZ
            elapsed_time = time.monotonic() - t
            last_elapsed_time = elapsed_time * 1000.0
            timeout = max(0, self.control_loop_period - elapsed_time)
            select.select([], [], [], timeout)

        self.gamepad.disconnect()


if __name__ == "__main__":
    robot = TeleopRobot()
    robot.run()
