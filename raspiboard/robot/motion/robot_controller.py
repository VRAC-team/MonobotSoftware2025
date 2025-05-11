import threading
from robot import HallEncoder, Odometry
from robot.parameters import RobotParameters
from robot.filters import RampFilter
from robot.pid import PID


class RobotController:
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
