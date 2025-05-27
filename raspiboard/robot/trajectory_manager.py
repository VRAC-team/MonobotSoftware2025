import enum
import math
import threading
import logging
import time

from robot.parameters import RobotParameters, TeamColor
from robot.trapzoid_profile import TrapezoidProfile
from robot.odometry import Odometry
from robot.lidar import Lidar
from robot.hall_encoder import HallEncoder
from robot.pid import PID


class DistanceVelocity(enum.Enum):
    VERY_SLOW = (250, 100)
    SLOW = (600, 250)
    NORMAL = (600, 350)


class RotateVelocity(enum.Enum):
    VERY_SLOW = (140, 90)
    SLOW = (180, 120)
    NORMAL = (360, 360)


class TrajectoryType(enum.Enum):
    STAY_AT_POSITION = "stay_at_position"
    LINE = "line"
    ROTATE = "rotate"
    LOOK_AT = "look_at"
    GOTO_XY = "xy"

    TEST_CONSTANT_VELOCITY_DISTANCE = "test_constant_velocity_distance"
    TEST_CONSTANT_VELOCITY_THETA = "test_constant_velocity_theta"


def normalize_theta(theta: float):
    while theta > 180:
        theta -= 360
    while theta < -180:
        theta += 360

    return theta


class Setpoints:
    # TrajectoryType.STAY_AT_POSITION is using distance_mm and theta_deg
    # TrajectoryType.LINE is using distance_mm and theta_deg
    # TrajectoryType.ROTATE is using distance_mm and theta_deg
    # TrajectoryType.LOOK_AT is using distance_mm and theta_deg

    # TrajectoryType.GOTO_XY is using x_mm and y_mm
    def __init__(self):
        self.theta_deg = 0
        self.distance_mm = 0
        self.x_mm = 0
        self.y_mm = 0


class TrajectoryManager:
    def __init__(self, params: RobotParameters, odometry: Odometry, lidar: Lidar, stop_event: threading.Event):
        self.params = params
        self.odometry = odometry
        self.lidar = lidar
        self.stop_event = stop_event

        self.encoder_left = HallEncoder(params)
        self.encoder_right = HallEncoder(params)

        self.lock = threading.Lock()

        # NEW VALUES TUNNED 14/05/2025 22h10
        self.pid_dist = PID(kp=20, ki=0, kd=60)
        self.pid_theta = PID(kp=20, ki=0, kd=60)
        self.pid_vel_dist = PID(kp=15, ki=1.1, kd=0, integrator_max=10000)
        self.pid_vel_theta = PID(kp=50, ki=2.5, kd=0, integrator_max=10000)

        self.trapezoid_profile = TrapezoidProfile(self.params.CONTROLLOOP_PERIOD_S)
        self.type = TrajectoryType.STAY_AT_POSITION
        self.setpoints = Setpoints()
        self.startpoints = Setpoints()

        self.logger = logging.getLogger(self.__class__.__name__)

    def on_status(self, state_error: bool, enc_left: int, enc_right: int) -> None:
        """
        This must be called by the motorboard callback (from on_message_received from the python-can Notifier thread)
        """
        with self.lock:
            self.encoder_left.update(enc_left)
            self.encoder_right.update(enc_right)
            self.odometry.update(self.encoder_left.get(), -self.encoder_right.get())

    def set_team(self, team: TeamColor):
        self.team = team

    def set_odometry(self, x_mm: float = None, y_mm: float = None, theta_deg: float = None):
        if self.type != TrajectoryType.STAY_AT_POSITION:
            self.logger.error("can't set_odometry when state is not STAY_AT_POSITION (state:%s)", self.type)
            return False

        with self.lock:
            self.pid_dist.reset()
            self.pid_theta.reset()
            self.pid_vel_dist.reset()
            self.pid_vel_theta.reset()

            self.encoder_left.reset()
            self.encoder_right.reset()
            theta_rad = theta_deg * math.pi / 180.0
            self.odometry.set(x_mm, y_mm, theta_rad)

            if x_mm is not None:
                self.startpoints.x_mm = x_mm
                self.setpoints.x_mm = x_mm
            if y_mm is not None:
                self.startpoints.y_mm = y_mm
                self.setpoints.y_mm = y_mm
            if theta_deg is not None:
                self.startpoints.theta_deg = theta_deg
                self.setpoints.theta_deg = theta_deg

            self.startpoints.distance_mm = 0
            self.setpoints.distance_mm = 0

    def line(self, distance: float, vel: DistanceVelocity) -> bool:
        if self.type != TrajectoryType.STAY_AT_POSITION:
            self.logger.error("asked for line but state is not STAY_AT_POSITION (state:%s)", self.type)
            return False

        acceleration, max_velocity = vel.value
        self.trapezoid_profile.plan(acceleration, max_velocity, distance)

        self.startpoints.distance_mm = self.setpoints.distance_mm
        self.setpoints.distance_mm += distance
        self.type = TrajectoryType.LINE
        return True

    def rotate(self, degree: float, vel: RotateVelocity) -> bool:
        if self.type != TrajectoryType.STAY_AT_POSITION:
            self.logger.error("asked for rotate but state is not STAY_AT_POSITION (state:%s)", self.type)
            return False

        if self.team is TeamColor.YELLOW:
            degree = -degree

        acceleration, max_velocity = vel.value
        self.trapezoid_profile.plan(acceleration, max_velocity, degree)

        self.startpoints.theta_deg = self.setpoints.theta_deg
        self.setpoints.theta_deg += degree
        self.type = TrajectoryType.ROTATE
        return True

    def look_at(self, x: float, y: float, vel: RotateVelocity) -> bool:
        if self.type != TrajectoryType.STAY_AT_POSITION:
            self.logger.error("asked for look_at but state is not STAY_AT_POSITION (state:%s)", self.type)
            return False

        if self.team is TeamColor.YELLOW:
            x = -x

        dx = x - self.odometry.get_x()
        dy = y - self.odometry.get_y()
        theta = normalize_theta(math.atan2(dy, dx) * (180.0 / math.pi))

        self.startpoints.theta_deg = self.setpoints.theta_deg
        self.setpoints.theta_deg = theta

        distance = self.setpoints.theta_deg - self.startpoints.theta_deg

        acceleration, max_velocity = vel.value
        self.trapezoid_profile.plan(acceleration, max_velocity, distance)

        self.type = TrajectoryType.LOOK_AT
        return True

    def goto_xy(self, x: float, y: float, velocity_distance: DistanceVelocity, velocity_theta: RotateVelocity) -> bool:
        if self.type != TrajectoryType.STAY_AT_POSITION:
            self.logger.error("asked for goto_xy but state is not STAY_AT_POSITION (state:%s)", self.type)
            return False

        # acceleration_dist, max_velocity_dist = velocity_distance.value
        # self.trapezoid_profile.plan(acceleration_dist, max_velocity_dist, distance)

        # self.startpoints.distance_mm = self.setpoints.distance_mm
        # self.setpoints.distance_mm += distance
        # self.type = TrajectoryType.LINE
        # return True

    def compute(self) -> tuple[int, int]:
        # check if current motion is finished
        if self.type is not TrajectoryType.STAY_AT_POSITION:
            if self.trapezoid_profile.is_force_brake():
                pass
            elif self.trapezoid_profile.is_finished():
                self.type = TrajectoryType.STAY_AT_POSITION
                self.pid_vel_dist.reset()
                self.pid_vel_theta.reset()

        dist_error = 0.0
        theta_error = 0.0

        # feedforward
        ff_vel_dist = 0.0
        ff_vel_theta = 0.0

        match self.type:
            case TrajectoryType.STAY_AT_POSITION:
                dist_error = self.setpoints.distance_mm - self.odometry.get_distance()
                theta_error = self.setpoints.theta_deg - self.odometry.get_theta_deg()

            case TrajectoryType.LINE:
                check_lidar_direction = True if self.trapezoid_profile.distance_sign == 1 else False
                if self.lidar.is_emergency_stop(check_lidar_direction) and not self.trapezoid_profile.is_force_brake():
                    self.logger.fatal("Emergency stop! gg %s", check_lidar_direction)
                    self.trapezoid_profile.force_brake()
                    self.stop_event.set()

                pos, vel = self.trapezoid_profile.process(time.monotonic())
                pos += self.startpoints.distance_mm

                dist_error = pos - self.odometry.get_distance()
                theta_error = self.setpoints.theta_deg - self.odometry.get_theta_deg()

                ff_vel_dist = vel

            case TrajectoryType.ROTATE | TrajectoryType.LOOK_AT:
                pos, vel = self.trapezoid_profile.process(time.monotonic())
                pos += self.startpoints.theta_deg

                dist_error = self.setpoints.distance_mm - self.odometry.get_distance()
                theta_error = pos - self.odometry.get_theta_deg()

                ff_vel_theta = vel

            case TrajectoryType.XY:
                dx = self.sp.x_mm - self.odometry.get_x()
                dy = self.sp.y_mm - self.odometry.get_y()
                dist_error = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
                theta_error = math.atan2(dy, dx) * (180.0 / math.pi) - self.odometry.get_theta_deg()
                theta_error = normalize_theta(theta_error)

                auto_direction = True
                if auto_direction and math.fabs(theta_error) > 90:
                    theta_error = normalize_theta(theta_error + 180)
                    dist_error *= -1

                reduce_dist_speed_when_big_theta_err = True
                k_theta_big_angle_threshold = 15
                if reduce_dist_speed_when_big_theta_err:
                    if math.fabs(theta_error) > k_theta_big_angle_threshold:
                        dist_error = 0
                    else:
                        correction_coef = (
                            k_theta_big_angle_threshold - math.fabs(theta_error)
                        ) / k_theta_big_angle_threshold
                        dist_error *= correction_coef

        vel_dist = self.pid_dist.compute(dist_error) + ff_vel_dist - self.odometry.get_velocity_distance()
        vel_theta = self.pid_theta.compute(theta_error) + ff_vel_theta - self.odometry.get_velocity_theta()

        pwm_dist = self.pid_vel_dist.compute(vel_dist)
        pwm_theta = self.pid_vel_theta.compute(vel_theta)

        pwm_left = int(-pwm_dist + pwm_theta)
        pwm_right = int(pwm_dist + pwm_theta)

        # telemetry.send_enum("type", self.type)
        # telemetry.send_enum("p_state", self.trapezoid_profile.get_state())

        # telemetry.send("dist_error", dist_error)
        # telemetry.send("vel_dist", self.odometry.get_velocity_distance())
        # telemetry.send("ff_vel_dist", ff_vel_dist)
        # telemetry.send("dist", self.odometry.get_distance())
        # telemetry.send("pwm_dist", pwm_dist)

        # telemetry.send("theta_error", theta_error)
        # telemetry.send("vel_theta", self.odometry.get_velocity_theta())
        # telemetry.send("ff_vel_theta", ff_vel_theta)
        # telemetry.send("theta", self.odometry.get_theta_deg())
        # telemetry.send("pwm_theta", pwm_theta)

        return (pwm_left, pwm_right)

    def wait_motion_finished(self):
        while self.type is not TrajectoryType.STAY_AT_POSITION:
            time.sleep(0.1)
