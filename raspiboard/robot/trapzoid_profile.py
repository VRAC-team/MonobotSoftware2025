import enum
import math
import logging

from robot.utils import clamp


class TrapezoidProfileType(enum.Enum):
    TRAPZEOID = 0
    TRIANGLE = 1


class TrapezoidProfileState(enum.Enum):
    READY_TO_START = 0
    ACCELERATION = 1
    MAX_VELOCITY = 2
    DECELERATION = 3
    FINISHED = 4


class TrapezoidProfile:
    def __init__(self, control_loop_period: float):
        self.control_loop_period = control_loop_period

        self.acceleration = 0.0
        self.max_velocity = 0.0
        self.distance_sign = 1  # 1 if positive distance, else -1

        self.type = TrapezoidProfileType.TRAPZEOID
        self.accel_time = 0.0
        self.accel_dist = 0.0
        self.maxvel_time = 0.0
        self.maxvel_dist = 0.0

        self.force_brake_ = False
        self.state = TrapezoidProfileState.FINISHED
        self.start_time = 0.0
        self.last_velocity = 0.0
        self.last_position = 0.0

        self.logger = logging.getLogger(self.__class__.__name__)

    def plan(self, acceleration: float, max_velocity: float, current_distance: float, distance: float):
        """
        :param acceleration: in in mm/s^-2 or deg/s^-2
        :param max_velocity: in mm/s or deg/s
        :distance: in mm or deg
        """
        self.type = TrapezoidProfileType.TRAPZEOID
        self.accel_time = max_velocity / acceleration
        self.accel_dist = 0.5 * acceleration * self.accel_time**2
        self.maxvel_dist = math.fabs(distance) - self.accel_dist * 2
        self.maxvel_time = self.maxvel_dist / max_velocity

        # if max_velocity can't be reached, recalculate as TRIANGLE profile
        if self.accel_dist * 2 > math.fabs(distance):
            self.type = TrapezoidProfileType.TRIANGLE
            self.accel_time = math.sqrt((2.0 * math.fabs(distance)) / (2.0 * acceleration))
            self.accel_dist = 0.5 * acceleration * self.accel_time**2
            self.maxvel_dist = 0.0
            self.maxvel_time = 0.0

        self.acceleration = acceleration
        self.max_velocity = max_velocity
        self.distance_sign = 1 if distance > 0.0 else -1
        self.state = TrapezoidProfileState.READY_TO_START
        self.force_brake_ = False
        self.last_velocity = 0.0
        self.last_position = current_distance

        self.logger.debug("---- TrapezoidProfile plan ----")
        self.logger.debug("type:%s", self.type.name)
        self.logger.debug("distance:%f", distance)
        self.logger.debug("accel_time:%f", self.accel_time)
        self.logger.debug("accel_dist:%f", self.accel_dist)
        self.logger.debug("maxvel_time:%f", self.maxvel_time)
        self.logger.debug("maxvel_dist:%f", self.maxvel_dist)

    def process(self, current_time: float) -> tuple[float, float]:
        """
        :param time_: global timer in seconds
        """
        if self.state == TrapezoidProfileState.READY_TO_START:
            self.start_time = current_time

        t = current_time - self.start_time

        if t < self.accel_time:
            self.state = TrapezoidProfileState.ACCELERATION
        elif t < self.accel_time + self.maxvel_time:
            self.state = TrapezoidProfileState.MAX_VELOCITY
        elif t < self.accel_time + self.maxvel_time + self.accel_time:
            self.state = TrapezoidProfileState.DECELERATION
        else:
            self.state = TrapezoidProfileState.FINISHED
            self.force_brake_ = False

        if self.force_brake_:
            self.state = TrapezoidProfileState.DECELERATION

        velocity = 0.0

        match self.state:
            case TrapezoidProfileState.ACCELERATION:
                velocity = self.last_velocity + self.acceleration * self.control_loop_period
                velocity = clamp(velocity, 0.0, self.max_velocity)
            case TrapezoidProfileState.MAX_VELOCITY:
                velocity = self.max_velocity
            case TrapezoidProfileState.DECELERATION:
                velocity = self.last_velocity - self.acceleration * self.control_loop_period
                velocity = clamp(velocity, 0.0, self.max_velocity)
            case TrapezoidProfileState.FINISHED:
                velocity = 0.0

        self.last_velocity = velocity
        velocity *= self.distance_sign

        position = self.last_position + velocity * self.control_loop_period
        self.last_position = position

        return (position, velocity)

    def force_brake(self):
        self.force_brake_ = True

    def get_state(self):
        return self.state

    def is_finished(self):
        return self.state == TrapezoidProfileState.FINISHED
