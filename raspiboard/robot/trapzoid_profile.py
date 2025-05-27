import enum
import math
import logging


class TrapezoidProfileType(enum.Enum):
    TRAPZEOID = 0
    TRIANGLE = 1


class TrapezoidProfileState(enum.Enum):
    READY_TO_START = 0
    ACCELERATION = 1
    MAX_VELOCITY = 2
    DECELERATION = 3
    FINISHED = (4,)
    FINISHED_BY_FORCE_BRAKE = (5,)


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

        self._force_brake = False
        self.state = TrapezoidProfileState.FINISHED
        self.start_time = 0.0

        self.logger = logging.getLogger(self.__class__.__name__)

    def plan(self, acceleration: float, max_velocity: float, distance: float):
        """
        :param acceleration: in in mm/s^-2 or deg/s^-2
        :param max_velocity: in mm/s or deg/s
        :distance: in mm or deg
        """
        self.target_position = math.fabs(distance)

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
        self._force_brake = False
        self.last_position = 0
        self.last_velocity = 0

        # self.logger.debug("---- TrapezoidProfile plan ----")
        # self.logger.debug("type:%s", self.type.name)
        # self.logger.debug("distance:%f", distance)
        # self.logger.debug("accel_time:%f", self.accel_time)
        # self.logger.debug("accel_dist:%f", self.accel_dist)
        # self.logger.debug("maxvel_time:%f", self.maxvel_time)
        # self.logger.debug("maxvel_dist:%f", self.maxvel_dist)

    def process(self, current_time: float) -> tuple[float, float]:
        """
        :param time_: global timer in seconds
        """
        if self.state == TrapezoidProfileState.READY_TO_START:
            self.start_time = current_time

        if self.state is TrapezoidProfileState.FINISHED:
            velocity = 0.0
            position = self.target_position * self.distance_sign
            return (position, velocity)

        if self.state is TrapezoidProfileState.FINISHED_BY_FORCE_BRAKE:
            velocity = 0.0
            position = self.last_position * self.distance_sign
            return (position, velocity)

        if self._force_brake:
            velocity = self.last_velocity - self.acceleration * self.control_loop_period
            if velocity < 0:
                velocity = 0
                self.state = TrapezoidProfileState.FINISHED_BY_FORCE_BRAKE

            position = self.last_position + velocity * self.control_loop_period

            self.last_position = position
            self.last_velocity = velocity

            position *= self.distance_sign
            velocity *= self.distance_sign

            return (position, velocity)

        t = current_time - self.start_time

        if t < self.accel_time:
            self.state = TrapezoidProfileState.ACCELERATION
        elif t < self.accel_time + self.maxvel_time:
            self.state = TrapezoidProfileState.MAX_VELOCITY
        elif t < self.accel_time + self.maxvel_time + self.accel_time:
            self.state = TrapezoidProfileState.DECELERATION
        else:
            self.state = TrapezoidProfileState.FINISHED

        velocity = 0.0
        position = 0.0

        match self.state:
            case TrapezoidProfileState.ACCELERATION:
                velocity = self.acceleration * t
                position = 0.5 * self.acceleration * t**2
            case TrapezoidProfileState.MAX_VELOCITY:
                t1 = t - self.accel_time
                velocity = self.max_velocity
                position = self.accel_dist + self.max_velocity * t1
            case TrapezoidProfileState.DECELERATION:
                t2 = t - (self.accel_time + self.maxvel_time)
                velocity = self.max_velocity - self.acceleration * t2
                position = (
                    self.accel_dist + self.maxvel_dist + (self.max_velocity * t2 - 0.5 * self.acceleration * t2**2)
                )
            case TrapezoidProfileState.FINISHED:
                velocity = 0.0
                position = self.target_position

        self.last_position = position
        self.last_velocity = velocity

        position *= self.distance_sign
        velocity *= self.distance_sign

        return (position, velocity)

    def force_brake(self):
        if self.state is TrapezoidProfileState.FINISHED:
            return False

        if self.state is TrapezoidProfileState.DECELERATION:
            return True

        self._force_brake = True
        return True

    def is_force_brake(self):
        return self._force_brake

    def get_state(self) -> TrapezoidProfileState:
        return self.state

    def is_finished(self) -> bool:
        return self.state is TrapezoidProfileState.FINISHED
