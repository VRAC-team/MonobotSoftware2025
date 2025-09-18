import enum
import math
import logging


class TimeTrapezoidProfileType(enum.Enum):
    TRAPZEOID = 0
    TRIANGLE = 1


class TimeTrapezoidProfileState(enum.Enum):
    READY_TO_START = 0
    ACCELERATION = 1
    MAX_VELOCITY = 2
    DECELERATION = 3
    FINISHED = 4
    FINISHED_BY_FORCE_BRAKE = 5


class TimeTrapezoidProfile:
    """
    Time based trapezeoid profile
    Use it when the trajectory is predefined and predictable.
    Pros: Smooth, predictable motion
    Cons: Doesn’t respond to disturbances or errors (no feedback)
    """

    def __init__(self, control_loop_period: float, deceleration_force_brake: float):
        self.control_loop_period = control_loop_period

        self.acceleration = 0.0
        self.deceleration_force_brake = deceleration_force_brake
        self.max_velocity = 0.0
        self.sign = 1  # 1 if positive distance, else -1
        self.distance_abs = 0.0
        self.start_position = 0.0

        self.state = TimeTrapezoidProfileState.FINISHED
        self.type = TimeTrapezoidProfileType.TRAPZEOID
        self.accel_time = 0.0
        self.accel_dist = 0.0
        self.maxvel_time = 0.0
        self.maxvel_dist = 0.0
        self.last_position = 0
        self.last_velocity = 0
        self.start_time = 0.0
        self._force_brake = False

        self.logger = logging.getLogger(self.__class__.__name__)

    def plan(self, distance: float, acceleration: float, max_velocity: float, start_position: float):
        """
        Precomputes the motion profile for the given distance, acceleration, and max velocity.

        :param acceleration: Desired acceleration (mm/s^-2 or deg/s^-2)
        :param max_velocity: Maximum velocity (mm/s or deg/s)
        :distance: motion distance (mm or deg)
        """
        self.acceleration = acceleration
        self.max_velocity = max_velocity
        self.sign = 1 if distance > 0.0 else -1
        self.distance_abs = math.fabs(distance)
        self.start_position = start_position

        self.state = TimeTrapezoidProfileState.READY_TO_START
        self.type = TimeTrapezoidProfileType.TRAPZEOID
        self.accel_time = max_velocity / acceleration
        self.accel_dist = 0.5 * acceleration * self.accel_time**2
        self.maxvel_dist = math.fabs(distance) - self.accel_dist * 2
        self.maxvel_time = self.maxvel_dist / max_velocity

        # if max_velocity can't be reached, recalculate as TRIANGLE profile
        if self.accel_dist * 2 > math.fabs(distance):
            self.type = TimeTrapezoidProfileType.TRIANGLE
            self.accel_time = math.sqrt(math.fabs(distance) / acceleration)
            self.accel_dist = 0.5 * acceleration * self.accel_time**2
            self.maxvel_dist = 0.0
            self.maxvel_time = 0.0
            self.max_velocity = acceleration * self.accel_time

        self.last_position = 0
        self.last_velocity = 0
        self._force_brake = False

        # self.logger.debug("--TRAPEZOID PROFILE--")
        # self.logger.debug("target_position:%f", target_position)
        # self.logger.debug("start_position:%f", start_position)
        # self.logger.debug("type:%s", self.type.name)
        # self.logger.debug("accel_time:%f", self.accel_time)
        # self.logger.debug("accel_dist:%f", self.accel_dist)
        # self.logger.debug("maxvel_dist:%f", self.accel_time)
        # self.logger.debug("maxvel_time:%f", self.accel_time)
        # self.logger.debug("max_velocity:%f", self.max_velocity)

    def process(self, current_time: float) -> tuple[float, float]:
        """
        Updates and returns the current position and velocity based on elapsed time.

        :param current_time: Global time in seconds.
        :return: (position, velocity) tuple.
        """
        if self.state == TimeTrapezoidProfileState.READY_TO_START:
            self.start_time = current_time

        if self.state == TimeTrapezoidProfileState.FINISHED:
            velocity = 0.0
            position = self.distance_abs * self.sign
            return (self.start_position + position, velocity)
        elif self.state == TimeTrapezoidProfileState.FINISHED_BY_FORCE_BRAKE:
            velocity = 0.0
            position = self.last_position * self.sign
            return (self.start_position + position, velocity)

        if self._force_brake:
            velocity = self.last_velocity - self.deceleration_force_brake * self.control_loop_period
            if velocity <= 0:
                velocity = 0
                self.state = TimeTrapezoidProfileState.FINISHED_BY_FORCE_BRAKE

            position = self.last_position + velocity * self.control_loop_period

            self.last_position = position
            self.last_velocity = velocity

            position *= self.sign
            velocity *= self.sign

            return (self.start_position + position, velocity)

        t = current_time - self.start_time

        if t < self.accel_time:
            self.state = TimeTrapezoidProfileState.ACCELERATION
        elif t < self.accel_time + self.maxvel_time:
            self.state = TimeTrapezoidProfileState.MAX_VELOCITY
        elif t < self.accel_time + self.maxvel_time + self.accel_time:
            self.state = TimeTrapezoidProfileState.DECELERATION
        else:
            self.state = TimeTrapezoidProfileState.FINISHED

        velocity = 0.0
        position = 0.0

        match self.state:
            case TimeTrapezoidProfileState.ACCELERATION:
                velocity = self.acceleration * t
                position = 0.5 * self.acceleration * t**2
            case TimeTrapezoidProfileState.MAX_VELOCITY:
                t1 = t - self.accel_time
                velocity = self.max_velocity
                position = self.accel_dist + self.max_velocity * t1
            case TimeTrapezoidProfileState.DECELERATION:
                t2 = t - (self.accel_time + self.maxvel_time)
                velocity = self.max_velocity - self.acceleration * t2
                position = self.accel_dist + self.maxvel_dist + (self.max_velocity * t2 - 0.5 * self.acceleration * t2**2)
            case TimeTrapezoidProfileState.FINISHED:
                velocity = 0.0
                position = self.distance_abs

        self.last_position = position
        self.last_velocity = velocity

        position *= self.sign
        velocity *= self.sign

        return (self.start_position + position, velocity)

    def force_brake(self) -> bool:
        if self.is_finished():
            return False

        self._force_brake = True
        self.logger.debug("force brake (deceleration:%f)", self.deceleration_force_brake)

        return True

    def is_force_brake(self) -> bool:
        return self._force_brake

    def get_state(self) -> TimeTrapezoidProfileState:
        return self.state

    def is_finished(self) -> bool:
        return self.state == TimeTrapezoidProfileState.FINISHED or self.state == TimeTrapezoidProfileState.FINISHED_BY_FORCE_BRAKE
