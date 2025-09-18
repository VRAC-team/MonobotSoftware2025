import enum
import math
import logging


class VelocityRampState(enum.Enum):
    # accelerate to max velocity (or decelerate to max_velocity if start_velocity is higher than max_velocity) and maintain max_velocity
    DECEL_MAXVEL = "decel_maxvel"
    ACCEL_MAXVEL = "accel_maxvel"

    # decelerate to end_velocity
    DECELERATION = "deceleration"
    DECELERATION_FORCE_BRAKE = "deceleration_force_brake"

    FINISHED = "finished"
    FINISHED_FORCE_BRAKE = "finished_force_brake"


class VelocityRamp:
    def __init__(self, control_loop_period: float, deceleration_abs_force_brake: float):
        self.control_loop_period = control_loop_period

        self.target_position = 0.0
        self.acceleration_abs = 0.0
        self.deceleration_abs = 0.0
        self.deceleration_abs_force_brake = deceleration_abs_force_brake
        self.start_velocity = 0.0
        self.max_velocity_abs = 0.0
        self.end_velocity_abs = 0.0
        self.state = VelocityRampState.FINISHED
        self.sign = 1
        self.must_decelerate = False

        self.last_position = 0.0
        self.last_velocity = 0.0

        self.decel_start_position = 0.0
        self.decel_start_velocity = 0.0
        self.decel_time_total = 0.0
        self.decel_time_elapsed = 0.0

        self.logger = logging.getLogger(self.__class__.__name__)

    def force_brake(self) -> bool:
        if self.state in [
            VelocityRampState.DECEL_MAXVEL,
            VelocityRampState.ACCEL_MAXVEL,
            VelocityRampState.DECELERATION,
        ]:
            self.state = VelocityRampState.DECELERATION_FORCE_BRAKE
            return True
        return False

    def has_force_braked(self) -> bool:
        return self.state in [VelocityRampState.DECELERATION_FORCE_BRAKE, VelocityRampState.FINISHED_FORCE_BRAKE]

    def is_finished_by_force_brake(self) -> bool:
        return self.state == VelocityRampState.FINISHED_FORCE_BRAKE

    def is_finished(self) -> bool:
        return self.state == VelocityRampState.FINISHED

    def start(
        self,
        distance: float,
        acceleration_abs: float,
        deceleration_abs: float,
        max_velocity_abs: float,
        start_position: float,
        start_velocity: float = 0.0,
        end_velocity_abs: float = 0.0,
    ) -> float:
        if end_velocity_abs > max_velocity_abs:
            raise RuntimeError("end velocity greater than max velocity")

        self.target_position = start_position + distance
        self.acceleration_abs = acceleration_abs
        self.deceleration_abs = deceleration_abs
        self.max_velocity_abs = max_velocity_abs
        self.start_velocity = start_velocity
        self.end_velocity_abs = end_velocity_abs
        self.state = VelocityRampState.ACCEL_MAXVEL if abs(start_velocity) <= max_velocity_abs else VelocityRampState.DECEL_MAXVEL
        self.sign = 1 if distance > 0.0 else -1
        self.must_decelerate = max_velocity_abs != end_velocity_abs

        self.last_position = start_position
        self.last_velocity = start_velocity

        return self.estimate_total_time(distance)

    def estimate_total_time(self, distance: float):
        distance_abs = abs(distance)

        # compute the trapezoid profile
        accel_time = (self.max_velocity_abs - self.start_velocity) / self.acceleration_abs
        accel_dist = 0.5 * self.acceleration_abs * accel_time * accel_time

        decel_time = (self.max_velocity_abs - self.end_velocity_abs) / self.deceleration_abs
        decel_dist = 0.5 * self.deceleration_abs * decel_time * decel_time

        maxvel_dist = distance_abs - accel_dist - decel_dist
        maxvel_time = maxvel_dist / self.max_velocity_abs

        # if max_velocity can't be reached, recalculate as a triangle profile
        if accel_dist + decel_dist > distance_abs:
            accel_time = math.sqrt(distance_abs / self.acceleration_abs)
            accel_dist = 0.5 * self.acceleration_abs * accel_time * accel_time

            decel_time = accel_time
            decel_dist = accel_dist

            maxvel_dist = 0.0
            maxvel_time = 0.0

            # self.max_velocity_abs = self.acceleration_abs * accel_time

        total_time = accel_time + maxvel_time + decel_time
        return total_time

    def process(self, remaining_distance: float, current_distance: float, current_velocity: float) -> tuple[float, float]:
        vel = self.last_velocity
        pos = self.last_position

        # # method 1: use theorical last_velocity (pro: is precise at all speeds / cons: doesn't react when the robot is not moving, deceleration and finish WILL happen)
        # position_to_decel_abs = (self.last_velocity * self.last_velocity - self.end_velocity_abs * self.end_velocity_abs) / (
        #     2.0 * self.deceleration_abs
        # )
        # tracking_error = abs(self.last_position - current_distance)
        # position_to_decel_abs += tracking_error

        # # method 2: use real velocity (cons: velocity can be a bit noisy, not very precise)
        position_to_decel_abs = (current_velocity * current_velocity - self.end_velocity_abs * self.end_velocity_abs) / (2.0 * self.deceleration_abs)
        position_to_decel_abs += abs(self.last_position - current_distance)  # add tracking error

        if self.must_decelerate and self.state in [VelocityRampState.ACCEL_MAXVEL, VelocityRampState.DECEL_MAXVEL]:
            if abs(remaining_distance) <= position_to_decel_abs:  # and self.target_position <= remaining_distance
                self.logger.debug("WHATTHEFRIKC %f %f %f %f", pos, vel, remaining_distance, position_to_decel_abs)
                self.state = VelocityRampState.DECELERATION
                self.decel_start_velocity = vel
                self.decel_start_position = pos
                self.decel_time_elapsed = 0
                self.decel_time_total = (abs(vel) - self.end_velocity_abs) / self.deceleration_abs

        match self.state:
            case VelocityRampState.FINISHED | VelocityRampState.FINISHED_FORCE_BRAKE:
                return (pos, 0.0)

            case VelocityRampState.DECELERATION_FORCE_BRAKE:
                vel -= self.deceleration_abs_force_brake * self.sign * self.control_loop_period

                if vel * self.sign <= 0.0:
                    vel = 0.0
                    self.state = VelocityRampState.FINISHED_FORCE_BRAKE

            case VelocityRampState.DECELERATION:
                vel -= self.deceleration_abs * self.sign * self.control_loop_period
                if vel * self.sign <= 0.0:
                    vel = 0.0
                    self.state = VelocityRampState.FINISHED

            case VelocityRampState.ACCEL_MAXVEL:
                vel += self.acceleration_abs * self.sign * self.control_loop_period
                if self.sign == 1:
                    vel = min(vel, self.max_velocity_abs)
                else:
                    vel = max(vel, -self.max_velocity_abs)

            case VelocityRampState.DECEL_MAXVEL:
                vel -= self.deceleration_abs * self.sign * self.control_loop_period
                if self.sign == 1:
                    vel = max(vel, self.max_velocity_abs)
                else:
                    vel = min(vel, -self.max_velocity_abs)

        pos += vel * self.control_loop_period

        self.last_position = pos
        self.last_velocity = vel

        return (pos, vel)
