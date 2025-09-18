import enum
import math
import logging
import time
import dataclasses
from typing import Generator

from .robot_config import RobotConfig
from .team_color import TeamColor
from .velocity_ramp import VelocityRamp
from .hall_encoder import HallEncoder
from .odometry import Odometry
from .pid import PID_RCVA
from .filters import RampFilter
from .telemetry import telemetry


@dataclasses.dataclass(frozen=True)
class VelocityParam:
    accel: float
    decel: float
    max_vel: float

    blocked_error: float  # [mm or deg] if error (consign-current) is greather than this, increment internal counter, else reset the counter
    blocked_counter: int = 3  # if the internal counter is greater than this, emergency brake and consider the motion as blocked

    def __str__(self) -> str:
        return f"VelocityParam(acc={self.accel:.1f} dec={self.decel:.1f} vel={self.max_vel:.1f})"


class DistanceParams(enum.Enum):
    SLOW_HOMING = VelocityParam(400, 400, 100, blocked_error=25.0, blocked_counter=10)

    VERY_SLOW = VelocityParam(300, 300, 100, blocked_error=14.0 + 5000.0)
    SLOW = VelocityParam(1000, 1000, 300, blocked_error=26.0 + 3)
    NORMAL = VelocityParam(2500, 2900, 600, blocked_error=60.0 + 3)
    FAST = VelocityParam(2600, 2900, 800, blocked_error=69.0 + 5)
    VERY_FAST = VelocityParam(2600, 2900, 1200, blocked_error=102.0 + 5)

    def __str__(self) -> str:
        return f"DistanceParams.{self.name}({self.value})"


class ThetaParams(enum.Enum):
    VERY_SLOW = VelocityParam(200, 200, 100, blocked_error=6 + 2)
    SLOW = VelocityParam(800, 800, 200, blocked_error=12 + 2)
    NORMAL = VelocityParam(1500, 1200, 300, blocked_error=18 + 3)
    FAST = VelocityParam(1500, 1200, 500, blocked_error=29 + 5)
    VERY_FAST = VelocityParam(1500, 1200, 600, blocked_error=35 + 5)

    def __str__(self) -> str:
        return f"ThetaParams.{self.name}({self.value})"


class RotationDirection(enum.Enum):
    AUTO = "auto"
    CLOCKWISE = "clockwise"
    COUNTERCLOCKWISE = "counterclockwise"


class RobotOrientation(enum.Enum):
    AUTO = "auto"
    FRONT = "front"
    BACK = "back"


class MotionState(enum.Enum):
    STAY_AT_POSITION = "stay_at_position"
    LINE = "line"
    HOME = "home"
    ROTATE = "rotate"
    LOOK_AT = "look_at"
    GOTO_XY_PHASE1_LOOK_AT = "goto_xy_phase1_look_at"
    GOTO_XY_PHASE2_LINE_TO = "goto_xy_phase2_line_to"
    WAYPOINT_XY = "waypoint_xy"
    WAIT_NEXT_WAYPOINT_XY = "wait_next_waypoint_xy"
    DISABLED = "disabled"


class MotionError(enum.Enum):
    BLOCKED = "blocked"
    TIMEOUT = "timeout"
    NO_WAYPOINT_XY_RECEIVED = "no_waypoint_xy_received"
    HOME_MAX_DISTANCE_REACHED = "home_max_distance_reached"


class Setpoints:
    def __init__(self):
        self.theta_deg = 0.0
        self.distance_mm = 0.0
        self.x_mm = 0.0
        self.y_mm = 0.0


class TrajectoryHelper:
    @staticmethod
    def compute_delta_theta_look_at(
        dx: float,
        dy: float,
        current_theta: float,
        robot_orientation: RobotOrientation = RobotOrientation.FRONT,
        rotation_direction: RotationDirection = RotationDirection.AUTO,
    ) -> tuple[float, RobotOrientation]:
        target_front = math.degrees(math.atan2(dy, dx))
        delta_front = TrajectoryHelper.normalize_theta_deg(target_front - current_theta)
        delta_back = TrajectoryHelper.normalize_theta_deg(target_front + 180.0 - current_theta)

        if rotation_direction == RotationDirection.CLOCKWISE:
            if delta_front > 0.0:
                delta_front -= 360.0
            if delta_back > 0.0:
                delta_back -= 360.0
        elif rotation_direction == RotationDirection.COUNTERCLOCKWISE:
            if delta_front < 0.0:
                delta_front += 360.0
            if delta_back < 0.0:
                delta_back += 360.0

        match robot_orientation:
            case RobotOrientation.FRONT:
                return delta_front, RobotOrientation.FRONT

            case RobotOrientation.BACK:
                return delta_back, RobotOrientation.BACK

            case RobotOrientation.AUTO:
                if abs(delta_front) < abs(delta_back):
                    return delta_front, RobotOrientation.FRONT
                else:
                    return delta_back, RobotOrientation.BACK

    @staticmethod
    def normalize_theta_deg(theta_deg: float):
        return (theta_deg + 180.0) % 360.0 - 180.0


class TrajectoryManager:
    def __init__(self, config: RobotConfig, odometry: Odometry):
        self.config = config
        self.odo = odometry
        self.team = TeamColor.BLUE

        self.encoder_left = HallEncoder(self.config)
        self.encoder_right = HallEncoder(self.config)

        # self.pid_dist = PID(kp=650.0, ki=0, kd=40.0, frequency=params.CONTROLLOOP_FREQUENCY) # aggresif 650 40
        # self.pid_theta = PID(kp=2000.0, ki=0.0, kd=130.0, frequency=params.CONTROLLOOP_FREQUENCY) # aggresif 2000 130
        self.pid_dist = PID_RCVA(kp=800, kd=0.25, frequency=self.config.CONTROLLOOP_FREQUENCY)  # 1000 0.28
        self.pid_theta = PID_RCVA(kp=1800, kd=0.35, frequency=self.config.CONTROLLOOP_FREQUENCY)  # 2000 0.35

        self.ramp_theta = VelocityRamp(self.config.get_controlloop_period(), self.config.FORCEBRAKE_THETA_DECEL)
        self.ramp_dist = VelocityRamp(self.config.get_controlloop_period(), self.config.FORCEBRAKE_DIST_DECEL)

        # HACK this is a hack because there is discontinuities in the velocityramp pos/vel consign between maxvel/decel
        pwm_accel = 0.6 * 32767.0 * 200.0
        self.pwmtheta_limiter = RampFilter(self.config.get_controlloop_period(), pwm_accel, pwm_accel)
        self.pwmdist_limiter = RampFilter(self.config.get_controlloop_period(), pwm_accel, pwm_accel)

        self.velparams_dist: VelocityParam = DistanceParams.NORMAL.value
        self.velparams_theta: VelocityParam = ThetaParams.NORMAL.value

        self.state: MotionState = MotionState.STAY_AT_POSITION
        self.setpoints: Setpoints = Setpoints()

        self.blocked_counter_theta = 0
        self.blocked_counter_dist = 0

        self.logger = logging.getLogger(self.__class__.__name__)

        # used by other than STAY_AT_POSITION
        self.motion_start_time = 0.0
        self.motion_timeout_after_duration = 0.0
        self.motion_error: MotionError | None = None

        # home specific
        self.home_robot_orientation = RobotOrientation.BACK

        # gotoxy specific
        self.gotoxy_rotation_direction: RotationDirection = RotationDirection.AUTO
        self.gotoxy_robot_orientation: RobotOrientation = RobotOrientation.FRONT

        # waypointxy specific
        self.waypointxy_robot_orientation: RobotOrientation = RobotOrientation.FRONT
        self.waypointxy_max_theta_error: float = float("inf")
        self.waypointxy_has_more_waypoints_to_come: bool = False

    def on_status(self, state_error: bool, enc_left: int, enc_right: int) -> None:
        """
        This must be called by the motorboard callback (from on_message_received from the python-can Notifier thread)
        """
        # if state_error:
        #     self.state = MotionState.DISABLED
        self.encoder_left.update(enc_left)
        self.encoder_right.update(enc_right)
        self.odo.update(self.encoder_left.get(), -self.encoder_right.get())

    def set_team(self, team: TeamColor):
        self.team = team

    def force_brake(self) -> bool:
        if self.state == MotionState.STAY_AT_POSITION:
            self.logger.error("asked for emergency_brake but robot is idle (state:%s)", self.state)
            return False

        self.ramp_dist.force_brake()
        self.ramp_theta.force_brake()
        self.logger.debug("force braking ! (state:%s)", self.state)
        return True

    def line(self, distance_mm: float, params: DistanceParams = DistanceParams.SLOW) -> bool:
        if self.state != MotionState.STAY_AT_POSITION:
            self.logger.error("asked for line but state is not STAY_AT_POSITION (state:%s)", self.state)
            return False

        p = params.value
        estimated_time = self.ramp_dist.start(distance_mm, p.accel, p.decel, p.max_vel, self.setpoints.distance_mm)

        self.setpoints.distance_mm += distance_mm
        self.setpoints.x_mm += distance_mm * math.cos(math.radians(self.setpoints.theta_deg))
        self.setpoints.y_mm += distance_mm * math.sin(math.radians(self.setpoints.theta_deg))
        self.velparams_dist = p
        self._trajectory_start(MotionState.LINE, estimated_time)

        self.logger.debug("LINE start (distance_mm:%.1f params:%s estimated_time:%.2f)", distance_mm, params.name, estimated_time)

        return True

    def home(
        self,
        orientation: RobotOrientation = RobotOrientation.BACK,
        max_distance_mm_abs: float = 300.0,
        params: DistanceParams = DistanceParams.SLOW_HOMING,
    ) -> bool:
        if self.state != MotionState.STAY_AT_POSITION:
            self.logger.error("asked for home but state is not STAY_AT_POSITION (state:%s)", self.state)
            return False

        if orientation == RobotOrientation.AUTO:
            self.logger.error("RobotOrientation.AUTO not supported for home")
            return False

        max_distance_mm = abs(max_distance_mm_abs)
        if orientation == RobotOrientation.BACK:
            max_distance_mm = -max_distance_mm

        p = params.value
        estimated_time = self.ramp_dist.start(max_distance_mm, p.accel, p.decel, p.max_vel, self.setpoints.distance_mm)

        self.setpoints.distance_mm += max_distance_mm
        self.setpoints.x_mm += max_distance_mm * math.cos(math.radians(self.setpoints.theta_deg))
        self.setpoints.y_mm += max_distance_mm * math.sin(math.radians(self.setpoints.theta_deg))
        self.velparams_dist = p
        self.home_robot_orientation = orientation
        self._trajectory_start(MotionState.HOME, estimated_time)

        self.logger.debug("HOME start (max_distance_mm:%.1f params:%s estimated_time:%.2f)", max_distance_mm, params.name, estimated_time)

        return True

    def rotate(self, theta_deg: float, params: ThetaParams = ThetaParams.SLOW) -> bool:
        if self.state != MotionState.STAY_AT_POSITION:
            self.logger.error("asked for rotate but state is not STAY_AT_POSITION (state:%s)", self.state)
            return False

        if self.team == TeamColor.YELLOW:
            theta_deg = -theta_deg

        p = params.value
        estimated_time = self.ramp_theta.start(theta_deg, p.accel, p.decel, p.max_vel, self.setpoints.theta_deg)

        self.setpoints.theta_deg += theta_deg
        self.velparams_theta = p
        self._trajectory_start(MotionState.ROTATE, estimated_time)

        self.logger.debug("ROTATE start (theta_deg:%.1f params:%s estimated_time:%.2f)", theta_deg, params.name, estimated_time)

        return True

    def look_at(
        self,
        x: float,
        y: float,
        params: ThetaParams = ThetaParams.SLOW,
        robot_orientation: RobotOrientation = RobotOrientation.FRONT,
        rotation_direction: RotationDirection = RotationDirection.AUTO,
    ) -> bool:
        if self.state != MotionState.STAY_AT_POSITION:
            self.logger.error("asked for look_at but state is not STAY_AT_POSITION (state:%s)", self.state)
            return False

        if self.team == TeamColor.YELLOW:
            x = 3000.0 - x

        dx = x - self.odo.get_x()
        dy = y - self.odo.get_y()
        theta_deg, orientation_computed = TrajectoryHelper.compute_delta_theta_look_at(
            dx, dy, self.setpoints.theta_deg, robot_orientation, rotation_direction
        )

        p = params.value
        estimated_time = self.ramp_theta.start(theta_deg, p.accel, p.decel, p.max_vel, self.setpoints.theta_deg)

        self.setpoints.theta_deg += theta_deg
        self.velparams_theta = p
        self._trajectory_start(MotionState.LOOK_AT, estimated_time)

        self.logger.debug(
            "LOOK_AT start (x:%.1f y:%.1f theta:%.1f asked_orient:%s computed_orient:%s params:%s estimated_time:%.2f) ",
            x,
            y,
            theta_deg,
            robot_orientation.name,
            orientation_computed.name,
            params.name,
            estimated_time,
        )

        return True

    def goto_xy(
        self,
        x: float,
        y: float,
        params_dist: DistanceParams = DistanceParams.SLOW,
        params_theta: ThetaParams = ThetaParams.SLOW,
        robot_orientation: RobotOrientation = RobotOrientation.FRONT,
        rotation_direction: RotationDirection = RotationDirection.AUTO,
    ) -> bool:
        if self.state != MotionState.STAY_AT_POSITION:
            self.logger.error("asked for goto_xy but state is not STAY_AT_POSITION (state:%s)", self.state)
            return False

        if self.team is TeamColor.YELLOW:
            x = 3000.0 - x

        dx = x - self.odo.get_x()
        dy = y - self.odo.get_y()
        distance = math.sqrt(dx * dx + dy * dy)
        theta_deg, orientation_computed = TrajectoryHelper.compute_delta_theta_look_at(
            dx, dy, self.setpoints.theta_deg, robot_orientation, rotation_direction
        )
        if orientation_computed == RobotOrientation.BACK:
            distance *= -1.0

        p_dist = params_dist.value
        p_theta = params_theta.value
        estimated_time_theta = self.ramp_theta.start(theta_deg, p_theta.accel, p_theta.decel, p_theta.max_vel, self.setpoints.theta_deg)
        estimated_time_dist = self.ramp_dist.start(distance, p_dist.accel, p_dist.decel, p_dist.max_vel, self.setpoints.distance_mm)
        estimated_time = estimated_time_theta + estimated_time_dist

        self.gotoxy_robot_orientation = orientation_computed
        self.gotoxy_rotation_direction = rotation_direction
        self.setpoints.theta_deg += theta_deg
        self.setpoints.x_mm = x
        self.setpoints.y_mm = y
        self.velparams_theta = p_theta
        self.velparams_dist = p_dist
        self._trajectory_start(MotionState.GOTO_XY_PHASE1_LOOK_AT, estimated_time)

        self.logger.debug(
            "GOTO_XY start (x:%f y:%f phase1_theta:%.1f phase2_dist:%.1f asked_orient:%s computed_orient:%s params_dist:%s params_theta:%s estimated_time:%.2f)",
            x,
            y,
            theta_deg,
            distance,
            robot_orientation,
            orientation_computed,
            params_dist.name,
            params_theta.name,
            estimated_time,
        )

        return True

    def waypoint_xy_start(
        self,
        x: float,
        y: float,
        params_dist: DistanceParams = DistanceParams.SLOW,
        params_theta: ThetaParams = ThetaParams.SLOW,
        robot_orientation: RobotOrientation = RobotOrientation.FRONT,
    ) -> bool:
        """
        this function implies that the end distance velocity is maxvel
        """
        if self.state != MotionState.STAY_AT_POSITION:
            self.logger.error("asked for waypoint_xy but state is not STAY_AT_POSITION (state:%s)", self.state)
            return False

        if robot_orientation == RobotOrientation.AUTO:
            self.logger.error("RobotOrientation.AUTO not supported for waypoint_xy")
            return False

        if self.team is TeamColor.YELLOW:
            x = 3000.0 - x

        dx = x - self.odo.get_x()
        dy = y - self.odo.get_y()
        distance_mm = math.sqrt(dx * dx + dy * dy)
        theta_deg = math.degrees(math.atan2(dy, dx))
        if robot_orientation == RobotOrientation.BACK:
            theta_deg += 180
        theta_deg = TrajectoryHelper.normalize_theta_deg(theta_deg - self.odo.get_theta())

        p_dist = params_dist.value
        estimated_time_dist = self.ramp_dist.start(
            distance_mm, p_dist.accel, p_dist.decel, p_dist.max_vel, self.setpoints.distance_mm, 0.0, p_dist.max_vel
        )
        p_theta = params_theta.value
        estimated_time_theta = self.ramp_theta.start(theta_deg, p_theta.accel, p_theta.decel, p_theta.max_vel, self.setpoints.theta_deg)
        estimated_time = estimated_time_dist + estimated_time_theta  # * 0.7

        self.waypointxy_robot_orientation = robot_orientation
        self.waypointxy_max_theta_error = math.degrees(p_theta.max_vel / self.config.WAYPOINT_XY_MIN_RADIUS)
        self.waypointxy_has_more_waypoints_to_come = True

        self.setpoints.x_mm = x
        self.setpoints.y_mm = y
        self.velparams_theta = p_theta
        self.velparams_dist = p_dist
        self._trajectory_start(MotionState.WAYPOINT_XY, estimated_time)

        self.logger.debug(
            "WAYPOINT_XY start (x:%.1f y:%.1f orient:%s params_dist:%s params_theta:%s max_theta_error:%.2f estimated_time:%.2f)",
            x,
            y,
            robot_orientation,
            params_dist.name,
            params_theta.name,
            self.waypointxy_max_theta_error,
            estimated_time,
        )

        return True

    def waypoint_xy_chained(self, x: float, y: float, has_more_waypoints_to_come: bool = False) -> bool:
        """
        when chained with a previous waypoint, this must be called less than a CONTROLLOOP_PERIOD after wait_waypoint_at_goal else it will error
        """
        if self.state != MotionState.WAIT_NEXT_WAYPOINT_XY:
            self.logger.error("asked for waypoint_xy_chained but state is not WAIT_NEXT_WAYPOINT_XY (state:%s)", self.state)
            return False

        if self.team is TeamColor.YELLOW:
            x = 3000.0 - x

        dx = x - self.odo.get_x()
        dy = y - self.odo.get_y()
        distance_mm = math.sqrt(dx * dx + dy * dy)
        theta_deg = math.degrees(math.atan2(dy, dx))
        if self.waypointxy_robot_orientation == RobotOrientation.BACK:
            theta_deg += 180
        theta_deg = TrajectoryHelper.normalize_theta_deg(theta_deg - self.odo.get_theta())

        end_velocity = self.velparams_dist.max_vel if has_more_waypoints_to_come else 0.0
        start_velocity = self.velparams_dist.max_vel
        estimated_time_dist = self.ramp_dist.start(
            distance_mm,
            self.velparams_dist.accel,
            self.velparams_dist.decel,
            self.velparams_dist.max_vel,
            self.setpoints.distance_mm,
            start_velocity,
            end_velocity,
        )
        estimated_time_theta = self.ramp_theta.start(
            theta_deg,
            self.velparams_theta.accel,
            self.velparams_theta.decel,
            self.velparams_theta.max_vel,
            self.setpoints.theta_deg,
            self.odo.get_theta_vel(),
        )
        estimated_time = estimated_time_dist + estimated_time_theta  # * 0.7

        self.waypointxy_has_more_waypoints_to_come = has_more_waypoints_to_come
        self.setpoints.x_mm = x
        self.setpoints.y_mm = y
        self._trajectory_start(MotionState.WAYPOINT_XY, estimated_time)

        self.logger.debug(
            "WAYPOINT_XY chained (x:%.1f y:%.1f has_more_waypoints_to_come:%s estimated_time:%.2f)",
            x,
            y,
            has_more_waypoints_to_come,
            estimated_time,
        )
        return True

    def wait_waypoint(self) -> Generator[None, None, MotionError | None]:
        while self.state == MotionState.WAYPOINT_XY and not self.motion_error:
            yield
        return self.motion_error

    def _waypoint_at_goal(self):
        total_time = time.monotonic() - self.motion_start_time
        if self.waypointxy_has_more_waypoints_to_come:
            self.state = MotionState.WAIT_NEXT_WAYPOINT_XY
        else:
            self.state = MotionState.STAY_AT_POSITION
        self.logger.debug("_waypoint_at_goal (total_time:%.2f)", total_time)

    def wait(self) -> Generator[None, None, MotionError | None]:
        while self.state != MotionState.STAY_AT_POSITION and not self.motion_error:
            yield
        return self.motion_error

    def _trajectory_start(self, state: MotionState, estimated_time: float):
        self.state = state
        self.blocked_counter_dist = 0
        self.blocked_counter_theta = 0
        self.motion_timeout_after_duration = estimated_time * self.config.BLOCKED_TOTALTIME_COEF
        self.motion_start_time = time.monotonic()
        self.motion_error = None

    def _trajectory_error(self, error_state: MotionError):
        self.motion_error = error_state
        match error_state:
            case MotionError.TIMEOUT:
                self.logger.error(
                    "_trajectory_error TIMEOUT %s (timeout_after_duration:%.2f)",
                    self.state.name,
                    self.motion_timeout_after_duration,
                )
            case MotionError.BLOCKED:
                self.logger.error(
                    "_trajectory_error BLOCKED %s (blocked_counter_dist:%d blocked_counter_theta:%d)",
                    self.state.name,
                    self.blocked_counter_dist,
                    self.blocked_counter_theta,
                )
            case MotionError.NO_WAYPOINT_XY_RECEIVED:
                self.logger.error("_trajectory_error NO_WAYPOINT_XY_RECEIVED %s", self.state.name)
            case MotionError.HOME_MAX_DISTANCE_REACHED:
                self.logger.error("_trajectory_error HOME_MAX_DISTANCE_REACHED %s", self.state.name)

    def _trajectory_finished(self):
        total_time = time.monotonic() - self.motion_start_time
        error_name = "None" if self.motion_error is None else self.motion_error.value
        self.logger.debug("_trajectory_finished %s err:%s (total_time:%.2f)", self.state.name, error_name, total_time)
        self.state = MotionState.STAY_AT_POSITION

    def process(self, t: float) -> tuple[float, float]:
        out_dist_error_mm = 0.0
        out_theta_error_deg = 0.0

        # HACK temporary to use RCVA modified PID
        consign_vel_dist = 0.0

        trajectory_timeout = False
        if t - self.motion_start_time > self.motion_timeout_after_duration:
            trajectory_timeout = True

        if self.state == MotionState.WAIT_NEXT_WAYPOINT_XY:
            self.state = MotionState.DISABLED
            self.ramp_dist.force_brake()
            self.ramp_theta.force_brake()
            self._trajectory_error(MotionError.NO_WAYPOINT_XY_RECEIVED)
            self.logger.error("did not received the next waypoint in time! disabling..")
            return (0.0, 0.0)

        match self.state:
            case MotionState.DISABLED:
                return (0.0, 0.0)

            case MotionState.STAY_AT_POSITION:
                out_dist_error_mm = self.setpoints.distance_mm - self.odo.get_dist()
                out_theta_error_deg = self.setpoints.theta_deg - self.odo.get_theta()

            case MotionState.LINE:
                remaining_dist = self.setpoints.distance_mm - self.odo.get_dist()
                consign_dist, consign_vel_dist = self.ramp_dist.process(remaining_dist, self.odo.get_dist(), self.odo.get_dist_vel())

                out_dist_error_mm = consign_dist - self.odo.get_dist()
                out_theta_error_deg = self.setpoints.theta_deg - self.odo.get_theta()

                if not self.ramp_dist.has_force_braked():
                    if abs(out_dist_error_mm) >= self.velparams_dist.blocked_error:
                        self.blocked_counter_dist += 1
                    else:
                        self.blocked_counter_dist = 0

                    if self.blocked_counter_dist >= self.velparams_dist.blocked_counter:
                        self.ramp_dist.force_brake()
                        self._trajectory_error(MotionError.BLOCKED)
                    elif trajectory_timeout:
                        self.ramp_dist.force_brake()
                        self._trajectory_error(MotionError.TIMEOUT)

                if self.ramp_dist.is_finished_by_force_brake() and abs(self.odo.get_dist_vel()) < 1.0:
                    self.logger.info("LINE done by force brake (remaining:%.1fmm)", remaining_dist)
                    self.setpoints.theta_deg = self.odo.get_theta()
                    self.setpoints.distance_mm = self.odo.get_dist()
                    self.setpoints.x_mm = self.odo.get_x()
                    self.setpoints.y_mm = self.odo.get_y()
                    self._trajectory_finished()
                elif self.ramp_dist.is_finished() or abs(remaining_dist) < 0.4:
                    self._trajectory_finished()

            case MotionState.HOME:
                remaining_dist = self.setpoints.distance_mm - self.odo.get_dist()
                consign_dist, consign_vel_dist = self.ramp_dist.process(remaining_dist, self.odo.get_dist(), self.odo.get_dist_vel())

                out_dist_error_mm = consign_dist - self.odo.get_dist()
                out_theta_error_deg = (self.setpoints.theta_deg - self.odo.get_theta()) / 8.0  # here we divide the theta error to ease the home

                if abs(out_dist_error_mm) >= self.velparams_dist.blocked_error:
                    self.blocked_counter_dist += 1
                else:
                    self.blocked_counter_dist = 0

                if self.blocked_counter_dist >= self.velparams_dist.blocked_counter:
                    self.setpoints.theta_deg = self.odo.get_theta()
                    self.setpoints.distance_mm = self.odo.get_dist()
                    self.setpoints.x_mm = self.odo.get_x()
                    self.setpoints.y_mm = self.odo.get_y()
                    self._trajectory_finished()
                elif self.ramp_dist.is_finished():
                    self._trajectory_error(MotionError.HOME_MAX_DISTANCE_REACHED)
                # elif self.ramp_dist.is_finished_by_force_brake(): #TODO what do we do there ?
                #     self._trajectory_error(MotionError.TIMEOUT)

            case MotionState.ROTATE | MotionState.LOOK_AT | MotionState.GOTO_XY_PHASE1_LOOK_AT:
                remaining_theta = self.setpoints.theta_deg - self.odo.get_theta()
                consign_theta, consign_vel_theta = self.ramp_theta.process(remaining_theta, self.odo.get_theta(), self.odo.get_theta_vel())

                telemetry.send("consign_theta", consign_theta)
                telemetry.send("consign_vel_theta", consign_vel_theta)

                out_dist_error_mm = self.setpoints.distance_mm - self.odo.get_dist()
                out_theta_error_deg = consign_theta - self.odo.get_theta()

                if not self.ramp_theta.has_force_braked():
                    if abs(out_theta_error_deg) >= self.velparams_theta.blocked_error:
                        self.blocked_counter_theta += 1
                    else:
                        self.blocked_counter_theta = 0

                    if self.blocked_counter_theta >= self.velparams_theta.blocked_counter:
                        self.ramp_theta.force_brake()
                        self._trajectory_error(MotionError.BLOCKED)
                    elif trajectory_timeout:
                        self.ramp_theta.force_brake()
                        self._trajectory_error(MotionError.TIMEOUT)

                if self.ramp_theta.is_finished_by_force_brake() and abs(self.odo.get_theta_vel()) < 1.0:
                    self.logger.debug("%s done by force brake (remaining:%.1fdeg)", self.state.name, remaining_theta)
                    self.setpoints.theta_deg = self.odo.get_theta()
                    self.setpoints.distance_mm = self.odo.get_dist()
                    self.setpoints.x_mm = self.odo.get_x()
                    self.setpoints.y_mm = self.odo.get_y()
                    self._trajectory_finished()
                elif self.ramp_theta.is_finished() or abs(remaining_theta) < 0.3:
                    if self.state == MotionState.GOTO_XY_PHASE1_LOOK_AT:
                        self.state = MotionState.GOTO_XY_PHASE2_LINE_TO
                    else:
                        self._trajectory_finished()

            case MotionState.GOTO_XY_PHASE2_LINE_TO:
                dx = self.setpoints.x_mm - self.odo.get_x()
                dy = self.setpoints.y_mm - self.odo.get_y()
                remaining_dist = math.sqrt(dx * dx + dy * dy)
                target_theta = math.degrees(math.atan2(dy, dx))

                if self.gotoxy_robot_orientation == RobotOrientation.BACK:
                    if self.gotoxy_rotation_direction == RotationDirection.CLOCKWISE:
                        target_theta -= 180.0
                    else:
                        target_theta += 180.0
                    remaining_dist *= -1.0

                out_theta_error_deg = TrajectoryHelper.normalize_theta_deg(target_theta - self.odo.get_theta())

                consign_dist, consign_vel_dist = self.ramp_dist.process(remaining_dist, self.odo.get_dist(), self.odo.get_dist_vel())
                out_dist_error_mm = consign_dist - self.odo.get_dist()

                if not self.ramp_dist.has_force_braked():
                    if abs(out_dist_error_mm) >= self.velparams_dist.blocked_error:
                        self.blocked_counter_dist += 1
                    else:
                        self.blocked_counter_dist = 0

                    if self.blocked_counter_dist >= self.velparams_dist.blocked_counter:
                        self.ramp_dist.force_brake()
                        self._trajectory_error(MotionError.BLOCKED)
                    elif trajectory_timeout:
                        self.ramp_dist.force_brake()
                        self._trajectory_error(MotionError.TIMEOUT)

                if self.ramp_dist.is_finished_by_force_brake() and abs(self.odo.get_dist_vel()) < 1.0:
                    self.logger.info("GOTO_XY_PHASE2_LINE_TO done by force brake (remaining:%.1fmm)", remaining_dist)
                    self.setpoints.theta_deg = self.odo.get_theta()
                    self.setpoints.distance_mm = self.odo.get_dist()
                    self.setpoints.x_mm = self.odo.get_x()
                    self.setpoints.y_mm = self.odo.get_y()
                    self._trajectory_finished()
                elif self.ramp_dist.is_finished() or abs(remaining_dist) < 0.4:
                    self.setpoints.distance_mm = remaining_dist + self.odo.get_dist()
                    self._trajectory_finished()

            case MotionState.WAYPOINT_XY:
                dx = self.setpoints.x_mm - self.odo.get_x()
                dy = self.setpoints.y_mm - self.odo.get_y()
                remaining_dist = math.sqrt(dx * dx + dy * dy)
                target_theta = math.degrees(math.atan2(dy, dx))

                if self.waypointxy_robot_orientation == RobotOrientation.BACK:
                    target_theta += 180.0
                    remaining_dist *= -1.0

                consign_dist, consign_vel_dist = self.ramp_dist.process(remaining_dist, self.odo.get_dist(), self.odo.get_dist_vel())
                out_dist_error_mm = consign_dist - self.odo.get_dist()

                out_theta_error_deg = TrajectoryHelper.normalize_theta_deg(target_theta - self.odo.get_theta())

                # if theta_error is big we ramp the consign, else we locked the target_theta no need to ramp
                if abs(out_theta_error_deg) > 3.0:
                    # clamp the theta_error to a max value to force a minimum arc radius when turning to next point
                    if out_theta_error_deg > self.waypointxy_max_theta_error:
                        out_theta_error_deg = self.waypointxy_max_theta_error
                    elif out_theta_error_deg < -self.waypointxy_max_theta_error:
                        out_theta_error_deg = -self.waypointxy_max_theta_error

                    consign_theta, consign_vel_theta = self.ramp_theta.process(out_theta_error_deg, self.odo.get_theta(), self.odo.get_theta_vel())
                    out_theta_error_deg = TrajectoryHelper.normalize_theta_deg(consign_theta - self.odo.get_theta())

                telemetry.send("consign_dist", consign_dist)
                telemetry.send("consign_vel_dist", consign_vel_dist)
                telemetry.send("out_theta_error_deg", out_theta_error_deg)
                telemetry.send("remaining_dist", remaining_dist)

                # no need to also check ramp_theta.has_force_braked() because both are force braked when blocked or timeout
                if not self.ramp_dist.has_force_braked():
                    if abs(out_dist_error_mm) >= self.velparams_dist.blocked_error:
                        self.blocked_counter_dist += 1
                    else:
                        self.blocked_counter_dist = 0

                    if abs(out_theta_error_deg) >= self.velparams_theta.blocked_error:
                        self.blocked_counter_theta += 1
                    else:
                        self.blocked_counter_theta = 0

                    if (
                        self.blocked_counter_dist >= self.velparams_dist.blocked_counter
                        or self.blocked_counter_theta >= self.velparams_theta.blocked_counter
                    ):
                        self.ramp_dist.force_brake()
                        self.ramp_theta.force_brake()
                        self._trajectory_error(MotionError.BLOCKED)
                    elif trajectory_timeout:
                        self.ramp_dist.force_brake()
                        self.ramp_theta.force_brake()
                        self._trajectory_error(MotionError.TIMEOUT)

                if self.ramp_dist.is_finished_by_force_brake() and self.ramp_theta.is_finished_by_force_brake():
                    self.logger.info(
                        "WAYPOINT_XY done by force brake (remaining_dist:%.1f theta_error_deg:%.1f)",
                        remaining_dist,
                        out_theta_error_deg,
                    )
                    self.setpoints.distance_mm = self.odo.get_dist()
                    self.setpoints.theta_deg = self.odo.get_theta()
                    self.setpoints.x_mm = self.odo.get_x()
                    self.setpoints.y_mm = self.odo.get_y()
                    self._waypoint_at_goal()
                    self._trajectory_finished()
                elif abs(remaining_dist) < 5.0 and self.waypointxy_has_more_waypoints_to_come:
                    self.setpoints.distance_mm = consign_dist
                    self.setpoints.theta_deg = self.odo.get_theta() + out_theta_error_deg
                    self._waypoint_at_goal()
                elif self.ramp_dist.is_finished() and not self.waypointxy_has_more_waypoints_to_come:
                    self.setpoints.distance_mm = self.odo.get_dist() + remaining_dist
                    self.setpoints.theta_deg = self.odo.get_theta() + out_theta_error_deg
                    self._waypoint_at_goal()
                    self._trajectory_finished()

            case MotionState.DISABLED:
                return (0.0, 0.0)

        # RCVA ALTERNATIVE PID FOR DISTANCE
        # pwm_dist = 750.0 * dist_error_mm
        # pwm_dist += 20.0 * (consign_vel_dist/2.0 - self.odo.get_dist_vel())
        pwm_dist = self.pid_dist.compute(out_dist_error_mm, self.odo.get_dist_vel())
        pwm_theta = self.pid_theta.compute(out_theta_error_deg, self.odo.get_theta_vel())

        # HACK temporary hack, see comment in the __init__
        pwm_dist = self.pwmdist_limiter.update(pwm_dist)
        pwm_theta = self.pwmtheta_limiter.update(pwm_theta)

        pwm_right = pwm_dist + pwm_theta
        pwm_left = -pwm_dist + pwm_theta

        telemetry.send_str("type", self.state.name)
        telemetry.send("setpoints.distance_mm", self.setpoints.distance_mm)
        telemetry.send("setpoints.theta_deg", self.setpoints.theta_deg)
        telemetry.send("setpoints.x_mm", self.setpoints.x_mm)
        telemetry.send("setpoints.y_mm", self.setpoints.y_mm)

        telemetry.send("dist_error_mm", out_dist_error_mm)
        telemetry.send("theta_error_deg", out_theta_error_deg)
        telemetry.send("get_dist_vel", self.odo.get_dist_vel())
        telemetry.send("get_theta_vel", self.odo.get_theta_vel())
        # telemetry.send("x", self.odo.get_x())
        # telemetry.send("y", self.odo.get_y())
        telemetry.send("theta", self.odo.get_theta())
        telemetry.send("dist", self.odo.get_dist())

        telemetry.send("pwm_dist", pwm_dist / 32767)
        telemetry.send("pwm_theta", pwm_theta / 32767)
        # telemetry.send("pwm_left", pwm_left/32767)
        # telemetry.send("pwm_right", pwm_right/32767)

        telemetry.send_str("theta_state", self.ramp_theta.state.name)
        telemetry.send_str("dist_state", self.ramp_dist.state.name)

        return (pwm_left, pwm_right)
