import enum
import math
import logging
import time
import threading
import dataclasses

from robot.parameters import RobotParameters, TeamColor
from robot.velocity_ramp import VelocityRamp
from robot.hall_encoder import HallEncoder
from robot.odometry import Odometry
from robot.pid import PID_RCVA
from robot.filters import RampFilter
from robot.telemetry import telemetry


@dataclasses.dataclass(frozen=True)
class Velocity:
    accel: float
    decel: float
    max_vel: float

    blocked_error: float  # [mm or deg] if error (consign-current) is greather than this, increment internal counter, else reset the counter
    blocked_counter: int = (
        3  # if the internal counter is greater than this, emergency brake and consider the motion as blocked
    )

    def __str__(self) -> str:
        return f"Velocity(acc={self.accel:.1f} dec={self.decel:.1f} vel={self.max_vel:.1f})"


class DistanceParams(enum.Enum):
    VERY_SLOW = Velocity(300, 300, 150, blocked_error=200.0)
    SLOW = Velocity(1000, 1000, 300, blocked_error=200.0)
    NORMAL = Velocity(2500, 2900, 600, blocked_error=200.0)
    FAST = Velocity(2600, 2900, 800, blocked_error=200.0)
    VERY_FAST = Velocity(2600, 2900, 1200, blocked_error=200.0)

    def __str__(self) -> str:
        return f"DistanceParams.{self.name}({self.value})"


class ThetaParams(enum.Enum):
    VERY_SLOW = Velocity(200, 200, 100, blocked_error=6 + 2)
    SLOW = Velocity(800, 800, 200, blocked_error=12 + 2)
    NORMAL = Velocity(1500, 1200, 300, blocked_error=18 + 3)
    FAST = Velocity(1500, 1200, 500, blocked_error=29 + 5)
    VERY_FAST = Velocity(1500, 1200, 600, blocked_error=35 + 5)

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
    ROTATE = "rotate"
    LOOK_AT = "look_at"
    GOTO_XY_PHASE1_LOOK_AT = "goto_xy_phase1_look_at"
    GOTO_XY_PHASE2_LINE_TO = "goto_xy_phase2_line_to"
    WAYPOINT_XY = "waypoint_xy"
    HOME = "home"

    DISABLED = "disabled"


class MotionFinishedState(enum.Enum):
    SUCCESS = "success"
    BLOCKED = "blocked"
    TIMEOUT = "timeout"

    TIMEOUT_NO_WAYPOINT_RECEIVED = "timeout_no_waypoint_received"


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
        print("current_theta:", current_theta)
        print("target_front:", target_front)
        print("delta_front:", delta_front)
        print("delta_back:", delta_back)

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
    def __init__(self, params: RobotParameters, odometry: Odometry):
        self.params = params
        self.odo = odometry
        self.team = TeamColor.BLUE

        self.encoder_left = HallEncoder(params)
        self.encoder_right = HallEncoder(params)

        # self.pid_dist = PID(kp=650.0, ki=0, kd=40.0, frequency=params.CONTROLLOOP_FREQUENCY) # aggresif 650 40
        # self.pid_theta = PID(kp=2000.0, ki=0.0, kd=130.0, frequency=params.CONTROLLOOP_FREQUENCY) # aggresif 2000 130

        self.pid_dist = PID_RCVA(kp=800, kd=0.25, frequency=params.CONTROLLOOP_FREQUENCY)  # 1000 0.28
        self.pid_theta = PID_RCVA(kp=1800, kd=0.35, frequency=params.CONTROLLOOP_FREQUENCY)  # 2000 0.35

        self.ramp_theta = VelocityRamp(self.params.CONTROLLOOP_PERIOD, self.params.FORCEBRAKE_THETA_DECEL)
        self.ramp_dist = VelocityRamp(self.params.CONTROLLOOP_PERIOD, self.params.FORCEBRAKE_DIST_DECEL)

        # HACK this is a hack because there is discontinuities in the velocityramp pos/vel consign between maxvel/decel
        pwm_accel = 0.6 * 32767.0
        self.pwmtheta_limiter = RampFilter(self.params.CONTROLLOOP_PERIOD, pwm_accel, pwm_accel)
        self.pwmdist_limiter = RampFilter(self.params.CONTROLLOOP_PERIOD, pwm_accel, pwm_accel)

        self.params_dist = DistanceParams.NORMAL
        self.params_theta = ThetaParams.NORMAL

        self.state = MotionState.STAY_AT_POSITION
        self.setpoints = Setpoints()
        self.trajectory_finished = threading.Event()

        self.blocked_counter_theta = 0
        self.blocked_counter_dist = 0

        self.logger = logging.getLogger(self.__class__.__name__)

        # used by other than STAY_AT_POSITION
        self.motion_start_time = 0.0
        self.motion_timeout_after_duration = 0.0
        self.motion_finished_state = MotionFinishedState.SUCCESS

        # gotoxy specific
        self.gotoxy_rotation_direction = RotationDirection.AUTO
        self.gotoxy_robot_orientation = RobotOrientation.FRONT

        # waypointxy specific
        self.waypointxy_robot_orientation = RobotOrientation.FRONT
        self.waypointxy_max_theta_error = float("inf")
        self.waypointxy_has_more_waypoints_to_come = False
        self.waypointxy_atgoal = threading.Event()
        self.waypointxy_atgoal_wait_next = threading.Event()

        # home specific
        self.home_robot_orientation = RobotOrientation.BACK

    def on_status(self, state_error: bool, enc_left: int, enc_right: int) -> None:
        """
        This must be called by the motorboard callback (from on_message_received from the python-can Notifier thread)
        """
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

    def line(self, distance_mm: float, params: DistanceParams = DistanceParams.NORMAL) -> bool:
        if self.state != MotionState.STAY_AT_POSITION:
            self.logger.error("asked for line but state is not STAY_AT_POSITION (state:%s)", self.state)
            return False

        p = params.value
        estimated_time = self.ramp_dist.start(distance_mm, p.accel, p.decel, p.max_vel, self.setpoints.distance_mm)

        self.setpoints.distance_mm += distance_mm
        self.setpoints.x_mm += distance_mm * math.cos(math.radians(self.setpoints.theta_deg))
        self.setpoints.y_mm += distance_mm * math.sin(math.radians(self.setpoints.theta_deg))
        self.params_dist = p
        self._trajectory_start(MotionState.LINE, estimated_time)

        self.logger.debug(
            "LINE start (distance_mm:%.1f params:%s estimated_time:%.2f)", distance_mm, params.name, estimated_time
        )

        return True

    def rotate(self, theta_deg: float, params: ThetaParams = ThetaParams.NORMAL) -> bool:
        if self.state != MotionState.STAY_AT_POSITION:
            self.logger.error("asked for rotate but state is not STAY_AT_POSITION (state:%s)", self.state)
            return False

        if self.team == TeamColor.YELLOW:
            theta_deg = -theta_deg

        p = params.value
        estimated_time = self.ramp_theta.start(theta_deg, p.accel, p.decel, p.max_vel, self.setpoints.theta_deg)

        self.setpoints.theta_deg += theta_deg
        self.params_theta = p
        self._trajectory_start(MotionState.ROTATE, estimated_time)

        self.logger.debug(
            "ROTATE start (theta_deg:%.1f params:%s estimated_time:%.2f)", theta_deg, params.name, estimated_time
        )

        return True

    def look_at(
        self,
        x: float,
        y: float,
        params: ThetaParams = ThetaParams.NORMAL,
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
        self.params_theta = p
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
        params_dist: DistanceParams = DistanceParams.NORMAL,
        params_theta: ThetaParams = ThetaParams.NORMAL,
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
        estimated_time_theta = self.ramp_theta.start(
            theta_deg, p_theta.accel, p_theta.decel, p_theta.max_vel, self.setpoints.theta_deg
        )
        estimated_time_dist = self.ramp_dist.start(
            distance, p_dist.accel, p_dist.decel, p_dist.max_vel, self.setpoints.distance_mm
        )
        estimated_time = estimated_time_theta + estimated_time_dist

        self.gotoxy_robot_orientation = orientation_computed
        self.gotoxy_rotation_direction = rotation_direction
        self.setpoints.theta_deg += theta_deg
        self.setpoints.x_mm = x
        self.setpoints.y_mm = y
        self.params_theta = p_theta
        self.params_dist = p_dist
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

    def waypoint_xy_chained(self, x: float, y: float, has_more_waypoints_to_come: bool = False) -> bool:
        """
        when chained with a previous waypoint, this must be called less than a CONTROLLOOP_PERIOD after wait_waypoint_at_goal else it will error
        """
        if self.state != MotionState.WAYPOINT_XY:
            self.logger.error("can only by chained with a previous WAYPOINT_XY (state:%s)", self.state)
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

        end_velocity = self.params_dist.max_vel if has_more_waypoints_to_come else 0.0
        start_velocity = self.params_dist.max_vel
        estimated_time_dist = self.ramp_dist.start(
            distance_mm,
            self.params_dist.accel,
            self.params_dist.decel,
            self.params_dist.max_vel,
            self.setpoints.distance_mm,
            start_velocity,
            end_velocity,
        )
        estimated_time_theta = self.ramp_theta.start(
            theta_deg,
            self.params_theta.accel,
            self.params_theta.decel,
            self.params_theta.max_vel,
            self.setpoints.theta_deg,
            self.odo.get_theta_vel(),
        )
        estimated_time = estimated_time_dist + estimated_time_theta  # * 0.7

        self.waypointxy_has_more_waypoints_to_come = has_more_waypoints_to_come
        self.setpoints.x_mm = x
        self.setpoints.y_mm = y
        self._trajectory_start(MotionState.WAYPOINT_XY, estimated_time)

        if has_more_waypoints_to_come:
            self.waypointxy_atgoal_wait_next.set()

        self.logger.debug(
            "WAYPOINT_XY chained (x:%.1f y:%.1f has_more_waypoints_to_come:%s estimated_time:%.2f)",
            x,
            y,
            has_more_waypoints_to_come,
            estimated_time,
        )

    def waypoint_xy_start(
        self,
        x: float,
        y: float,
        params_dist: DistanceParams = DistanceParams.NORMAL,
        params_theta: ThetaParams = ThetaParams.NORMAL,
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
        estimated_time_theta = self.ramp_theta.start(
            theta_deg, p_theta.accel, p_theta.decel, p_theta.max_vel, self.setpoints.theta_deg
        )
        estimated_time = estimated_time_dist + estimated_time_theta  # * 0.7

        self.waypointxy_robot_orientation = robot_orientation
        self.waypointxy_max_theta_error = math.degrees(p_theta.max_vel / self.params.WAYPOINT_XY_MIN_RADIUS)
        self.waypointxy_has_more_waypoints_to_come = True
        self.waypointxy_atgoal.clear()
        self.waypointxy_atgoal_wait_next.clear()

        self.setpoints.x_mm = x
        self.setpoints.y_mm = y
        self.params_theta = p_theta
        self.params_dist = p_dist
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

    def wait_waypoint_at_goal(self):
        self.waypointxy_atgoal.wait()
        self.waypointxy_atgoal.clear()
        return self.motion_finished_state

    def _waypoint_at_goal(self):
        self.waypointxy_atgoal.set()
        total_time = time.monotonic() - self.motion_start_time
        self.logger.debug("_waypoint_at_goal (total_time:%.2f)", total_time)

    def wait_trajectory_finished(self) -> MotionFinishedState:
        self.trajectory_finished.wait()
        return self.motion_finished_state

    def _trajectory_start(self, state: MotionState, estimated_time: float):
        self.state = state
        self.trajectory_finished.clear()
        self.blocked_counter_dist = 0
        self.blocked_counter_theta = 0
        self.motion_timeout_after_duration = estimated_time * self.params.BLOCKED_TOTALTIME_COEF
        self.motion_start_time = time.monotonic()
        self.motion_finished_state = MotionFinishedState.SUCCESS

    def _trajectory_error(self, error_state: MotionFinishedState):
        self.motion_finished_state = error_state
        match error_state:
            case MotionFinishedState.TIMEOUT:
                self.logger.error(
                    "_trajectory_error TIMEOUT %s (timeout_after_duration:%.2f)",
                    self.state.name,
                    self.motion_timeout_after_duration,
                )
            case MotionFinishedState.BLOCKED:
                self.logger.error(
                    "_trajectory_error BLOCKED %s (blocked_counter_dist:%d blocked_counter_theta:%d)",
                    self.state.name,
                    self.blocked_counter_dist,
                    self.blocked_counter_theta,
                )
            case MotionFinishedState.TIMEOUT_NO_WAYPOINT_RECEIVED:
                self.logger.error("_trajectory_error TIMEOUT_NO_WAYPOINT_RECEIVED %s", self.state.name)

    def _trajectory_finished(self):
        total_time = time.monotonic() - self.motion_start_time
        self.logger.debug(
            "_trajectory_finished %s %s (total_time:%.2f)", self.state.name, self.motion_finished_state.name, total_time
        )
        self.state = MotionState.STAY_AT_POSITION
        self.trajectory_finished.set()

    def process(self) -> tuple[float, float]:
        out_dist_error_mm = 0.0
        out_theta_error_deg = 0.0

        # HACK temporary to use RCVA modified PID
        consign_vel_dist = 0.0

        trajectory_timeout = False
        current_time = time.monotonic()
        if current_time - self.motion_start_time > self.motion_timeout_after_duration:
            trajectory_timeout = True

        match self.state:
            case MotionState.STAY_AT_POSITION:
                out_dist_error_mm = self.setpoints.distance_mm - self.odo.get_dist()
                out_theta_error_deg = self.setpoints.theta_deg - self.odo.get_theta()

            case MotionState.LINE:
                remaining_dist = self.setpoints.distance_mm - self.odo.get_dist()
                consign_dist, consign_vel_dist = self.ramp_dist.process(remaining_dist, self.odo.get_dist())

                out_dist_error_mm = consign_dist - self.odo.get_dist()
                out_theta_error_deg = self.setpoints.theta_deg - self.odo.get_theta()

                if not self.ramp_dist.has_force_braked():
                    if abs(out_dist_error_mm) >= self.params_dist.blocked_error:
                        self.blocked_counter_dist += 1
                    else:
                        self.blocked_counter_dist = 0

                    if self.blocked_counter_dist >= self.params_dist.blocked_counter:
                        self.ramp_dist.force_brake()
                        self._trajectory_error(MotionFinishedState.BLOCKED)
                    elif trajectory_timeout:
                        self.ramp_dist.force_brake()
                        self._trajectory_error(MotionFinishedState.TIMEOUT)

                if self.ramp_dist.is_finished_by_force_brake() and abs(self.odo.get_dist_vel()) < 1.0:
                    self.logger.info("LINE done by force brake (remaining:%.1fmm)", remaining_dist)
                    self.setpoints.theta_deg = self.odo.get_theta()
                    self.setpoints.distance_mm = self.odo.get_dist()
                    self.setpoints.x_mm = self.odo.get_x()
                    self.setpoints.y_mm = self.odo.get_y()
                    self._trajectory_finished()
                elif self.ramp_dist.is_finished() or abs(remaining_dist) < 0.4:
                    self._trajectory_finished()

            case MotionState.ROTATE | MotionState.LOOK_AT | MotionState.GOTO_XY_PHASE1_LOOK_AT:
                remaining_theta = self.setpoints.theta_deg - self.odo.get_theta()
                consign_theta, consign_vel_theta = self.ramp_theta.process(remaining_theta, self.odo.get_theta())

                telemetry.send("consign_theta", consign_theta)
                telemetry.send("consign_vel_theta", consign_vel_theta)

                out_dist_error_mm = self.setpoints.distance_mm - self.odo.get_dist()
                out_theta_error_deg = consign_theta - self.odo.get_theta()

                if not self.ramp_theta.has_force_braked():
                    if abs(out_theta_error_deg) >= self.params_theta.blocked_error:
                        self.blocked_counter_theta += 1
                    else:
                        self.blocked_counter_theta = 0

                    if self.blocked_counter_theta >= self.params_theta.blocked_counter:
                        self.ramp_theta.force_brake()
                        self._trajectory_error(MotionFinishedState.BLOCKED)
                    elif trajectory_timeout:
                        self.ramp_theta.force_brake()
                        self._trajectory_error(MotionFinishedState.TIMEOUT)

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

                consign_dist, consign_vel_dist = self.ramp_dist.process(remaining_dist, self.odo.get_dist())
                out_dist_error_mm = consign_dist - self.odo.get_dist()

                if not self.ramp_dist.has_force_braked():
                    if abs(out_dist_error_mm) >= self.params_dist.blocked_error:
                        self.blocked_counter_dist += 1
                    else:
                        self.blocked_counter_dist = 0

                    if self.blocked_counter_dist >= self.params_dist.blocked_counter:
                        self.ramp_dist.force_brake()
                        self._trajectory_error(MotionFinishedState.BLOCKED)
                    elif trajectory_timeout:
                        self.ramp_dist.force_brake()
                        self._trajectory_error(MotionFinishedState.TIMEOUT)

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

                consign_dist, consign_vel_dist = self.ramp_dist.process(remaining_dist, self.odo.get_dist())
                out_dist_error_mm = consign_dist - self.odo.get_dist()

                telemetry.send("consign_dist", consign_dist)
                telemetry.send("consign_vel_dist", consign_vel_dist)

                out_theta_error_deg = TrajectoryHelper.normalize_theta_deg(target_theta - self.odo.get_theta())

                # if theta_error is big we ramp the consign, else we locked the target_theta no need to ramp
                if abs(out_theta_error_deg) > 3.0:
                    # clamp the theta_error to a max value to force a minimum arc radius when turning to next point
                    if out_theta_error_deg > self.waypointxy_max_theta_error:
                        out_theta_error_deg = self.waypointxy_max_theta_error
                    elif out_theta_error_deg < -self.waypointxy_max_theta_error:
                        out_theta_error_deg = -self.waypointxy_max_theta_error

                    consign_theta, consign_vel_theta = self.ramp_theta.process(
                        out_theta_error_deg, self.odo.get_theta()
                    )
                    out_theta_error_deg = TrajectoryHelper.normalize_theta_deg(consign_theta - self.odo.get_theta())

                print("remaining_dist", remaining_dist, "out_theta_error_deg", out_theta_error_deg)

                if (
                    not self.ramp_dist.has_force_braked()
                ):  # no need to also check ramp_theta.has_force_braked() because both are force braked when blocked or timeout
                    if abs(out_dist_error_mm) >= self.params_dist.blocked_error:
                        self.blocked_counter_dist += 1
                    else:
                        self.blocked_counter_dist = 0

                    if abs(out_theta_error_deg) >= self.params_theta.blocked_error:
                        self.blocked_counter_theta += 1
                    else:
                        self.blocked_counter_theta = 0

                    if (
                        self.blocked_counter_dist >= self.params_dist.blocked_counter
                        or self.blocked_counter_theta >= self.params_theta.blocked_counter
                    ):
                        self.ramp_dist.force_brake()
                        self.ramp_theta.force_brake()
                        self._trajectory_error(MotionFinishedState.BLOCKED)
                    elif trajectory_timeout:
                        self.ramp_dist.force_brake()
                        self.ramp_theta.force_brake()
                        self._trajectory_error(MotionFinishedState.TIMEOUT)

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
                elif abs(remaining_dist) < 5.0 and self.waypointxy_has_more_waypoints_to_come:
                    self.setpoints.distance_mm = consign_dist
                    self.setpoints.theta_deg = self.odo.get_theta() + out_theta_error_deg
                    self._waypoint_at_goal()

                    # timed_out = self.waypointxy_atgoal_wait_next.wait(self.params.CONTROLLOOP_PERIOD*5.0) # the timeout doesn't seems precise at all
                    timeout_t1 = time.monotonic() + self.params.CONTROLLOOP_PERIOD * 2.0
                    while not self.waypointxy_atgoal_wait_next.is_set():
                        if time.monotonic() > timeout_t1:
                            self.ramp_dist.force_brake()
                            self.ramp_theta.force_brake()
                            self.logger.error(
                                "did not received the next waypoint within a CONTROLLOOP_PERIOD ! => force braking"
                            )
                            self._trajectory_error(MotionFinishedState.TIMEOUT_NO_WAYPOINT_RECEIVED)
                            self.state = MotionState.DISABLED
                            break
                    self.waypointxy_atgoal_wait_next.clear()
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
        # telemetry.send("setpoints.distance_mm", self.setpoints.distance_mm)
        # telemetry.send("setpoints.theta_deg", self.setpoints.theta_deg)

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
