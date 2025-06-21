from dataclasses import dataclass
import math
import enum


@dataclass(frozen=True)
class RobotParameters:
    CONTROLLOOP_FREQUENCY: int = 200  # [Hz]
    CONTROLLOOP_PERIOD: float = 1.0 / CONTROLLOOP_FREQUENCY  # [s]

    ODOMETRY_WHEEL_PERIMETER: float = 52.42 * math.pi  # [mm]
    ODOMETRY_WHEEL_SPACING: float = 256.5  # [mm]
    ODOMETRY_TICKS_PER_REV: int = 16384  # [ticks/rev]

    BLOCKED_TOTALTIME_COEF: float = 3.0  # multiply the estimated time

    FORCEBRAKE_DIST_DECEL: float = 3000  # [mm/s]
    FORCEBRAKE_THETA_DECEL: float = 1500  # [deg/s]

    THETA_FINISHED_WINDOW: float = 0.5  # [deg]
    DIST_FINISHED_WINDOW: float = 1  # [mm]

    WAYPOINT_XY_MIN_RADIUS: float = 15.0  # [mm]

    STEPPER_STEPS_PER_REV: int = 200 * 8  # [steps/rev]

    GPIO_START = 5
    GPIO_SHUTDOWN = 6


class TeamColor(enum.Enum):
    BLUE = "blue"
    YELLOW = "yellow"
