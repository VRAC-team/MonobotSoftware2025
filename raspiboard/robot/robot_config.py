from typing import Protocol

from .boards import Servo


class RobotConfig(Protocol):
    CONTROLLOOP_FREQUENCY: int  # [Hz]

    # Odometry
    ODOMETRY_WHEEL_PERIMETER: float  # [mm]
    ODOMETRY_WHEEL_SPACING: float  # [mm]
    ODOMETRY_TICKS_PER_REV: int  # [ticks/rev]

    # TrajectoryManager
    BLOCKED_TOTALTIME_COEF: float  # multiply the estimated time

    FORCEBRAKE_DIST_DECEL: float  # [mm/s]
    FORCEBRAKE_THETA_DECEL: float  # [deg/s]

    THETA_FINISHED_WINDOW: float  # [deg]
    DIST_FINISHED_WINDOW: float  # [mm]

    WAYPOINT_XY_MIN_RADIUS: float  # [mm]

    # IOBoard
    STEPPER_STEPS_PER_REV: int  # [steps/rev]

    # PumpBoard
    PUMP_AUTO_CLOSE_VALVE_AFTER: float  # [s]

    # GPIO
    GPIO_START: int
    GPIO_SHUTDOWN: int

    # Telemetry
    TELEMETRY_HOST_ADDR: tuple[str, int]  # hostname, port of the Teleplot on host device

    # Robot Actuators
    SERVOS: dict[int, Servo]

    def get_controlloop_period(self) -> float:
        return 1.0 / self.CONTROLLOOP_FREQUENCY
