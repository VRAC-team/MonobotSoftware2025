from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RobotParameters:
    CONTROLLOOP_FREQ_HZ: int = 200
    CONTROLLOOP_PERIOD_S: float = 1.0 / CONTROLLOOP_FREQ_HZ

    ODOMETRY_WHEEL_PERIMETER_MM: float = 52.42 * math.pi
    ODOMETRY_WHEEL_SPACING_MM: float = 258.5
    ODOMETRY_TICKS_PER_REV: int = 16384

    STEPPER_STEPS_PER_REV: int = 200 * 8

    GPIO_START = 5
    GPIO_SHUTDOWN = 6
