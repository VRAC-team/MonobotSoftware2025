from robot.filters import ThresholdFilter
from robot.parameters import RobotParameters


class HallEncoder:
    """
    IMPORTANT:
    To ensure encoder rollover can be reliably detected, the call to `update()` rate MUST be high enough.
    Specifically, you must update at least 4 times per full wheel revolution at the robot's maximum speed.
    Given:
    - D = wheel diameter (in meters)
    - S = maximum robot speed (in meters/second)
    The maximum safe polling period (in seconds) is:
    min_period = (pi * (D / 4)) / S
    Example:
        D = 0.052 m, S = 1.2 m/s
        min_period = (pi * 0.052 / 4) / 1.2 = 0.034 seconds (34 ms)
    """

    def __init__(self, params: RobotParameters):
        self.params = params
        self.ticks_first_quarter = self.params.ODOMETRY_TICKS_PER_REV / 4
        self.ticks_last_quarter = (3 * self.params.ODOMETRY_TICKS_PER_REV) / 4

        self.last_reading_ticks = 0
        self.total_ticks = 0
        self.first_measure = True
        self.filter = ThresholdFilter(3)

    def reset(self):
        self.last_reading_ticks = 0
        self.total_ticks = 0
        self.first_measure = True
        self.filter.reset()

    def update(self, current_reading_ticks: int):
        delta = current_reading_ticks - self.last_reading_ticks

        # handle encoder rollover/rolldown
        if self.last_reading_ticks < self.ticks_first_quarter and current_reading_ticks > self.ticks_last_quarter:
            delta -= self.params.ODOMETRY_TICKS_PER_REV
        elif self.last_reading_ticks > self.ticks_last_quarter and current_reading_ticks < self.ticks_first_quarter:
            delta += self.params.ODOMETRY_TICKS_PER_REV

        self.last_reading_ticks = current_reading_ticks

        if self.first_measure:
            self.first_measure = False
            return 0

        self.total_ticks += delta
        self.filter.update(self.total_ticks)

        return delta

    def get(self):
        return self.filter.get()
