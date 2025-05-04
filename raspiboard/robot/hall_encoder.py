import threading
from robot.filters import ThresholdFilter


class HallEncoder:
    TICKS_PER_REVOLUTION = 16384
    TICKS_PER_REVOLUTION_FIRST_QUARTER = TICKS_PER_REVOLUTION / 4
    TICKS_PER_REVOLUTION_LAST_QUARTER = 3 * TICKS_PER_REVOLUTION / 4

    def __init__(self):
        self.last_reading_ticks = 0
        self.total_ticks = 0
        self.lock = threading.Lock()
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
        if (
            self.last_reading_ticks < self.TICKS_PER_REVOLUTION_FIRST_QUARTER
            and current_reading_ticks > self.TICKS_PER_REVOLUTION_LAST_QUARTER
        ):
            delta -= self.TICKS_PER_REVOLUTION
        elif (
            self.last_reading_ticks > self.TICKS_PER_REVOLUTION_LAST_QUARTER
            and current_reading_ticks < self.TICKS_PER_REVOLUTION_FIRST_QUARTER
        ):
            delta += self.TICKS_PER_REVOLUTION

        self.last_reading_ticks = current_reading_ticks

        if self.first_measure:
            self.first_measure = False
            return 0

        with self.lock:
            self.total_ticks += delta
            self.filter.update(self.total_ticks)

        return delta

    def get(self):
        return self.filter.get()
