import gpiod
import datetime
from typing import Generator

from .robot_config import RobotConfig


class GPIO:
    def __init__(self, config: RobotConfig):
        self.config = config

        self.chip = gpiod.Chip("/dev/gpiochip0")
        self.lines_req = gpiod.request_lines(
            "/dev/gpiochip0",
            consumer="thread_watch",
            config={
                (self.config.GPIO_START, self.config.GPIO_SHUTDOWN): gpiod.LineSettings(
                    direction=gpiod.line.Direction.INPUT,
                    active_low=True,
                    bias=gpiod.line.Bias.DISABLED,
                    edge_detection=gpiod.line.Edge.BOTH,
                    debounce_period=datetime.timedelta(milliseconds=10),
                )
            },
        )

    def is_starter_present(self) -> bool:
        return bool(self.lines_req.get_value(self.config.GPIO_START))

    def wait_until_starter_inserted(self) -> Generator[None, None, None]:
        while not self.is_starter_present():
            yield

    def wait_until_starter_removed(self) -> Generator[None, None, None]:
        while self.is_starter_present():
            yield

    def is_shutdown_pressed(self) -> bool:
        return bool(self.lines_req.get_value(self.config.GPIO_SHUTDOWN))
