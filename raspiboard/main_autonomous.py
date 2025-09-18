import select
import time
import logging
from typing import Generator

from robot import (
    Robot,
    telemetry,
    TrajectoryManager,
    Lidar,
    Odometry,
    DistanceParams,
    ThetaParams,
    TeamColor,
    RobotOrientation,
)

from robot_config_2025 import RobotConfig2025


def exception_on_error(gen) -> Generator[None, None, None]:
    err = yield from gen
    if err:
        raise RuntimeError(err)


def delay(duration: float):
    start = time.monotonic()
    print(f"delay duration:{duration} start..", end="", flush=True)
    while time.monotonic() - start < duration:
        yield
    print("DONE!")


class AutonomousRobot(Robot):
    def __init__(self):
        self.config = RobotConfig2025()
        super().__init__(self.config)

        self.odometry = Odometry(self.config)
        self.lidar = Lidar(self.odometry)
        self.tm = TrajectoryManager(self.config, self.odometry)
        self.motorboard.set_status_callback(self.tm.on_status)
        self.team = TeamColor.BLUE

        self.control_loop_period = self.config.get_controlloop_period()

        self.match_end_time: float | None = None

        self.tasks = [self.action_test_waypoint(), self.action_ne_rien_faire()]
        self.current_task = self.tasks.pop(0)

        self.logger = logging.getLogger(self.__class__.__name__)

    def action_ne_rien_faire(self) -> Generator[None, None, None]:
        print("== action_ne_rien_faire ==")
        while True:
            yield

    def action_preparation_match(self) -> Generator[None, None, None]:
        print("== test_action ==")
        # aligner le robot sur le trait de couleur extérieur

        if not self.gpio.is_starter_present():
            self.logger.info("waiting for starter insertion..")
            yield from self.gpio.wait_until_starter_inserted()
            self.logger.info("starter inserted!")

        # disable avoidance

        self.motorboard.reset_error()

        self.tm.home(RobotOrientation.FRONT)
        yield from exception_on_error(self.tm.wait())
        # TODO odo reset there

        self.tm.line(-200, DistanceParams.VERY_SLOW)
        yield from exception_on_error(self.tm.wait())

        self.tm.rotate(90, ThetaParams.VERY_SLOW)
        yield from exception_on_error(self.tm.wait())

        self.tm.line(100, DistanceParams.VERY_SLOW)
        yield from exception_on_error(self.tm.wait())

        self.tm.rotate(90, ThetaParams.VERY_SLOW)
        yield from exception_on_error(self.tm.wait())

        yield from delay(1.0)

        self.tm.line(100, DistanceParams.VERY_SLOW)
        yield from exception_on_error(self.tm.wait())

        # self.tm.home(RobotOrientation.FRONT)
        # yield from exception_on_error(tm.wait())

        self.logger.info("ready to go on starter..")
        yield from self.gpio.wait_until_starter_removed()
        self.logger.info("GO!")

        self.match_end_time = time.monotonic() + 30.0

    def action_retour_a_position_initiale(self):
        self.tm.goto_xy(0, 0, DistanceParams.VERY_SLOW, ThetaParams.VERY_SLOW)
        yield from exception_on_error(self.tm.wait())

        self.tm.look_at(1000, 0, ThetaParams.VERY_SLOW)
        yield from exception_on_error(self.tm.wait())

    def action_test_waypoint(self):
        self.motorboard.reset_error()

        self.tm.waypoint_xy_start(400, 0, DistanceParams.VERY_SLOW, ThetaParams.SLOW)
        yield from exception_on_error(self.tm.wait_waypoint())

        self.tm.waypoint_xy_chained(700, 200, True)
        yield from exception_on_error(self.tm.wait_waypoint())

        self.tm.waypoint_xy_chained(600, 500, True)
        yield from exception_on_error(self.tm.wait_waypoint())

        self.tm.waypoint_xy_chained(200, 0, False)
        yield from exception_on_error(self.tm.wait_waypoint())

        self.tm.goto_xy(0, 0)
        yield from exception_on_error(self.tm.wait())

        self.tm.look_at(1000, 0)
        yield from exception_on_error(self.tm.wait())

    def run(self):
        self.start()
        self.lidar.start()

        last_elapsed_time = 0.0

        while not self.stop_event.is_set():
            # main critical loop, the elapsed_time for this loop MUST strictly be under 5ms
            t = time.monotonic()

            if self.match_end_time is not None:
                if t >= self.match_end_time:
                    self.logger.info("match finished !")
                    time.sleep(self.control_loop_period * 2)
                    break

            pwm_left, pwm_right = self.tm.process(t)
            pwm_left = int(pwm_left)
            pwm_right = int(pwm_right)
            can_err = self.motorboard.pwm_write(pwm_left, pwm_right)
            if not can_err:
                self.logger.critical("ESTOP! CAN iz dead")
                self.stop_event.set()

            self.process(t)

            # task next step
            try:
                next(self.current_task)
            except StopIteration:
                if self.tasks:
                    self.current_task = self.tasks.pop(0)
                else:
                    print("all tasks done!")
                    break
            except Exception:
                break

            telemetry.send("elapsed_time", last_elapsed_time)
            if last_elapsed_time > 5.0:
                self.logger.warning("elapsed time moar than 5ms! danger zone!")

            # try our best to execute the function exactly at CONTROLLOOP_FREQ_HZ
            elapsed_time = time.monotonic() - t
            last_elapsed_time = elapsed_time * 1000.0
            timeout = max(0, self.control_loop_period - elapsed_time)
            select.select([], [], [], timeout)

        self.logger.debug("done main loop")

        self.servoboard.enable_power(False, False, False)
        self.ioboard.enable(False)

        self.lidar.stop()
        self.stop()


if __name__ == "__main__":
    robot = AutonomousRobot()
    robot.run()
