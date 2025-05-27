import select
import threading
import time
import logging

from robot import Robot, telemetry, TrajectoryManager, Lidar, Odometry, DistanceVelocity, RotateVelocity, TeamColor


class AutonomousRobot(Robot):
    def __init__(self):
        super().__init__()

        self.odometry = Odometry(self.params)
        self.lidar = Lidar(self.odometry)
        self.tm = TrajectoryManager(self.params, self.odometry, self.lidar, self.stop_event)
        self.motorboard.set_status_callback(self.tm.on_status)
        self.team = TeamColor.YELLOW

        self.t_strategy = threading.Thread(target=self.thread_strategy, daemon=True)

        self.event_end_match = threading.Event()
        self.t_end_match = threading.Thread(target=self.thread_end_match, daemon=True)

        self.logger = logging.getLogger(self.__class__.__name__)

    def set_grabber(self, grab: bool):
        self.servoboard.servo_write_angle(8, 0 if grab else 180)
        self.servoboard.servo_write_angle(9, 180 if grab else 0)
        self.servoboard.servo_write_angle(10, 0 if grab else 180)
        self.servoboard.servo_write_angle(11, 180 if grab else 0)

    def thread_end_match(self):
        start_time = time.monotonic()
        while not self.stop_event.is_set():
            elapsed_time = time.monotonic() - start_time
            if elapsed_time >= 100.0:
                self.event_end_match.set()
                break

            time.sleep(0.5)

    def basic_strategy_intech(self):
        # BLOCK 1: pousse tas devant sur petite zone
        self.tm.line(500, DistanceVelocity.NORMAL)
        self.tm.wait_motion_finished()
        time.sleep(0.2)

        self.tm.rotate(-90, RotateVelocity.SLOW)
        self.tm.wait_motion_finished()
        time.sleep(0.2)

        self.tm.line(380, DistanceVelocity.NORMAL)
        self.tm.wait_motion_finished()
        time.sleep(0.2)

        self.tm.rotate(-90, RotateVelocity.SLOW)
        self.tm.wait_motion_finished()
        time.sleep(0.2)

        self.set_grabber(True)

        self.tm.line(400, DistanceVelocity.VERY_SLOW)
        self.tm.wait_motion_finished()

        self.set_grabber(False)
        time.sleep(0.2)

        self.tm.line(-300, DistanceVelocity.VERY_SLOW)
        self.tm.wait_motion_finished()
        time.sleep(0.2)

        # BLOCK 2: pousse tas sur grande zone
        self.tm.rotate(-90, RotateVelocity.SLOW)
        self.tm.wait_motion_finished()
        time.sleep(0.2)

        self.tm.line(325, DistanceVelocity.NORMAL)
        self.tm.wait_motion_finished()
        time.sleep(0.2)

        self.tm.rotate(-90, RotateVelocity.SLOW)
        self.tm.wait_motion_finished()
        time.sleep(0.2)

        self.set_grabber(True)

        self.tm.line(280, DistanceVelocity.VERY_SLOW)
        self.tm.wait_motion_finished()
        time.sleep(0.2)

        self.tm.rotate(160, RotateVelocity.VERY_SLOW)
        self.tm.wait_motion_finished()
        time.sleep(0.2)

        self.tm.line(280, DistanceVelocity.VERY_SLOW)
        self.tm.wait_motion_finished()

        self.set_grabber(False)
        time.sleep(0.2)

        self.tm.line(-240, DistanceVelocity.VERY_SLOW)
        self.tm.wait_motion_finished()
        time.sleep(0.2)

        # BLOCK 3: Return to backstage
        self.tm.rotate(-160, RotateVelocity.VERY_SLOW)
        self.tm.wait_motion_finished()
        time.sleep(0.2)

        self.tm.rotate(-90, RotateVelocity.SLOW)
        self.tm.wait_motion_finished()
        time.sleep(0.2)

        self.tm.line(720, DistanceVelocity.SLOW)
        self.tm.wait_motion_finished()
        time.sleep(0.2)

        self.tm.rotate(90, RotateVelocity.SLOW)
        self.tm.wait_motion_finished()
        time.sleep(0.2)

        self.tm.line(650, DistanceVelocity.SLOW)
        self.tm.wait_motion_finished()
        time.sleep(0.2)

    def thread_strategy(self):
        self.logger.info("Starting strategy")

        while True:
            teamcolor = input("enter team color: [b/y]: ")
            if teamcolor.lower() == "b":
                self.team = TeamColor.BLUE
                break
            elif teamcolor.lower() == "y":
                self.team = TeamColor.YELLOW
                break
            else:
                print("incorrect team, retrying..")

        self.tm.set_team(self.team)
        self.logger.info("team selected: %s", self.team)

        if self.gpio.is_starter_present():
            self.logger.warning("started is present! waiting for remove...")
            self.gpio.wait_for_starter_removed()

        self.logger.info("place the robot at the start point, then insert the starter to confirm")

        self.logger.info("waiting for starter insertion..")
        self.gpio.wait_for_starter_inserted()

        self.servoboard.enable_power(False, True, True)

        self.logger.info("waiting for start..")
        self.gpio.wait_for_starter_removed()

        self.t_end_match.start()

        self.tm.set_odometry(x_mm=1500, y_mm=1000, theta_deg=90)
        self.logger.info("odo reset to %s", self.tm.odometry)

        self.logger.info("GO!")

        self.motorboard.reset_error()

        # self.basic_strategy_intech()

        self.logger.info("strategy done")
        self.stop_event.set()

    def run(self):
        self.start()
        self.lidar.start()
        self.t_strategy.start()

        last_elapsed_time = 0.0

        while not self.stop_event.is_set():
            # main critical loop, the elapsed_time for this loop MUST strictly be under 5ms
            start_time = time.monotonic()

            if self.event_end_match.is_set():
                self.motorboard.pwm_write(0, 0)
                self.logger.info("match finished, 100s timer !")
                break

            pwm_left, pwm_right = self.tm.compute()
            can_err = self.motorboard.pwm_write(pwm_left, pwm_right)
            if not can_err:
                self.logger.critical("BAU, CAN dead")
                self.stop_event.set()

            telemetry.send("elapsed_time", last_elapsed_time)

            # try our best to execute the function exactly at CONTROLLOOP_FREQ_HZ
            elapsed_time = time.monotonic() - start_time
            last_elapsed_time = elapsed_time * 1000.0
            timeout = max(0, self.params.CONTROLLOOP_PERIOD_S - elapsed_time)
            select.select([], [], [], timeout)

        self.servoboard.enable_power(False, False, False)
        self.ioboard.enable(False)

        self.lidar.stop()
        self.t_strategy.join(timeout=1)
        self.t_end_match.join(timeout=1)
        self.stop()


if __name__ == "__main__":
    robot = AutonomousRobot()
    robot.run()
