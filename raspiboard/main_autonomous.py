import select
import threading
import time
import logging

from robot import (
    Robot,
    telemetry,
    TrajectoryManager,
    Lidar,
    Odometry,
    DistanceParams,
    ThetaParams,
    TeamColor,
    MotionFinishedState,
    RobotOrientation,
    RotationDirection,
)


class AutonomousRobot(Robot):
    def __init__(self):
        super().__init__()

        self.odometry = Odometry(self.params)
        self.lidar = Lidar(self.odometry)
        self.tm = TrajectoryManager(self.params, self.odometry)
        self.motorboard.set_status_callback(self.tm.on_status)
        self.team = TeamColor.BLUE

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
        self.tm.line(500, DistanceParams.NORMAL)
        self.tm.wait_trajectory_finished()
        time.sleep(0.2)

        self.tm.rotate(-90, ThetaParams.SLOW)
        self.tm.wait_trajectory_finished()
        time.sleep(0.2)

        self.tm.line(380, DistanceParams.NORMAL)
        self.tm.wait_trajectory_finished()
        time.sleep(0.2)

        self.tm.rotate(-90, ThetaParams.SLOW)
        self.tm.wait_trajectory_finished()
        time.sleep(0.2)

        self.set_grabber(True)

        self.tm.line(400, DistanceParams.VERY_SLOW)
        self.tm.wait_trajectory_finished()

        self.set_grabber(False)
        time.sleep(0.2)

        self.tm.line(-300, DistanceParams.VERY_SLOW)
        self.tm.wait_trajectory_finished()
        time.sleep(0.2)

        # BLOCK 2: pousse tas sur grande zone
        self.tm.rotate(-90, ThetaParams.SLOW)
        self.tm.wait_trajectory_finished()
        time.sleep(0.2)

        self.tm.line(325, DistanceParams.NORMAL)
        self.tm.wait_trajectory_finished()
        time.sleep(0.2)

        self.tm.rotate(-90, ThetaParams.SLOW)
        self.tm.wait_trajectory_finished()
        time.sleep(0.2)

        self.set_grabber(True)

        self.tm.line(280, DistanceParams.VERY_SLOW)
        self.tm.wait_trajectory_finished()
        time.sleep(0.2)

        self.tm.rotate(160, ThetaParams.VERY_SLOW)
        self.tm.wait_trajectory_finished()
        time.sleep(0.2)

        self.tm.line(280, DistanceParams.VERY_SLOW)
        self.tm.wait_trajectory_finished()

        self.set_grabber(False)
        time.sleep(0.2)

        self.tm.line(-240, DistanceParams.VERY_SLOW)
        self.tm.wait_trajectory_finished()
        time.sleep(0.2)

        # BLOCK 3: Return to backstage
        self.tm.rotate(-160, ThetaParams.VERY_SLOW)
        self.tm.wait_trajectory_finished()
        time.sleep(0.2)

        self.tm.rotate(-90, ThetaParams.SLOW)
        self.tm.wait_trajectory_finished()
        time.sleep(0.2)

        self.tm.line(720, DistanceParams.SLOW)
        self.tm.wait_trajectory_finished()
        time.sleep(0.2)

        self.tm.rotate(90, ThetaParams.SLOW)
        self.tm.wait_trajectory_finished()
        time.sleep(0.2)

        self.tm.line(650, DistanceParams.SLOW)
        self.tm.wait_trajectory_finished()
        time.sleep(0.2)

    def thread_strategy(self):
        self.logger.info("Starting strategy")

        # while True:
        #     teamcolor = input("enter team color: [b/y]: ")
        #     if teamcolor.lower() == "b":
        #         self.team = TeamColor.BLUE
        #         break
        #     elif teamcolor.lower() == "y":
        #         self.team = TeamColor.YELLOW
        #         break
        #     else:
        #         print("incorrect team")

        # self.tm.set_team(self.team)
        # self.logger.info("team selected: %s", self.team)

        # if self.gpio.is_starter_present():
        #     self.logger.warning("started is present! waiting for remove...")
        #     self.gpio.wait_for_starter_removed()

        # self.logger.info("place the robot at the start point, then insert the starter to confirm")

        # self.logger.info("waiting for starter insertion..")
        # self.gpio.wait_for_starter_inserted()

        # self.servoboard.enable_power(False, True, True)

        # self.logger.info("waiting for start..")
        # self.gpio.wait_for_starter_removed()

        # self.t_end_match.start()

        # self.tm.set_odometry(x_mm=0, y_mm=0, theta_deg=0)
        # self.logger.info("odo reset to %s", self.tm.odometry)

        self.logger.info("GO!")

        self.motorboard.reset_error()

        # self.basic_strategy_intech()

        # self.logger.info("strategy done")
        # self.stop_event.set()

        # time.sleep(100)

        # self.tm.rotate(360*10, RotateVelocity.SLOW)
        # self.tm.wait_trajectory_finished()

        # vels_theta = [ThetaParams.VERY_SLOW, ThetaParams.SLOW, ThetaParams.NORMAL, ThetaParams.FAST, ThetaParams.VERY_FAST]
        # for vel in vels_theta:
        #     self.tm.rotate(360*2, vel)
        #     if self.tm.wait_trajectory_finished() != MotionFinishedState.SUCCESS:
        #         break

        #     time.sleep(0.5)

        #     self.tm.rotate(45, vel)
        #     if self.tm.wait_trajectory_finished() != MotionFinishedState.SUCCESS:
        #         break

        #     self.tm.rotate(-45, vel)
        #     if self.tm.wait_trajectory_finished() != MotionFinishedState.SUCCESS:
        #         break

        #     time.sleep(0.5)

        # vels_dist = [DistanceParams.VERY_SLOW, DistanceParams.SLOW, DistanceParams.NORMAL, DistanceParams.FAST, DistanceParams.VERY_FAST]
        # vels_dist = [DistanceParams.NORMAL, DistanceParams.FAST, DistanceParams.VERY_FAST]
        # vels_dist = [DistanceParams.NORMAL]
        # for vel in vels_dist:
        #     self.tm.line(1000, vel)
        #     if self.tm.wait_trajectory_finished() != MotionFinishedState.SUCCESS:
        #         break
        #     time.sleep(0.5)

        #     self.tm.line(-1000, vel)
        #     if self.tm.wait_trajectory_finished() != MotionFinishedState.SUCCESS:
        #         break
        #     time.sleep(0.5)

        #     self.tm.line(50, vel)
        #     if self.tm.wait_trajectory_finished() != MotionFinishedState.SUCCESS:
        #         break

        #     self.tm.line(-50, vel)
        #     if self.tm.wait_trajectory_finished() != MotionFinishedState.SUCCESS:
        #         break
        #     time.sleep(0.5)

        # TEST LOOK_AT
        # self.tm.look_at(100, 0.1, ThetaParams.SLOW, RobotOrientation.FRONT)
        # self.tm.wait_trajectory_finished()
        # time.sleep(0.5)
        # self.tm.look_at(-100, 0, ThetaParams.SLOW, RobotOrientation.FRONT)
        # self.tm.wait_trajectory_finished()
        # time.sleep(0.5)
        # self.tm.look_at(0, 100, ThetaParams.SLOW, RobotOrientation.FRONT)
        # self.tm.wait_trajectory_finished()
        # time.sleep(0.5)
        # self.tm.look_at(0, -100, ThetaParams.SLOW, RobotOrientation.FRONT)
        # self.tm.wait_trajectory_finished()
        # time.sleep(0.5)

        # TEST GOTO_XY
        # self.tm.goto_xy(200, 0, DistanceParams.SLOW, ThetaParams.SLOW, RobotOrientation.BACK, RotationDirection.COUNTERCLOCKWISE)
        # self.tm.wait_trajectory_finished()
        # self.tm.goto_xy(400, 400, DistanceParams.SLOW, ThetaParams.SLOW, RobotOrientation.BACK, RotationDirection.COUNTERCLOCKWISE)
        # self.tm.wait_trajectory_finished()
        # self.tm.goto_xy(0, 0, DistanceParams.SLOW, ThetaParams.SLOW, RobotOrientation.BACK, RotationDirection.COUNTERCLOCKWISE)
        # self.tm.wait_trajectory_finished()
        # self.tm.look_at(1000, 0, ThetaParams.SLOW, RobotOrientation.BACK, RotationDirection.COUNTERCLOCKWISE)
        # self.tm.wait_trajectory_finished()

        # TEST WAYPOINT_XY

        for i in range(50):
            self.logger.debug("doing notin")
            time.sleep(1)

    # def strategy_coroutine(self):
    #     self.tm.waypoint_xy_start(200, 0, DistanceParams.VERY_SLOW, ThetaParams.NORMAL, RobotOrientation.FRONT)
    #     if self.tm.wait_waypoint_at_goal() != MotionFinishedState.SUCCESS:
    #         return
    #     self.tm.waypoint_xy_chained(300, 100, True)
    #     if self.tm.wait_waypoint_at_goal() != MotionFinishedState.SUCCESS:
    #         return
    #     self.tm.waypoint_xy_chained(200, 200, True)
    #     if self.tm.wait_waypoint_at_goal() != MotionFinishedState.SUCCESS:
    #         return
    #     self.tm.waypoint_xy_chained(100, 200)
    #     if self.tm.wait_waypoint_at_goal() != MotionFinishedState.SUCCESS:
    #         return
    #     time.sleep(0.5)
    #     self.tm.goto_xy(0, 0, DistanceParams.SLOW, ThetaParams.SLOW)
    #     if self.tm.wait_trajectory_finished() != MotionFinishedState.SUCCESS:
    #         return
    #     self.tm.look_at(1000, 0, ThetaParams.SLOW)
    #     if self.tm.wait_trajectory_finished() != MotionFinishedState.SUCCESS:
    #         return

    # def generator_test_waypoints(self):
    #     if (yield self.tm.line(100, DistanceParams.SLOW)):
    #         return

    #     if (yield self.tm.waypoint_xy_start(200, 0, DistanceParams.VERY_SLOW, ThetaParams.NORMAL, RobotOrientation.FRONT)):
    #         return
    #     if (yield self.tm.waypoint_xy_chained(300, 100, True)):
    #         return
    #     if (yield self.tm.waypoint_xy_chained(200, 200, True)):
    #         return
    #     if (yield self.tm.waypoint_xy_chained(100, 200)):
    #         return

    #     yield self.time_provider.wait(0.5)

    # def action_start(self):
    # WE ARE AT start position

    # abaisser ascenseur arrière
    # servo ascenseur arrière horizontal
    # WAIT servos

    # waypointxy_start a SLOW
    # waypointxy_chained b SLOW
    # waypointxy_chained c SLOW
    # WAIT WAYPOINT

    # def action_recup_stock(self, stock, recallage_bordure: bool):
    # WE ARE AT stock entrypoint

    # lookat stock origin
    # preparer baguettes
    # preparer aimants
    # WAIT servos

    # if recallage_bordure
    # HOME 300 FRONT
    # WAIT HOME
    # else
    # line 300 SLOW
    # WAIT line

    # activer les pompes
    # remonter baguettes en prise
    # WAIT servos

    # line -100 SLOW_FASTDECEL (permet s'assurer que les planches sont bien)
    # baisser servo prise planche
    # WAIT line

    # def action_recup_stock_error(self):
    # desactiver pompes
    # ranger en bas les baguettes
    # ranger servo prise planche
    # WAIT servos

    # def action_construire_stock(self, construction_slot):
    # WE ARE AT slot entrypoint

    # WAIT GOTOXY
    # lookat slot origin
    # WAIT lookat

    # bras ascenseurs position exterieur
    # line 200 SLOW
    # WAIT servos
    # monter ascenseur
    # WAIT steppers
    # bras ascenseurs position defaut
    # WAIT servos
    # WAIT line

    # baguettes en bas
    # WAIT servos

    # line -200 SLOW
    # WAIT line

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

            pwm_left, pwm_right = self.tm.process()

            pwm_left = int(pwm_left)
            pwm_right = int(pwm_right)

            can_err = self.motorboard.pwm_write(pwm_left, pwm_right)
            if not can_err:
                self.logger.critical("ESTOP! CAN iz dead")
                self.stop_event.set()

            telemetry.send("elapsed_time", last_elapsed_time)

            # try our best to execute the function exactly at CONTROLLOOP_FREQ_HZ
            elapsed_time = time.monotonic() - start_time
            last_elapsed_time = elapsed_time * 1000.0
            timeout = max(0, self.params.CONTROLLOOP_PERIOD - elapsed_time)
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
