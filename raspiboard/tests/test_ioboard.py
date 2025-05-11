import time
import can
from colorama import Fore, Style

import tests.can_bus_test as can_bus_test
from robot.boards.ioboard import IOBoard, StepperMotionResult
from robot.can_identifiers import CANIDS
from robot.utils import setup_logging

STEPS_PER_REV = 200 * 8
ACCEL = STEPS_PER_REV * 50
MAX_VEL = int(STEPS_PER_REV * 4.5)
STEPPER_ID = 4
TOR_ID = 15
TOR_STATE_TO_END_HOMING = False

INVALID_STEPPER_ID = [-1, 5, 8]


def set_can_filter(bus: can.BusABC):
    # filters IDs from 0x200 to 0x2FF
    bus.set_filters([{"can_id": 0x200, "can_mask": 0x700, "extended": False}])


class IOBoardIntegrationTests(can_bus_test.CanBusTest):
    def setUp(self):
        # {CANIDS.CANID_IO_STATUS, CANIDS.CANID_IO_ALIVE}
        super().setUp()
        set_can_filter(self.bus)
        self.ioboard = IOBoard(self.bus)
        self.notifier.add_listener(self.ioboard)

    def tearDown(self):
        self.ioboard.reboot()
        self.notifier.remove_listener(self.ioboard)
        super().tearDown()

    def return_to_zero(self):
        self.ioboard.goto_abs(STEPPER_ID, 0, ACCEL, MAX_VEL)
        self.ioboard.wait_motion_finished(STEPPER_ID)

    def test_01_reboot(self):
        self.assertTrue(self.ioboard.reboot())
        self.assertCanMessageReceived([CANIDS.CANID_IO_ALIVE], [True], timeout=2)
        self.assertCanMessageReceived([CANIDS.CANID_IO_ALIVE], [False], timeout=2)

    def test_02_alive(self):
        for i in range(3):
            self.assertCanMessageReceived([CANIDS.CANID_IO_ALIVE], timeout=2)

    def test_03_status(self):
        for i in range(5):
            self.assertCanMessageReceived([CANIDS.CANID_IO_STATUS])

    def test_04_disable(self):
        self.assertTrue(self.ioboard.enable(False))
        self.assertCanMessageReceived([CANIDS.CANID_IO_STATUS], expected_can_data=[False])

    def test_05_enable(self):
        self.assertTrue(self.ioboard.enable(True))
        self.assertCanMessageReceived([CANIDS.CANID_IO_STATUS], expected_can_data=[True])

    def test_06_goto(self):
        self.return_to_zero()

        for dir in [1, -1]:
            self.assertEqual(
                self.ioboard.goto_abs(STEPPER_ID, STEPS_PER_REV * 4 * dir, ACCEL, MAX_VEL),
                StepperMotionResult.IS_DOING_GOTO,
            )
            self.assertEqual(self.ioboard.wait_motion_finished(STEPPER_ID), StepperMotionResult.GOTO_FINISHED)

    def test_07_goto_same_position(self):
        self.return_to_zero()

        for pos in [STEPS_PER_REV * 4, 0, -STEPS_PER_REV * 4]:
            self.assertEqual(self.ioboard.goto_abs(STEPPER_ID, pos, ACCEL, MAX_VEL), StepperMotionResult.IS_DOING_GOTO)
            self.assertEqual(self.ioboard.wait_motion_finished(STEPPER_ID), StepperMotionResult.GOTO_FINISHED)

            # these same position must finish immediately as the position has not changed
            for i in range(5):
                self.assertEqual(
                    self.ioboard.goto_abs(STEPPER_ID, pos, ACCEL, MAX_VEL), StepperMotionResult.IS_DOING_GOTO
                )
                self.assertEqual(
                    self.ioboard.wait_motion_finished(STEPPER_ID, timeout=0.5), StepperMotionResult.GOTO_FINISHED
                )

    def test_08_goto_instant_accel(self):
        # purpose of this test is to have zero accelerations steps (very high accel, very low speed)
        for dir in [1, -1]:
            self.return_to_zero()

            pos = STEPS_PER_REV // 10
            pos *= dir

            self.assertEqual(
                self.ioboard.goto_abs(STEPPER_ID, pos, STEPS_PER_REV * 60, STEPS_PER_REV // 30),
                StepperMotionResult.IS_DOING_GOTO,
            )
            self.assertEqual(
                self.ioboard.wait_motion_finished(STEPPER_ID, timeout=10), StepperMotionResult.GOTO_FINISHED
            )

    def test_09_lot_of_little_goto(self):
        for dir in [1, -1]:
            self.return_to_zero()
            pos = 0

            for i in range(15):
                pos += (STEPS_PER_REV // 8) * dir
                self.assertEqual(
                    self.ioboard.goto_abs(STEPPER_ID, pos, ACCEL // 2, MAX_VEL), StepperMotionResult.IS_DOING_GOTO
                )
                self.assertEqual(self.ioboard.wait_motion_finished(STEPPER_ID), StepperMotionResult.GOTO_FINISHED)

        self.return_to_zero()
        pos = 0
        for i in range(15):
            pos = STEPS_PER_REV // 15
            pos = pos if i % 2 == 0 else -pos
            self.assertEqual(self.ioboard.goto_abs(STEPPER_ID, pos, ACCEL, MAX_VEL), StepperMotionResult.IS_DOING_GOTO)
            self.assertEqual(self.ioboard.wait_motion_finished(STEPPER_ID), StepperMotionResult.GOTO_FINISHED)

    def test_10_home_failed(self):
        # home 1 turns
        for dir in [1, -1]:
            self.assertEqual(
                self.ioboard.home(STEPPER_ID, STEPS_PER_REV * dir, TOR_ID, TOR_STATE_TO_END_HOMING),
                StepperMotionResult.IS_DOING_HOME,
            )
            self.assertEqual(self.ioboard.wait_motion_finished(STEPPER_ID, timeout=5), StepperMotionResult.HOME_FAILED)
            time.sleep(0.3)

    def test_11_home_at_first_step(self):
        # home at first step (inverting tor state to end homing at first step)
        self.assertEqual(
            self.ioboard.home(STEPPER_ID, STEPS_PER_REV * 3, TOR_ID, not TOR_STATE_TO_END_HOMING),
            StepperMotionResult.IS_DOING_HOME,
        )
        self.assertEqual(self.ioboard.wait_motion_finished(STEPPER_ID, timeout=0.5), StepperMotionResult.HOME_SUCCEEDED)
        time.sleep(0.3)

        # home at first step (zero steps)
        self.assertEqual(
            self.ioboard.home(STEPPER_ID, 0, TOR_ID, TOR_STATE_TO_END_HOMING), StepperMotionResult.IS_DOING_HOME
        )
        self.assertEqual(self.ioboard.wait_motion_finished(STEPPER_ID, timeout=0.5), StepperMotionResult.HOME_SUCCEEDED)
        time.sleep(0.3)

        # home at first step (zero steps and inverting tor state to end homing at first step)
        self.assertEqual(
            self.ioboard.home(STEPPER_ID, 0, TOR_ID, not TOR_STATE_TO_END_HOMING), StepperMotionResult.IS_DOING_HOME
        )
        self.assertEqual(self.ioboard.wait_motion_finished(STEPPER_ID, timeout=0.5), StepperMotionResult.HOME_SUCCEEDED)
        time.sleep(0.3)

    def test_12_home_error_not_enabled(self):
        self.assertTrue(self.ioboard.enable(False))
        self.assertEqual(
            self.ioboard.home(STEPPER_ID, STEPS_PER_REV * 3, TOR_ID, TOR_STATE_TO_END_HOMING),
            StepperMotionResult.ERROR_NOT_ENABLED,
        )
        self.assertTrue(self.ioboard.enable(True))

    def test_13_goto_error_not_enabled(self):
        self.assertTrue(self.ioboard.enable(False))
        self.assertEqual(
            self.ioboard.goto_abs(STEPPER_ID, STEPS_PER_REV * 2, ACCEL, MAX_VEL), StepperMotionResult.ERROR_NOT_ENABLED
        )
        self.assertTrue(self.ioboard.enable(True))

    def test_13_goto_goto_error_motion_in_progress(self):
        for dir in [1, -1]:
            self.return_to_zero()

            self.assertEqual(
                self.ioboard.goto_abs(STEPPER_ID, (STEPS_PER_REV * 2) * dir, ACCEL, MAX_VEL // 5),
                StepperMotionResult.IS_DOING_GOTO,
            )
            self.assertEqual(
                self.ioboard.goto_abs(STEPPER_ID, STEPS_PER_REV * 2 * dir * -1, ACCEL, MAX_VEL // 5),
                StepperMotionResult.ERROR_MOTION_IN_PROGRESS,
            )
            self.assertEqual(self.ioboard.wait_motion_finished(STEPPER_ID), StepperMotionResult.GOTO_FINISHED)

    def test_14_goto_home_error_motion_in_progress(self):
        for dir in [1, -1]:
            self.return_to_zero()

            self.assertEqual(
                self.ioboard.goto_abs(STEPPER_ID, (STEPS_PER_REV * 2) * dir, ACCEL, MAX_VEL // 5),
                StepperMotionResult.IS_DOING_GOTO,
            )
            self.assertEqual(
                self.ioboard.home(STEPPER_ID, STEPS_PER_REV * 3, TOR_ID, True),
                StepperMotionResult.ERROR_MOTION_IN_PROGRESS,
            )
            self.assertEqual(self.ioboard.wait_motion_finished(STEPPER_ID), StepperMotionResult.GOTO_FINISHED)

    def test_15_home_home_error_motion_in_progress(self):
        for dir in [1, -1]:
            self.assertEqual(
                self.ioboard.home(STEPPER_ID, (STEPS_PER_REV * 3) * dir, TOR_ID, TOR_STATE_TO_END_HOMING),
                StepperMotionResult.IS_DOING_HOME,
            )
            self.assertEqual(
                self.ioboard.home(STEPPER_ID, (STEPS_PER_REV * 3) * dir, TOR_ID, TOR_STATE_TO_END_HOMING),
                StepperMotionResult.ERROR_MOTION_IN_PROGRESS,
            )
            self.assertEqual(self.ioboard.wait_motion_finished(STEPPER_ID), StepperMotionResult.HOME_FAILED)

    def test_16_home_goto_error_motion_in_progress(self):
        for dir in [1, -1]:
            self.assertEqual(
                self.ioboard.home(STEPPER_ID, (STEPS_PER_REV * 3) * dir, TOR_ID, TOR_STATE_TO_END_HOMING),
                StepperMotionResult.IS_DOING_HOME,
            )
            self.assertEqual(
                self.ioboard.goto_abs(STEPPER_ID, STEPS_PER_REV * 2 * dir * -1, ACCEL, MAX_VEL // 5),
                StepperMotionResult.ERROR_MOTION_IN_PROGRESS,
            )
            self.assertEqual(self.ioboard.wait_motion_finished(STEPPER_ID), StepperMotionResult.HOME_FAILED)

    def test_17_goto_disable_error_disabled_during_motion(self):
        self.return_to_zero()

        self.assertEqual(
            self.ioboard.goto_abs(STEPPER_ID, STEPS_PER_REV * 2, ACCEL, MAX_VEL // 5), StepperMotionResult.IS_DOING_GOTO
        )
        time.sleep(0.5)
        self.assertTrue(self.ioboard.enable(False))
        self.assertEqual(
            self.ioboard.wait_motion_finished(STEPPER_ID), StepperMotionResult.ERROR_DISABLED_DURING_MOTION
        )
        time.sleep(0.5)
        self.assertTrue(self.ioboard.enable(True))

    def test_18_home_disable_error_disabled_during_motion(self):
        self.return_to_zero()

        self.assertEqual(
            self.ioboard.home(STEPPER_ID, STEPS_PER_REV * 3, TOR_ID, TOR_STATE_TO_END_HOMING),
            StepperMotionResult.IS_DOING_HOME,
        )
        time.sleep(0.5)
        self.assertTrue(self.ioboard.enable(False))
        self.assertEqual(
            self.ioboard.wait_motion_finished(STEPPER_ID), StepperMotionResult.ERROR_DISABLED_DURING_MOTION
        )
        time.sleep(0.5)
        self.assertTrue(self.ioboard.enable(True))

    def test_19_goto_invalid_param(self):
        self.assertEqual(
            self.ioboard.goto_abs(STEPPER_ID, STEPS_PER_REV * 3, 0, MAX_VEL), StepperMotionResult.ERROR_INVALID_PARAM
        )
        self.assertEqual(
            self.ioboard.goto_abs(STEPPER_ID, STEPS_PER_REV * 3, ACCEL, 0), StepperMotionResult.ERROR_INVALID_PARAM
        )


class IOBoardManualTests(can_bus_test.CanBusTest):
    def get_can_silent_ids(cls):
        return {CANIDS.CANID_IO_STATUS, CANIDS.CANID_IO_ALIVE}

    def setUp(self):
        super().setUp()
        set_can_filter(self.bus)
        self.ioboard = IOBoard(self.bus)
        self.ioboard.enable(True)
        self.notifier.add_listener(self.ioboard)

    def tearDown(self):
        self.ioboard.reboot()
        self.notifier.remove_listener(self.ioboard)
        super().tearDown()

    def test_01_home_succeeded(self):
        print(f"{Fore.YELLOW}HEY TESTER, YOUR INTERACTION IS REQUIRED THERE!")
        print(
            f"{Fore.YELLOW}Change the tor state for these 2 incomming home (20 turns) else this test will fail{Style.RESET_ALL}"
        )
        for i in range(5, 0, -1):
            print(f"starting in {i}..")
            time.sleep(1)

        for dir in [1, -1]:
            self.assertEqual(
                self.ioboard.home(
                    STEPPER_ID,
                    (STEPS_PER_REV * 20) * dir,
                    TOR_ID,
                    TOR_STATE_TO_END_HOMING,
                ),
                StepperMotionResult.IS_DOING_HOME,
            )
            self.assertEqual(
                self.ioboard.wait_motion_finished(STEPPER_ID, timeout=20), StepperMotionResult.HOME_SUCCEEDED
            )

            time.sleep(1)


if __name__ == "__main__":
    setup_logging()

    integration_tests = IOBoardIntegrationTests()
    integration_tests.run()

    manual_tests = IOBoardManualTests()
    manual_tests.run()
