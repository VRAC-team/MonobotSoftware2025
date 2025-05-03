import can_utils as can_utils
import can_test_utils as can_test_utils
from canids import CANIDS
import time
import unittest
import colorama
from boards.ioboard import IOBoard
from colorama import Fore, Style

colorama.init(autoreset=True)
bus = can_utils.get_can_interface()
# filters IDs from 0x200 to 0x2FF
bus.set_filters([{
    "can_id": 0x200,
    "can_mask": 0x700,
    "extended": False
}])
ioboard = IOBoard(bus)

STEPS_PER_REV = 200 * 8
ACCEL = STEPS_PER_REV * 50
MAX_VEL = int(STEPS_PER_REV * 4.5)
STEPPER_ID = 4
TOR_ID = 15
TOR_STATE_TO_END_HOMING = False

class IOBoardTests(can_test_utils.CanBusTestCase):
    @classmethod
    def get_can_silent_ids(cls):
        return {
            CANIDS.CANID_IO_STATUS,
            CANIDS.CANID_IO_ALIVE
        }

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
    
    @classmethod
    def tearDownClass(cls):
        # disable power after the tests
        ioboard.enable(False)
        super().tearDownClass()

    def return_to_zero(self):
        ioboard.goto_abs(STEPPER_ID, 0, ACCEL, MAX_VEL)
        self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_STARTING])
        self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])

    def test_00_reboot(self):
        ioboard.reboot()
        self.assertCanMessageReceived([CANIDS.CANID_IO_ALIVE], [True], timeout=2)
        self.assertCanMessageReceived([CANIDS.CANID_IO_ALIVE], [False], timeout=2)
    
    def test_01_alive(self):
        for i in range(3):
            self.assertCanMessageReceived([CANIDS.CANID_IO_ALIVE], timeout=2)

    def test_02_status(self):
        for i in range(5):
            self.assertCanMessageReceived([CANIDS.CANID_IO_STATUS], timeout=1)

    def test_03_disable(self):
        ioboard.enable(False)

    def test_04_enable(self):
        ioboard.enable(True)

    def test_05_goto(self):
        self.return_to_zero()

        for dir in [1, -1]:
            ioboard.goto_abs(STEPPER_ID, STEPS_PER_REV*4 * dir, ACCEL, MAX_VEL)
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_STARTING])
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])

    def test_106_goto_same_position(self):
        self.return_to_zero()

        for pos in [STEPS_PER_REV*4, 0, -STEPS_PER_REV*4]:
            ioboard.goto_abs(STEPPER_ID, pos, ACCEL, MAX_VEL)
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_STARTING])
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED], timeout=5)

            # these same position must finish immediately as the position has not changed
            for i in range(5):
                ioboard.goto_abs(STEPPER_ID, pos, ACCEL, MAX_VEL)
                self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_STARTING])
                self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED], timeout=0.1)

    
    def test_07_goto_instant_accel(self):
        # purpose of this test is to have zero accelerations steps (very high accel, very low speed)
        for dir in [1, -1]:
            self.return_to_zero()

            pos = STEPS_PER_REV//10
            pos *= dir
            print("pos:", pos)

            ioboard.goto_abs(STEPPER_ID, pos, STEPS_PER_REV*60, STEPS_PER_REV//30)
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_STARTING])
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])

    def test_08_lot_of_little_goto(self):
        for dir in [1, -1]:
            self.return_to_zero()
            pos = 0

            for i in range(8):
                pos += (STEPS_PER_REV//8) * dir
                ioboard.goto_abs(STEPPER_ID, pos, ACCEL//2, MAX_VEL)
                self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_STARTING])
                self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])

        self.return_to_zero()
        pos = 0
        for i in range(15):
            pos = STEPS_PER_REV//15
            pos = pos if i % 2 == 0 else -pos
            ioboard.goto_abs(STEPPER_ID, pos, ACCEL, MAX_VEL)
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_STARTING])
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])

    def test_09_home_failed(self):
        # home 1 turns
        for dir in [1, -1]:
            ioboard.home(STEPPER_ID, STEPS_PER_REV * dir, TOR_ID, TOR_STATE_TO_END_HOMING)
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_HOME_STARTING])
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_HOME_FAILED])
            time.sleep(0.3)

    def test_10_home_succeeded(self):
        print(f"{Fore.YELLOW}HEY TESTER, YOUR INTERACTION IS REQUIRED THERE!")
        print(f"{Fore.YELLOW}Change the tor state for these 2 incomming home (20 turns) else this test will fail")
        for i in range(5, 0, -1):
            print(f"starting in {i}..")
            time.sleep(1)

        for dir in [1, -1]:
            ioboard.home(STEPPER_ID, (STEPS_PER_REV*20) * dir, TOR_ID, TOR_STATE_TO_END_HOMING)
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_HOME_STARTING])
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED])
            time.sleep(0.3)

    def test_11_home_at_first_step(self):
        # home at first step (inverting tor state to end homing at first step)
        ioboard.home(STEPPER_ID, STEPS_PER_REV*3, TOR_ID, not TOR_STATE_TO_END_HOMING)
        self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_HOME_STARTING])
        self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED])
        time.sleep(0.3)

        # home at first step (zero steps)
        ioboard.home(STEPPER_ID, 0, TOR_ID, TOR_STATE_TO_END_HOMING)
        self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_HOME_STARTING])
        self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED])
        time.sleep(0.3)

        # home at first step (zero steps and inverting tor state to end homing at first step)
        ioboard.home(STEPPER_ID, 0, TOR_ID, not TOR_STATE_TO_END_HOMING)
        self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_HOME_STARTING])
        self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED])
        time.sleep(0.3)
    
    def test_12_goto_goto_error_motion_in_progress(self):
        for dir in [1, -1]:
            self.return_to_zero()

            ioboard.goto_abs(STEPPER_ID, (STEPS_PER_REV*2) * dir, ACCEL, MAX_VEL//5)
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_STARTING])
            time.sleep(0.5)
            ioboard.goto_abs(STEPPER_ID, STEPS_PER_REV*2 * dir * -1, ACCEL, MAX_VEL//5)
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_ERROR_MOTION_IN_PROGRESS])
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])            

    def test_13_goto_home_error_motion_in_progress(self):
        for dir in [1, -1]:
            self.return_to_zero()

            ioboard.goto_abs(STEPPER_ID, (STEPS_PER_REV*2) * dir, ACCEL, MAX_VEL//5)
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_STARTING])
            time.sleep(0.5)
            ioboard.home(STEPPER_ID, STEPS_PER_REV*3, TOR_ID, True)
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_ERROR_MOTION_IN_PROGRESS])
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])

    def test_14_home_home_error_motion_in_progress(self):
        for dir in [1, -1]:
            ioboard.home(STEPPER_ID, (STEPS_PER_REV*3) * dir, TOR_ID, TOR_STATE_TO_END_HOMING)
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_HOME_STARTING])
            ioboard.home(STEPPER_ID, (STEPS_PER_REV*3) * dir, TOR_ID, TOR_STATE_TO_END_HOMING)
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_ERROR_MOTION_IN_PROGRESS])
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED, CANIDS.CANID_IO_STEPPER_HOME_FAILED])
            time.sleep(0.3)

    def test_15_home_goto_error_motion_in_progress(self):
        for dir in [1, -1]:
            ioboard.home(STEPPER_ID, (STEPS_PER_REV*3) * dir, TOR_ID, TOR_STATE_TO_END_HOMING)
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_HOME_STARTING])
            ioboard.goto_abs(STEPPER_ID, (STEPS_PER_REV*2) * dir, ACCEL, MAX_VEL//5)
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_ERROR_MOTION_IN_PROGRESS])
            self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED, CANIDS.CANID_IO_STEPPER_HOME_FAILED])
            time.sleep(0.3)
    
    def test_16_disable_goto_error_not_enabled(self):
        self.return_to_zero()

        ioboard.enable(False)
        ioboard.goto_abs(STEPPER_ID, STEPS_PER_REV*2, ACCEL, MAX_VEL)
        self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_ERROR_NOT_ENABLED])

        ioboard.home(STEPPER_ID, STEPS_PER_REV*3, TOR_ID, TOR_STATE_TO_END_HOMING)
        self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_ERROR_NOT_ENABLED])

        ioboard.enable(True)

    def test_17_goto_disable_error_disabled_during_motion(self):
        self.return_to_zero()

        ioboard.goto_abs(STEPPER_ID, STEPS_PER_REV*2, ACCEL, MAX_VEL//3)
        self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_GOTO_STARTING])
        time.sleep(0.5)
        ioboard.enable(False)
        self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_ERROR_DISABLED_DURING_MOTION])

        ioboard.enable(True)

    def test_18_home_disable_error_disabled_during_motion(self):
        self.return_to_zero()

        ioboard.home(STEPPER_ID, STEPS_PER_REV*3, TOR_ID, TOR_STATE_TO_END_HOMING)
        self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_HOME_STARTING])
        time.sleep(0.5)
        ioboard.enable(False)
        self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_ERROR_DISABLED_DURING_MOTION])

        ioboard.enable(True)

    def test_19_goto_invalid_args(self):
        ioboard.goto_abs(STEPPER_ID, STEPS_PER_REV*3, 0, MAX_VEL)
        self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_ERROR_INVALID_PARAMS])

        ioboard.goto_abs(STEPPER_ID, STEPS_PER_REV*3, ACCEL, 0)
        self.assertCanMessageReceived([CANIDS.CANID_IO_STEPPER_ERROR_INVALID_PARAMS])

if __name__ == '__main__':
    IOBoardTests.bus = bus
    suite = unittest.TestLoader().loadTestsFromTestCase(IOBoardTests)
    runner = can_test_utils.CustomRunner()
    runner.run(suite)
    bus.shutdown()