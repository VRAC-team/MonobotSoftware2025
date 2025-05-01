import utils
from canids import CANIDS
import threading
import time
import can
import struct
import queue
import unittest
import colorama
import random
from colorama import Fore, Style

class IOBoard:
    def reboot():
        msg = can.Message(arbitration_id=CANIDS.CANID_IO_REBOOT)
        bus.send(msg)

    def enable(state: bool):
        msg = can.Message(arbitration_id=CANIDS.CANID_IO_STEPPER_ENABLE, data=[state])
        bus.send(msg)


    def home(stepper_id: int, max_relative_steps_before_error: int, tor_id: int, tor_state_to_end_homing: bool):
        data = bytearray(
            stepper_id.to_bytes(1) +
            max_relative_steps_before_error.to_bytes(2, signed=True) +
            tor_id.to_bytes(1) +
            tor_state_to_end_homing.to_bytes(1)
        )
        msg = can.Message(arbitration_id=CANIDS.CANID_IO_STEPPER_HOME, data=data)
        bus.send(msg)

    def goto_abs(stepper_id: int, absolute_steps: int, acceleleration: int, max_velocity: int):
        data = bytearray(
            stepper_id.to_bytes(1) +
            absolute_steps.to_bytes(2, signed=True) +
            acceleleration.to_bytes(3) +
            max_velocity.to_bytes(2)
        )
        msg = can.Message(arbitration_id=CANIDS.CANID_IO_STEPPER_GOTO, data=data)
        bus.send(msg)

colorama.init(autoreset=True)
bus = utils.get_can_interface()

STEPS_PER_REV = 200 * 8
ACCEL = STEPS_PER_REV * 50
MAX_VEL = int(STEPS_PER_REV * 4.5)

class VerboseTestResult(unittest.TextTestResult):
    def startTest(self, test):
        self._start_time = time.time()
        test_name = self.getDescription(test)
        print(f"\nStarting {test_name} ... ")

    def addSuccess(self, test):
        elapsed = time.time() - self._start_time
        print(f"{Fore.GREEN}PASSED{Style.RESET_ALL} in {elapsed:.2f}s")
        super().addSuccess(test)

    def addFailure(self, test, err):
        elapsed = time.time() - self._start_time
        print(f"{Fore.RED}FAILED{Style.RESET_ALL} in {elapsed:.2f}s")
        super().addFailure(test, err)

    def addError(self, test, err):
        elapsed = time.time() - self._start_time
        print(f"{Fore.MAGENTA}ERROR{Style.RESET_ALL} in {elapsed:.2f}s")
        super().addError(test, err)

class CustomRunner(unittest.TextTestRunner):
    def __init__(self, **kwargs):
        super().__init__(resultclass=VerboseTestResult, **kwargs)

class IOBoardTests(unittest.TestCase):
    can_thread = None
    can_thread_running = False
    can_messages = queue.Queue()

    @classmethod
    def setUpClass(cls):
        cls.can_thread_running = True
        cls.can_thread = threading.Thread(target=cls._can_read_loop, daemon=True)
        cls.can_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.can_thread_running = False
        cls.can_thread.join(timeout=2)

    @classmethod
    def _can_read_loop(cls):
        try:
            while cls.can_thread_running:
                for msg in bus:
                    if msg.is_error_frame:
                        print("CAN ERROR >", msg)

                    elif msg.arbitration_id == CANIDS.CANID_IO_STATUS:
                        tors, = struct.unpack(">H", msg.data)
                        # print(f"status: {tors:016b}")
                    
                    elif msg.arbitration_id == CANIDS.CANID_IO_ALIVE:
                        just_rebooted, = struct.unpack(">?", msg.data)
                        # print(f"alive: just_rebooted:{just_rebooted}")

                    else:
                        frame_name = CANIDS.get_name(msg.arbitration_id)
                        print("CAN > ", frame_name, " : ", msg)

                    cls.can_messages.put(msg)
        except can.exceptions.CanOperationError as e:
            print(e)

    def assertCanIdReceived(
        self,
        expected_can_id: list[int],
        expected_can_data: bytes | list[int] | None = None,
        timeout: float = 30
    ) -> bool:
        deadline = time.time() + timeout

        while time.time() < deadline:
            if not self.can_messages.empty():
                msg = self.can_messages.get(timeout=timeout)

                if msg.arbitration_id in expected_can_id:
                    if expected_can_data is not None:
                        data = bytes(expected_can_data) if isinstance(expected_can_data, list) else expected_can_data
                        if msg.data != data:
                            continue
                    return True
        
        self.fail(f"CAN ID {expected_can_id} was not received within {timeout} seconds.")

    def run(self, result=None):
        super_result = super().run(result)
        time.sleep(0.2)
        return super_result

    def return_to_zero(self):
        IOBoard.goto_abs(4, 0, ACCEL, MAX_VEL)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])

    def test_01_reboot_and_alive(self):
        IOBoard.reboot()
        self.assertCanIdReceived([CANIDS.CANID_IO_ALIVE], [True])
        self.assertCanIdReceived([CANIDS.CANID_IO_ALIVE], [False])

    def test_02_status(self):
        self.assertCanIdReceived([CANIDS.CANID_IO_STATUS])
        self.assertCanIdReceived([CANIDS.CANID_IO_STATUS])
        self.assertCanIdReceived([CANIDS.CANID_IO_STATUS])

    def test_03_disable(self):
        IOBoard.enable(False)

    def test_04_enable(self):
        IOBoard.enable(True)

    def test_05_goto(self):
        self.return_to_zero()

        IOBoard.goto_abs(4, STEPS_PER_REV*4, ACCEL, MAX_VEL)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])
    
        IOBoard.goto_abs(4, -STEPS_PER_REV*4, ACCEL, MAX_VEL)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])

    def test_06_goto_same_position(self):
        self.return_to_zero()

        pos = STEPS_PER_REV*4
        IOBoard.goto_abs(4, pos, ACCEL, MAX_VEL)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])
        IOBoard.goto_abs(4, pos, ACCEL, MAX_VEL)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])

        pos = 0
        IOBoard.goto_abs(4, pos, ACCEL, MAX_VEL)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])
        IOBoard.goto_abs(4, pos, ACCEL, MAX_VEL)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])

    # purpose of this test is to have zero accel/decel steps
    def test_07_goto_instant_accel(self):
        self.return_to_zero()

        IOBoard.goto_abs(4, STEPS_PER_REV//8, STEPS_PER_REV*60, STEPS_PER_REV//30)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])

        self.return_to_zero()

        IOBoard.goto_abs(4, -STEPS_PER_REV//8, STEPS_PER_REV*60, STEPS_PER_REV//30)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])

    def test_08_lot_of_little_goto(self):
        for dir in [1, -1]:
            self.return_to_zero()
            pos = 0

            for i in range(8):
                pos += (STEPS_PER_REV//8) * dir
                IOBoard.goto_abs(4, pos, ACCEL//2, MAX_VEL)
                self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])

        self.return_to_zero()
        pos = 0
        for i in range(15):
            pos = STEPS_PER_REV//15
            pos = pos if i % 2 == 0 else -pos
            IOBoard.goto_abs(4, pos, ACCEL, MAX_VEL)
            self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])

    def test_09_home(self):
        # home 1 turns
        IOBoard.home(4, STEPS_PER_REV, 15, False)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED, CANIDS.CANID_IO_STEPPER_HOME_FAILED])
        time.sleep(0.3)
        IOBoard.home(4, -STEPS_PER_REV, 15, False)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED, CANIDS.CANID_IO_STEPPER_HOME_FAILED])

        # home 5 turns
        IOBoard.home(4, STEPS_PER_REV*5, 15, False)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED, CANIDS.CANID_IO_STEPPER_HOME_FAILED])
        time.sleep(0.3)
        IOBoard.home(4, -STEPS_PER_REV*5, 15, False)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED, CANIDS.CANID_IO_STEPPER_HOME_FAILED])

    def test_10_home_at_first_step(self):
        # home at first step (tor is True by default)
        IOBoard.home(4, STEPS_PER_REV*3, 15, True)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED])
        time.sleep(0.3)

        # home at first step (zero steps)
        IOBoard.home(4, 0, 15, False)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED])
        time.sleep(0.3)

        # home at first step (zero steps and tor is True by default)
        IOBoard.home(4, 0, 15, True)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED])
        time.sleep(0.3)
    
    def test_11_goto_goto_error_motion_in_progress(self):
        for dir in [1, -1]:
            self.return_to_zero()

            IOBoard.goto_abs(4, STEPS_PER_REV*2 * dir, ACCEL, MAX_VEL//5)
            time.sleep(0.5)
            IOBoard.goto_abs(4, STEPS_PER_REV*2 * dir * -1, ACCEL, MAX_VEL//5)
            self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_ERROR_MOTION_IN_PROGRESS])
            self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])            

    def test_12_goto_home_error_motion_in_progress(self):
        for dir in [1, -1]:
            self.return_to_zero()

            IOBoard.goto_abs(4, STEPS_PER_REV*2 * dir, ACCEL, MAX_VEL//5)
            time.sleep(0.5)
            IOBoard.home(4, STEPS_PER_REV*3, 15, True)
            self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_ERROR_MOTION_IN_PROGRESS])
            self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_GOTO_FINISHED])

    def test_13_home_home_error_motion_in_progress(self):
        for dir in [1, -1]:
            IOBoard.home(4, STEPS_PER_REV*3 * dir, 15, False)
            IOBoard.home(4, STEPS_PER_REV*3 * dir, 15, False)
            self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_ERROR_MOTION_IN_PROGRESS])
            self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED, CANIDS.CANID_IO_STEPPER_HOME_FAILED])
            time.sleep(0.3)

    def test_14_home_goto_error_motion_in_progress(self):
        for dir in [1, -1]:
            IOBoard.home(4, STEPS_PER_REV*3 * dir, 15, False)
            IOBoard.goto_abs(4, STEPS_PER_REV*2 * dir, ACCEL, MAX_VEL//5)
            self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_ERROR_MOTION_IN_PROGRESS])
            self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED, CANIDS.CANID_IO_STEPPER_HOME_FAILED])
            time.sleep(0.3)
    
    def test_15_disable_goto_error_not_enabled(self):
        self.return_to_zero()

        IOBoard.enable(False)
        IOBoard.goto_abs(4, STEPS_PER_REV*2, ACCEL, MAX_VEL)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_ERROR_NOT_ENABLED])

        IOBoard.home(4, STEPS_PER_REV*3, 15, False)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_ERROR_NOT_ENABLED])

        IOBoard.enable(True)

    def test_16_goto_disable_error_disabled_during_motion(self):
        self.return_to_zero()

        IOBoard.goto_abs(4, STEPS_PER_REV*2, ACCEL, MAX_VEL//3)
        time.sleep(0.5)
        IOBoard.enable(False)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_ERROR_DISABLED_DURING_MOTION])

        IOBoard.enable(True)

    def test_17_home_disable_error_disabled_during_motion(self):
        self.return_to_zero()

        IOBoard.home(4, STEPS_PER_REV*3, 15, False)
        time.sleep(0.5)
        IOBoard.enable(False)
        self.assertCanIdReceived([CANIDS.CANID_IO_STEPPER_ERROR_DISABLED_DURING_MOTION])

        IOBoard.enable(True)

if __name__ == '__main__':
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(IOBoardTests)
    runner = CustomRunner(verbosity=0)
    runner.run(suite)

    bus.shutdown()
