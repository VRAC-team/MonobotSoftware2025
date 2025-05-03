import unittest
import time
import queue
import threading
from canids import CANIDS
from colorama import Fore, Style
import can

class CustomTestResult(unittest.TextTestResult):
    def startTestRun(self):
        super().startTestRun()

    def stopTestRun(self):
        super().stopTestRun()

    def startTest(self, test):
        self._start_time = time.time()
        test_name = self.getDescription(test)
        self.stream.write(f"\n{Fore.CYAN}Starting {test_name}:{Style.RESET_ALL}\n")
        super().startTest(test)

    def addSuccess(self, test):
        elapsed = time.time() - self._start_time
        self.stream.write(f"{Fore.GREEN}PASSED{Style.RESET_ALL} in {elapsed:.2f}s\n")
        super().addSuccess(test)

    def addFailure(self, test, err):
        elapsed = time.time() - self._start_time
        self.stream.write(f"{Fore.RED}FAILED{Style.RESET_ALL} in {elapsed:.2f}s\n")
        super().addFailure(test, err)

    def addError(self, test, err):
        elapsed = time.time() - self._start_time
        self.stream.write(f"{Fore.MAGENTA}ERROR{Style.RESET_ALL} in {elapsed:.2f}s\n")
        super().addError(test, err)

class CustomRunner(unittest.TextTestRunner):
    def __init__(self, **kwargs):
        super().__init__(resultclass=CustomTestResult, **kwargs)

class CanBusTestCase(unittest.TestCase):
    @classmethod
    def get_can_silent_ids(cls) -> set[int]:
        return set()

    @classmethod
    def setUpClass(cls):
        if cls.bus is None:
            raise RuntimeError("CanBusTestCase.bus must be set before setUpClass() is called.")
        cls.can_silent_ids = cls.get_can_silent_ids()

    def setUp(self):
        # before each test_
        self.flushCanMessages()

    def tearDown(self):
        # after each test_
        time.sleep(0.1)

    def flushCanMessages(self):
        counter = 0
        while self.bus.recv(timeout=0):
            counter += 1
            pass
        print(f"flushed {counter} can messages")

    def assertNextCanMessageIs(
        self,
        expected_can_id: list[int],
        expected_can_data: bytes | list[int] | None = None,
        timeout: float = 5
    ) -> bool:
        deadline = time.time() + timeout

        expected_data = (
            bytes(expected_can_data)
            if isinstance(expected_can_data, list)
            else expected_can_data
        )

        msg = self.bus.recv(timeout=timeout)

        if msg is None:
            self.fail(f"No CAN message received within {timeout} seconds.")
        
        if msg.arbitration_id not in expected_can_id:
            self.fail(f"Received unexpected CAN ID {hex(msg.arbitration_id)}. Expected one of {list(map(hex, expected_can_id))}.")

        if expected_data is not None and not msg.data.startswith(expected_data):
            self.fail(f"Received CAN ID {hex(msg.arbitration_id)} with data {msg.data.hex()} "
                    f"which does not start with expected data {expected_data.hex()}.")

        return True

    def assertCanMessageReceived(
        self,
        expected_can_id: list[int],
        expected_can_data: bytes | list[int] | None = None,
        timeout: float = 1
    ) -> bool:
        deadline = time.time() + timeout

        expected_data = (
            bytes(expected_can_data)
            if isinstance(expected_can_data, list)
            else expected_can_data
        )

        while time.time() < deadline:
            remaining_time = max(0, deadline - time.time())
            msg = self.bus.recv(timeout=remaining_time)
            
            if msg is None:
                continue

            if msg.arbitration_id in expected_can_id:
                if expected_can_data is None or msg.data.startswith(expected_data):

                    if msg.arbitration_id not in self.can_silent_ids:
                        frame_name = CANIDS.get_name(msg.arbitration_id)
                        print("CAN FRAME > ", frame_name, " : ", msg)
                    
                    return True

        self.fail(f"CAN ID {expected_can_id} was not received within {timeout} seconds with data {expected_can_data}.")

    def assertCanMessageNotReceived(
        self,
        unexpected_can_id: list[int],
        unexpected_can_data: bytes | list[int] | None = None,
        timeout: float = 5
    ) -> bool:
        deadline = time.time() + timeout

        expected_data = (
            bytes(unexpected_can_data)
            if isinstance(unexpected_can_data, list)
            else unexpected_can_data
        )

        while time.time() < deadline:
            remaining_time = max(0, deadline - time.time())
            msg = self.bus.recv(timeout=remaining_time)
            
            if msg is None:
                continue

            if msg.arbitration_id in unexpected_can_id:
                if expected_data is None or msg.data.startswith(expected_data):
                    if msg.arbitration_id not in self.can_silent_ids:
                        frame_name = CANIDS.get_name(msg.arbitration_id)
                        print("CAN FRAME > ", frame_name, " : ", msg)

                    frame_name = CANIDS.get_name(msg.arbitration_id)
                    self.fail(f"Unexpected CAN message received: {frame_name} : {msg}")

        return True