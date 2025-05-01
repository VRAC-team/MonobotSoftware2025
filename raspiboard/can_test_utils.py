import unittest
import time
import queue
import threading
from canids import CANIDS
from colorama import Fore, Style
import can
import can_utils

class VerboseTestResult(unittest.TextTestResult):
    def startTest(self, test):
        self._start_time = time.time()
        test_name = self.getDescription(test)
        print(f"\n{Fore.CYAN}Starting {test_name}:")

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

class CanBusTestCase(unittest.TestCase):
    silent_can_ids: set[int] = set()

    @classmethod
    def setUpClass(cls):
        # maybe enforce here a verification that cls.bus is set ?
        cls.can_queue = queue.Queue()
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
                for msg in cls.bus:
                    if msg.is_error_frame:
                        print("CAN ERROR >", msg)
                    elif msg.arbitration_id not in cls.silent_can_ids:
                        frame_name = CANIDS.get_name(msg.arbitration_id)
                        print("CAN FRAME > ", frame_name, " : ", msg)
                    else:
                        # CAN FRAME silenced
                        pass

                    cls.can_queue.put(msg)
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
            if not self.can_queue.empty():
                msg = self.can_queue.get(timeout=timeout)

                if msg.arbitration_id in expected_can_id:
                    if expected_can_data is None:
                        return True

                    if expected_can_data is not None:
                        data = bytes(expected_can_data) if isinstance(expected_can_data, list) else expected_can_data
                        if msg.data == data:
                            return True
        
        self.fail(f"CAN ID {expected_can_id} was not received within {timeout} seconds.")

    def setUp(self):
        # flush previous can messages
        while not self.can_queue.empty():
            try:
                self.can_queue.get_nowait()
            except queue.Empty:
                break

    def tearDown(self):
        time.sleep(0.2)