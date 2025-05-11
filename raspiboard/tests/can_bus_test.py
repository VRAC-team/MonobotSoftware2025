import time
import can
import queue

from robot.can_utils import get_can_interface
from robot.can_identifiers import CANIDS
from .simple_test import SimpleTest


class CanBusTest(SimpleTest, can.Listener):
    def setUp(self, debug_print: bool = False, silent_ids: set[int] = set()):
        self.debug_print = debug_print
        self.silent_ids = silent_ids
        self.bus = get_can_interface()
        self.notifier = can.Notifier(self.bus, [self])
        self.message_queue = queue.Queue()

    def tearDown(self):
        self.notifier.stop()
        self.bus.shutdown()

    def setUpTest(self):
        self.flushCanMessages()

    def tearDownTest(self):
        time.sleep(0.5)

    def on_message_received(self, msg: can.Message) -> None:
        if self.debug_print and msg.arbitration_id not in self.silent_ids:
            frame_name = CANIDS.get_name(msg.arbitration_id)
            print("CAN FRAME > ", frame_name, " : ", msg)

        self.message_queue.put(msg)

    def flushCanMessages(self):
        counter = 0
        while not self.message_queue.empty():
            self.message_queue.get_nowait()
            counter += 1
        print(f"flushed {counter} can messages")

    def _wait_for_message(self, timeout):
        try:
            return self.message_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def assertNextCanMessageIs(
        self,
        expected_can_id: list[int],
        expected_can_data: bytes | list[int] | None = None,
        timeout: float = 5,
    ) -> bool:
        expected_data = bytes(expected_can_data) if isinstance(expected_can_data, list) else expected_can_data
        msg = self._wait_for_message(timeout)

        if msg is None:
            raise AssertionError(f"No CAN message received within {timeout} seconds.")

        if msg.arbitration_id not in expected_can_id:
            raise AssertionError(
                f"Received unexpected CAN ID {hex(msg.arbitration_id)}. "
                f"Expected one of {list(map(hex, expected_can_id))}."
            )

        if expected_data is not None and not msg.data.startswith(expected_data):
            raise AssertionError(
                f"Received CAN ID {hex(msg.arbitration_id)} with data {msg.data.hex()} "
                f"which does not start with expected data {expected_data.hex()}."
            )

        return True

    def assertCanMessageReceived(
        self,
        expected_can_id: list[int],
        expected_can_data: bytes | list[int] | None = None,
        timeout: float = 1,
    ) -> bool:
        expected_data = bytes(expected_can_data) if isinstance(expected_can_data, list) else expected_can_data
        deadline = time.time() + timeout

        while time.time() < deadline:
            remaining_time = max(0, deadline - time.time())
            msg = self._wait_for_message(remaining_time)

            if msg is None:
                continue

            if msg.arbitration_id in expected_can_id:
                if expected_data is None or msg.data.startswith(expected_data):
                    return True

        raise AssertionError(
            f"CAN ID {expected_can_id} was not received within {timeout} seconds with data {expected_can_data}."
        )

    def assertCanMessageNotReceived(
        self,
        unexpected_can_id: list[int],
        unexpected_can_data: bytes | list[int] | None = None,
        timeout: float = 5,
    ) -> bool:
        expected_data = bytes(unexpected_can_data) if isinstance(unexpected_can_data, list) else unexpected_can_data
        deadline = time.time() + timeout

        while time.time() < deadline:
            remaining_time = max(0, deadline - time.time())
            msg = self._wait_for_message(remaining_time)

            if msg is None:
                continue

            if msg.arbitration_id in unexpected_can_id:
                if expected_data is None or msg.data.startswith(expected_data):
                    frame_name = CANIDS.get_name(msg.arbitration_id)
                    raise AssertionError(f"Should not have received this CAN message: {frame_name} : {msg}")

        return True
