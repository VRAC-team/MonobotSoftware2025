import time
from typing import Generator
import can

from .can_bus_test import CanBusTest
from robot.boards.pumpboard import PumpBoard, PumpValveState
from robot.can_identifiers import CANIDS
from robot.robot_config import RobotConfig


def set_can_filter(bus: can.BusABC):
    # filters IDs from 0x300 to 0x3FF
    bus.set_filters([{"can_id": 0x300, "can_mask": 0x700, "extended": False}])


class PumpBoardIntegrationTests(CanBusTest):
    def __init__(self, config: RobotConfig):
        self.config = config

    def setup(self):
        super().setup()
        set_can_filter(self.bus)
        self.pumpboard = PumpBoard(self.bus, self.config.PUMP_AUTO_CLOSE_VALVE_AFTER)
        self.notifier.add_listener(self.pumpboard)

    def teardown(self):
        self.pumpboard.reboot()
        self.notifier.remove_listener(self.pumpboard)
        super().teardown()

    def test_01_reboot_and_alive(self):
        self.assert_true(self.pumpboard.reboot())
        self.assert_can_message_received([CANIDS.CANID_PUMP_ALIVE], [True], timeout=2)
        self.assert_can_message_received([CANIDS.CANID_PUMP_ALIVE], [False], timeout=2)

    def test_02_status(self):
        for _ in range(5):
            self.assert_can_message_received([CANIDS.CANID_PUMP_STATUS], timeout=2)

    def test_03_test_pumps(self):
        for i in range(6):
            self.pumpboard.set_states({i: (PumpValveState.ENABLE, PumpValveState.NO_CHANGE)})
            time.sleep(1)
            self.pumpboard.set_states({i: (PumpValveState.DISABLE, PumpValveState.NO_CHANGE)})

    def test_04_test_valves(self):
        for i in range(6):
            self.pumpboard.set_states({i: (PumpValveState.NO_CHANGE, PumpValveState.ENABLE)})
            time.sleep(1)
            self.pumpboard.set_states({i: (PumpValveState.NO_CHANGE, PumpValveState.DISABLE)})

    def test_05_enable_disable_pumps_auto_valve(self):
        def generator_test_05() -> Generator[None, None, None]:
            for i in range(6):
                self.pumpboard.enable_pumps((i,))
                time.sleep(1)
                self.pumpboard.disable_pump_with_auto_valve_release((i,))
                yield from self.pumpboard.wait_valve_auto_release()
                time.sleep(1)

        g = generator_test_05()
        while True:
            self.pumpboard.process(time.monotonic())
            try:
                next(g)
            except StopIteration:
                break
