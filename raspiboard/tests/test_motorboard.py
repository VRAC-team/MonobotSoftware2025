import select
import time
import can

from .can_bus_test import CanBusTest
from robot.boards.motorboard import MotorBoard
from robot.can_identifiers import CANIDS
from robot.robot_config import RobotConfig


def set_can_filter(bus: can.BusABC):
    # filters IDs from 0x000 to 0x0FF
    bus.set_filters([{"can_id": 0x000, "can_mask": 0x700, "extended": False}])


class MotorBoardIntegrationTests(CanBusTest):
    def __init__(self, config: RobotConfig):
        self.config = config

    def setup(self):
        # {CANIDS.CANID_MOTOR_STATUS, CANIDS.CANID_MOTOR_ALIVE}
        super().setup()
        set_can_filter(self.bus)
        self.motorboard = MotorBoard(self.bus)
        self.notifier.add_listener(self.motorboard)

    def teardown(self):
        self.motorboard.reboot()
        self.notifier.remove_listener(self.motorboard)
        super().teardown()

    def test_01_reboot_and_alive(self):
        self.assert_true(self.motorboard.reboot())
        self.assert_can_message_received([CANIDS.CANID_MOTOR_ALIVE], [True], timeout=2)
        self.assert_can_message_received([CANIDS.CANID_MOTOR_ALIVE], [False], timeout=2)

    def test_02_reboot_and_error(self):
        self.assert_true(self.motorboard.reboot())
        self.assert_can_message_received([CANIDS.CANID_MOTOR_STATE_ERROR], timeout=0.5)

    def test_03_status_error(self):
        for i in range(200):
            self.assert_can_message_received([CANIDS.CANID_MOTOR_STATUS], [True], timeout=0.1)

    def test_04_pwm_write_valid(self):
        self.assert_can_message_received([CANIDS.CANID_MOTOR_STATUS], [True], timeout=0.5)

        period = self.config.get_controlloop_period()

        self.assert_true(self.motorboard.reset_error())
        for i in range(self.config.CONTROLLOOP_FREQUENCY * 3):
            start_time = time.monotonic()

            self.assert_true(self.motorboard.pwm_write(0, 0))
            self.assert_can_message_received([CANIDS.CANID_MOTOR_STATUS], [False], timeout=0.1)

            elapsed_time = time.monotonic() - start_time
            timeout = max(0, period - elapsed_time)
            select.select([], [], [], timeout)

        time.sleep(0.015)
        self.flush_can_messages()
        # after sleep 15ms we should NOT get state_error=False on the STATUS
        self.assert_can_message_not_received([CANIDS.CANID_MOTOR_STATUS], [False], timeout=0.1)

    def test_05_reset_error_invalid(self):
        self.assert_can_message_received([CANIDS.CANID_MOTOR_STATUS], [True], timeout=0.5)
        self.assert_true(self.motorboard.reset_error())
        # here we do not send the PWM_WRITE, we should not receive MOTOR_STATUS with state_error=False
        self.assert_can_message_not_received([CANIDS.CANID_MOTOR_STATUS], [False], timeout=1)
