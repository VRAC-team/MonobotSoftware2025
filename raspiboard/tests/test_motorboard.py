import select
import time
import can

import tests.can_bus_test as can_bus_test
from robot.boards.motorboard import MotorBoard
from robot.can_identifiers import CANIDS
from robot.utils import setup_logging
from robot.parameters import RobotParameters


def set_can_filter(bus: can.BusABC):
    # filters IDs from 0x000 to 0x0FF
    bus.set_filters([{"can_id": 0x000, "can_mask": 0x700, "extended": False}])


params = RobotParameters()


class MotorBoardIntegrationTests(can_bus_test.CanBusTest):
    def setUp(self):
        # {CANIDS.CANID_MOTOR_STATUS, CANIDS.CANID_MOTOR_ALIVE}
        super().setUp()
        set_can_filter(self.bus)
        self.motorboard = MotorBoard(self.bus)
        self.notifier.add_listener(self.motorboard)

    def tearDown(self):
        self.motorboard.reboot()
        self.notifier.remove_listener(self.motorboard)
        super().tearDown()

    def test_01_reboot_and_alive(self):
        self.assertTrue(self.motorboard.reboot())
        self.assertCanMessageReceived([CANIDS.CANID_MOTOR_ALIVE], [True], timeout=2)
        self.assertCanMessageReceived([CANIDS.CANID_MOTOR_ALIVE], [False], timeout=2)

    def test_02_reboot_and_error(self):
        self.assertTrue(self.motorboard.reboot())
        self.assertCanMessageReceived([CANIDS.CANID_MOTOR_STATE_ERROR], timeout=0.5)

    def test_03_status_error(self):
        for i in range(200):
            self.assertCanMessageReceived([CANIDS.CANID_MOTOR_STATUS], [True], timeout=0.1)

    def test_04_pwm_write_valid(self):
        self.assertCanMessageReceived([CANIDS.CANID_MOTOR_STATUS], [True], timeout=0.5)

        self.assertTrue(self.motorboard.reset_error())
        for i in range(params.CONTROLLOOP_FREQ_HZ * 3):
            start_time = time.monotonic()

            self.assertTrue(self.motorboard.pwm_write(0, 0))
            self.assertCanMessageReceived([CANIDS.CANID_MOTOR_STATUS], [False], timeout=0.1)

            elapsed_time = time.monotonic() - start_time
            timeout = max(0, params.CONTROLLOOP_PERIOD_S - elapsed_time)
            select.select([], [], [], timeout)

        time.sleep(0.015)
        self.flushCanMessages()
        # after sleep 15ms we should NOT get state_error=False on the STATUS
        self.assertCanMessageNotReceived([CANIDS.CANID_MOTOR_STATUS], [False], timeout=0.1)

    def test_05_reset_error_invalid(self):
        self.assertCanMessageReceived([CANIDS.CANID_MOTOR_STATUS], [True], timeout=0.5)
        self.assertTrue(self.motorboard.reset_error())
        # here we do not send the PWM_WRITE, we should not receive MOTOR_STATUS with state_error=False
        self.assertCanMessageNotReceived([CANIDS.CANID_MOTOR_STATUS], [False], timeout=1)


if __name__ == "__main__":
    setup_logging()

    integration_tests = MotorBoardIntegrationTests()
    integration_tests.run()
