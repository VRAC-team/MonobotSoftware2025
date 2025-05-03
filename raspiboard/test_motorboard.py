import can_utils as can_utils
import can_test_utils as can_test_utils
from canids import CANIDS
import unittest
import colorama
import time
import select
from boards.motorboard import MotorBoard
from colorama import Fore, Style

colorama.init(autoreset=True)
bus = can_utils.get_can_interface()
# filters IDs from 0x000 to 0x0FF
bus.set_filters([{
    "can_id": 0x000,
    "can_mask": 0x700,
    "extended": False
}])
motorboard = MotorBoard(bus)

CONTROL_LOOP_PERIOD = 1/200

class MotorBoardIntegrationTests(can_test_utils.CanBusTestCase):
    @classmethod
    def get_can_silent_ids(cls):
        return {
            CANIDS.CANID_MOTOR_STATUS,
            CANIDS.CANID_MOTOR_ALIVE
        }

    @classmethod
    def setUpClass(cls):
        super().setUpClass()

    def test_01_reboot_and_alive(self):
        self.assertTrue(motorboard.reboot())
        self.assertCanMessageReceived([CANIDS.CANID_MOTOR_ALIVE], [True], timeout=2)
        self.assertCanMessageReceived([CANIDS.CANID_MOTOR_ALIVE], [False], timeout=2)
        
    def test_02_reboot_and_error(self):
        self.assertTrue(motorboard.reboot())
        self.assertCanMessageReceived([CANIDS.CANID_MOTOR_STATE_ERROR], timeout=0.5)

    def test_03_status_error(self):
        for i in range(200):
            self.assertCanMessageReceived([CANIDS.CANID_MOTOR_STATUS], [True], timeout=0.1)

    def test_04_pwm_write_valid(self):
        self.assertCanMessageReceived([CANIDS.CANID_MOTOR_STATUS], [True], timeout=0.5)

        self.assertTrue(motorboard.reset_error())
        for i in range(200):
            start_time = time.monotonic()
            
            self.assertTrue(motorboard.pwm_write(0, 0))
            self.assertCanMessageReceived([CANIDS.CANID_MOTOR_STATUS], [False], timeout=0.5)

            elapsed_time = time.monotonic() - start_time
            timeout = max(0, CONTROL_LOOP_PERIOD - elapsed_time)
            select.select([], [], [], timeout)
        
        time.sleep(0.015)
        self.flushCanMessages()
        # after sleep 15ms we should not receive status with state_error=False
        self.assertCanMessageNotReceived([CANIDS.CANID_MOTOR_STATUS], [False], timeout=2)


    def test_05_reset_error_invalid(self):
        self.assertCanMessageReceived([CANIDS.CANID_MOTOR_STATUS], [True], timeout=0.5)
        self.assertTrue(motorboard.reset_error())
        # here we do not send the PWM_WRITE, we should not receive MOTOR_STATUS with state_error=False
        self.assertCanMessageNotReceived([CANIDS.CANID_MOTOR_STATUS], [False], timeout=1)


if __name__ == '__main__':
    MotorBoardIntegrationTests.bus = bus

    runner = can_test_utils.CustomRunner()

    integration_tests = unittest.TestLoader().loadTestsFromTestCase(MotorBoardIntegrationTests)
    runner.run(integration_tests)

    motorboard.reboot()

    bus.shutdown()