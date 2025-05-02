import can_utils as can_utils
import can_test_utils as can_test_utils
from canids import CANIDS
import unittest
import colorama
import time
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

class MotorBoardTests(can_test_utils.CanBusTestCase):
    silent_can_ids = {
        CANIDS.CANID_MOTOR_STATUS,
        CANIDS.CANID_MOTOR_ALIVE
    }

    @classmethod
    def setUpClass(cls):
        cls.bus = bus
        super().setUpClass()

    def test_00_reboot(self):
        motorboard.reboot()
        self.assertCanIdReceived([CANIDS.CANID_MOTOR_ALIVE], [True, True], timeout=2)
        self.assertCanIdReceived([CANIDS.CANID_MOTOR_ALIVE], [False, True], timeout=2)

    def test_01_alive(self):
        for i in range(3):
            self.assertCanIdReceived([CANIDS.CANID_MOTOR_ALIVE], timeout=2)

    def test_02_status(self):
        for i in range(5):
            self.assertCanIdReceived([CANIDS.CANID_MOTOR_STATUS], timeout=1)



if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(MotorBoardTests)
    runner = can_test_utils.CustomRunner()
    runner.run(suite)
    bus.shutdown()