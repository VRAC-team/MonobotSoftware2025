import can_utils as can_utils
import can_test_utils as can_test_utils
from canids import CANIDS
import unittest
import colorama
import time
from boards.servoboard import ServoBoard
from colorama import Fore, Style

colorama.init(autoreset=True)
bus = can_utils.get_can_interface()
# filters IDs from 0x100 to 0x1FF
bus.set_filters([{
    "can_id": 0x100,
    "can_mask": 0x700,
    "extended": False
}])
servoboard = ServoBoard(bus)

class ServoBoardTests(can_test_utils.CanBusTestCase):
    silent_can_ids = {
        CANIDS.CANID_SERVO_STATUS,
        CANIDS.CANID_SERVO_ALIVE
    }

    @classmethod
    def setUpClass(cls):
        cls.bus = bus
        super().setUpClass()

    def test_00_reboot(self):
        servoboard.reboot()
        self.assertCanIdReceived([CANIDS.CANID_SERVO_ALIVE], [True], timeout=2)
        self.assertCanIdReceived([CANIDS.CANID_SERVO_ALIVE], [False], timeout=2)

    def test_01_alive(self):
        for i in range(3):
            self.assertCanIdReceived([CANIDS.CANID_SERVO_ALIVE], timeout=2)

    def test_02_status(self):
        for i in range(3):
            self.assertCanIdReceived([CANIDS.CANID_SERVO_STATUS], timeout=2)

    def test_03_enable_power_none(self):
        servoboard.enable_power(False, False, False)

    def test_04_enable_power_all(self):
        servoboard.enable_power(True, True, True)

    def test_05_servo_write_us_valid(self):
        for us in [1500, 2000, 2500]:
            print(f"setting all servos to {us}")
            for id in range(16):
                servoboard.servo_write_us(servo_id=id, servo_us=us)
            time.sleep(3)

    def test_06_servo_write_us_invalid_id(self):
        servoboard.servo_write_us(servo_id=20, servo_us=1500)  # Should silently do nothing

    def test_07_servo_write_us_invalid_pulse(self):
        servoboard.servo_write_us(servo_id=0, servo_us=300)    # Too low
        servoboard.servo_write_us(servo_id=0, servo_us=3000)   # Too high

    def test_08_set_led_pattern_valid(self):
        for pattern in range(8):
            print(f"setting all led to pattern to {pattern}")
            for id in range(4):
                servoboard.set_led_pattern(led_id=id, led_pattern=pattern)
            time.sleep(3)

    def test_09_set_led_pattern_invalid_id(self):
        servoboard.set_led_pattern(led_id=5, led_pattern=2)  # Should silently do nothing

    def test_10_set_led_pattern_invalid_pattern(self):
        servoboard.set_led_pattern(led_id=0, led_pattern=-1)  # Should silently do nothing


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(ServoBoardTests)
    runner = can_test_utils.CustomRunner(verbosity=2)
    runner.run(suite)
    bus.shutdown()