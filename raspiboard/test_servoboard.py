import can_utils as can_utils
import can_test_utils as can_test_utils
from canids import CANIDS
import unittest
import colorama
import time
import can
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
    @classmethod
    def get_can_silent_ids(cls):
        return {
            CANIDS.CANID_SERVO_STATUS,
            CANIDS.CANID_SERVO_ALIVE
        }

    @classmethod
    def setUpClass(cls):
        super().setUpClass()

    @classmethod
    def tearDownClass(cls):
        # disable power after the tests
        servoboard.enable_power(False, False, False)
        super().tearDownClass()

    def test_00_reboot(self):
        servoboard.reboot()
        time.sleep(4) # servoboard has 3s wait at startup
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
        servo_us = 2000
        for servo_id in [16, 200]:
            data = bytearray(
                servo_id.to_bytes(1) +
                servo_us.to_bytes(2)
            )
            msg = can.Message(arbitration_id=CANIDS.CANID_SERVO_WRITE_US, data=data, is_extended_id=False)
            self.bus.send(msg)
            self.assertCanIdReceived([CANIDS.CANID_SERVO_ERROR_INVALID_PARAMS], timeout=1)

    def test_07_servo_write_us_invalid_pulse(self):
        servo_id = 0
        for servo_us in [300, 3000]:
            data = bytearray(
                servo_id.to_bytes(1) +
                servo_us.to_bytes(2)
            )
            msg = can.Message(arbitration_id=CANIDS.CANID_SERVO_WRITE_US, data=data, is_extended_id=False)
            self.bus.send(msg)
            self.assertCanIdReceived([CANIDS.CANID_SERVO_ERROR_INVALID_PARAMS], timeout=1)

    def test_08_set_led_pattern_valid(self):
        for pattern in range(8):
            print(f"setting all led to pattern to {pattern}")
            for id in range(4):
                servoboard.set_led_pattern(led_id=id, led_pattern=pattern)
            time.sleep(3)

    def test_09_set_led_pattern_invalid_id(self):
        led_pattern = 1
        for led_id in [4, 5, 6, 10, 100]:
            data = bytearray(
                led_id.to_bytes(1) +
                led_pattern.to_bytes(1)
            )
            msg = can.Message(arbitration_id=CANIDS.CANID_SERVO_SET_LED_PATTERN, data=data, is_extended_id=False)
            self.bus.send(msg)
            self.assertCanIdReceived([CANIDS.CANID_SERVO_ERROR_INVALID_PARAMS], timeout=1)

    def test_10_set_led_pattern_invalid_pattern(self):
        servoboard.set_led_pattern(led_id=0, led_pattern=200)  # Should silently do nothing

    def test_11_servo_write_us_error_not_enabled(self):
        # disabling power for servos 0-7
        servoboard.enable_power(False, True, True)
        for servo_id in range(8):
            servoboard.servo_write_us(servo_id=servo_id, servo_us=2000)
            self.assertCanIdReceived([CANIDS.CANID_SERVO_ERROR_NOT_ENABLED], timeout=1)

        # disabling power for servos 8-15
        servoboard.enable_power(True, False, True)
        for servo_id in range(8, 16):
            servoboard.servo_write_us(servo_id=servo_id, servo_us=2000)
            self.assertCanIdReceived([CANIDS.CANID_SERVO_ERROR_NOT_ENABLED], timeout=1)

    def test_12_set_led_pattern_error_not_enabled(self):
        # disabling power for leds
        servoboard.enable_power(True, True, False)
        for led_id in range(4):
            servoboard.set_led_pattern(led_id=led_id, led_pattern=2)
            self.assertCanIdReceived([CANIDS.CANID_SERVO_ERROR_NOT_ENABLED], timeout=1)

if __name__ == '__main__':
    ServoBoardTests.bus = bus
    suite = unittest.TestLoader().loadTestsFromTestCase(ServoBoardTests)
    runner = can_test_utils.CustomRunner()
    runner.run(suite)
    bus.shutdown()