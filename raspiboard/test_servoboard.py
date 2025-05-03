import can_utils as can_utils
import can_test_utils as can_test_utils
from canids import CANIDS
import unittest
import colorama
import time
import can
from boards.servoboard import ServoBoard, ServoConfig
from colorama import Fore, Style

colorama.init(autoreset=True)
bus = can_utils.get_can_interface()
# filters IDs from 0x100 to 0x1FF
bus.set_filters([{
    "can_id": 0x100,
    "can_mask": 0x700,
    "extended": False
}])

servoboard = ServoBoard(bus, { i: ServoConfig() for i in range(16) })

VALID_SERVO_ID = [0, 1, 8, 15]
INVALID_SERVO_ID = [-1, 16, 17, 200]

VALID_SERVO_US = [1500, 1000, 2000]
INVALID_SERVO_US = [-40, 300, 3000]

VALID_SERVO_ANGLE = [0, 90, 180]
INVALID_SERVO_ANGLE = [-1, 360, 720]

VALID_LED_ID = [0, 1, 2, 3]
INVALID_LED_ID = [-1, 4, 5, 16]

VALID_LED_PATTERN = [i for i in range(7)]

class ServoBoardIntegrationTests(can_test_utils.CanBusTestCase):
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
        super().tearDownClass()

    def test_00_reboot(self):
        self.assertTrue(servoboard.reboot())
        time.sleep(4) # servoboard has 3s wait at startup
        self.assertCanMessageReceived([CANIDS.CANID_SERVO_ALIVE], [True], timeout=2)
        self.assertCanMessageReceived([CANIDS.CANID_SERVO_ALIVE], [False], timeout=2)

    def test_01_alive(self):
        for i in range(3):
            self.assertCanMessageReceived([CANIDS.CANID_SERVO_ALIVE], timeout=2)

    def test_02_status(self):
        for i in range(3):
            self.assertCanMessageReceived([CANIDS.CANID_SERVO_STATUS], timeout=2)

    def test_03_enable_power_none(self):
        self.assertTrue(servoboard.enable_power(False, False, False))

    def test_04_enable_power_all(self):
        self.assertTrue(servoboard.enable_power(True, True, True))

    ##### servo_write_us #####
    def test_05_servo_write_us_valid(self):
        for us in VALID_SERVO_US:
            print(f"setting all servos to {us}")
            for id in range(16):
                self.assertTrue(servoboard.servo_write_us(id, us))
            time.sleep(3)

    ##### servo_write_angle #####
    def test_06_servo_write_angle_valid(self):
        for id in VALID_SERVO_ID:
            for angle in VALID_SERVO_ANGLE:
                self.assertTrue(servoboard.servo_write_angle(id, angle))

    ##### set_led_pattern #####
    def test_07_set_led_pattern_valid(self):
        for pattern in range(8):
            print(f"setting all led to pattern to {pattern}")
            for id in range(4):
                self.assertTrue(servoboard.set_led_pattern(id, pattern))
            
            time.sleep(3) # sleep a little bit to admire these cool led patterns :D

    ##### test ERROR_NOT_ENABLED #####
    def test_08_servo_write_us_error_not_enabled(self):
        # disabling power only for servos 0-7
        self.assertTrue(servoboard.enable_power(False, True, True))
        for id in range(8):
            for us in VALID_SERVO_US:
                self.assertTrue(servoboard.servo_write_us(id, us))
                self.assertCanMessageReceived([CANIDS.CANID_SERVO_ERROR_NOT_ENABLED])

        # disabling power only for servos 8-15
        self.assertTrue(servoboard.enable_power(True, False, True))
        for id in range(8, 16):
            for us in VALID_SERVO_US:
                self.assertTrue(servoboard.servo_write_us(id, us))
                self.assertCanMessageReceived([CANIDS.CANID_SERVO_ERROR_NOT_ENABLED])

    def test_09_set_led_pattern_error_not_enabled(self):
        # disabling power only for leds
        self.assertTrue(servoboard.enable_power(True, True, False))
        for id in range(4):
            for pattern in VALID_LED_PATTERN:
                self.assertTrue(servoboard.set_led_pattern(id, pattern))
                self.assertCanMessageReceived([CANIDS.CANID_SERVO_ERROR_NOT_ENABLED])


class ServoBoardUnitTests(unittest.TestCase):
    def test_01_servo_write_us_invalid_id(self):
        for id in INVALID_SERVO_ID:
            for us in VALID_SERVO_US:
                # test ServoBoard class
                self.assertFalse(servoboard.servo_write_us(id, us))

    def test_02_servo_write_us_invalid_us(self):
        for id in VALID_SERVO_ID:
            for us in INVALID_SERVO_US:
                # test ServoBoard class
                self.assertFalse(servoboard.servo_write_us(id, us))

    def test_03_servo_write_angle_invalid_id(self):
        for id in INVALID_SERVO_ID:
            for angle in VALID_SERVO_ANGLE:
                self.assertFalse(servoboard.servo_write_angle(id, angle))

    def test_04_servo_write_angle_invalid_angle(self):
        for id in VALID_SERVO_ID:
            for angle in INVALID_SERVO_ANGLE:
                self.assertFalse(servoboard.servo_write_angle(id, angle))

    def test_05_set_led_pattern_invalid_id(self):
        for id in INVALID_LED_ID:
            for pattern in VALID_LED_PATTERN:
                self.assertFalse(servoboard.set_led_pattern(id, pattern))

if __name__ == '__main__':
    ServoBoardIntegrationTests.bus = bus
    
    runner = can_test_utils.CustomRunner()

    unit_tests = unittest.TestLoader().loadTestsFromTestCase(ServoBoardUnitTests)
    runner.run(unit_tests)

    integration_tests = unittest.TestLoader().loadTestsFromTestCase(ServoBoardIntegrationTests)
    runner.run(integration_tests)

    servoboard.reboot()

    bus.shutdown()