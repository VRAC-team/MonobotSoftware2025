import time
import can

import tests.can_bus_test as can_bus_test
from robot.boards.servoboard import ServoBoard, Servo
from robot.can_identifiers import CANIDS
from robot.utils import setup_logging


def get_default_servos_config():
    return {i: Servo() for i in range(16)}


def set_can_filter(bus: can.BusABC):
    # filters IDs from 0x100 to 0x1FF
    bus.set_filters([{"can_id": 0x100, "can_mask": 0x700, "extended": False}])


VALID_SERVO_ID = [0, 1, 8, 15]
INVALID_SERVO_ID = [-1, 16, 17, 200]

VALID_SERVO_US = [1500, 1000, 2000]
INVALID_SERVO_US = [-40, 300, 3000]

VALID_SERVO_ANGLE = [0, 90, 180]
INVALID_SERVO_ANGLE = [-1, 360, 720]

VALID_LED_ID = [0, 1, 2, 3]
INVALID_LED_ID = [-1, 4, 5, 16]

VALID_LED_PATTERN = [i for i in range(7)]


class ServoBoardUnitTests(can_bus_test.CanBusTest):
    def setUp(self):
        super().setUp()
        set_can_filter(self.bus)
        self.servoboard = ServoBoard(self.bus, get_default_servos_config())
        self.notifier.add_listener(self.servoboard)

    def tearDown(self):
        self.servoboard.reboot()
        self.notifier.remove_listener(self.servoboard)
        super().tearDown()

    def test_01_servo_write_us_invalid_id(self):
        for id in INVALID_SERVO_ID:
            for us in VALID_SERVO_US:
                self.assertFalse(self.servoboard.servo_write_us(id, us))

    def test_02_servo_write_us_invalid_us(self):
        for id in VALID_SERVO_ID:
            for us in INVALID_SERVO_US:
                self.assertFalse(self.servoboard.servo_write_us(id, us))

    def test_03_servo_write_angle_invalid_id(self):
        for id in INVALID_SERVO_ID:
            for angle in VALID_SERVO_ANGLE:
                self.assertFalse(self.servoboard.servo_write_angle(id, angle))

    def test_04_servo_write_angle_invalid_angle(self):
        for id in VALID_SERVO_ID:
            for angle in INVALID_SERVO_ANGLE:
                self.assertFalse(self.servoboard.servo_write_angle(id, angle))

    def test_05_set_led_pattern_invalid_id(self):
        for id in INVALID_LED_ID:
            for pattern in VALID_LED_PATTERN:
                self.assertFalse(self.servoboard.set_led_pattern(id, pattern))


class ServoBoardIntegrationTests(can_bus_test.CanBusTest):
    def setUp(self):
        # {CANIDS.CANID_SERVO_STATUS, CANIDS.CANID_SERVO_ALIVE}
        super().setUp()
        set_can_filter(self.bus)
        self.servoboard = ServoBoard(self.bus, get_default_servos_config())
        self.notifier.add_listener(self.servoboard)

    def tearDown(self):
        self.servoboard.reboot()
        self.notifier.remove_listener(self.servoboard)
        super().tearDown()

    def test_00_reboot(self):
        self.assertTrue(self.servoboard.reboot())
        self.assertCanMessageReceived([CANIDS.CANID_SERVO_ALIVE], [True], timeout=4)
        self.assertCanMessageReceived([CANIDS.CANID_SERVO_ALIVE], [False], timeout=2)

    def test_01_alive(self):
        for i in range(3):
            self.assertCanMessageReceived([CANIDS.CANID_SERVO_ALIVE], timeout=2)

    def test_02_status(self):
        for i in range(3):
            self.assertCanMessageReceived([CANIDS.CANID_SERVO_STATUS], timeout=2)

    def test_03_enable_power_none(self):
        self.assertTrue(self.servoboard.enable_power(False, False, False))

    def test_04_enable_power_all(self):
        self.assertTrue(self.servoboard.enable_power(True, True, True))

    def test_05_servo_write_us_valid(self):
        for us in VALID_SERVO_US:
            print(f"setting all servos to {us}")
            for id in range(16):
                self.assertTrue(self.servoboard.servo_write_us(id, us))
            time.sleep(1)

    def test_06_servo_write_angle_valid(self):
        for id in VALID_SERVO_ID:
            for angle in VALID_SERVO_ANGLE:
                self.assertTrue(self.servoboard.servo_write_angle(id, angle))

    def test_07_set_led_pattern_valid(self):
        for pattern in range(8):
            print(f"setting all led to pattern to {pattern}")
            for id in range(4):
                self.assertTrue(self.servoboard.set_led_pattern(id, pattern))

            time.sleep(3)  # sleep a little bit to admire these cool led patterns :D

    def test_08_servo_write_us_error_not_enabled(self):
        # disabling power only for servos 0-7
        self.assertTrue(self.servoboard.enable_power(False, True, True))
        for id in range(8):
            for us in VALID_SERVO_US:
                self.assertTrue(self.servoboard.servo_write_us(id, us))
                self.assertCanMessageReceived([CANIDS.CANID_SERVO_ERROR_NOT_ENABLED])

        # disabling power only for servos 8-15
        self.assertTrue(self.servoboard.enable_power(True, False, True))
        for id in range(8, 16):
            for us in VALID_SERVO_US:
                self.assertTrue(self.servoboard.servo_write_us(id, us))
                self.assertCanMessageReceived([CANIDS.CANID_SERVO_ERROR_NOT_ENABLED])

    def test_09_set_led_pattern_error_not_enabled(self):
        # disabling power only for leds
        self.assertTrue(self.servoboard.enable_power(True, True, False))
        for id in range(4):
            for pattern in VALID_LED_PATTERN:
                self.assertTrue(self.servoboard.set_led_pattern(id, pattern))
                self.assertCanMessageReceived([CANIDS.CANID_SERVO_ERROR_NOT_ENABLED])


if __name__ == "__main__":
    setup_logging()

    unit_tests = ServoBoardUnitTests()
    unit_tests.run()

    integration_tests = ServoBoardIntegrationTests()
    integration_tests.run()
