import time
import can

from .can_bus_test import CanBusTest
from robot.boards.servoboard import ServoBoard, Servo
from robot.can_identifiers import CANIDS


def get_default_servos_config():
    return {i: Servo() for i in range(18)}


def set_can_filter(bus: can.BusABC):
    # filters IDs from 0x100 to 0x1FF
    bus.set_filters([{"can_id": 0x100, "can_mask": 0x700, "extended": False}])


VALID_SERVO_ID = [0, 1, 8, 15]
INVALID_SERVO_ID = [-1, 16, 17, 200]

VALID_SERVO_US = [1500, 1000, 2000]
INVALID_SERVO_US = [-40, 300, 3000]

VALID_LED_ID = [0, 1, 2, 3]
INVALID_LED_ID = [-1, 4, 5, 16]

VALID_LED_PATTERN = list(range(7))


class ServoBoardUnitTests(CanBusTest):
    def setup(self):
        super().setup()
        set_can_filter(self.bus)
        self.servoboard = ServoBoard(self.bus, get_default_servos_config())
        self.notifier.add_listener(self.servoboard)

    def teardown(self):
        self.servoboard.reboot()
        self.notifier.remove_listener(self.servoboard)
        super().teardown()

    def test_01_servo_set_us_invalid_id(self):
        for id in INVALID_SERVO_ID:
            for us in VALID_SERVO_US:
                self.assert_false(self.servoboard.servo_set_us(id, us))

    def test_02_servo_set_us_interp_invalid_id(self):
        for id in INVALID_SERVO_ID:
            for us in VALID_SERVO_US:
                self.assert_false(self.servoboard.servo_set_us_interp(id, us))

    def test_03_servo_set_us_invalid_us(self):
        for id in VALID_SERVO_ID:
            for us in INVALID_SERVO_US:
                self.assert_false(self.servoboard.servo_set_us(id, us))

    def test_04_servo_set_us_interp_invalid_us(self):
        for id in VALID_SERVO_ID:
            for us in INVALID_SERVO_US:
                self.assert_false(self.servoboard.servo_set_us_interp(id, us))

    def test_05_set_led_pattern_invalid_id(self):
        for id in INVALID_LED_ID:
            for pattern in VALID_LED_PATTERN:
                self.assert_false(self.servoboard.set_led_pattern(id, pattern))


class ServoBoardIntegrationTests(CanBusTest):
    def setup(self):
        # {CANIDS.CANID_SERVO_STATUS, CANIDS.CANID_SERVO_ALIVE}
        super().setup()
        set_can_filter(self.bus)
        self.servoboard = ServoBoard(self.bus, get_default_servos_config())
        self.notifier.add_listener(self.servoboard)

    def teardown(self):
        self.servoboard.reboot()
        self.notifier.remove_listener(self.servoboard)
        super().teardown()

    def test_00_reboot(self):
        self.assert_true(self.servoboard.reboot())
        self.assert_can_message_received([CANIDS.CANID_SERVO_ALIVE], [True], timeout=4)
        self.assert_can_message_received([CANIDS.CANID_SERVO_ALIVE], [False], timeout=2)

    def test_01_alive(self):
        for i in range(3):
            self.assert_can_message_received([CANIDS.CANID_SERVO_ALIVE], timeout=2)

    def test_02_status(self):
        for i in range(3):
            self.assert_can_message_received([CANIDS.CANID_SERVO_STATUS], timeout=2)

    def test_03_enable_power_none(self):
        self.assert_true(self.servoboard.enable_power(False, False, False))

    def test_04_enable_power_all(self):
        self.assert_true(self.servoboard.enable_power(True, True, True))

    def test_05_servo_set_us(self):
        for us in VALID_SERVO_US:
            print(f"setting all servos to {us}")
            for id in range(18):
                self.assert_true(self.servoboard.servo_set_us(id, us))
            time.sleep(1)

    def test_06_servo_set_us_interp(self):
        def generator_test_06():
            for us in VALID_SERVO_US:
                for id in range(18):
                    self.servoboard.servo_set_us_interp(id, us, 200)
                yield from self.servoboard.wait_servos()

        self.assert_true(self.servoboard.servo_set_us(8, 1000))
        time.sleep(2)

        g = generator_test_06()

        while True:
            self.servoboard.process(time.monotonic())
            try:
                next(g)
            except StopIteration:
                return

    def test_07_set_led_pattern_valid(self):
        for pattern in range(8):
            print(f"setting all led to pattern to {pattern}")
            for id in range(2):
                self.assert_true(self.servoboard.set_led_pattern(id, pattern))

            time.sleep(3)  # sleep a little bit to admire these cool led patterns :D

    def test_08_servo_ser_us_error_not_enabled(self):
        # disabling power only for servos 0-7
        self.assert_true(self.servoboard.enable_power(False, True, True))
        for id in range(8):
            for us in VALID_SERVO_US:
                self.assert_true(self.servoboard.servo_set_us(id, us))
                self.assert_can_message_received([CANIDS.CANID_SERVO_ERROR_NOT_ENABLED])

        # disabling power only for servos 8-15
        self.assert_true(self.servoboard.enable_power(True, False, True))
        for id in range(8, 16):
            for us in VALID_SERVO_US:
                self.assert_true(self.servoboard.servo_set_us(id, us))
                self.assert_can_message_received([CANIDS.CANID_SERVO_ERROR_NOT_ENABLED])

    def test_09_set_led_pattern_error_not_enabled(self):
        # disabling power only for leds
        self.assert_true(self.servoboard.enable_power(True, True, False))
        for id in range(2):
            for pattern in VALID_LED_PATTERN:
                self.assert_true(self.servoboard.set_led_pattern(id, pattern))
                self.assert_can_message_received([CANIDS.CANID_SERVO_ERROR_NOT_ENABLED])
