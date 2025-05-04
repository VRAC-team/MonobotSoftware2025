import can
import can_utils
from canids import CANIDS


def map_range(value, from_min, from_max, to_min, to_max):
    if from_max - from_min == 0:
        raise ValueError("Invalid input range")
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min


class ServoConfig:
    def __init__(self, min_us: int = 500, max_us: int = 2500, max_angle: int = 180):
        self.min_us = min_us
        self.max_us = max_us
        self.max_angle = max_angle


def default_servos_configs():
    return {
        8: ServoConfig(),
        9: ServoConfig(),
        10: ServoConfig(),
        11: ServoConfig(),
        15: ServoConfig(max_angle=270),
    }


class ServoBoard:
    def __init__(self, bus: can.Bus, servos_configs: dict = None):
        self.bus = bus
        self.servos_configs = servos_configs or default_servos_configs()

    def reboot(self) -> bool:
        msg = can.Message(
            arbitration_id=CANIDS.CANID_SERVO_REBOOT, is_extended_id=False
        )
        return can_utils.send(self.bus, msg)

    def enable_power(self, power1: bool, power2: bool, power3: bool) -> bool:
        msg = can.Message(
            arbitration_id=CANIDS.CANID_SERVO_ENABLE_POWER,
            data=[power1, power2, power3],
            is_extended_id=False,
        )
        return can_utils.send(self.bus, msg)

    def servo_write_us(self, servo_id: int, servo_us: int) -> bool:
        if servo_id not in self.servos_configs:
            print(
                f"servo_write_us: servo_id:{servo_id} not mapped in self.servos_configs"
            )
            return False

        servo_config = self.servos_configs[servo_id]

        if servo_us < servo_config.min_us or servo_us > servo_config.max_us:
            print(
                f"servo_write_us: invalid servo_us:{servo_us} (servo_config:{servo_config})"
            )
            return False

        data = bytearray(servo_id.to_bytes(1) + servo_us.to_bytes(2))
        msg = can.Message(
            arbitration_id=CANIDS.CANID_SERVO_WRITE_US, data=data, is_extended_id=False
        )
        return can_utils.send(self.bus, msg)

    def servo_write_angle(self, servo_id: int, servo_angle: int) -> bool:
        if servo_id not in self.servos_configs:
            print(
                f"servo_write_angle: servo_id:{servo_id} not mapped in self.servos_configs"
            )
            return False

        servo_config = self.servos_configs[servo_id]

        if servo_angle < 0 or servo_angle > servo_config.max_angle:
            print(
                f"servo_write_angle: invalid angle:{servo_angle} (servo_config:{servo_config})"
            )
            return False

        servo_us = map_range(
            servo_angle,
            0,
            servo_config.max_angle,
            servo_config.min_us,
            servo_config.max_us,
        )
        servo_us = int(servo_us)

        data = bytearray(servo_id.to_bytes(1) + servo_us.to_bytes(2))
        msg = can.Message(
            arbitration_id=CANIDS.CANID_SERVO_WRITE_US, data=data, is_extended_id=False
        )
        return can_utils.send(self.bus, msg)

    def set_led_pattern(self, led_id: int, led_pattern: int) -> bool:
        if led_id < 0 or led_id > 3:
            return False

        if led_pattern < 0:
            return False

        msg = can.Message(
            arbitration_id=CANIDS.CANID_SERVO_SET_LED_PATTERN,
            data=[led_id, led_pattern],
            is_extended_id=False,
        )
        return can_utils.send(self.bus, msg)
