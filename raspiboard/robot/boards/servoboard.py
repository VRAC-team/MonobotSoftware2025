import can
import struct
import logging

import robot.can_utils as can_utils
from robot.can_identifiers import CANIDS


def map_range(value, from_min, from_max, to_min, to_max):
    if from_max - from_min == 0:
        raise ValueError("Invalid input range")
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min


class Servo:
    def __init__(
        self,
        min_us: int = 500,
        max_us: int = 2500,
        max_angle: int = 180,
        seconds_per_60deg=0.17,
    ):
        self.min_us = min_us
        self.max_us = max_us
        self.max_angle = max_angle
        self.seconds_per_60deg = seconds_per_60deg
        self.position = 0


class ServoBoard(can.Listener):
    def __init__(self, bus: can.Bus, servos: dict[int, Servo]):
        self.bus = bus
        self.servos = servos
        self.logger = logging.getLogger(self.__class__.__name__)

        for id in self.servos.keys():
            if not (0 <= id <= 15):
                raise ValueError(f"Servo ID:{id} is out of range. Valid IDs are between 0 and 15.")

    def on_message_received(self, msg):
        if not (0x100 <= msg.arbitration_id <= 0x1FF):
            return

        match msg.arbitration_id:
            case CANIDS.CANID_SERVO_ERROR_INVALID_PARAMS:
                self.logger.error("on_message_received: ERROR_INVALID_PARAMS")

            case CANIDS.CANID_SERVO_ERROR_NOT_ENABLED:
                self.logger.error("on_message_received: ERROR_NOT_ENABLED")

            case CANIDS.CANID_SERVO_STATUS:
                power1_adc, power2_adc, power3_adc, powers_en = struct.unpack(">HHHB", msg.data)
                # power1_en = powers_en & 1
                # power2_en = powers_en >> 1 & 1
                # power3_en = powers_en >> 2 & 1
                self.logger.log(
                    -10,
                    "STATUS (power1_adc:%d power1_adc:%d power1_adc:%d powers_en:%s)",
                    power1_adc,
                    power2_adc,
                    power3_adc,
                    format(powers_en, "03b"),
                )

            case CANIDS.CANID_SERVO_ALIVE:
                (first_alive_since_reboot,) = struct.unpack(">?", msg.data)
                self.logger.log(-10, "ALIVE (first_alive_since_reboot:%s)", first_alive_since_reboot)

    def reboot(self) -> bool:
        self.logger.debug("reboot")
        msg = can.Message(arbitration_id=CANIDS.CANID_SERVO_REBOOT, is_extended_id=False)
        return can_utils.send(self.bus, msg)

    def enable_power(self, power1: bool, power2: bool, power3: bool) -> bool:
        self.logger.debug("enable power %s %s %s", power1, power2, power3)
        msg = can.Message(
            arbitration_id=CANIDS.CANID_SERVO_ENABLE_POWER,
            data=[power1, power2, power3],
            is_extended_id=False,
        )
        return can_utils.send(self.bus, msg)

    def servo_write_us(self, id: int, us: int) -> bool:
        if id not in self.servos:
            self.logger.error("servo_write_us(): id:%d not mapped in self.servos", id)
            return False

        servo = self.servos[id]

        if us < servo.min_us or us > servo.max_us:
            self.logger.error("servo_write_us(): invalid us (id:%d us:%d)", id, us)
            return False

        data = bytearray(id.to_bytes(1) + us.to_bytes(2))
        msg = can.Message(arbitration_id=CANIDS.CANID_SERVO_WRITE_US, data=data, is_extended_id=False)
        return can_utils.send(self.bus, msg)

    def servo_write_angle(self, id: int, angle: int) -> bool:
        if id not in self.servos:
            self.logger.error("servo_write_angle(): id:%d not mapped in self.servos", id)
            return False

        servo = self.servos[id]

        if angle < 0 or angle > servo.max_angle:
            self.logger.error("servo_write_angle(): invalid angle (id:%d angle:%d)", id, angle)
            return False

        servo_us = map_range(
            angle,
            0,
            servo.max_angle,
            servo.min_us,
            servo.max_us,
        )
        servo_us = int(servo_us)

        # TODO add wait_for_motion_finished()
        # movement_time = (abs(servo.position - angle) / 60.0) * servo.seconds_per_60deg
        # asyncio.sleep(movement_time)
        # print(f"OK servo:{id} finished moving from:{servo.position} to:{angle}")
        servo.position = angle

        data = bytearray(id.to_bytes(1) + servo_us.to_bytes(2))
        msg = can.Message(arbitration_id=CANIDS.CANID_SERVO_WRITE_US, data=data, is_extended_id=False)
        return can_utils.send(self.bus, msg)

    def set_led_pattern(self, id: int, pattern: int) -> bool:
        if id < 0 or id > 3:
            self.logger.error("set_led_pattern(): invalid led id:%d", id)
            return False

        if pattern < 0:
            return False

        msg = can.Message(
            arbitration_id=CANIDS.CANID_SERVO_SET_LED_PATTERN,
            data=[id, pattern],
            is_extended_id=False,
        )
        return can_utils.send(self.bus, msg)
