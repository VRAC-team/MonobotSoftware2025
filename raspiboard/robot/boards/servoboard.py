import struct
import logging

from robot.can_utils import can_send
from robot.can_identifiers import CANIDS

import can


class Servo:
    def __init__(self, min_us: int = 500, max_us: int = 2500, us_per_sec: float = 2000.0):
        self.min_us = min_us
        self.max_us = max_us
        self.us_per_sec = us_per_sec

        # set fictive default position at 1500 us (middle of the range)
        self.target_us = 1500
        self.last_us = 1500


class ServoBoard(can.Listener):
    def __init__(self, bus: can.BusABC, servos: dict[int, Servo], process_loop_period: float = 0.03):
        self.bus = bus
        self.servos = servos
        self.servos_in_motion: dict[int, Servo] = {}
        self.process_loop_period = process_loop_period
        self.last_process_t = 0.0
        self.logger = logging.getLogger(self.__class__.__name__)

        for id in self.servos.keys():
            if not (0 <= id <= 17):
                raise ValueError(f"Servo ID:{id} is out of range. Valid IDs are between 0 and 17.")

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
        return can_send(self.bus, msg)

    def enable_power(self, power1: bool, power2: bool, power3: bool) -> bool:
        self.logger.debug("enable power %s %s %s", power1, power2, power3)
        msg = can.Message(
            arbitration_id=CANIDS.CANID_SERVO_ENABLE_POWER,
            data=[power1, power2, power3],
            is_extended_id=False,
        )
        return can_send(self.bus, msg)

    def servo_set_us(self, id: int, us: int) -> bool:
        """
        set instantaneously the servo position
        """
        if id not in self.servos:
            self.logger.error("servo_set_us(): id:%d not mapped in self.servos", id)
            return False

        servo = self.servos[id]

        if us < servo.min_us or us > servo.max_us:
            self.logger.error("servo_set_us(): invalid us (id:%d us:%d)", id, us)
            return False

        servo.target_us = us
        servo.last_us = us

        data = bytearray(id.to_bytes(1) + us.to_bytes(2))
        msg = can.Message(arbitration_id=CANIDS.CANID_SERVO_WRITE_US, data=data, is_extended_id=False)
        return can_send(self.bus, msg)

    def servo_set_us_interp(self, id: int, us: int, us_per_sec: float = 2000.0) -> bool:
        """
        set servo position with a given speed, process() must be called periodically on the main loop
        """
        if id not in self.servos:
            self.logger.error("servo_set_us_interp(): id:%d not mapped in self.servos", id)
            return False

        servo = self.servos[id]

        if us < servo.min_us or us > servo.max_us:
            self.logger.error("servo_set_us_interp(): invalid us (id:%d us:%d)", id, us)
            return False

        if servo.target_us != us:
            self.servos_in_motion[id] = servo
        servo.target_us = us
        servo.us_per_sec = us_per_sec

        return True

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
        return can_send(self.bus, msg)

    def process(self, t: float):
        if not self.servos_in_motion:
            return

        if t >= self.last_process_t + self.process_loop_period:
            for id in list(self.servos_in_motion.keys()):
                servo = self.servos_in_motion[id]
                us = servo.last_us
                if servo.target_us > servo.last_us:
                    us += int(servo.us_per_sec * self.process_loop_period)
                    if us >= servo.target_us:
                        us = servo.target_us
                        self.servos_in_motion.pop(id)
                elif servo.target_us < servo.last_us:
                    us -= int(servo.us_per_sec * self.process_loop_period)
                    if us <= servo.target_us:
                        us = servo.target_us
                        self.servos_in_motion.pop(id)
                else:
                    self.servos_in_motion.pop(id)
                    continue

                data = bytearray(id.to_bytes(1) + us.to_bytes(2))
                msg = can.Message(arbitration_id=CANIDS.CANID_SERVO_WRITE_US, data=data, is_extended_id=False)
                can_send(self.bus, msg)

                servo.last_us = us

            self.last_process_t = t

    def wait_servos(self):
        while self.servos_in_motion:
            yield
