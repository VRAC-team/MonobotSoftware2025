import can
import struct
import logging
from collections.abc import Callable

import robot.can_utils as can_utils
from robot.can_identifiers import CANIDS


class MotorBoard(can.Listener):
    def __init__(self, bus: can.Bus):
        self.bus = bus
        self.state_error = True
        self.status_callback = None
        self.logger = logging.getLogger(self.__class__.__name__)

    def set_status_callback(self, callback: Callable[[bool, int, int], None]):
        self.status_callback = callback

    def on_message_received(self, msg):
        if not (msg.arbitration_id <= 0x0FF):
            return

        match msg.arbitration_id:
            case CANIDS.CANID_MOTOR_STATUS:
                state_error, enc_left, enc_right = struct.unpack(">?HH", msg.data)

                if self.state_error and not state_error:
                    self.logger.debug("state error has been reset")
                    self.state_error = False

                if self.status_callback is not None:
                    self.status_callback(state_error, enc_left, enc_right)

            case CANIDS.CANID_MOTOR_STATE_ERROR:
                self.state_error = True
                self.logger.debug("STATE_ERROR")

            case CANIDS.CANID_MOTOR_ALIVE:
                (first_alive_since_reboot,) = struct.unpack(">?", msg.data)
                self.logger.log(-10, "ALIVE (first_alive_since_reboot:%s)", first_alive_since_reboot)

    def reboot(self) -> bool:
        self.state_error = True
        msg = can.Message(arbitration_id=CANIDS.CANID_MOTOR_REBOOT, is_extended_id=False)
        return can_utils.send(self.bus, msg)

    def reset_error(self) -> bool:
        self.state_error = False
        msg = can.Message(arbitration_id=CANIDS.CANID_MOTOR_RESET_STATE_ERROR, is_extended_id=False)
        return can_utils.send(self.bus, msg)

    def pwm_write(self, left: int, right: int) -> bool:
        """
        Parameters:
        - left: The PWM value for the left motor. A signed 16-bit integer, ranging from -32768 to 32767.
        The value will be clamped if it falls outside this range.
        - right: The PWM value for the right motor. A signed 16-bit integer, ranging from -32768 to 32767.
        The value will be clamped if it falls outside this range.
        """

        if self.state_error:
            return False

        # Clamp the left motor PWM value to the range of signed 16-bit integers (-32768 to 32767)
        left = max(-32768, min(32767, left))
        right = max(-32768, min(32767, right))

        data = bytearray(left.to_bytes(2, signed=True) + right.to_bytes(2, signed=True))
        msg = can.Message(arbitration_id=CANIDS.CANID_MOTOR_PWM_WRITE, data=data, is_extended_id=False)
        return can_utils.send(self.bus, msg)
