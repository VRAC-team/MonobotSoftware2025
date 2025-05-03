import can
from canids import CANIDS

class MotorBoard:
    def __init__(self, bus: can.Bus):
        self.bus = bus
    
    def reboot(self):
        msg = can.Message(arbitration_id=CANIDS.CANID_MOTOR_REBOOT, is_extended_id=False)
        self.bus.send(msg)
    
    def reset_error(self):
        msg = can.Message(arbitration_id=CANIDS.CANID_MOTOR_RESET_STATE_ERROR, is_extended_id=False)
        self.bus.send(msg)
    
    def pwm_write(self, right: int, left: int):
        """
        Parameters:
        - right: The PWM value for the right motor. A signed 16-bit integer, ranging from -32768 to 32767.
        The value will be clamped if it falls outside this range.
        - left: The PWM value for the left motor. A signed 16-bit integer, ranging from -32768 to 32767.
        The value will be clamped if it falls outside this range.
        """

        # Clamp the left motor PWM value to the range of signed 16-bit integers (-32768 to 32767)
        right = max(-32768, min(32767, right))
        left = max(-32768, min(32767, left))
    
        data = bytearray(
            right.to_bytes(2, signed=True) +
            left.to_bytes(2, signed=True)
        )
        msg = can.Message(arbitration_id=CANIDS.CANID_MOTOR_PWM_WRITE, data=data, is_extended_id=False)
        self.bus.send(msg)