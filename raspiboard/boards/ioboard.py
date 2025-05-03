import can
from canids import CANIDS
import can_utils

class IOBoard:
    def __init__(self, bus: can.Bus):
        self.bus = bus
    
    def reboot(self) -> bool:
        msg = can.Message(arbitration_id=CANIDS.CANID_IO_REBOOT, is_extended_id=False)
        return can_utils.send(self.bus, msg)

    def enable(self, state: bool) -> bool:
        msg = can.Message(arbitration_id=CANIDS.CANID_IO_STEPPER_ENABLE, data=[state], is_extended_id=False)
        return can_utils.send(self.bus, msg)

    def home(self, stepper_id: int, max_relative_steps_before_error: int, tor_id: int, tor_state_to_end_homing: bool) -> bool:
        if stepper_id < 0 or stepper_id > 4:
            return False

        data = bytearray(
            stepper_id.to_bytes(1) +
            max_relative_steps_before_error.to_bytes(2, signed=True) +
            tor_id.to_bytes(1) +
            tor_state_to_end_homing.to_bytes(1)
        )
        msg = can.Message(arbitration_id=CANIDS.CANID_IO_STEPPER_HOME, data=data, is_extended_id=False)
        return can_utils.send(self.bus, msg)

    def goto_abs(self, stepper_id: int, absolute_steps: int, acceleleration: int, max_velocity: int) -> bool:
        if stepper_id < 0 or stepper_id > 4:
            return False

        data = bytearray(
            stepper_id.to_bytes(1) +
            absolute_steps.to_bytes(2, signed=True) +
            acceleleration.to_bytes(3) +
            max_velocity.to_bytes(2)
        )
        msg = can.Message(arbitration_id=CANIDS.CANID_IO_STEPPER_GOTO, data=data, is_extended_id=False)
        return can_utils.send(self.bus, msg)