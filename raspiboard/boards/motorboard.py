import can
from canids import CANIDS

class MotorBoard:
    def __init__(self, bus: can.Bus):
        self.bus = bus
    
    def reboot(self):
        msg = can.Message(arbitration_id=CANIDS.CANID_MOTOR_REBOOT, is_extended_id=False)
        self.bus.send(msg)