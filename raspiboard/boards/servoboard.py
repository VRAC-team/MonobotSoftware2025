import can
from canids import CANIDS

class ServoBoard:
    def __init__(self, bus: can.Bus):
        self.bus = bus
    
    def reboot(self):
        msg = can.Message(arbitration_id=CANIDS.CANID_SERVO_REBOOT, is_extended_id=False)
        self.bus.send(msg)

    def enable_power(self, power1: bool, power2: bool, power3: bool):
        msg = can.Message(arbitration_id=CANIDS.CANID_SERVO_ENABLE_POWER, data=[power1, power2, power3], is_extended_id=False)
        self.bus.send(msg)

    def servo_write_us(self, servo_id: int, servo_us: int):
        if servo_id >= 16:
            print(f"servo_write_us: invalid servo_id! {servo_id}")
            return

        if servo_us < 500 or servo_us > 2500:
            print(f"servo_write_us: invalid servo_us! {servo_us}")
            return
        
        data = bytearray(
            servo_id.to_bytes(1) +
            servo_us.to_bytes(2)
        )
        msg = can.Message(arbitration_id=CANIDS.CANID_SERVO_WRITE_US, data=data, is_extended_id=False)
        self.bus.send(msg)
    
    def set_led_pattern(self, led_id: int, led_pattern: int):
        if led_id >= 4:
            return
        
        if led_pattern < 0:
            return
        
        msg = can.Message(arbitration_id=CANIDS.CANID_SERVO_SET_LED_PATTERN, data=[led_id, led_pattern], is_extended_id=False)
        self.bus.send(msg)