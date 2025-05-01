import can

bus = can.interface.Bus(channel='can0', interface='socketcan')
bus.set_filters([{
    "can_id": 0x100,
    "can_mask": 0x700,
    "extended": False
}])

msg = can.Message(arbitration_id=0x100, is_extended_id=False)

try:
    bus.send(msg)
    print("reboot servoboard sent")
except can.CanError:
    print("Message NOT sent")

# Listen for all incoming messages
print("Listening for incoming messages...")

try:
    while True:
        message = bus.recv(timeout=1.0)  # Adjust timeout as needed
        if message:
            print(f"Received: ID=0x{message.arbitration_id:X}, Data={message.data}")
except KeyboardInterrupt:
    print("Stopped.")