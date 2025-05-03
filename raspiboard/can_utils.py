import can
from can.interfaces.socketcan import SocketcanBus

def send(bus: can.Bus, msg: can.Message) -> bool:
    """
    Returns:
        bool: True if the message was accepted by the socket layer for transmission.
              False if a can.CanError was raised (e.g., bus off, send failure).
    Note:
        This function does not guarantee that the message was ACKed by any receiver on the bus!
        the can interface used is SocketCAN, a return value of True only confirms that the message was successfully
        sent to the CAN interface; it does not mean that another node received or acknowledged it.
        Be warned!
    """
    try:
        bus.send(msg)
        return True
    except can.CanError as e:
        return False

def get_can_interface(preferred_interface = ('can0', 'vcan0'), bitrate: int = 1000000):
    for chan in preferred_interface:
        try:
            bus = SocketcanBus(channel=chan, bitrate=bitrate, local_loopback=True)
            print(f"found SocketCAN interface: {bus}")
            return bus
        except Exception as e:
            print(e)
            pass
    print("No SocketCAN interface found")
    quit()