from can.interfaces.socketcan import SocketcanBus

def get_can_interface(preferred_interface = ('can0', 'vcan0'), bitrate: int = 1000000):
    for chan in preferred_interface:
        try:
            bus = SocketcanBus(channel=chan, bitrate=bitrate, local_loopback=False)
            print(f"found SocketCAN interface: {bus}")
            return bus
        except Exception as e:
            print(e)
            pass
    print("No SocketCAN interface found")
    quit()