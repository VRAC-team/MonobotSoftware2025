import re
import pathlib
import can
from can.interfaces.socketcan import SocketcanBus

def parse_header_file():
    path = pathlib.Path("can_identifiers.hpp")
    if not path.exists():
        path = pathlib.Path("../common/can_identifiers.hpp")
        if not path.exists():
            quit()

    define_pattern = re.compile(r'^\s*#define\s+(\w+)\s+(0x[0-9A-Fa-f]+)\b')
    defines = {}

    with path.open('r') as file:
        for line in file:
            line = line.split('//')[0].strip()
            match = define_pattern.match(line)
            if match:
                key, value = match.groups()
                defines[key] = int(value, 16)

    return defines

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