import curses
import time
import can
import struct
import sys

from robot import can_utils, CANIDS
from robot.boards import IOBoard


class SensorCLI:
    def __init__(self, bus: can.BusABC) -> None:
        self.bus = bus

        self.ioboard = IOBoard(bus)
        self.ioboard.reboot()

        self.last_alive_timestamp = -1.0
        self.last_status_timestamp = -1.0
        self.last_tors = 0

    def run(self, stdscr):
        curses.curs_set(0)
        curses.halfdelay(1)

        stdscr.addstr(0, 0, "[cli_sensors] (Ctrl+C to quit)")
        stdscr.addstr(1, 0, "=> last_alive_timestamp:")
        stdscr.addstr(2, 0, "=> last_status_timestamp:")

        for i in range(16):
            stdscr.addstr(4 + i, 0, f"TOR {i:01d} =")

        while True:
            while True:
                msg = self.bus.recv(0)
                if msg is None:
                    break

                match msg.arbitration_id:
                    case CANIDS.CANID_IO_STATUS:
                        self.last_status_timestamp = time.monotonic()
                        _, tors = struct.unpack(">?H", msg.data)
                        self.last_tors = tors

                    case CANIDS.CANID_IO_ALIVE:
                        self.last_alive_timestamp = time.monotonic()

            stdscr.addstr(1, 28, f"{self.last_alive_timestamp:.1f}   ")
            stdscr.addstr(2, 28, f"{self.last_status_timestamp:.1f}   ")

            for i in range(16):
                value = (self.last_tors >> i) & 1
                stdscr.addstr(4 + i, 9, f"{value}")

            stdscr.refresh()


if __name__ == "__main__":
    bus = can_utils.get_can_interface()
    bus.set_filters([{"can_id": 0x200, "can_mask": 0x700, "extended": False}])  # filters IDs from 0x200 to 0x2FF

    cli = SensorCLI(bus)

    try:
        curses.wrapper(cli.run)
    except KeyboardInterrupt:
        cli.ioboard.enable(False)
        print("\n[Interrupted] Exiting gracefully...")
        sys.exit(0)
    finally:
        bus.shutdown()
