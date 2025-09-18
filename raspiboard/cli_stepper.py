import curses
import argparse
import time
import can
import struct
import sys

from robot import can_utils, CANIDS
from robot.boards import IOBoard, StepperMotionResult


class StepperCLI:
    STEPS_INCREMENT_FINE = 20
    STEPS_INCREMENT_COARSE = 200
    ACCELERATION = 4000
    MAX_VELOCITY = 1200
    MAX_LOG_LINES = 6

    def __init__(self, bus: can.BusABC, stepper_id: int) -> None:
        self.bus = bus

        self.ioboard = IOBoard(bus)
        self.ioboard.reboot()

        self.stepper_id = stepper_id
        self.cmd_enable = False
        self.cmd_steps_absolute = 0

        self.last_alive = -1.0
        self.last_status = ""

        self.logs = []
        self.start_time = time.monotonic()

    def log(self, msg: str) -> None:
        elapsed = time.monotonic() - self.start_time
        entry = f"[{elapsed:7.3f}] {msg}"
        self.logs.append(entry)
        if len(self.logs) > self.MAX_LOG_LINES:
            self.logs = self.logs[-self.MAX_LOG_LINES :]

    def move_stepper(self, increment: int):
        self.cmd_steps_absolute += increment
        res = self.ioboard.goto_abs(self.stepper_id, self.cmd_steps_absolute, self.ACCELERATION, self.MAX_VELOCITY)
        if res == StepperMotionResult.IS_DOING_GOTO:
            self.log(f"move_stepper IS_DOING_GOTO abs={self.cmd_steps_absolute}")
        else:
            self.log(f"move_stepper error: {res.name}")

    def run(self, stdscr):
        curses.curs_set(0)
        curses.halfdelay(1)

        height, width = stdscr.getmaxyx()

        # Reserve bottom window for logs
        log_win_height = self.MAX_LOG_LINES + 2
        log_win = curses.newwin(log_win_height, width, 13, 0)
        log_win.box()

        stdscr.addstr(0, 0, "[cli_stepper] (Ctrl+C to quit)")
        stdscr.addstr(1, 0, "    UP/DOWN: coarse step increment/decrement")
        stdscr.addstr(2, 0, "    RIGHT/LEFT: fine step increment/decrement")
        stdscr.addstr(3, 0, "    SPACE: toggle enable")
        stdscr.addstr(5, 0, f"stepper_id:{self.stepper_id}")

        while True:
            while True:
                msg = self.bus.recv(0)
                if msg is None:
                    break

                match msg.arbitration_id:
                    case CANIDS.CANID_IO_STEPPER_ERROR_DISABLED_DURING_MOTION:
                        self.log("ERROR: CAN DISABLED_DURING_MOTION")

                    case CANIDS.CANID_IO_STEPPER_ERROR_NOT_ENABLED:
                        self.log("ERROR: CAN NOT_ENABLED")

                    case CANIDS.CANID_IO_STEPPER_ERROR_MOTION_IN_PROGRESS:
                        self.log("ERROR: CAN MOTION_IN_PROGRESS")

                    case CANIDS.CANID_IO_STEPPER_ERROR_INVALID_PARAMS:
                        self.log("ERROR: CAN INVALID_PARAMS")

                    case CANIDS.CANID_IO_STEPPER_HOME_STARTING:
                        self.log("CAN HOME_STARTING")

                    case CANIDS.CANID_IO_STEPPER_HOME_FAILED:
                        self.log("ERROR: CAN HOME_FAILED")

                    case CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED:
                        self.log("CAN HOME_SUCCEEDED")

                    case CANIDS.CANID_IO_STEPPER_GOTO_STARTING:
                        self.log("CAN GOTO_STARTING")

                    case CANIDS.CANID_IO_STEPPER_GOTO_FINISHED:
                        self.log("CAN GOTO_FINISHED")

                    case CANIDS.CANID_IO_STATUS:
                        enable, tors = struct.unpack(">?H", msg.data)
                        self.last_status = f"enable:{enable} tors:{tors:016b}"

                    case CANIDS.CANID_IO_ALIVE:
                        self.last_alive = time.monotonic()

            stdscr.addstr(6, 0, f"=> steps_abs:{self.cmd_steps_absolute}".ljust(width))
            stdscr.addstr(7, 0, f"=> enable:{self.cmd_enable} ")
            stdscr.addstr(8, 0, f"=> last_alive_timestamp:{self.last_alive:.1f} ")
            stdscr.addstr(9, 0, f"=> last_status {self.last_status}   ")
            stdscr.addstr(10, 0, f"=> stepper_state:{self.ioboard.steppers[self.stepper_id].get_state()}".ljust(width))

            log_win.erase()
            log_win.box()
            for i, line in enumerate(self.logs[-self.MAX_LOG_LINES :]):
                log_win.addstr(i + 1, 1, line[: width - 2])
            log_win.refresh()

            stdscr.refresh()

            key = stdscr.getch()
            if key == curses.KEY_UP:
                self.move_stepper(self.STEPS_INCREMENT_FINE)
            elif key == curses.KEY_DOWN:
                self.move_stepper(-self.STEPS_INCREMENT_FINE)
            elif key == curses.KEY_RIGHT:
                self.move_stepper(self.STEPS_INCREMENT_COARSE)
            elif key == curses.KEY_LEFT:
                self.move_stepper(-self.STEPS_INCREMENT_COARSE)
            elif key == ord(" "):
                self.cmd_enable = not self.cmd_enable
                self.ioboard.enable(self.cmd_enable)
                self.log(f"Enable toggled: {self.cmd_enable}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Terminal-based stepper tester")
    parser.add_argument("stepper_id", type=int, help="stepper_id (integer), must be in mapped servos")
    args = parser.parse_args()

    bus = can_utils.get_can_interface()
    bus.set_filters([{"can_id": 0x200, "can_mask": 0x700, "extended": False}])  # filters IDs from 0x200 to 0x2FF

    cli = StepperCLI(bus, args.stepper_id)

    try:
        curses.wrapper(cli.run)
    except KeyboardInterrupt:
        cli.ioboard.enable(False)
        print("\n[Interrupted] Exiting gracefully...")
        sys.exit(0)
    finally:
        bus.shutdown()
