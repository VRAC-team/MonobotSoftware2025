import curses
import argparse
import time
import can
import struct
import sys

from robot import can_utils, CANIDS, utils
from robot.boards import ServoBoard, Servo


class ServoCLI:
    US_INCREMENT_FINE = 10
    US_INCREMENT_COARSE = 100

    def __init__(self, bus: can.BusABC, servo_id: int) -> None:
        self.bus = bus
        self.servos = {
            0: Servo(min_us=500, max_us=2500),  # arriere / ascenseur / bras ext gauche / aimant (525 ON, 2500 OFF)
            1: Servo(min_us=500, max_us=2500),  # arriere / ascenseur / bras ext droite / aimant (500 ON, 2500 OFF)
            2: Servo(min_us=500, max_us=2500),  # arriere / ascenseur  / bras ext droite / bras (2225 PRISE, 1525 EXTERIEUR)
            3: Servo(min_us=500, max_us=2500),  # arriere / baguette gauche (BAS 1200, 1475 PRISE, 2225 RANGE HAUT)
            4: Servo(min_us=500, max_us=2500),  # arriere / base droite / aimant (575 ON, 2500 0FF)
            5: Servo(min_us=500, max_us=2500),  # arriere / baguette droite (BAS 2275, 1975 PRISE, 1225 RANGE HAUT)
            6: Servo(min_us=500, max_us=2500),  # arriere / ascenseur / bras ext droite / bras (905 PRISE, 1925 EXTERIEUR)
            7: Servo(min_us=500, max_us=2500),  # arriere / base gauche / aimant (2200 ON, 1100 OFF)
            8: Servo(min_us=500, max_us=2500),  # avant / ascenseur / bras ext droite / bras (1150 PRISE, 1775 EXTERIEUR)
            9: Servo(min_us=500, max_us=2500),  # avant / ascenseur / bras ext gauche / aimant (625 ON, 2500 OFF)
            10: Servo(min_us=500, max_us=2500),  # avant / ascenseur / bras ext droite / aimant (2325 ON, 500 OFF)
            11: Servo(min_us=500, max_us=2500),  # avant / ascenseur / bras ext gauche / bras (2275 PRISE, 1650 EXTERIEUR)
            12: Servo(min_us=500, max_us=2500),  # avant / baguette droite (1150 BAS, 1425 PRISE, 2175 RANGE HAUT)
            13: Servo(min_us=500, max_us=2500),  # avant / base droite / aimant (2325 ON, 500 OFF)
            14: Servo(min_us=500, max_us=2500),  # avant / baguette gauche (2175 BAS, 1850 PRISE, 1100 RANGE HAUT)
            15: Servo(min_us=500, max_us=2500),  # avant / base gauche / aimant (750 ON, 2500 OFF)
            16: Servo(min_us=500, max_us=2500),
            17: Servo(min_us=500, max_us=2500),
        }
        if servo_id not in self.servos:
            self.bus.shutdown()
            raise RuntimeError("servo_id:{servo_id} not mapped in servos!")

        self.servoboard = ServoBoard(bus, self.servos)
        self.servoboard.reboot()

        self.servo = self.servos[servo_id]
        self.id = servo_id
        self.cmd_us = self.servo.target_us
        self.cmd_enable = False

        self.last_error = "None"
        self.last_alive = -1.0
        self.last_status = "None"

    def set_servo(self, increment: int):
        self.cmd_us = utils.clamp(self.cmd_us + increment, self.servo.min_us, self.servo.max_us)
        if self.servoboard.servo_set_us(self.id, self.cmd_us):
            self.last_error = "None"
        else:
            self.last_error = "CAN BUS ERROR"

    def run(self, stdscr):
        curses.curs_set(0)
        curses.halfdelay(1)

        stdscr.addstr(0, 0, "[cli_servo] (Ctrl+C to quit)")
        stdscr.addstr(1, 0, "    UP/DOWN: coarse servo us increment/decrement")
        stdscr.addstr(2, 0, "    RIGHT/LEFT: fine servo us increment/decrement")
        stdscr.addstr(3, 0, "    SPACE: toggle power")
        stdscr.addstr(5, 0, f"id:{self.id} min_us:{self.servo.min_us} max_us:{self.servo.max_us}")

        while True:
            while True:
                msg = self.bus.recv(0)
                if msg is None:
                    break
                if msg.arbitration_id == CANIDS.CANID_SERVO_ALIVE:
                    self.last_alive = time.monotonic()
                elif msg.arbitration_id == CANIDS.CANID_SERVO_ERROR_NOT_ENABLED:
                    self.last_error = "CANID_SERVO_ERROR_NOT_ENABLED"
                elif msg.arbitration_id == CANIDS.CANID_SERVO_STATUS:
                    power1_adc, power2_adc, power3_adc, powers_en = struct.unpack(">HHHB", msg.data)
                    power1_en = bool(powers_en & 1)
                    power2_en = bool((powers_en >> 1) & 1)
                    power3_en = bool((powers_en >> 2) & 1)
                    self.last_status = f"en1:{power1_en} en2:{power2_en} en3:{power3_en}"

            stdscr.addstr(6, 0, f"=> us:{self.cmd_us}     ")
            stdscr.addstr(7, 0, f"=> power_enable:{self.cmd_enable} ")
            stdscr.addstr(8, 0, f"=> last_alive_timestamp:{self.last_alive:.1f}        ")
            stdscr.addstr(9, 0, f"=> last_status {self.last_status}    ")
            stdscr.addstr(10, 0, f"=> last_error:{self.last_error}")
            stdscr.refresh()

            key = stdscr.getch()
            if key == curses.KEY_UP:
                self.set_servo(self.US_INCREMENT_FINE)
            elif key == curses.KEY_DOWN:
                self.set_servo(-self.US_INCREMENT_FINE)
            elif key == curses.KEY_RIGHT:
                self.set_servo(self.US_INCREMENT_COARSE)
            elif key == curses.KEY_LEFT:
                self.set_servo(-self.US_INCREMENT_COARSE)
            elif key == ord(" "):
                self.cmd_enable = not self.cmd_enable
                self.servoboard.enable_power(self.cmd_enable, self.cmd_enable, self.cmd_enable)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Terminal-based servo tester")
    parser.add_argument("servo_id", type=int, help="servo_id (integer), must be in mapped servos")
    args = parser.parse_args()

    bus = can_utils.get_can_interface()
    bus.set_filters([{"can_id": 0x100, "can_mask": 0x700, "extended": False}])  # filters IDs from 0x100 to 0x1FF

    cli = ServoCLI(bus, args.servo_id)

    try:
        curses.wrapper(cli.run)
    except KeyboardInterrupt:
        cli.servoboard.enable_power(False, False, False)
        print("\n[Interrupted] Exiting gracefully...")
        sys.exit(0)
    finally:
        bus.shutdown()
