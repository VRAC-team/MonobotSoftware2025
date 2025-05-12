import curses
import time
import argparse

from robot import can_utils, ServoBoard, Servo, utils

servos = {
    0: Servo(),
    1: Servo(),
    2: Servo(),
    3: Servo(),
    4: Servo(),
    5: Servo(),
    6: Servo(),
    7: Servo(),
    8: Servo(),
    9: Servo(),
    10: Servo(),
    11: Servo(),
    12: Servo(),
    13: Servo(),
    14: Servo(),
    15: Servo(),
}


def servo_id_type(arg_servo_id):
    servo_id = int(arg_servo_id)
    if servo_id not in servos.keys():
        raise argparse.ArgumentTypeError("servo_id is not in mapped servos")
    return servo_id


parser = argparse.ArgumentParser(description="Terminal-based servo tester")
parser.add_argument("servo_id", type=servo_id_type, help="servo_id (integer), must also be in mapped servos")
args = parser.parse_args()

SERVO_ID = args.servo_id
US_INCREMENT_FINE = 25
US_INCREMENT_COARSE = 200


def main(stdscr):
    curses.curs_set(0)
    curses.halfdelay(1)

    bus = can_utils.get_can_interface()
    if bus is None:
        print("no CAN bus found")
        quit()

    servoboard = ServoBoard(bus, servos)
    servoboard.reboot()

    servo = servos[SERVO_ID]

    power_enable = False
    servo_us = servo.min_us

    while True:
        stdscr.clear()
        stdscr.addstr(0, 0, "Servo tester: (Ctrl+C to quit)")
        stdscr.addstr(1, 0, "    UP/DOWN: coarse servo us increment/decrement")
        stdscr.addstr(2, 0, "    RIGHT/LEFT: fine servo us increment/decrement")
        stdscr.addstr(3, 0, "    SPACE: toggle power")
        stdscr.addstr(4, 0, f"servo_id:{SERVO_ID} servo_us:{servo_us}")
        stdscr.addstr(5, 0, f"power_enable:{power_enable}")
        stdscr.refresh()

        try:
            key = stdscr.getch()

            if key == curses.KEY_UP:
                servo_us += US_INCREMENT_FINE
                servo_us = utils.clamp(servo_us, servo.min_us, servo.max_us)
                servoboard.servo_write_us(SERVO_ID, servo_us)
            elif key == curses.KEY_DOWN:
                servo_us -= US_INCREMENT_FINE
                servo_us = utils.clamp(servo_us, servo.min_us, servo.max_us)
                servoboard.servo_write_us(SERVO_ID, servo_us)
            if key == curses.KEY_RIGHT:
                servo_us += US_INCREMENT_COARSE
                servo_us = utils.clamp(servo_us, servo.min_us, servo.max_us)
                servoboard.servo_write_us(SERVO_ID, servo_us)
            if key == curses.KEY_LEFT:
                servo_us -= US_INCREMENT_COARSE
                servo_us = utils.clamp(servo_us, servo.min_us, servo.max_us)
                servoboard.servo_write_us(SERVO_ID, servo_us)
            elif key == ord(" "):
                power_enable = not power_enable
                servoboard.enable_power(False, power_enable, False)

            time.sleep(0.05)
        except KeyboardInterrupt:
            servoboard.enable_power(False, False, False)
            break


if __name__ == "__main__":
    curses.wrapper(main)
