import logging
import can
import threading
import time
import signal

from robot import MotorBoard, ServoBoard, Servo, IOBoard, CANIDS, GPIO, RobotParameters, telemetry, utils, can_utils


class Robot:
    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)

        self.bus = can_utils.get_can_interface()
        if self.bus is None:
            self.logger.error("no CAN bus found, quitting..")
            quit()

        self.params = RobotParameters()

        self.stop_event = threading.Event()
        self.t_can_alive = threading.Thread(target=self.thread_can_alive)
        self.gpio = GPIO(self.params)

        self.motorboard = MotorBoard(self.bus)
        self.servoboard = ServoBoard(
            self.bus,
            {
                8: Servo(min_us=675, max_us=2125),  # ON / OFF
                9: Servo(min_us=900, max_us=2225),  # OFF / ON
                10: Servo(min_us=500, max_us=1750),  # ON / OFF
                11: Servo(min_us=525, max_us=1750),  # OFF / ON
            },
        )
        self.ioboard = IOBoard(self.bus)
        self.notifier = can.Notifier(self.bus, [self.motorboard, self.servoboard, self.ioboard])

        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, sig, frame):
        self.logger.info("SIGINT received!")
        self.stop_event.set()

    def thread_can_alive(self):
        while not self.stop_event.is_set():
            msg = can.Message(arbitration_id=CANIDS.CANID_RASPI_ALIVE, is_extended_id=False)
            can_utils.send(self.bus, msg)
            time.sleep(1)

    def start(self):
        telemetry.start(("192.168.0.10", 47269))
        utils.setup_logging()
        utils.setup_realtime()

        self.gpio.start()
        self.t_can_alive.start()

        self.motorboard.reboot()
        self.servoboard.reboot()
        self.ioboard.reboot()

    def stop(self):
        self.stop_event.set()

        self.notifier.stop(timeout=1)
        self.t_can_alive.join(timeout=1)
        self.bus.shutdown()
        self.gpio.stop()
        telemetry.stop()
