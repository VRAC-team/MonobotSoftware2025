import logging
import threading
import time
import signal
import can

from .boards import MotorBoard, ServoBoard, IOBoard, PumpBoard
from .can_identifiers import CANIDS
from .gpio import GPIO
from .robot_config import RobotConfig
from .telemetry import telemetry
from .can_utils import get_can_interface, can_send
from .utils import setup_logging, setup_realtime


class Robot:
    def __init__(self, config: RobotConfig):
        self.config = config

        self.logger = logging.getLogger(self.__class__.__name__)

        self.bus = get_can_interface()

        self.stop_event = threading.Event()
        self.t_can_alive = threading.Thread(target=self.thread_can_alive)
        self.gpio = GPIO(self.config)

        self.motorboard = MotorBoard(self.bus)

        self.servoboard = ServoBoard(self.bus, self.config.SERVOS)
        self.ioboard = IOBoard(self.bus)
        self.pumpboard = PumpBoard(self.bus)
        self.notifier = can.Notifier(self.bus, [self.motorboard, self.servoboard, self.ioboard])

        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, sig, frame):
        self.logger.info("SIGINT received!")
        self.stop_event.set()

    def thread_can_alive(self):
        while not self.stop_event.is_set():
            msg = can.Message(arbitration_id=CANIDS.CANID_RASPI_ALIVE, is_extended_id=False)
            can_send(self.bus, msg)
            time.sleep(1)

    def start(self):
        telemetry.start(self.config.TELEMETRY_HOST_ADDR)
        setup_logging()
        setup_realtime()

        self.t_can_alive.start()

        self.motorboard.reboot()
        self.servoboard.reboot()
        self.ioboard.reboot()

    def process(self, t: float):
        self.servoboard.process(t)
        # self.ioboard.process(t)
        self.pumpboard.process(t)

    def stop(self):
        self.stop_event.set()

        self.notifier.stop(timeout=1)
        self.t_can_alive.join(timeout=1)
        self.bus.shutdown()
        telemetry.stop()
