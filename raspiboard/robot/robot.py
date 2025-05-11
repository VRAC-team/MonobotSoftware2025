import logging
import can
import threading
import time
import datetime
import gpiod
import signal

from robot import MotorBoard, ServoBoard, IOBoard, CANIDS, RobotController, RobotParameters, telemetry, utils, can_utils


class Robot:
    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)

        self.bus = can_utils.get_can_interface()
        if self.bus is None:
            self.logger.error("no CAN bus found, quitting..")
            quit()

        self.params = RobotParameters()

        self.stop_event = threading.Event()
        self.t_watch_gpios = threading.Thread(target=self.thread_watch_gpios)
        self.t_can_alive = threading.Thread(target=self.thread_can_alive)

        self.motorboard = MotorBoard(self.bus)
        self.servoboard = ServoBoard(self.bus)
        self.ioboard = IOBoard(self.bus)
        self.robotcontroller = RobotController(self.params)
        self.motorboard.set_status_callback(self.robotcontroller.on_status)

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

    def thread_watch_gpios(self):
        with gpiod.request_lines(
            "/dev/gpiochip0",
            consumer="gpiod_consumer",
            config={
                (5, 6): gpiod.LineSettings(
                    direction=gpiod.line.Direction.INPUT,
                    bias=gpiod.line.Bias.DISABLED,
                    edge_detection=gpiod.line.Edge.BOTH,
                    debounce_period=datetime.timedelta(milliseconds=10),
                )
            },
        ) as request:
            while not self.stop_event.is_set():
                if not request.wait_edge_events(timeout=0.5):
                    continue

                for event in request.read_edge_events():
                    print(event)

    def start(self):
        telemetry.start(("192.168.0.10", 47269))
        utils.setup_logging()
        utils.setup_realtime()

        self.t_watch_gpios.start()
        self.t_can_alive.start()

        self.motorboard.reboot()
        self.servoboard.reboot()
        self.ioboard.reboot()

    def stop(self):
        self.notifier.stop()

        self.t_watch_gpios.join()
        self.t_can_alive.join()
        self.bus.shutdown()
        telemetry.stop()
