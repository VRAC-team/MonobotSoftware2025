import gpiod
import datetime
import threading

from robot.parameters import RobotParameters


class GPIO:
    def __init__(self, params: RobotParameters):
        self.params = params
        self.t_watch = threading.Thread(target=self.thread_watch, daemon=True)
        self.stop_event = threading.Event()
        self.starter_removed_event = threading.Event()
        self.starter_inserted_event = threading.Event()

        self.chip = gpiod.Chip("/dev/gpiochip0")
        self.lines_req = gpiod.request_lines(
            "/dev/gpiochip0",
            consumer="thread_watch",
            config={
                (self.params.GPIO_START, self.params.GPIO_SHUTDOWN): gpiod.LineSettings(
                    direction=gpiod.line.Direction.INPUT,
                    active_low=True,
                    bias=gpiod.line.Bias.DISABLED,
                    edge_detection=gpiod.line.Edge.BOTH,
                    debounce_period=datetime.timedelta(milliseconds=10),
                )
            },
        )

    def start(self):
        self.t_watch.start()

    def stop(self):
        self.stop_event.set()
        self.t_watch.join(timeout=5)

    def thread_watch(self):
        while not self.stop_event.is_set():
            if not self.lines_req.wait_edge_events(timeout=0.5):
                continue

            for event in self.lines_req.read_edge_events():
                if event.line_offset == self.params.GPIO_START:
                    if event.event_type == gpiod.edge_event.EdgeEvent.Type.RISING_EDGE:
                        self.starter_inserted_event.set()
                    elif event.event_type == gpiod.edge_event.EdgeEvent.Type.FALLING_EDGE:
                        self.starter_removed_event.set()

    def is_starter_present(self) -> bool:
        return self.lines_req.get_value(self.params.GPIO_START)

    def wait_for_starter_inserted(self):
        self.starter_inserted_event.clear()
        self.starter_inserted_event.wait()

    def wait_for_starter_removed(self):
        self.starter_removed_event.clear()
        self.starter_removed_event.wait()
