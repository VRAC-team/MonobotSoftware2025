import socket
import threading
import time
import queue
from typing import Any


class Telemetry:
    def __init__(self) -> None:
        self.address = None
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.queue: queue.Queue[bytes] = queue.Queue()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._telemetry_loop, daemon=True)

    def start(self, address: tuple[str, int]) -> bool:
        if self.thread.is_alive():
            return False

        self.address = address
        self.thread.start()
        return True

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join()

    def send(self, name: str, value: Any) -> None:
        timestamp_ms = time.time() * 1000
        try:
            self.queue.put_nowait(f"{name}:{timestamp_ms:.0f}:{value}".encode())
        except queue.Full:
            pass

    def _telemetry_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                data = self.queue.get(timeout=0.1)
                self.socket.sendto(data, self.address)
            except queue.Empty:
                continue


telemetry = Telemetry()
