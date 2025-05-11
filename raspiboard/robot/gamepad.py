import evdev
import logging
import threading
import dataclasses


@dataclasses.dataclass(frozen=True)
class GamepadState:
    x: float = 0.0  # left stick horizontal [-1,1]
    y: float = 0.0  # left stick vertical [-1,1]
    rx: float = 0.0  # right stick horizontal [-1,1]
    ry: float = 0.0  # right stick vertical [-1,1]
    z: float = 0.0  # left trigger [-1,1]
    rz: float = 0.0  # right trigger [-1,1]

    keys_active: tuple[int, ...] = dataclasses.field(default_factory=tuple)
    keys_pressed: tuple[int, ...] = dataclasses.field(default_factory=tuple)
    keys_released: tuple[int, ...] = dataclasses.field(default_factory=tuple)


class Gamepad:
    def __init__(self, device_name: str | None = None):
        self.device_name: str | None = device_name
        self.device: evdev.InputDevice | None = None

        self.state = GamepadState()
        self.last_keys_active: list[int] = []
        self.lock = threading.Lock()

        self.logger = logging.getLogger(self.__class__.__name__)

    def get_state(self) -> GamepadState:
        with self.lock:
            return self.state

    def connect(self) -> bool:
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for dev in devices:
            name_match = (
                self.device_name in dev.name if self.device_name else "Controller" in dev.name or "Gamepad" in dev.name
            )
            if name_match:
                try:
                    dev.grab()
                    self.device = dev
                    self.logger.info("Connected to %s at %s", dev.name, dev.path)
                    return True
                except OSError:
                    self.logger.error("Could not grab device %s", dev)
                    self.device = None
                    return False
        return False

    def disconnect(self):
        if self.device:
            self.logger.info("Disconnect")
            try:
                self.device.ungrab()
            except OSError:
                pass
        self.device = None

    def is_connected(self) -> bool:
        return self.device is not None

    def update(self) -> GamepadState | None:
        try:
            keys_active = self.device.active_keys()
            keys_pressed = []
            keys_released = []

            # dpad is an axis for some reason, append it to keys_active
            x = self.device.absinfo(evdev.ecodes.ABS_HAT0X)
            y = self.device.absinfo(evdev.ecodes.ABS_HAT0Y)
            if x.value == 1:
                keys_active.append(evdev.ecodes.BTN_DPAD_RIGHT)
            elif x.value == -1:
                keys_active.append(evdev.ecodes.BTN_DPAD_LEFT)
            if y.value == 1:
                keys_active.append(evdev.ecodes.BTN_DPAD_DOWN)
            elif y.value == -1:
                keys_active.append(evdev.ecodes.BTN_DPAD_UP)

            for k in keys_active:
                if k not in self.last_keys_active:
                    keys_pressed.append(k)
            for k in self.last_keys_active:
                if k not in keys_active:
                    keys_released.append(k)

            abs_x = self.device.absinfo(evdev.ecodes.ABS_X)
            abs_y = self.device.absinfo(evdev.ecodes.ABS_Y)
            abs_rx = self.device.absinfo(evdev.ecodes.ABS_RX)
            abs_ry = self.device.absinfo(evdev.ecodes.ABS_RY)
            abs_z = self.device.absinfo(evdev.ecodes.ABS_Z)
            abs_rz = self.device.absinfo(evdev.ecodes.ABS_RZ)
            x = abs_x.value / abs_x.max
            y = abs_y.value / abs_y.max
            rx = abs_rx.value / abs_rx.max
            ry = abs_ry.value / abs_ry.max
            z = abs_z.value / abs_z.max
            rz = abs_rz.value / abs_rz.max

            new_state = GamepadState(
                x=x,
                y=y,
                rx=rx,
                ry=ry,
                z=z,
                rz=rz,
                keys_active=tuple(keys_active),
                keys_pressed=tuple(keys_pressed),
                keys_released=tuple(keys_released),
            )

            with self.lock:
                self.last_keys_active = keys_active
                self.state = new_state

            return self.state

        except (SystemError, OSError):
            self.disconnect()
            return None
