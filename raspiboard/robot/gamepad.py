import evdev


class Gamepad:
    def __init__(self, device_name: str | None = None):
        self.device_name: str | None = device_name
        self.device: evdev.InputDevice | None = None

        self.x = 0.0  # left stick horizontal
        self.y = 0.0  # left stick vertical
        self.rx = 0.0  # right stick horizontal
        self.ry = 0.0  # right stick vertical
        self.z = 0.0  # left trigger
        self.rz = 0.0  # right trigger

        self.keys_active: list[int] = []
        self.keys_pressed: list[int] = []
        self.keys_released: list[int] = []
        self.last_keys_active: list[int] = []

    def connect(self) -> bool:
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for dev in devices:
            name_match = (
                self.device_name in dev.name
                if self.device_name
                else "Controller" in dev.name or "Gamepad" in dev.name
            )
            if name_match:
                try:
                    dev.grab()
                    self.device = dev
                    print(f"Connected to: {dev.name} at {dev.path}")
                    return True
                except OSError:
                    print(f"Could not grab device {dev}")
                    self.device = None
                    return False
        return False

    def disconnect(self):
        if self.device:
            print("gamepad disconnected!")
            try:
                self.device.ungrab()
            except OSError:
                pass
        self.device = None

    def is_connected(self) -> bool:
        return self.device is not None

    def update(self) -> bool:
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

            self.keys_active = keys_active
            self.keys_pressed = keys_pressed
            self.keys_released = keys_released
            self.last_keys_active = keys_active

            x = self.device.absinfo(evdev.ecodes.ABS_X)
            y = self.device.absinfo(evdev.ecodes.ABS_Y)
            rx = self.device.absinfo(evdev.ecodes.ABS_RX)
            ry = self.device.absinfo(evdev.ecodes.ABS_RY)
            z = self.device.absinfo(evdev.ecodes.ABS_Z)
            rz = self.device.absinfo(evdev.ecodes.ABS_RZ)
            self.x = x.value / x.max
            self.y = y.value / y.max
            self.rx = rx.value / rx.max
            self.ry = ry.value / ry.max
            self.z = z.value / z.max
            self.rz = rz.value / rz.max

        except (SystemError, OSError):
            self.disconnect()
            return False

        return True
