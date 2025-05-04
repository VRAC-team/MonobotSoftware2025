import evdev
import dataclasses


@dataclasses.dataclass
class GamepadUpdateData:
    x: float
    y: float
    rx: float
    ry: float
    z: float
    rz: float
    keys_active: list[int]
    keys_pressed: list[int]
    keys_released: list[int]


class Gamepad:
    def __init__(self):
        self.last_keys_active = []
        self.device = None

    def init(self, device_name: str):
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            print(device)
            if device_name in device.name:
                print(f"Found gamepad:{device}")
                self.device = device
                return True

        print("No gamepad found")
        return False

    def update(self):
        keys_active = self.device.active_keys()
        keys_pressed = []
        keys_released = []
        # BTN_A BTN_B BTN_X BTN_Y
        # BTN_TR BTN_TR
        # BTN_THUMBL BTN_THUMBR
        # BTN_SELECT BTN_START
        # BTN_DPAD_UP BTN_DPAD_DOWN BTN_DPAD_RIGHT BTN_DPAD_LEFT

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

        x = self.device.absinfo(evdev.ecodes.ABS_X)  # joystick left horizontal
        y = self.device.absinfo(evdev.ecodes.ABS_Y)  # joystick left vertical
        rx = self.device.absinfo(evdev.ecodes.ABS_RX)  # joystick right horizontal
        ry = self.device.absinfo(evdev.ecodes.ABS_RY)  # joystick right vertical
        z = self.device.absinfo(evdev.ecodes.ABS_Z)  # gachette left
        rz = self.device.absinfo(evdev.ecodes.ABS_RZ)  # gachette right

        self.last_keys_active = keys_active

        return GamepadUpdateData(
            x=x.value / x.max,
            y=y.value / y.max,
            rx=rx.value / rx.max,
            ry=ry.value / ry.max,
            z=z.value / z.max,
            rz=rz.value / rz.max,
            keys_active=keys_active,
            keys_pressed=keys_pressed,
            keys_released=keys_released,
        )
