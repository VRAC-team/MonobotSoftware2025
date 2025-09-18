import time
import enum
from typing import Generator
import dataclasses

CONTORL_LOOP_PERIOD = 0.05


def close_to_position(x1: float, y1: float, x2: float, y2: float, tolerance=2.0):
    return abs(x1 - x2) < tolerance and abs(y1 - y2) < tolerance


class RobotOrientation(enum.Enum):
    AUTO = "auto"
    FRONT = "front"
    BACK = "back"


class MotionState(enum.Enum):
    STAY_AT_POSITION = "stay_at_position"
    LINE = "line"
    ROTATE = "rotate"
    LOOK_AT = "look_at"
    GOTO_XY = "goto_xy"
    WAYPOINT_XY = "waypoint_xy"
    WAIT_NEXT_WAYPOINT_XY = "wait_next_waypoint_xy"
    HOME = "home"
    ERROR = "error"


class MotionError(enum.Enum):
    TIMEOUT = "timeout"
    BLOCKED = "blocked"
    FORCE_BRAKED = "force_braked"


class TrajectoryManager:
    def __init__(self):
        self.motion_start_time = 0.0
        self.motion_estimated_time = 0.0
        self.state = MotionState.STAY_AT_POSITION
        self.error: None | MotionError = None

    def _start_motion(self, state: MotionState, estimated_time: float):
        self.motion_start_time = time.monotonic()
        self.state = state
        self.motion_estimated_time = estimated_time
        self.error = None

    def home(self, orientation: RobotOrientation):
        self._start_motion(MotionState.HOME, 2.0)
        print(f"TrajectoryManager home orientation:{orientation} time:{self.motion_estimated_time}")

    def line(self, distance: float, timeout: float | None = None):
        self._start_motion(MotionState.LINE, (distance / 1000.0) * 2.0)
        print(f"TrajectoryManager line distance:{distance} time:{self.motion_estimated_time}")

    def rotate(self, degree: float):
        self._start_motion(MotionState.ROTATE, (degree / 360.0) * 2.0)
        print(f"TrajectoryManager rotate degree:{degree} time:{self.motion_estimated_time}")

    def look_at(self, x: float, y: float):
        self._start_motion(MotionState.LOOK_AT, 0.5)
        print(f"TrajectoryManager look_at x:{x} y:{y} time:{self.motion_estimated_time}")

    def goto_xy(self, x: float, y: float):
        self._start_motion(MotionState.GOTO_XY, (x / 1000.0 + y / 1000.0) * 2.0 + 0.5)
        print(f"TrajectoryManager goto_xy x:{x} y:{y} time:{self.motion_estimated_time}")

    def wait(self) -> Generator[None, None, None | MotionError]:
        """
        used to line for home, line, rotate, look_at, goto_xy
        """
        print("TrajectoryManager wait", end="", flush=True)
        while self.state != MotionState.STAY_AT_POSITION and not self.error:
            print(".", end="", flush=True)
            yield
        print(f"DONE! err:{self.error}")
        return self.error

    def waypoint_xy_start(self, x: float, y: float):
        self._start_motion(MotionState.WAYPOINT_XY, (x / 1000.0 + y / 1000.0) * 2.0)
        print(f"TrajectoryManager waypoint_xy_start x:{x} y:{y} time:{self.motion_estimated_time}")

    def waypoint_xy_chained(self, x: float, y: float, has_more_waypoints_to_come: bool = False):
        self._start_motion(MotionState.WAYPOINT_XY, (x / 1000.0 + y / 1000.0) * 2.0)
        print(
            f"TrajectoryManager waypoint_xy_chained x:{x} y:{y} has_more_waypoints_to_come:{has_more_waypoints_to_come} time:{self.motion_estimated_time}"
        )

    def wait_waypoint_xy(self) -> Generator[None, None, None | MotionError]:
        print("TrajectoryManager wait_waypoint_xy", end="", flush=True)
        while self.state != MotionState.WAIT_NEXT_WAYPOINT_XY and not self.error:
            print(".", end="", flush=True)
            yield
        print(f"DONE! err:{self.error}")
        return self.error

    def process(self, t: float) -> tuple[float, float]:
        if self.state == MotionState.WAIT_NEXT_WAYPOINT_XY:
            self.state = MotionState.ERROR
            return (0.0, 0.0)

        if self.state in [MotionState.LINE, MotionState.ROTATE, MotionState.HOME, MotionState.LOOK_AT, MotionState.GOTO_XY]:
            if t - self.motion_start_time >= self.motion_estimated_time:
                self.state = MotionState.STAY_AT_POSITION
        if self.state == MotionState.WAYPOINT_XY:
            if t - self.motion_start_time >= self.motion_estimated_time:
                self.state = MotionState.WAIT_NEXT_WAYPOINT_XY
        return (0.0, 0.0)


class AvoidanceManager:
    def enable(self):
        print("AvoidanceManager enable")

    def disable(self):
        print("AvoidanceManager disable")


class ServoBoard:
    CONTORL_LOOP_PERIOD = 0.1
    NB_SERVOS = 20
    DEFAULT_INCREMENT_US = 150  # [us/s^2] acceleration rate at each process
    DEFAULT_POSITION_US = 2000

    def __init__(self):
        self.is_moving = [False] * self.NB_SERVOS
        self.increments_us = [self.DEFAULT_INCREMENT_US] * self.NB_SERVOS
        self.target_us = [self.DEFAULT_POSITION_US] * self.NB_SERVOS
        self.last_position_us = self.target_us.copy()
        self.last_process_t = 0.0

    def write_us(self, id: int, us: int, increment: float = DEFAULT_INCREMENT_US) -> bool:
        self.is_moving[id] = True
        self.increments_us[id] = increment
        self.target_us[id] = us
        print(f"ServoBoard servo_write_us id:{id} us:{us} increment:{increment}")

    def wait(self):
        print("ServoBoard wait", end="", flush=True)
        while any(self.is_moving):
            print(".", end="", flush=True)
            yield
        print("DONE!")

    def process(self, t: float):
        if t >= self.last_process_t + self.CONTORL_LOOP_PERIOD:
            for id, target_us in enumerate(self.target_us):
                if not self.is_moving[id]:
                    continue

                pos = 0.0
                if target_us > self.last_position_us[id]:
                    pos = self.last_position_us[id] + self.increments_us[id]
                    if pos >= target_us:
                        pos = target_us
                        self.is_moving[id] = False
                elif target_us < self.last_position_us[id]:
                    pos = self.last_position_us[id] - self.increments_us[id]
                    if pos <= target_us:
                        pos = target_us
                        self.is_moving[id] = False

                self.last_position_us[id] = pos

        self.last_process_t = t


class IOBoard:
    NB_STEPPERS = 5

    def __init__(self):
        self.is_active = [False] * IOBoard.NB_STEPPERS
        self.start_time = [0.0] * IOBoard.NB_STEPPERS
        self.estimated_time = [0.0] * IOBoard.NB_STEPPERS
        self.tors = [False] * 16

    def get_state(self, id: int) -> bool:
        return self.tors[id]

    def move_absolute(self, id: int, steps: int):
        self.start_time[id] = time.monotonic()
        self.estimated_time[id] = (steps / 8500.0) * 1.5
        self.is_active[id] = True
        print(f"IOBoard move_absolute id:{id} steps:{steps}")

    def wait(self):
        print("IOBoard wait", end="", flush=True)
        while any(self.is_active):
            now = time.monotonic()
            for id, is_active in enumerate(self.is_active):
                if is_active and now - self.start_time[id] >= self.estimated_time[id]:
                    self.is_active[id] = False
            print(".", end="", flush=True)
            yield
        print("DONE!")


class VacuumBoard:
    AUTO_RELEASE_CLOSE_AFTER = 0.5  # [s]

    def __init__(self):
        self.auto_close_valve = {}

    def activate(self, id: int):
        print(f"VacuumBoard enable id:{id}")

    def deactivate_with_valve(self, pump_id: int, valve_id: int):
        print(f"VacuumBoard deactivate_with_valve pump_id:{pump_id} valve_id:{valve_id}")
        self.auto_close_valve[valve_id] = time.monotonic() + self.AUTO_RELEASE_CLOSE_AFTER

    def wait(self):
        print("VacuumBoard wait", end="", flush=True)
        while self.auto_close_valve:
            print(".", end="", flush=True)
            yield
        print("DONE!")

    def process(self, t: float):
        for valve_id in list(self.auto_close_valve.keys()):
            close_at = self.auto_close_valve[valve_id]
            if t >= close_at:
                self.auto_close_valve.pop(valve_id)
                print(f"VacuumBoard process auto release close valve_id:{valve_id}")


@dataclasses.dataclass(frozen=True)
class VacuumID:
    AV_POMPE_D = 0
    AV_POMPE_G = 1
    AR_POMPE_D = 2
    AR_POMPE_G = 3
    AV_VALVE_D = 4
    AV_VALVE_G = 5
    AR_VALVE_D = 6
    AR_VALVE_G = 7


@dataclasses.dataclass(frozen=True)
class SensorsID:
    AV_BRAS_D = 0
    AV_BRAS_G = 1
    AV_BASE_D = 2
    AV_BASE_G = 4
    AV_ASCENSEUR_HOME = 5

    AR_BRAS_D = 6
    AR_BRAS_G = 7
    AR_BASE_D = 8
    AR_BASE_G = 9
    AR_ASCENSEUR_HOME = 10


@dataclasses.dataclass(frozen=True)
class ServoID:
    AV_PLATEFORME = 0
    AV_BRAS_D = 1
    AV_BRAS_D_AIMANT = 2
    AV_BRAS_G = 3
    AV_BRAS_G_AIMANT = 4
    AV_BASE_D_BAGUETTE = 5
    AV_BASE_D_AIMANT = 6
    AV_BASE_G_BAGUETTE = 7
    AV_BASE_G_AIMANT = 8

    AR_PLATEFORME = 9
    AR_BRAS_D = 10
    AR_BRAS_D_AIMANT = 11
    AR_BRAS_G = 12
    AR_BRAS_G_AIMANT = 13
    AR_BASE_D_BAGUETTE = 14
    AR_BASE_D_AIMANT = 15
    AR_BASE_G_BAGUETTE = 16
    AR_BASE_G_AIMANT = 17


@dataclasses.dataclass(frozen=True)
class ServoAvPlatforme:
    ID = 0
    US_RANGER = 500
    US_PRISE_PLATFORME = 2500


@dataclasses.dataclass(frozen=True)
class StepperAvant:
    ID = 0
    STEPS_ETAGE1 = 8500
    STEPS_ETAGE2 = 2000
    STEPS_TOP = 0


tm = TrajectoryManager()
am = AvoidanceManager()
sb = ServoBoard()
io = IOBoard()
vb = VacuumBoard()


def exception_on_error(gen) -> Generator[None, None, None]:
    err = yield from gen
    if err:
        raise Exception(err)


def delay(duration: float):
    start = time.monotonic()
    print(f"delay duration:{duration} start", end="")
    while time.monotonic() - start < duration:
        print(".", end="", flush=True)
        yield
    print("DONE")


def action_prepare_match():
    print("== action_prepare_match ==")
    # aligner le robot sur le trait de couleur extérieur
    # attendre jack

    # disable avoidance

    tm.home(RobotOrientation.BACK)
    yield from exception_on_error(tm.wait())

    tm.line(200)
    yield from exception_on_error(tm.wait())

    tm.rotate(90)
    yield from exception_on_error(tm.wait())

    tm.line(100)
    yield from exception_on_error(tm.wait())

    tm.rotate(90)
    yield from exception_on_error(tm.wait())

    tm.line(100)
    yield from exception_on_error(tm.wait())

    yield from delay(1.0)

    tm.home(RobotOrientation.FRONT)
    yield from exception_on_error(tm.wait())

    # enable avoidance


def action_depart_banderole_et_rush():
    print("== action_depart_banderole_et_rush ==")
    # ASSERT WE ARE AT start position

    io.move_absolute(0, 0)
    sb.write_us(ServoAvPlatforme.ID, ServoAvPlatforme.US_PRISE_PLATFORME)

    tm.waypoint_xy_start(200, 0)
    yield from tm.wait_waypoint_xy()
    tm.waypoint_xy_chained(250, 100, True)
    yield from tm.wait_waypoint_xy()
    tm.waypoint_xy_chained(250, 200)
    yield from tm.wait_waypoint_xy()


def action_construire_stock_face_avant():
    print("== action_construire_stock_face_avant ==")
    # ASSERT we are at slot entrypoint
    # ASSERT baguettes en prise
    # ASSERT ascenseur est a position etage 1
    # ASSERT pompes actives

    slot_origin = (100, 100)

    # lookat slot origin
    tm.look_at(slot_origin[0], slot_origin[1])
    yield from tm.wait()

    can_build_level_2 = (
        io.get_state(SensorsID.AV_BRAS_D)
        and io.get_state(SensorsID.AV_BRAS_G)
        and io.get_state(SensorsID.AV_BASE_D)
        and io.get_state(SensorsID.AV_BASE_G)
    )
    can_build_level_2 = True

    # séquence de construction 2 étages
    if can_build_level_2:
        print("\t_building level 2_")
        # 1. bras vers extérieur
        sb.write_us(ServoID.AR_BRAS_D, 500)
        sb.write_us(ServoID.AR_BRAS_G, 2500)
        yield from sb.wait()
        # 2. petite attente de sécurité
        yield from delay(0.4)
        # 3. monter ascenseur
        io.move_absolute(StepperAvant.ID, StepperAvant.STEPS_ETAGE2)
        yield from io.wait()
        # 4. bras vers intérieur
        sb.write_us(ServoID.AR_BRAS_D, 2500)
        sb.write_us(ServoID.AR_BRAS_G, 500)
        yield from sb.wait()

    tm.line(200)
    yield from tm.wait()

    sb.write_us(ServoID.AR_BASE_D_BAGUETTE, 500)
    sb.write_us(ServoID.AR_BASE_G_BAGUETTE, 500)
    sb.write_us(ServoID.AV_PLATEFORME, ServoAvPlatforme.US_RANGER)
    vb.deactivate_with_valve(VacuumID.AV_POMPE_D, VacuumID.AV_VALVE_D)
    vb.deactivate_with_valve(VacuumID.AV_POMPE_G, VacuumID.AV_VALVE_G)
    yield from sb.wait()

    tm.line(-200)
    yield from tm.wait()


def action_test_servos():
    print("== action_test_servos ==")
    vb.activate(VacuumID.AV_POMPE_D)
    vb.activate(VacuumID.AV_POMPE_G)
    sb.write_us(ServoID.AR_BRAS_D, 500)
    sb.write_us(ServoID.AR_BRAS_G, 2500)
    yield from sb.wait()

    vb.deactivate_with_valve(VacuumID.AV_POMPE_D, VacuumID.AV_VALVE_D)
    vb.deactivate_with_valve(VacuumID.AV_POMPE_G, VacuumID.AV_VALVE_G)
    sb.write_us(ServoID.AR_BRAS_D, 2500)
    yield from vb.wait()
    yield from sb.wait()


def action_ne_rien_faire():
    print("== action_ne_rien_faire ==")
    while True:
        yield


def main():
    start_time = time.monotonic()
    tasks = [
        action_test_servos(),
        action_prepare_match(),
        action_depart_banderole_et_rush(),
        action_construire_stock_face_avant(),
        action_ne_rien_faire(),
    ]
    current_task = tasks.pop(0)
    next(current_task)

    while True:
        t = time.monotonic()

        if t - start_time > 50.0:
            print("fin du match!")
            break

        # odometry.update()
        pwm_right, pwm_left = tm.process(t)
        sb.process(t)
        vb.process(t)
        # motors.set_pwm(right, left)

        try:
            next(current_task)
        except StopIteration:
            if tasks:
                current_task = tasks.pop(0)
            else:
                print("all tasks done!")
                break

        time.sleep(0.1)

    print("bye")


main()
