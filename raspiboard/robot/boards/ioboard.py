import can
import struct
from enum import Enum, auto
import logging
import threading

import robot.can_utils as can_utils
from robot.can_identifiers import CANIDS


class StepperState(Enum):
    IDLE = auto()

    HAS_REQUESTED_HOME = auto()
    HAS_REQUESTED_GOTO = auto()

    IS_DOING_HOME = auto()
    IS_DOING_GOTO = auto()

    FLAG_HOME_SUCCEEDED = auto()
    FLAG_HOME_FAILED = auto()
    FLAG_GOTO_FINISHED = auto()
    FLAG_ERROR_DISABLED_DURING_MOTION = auto()
    FLAG_ERROR_NOT_ENABLED = auto()
    FLAG_ERROR_MOTION_IN_PROGRESS = auto()
    FLAG_ERROR_INVALID_PARAM = auto()

    ERROR_MOTION_TIMEOUT = auto()


class StepperMotionResult(Enum):
    IS_DOING_HOME = auto()
    IS_DOING_GOTO = auto()

    HOME_SUCCEEDED = auto()
    HOME_FAILED = auto()
    GOTO_FINISHED = auto()

    ERROR_DISABLED_DURING_MOTION = auto()
    ERROR_NOT_ENABLED = auto()
    ERROR_MOTION_IN_PROGRESS = auto()
    ERROR_INVALID_PARAM = auto()

    ERROR_REQUEST_TIMEOUT = auto()
    ERROR_MOTION_TIMEOUT = auto()
    ERROR_CAN_SEND = auto()
    ERROR_WAS_NOT_DOING_MOTION = auto()


class StepperError(Exception):
    pass


class StepperTimeoutError(StepperError):
    pass


class StepperDesyncError(StepperError):
    pass


class Stepper:
    def __init__(self):
        self.position: int = 0
        self.state: StepperState = StepperState.IDLE
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)

    def get_state(self) -> StepperState:
        with self.lock:
            return self.state

    def set_state(self, new_state: StepperState):
        with self.condition:
            self.state = new_state
            self.condition.notify_all()

    def wait_for_state_change(self, from_state: StepperState, timeout: float) -> tuple[StepperState, bool]:
        with self.condition:
            if self.state != from_state:
                return self.state, False

            timed_out = not self.condition.wait(timeout)
            return self.state, timed_out


def default_stepper_mapping() -> dict[int, Stepper]:
    return {4: Stepper()}


class IOBoard(can.Listener):
    def __init__(self, bus: can.Bus, steppers: dict[int, Stepper] = None):
        self.bus: can.BusABC = bus
        self.steppers: dict[int, Stepper] = steppers or default_stepper_mapping()

        for id in self.steppers.keys():
            if not self._is_valid_id(id):
                raise ValueError(f"Stepper ID:{id} is out of range. Valid IDs are between 0 and 4.")

        self.logger = logging.getLogger(self.__class__.__name__)

    def _is_valid_id(self, id: int) -> bool:
        return 0 <= id <= 4

    def _clear_flags(self, stepper_id: int):
        if not self._is_valid_id(stepper_id):
            return

        if self.steppers[stepper_id].get_state() not in [
            StepperState.IDLE,
            StepperState.HAS_REQUESTED_HOME,
            StepperState.HAS_REQUESTED_GOTO,
            StepperState.IS_DOING_HOME,
            StepperState.IS_DOING_GOTO,
        ]:
            self.steppers[stepper_id].set_state(StepperState.IDLE)

    def on_message_received(self, msg: can.Message) -> None:
        if not (0x200 <= msg.arbitration_id <= 0x2FF):
            return

        match msg.arbitration_id:
            case CANIDS.CANID_IO_STEPPER_ERROR_DISABLED_DURING_MOTION:
                (steppers_disabled_bitset,) = struct.unpack(">B", msg.data)

                for stepper_id in range(5):
                    has_been_disabled = (steppers_disabled_bitset >> stepper_id & 1) == 1
                    if has_been_disabled:
                        if self.steppers[stepper_id].get_state() not in [
                            StepperState.HAS_REQUESTED_HOME,
                            StepperState.HAS_REQUESTED_GOTO,
                            StepperState.IS_DOING_HOME,
                            StepperState.IS_DOING_GOTO,
                        ]:
                            raise StepperDesyncError(
                                f"Reiceived DISABLED_DURING_MOTION but was not doing motion (state:{self.steppers[stepper_id].get_state()})"
                            )

                        self.steppers[stepper_id].set_state(StepperState.FLAG_ERROR_DISABLED_DURING_MOTION)
                        self.logger.debug("DISABLED_DURING_MOTION (stepper_id:%d)", stepper_id)

            case CANIDS.CANID_IO_STEPPER_ERROR_NOT_ENABLED:
                (stepper_id,) = struct.unpack(">B", msg.data)
                if not self._is_valid_id(stepper_id):
                    return

                if self.steppers[stepper_id].get_state() not in [
                    StepperState.HAS_REQUESTED_HOME,
                    StepperState.HAS_REQUESTED_GOTO,
                ]:
                    raise StepperDesyncError(
                        f"Reiceived ERROR_NOT_ENABLED but has not requested motion (state:{self.steppers[stepper_id].get_state()})"
                    )

                self.steppers[stepper_id].set_state(StepperState.FLAG_ERROR_NOT_ENABLED)
                self.logger.debug("ERROR_NOT_ENABLED (stepper_id:%d)", stepper_id)

            case CANIDS.CANID_IO_STEPPER_ERROR_MOTION_IN_PROGRESS:
                (stepper_id,) = struct.unpack(">B", msg.data)
                if not self._is_valid_id(stepper_id):
                    return

                # if we receive ERROR_MOTION_IN_PROGRESS it means the state of this class is not synced anymore with ioboard
                raise StepperDesyncError(
                    f"Reiceived ERROR_MOTION_IN_PROGRESS (state:{self.steppers[stepper_id].get_state()})"
                )

            case CANIDS.CANID_IO_STEPPER_ERROR_INVALID_PARAMS:
                (stepper_id,) = struct.unpack(">B", msg.data)
                if not self._is_valid_id(stepper_id):
                    return

                # we should never receive this because all parameters are already validated, maybe raise StepperDesyncError .
                self.steppers[stepper_id].set_state(StepperState.FLAG_ERROR_INVALID_PARAM)
                self.logger.error("ERROR_INVALID_PARAM (stepper_id:%d)", stepper_id)

            case CANIDS.CANID_IO_STEPPER_HOME_STARTING:
                (stepper_id,) = struct.unpack(">B", msg.data)
                if not self._is_valid_id(stepper_id):
                    return

                if self.steppers[stepper_id].get_state() is not StepperState.HAS_REQUESTED_HOME:
                    raise StepperDesyncError(
                        f"Reiceived STEPPER_HOME_STARTING but state was not HAS_REQUESTED_HOME (state:{self.steppers[stepper_id].get_state()})"
                    )

                self.steppers[stepper_id].set_state(StepperState.IS_DOING_HOME)
                self.logger.debug("HOME_STARTING (stepper_id:%d)", stepper_id)

            case CANIDS.CANID_IO_STEPPER_HOME_FAILED:
                (stepper_id,) = struct.unpack(">B", msg.data)
                if not self._is_valid_id(stepper_id):
                    return

                if self.steppers[stepper_id].get_state() not in [
                    StepperState.HAS_REQUESTED_HOME,
                    StepperState.IS_DOING_HOME,
                ]:
                    raise StepperDesyncError(
                        f"Reiceived HOME_FAILED but state was not doing home (state:{self.steppers[stepper_id].get_state()})"
                    )

                self.steppers[stepper_id].set_state(StepperState.FLAG_HOME_FAILED)
                self.logger.debug("HOME_FAILED (stepper_id:%d)", stepper_id)

            case CANIDS.CANID_IO_STEPPER_HOME_SUCCEEDED:
                (stepper_id,) = struct.unpack(">B", msg.data)
                if not self._is_valid_id(stepper_id):
                    return

                if self.steppers[stepper_id].get_state() not in [
                    StepperState.HAS_REQUESTED_HOME,
                    StepperState.IS_DOING_HOME,
                ]:
                    raise StepperDesyncError(
                        f"Reiceived HOME_SUCCEEDED but state was not doing home (state:{self.steppers[stepper_id].get_state()})"
                    )

                self.steppers[stepper_id].set_state(StepperState.FLAG_HOME_SUCCEEDED)
                self.logger.debug("HOME_SUCCEEDED (stepper_id:%d)", stepper_id)

            case CANIDS.CANID_IO_STEPPER_GOTO_STARTING:
                (stepper_id,) = struct.unpack(">B", msg.data)
                if not self._is_valid_id(stepper_id):
                    return

                if self.steppers[stepper_id].get_state() is not StepperState.HAS_REQUESTED_GOTO:
                    raise StepperDesyncError(
                        f"Reiceived GOTO_STARTING but state was not HAS_REQUESTED_GOTO (state:{self.steppers[stepper_id].get_state()})"
                    )

                self.steppers[stepper_id].set_state(StepperState.IS_DOING_GOTO)
                self.logger.debug("GOTO_STARTING (stepper_id:%d)", stepper_id)

            case CANIDS.CANID_IO_STEPPER_GOTO_FINISHED:
                (stepper_id,) = struct.unpack(">B", msg.data)
                if not self._is_valid_id(stepper_id):
                    return

                if self.steppers[stepper_id].get_state() not in [
                    StepperState.HAS_REQUESTED_GOTO,
                    StepperState.IS_DOING_GOTO,
                ]:
                    raise StepperDesyncError(
                        f"Reiceived GOTO_FINISHED but state was not doing goto (state:{self.steppers[stepper_id].get_state()})"
                    )

                self.steppers[stepper_id].set_state(StepperState.FLAG_GOTO_FINISHED)
                self.logger.debug("GOTO_FINISHED (stepper_id:%d)", stepper_id)

            case CANIDS.CANID_IO_STATUS:
                enable, tors = struct.unpack(">BH", msg.data)
                self.logger.log(-10, "STATUS (enable:%s tors:%s)", enable, format(tors, "016b"))

            case CANIDS.CANID_IO_ALIVE:
                (first_alive_since_reboot,) = struct.unpack(">?", msg.data)
                self.logger.log(-10, "ALIVE (first_alive_since_reboot:%s)", first_alive_since_reboot)

    def reboot(self) -> bool:
        msg = can.Message(arbitration_id=CANIDS.CANID_IO_REBOOT, is_extended_id=False)
        return can_utils.send(self.bus, msg)

    def enable(self, state: bool) -> bool:
        msg = can.Message(
            arbitration_id=CANIDS.CANID_IO_STEPPER_ENABLE,
            data=[state],
            is_extended_id=False,
        )
        return can_utils.send(self.bus, msg)

    def home(
        self,
        stepper_id: int,
        max_relative_steps_before_error: int,
        tor_id: int,
        tor_state_to_end_homing: bool,
    ) -> StepperMotionResult:
        if stepper_id not in self.steppers:
            self.logger.error("home(): stepper_id:%d not mapped in self.steppers", stepper_id)
            return StepperMotionResult.ERROR_INVALID_PARAM

        stepper = self.steppers[stepper_id]

        self._clear_flags(stepper_id)

        if stepper.get_state() is not StepperState.IDLE:
            self.logger.error("home(): A motion is already in progress! state:%s", stepper.get_state())
            return StepperMotionResult.ERROR_MOTION_IN_PROGRESS

        try:
            data = bytearray(
                stepper_id.to_bytes(1)
                + max_relative_steps_before_error.to_bytes(2, signed=True)
                + tor_id.to_bytes(1)
                + tor_state_to_end_homing.to_bytes(1)
            )
        except OverflowError:
            self.logger.error("home(): invalid parameter causing overflow (stepper_id:%d)", stepper_id)
            return StepperMotionResult.ERROR_INVALID_PARAM

        msg = can.Message(arbitration_id=CANIDS.CANID_IO_STEPPER_HOME, data=data, is_extended_id=False)
        if not can_utils.send(self.bus, msg):
            return StepperMotionResult.ERROR_CAN_SEND

        stepper.set_state(StepperState.HAS_REQUESTED_HOME)

        new_state, timed_out = stepper.wait_for_state_change(StepperState.HAS_REQUESTED_HOME, timeout=0.5)
        if timed_out:
            stepper.set_state(StepperState.IDLE)
            self.logger.error("home(): timeout HAS_REQUESTED_HOME (stepper_id:%d)", stepper_id)
            return StepperMotionResult.ERROR_REQUEST_TIMEOUT

        match new_state:
            case StepperState.IS_DOING_HOME:
                return StepperMotionResult.IS_DOING_HOME

            case StepperState.FLAG_HOME_SUCCEEDED:
                stepper.set_state(StepperState.IDLE)
                return StepperMotionResult.HOME_SUCCEEDED
            case StepperState.FLAG_HOME_FAILED:
                stepper.set_state(StepperState.IDLE)
                return StepperMotionResult.HOME_FAILED

            case StepperState.FLAG_ERROR_DISABLED_DURING_MOTION:
                stepper.set_state(StepperState.IDLE)
                return StepperMotionResult.ERROR_DISABLED_DURING_MOTION
            case StepperState.FLAG_ERROR_NOT_ENABLED:
                stepper.set_state(StepperState.IDLE)
                return StepperMotionResult.ERROR_NOT_ENABLED
            case StepperState.FLAG_ERROR_MOTION_IN_PROGRESS:
                stepper.set_state(StepperState.IDLE)
                return StepperMotionResult.ERROR_MOTION_IN_PROGRESS
            case StepperState.FLAG_ERROR_INVALID_PARAM:
                stepper.set_state(StepperState.IDLE)
                return StepperMotionResult.ERROR_INVALID_PARAM

            case _:
                raise RuntimeError(
                    f"bruh this should not be possible (state:{stepper.get_state()} new_state:{new_state})"
                )

    def goto_abs(
        self,
        stepper_id: int,
        absolute_steps: int,
        acceleleration: int,
        max_velocity: int,
    ) -> StepperMotionResult:
        if stepper_id not in self.steppers:
            self.logger.error("goto_abs(): stepper_id:%d not mapped in self.steppers", stepper_id)
            return StepperMotionResult.ERROR_INVALID_PARAM

        stepper = self.steppers[stepper_id]

        self._clear_flags(stepper_id)

        if stepper.get_state() is not StepperState.IDLE:
            self.logger.error(
                "goto_abs(): A motion is already in progress! (stepper_id:%d state:%s)", stepper_id, stepper.get_state()
            )
            return StepperMotionResult.ERROR_MOTION_IN_PROGRESS

        try:
            data = bytearray(
                stepper_id.to_bytes(1)
                + absolute_steps.to_bytes(2, signed=True)
                + acceleleration.to_bytes(3)
                + max_velocity.to_bytes(2)
            )
        except OverflowError:
            self.logger.error("goto_abs(): invalid parameter causing overflow (stepper_id:%d)", stepper_id)
            return StepperMotionResult.ERROR_INVALID_PARAM

        msg = can.Message(arbitration_id=CANIDS.CANID_IO_STEPPER_GOTO, data=data, is_extended_id=False)
        if not can_utils.send(self.bus, msg):
            return StepperMotionResult.ERROR_CAN_SEND

        stepper.set_state(StepperState.HAS_REQUESTED_GOTO)

        new_state, timed_out = stepper.wait_for_state_change(StepperState.HAS_REQUESTED_GOTO, timeout=0.5)
        if timed_out:
            stepper.set_state(StepperState.IDLE)
            self.logger.error("goto_abs(): timeout HAS_REQUESTED_GOTO (stepper_id:%d)", stepper_id)
            return StepperMotionResult.ERROR_REQUEST_TIMEOUT

        match new_state:
            case StepperState.IS_DOING_GOTO:
                return StepperMotionResult.IS_DOING_GOTO

            case StepperState.FLAG_GOTO_FINISHED:
                stepper.set_state(StepperState.IDLE)
                return StepperMotionResult.GOTO_FINISHED

            case StepperState.FLAG_ERROR_DISABLED_DURING_MOTION:
                stepper.set_state(StepperState.IDLE)
                return StepperMotionResult.ERROR_DISABLED_DURING_MOTION
            case StepperState.FLAG_ERROR_NOT_ENABLED:
                stepper.set_state(StepperState.IDLE)
                return StepperMotionResult.ERROR_NOT_ENABLED
            case StepperState.FLAG_ERROR_MOTION_IN_PROGRESS:
                stepper.set_state(StepperState.IDLE)
                return StepperMotionResult.ERROR_MOTION_IN_PROGRESS
            case StepperState.FLAG_ERROR_INVALID_PARAM:
                stepper.set_state(StepperState.IDLE)
                return StepperMotionResult.ERROR_INVALID_PARAM

            case _:
                raise RuntimeError(
                    f"bruh this should not be possible (state:{stepper.get_state()} new_state:{new_state})"
                )

    def wait_motion_finished(self, stepper_id: int, timeout: float = 10) -> StepperMotionResult:
        if stepper_id not in self.steppers:
            self.logger.error("wait_motion_finished(): stepper_id:%d not mapped in self.steppers", stepper_id)
            return StepperMotionResult.ERROR_INVALID_PARAM

        stepper = self.steppers[stepper_id]
        state = stepper.get_state()

        if state == StepperState.FLAG_ERROR_DISABLED_DURING_MOTION:
            stepper.set_state(StepperState.IDLE)
            self.logger.error("wait_motion_finished(): DISABLED_DURING_MOTION (stepper_id:%d)", stepper_id)
            return StepperMotionResult.ERROR_DISABLED_DURING_MOTION

        if state not in [StepperState.IS_DOING_GOTO, StepperState.IS_DOING_HOME]:
            self.logger.error(
                "wait_motion_finished(): is not doing HOME or GOTO (stepper_id:%d, state:%s)", stepper_id, state
            )
            return StepperMotionResult.ERROR_WAS_NOT_DOING_MOTION

        new_state, timed_out = stepper.wait_for_state_change(state, timeout=timeout)
        if timed_out:
            stepper.set_state(StepperState.ERROR_MOTION_TIMEOUT)
            self.logger.error("wait_motion_finished(): timeout (stepper_id:%d)", stepper_id)
            return StepperMotionResult.ERROR_MOTION_TIMEOUT

        result_states = {
            StepperState.FLAG_HOME_SUCCEEDED: StepperMotionResult.HOME_SUCCEEDED,
            StepperState.FLAG_HOME_FAILED: StepperMotionResult.HOME_FAILED,
            StepperState.FLAG_GOTO_FINISHED: StepperMotionResult.GOTO_FINISHED,
            StepperState.FLAG_ERROR_DISABLED_DURING_MOTION: StepperMotionResult.ERROR_DISABLED_DURING_MOTION,
        }

        result = result_states.get(new_state, None)
        if result is None:
            raise RuntimeError(f"bruh this should not be possible (state:{stepper.get_state()} new_state:{new_state})")

        stepper.set_state(StepperState.IDLE)
        return result

    # def wait_goto_finished(self, stepper_id: int, timeout: float = 5) -> StepperGotoResult:
    #     if stepper_id not in self.steppers:
    #         self.logger.error("wait_goto_finished(): stepper_id:%d not mapped in self.steppers", stepper_id)
    #         return False

    #     stepper = self.steppers[stepper_id]

    #     state = stepper.get_state()

    #     if state == StepperState.FLAG_GOTO_FINISHED:
    #         stepper.set_state(StepperState.IDLE)
    #         return StepperGotoResult.GOTO_FINISHED

    #     new_state, timed_out = stepper.wait_for_state_change(state, timeout=timeout)
    #     if timed_out:
    #         stepper.set_state(StepperState.ERROR_MOTION_TIMEOUT)
    #         self.logger.error("wait_goto_finished(): timeout (stepper_id:%d)", stepper_id)
    #         return StepperGotoResult.ERROR_MOTION_TIMEOUT

    #     result_states = {
    #         StepperState.FLAG_GOTO_FINISHED: StepperGotoResult.GOTO_FINISHED,
    #         StepperState.FLAG_ERROR_DISABLED_DURING_MOTION: StepperGotoResult.ERROR_DISABLED_DURING_MOTION,
    #         StepperState.FLAG_ERROR_NOT_ENABLED: StepperGotoResult.ERROR_NOT_ENABLED,
    #         StepperState.FLAG_ERROR_INVALID_PARAM: StepperGotoResult.ERROR_INVALID_PARAM,
    #     }

    #     result = result_states.get(new_state, None)
    #     stepper.set_state(StepperState.IDLE)
    #     return result

    # def wait_home_finished(self, stepper_id: int, timeout: float = 5) -> StepperHomeResult:
    #     if stepper_id not in self.steppers:
    #         self.logger.error("wait_home_finished(): stepper_id:%d not mapped in self.steppers", stepper_id)
    #         return False

    #     stepper = self.steppers[stepper_id]
    #     state = stepper.get_state()

    #     if state == StepperState.FLAG_HOME_SUCCEEDED:
    #         stepper.set_state(StepperState.IDLE)
    #         return StepperHomeResult.GOTO_FINISHED
    #     elif state == StepperState.FLAG_HOME_FAILED:
    #         stepper.set_state(StepperState.IDLE)
    #         return StepperHomeResult.GOTO_FINISHED

    #     new_state, timed_out = stepper.wait_for_state_change(state, timeout=timeout)
    #     if timed_out:
    #         stepper.set_state(StepperState.ERROR_MOTION_TIMEOUT)
    #         self.logger.error("wait_home_finished(): timeout (stepper_id:%d)", stepper_id)
    #         return StepperGotoResult.ERROR_MOTION_TIMEOUT

    #     result_states = {
    #         StepperState.FLAG_HOME_SUCCEEDED: StepperHomeResult.HOME_SUCCEEDED,
    #         StepperState.FLAG_HOME_FAILED: StepperHomeResult.HOME_FAILED,
    #         StepperState.FLAG_ERROR_DISABLED_DURING_MOTION: StepperHomeResult.ERROR_DISABLED_DURING_MOTION,
    #         StepperState.FLAG_ERROR_NOT_ENABLED: StepperHomeResult.ERROR_NOT_ENABLED,
    #         StepperState.FLAG_ERROR_INVALID_PARAM: StepperHomeResult.ERROR_INVALID_PARAM,
    #     }

    #     result = result_states.get(new_state, None)
    #     stepper.set_state(StepperState.IDLE)
    #     return result
