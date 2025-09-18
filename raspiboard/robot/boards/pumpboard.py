import struct
import logging
import time
from enum import IntEnum

from robot.can_utils import can_send
from robot.can_identifiers import CANIDS

import can


class PumpValveState(IntEnum):
    DISABLE = 0
    ENABLE = 1
    NO_CHANGE = 3


class PumpBoard(can.Listener):
    DEFAULT_PAYLOAD_NO_CHANGE = [(PumpValveState.NO_CHANGE, PumpValveState.NO_CHANGE) for _ in range(6)]

    def __init__(self, bus: can.BusABC, auto_close_valve_after: float = 0.5):
        self.bus = bus
        self.auto_close_valve_after = auto_close_valve_after
        self.auto_close_valve: dict[float, tuple[int]] = {}
        self.logger = logging.getLogger(self.__class__.__name__)

    def on_message_received(self, msg):
        if not (0x300 <= msg.arbitration_id <= 0x3FF):
            return

        match msg.arbitration_id:
            case CANIDS.CANID_PUMP_STATUS:
                vacuum_states = struct.unpack(">B", msg.data)
                self.logger.log(-10, "STATUS (vacuum_states:%06b)", vacuum_states)

            case CANIDS.CANID_PUMP_ALIVE:
                (first_alive_since_reboot,) = struct.unpack(">?", msg.data)
                self.logger.log(-10, "ALIVE (first_alive_since_reboot:%s)", first_alive_since_reboot)

    def reboot(self) -> bool:
        self.logger.debug("reboot")
        msg = can.Message(arbitration_id=CANIDS.CANID_PUMP_REBOOT, is_extended_id=False)
        return can_send(self.bus, msg)

    def is_id_valid(self, id: int) -> bool:
        return id >= 0 and id <= 5

    def set_states(self, states: dict[int, tuple[PumpValveState, PumpValveState]]) -> bool:
        """
        Set multiple pump/valve states at once.

        Args:
            states: {pump_id: (pump_state, valve_state)}

        Returns:
            bool: True if message sent successfully
        """
        payload = PumpBoard.DEFAULT_PAYLOAD_NO_CHANGE.copy()

        for id, (pump_state, valve_state) in states.items():
            if not self.is_id_valid(id):
                self.logger.error("id:%d is not valid", id)
                return False

            payload[id] = (pump_state, valve_state)

        self.logger.debug("set_states states:%s", states)

        data = self._encode_payload(payload)

        msg = can.Message(arbitration_id=CANIDS.CANID_PUMP_SET, data=data, is_extended_id=False)
        return can_send(self.bus, msg)

    def enable_pumps(self, pumps_id: tuple[int, int]) -> bool:
        payload = PumpBoard.DEFAULT_PAYLOAD_NO_CHANGE.copy()

        for id in pumps_id:
            if not self.is_id_valid(id):
                self.logger.error("id:%d is not valid", id)
                return False
            payload[id] = (PumpValveState.ENABLE, PumpValveState.NO_CHANGE)

        self.logger.debug("enable_pumps pumps_id:%s", pumps_id)

        data = self._encode_payload(payload)

        msg = can.Message(arbitration_id=CANIDS.CANID_PUMP_SET, data=data, is_extended_id=False)
        return can_send(self.bus, msg)

    def disable_pump_with_auto_valve_release(self, pumps_id: tuple[int, int]) -> bool:
        payload = PumpBoard.DEFAULT_PAYLOAD_NO_CHANGE.copy()

        for id in pumps_id:
            if not self.is_id_valid(id):
                self.logger.error("id:%d is not valid", id)
                return False
            payload[id] = (PumpValveState.DISABLE, PumpValveState.ENABLE)

        self.logger.debug("disable_pump_with_auto_valve_release pumps_id:%s", pumps_id)

        data = self._encode_payload(payload)

        msg = can.Message(arbitration_id=CANIDS.CANID_PUMP_SET, data=data, is_extended_id=False)
        if can_send(self.bus, msg):
            time_to_close = time.monotonic() + self.auto_close_valve_after
            self.auto_close_valve[time_to_close] = pumps_id
            return True
        return False

    def process(self, t: float):
        if not self.auto_close_valve:
            return

        for close_at in list(self.auto_close_valve.keys()):
            valves_id = self.auto_close_valve[close_at]

            if t >= close_at:
                payload = PumpBoard.DEFAULT_PAYLOAD_NO_CHANGE.copy()
                for id in valves_id:
                    payload[id] = (PumpValveState.NO_CHANGE, PumpValveState.DISABLE)
                data = self._encode_payload(payload)

                msg = can.Message(arbitration_id=CANIDS.CANID_PUMP_SET, data=data, is_extended_id=False)
                if can_send(self.bus, msg):
                    self.logger.debug("auto release valves_id:%s", valves_id)

                del self.auto_close_valve[close_at]

    def wait_valve_auto_release(self):
        while self.auto_close_valve:
            yield

    def _encode_payload(self, payload: list[tuple[PumpValveState, PumpValveState]]) -> bytes:
        return bytes(((int(pump) & 0b11) | ((int(valve) & 0b11) << 2)) for pump, valve in payload)
