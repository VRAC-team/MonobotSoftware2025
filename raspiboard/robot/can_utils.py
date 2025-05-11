import can
import logging
import inspect

logger = logging.getLogger(__name__)


def send(bus: can.Bus, msg: can.Message) -> bool:
    """
    Returns:
        bool: True if the message was accepted by the socket layer for transmission.
              False if a can.CanError was raised (e.g., bus off, send failure).
    Note:
        This function does not guarantee that the message was ACKed by any receiver on the bus!
        the can interface used is SocketCAN, a return value of True only confirms that the message was successfully
        sent to the CAN interface; it does not mean that another node received or acknowledged it.
        Be warned!
    """
    frame = inspect.currentframe().f_back
    caller_name = frame.f_code.co_name
    caller_instance = frame.f_locals.get("self")
    if caller_instance:
        caller_name = f"{caller_instance.__class__.__name__}.{caller_name}"

    try:
        bus.send(msg)
        return True
    except can.CanError:
        logger.error("could not sent CAN: caller:%s msg:%s", caller_name, msg)
        return False


def get_can_interface(preferred_channel=("can0", "vcan0"), bitrate: int = 1000000) -> can.BusABC | None:
    for chan in preferred_channel:
        try:
            bus = can.ThreadSafeBus(
                channel=chan,
                interface="socketcan",
                bitrate=bitrate,
                local_loopback=True,
            )
            print(f"found SocketCAN interface: {bus}")
            return bus
        except Exception:
            pass
