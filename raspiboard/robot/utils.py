import logging
import gc
import os
import colorama
from colorama import Fore
import time


class ElapsedColorFormatter(logging.Formatter):
    def __init__(self):
        super().__init__()
        self.start_time = time.perf_counter()

    def format(self, record):
        elapsed = time.perf_counter() - self.start_time
        sec = int(elapsed)
        msec = int((elapsed - sec) * 1000)

        colored_name = f"{Fore.CYAN}{record.name}{Fore.RESET}"

        match record.levelno:
            case level if level >= logging.ERROR:
                level_color = Fore.RED
            case level if level >= logging.WARNING:
                level_color = Fore.YELLOW
            case _:
                level_color = Fore.RESET
        colored_level = f"{level_color}{record.levelname}{Fore.RESET}"

        message = f"[{sec:02}.{msec:03}] {colored_name} {colored_level}: {record.getMessage()}"
        return message


def setup_logging() -> None:
    colorama.init(autoreset=True)

    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)

    log_handler = logging.StreamHandler()
    log_handler.setLevel(logging.DEBUG)
    log_handler.setFormatter(ElapsedColorFormatter())
    logger.handlers.clear()
    logger.addHandler(log_handler)

    logging.getLogger("can").setLevel(logging.CRITICAL)


def setup_realtime() -> None:
    # I didn't observed any spike difference when disabling or enabling the gc (when measuring main loop time execution). But still prefer to disable it just in case :P
    gc.disable()

    # use CPU3 (isolcpus=3 in the kernel boot parameters)
    os.sched_setaffinity(0, {3})

    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(10))
    except PermissionError:
        print("cap_sys_nice+ep not enabled on python3 bin !")


def clamp(val, min_, max_):
    if val < min_:
        return min_
    if val > max_:
        return max_
    return val
