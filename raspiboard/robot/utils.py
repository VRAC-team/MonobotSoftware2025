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
        minutes, seconds = divmod(elapsed, 60)
        milliseconds = (seconds % 1) * 1000
        formatted_time = f"{int(minutes):02}:{int(seconds):02}.{int(milliseconds):03}"

        colored_name = f"{Fore.YELLOW}{record.name}{Fore.RESET}"

        level_color = Fore.RED if record.levelno == logging.ERROR else Fore.RESET
        colored_level = f"{level_color}{record.levelname}{Fore.RESET}"

        message = f"[{formatted_time}] {colored_name} {colored_level}: {record.getMessage()}"
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
