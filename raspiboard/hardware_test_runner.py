from tests.test_ioboard import IOBoardIntegrationTests, IOBoardManualTests
from tests.test_motorboard import MotorBoardIntegrationTests
from tests.test_servoboard import ServoBoardUnitTests, ServoBoardIntegrationTests
from tests.test_pumpboard import PumpBoardIntegrationTests
from robot.utils import setup_logging
from robot_config_2025 import RobotConfig2025

from colorama import Fore


def ask_to_run(testname: str, show_warning=False) -> bool:
    print("")

    if show_warning:
        print(f"{Fore.RED}WARNING: this test will move actuators in unexcepted ways{Fore.RESET}")
        print(f"{Fore.RED}Do not run the test if the board is connected to the real robot actuators!{Fore.RESET}")

    answer = input(f"Do you want to run: {testname} ? [yes/N]: ")

    if answer.lower() == "yes":
        return True

    print("Skipping")
    return False


if __name__ == "__main__":
    setup_logging()

    config = RobotConfig2025()

    mb_integration_test = MotorBoardIntegrationTests(config)
    if ask_to_run("MotorBoard integration tests"):
        mb_integration_test.run()

    ib_integration_test = IOBoardIntegrationTests()
    if ask_to_run("IOBoard integration tests", show_warning=True):
        ib_integration_test.run()

    ib_manual_test = IOBoardManualTests()
    if ask_to_run("IOBoard manual tests", show_warning=True):
        ib_manual_test.run()

    sb_unit_test = ServoBoardUnitTests()
    if ask_to_run("ServoBoard unit tests"):
        sb_unit_test.run()

    sb_integration_test = ServoBoardIntegrationTests()
    if ask_to_run("ServoBoard integration tests", show_warning=True):
        sb_integration_test.run()

    pb_integration_test = PumpBoardIntegrationTests(config)
    if ask_to_run("PumpBoard integration tests"):
        pb_integration_test.run()
