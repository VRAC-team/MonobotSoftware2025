from tests.test_ioboard import IOBoardIntegrationTests, IOBoardManualTests
from tests.test_motorboard import MotorBoardIntegrationTests
from tests.test_servoboard import ServoBoardUnitTests, ServoBoardIntegrationTests
from robot.utils import setup_logging
from tests.simple_test import SimpleTest
from colorama import Fore


def ask_and_run(testname: str, test: SimpleTest, show_warning: bool = False):
    print("")

    if show_warning:
        print(f"{Fore.RED}WARNING: {testname} will move actuators in unexcepted ways{Fore.RESET}")
        print(f"{Fore.RED}Do not run the test if the board is connected to the robot actuators!{Fore.RESET}")

    answer = input(f"Do you want to run: {testname} ? [yes/N]: ")
    if answer.lower() == "yes":
        test.run()
    else:
        print("Skipping")


if __name__ == "__main__":
    setup_logging()

    ask_and_run("MotorBoard integration tests", MotorBoardIntegrationTests())

    ask_and_run("IOBoard integration tests", IOBoardIntegrationTests(), True)
    ask_and_run("IOBoard manual tests", IOBoardManualTests(), True)

    ask_and_run("ServoBoard unit tests", ServoBoardUnitTests())
    ask_and_run("ServoBoard integration tests", ServoBoardIntegrationTests(), True)
