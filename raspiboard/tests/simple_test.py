import traceback
import inspect
import time
import colorama
from colorama import Fore, Style

colorama.init(autoreset=True)


class SimpleTest:
    def run(self):
        test_methods = sorted(
            name for name, method in inspect.getmembers(self, predicate=inspect.ismethod) if name.startswith("test_")
        )

        results = []
        print(f"Started testing {self.__class__.__name__} ({len(test_methods)} tests)...\n")

        if hasattr(self, "setUp"):
            self.setUp()

        total_start = time.perf_counter()

        for name in test_methods:
            method = getattr(self, name)
            test_desc = f"{self.__class__.__name__}.{name}"
            print(f"{Fore.CYAN}Starting {test_desc}:{Style.RESET_ALL}")

            if hasattr(self, "setUpTest"):
                self.setUpTest()

            start = time.perf_counter()

            try:
                method()
                duration = time.perf_counter() - start
                print(f"{Fore.GREEN}PASSED{Style.RESET_ALL} in {duration:.2f}s")
                results.append(("Success", name, duration, None))

            except AssertionError as e:
                duration = time.perf_counter() - start
                print(f"{Fore.RED}FAILED{Style.RESET_ALL} in {duration:.2f}s")
                traceback.print_exc()
                results.append(("Failure", name, duration, e))

            except Exception as e:
                duration = time.perf_counter() - start
                print(f"{Fore.MAGENTA}ERROR{Style.RESET_ALL} in {duration:.2f}s")
                traceback.print_exc()
                results.append(("Error", name, duration, e))

            if hasattr(self, "tearDownTest"):
                self.tearDownTest()

            print()  # newline

        total_duration = time.perf_counter() - total_start

        if hasattr(self, "tearDown"):
            self.tearDown()

        print("")
        print(f"Done testing {self.__class__.__name__} in {total_duration:.2f}s")
        print("=" * 40)
        print(f"Total tests:{len(results)}")
        counts = {"Success": 0, "Failure": 0, "Error": 0}
        for result in results:
            counts[result[0]] += 1

        print(f"{Fore.GREEN}PASSED{Style.RESET_ALL}:{counts['Success']}")
        print(f"{Fore.RED}FAILED{Style.RESET_ALL}:{counts['Failure']}")
        print(f"{Fore.MAGENTA}ERROR{Style.RESET_ALL}:{counts['Error']}")
        print("=" * 40)
        print("")

    def assertTrue(self, expr, msg=None):
        if not expr:
            raise AssertionError(msg or f"Expected True but got {expr!r}")

    def assertFalse(self, expr, msg=None):
        if expr:
            raise AssertionError(msg or f"Expected False but got {expr!r}")

    def assertEqual(self, a, b, msg=None):
        if a != b:
            raise AssertionError(msg or f"{a!r} != {b!r}")

    def assertNotEqual(self, a, b, msg=None):
        if a == b:
            raise AssertionError(msg or f"{a!r} == {b!r}")
