#include <modm/board.hpp>
#include <modm/processing.hpp>
#include <modm/architecture/interface/interrupt.hpp>

// #include "AccelStepper/AccelStepper.h"

// Set the log level
#undef MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::INFO

using namespace modm::literals;
using namespace std::chrono_literals;

using step_en = GpioInverted<GpioA15>;

using step1 = GpioA12;
using dir1 = GpioA11;

using step5 = GpioB12;
using dir5 = GpioC4;

// AccelStepper stepper(step5, dir5);

int main()
{
	Board::initialize();

	step_en::setOutput(Gpio::OutputType::PushPull);
	step1::setOutput(Gpio::OutputType::PushPull, Gpio::OutputSpeed::Low);
	dir1::setOutput(Gpio::OutputType::PushPull, Gpio::OutputSpeed::Low);
	step5::setOutput(Gpio::OutputType::PushPull, Gpio::OutputSpeed::Low);
	dir5::setOutput(Gpio::OutputType::PushPull, Gpio::OutputSpeed::Low);

	step_en::reset();
	step1::reset();
	dir1::reset();
	step5::reset();
	dir5::reset();

	step_en::set();

	MODM_LOG_INFO << "test stepper date:" << __DATE__ << " time:" __TIME__ << modm::endl;

	modm::PeriodicTimer blinker{200ms};

	while (true)
	{
		if (blinker.execute())
		{
			Board::Led::toggle();
			MODM_LOG_INFO << "time:" << modm::PreciseClock::now() << modm::endl;
		}
	}

	return 0;
}
