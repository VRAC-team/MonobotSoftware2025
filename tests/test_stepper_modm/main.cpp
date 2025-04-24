
#include <modm/board.hpp>
#include <modm/processing.hpp>
#include <modm/architecture/interface/interrupt.hpp>

// Set the log level
#undef MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::INFO

using namespace modm::literals;
using namespace std::chrono_literals;

#define CANID_CHECKALIVE 0x400
#define CANID_CHECKALIVE_RESPONSE 0x401 // RESPONSE
#define CANID_STEPPER_ENABLE 0x402
#define CANID_STEPPER_MOVE 0x403
#define CANID_STEPPER_HOME 0x404
#define CANID_STEPPER_EVENT 0x405 // RESPONSE
#define CANID_STATUS_TOR 0x40A

using step_en = GpioInverted<GpioOutputA15>;

using step1 = GpioOutputA12;
using dir1 = GpioOutputA11;

using step5 = GpioOutputB12;
using dir5 = GpioOutputC4;

const uint32_t acceleration_step = 50;
const uint32_t maxspeed = 120;

volatile uint32_t counter = 0;
volatile uint32_t current_period = 20000;

// https://electronics.stackexchange.com/questions/353281/stm32-c-stepper-motor-start-after-1-minute
// https://github.com/luni64/TeensyStep/blob/795d58bcf90401e4d3360265fd7d34ca8beb16ae/src/StepControlBase.h#L126
// https://www.ti.com/lit/an/slyt482/slyt482.pdf?ts=1743254058082
// https://stackoverflow.com/questions/68561222/stepper-motor-math-with-constant-acceleration	

void timer_setup() {
	Timer2::enable();
	Timer2::setMode(Timer2::Mode::UpCounter, Timer2::SlaveMode::Disabled, Timer2::SlaveModeTrigger::Internal0, Timer2::MasterMode::Reset, true);
	Timer2::setPeriod<Board::SystemClock>(std::chrono::microseconds(current_period));
	
	Timer2::enableInterrupt(Timer2::Interrupt::Update);
	Timer2::enableInterruptVector(true, 1);

	Timer2::applyAndReset();
	Timer2::start();
}

MODM_ISR(TIM2)
{
	Timer2::acknowledgeInterruptFlags(Timer2::InterruptFlag::Update);
	// Timer2::start();

	if (current_period <= 200) {
		// dir5::set();
		counter++;

		if (counter == 1000) {
			Timer2::pause();
			MODM_LOG_INFO << "Done" << modm::endl;
		} else {
			Timer2::setPeriod<Board::SystemClock>(std::chrono::microseconds(current_period));
			Timer2::applyAndReset();
			Timer2::start();
		}
	} else {
		// dir5::reset();

		current_period -= acceleration_step;
		Timer2::setPeriod<Board::SystemClock>(std::chrono::microseconds(current_period));
		Timer2::applyAndReset();
		Timer2::start();
	}

	//minimum high hold time is 100ns
	step5::set();
	modm::delay_ns(500);
	step5::reset();
	
	// step5::toggle();
	// dir5::toggle();
}

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

	// stepper Vref calculation:
	// Vref =  Irms / 0.707106

	step_en::set();

	// while (true) {
	// 	modm::delay_us(500);
	// 	dir5::toggle();
	// 	modm::delay_us(500);
	// 	step5::toggle();
	// 	modm::delay_us(500);
	// 	step_en::toggle();
	// }

	MODM_LOG_INFO << "io_board date:" << __DATE__ << " time:" __TIME__ << modm::endl;

	// MODM_LOG_INFO << "Initializing Can..." << modm::endl;
	// Can1::connect<GpioInputB8::Rx, GpioOutputB9::Tx>(Gpio::InputType::PullUp);
	// Can1::initialize<Board::SystemClock, 500_kbps>(9);

	// MODM_LOG_INFO << "Setting up Filter for Can..." << modm::endl;
	// CanFilter::setFilter(0, CanFilter::FIFO0, CanFilter::ExtendedIdentifier(0),
	// 					 CanFilter::ExtendedFilterMask(0));
	// CanFilter::setFilter(1, CanFilter::FIFO0, CanFilter::StandardIdentifier(0),
	// 					 CanFilter::StandardFilterMask(0));

	
	MODM_LOG_INFO << "Initializing Timer..." << modm::endl;
	timer_setup();

	modm::PeriodicTimer blinker{200ms};

	while (true)
	{
		if (blinker.execute())
		{
			Board::Led::toggle();
		}

		// if (!Can1::isMessageAvailable())
		// {
		// 	continue;
		// }

		// modm::can::Message message;
		// uint8_t filter_id;
		// Can1::getMessage(message, &filter_id);

		// // Check Alive
		// if (message.identifier == CANID_CHECKALIVE && message.length == 0)
		// {
		// 	MODM_LOG_INFO << "Can: Check Alive" << modm::endl;
		// 	modm::can::Message response(CANID_CHECKALIVE_RESPONSE, 0);
		// 	Can1::sendMessage(response);
		// }
	}

	return 0;
}
