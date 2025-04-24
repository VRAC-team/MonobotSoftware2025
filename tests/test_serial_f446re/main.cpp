#include <modm/board.hpp>

// Set the log level
#undef	MODM_LOG_LEVEL
#define	MODM_LOG_LEVEL modm::log::INFO

int main()
{
	Board::initialize();
	Board::LedD13::setOutput();

	MODM_LOG_INFO << "starting test_serial date:" << __DATE__ << " time:" __TIME__ << modm::endl;

	while (true)
	{
		MODM_LOG_INFO << "blink" << modm::endl;
		Board::LedD13::toggle();
		modm::delay(500ms);
	}

	return 0;
}
