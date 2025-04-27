#include <modm/board.hpp>
#include <modm/processing.hpp>
#include <modm/debug/logger.hpp>
#include <modm/math/filter.hpp>
#include <modm/io/iodevice.hpp>
#include "can_identifiers.hpp"

using bb_tx = GpioB7;

//using Pumps = SoftwareGpioPort<GpioB10, GpioB1, GpioA6, GpioA5, GpioA2, GpioA0>;
using Pumps = SoftwareGpioPort<GpioA0, GpioA2, GpioA5, GpioA6, GpioB1, GpioB10>;

//using PumpsState = SoftwareGpioPort<GpioB11, GpioB0, GpioA7, GpioA4, GpioA3, GpioA1>;
using PumpsState = SoftwareGpioPort<GpioA1, GpioA3, GpioA4, GpioA7, GpioB0, GpioB11>;

//using Valves = SoftwareGpioPort<GpioA9, GpioA8, GpioB15, GpioB14, GpioB13, GpioB12>;
using Valves = SoftwareGpioPort<GpioB12, GpioB13, GpioB14, GpioB15, GpioA8, GpioA9>;

// Set the log level
#undef MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::INFO

class bitbang_uart {
public:
	static bool write(uint8_t data) {
		bb_tx::reset();
		modm::delay_us(104);
		for (uint8_t i = 0 ; i < 8 ; ++i) {
			if ((data >> i) & 1) {
				bb_tx::set();
			} else {
				bb_tx::reset();
			}
			modm::delay_us(104);
		}
		bb_tx::set();
		modm::delay_us(104);
		return true;
	}

	static bool read(uint8_t &data) {
		return true;
	}

	static void flushWriteBuffer() {
	}
};

modm::IODeviceWrapper<bitbang_uart, modm::IOBuffer::BlockIfFull> loggerDevice;
modm::log::Logger modm::log::info(loggerDevice);

void test_can()
{
	MODM_LOG_INFO << "Starting test_can" << modm::endl;

	while (true)
	{
		if (!Can::isMessageAvailable())
		{
			continue;
		}

		modm::can::Message message;
		uint8_t filter_id;
		Can::getMessage(message, &filter_id);
		MODM_LOG_INFO << message << modm::endl;


		MODM_LOG_INFO << "testing SETPUMP" << modm::endl;
		for (uint8_t i = 0; i < 6; ++i)
		{
			modm::can::Message setpump1(CANID_PUMP_SET, 2);
			setpump1.data[0] = i;
			setpump1.data[1] = 1;
			Can::sendMessage(setpump1);
			modm::delay(500ms);

			modm::can::Message setpump2(CANID_PUMP_SET, 2);
			setpump2.data[0] = i;
			setpump2.data[1] = 0;
			Can::sendMessage(setpump2);
			modm::delay(500ms);
		}
	}
}

void test_current_measure() {
	MODM_LOG_INFO << "Starting test_current_measure" << modm::endl;

	const uint8_t id = 0;
	
	modm::filter::MovingAverage<float, 100> avg;

	Pumps::write(1 << id);

	int i = 0;

	while (true)
	{
		uint8_t states = PumpsState::read();
		uint8_t state = (states >> id) & 1;
		avg.update(state);

		if (++i == 50) {
			i = 0;
			MODM_LOG_INFO << (int)(avg.getValue()*100) << modm::endl;
		}

		if (avg.getValue() >= 0.75f) {
			Board::LedGreen::set();
		} else {
			Board::LedGreen::reset();
		}

		// modm::delay(1ms);
	}
}

void test_gpios()
{
	MODM_LOG_INFO << "Starting test_gpios" << modm::endl;

	while (true)
	{
		for (uint8_t i = 0; i < 6; ++i)
		{
			MODM_LOG_INFO << "pump:" << i << modm::endl;

			Pumps::write(1 << i);

			modm::delay(2s);

			Pumps::write(0);
			Valves::write(1 << i);

			modm::delay(200ms);

			Valves::write(0);
		}

		MODM_LOG_INFO << "all pumps" << modm::endl;

		Pumps::write(0b111111);
		modm::delay(500ms);
		Pumps::write(0);

		MODM_LOG_INFO << "all valves" << modm::endl;

		Valves::write(0b111111);
		modm::delay(500ms);
		Valves::write(0);
	}
}

int main()
{
	Board::initialize();

	Pumps::setOutput();
	PumpsState::setInput();
	Valves::setOutput();

	// Initialize Usart
	// Usart1::connect<GpioOutputB6::Tx>();
	// Usart1::initialize<Board::SystemClock, 115200_Bd>();
	bb_tx::setOutput();
	bb_tx::set();

	MODM_LOG_INFO << "pumpboard date:" << __DATE__ << " time:" __TIME__ << modm::endl;

	MODM_LOG_INFO << "Initializing Can..." << modm::endl;
	Can::connect<GpioInputB8::Rx, GpioOutputB9::Tx>(Gpio::InputType::PullUp);
	Can::initialize<Board::SystemClock, 1_Mbps>(9);
	CanFilter::setFilter(0, CanFilter::FIFO0, CanFilter::ExtendedIdentifier(0x300), CanFilter::ExtendedFilterMask(0x700)); // filter 0x300 to 0x3FF

	// test_can();
	// test_gpios();
	// test_current_measure();

    modm::PeriodicTimer blinker{100ms};

    bool first_can_alive = true;
	modm::PeriodicTimer can_alive_timer{1s};

	modm::filter::MovingAverage<float, 200> current_avg[6];
	modm::PeriodicTimer current_measure_timer{1ms};
	modm::PeriodicTimer can_status_timer{100ms};

	int valve_autoclose_counters[6] = {-1, -1, -1, -1, -1, -1};
	modm::PeriodicTimer valve_autoclose_timer{100ms};

	while (true)
	{
		if (blinker.execute())
		{
			Board::LedGreen::toggle();
		}

		if (valve_autoclose_timer.execute()) {
			for (uint8_t i = 0 ; i < 6 ; ++i) {
				if (valve_autoclose_counters[i] >= 0) {
					valve_autoclose_counters[i]++;
				}

				if (valve_autoclose_counters[i] == 5) {
					valve_autoclose_counters[i] = -1;
					
					uint8_t valves_states = Valves::read();
					valves_states &= ~(1 << i); // clear bit
					Valves::write(valves_states);
				} 
			}
		}

		if (current_measure_timer.execute())
		{
			uint8_t states = PumpsState::read();

			for (uint8_t i = 0 ; i < 6 ; ++i) {
				uint8_t state = (states >> i) & 1;
				current_avg[i].update(state);
			}
		}

		if (can_alive_timer.execute()) {
			modm::can::Message alive(CANID_PUMP_ALIVE, 1);
            alive.data[0] = first_can_alive;
			Can::sendMessage(alive);

            first_can_alive = false;
		}

		if (can_status_timer.execute()) {
			uint8_t pump_current_states = 0;
			for (uint8_t i = 0 ; i < 6 ; ++i) {
				if (current_avg[i].getValue() > 0.75f) {
					pump_current_states |= 1 << i;
				}
			}

			modm::can::Message response(CANID_PUMP_STATUS, 3);
			response.data[0] = Pumps::read();
			response.data[1] = Valves::read();
			response.data[2] = pump_current_states;
			Can::sendMessage(response);
		}
		
		if (!Can::isMessageAvailable())
		{
			continue;
		}

		modm::can::Message message;
		uint8_t filter_id;
		Can::getMessage(message, &filter_id);

		// Check Alive
		// if (message.identifier == CANdata::VacuumBoard::CheckBoardAlive_id && message.length == 0)
		// {
		// 	MODM_LOG_INFO << "Can: Check Alive" << modm::endl;
		// 	modm::can::Message response(CANdata::VacuumBoard::CheckBoardAliveResponse_id, 0);
		// 	Can::sendMessage(response);
		// }
		// Set Pumps
		if (message.identifier == CANID_PUMP_SET && message.length == 2)
		{
			uint8_t pumpid = message.data[0];
			uint8_t pumpstate = message.data[1];

			if (pumpid > 6) {
				continue;
			}

			uint8_t pumps_states = Pumps::read();
			

			if (pumpstate == 1)
			{
				pumps_states |= 1 << pumpid; // set bit
			}
			else
			{
				pumps_states &= ~(1 << pumpid); // clear bit

				// automatically open valve on pump shutdown

				uint8_t valves_states = Valves::read();
				valves_states |= 1 << pumpid; // set bit
				Valves::write(valves_states);
				valve_autoclose_counters[pumpid] = 0;
			}

			Pumps::write(pumps_states);

		}
	}

	return 0;
}
