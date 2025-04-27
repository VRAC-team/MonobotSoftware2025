#include <modm/platform.hpp>
#include <modm/board.hpp>
#include <modm/processing.hpp>
#include "pca9685.hpp"
#include "can_identifiers.hpp"

#undef	MODM_LOG_LEVEL
#define	MODM_LOG_LEVEL modm::log::INFO

using power1_oe = GpioOutputA3;
using power2_oe = GpioOutputA0;
using power3_oe = GpioOutputB1;

using power1_adc = GpioA4;
using power1_adc_chan = GpioA4::In9;
using power2_adc = GpioA5;
using power2_adc_chan = GpioA5::In10;
using power3_adc = GpioA6;
using power3_adc_chan = GpioA6::In11;

using pca_oe = GpioInverted<GpioA1>;

bool g_first_can_alive = true;

struct SystemClock
{
	static constexpr uint32_t Frequency = 80_MHz;
	static constexpr uint32_t Ahb = Frequency;
	static constexpr uint32_t Apb1 = Frequency;
	static constexpr uint32_t Apb2 = Frequency;

    static constexpr uint32_t Spi1 = Apb2;
	static constexpr uint32_t Spi3 = Apb1;

	static constexpr uint32_t Usart1 = Apb2;
	static constexpr uint32_t Usart2 = Apb1;

    static constexpr uint32_t Can1   = Apb1;

    static constexpr uint32_t I2c1   = Apb1;
	static constexpr uint32_t I2c3   = Apb1;

	static constexpr uint32_t Apb1Timer = Apb1 * 1;
	static constexpr uint32_t Apb2Timer = Apb2 * 1;
	static constexpr uint32_t Timer1  = Apb2Timer;
    static constexpr uint32_t Timer15  = Apb2Timer;
    static constexpr uint32_t Timer16  = Apb2Timer;
	static constexpr uint32_t Timer2  = Apb1Timer;
	static constexpr uint32_t Timer6  = Apb1Timer;
	static constexpr uint32_t Timer7  = Apb1Timer;

	static constexpr uint32_t Iwdg = Rcc::LsiFrequency;
	static constexpr uint32_t Rtc = 32.768_kHz;

	static bool inline
	enable()
	{
		const Rcc::PllFactors pllFactors{
			.pllM = 1,	//   4MHz /  1 -> 4MHz
			.pllN = 40,	//   4MHz * 40 -> 160MHz <= 344MHz = PLL VCO output max, >= 64 MHz = PLL VCO out min
			.pllR = 2,	// 160MHz /  2 -> 80MHz = F_cpu
		};
		Rcc::enablePll(Rcc::PllSource::MultiSpeedInternalClock, pllFactors);
		Rcc::setFlashLatency<Frequency>();

		// switch system clock to PLL output
		Rcc::enableSystemClock(Rcc::SystemClockSource::Pll);
		Rcc::setAhbPrescaler(Rcc::AhbPrescaler::Div1);
		// APB1 has max. 80MHz
		Rcc::setApb1Prescaler(Rcc::Apb1Prescaler::Div1);
		Rcc::setApb2Prescaler(Rcc::Apb2Prescaler::Div1);
		// update frequencies for busy-wait delay functions
		Rcc::updateCoreFrequency<Frequency>();

		return true;
	}
};

template<class I2cMaster>
class ServoboardArduino : public modm::I2cDevice<I2cMaster, 1, modm::I2cWriteTransaction> {
	public:
		ServoboardArduino(uint8_t address = 0x8);

		bool set_led_pattern(uint8_t id, uint8_t pattern);
};

template<typename I2cMaster>
ServoboardArduino<I2cMaster>::ServoboardArduino(uint8_t address) :
	modm::I2cDevice<I2cMaster, 1, modm::I2cWriteTransaction>(address)
{
}

template<typename I2cMaster>
bool
ServoboardArduino<I2cMaster>::set_led_pattern(uint8_t id, uint8_t pattern)
{
	if (id > 3) {
		return false;
	}
	uint8_t buffer[2];
	buffer[0] = id;
	buffer[1] = pattern;
	this->transaction.configureWrite(buffer, 2);
	return this->runTransaction();
}

int main()
{
	SystemClock::enable();
	SysTickTimer::initialize<SystemClock>();

	Board::stlink::Uart::connect<Board::stlink::Tx::Tx, Board::stlink::Rx::Rx>(Gpio::InputType::PullUp);
	Board::stlink::Uart::initialize<SystemClock, 115200_Bd>();

    Board::LedD13::setOutput();

	pca_oe::setOutput();
	pca_oe::reset();

    power1_oe::setOutput();
    power2_oe::setOutput();
    power3_oe::setOutput();
    power1_oe::reset();
    power2_oe::reset();
    power3_oe::reset();

	Adc1::initialize(Adc1::ClockMode::SynchronousPrescaler4, Adc1::ClockSource::SystemClock, Adc1::Prescaler::Disabled, Adc1::CalibrationMode::SingleEndedInputsMode, true);

    I2cMaster3::connect<GpioB4::Sda, GpioA7::Scl>();
    I2cMaster3::initialize<SystemClock, 400_kHz>();
	ServoboardArduino<I2cMaster3> servoboard_arduino;

	const uint8_t servos_mapping[16] = {11, 10, 9, 8, 7, 6, 5, 4, 12, 13, 14, 15, 3, 2, 1, 0};
    I2cMaster1::connect<GpioB7::Sda, GpioB6::Scl>();
    I2cMaster1::initialize<SystemClock, 400_kHz>();
    PCA9685 pca9685 = PCA9685<I2cMaster1>(0x40);
    pca9685.initialize();
	pca9685.set_pwm_freq(50);
	pca_oe::set();

	Can1::connect<GpioA11::Rx, GpioA12::Tx>(Gpio::InputType::PullUp);
	Can1::initialize<SystemClock, 1_Mbps>(9);
    CanFilter::setFilter(0, CanFilter::FIFO0, CanFilter::ExtendedIdentifier(0x100), CanFilter::ExtendedFilterMask(0x700)); // filter 0x100 to 0x1FF

	MODM_LOG_INFO << "starting servoboard_nucleo date:" << __DATE__ << " time:" __TIME__ << modm::endl;

    modm::PeriodicTimer blinker{50ms};
	modm::PeriodicTimer can_alive_timer{1s};
	modm::PeriodicTimer power_current_measuer_timer{50ms};

    while (true) {
		if (blinker.execute()) {
			Board::LedD13::toggle();
		}

		if (can_alive_timer.execute()) {

			modm::can::Message alive(CANID_SERVO_ALIVE, 1);
            alive.data[0] = g_first_can_alive;
			Can1::sendMessage(alive);

            g_first_can_alive = false;
		}

		if (power_current_measuer_timer.execute()) {
			Adc1::connect<power1_adc_chan>();
			Adc1::setPinChannel<power1_adc>(Adc1::SampleTime::Cycles13);
			Adc1::startConversion();
			while(!Adc1::isConversionFinished()) {}
			uint16_t adc1 = Adc1::getValue();
		
			Adc1::connect<power2_adc_chan>();
			Adc1::setPinChannel<power2_adc>(Adc1::SampleTime::Cycles13);
			Adc1::startConversion();
			while(!Adc1::isConversionFinished()) {}
			uint16_t adc2 = Adc1::getValue();
		
			Adc1::connect<power3_adc_chan>();
			Adc1::setPinChannel<power3_adc>(Adc1::SampleTime::Cycles13);
			Adc1::startConversion();
			while(!Adc1::isConversionFinished()) {}
			uint16_t adc3 = Adc1::getValue();

			modm::can::Message msg(CANID_SERVO_STATUS, 7);
            msg.data[0] = adc1 << 8 & 0xFF;
			msg.data[1] = adc1 & 0xFF;
			msg.data[2] = adc2 << 8 & 0xFF;
			msg.data[3] = adc2 & 0xFF;
			msg.data[4] = adc3 << 8 & 0xFF;
			msg.data[5] = adc3 & 0xFF;
			msg.data[6] = power1_oe::read() | power2_oe::read() << 1 | power3_oe::read() << 2;
			Can1::sendMessage(msg);

			MODM_LOG_INFO.printf("adc1: %d %d %d\n", adc1, adc2, adc3);
		}

		if (!Can1::isMessageAvailable()) {
			continue;
		}

		modm::can::Message message;
		uint8_t filter_id;
		Can1::getMessage(message, &filter_id);

        if (message.identifier == CANID_SERVO_ENABLE_POWER && message.length == 3) {
            bool power1 = message.data[0];
            bool power2 = message.data[1];
            bool power3 = message.data[2];

            power1_oe::set(power1);
            power2_oe::set(power2);
            power3_oe::set(power3);
        }

		else if (message.identifier == CANID_SERVO_WRITE_US && message.length == 3)
		{
            uint8_t servo_id = message.data[0];
            int16_t servo_us = (message.data[1] << 8) | message.data[2];

			if (servo_id >= 16) {
				continue;
			}

            pca9685.write_us(servos_mapping[servo_id], servo_us);
        }

        else if (message.identifier == CANID_SERVO_SET_LED_PATTERN && message.length == 2) {
            uint8_t led_id = message.data[0];
            uint8_t led_pattern = message.data[1];

			servoboard_arduino.set_led_pattern(led_id, led_pattern);
        }

    }
}