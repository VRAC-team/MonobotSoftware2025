#include <modm/platform.hpp>
#include <modm/processing.hpp>
#include <modm/board.hpp>
#include <modm/architecture/interface/clock.hpp>
#include <modm/debug/logger.hpp>
#include <modm/driver/encoder/as5047.hpp>

using namespace modm::literals;

// Set the log level
#undef	MODM_LOG_LEVEL
#define	MODM_LOG_LEVEL modm::log::DISABLED

#define CANID_MOTOR_ALIVE 0x100 //sent periodically by motorboard
#define CANID_MOTOR_SETPOINT 0x101
#define CANID_MOTOR_SETPOINT_RESPONSE 0x102 //sent by motorboard in response to setpoint
#define CANID_MOTOR_SETPOINT_ERROR 0x103 //sent by motorboard if no setpoint in the last x ms and was not in error
#define CANID_MOTOR_RESET_SETPOINT_ERROR 0x104


#define CANID_RASPI_ALIVE 0x200 //sent periodically by raspiboard

using M1_pwm = GpioA8; //timer1 chan1
using M2_pwm = GpioA9; //timer1 chan2
using M1_en = GpioOutputA10;
using M2_en = GpioOutputA11;
using M1_in1 = GpioOutputC6;
using M1_in2 = GpioOutputC7;
using M2_in1 = GpioOutputC8;
using M2_in2 = GpioOutputC9;

using enc1_cs = GpioOutputB10;
using enc2_cs = GpioOutputB12;
using enc_sck = GpioOutputB13;
using enc_miso = GpioOutputB14;
using enc_mosi = GpioOutputB15;
using enc_SPI = SpiMaster2;

modm::as5047::Data enc1_data{0};
modm::As5047<enc_SPI, enc1_cs> enc1{enc1_data};
modm::as5047::Data enc2_data{0};
modm::As5047<enc_SPI, enc2_cs> enc2{enc2_data};

struct SystemClock
{
	static constexpr uint32_t Frequency = 72_MHz;
	static constexpr uint32_t Ahb = Frequency;
	static constexpr uint32_t Apb1 = Frequency / 2;
	static constexpr uint32_t Apb2 = Frequency / 1;

	static constexpr uint32_t Adc    = Apb2;

	static constexpr uint32_t Spi1   = Apb2;
	static constexpr uint32_t Spi2   = Apb1;
	static constexpr uint32_t Spi3   = Apb1;
	static constexpr uint32_t Spi4   = Apb2;
	static constexpr uint32_t Spi5   = Apb2;

	static constexpr uint32_t Usart1 = Apb2;
	static constexpr uint32_t Usart2 = Apb1;
	static constexpr uint32_t Usart3 = Apb1;
	static constexpr uint32_t Uart4  = Apb1;
	static constexpr uint32_t Uart5  = Apb1;
	static constexpr uint32_t Usart6 = Apb2;

	static constexpr uint32_t Can1   = Apb1;
	static constexpr uint32_t Can2   = Apb1;

	static constexpr uint32_t I2c1   = Apb1;
	static constexpr uint32_t I2c2   = Apb1;
	static constexpr uint32_t I2c3   = Apb1;

	static constexpr uint32_t Apb1Timer = Apb1 * 1;
	static constexpr uint32_t Apb2Timer = Apb2 * 1;
	static constexpr uint32_t Timer1  = Apb2Timer;
	static constexpr uint32_t Timer2  = Apb1Timer;
	static constexpr uint32_t Timer3  = Apb1Timer;
	static constexpr uint32_t Timer4  = Apb1Timer;
	static constexpr uint32_t Timer5  = Apb1Timer;
	static constexpr uint32_t Timer9  = Apb2Timer;
	static constexpr uint32_t Timer10 = Apb2Timer;
	static constexpr uint32_t Timer11 = Apb2Timer;
	static constexpr uint32_t Iwdg = Rcc::LsiFrequency;
	static constexpr uint32_t Rtc = 32.768_kHz;

	static bool inline
	enable()
	{
		Rcc::enableExternalCrystal();
		const Rcc::PllFactors pllFactors{
			.pllM = 4,
			.pllN = 72,
			.pllP = 2,
		};
		Rcc::enablePll(Rcc::PllSource::Hse, pllFactors);
		// Rcc::enableOverdriveMode();
		Rcc::setFlashLatency<Frequency>();
		Rcc::enableSystemClock(Rcc::SystemClockSource::Pll);
		Rcc::setAhbPrescaler(Rcc::AhbPrescaler::Div1);
		Rcc::setApb1Prescaler(Rcc::Apb1Prescaler::Div2);
		Rcc::setApb2Prescaler(Rcc::Apb2Prescaler::Div1);
		Rcc::updateCoreFrequency<Frequency>();

		return true;
	}
};

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

void test_timer_sweep() {
	uint16_t timer_overflow = Timer1::getOverflow();

	while (true) {
		for (int32_t i = -255 ; i < 255 ; ++i) {
			uint16_t timer1_ch1_cmp = map(i, -255, 255, 0, timer_overflow);
			Timer1::setCompareValue<M1_pwm::Ch1>(timer1_ch1_cmp);
			modm::delay_ms(3);
		}
		Timer1::setCompareValue<M1_pwm::Ch1>(0);

		modm::delay(1s);

		for (int32_t i = -32767 ; i < 32767 ; ++i) {
			uint16_t timer1_ch2_cmp = map(i, -32767, 32767, 0, timer_overflow);
			Timer1::setCompareValue<M2_pwm::Ch2>(timer1_ch2_cmp);
			modm::delay_us(23);
		}
		Timer1::setCompareValue<M2_pwm::Ch2>(0);

		modm::delay(1s);
	}
}

int main()
{
	SystemClock::enable();
	SysTickTimer::initialize<SystemClock>();

	Board::stlink::Uart::connect<Board::stlink::Tx::Tx, Board::stlink::Rx::Rx>();
	Board::stlink::Uart::initialize<SystemClock, 115200_Bd>();

	Board::LedD13::setOutput();

	M1_pwm::setOutput(Gpio::OutputType::PushPull, Gpio::OutputSpeed::VeryHigh);
	M2_pwm::setOutput(Gpio::OutputType::PushPull, Gpio::OutputSpeed::VeryHigh);
	M1_en::setOutput();
	M2_en::setOutput();
	M1_in1::setOutput();
	M1_in2::setOutput();
	M2_in1::setOutput();
	M2_in2::setOutput();
	M1_en::reset();
	M2_en::reset();

	enc1_cs::setOutput(modm::Gpio::High);
	enc2_cs::setOutput(modm::Gpio::High);

	MODM_LOG_INFO << "starting motorboard_test_controller date:" << __DATE__ << " time:" __TIME__ << modm::endl;

	Timer1::connect<M1_pwm::Ch1>();
	Timer1::connect<M2_pwm::Ch2>();
	Timer1::enable();
	Timer1::setMode(Timer1::Mode::UpCounter);
	Timer1::setPeriod<SystemClock>(50us); //20 khz
	Timer1::configureOutputChannel<M1_pwm::Ch1>(Timer1::OutputCompareMode::Pwm, 0);
	Timer1::configureOutputChannel<M2_pwm::Ch2>(Timer1::OutputCompareMode::Pwm, 0);
	Timer1::applyAndReset();
	Timer1::start();
	Timer1::enableOutput();

	// test_timer_sweep();

	enc_SPI::connect<enc_miso::Miso, enc_mosi::Mosi, enc_sck::Sck>();
	enc_SPI::initialize<SystemClock, 1.125_MHz>();

	modm::PeriodicTimer blinker{50ms};
	modm::PeriodicTimer can_alive_timer{1s};

	MODM_LOG_INFO << "Initializing Can..." << modm::endl;
	Can1::connect<GpioInputB8::Rx, GpioOutputB9::Tx>(Gpio::InputType::PullUp);
	Can1::initialize<SystemClock, 500_kbps>(9);

	MODM_LOG_INFO << "Setting up Filter for Can..." << modm::endl;
	// Receive every message
	CanFilter::setFilter(0, CanFilter::FIFO0, CanFilter::ExtendedIdentifier(0),
						 CanFilter::ExtendedFilterMask(0));

	CanFilter::setFilter(1, CanFilter::FIFO0, CanFilter::StandardIdentifier(0),
						 CanFilter::StandardFilterMask(0));

	uint16_t timer_overflow = Timer1::getOverflow();

	bool setpoint_error = true;
	modm::Timeout setpoint_timeout{10ms};

	while (true)
	{
		if (blinker.execute())
		{
			Board::LedD13::toggle();
		}
		
		if (setpoint_timeout.execute()) {
			MODM_LOG_INFO << "SERPOINT ERROR" << modm::endl;
			setpoint_error = true;
			M1_in1::reset();
			M1_in2::reset();
			M2_in1::reset();
			M2_in2::reset();
			Timer1::setCompareValue<M1_pwm::Ch1>(0);
			Timer1::setCompareValue<M2_pwm::Ch2>(0);
			M1_en::reset();
			M2_en::reset();
			blinker.restart(50ms);

			modm::can::Message msg(CANID_MOTOR_RESET_SETPOINT_ERROR);
			Can1::sendMessage(msg);
		}

		if (can_alive_timer.execute()) {

			modm::can::Message response(CANID_MOTOR_ALIVE, 1);
			response.data[0] = setpoint_error;
			Can1::sendMessage(response);
		}
		
		if (!Can1::isMessageAvailable())
		{
			continue;
		}

		modm::can::Message message;
		uint8_t filter_id;
		Can1::getMessage(message, &filter_id);

		if (message.identifier == CANID_RASPI_ALIVE) {
			MODM_LOG_INFO << "[CAN] raspi alive" << modm::endl;
		}

		else if (message.identifier == CANID_MOTOR_RESET_SETPOINT_ERROR && message.length == 0)
		{
			MODM_LOG_INFO << "SERPOINT ERROR RESET" << modm::endl;
			setpoint_error = false;
			M1_en::set();
			M2_en::set();
			setpoint_timeout.restart();
			blinker.restart(1s);
		}

		else if (message.identifier == CANID_MOTOR_SETPOINT && message.length == 4)
		{
			if (setpoint_error)
				continue;

			int16_t cmdtheta = (message.data[0] << 8) | message.data[1];
			int16_t cmdspeed = (message.data[2] << 8) | message.data[3];

			// cmdtheta = cmdtheta/4;
	
			int16_t pwm_right = cmdspeed + cmdtheta;
			int16_t pwm_left = -cmdspeed + cmdtheta;

			if (pwm_right > 255)
				pwm_right = 255;
			else if (pwm_right < -255)
				pwm_right = -255;

			if (pwm_left > 255)
				pwm_left = 255;
			else if (pwm_left < -255)
				pwm_left = -255;
			
			if (pwm_right == 0) {
				M1_in1::reset();
				M1_in2::reset();
			} else if (pwm_right > 0) {
				M1_in1::set();
				M1_in2::reset();
			} else {
				M1_in1::reset();
				M1_in2::set();
			}

			if (pwm_left == 0) {
				M2_in1::reset();
				M2_in2::reset();
			} else if (pwm_left > 0) {
				M2_in1::set();
				M2_in2::reset();
			} else {
				M2_in1::reset();
				M2_in2::set();
			}

			uint16_t right_timer_cmp = map(abs(pwm_right), 0, 255, 0, timer_overflow);
			uint16_t left_timer_cmp = map(abs(pwm_left), 0, 255, 0, timer_overflow);
			
			Timer1::setCompareValue<M1_pwm::Ch1>(right_timer_cmp);
			Timer1::setCompareValue<M2_pwm::Ch2>(left_timer_cmp);
			
			// MODM_LOG_INFO << "[CAN] SETPOINT cmdtheta:" << cmdtheta << " cmdspeed:" << cmdspeed << modm::endl;

			enc1.read();
			enc2.read();
			uint16_t enc1_data_raw = enc1_data.data & 0x3FFF;
			uint16_t enc2_data_raw = enc2_data.data & 0x3FFF;

			modm::can::Message response(CANID_MOTOR_SETPOINT_RESPONSE, 5);
			response.data[0] = enc1_data_raw >> 8;
			response.data[1] = enc1_data_raw & 0xFF;
			response.data[2] = enc2_data_raw >> 8;
			response.data[3] = enc2_data_raw & 0xFF;
			response.data[4] = setpoint_timeout.remaining().count();
			Can1::sendMessage(response);

			setpoint_timeout.restart();
		}
	}

	return 0;
}
