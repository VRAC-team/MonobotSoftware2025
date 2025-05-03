#include "can_identifiers.hpp"
#include <limits>
#include <modm/architecture/interface/clock.hpp>
#include <modm/board.hpp>
#include <modm/debug/logger.hpp>
#include <modm/driver/encoder/as5047.hpp>
#include <modm/platform.hpp>
#include <modm/processing.hpp>

using namespace modm::literals;

// Set the log level
#undef MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::INFO

using M1_pwm = GpioA8; // timer1 chan1
using M2_pwm = GpioA9; // timer1 chan2
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

modm::as5047::Data enc1_data { 0 };
modm::As5047<enc_SPI, enc1_cs> enc1 { enc1_data };
modm::as5047::Data enc2_data { 0 };
modm::As5047<enc_SPI, enc2_cs> enc2 { enc2_data };

struct SystemClock {
    static constexpr uint32_t Frequency = 72_MHz;
    static constexpr uint32_t Ahb = Frequency;
    static constexpr uint32_t Apb1 = Frequency / 2;
    static constexpr uint32_t Apb2 = Frequency / 1;

    static constexpr uint32_t Adc = Apb2;

    static constexpr uint32_t Spi1 = Apb2;
    static constexpr uint32_t Spi2 = Apb1;
    static constexpr uint32_t Spi3 = Apb1;
    static constexpr uint32_t Spi4 = Apb2;
    static constexpr uint32_t Spi5 = Apb2;

    static constexpr uint32_t Usart1 = Apb2;
    static constexpr uint32_t Usart2 = Apb1;
    static constexpr uint32_t Usart3 = Apb1;
    static constexpr uint32_t Uart4 = Apb1;
    static constexpr uint32_t Uart5 = Apb1;
    static constexpr uint32_t Usart6 = Apb2;

    static constexpr uint32_t Can1 = Apb1;
    static constexpr uint32_t Can2 = Apb1;

    static constexpr uint32_t I2c1 = Apb1;
    static constexpr uint32_t I2c2 = Apb1;
    static constexpr uint32_t I2c3 = Apb1;

    static constexpr uint32_t Apb1Timer = Apb1 * 1;
    static constexpr uint32_t Apb2Timer = Apb2 * 1;
    static constexpr uint32_t Timer1 = Apb2Timer;
    static constexpr uint32_t Timer2 = Apb1Timer;
    static constexpr uint32_t Timer3 = Apb1Timer;
    static constexpr uint32_t Timer4 = Apb1Timer;
    static constexpr uint32_t Timer5 = Apb1Timer;
    static constexpr uint32_t Timer9 = Apb2Timer;
    static constexpr uint32_t Timer10 = Apb2Timer;
    static constexpr uint32_t Timer11 = Apb2Timer;
    static constexpr uint32_t Iwdg = Rcc::LsiFrequency;
    static constexpr uint32_t Rtc = 32.768_kHz;

    static bool inline enable()
    {
        Rcc::enableExternalCrystal();
        const Rcc::PllFactors pllFactors {
            .pllM = 4,
            .pllN = 72,
            .pllP = 2,
        };
        Rcc::enablePll(Rcc::PllSource::Hse, pllFactors);
        Rcc::setFlashLatency<Frequency>();
        Rcc::enableSystemClock(Rcc::SystemClockSource::Pll);
        Rcc::setAhbPrescaler(Rcc::AhbPrescaler::Div1);
        Rcc::setApb1Prescaler(Rcc::Apb1Prescaler::Div2);
        Rcc::setApb2Prescaler(Rcc::Apb2Prescaler::Div1);
        Rcc::updateCoreFrequency<Frequency>();

        return true;
    }
};

void send_status(bool state_error) {
    enc1.read();
    enc2.read();
    uint16_t enc1_data_raw = enc1_data.data & 0x3FFF;
    uint16_t enc2_data_raw = enc2_data.data & 0x3FFF;

    modm::can::Message status(CANID_MOTOR_STATUS, 5);
    status.setExtended(false);
    status.data[0] = state_error;
    status.data[1] = enc1_data_raw >> 8;
    status.data[2] = enc1_data_raw & 0xFF;
    status.data[3] = enc2_data_raw >> 8;
    status.data[4] = enc2_data_raw & 0xFF;
    Can1::sendMessage(status);
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

    MODM_LOG_INFO << "starting motorboard date:" << __DATE__ << " time:" __TIME__ << modm::endl;

    Timer1::connect<M1_pwm::Ch1>();
    Timer1::connect<M2_pwm::Ch2>();
    Timer1::enable();
    Timer1::setMode(Timer1::Mode::UpCounter);
    Timer1::setPeriod<SystemClock>(50us); // 20 khz
    Timer1::configureOutputChannel<M1_pwm::Ch1>(Timer1::OutputCompareMode::Pwm, 0);
    Timer1::configureOutputChannel<M2_pwm::Ch2>(Timer1::OutputCompareMode::Pwm, 0);
    Timer1::applyAndReset();
    Timer1::start();
    Timer1::enableOutput();

    enc_SPI::connect<enc_miso::Miso, enc_mosi::Mosi, enc_sck::Sck>();
    enc_SPI::initialize<SystemClock, 2.25_MHz>();

    Can1::connect<GpioInputB8::Rx, GpioOutputB9::Tx>(Gpio::InputType::PullUp);
    Can1::initialize<SystemClock, 1_Mbps>(9);
    // filter 0x000 to 0x0FF
    CanFilter::setFilter(0, CanFilter::FIFO0, CanFilter::StandardIdentifier(0), CanFilter::StandardFilterMask(0x700));
    Can1::setAutomaticRetransmission(true);

    const uint16_t timer1_overflow = Timer1::getOverflow();

    bool state_error = true;
    const std::chrono::milliseconds state_error_interval = 10ms;
    modm::PreciseTimeout timeout_state_error { state_error_interval };

    const std::chrono::milliseconds blinker_interval_error = 200ms;
    const std::chrono::milliseconds blinker_interval_nominal = 50ms;
    modm::PeriodicTimer timer_blinker { blinker_interval_error };

    bool first_alive_since_reboot = true;
    modm::PeriodicTimer timer_can_alive { 1s };
    
    const std::chrono::milliseconds timer_status_interval = 5ms;
    modm::PeriodicTimer timer_status { timer_status_interval };

    while (true) {
        if (timer_blinker.execute()) {
            Board::LedD13::toggle();
        }

        if (timeout_state_error.execute()) {
            state_error = true;
            M1_en::reset();
            M2_en::reset();
            M1_in1::reset();
            M1_in2::reset();
            M2_in1::reset();
            M2_in2::reset();
            Timer1::setCompareValue<M1_pwm::Ch1>(0);
            Timer1::setCompareValue<M2_pwm::Ch2>(0);

            timer_blinker.restart(blinker_interval_error);
            timer_status.restart(timer_status_interval);

            modm::can::Message msg(CANID_MOTOR_STATE_ERROR, 0);
            msg.setExtended(false);
            Can1::sendMessage(msg);
        }

        if (timer_status.execute()) {
            send_status(state_error);
        }

        if (timer_can_alive.execute()) {

            modm::can::Message alive(CANID_MOTOR_ALIVE, 1);
            alive.setExtended(false);
            alive.data[0] = first_alive_since_reboot;
            Can1::sendMessage(alive);

            first_alive_since_reboot = false;
        }

        if (!Can1::isMessageAvailable()) {
            continue;
        }

        modm::can::Message message;
        uint8_t filter_id;
        Can1::getMessage(message, &filter_id);

        if (message.identifier == CANID_MOTOR_REBOOT && message.length == 0) {
            NVIC_SystemReset();
        }

        else if (message.identifier == CANID_MOTOR_PWM_WRITE && message.length == 4) {
            if (state_error) {
                // silently fail here
                continue;
            }
                

            int16_t pwm_right = (message.data[0] << 8) | message.data[1];
            int16_t pwm_left = (message.data[2] << 8) | message.data[3];

            uint16_t right_timer_cmp = 0;
            uint16_t left_timer_cmp = 0;

            if (pwm_right == 0) {
                M1_in1::reset();
                M1_in2::reset();
            } else if (pwm_right > 0) {
                M1_in1::set();
                M1_in2::reset();
                right_timer_cmp = pwm_right * timer1_overflow / SHRT_MAX;
            } else {
                M1_in1::reset();
                M1_in2::set();
                right_timer_cmp = -pwm_right * timer1_overflow / -SHRT_MIN;
            }

            if (pwm_left == 0) {
                M2_in1::reset();
                M2_in2::reset();
            } else if (pwm_left > 0) {
                M2_in1::set();
                M2_in2::reset();
                left_timer_cmp = pwm_left * timer1_overflow / SHRT_MAX;
            } else {
                M2_in1::reset();
                M2_in2::set();
                left_timer_cmp = -pwm_left * timer1_overflow / -SHRT_MIN;
            }

            Timer1::setCompareValue<M1_pwm::Ch1>(right_timer_cmp);
            Timer1::setCompareValue<M2_pwm::Ch2>(left_timer_cmp);

            send_status(false);

            timeout_state_error.restart(state_error_interval);
        }

        else if (message.identifier == CANID_MOTOR_RESET_STATE_ERROR && message.length == 0) {
            state_error = false;
            M1_en::set();
            M2_en::set();
            timer_status.stop();
            timeout_state_error.restart(state_error_interval);
            timer_blinker.restart(blinker_interval_nominal);
        }
    }

    return 0;
}
