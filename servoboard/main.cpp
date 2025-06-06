#include "can_identifiers.hpp"
#include "pca9685.hpp"
#include <modm/board.hpp>
#include <modm/platform.hpp>
#include <modm/processing.hpp>
#include <array>

#undef MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::INFO

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

struct SystemClock {
    static constexpr uint32_t Frequency = 80_MHz;
    static constexpr uint32_t Ahb = Frequency;
    static constexpr uint32_t Apb1 = Frequency;
    static constexpr uint32_t Apb2 = Frequency;

    static constexpr uint32_t Spi1 = Apb2;
    static constexpr uint32_t Spi3 = Apb1;

    static constexpr uint32_t Usart1 = Apb2;
    static constexpr uint32_t Usart2 = Apb1;

    static constexpr uint32_t Can1 = Apb1;

    static constexpr uint32_t I2c1 = Apb1;
    static constexpr uint32_t I2c3 = Apb1;

    static constexpr uint32_t Apb1Timer = Apb1 * 1;
    static constexpr uint32_t Apb2Timer = Apb2 * 1;
    static constexpr uint32_t Timer1 = Apb2Timer;
    static constexpr uint32_t Timer2 = Apb1Timer;
    static constexpr uint32_t Timer6 = Apb1Timer;
    static constexpr uint32_t Timer7 = Apb1Timer;
    static constexpr uint32_t Timer15 = Apb2Timer;
    static constexpr uint32_t Timer16 = Apb2Timer;

    static bool inline enable()
    {
        Rcc::enableInternalClock();

        const Rcc::PllFactors pllFactors {
            .pllM = 1,
            .pllN = 10,
            .pllR = 2,
        };
        Rcc::enablePll(Rcc::PllSource::InternalClock, pllFactors);
        Rcc::setFlashLatency<Frequency>();
        Rcc::enableSystemClock(Rcc::SystemClockSource::Pll);
        Rcc::setAhbPrescaler(Rcc::AhbPrescaler::Div1);
        Rcc::setApb1Prescaler(Rcc::Apb1Prescaler::Div1);
        Rcc::setApb2Prescaler(Rcc::Apb2Prescaler::Div1);
        Rcc::updateCoreFrequency<Frequency>();

        return true;
    }
};

template <class I2cMaster>
class ServoboardArduino : public modm::I2cDevice<I2cMaster, 1, modm::I2cWriteTransaction> {
public:
    ServoboardArduino(uint8_t address = 0x8) : modm::I2cDevice<I2cMaster, 1, modm::I2cWriteTransaction>(address) {
    }

    void reset_all_leds() {
        set_led_pattern(0, 255);
        set_led_pattern(1, 255);
        set_led_pattern(2, 255);
        set_led_pattern(3, 255);
    }

    bool set_led_pattern(uint8_t id, uint8_t pattern) {
        if (id > 3) {
            return false;
        }
        uint8_t buffer[2];
        buffer[0] = id;
        buffer[1] = pattern;
        this->transaction.configureWrite(buffer, 2);
        return this->runTransaction();
    }
};

int main()
{
    SystemClock::enable();
    SysTickTimer::initialize<SystemClock>();

    Board::stlink::Uart::connect<Board::stlink::Tx::Tx, Board::stlink::Rx::Rx>(Gpio::InputType::PullUp);
    Board::stlink::Uart::initialize<SystemClock, 115200_Bd>();

    Board::LedD13::setOutput();
    Board::LedD13::set();

    pca_oe::setOutput();
    pca_oe::reset();

    power1_oe::setOutput();
    power2_oe::setOutput();
    power3_oe::setOutput();
    power1_oe::reset();
    power2_oe::reset();
    power3_oe::reset();

    // let's wait for the arduino nano bootleader to wake up
    modm::delay_ms(3000);

    power3_adc::setInput(Gpio::InputType::Floating);
    Adc1::initialize(Adc1::ClockMode::SynchronousPrescaler4, Adc1::ClockSource::SystemClock, Adc1::Prescaler::Div4, Adc1::CalibrationMode::SingleEndedInputsMode, true);
    Adc1::connect<power3_adc_chan>();
    Adc1::setPinChannel<power3_adc>(Adc1::SampleTime::Cycles248);

    I2cMaster3::connect<GpioB4::Sda, GpioA7::Scl>();
    I2cMaster3::initialize<SystemClock, 400_kHz>();
    ServoboardArduino<I2cMaster3> servoboard_arduino;
    servoboard_arduino.reset_all_leds();

    const uint8_t servos_mapping[16] = { 11, 10, 9, 8, 7, 6, 5, 4, 12, 13, 14, 15, 3, 2, 1, 0 };
    I2cMaster1::connect<GpioB7::Sda, GpioB6::Scl>();
    I2cMaster1::initialize<SystemClock, 400_kHz>();
    PCA9685 pca9685 = PCA9685<I2cMaster1>(0x40);
    pca9685.initialize();
    pca9685.set_pwm_freq(50);
    pca_oe::set();

    Can1::connect<GpioA11::Rx, GpioA12::Tx>(Gpio::InputType::PullUp);
    Can1::initialize<SystemClock, 1_Mbps>(9);
    // filter 0x100 to 0x1FF
    CanFilter::setFilter(0, CanFilter::FIFO0, CanFilter::StandardIdentifier(0x100), CanFilter::StandardFilterMask(0x700));
    Can1::setAutomaticRetransmission(true);

    MODM_LOG_INFO << "starting servoboard date:" << __DATE__ << " time:" __TIME__ << modm::endl;
    MODM_LOG_INFO.flush();

    modm::PeriodicTimer blinker { 50ms };

    bool first_alive_since_reboot = true;
    modm::PeriodicTimer timer_alive { 1s };

    modm::PeriodicTimer power_current_measuer_timer { 200ms };

    while (true) {
        if (blinker.execute()) {
            Board::LedD13::toggle();
        }

        if (timer_alive.execute() || first_alive_since_reboot) {
            modm::can::Message alive(CANID_SERVO_ALIVE, 1);
            alive.setExtended(false);
            alive.data[0] = first_alive_since_reboot;
            Can1::sendMessage(alive);

            first_alive_since_reboot = false;
        }

        if (power_current_measuer_timer.execute()) {
            uint8_t adc_values[3];

            Adc1::startConversion();
            while (!Adc1::isConversionFinished()) { }
            adc_values[0] = Adc1::getValue();

            Adc1::startConversion();
            while (!Adc1::isConversionFinished()) { }
            adc_values[1] = Adc1::getValue();

            Adc1::startConversion();
            while (!Adc1::isConversionFinished()) { }
            adc_values[2] = Adc1::getValue();

            uint8_t powers = power1_oe::read() | power2_oe::read() << 1 | power3_oe::read() << 2;

            MODM_LOG_INFO << "ADC0:" << adc_values[0] << modm::endl;
            MODM_LOG_INFO << "ADC1:" << adc_values[1] << modm::endl;
            MODM_LOG_INFO << "ADC2:" << adc_values[2] << modm::endl << modm::endl;
            MODM_LOG_INFO.flush();

            modm::can::Message msg(CANID_SERVO_STATUS, 7);
            msg.setExtended(false);
            msg.data[0] = adc_values[0] << 8 & 0xFF;
            msg.data[1] = adc_values[0] & 0xFF;
            msg.data[2] = adc_values[1] << 8 & 0xFF;
            msg.data[3] = adc_values[1] & 0xFF;
            msg.data[4] = adc_values[2] << 8 & 0xFF;
            msg.data[5] = adc_values[2] & 0xFF;
            msg.data[6] = powers;
            Can1::sendMessage(msg);
        }

        if (!Can1::isMessageAvailable()) {
            continue;
        }

        modm::can::Message message;
        Can1::getMessage(message);

        if (message.identifier == CANID_SERVO_REBOOT && message.length == 0) {
            NVIC_SystemReset();
        }

        else if (message.identifier == CANID_SERVO_ENABLE_POWER && message.length == 3) {
            bool power1 = message.data[0];
            bool power2 = message.data[1];
            bool power3 = message.data[2];

            power1_oe::set(power1);
            power2_oe::set(power2);
            power3_oe::set(power3);

            MODM_LOG_INFO << "ENABLE_POWER power1:" << power1 << " power2:" << power2 << " power3:" << power3 << modm::endl;
            MODM_LOG_INFO.flush();
        }

        else if (message.identifier == CANID_SERVO_WRITE_US && message.length == 3) {
            uint8_t servo_id = message.data[0];
            uint16_t servo_us = (message.data[1] << 8) | message.data[2];

            if (servo_id >= 16) {
                modm::can::Message err(CANID_SERVO_ERROR_INVALID_PARAMS, 0);
                err.setExtended(false);
                Can1::sendMessage(err);
                continue;
            }

            if (servo_us < 500 || servo_us > 2500) {
                modm::can::Message err(CANID_SERVO_ERROR_INVALID_PARAMS, 0);
                err.setExtended(false);
                Can1::sendMessage(err);
                continue;
            }
            
            if ((servo_id <= 7 && !power1_oe::read()) || (servo_id > 7 && !power2_oe::read())) {
                modm::can::Message err(CANID_SERVO_ERROR_NOT_ENABLED, 1);
                err.setExtended(false);
                err.data[0] = servo_id;
                Can1::sendMessage(err);
                continue;
            }

            pca9685.write_us(servos_mapping[servo_id], servo_us);
        }

        else if (message.identifier == CANID_SERVO_SET_LED_PATTERN && message.length == 2) {
            uint8_t led_id = message.data[0];
            uint8_t led_pattern = message.data[1];

            if (led_id >= 4) {
                modm::can::Message err(CANID_SERVO_ERROR_INVALID_PARAMS, 0);
                err.setExtended(false);
                Can1::sendMessage(err);
                continue;
            }

            if (!power3_oe::read()) {
                modm::can::Message err(CANID_SERVO_ERROR_NOT_ENABLED, 1);
                err.setExtended(false);
                err.data[0] = led_id;
                Can1::sendMessage(err);
                continue;
            }

            servoboard_arduino.set_led_pattern(led_id, led_pattern);
        }
    }
}