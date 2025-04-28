#include <modm/board.hpp>
#include <modm/platform.hpp>
#include <modm/processing.hpp>

#include "can_identifiers.hpp"

// Set the log level
#undef MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::INFO

using step_en = GpioInverted<GpioOutputA15>;
using step1 = GpioOutputA12;
using dir1 = GpioOutputA11;
using step2 = GpioOutputA10;
using dir2 = GpioOutputC7;
using step3 = GpioOutputC6;
using dir3 = GpioOutputB15;
using step4 = GpioOutputA14;
using dir4 = GpioOutputA13;
using step5 = GpioOutputB12;
using dir5 = GpioOutputC4;

using hc165_clk = GpioOutputC11;
using hc165_data = GpioOutputC10;
using hc165_latch = GpioOutputC12;

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
    static constexpr uint32_t Timer6 = Apb1Timer;
    static constexpr uint32_t Timer7 = Apb1Timer;
    static constexpr uint32_t Timer8 = Apb2Timer;
    static constexpr uint32_t Timer9 = Apb2Timer;
    static constexpr uint32_t Timer10 = Apb2Timer;
    static constexpr uint32_t Timer11 = Apb2Timer;
    static constexpr uint32_t Timer12 = Apb1Timer;
    static constexpr uint32_t Timer13 = Apb1Timer;
    static constexpr uint32_t Timer14 = Apb1Timer;

    static constexpr uint32_t Iwdg = Rcc::LsiFrequency;
    static constexpr uint32_t Rtc = 32.768_kHz;

    static bool inline enable()
    {
        Rcc::enableExternalCrystal();
        const Rcc::PllFactors pllFactors {
            .pllM = 4,
            .pllN = 72,
            .pllP = 2,
            .pllQ = 2,
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

uint16_t read_tors()
{
    uint16_t out = 0;

    const uint8_t tors_mapping[16] = { 12, 13, 14, 15, 0, 9, 10, 11, 5, 6, 7, 8, 1, 2, 3, 4 };

    hc165_latch::reset();
    modm::delay_ns(100);
    hc165_latch::set();
    modm::delay_ns(100);

    for (uint8_t i = 0; i < 16; ++i) {
        out |= hc165_data::read() << tors_mapping[i];

        hc165_clk::set();
        modm::delay_ns(100);
        hc165_clk::reset();
        modm::delay_ns(100);
    }
    return out;
}

int main()
{
    SystemClock::enable();
    SysTickTimer::initialize<SystemClock>();

    Board::stlink::Uart::connect<Board::stlink::Tx::Tx, Board::stlink::Rx::Rx>();
    Board::stlink::Uart::initialize<SystemClock, 115200_Bd>();

    Board::LedD13::setOutput();

    hc165_clk::setOutput();
    hc165_data::setInput();
    hc165_latch::setOutput();
    hc165_clk::reset();
    hc165_latch::set();

    step_en::setOutput();
    step1::setOutput();
    dir1::setOutput();
    step2::setOutput();
    dir2::setOutput();
    step3::setOutput();
    dir3::setOutput();
    step4::setOutput();
    dir4::setOutput();
    step5::setOutput();
    dir5::setOutput();
    step_en::reset();
    step1::reset();
    dir1::reset();
    step2::reset();
    dir2::reset();
    step3::reset();
    dir3::reset();
    step4::reset();
    dir4::reset();
    step5::reset();
    dir5::reset();

    MODM_LOG_INFO << "starting ioboard date:" << __DATE__ << " time:" __TIME__ << modm::endl;

    Can1::connect<GpioInputB8::Rx, GpioOutputB9::Tx>(Gpio::InputType::PullUp);
    Can1::initialize<SystemClock, 1_Mbps>(9);
    // filter 0x200 to 0x2FF
    CanFilter::setFilter(0, CanFilter::FIFO0, CanFilter::ExtendedIdentifier(0x200), CanFilter::ExtendedFilterMask(0x700));

    while (true) {
        uint16_t tors = read_tors();
        MODM_LOG_INFO << "tors:" << modm::bin << tors << modm::endl;
        modm::delay_ms(500);
    }

    modm::PeriodicTimer blinker { 50ms };

    bool first_can_alive = true;
    modm::PeriodicTimer can_alive_timer { 1s };

    while (true) {
        if (blinker.execute()) {
            Board::LedD13::toggle();
        }

        if (can_alive_timer.execute()) {
            modm::can::Message alive(CANID_IO_ALIVE, 1);
            alive.data[0] = first_can_alive;
            Can1::sendMessage(alive);

            first_can_alive = false;
        }

        if (!Can1::isMessageAvailable()) {
            continue;
        }

        modm::can::Message message;
        uint8_t filter_id;
        Can1::getMessage(message, &filter_id);

        if (message.identifier == CANID_IO_REBOOT && message.length == 0) {
            NVIC_SystemReset();
        }
    }

    return 0;
}
