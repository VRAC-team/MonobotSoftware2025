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

typedef enum {
    STEP_IDLE,
    STEP_ACCEL,
    STEP_CRUISE,
    STEP_DECEL
} StepperState;

typedef struct {
    int32_t current_position; // current absolute position in steps
    int32_t target_position;  // target absolute position in steps
    int32_t steps_remaining;  // how many steps left to target
    float accel;              // steps/s²
    float vmax;               // steps/s
    float current_speed;      // current speed in steps/s
    float target_speed;       // maximum target speed
    StepperState state;
    uint32_t next_step_delay_us;
    int32_t accel_steps;
    int32_t decel_steps;
    int32_t cruise_steps;
    int8_t direction;         // +1 or -1
} Stepper;

Stepper motor;

void stepper_start_move(int32_t target, float accel, float vmax) {
    int32_t delta = target - motor.current_position;

    if (delta == 0) return; // No movement needed

    if (delta > 0) {
        dir5::set();
        motor.direction = +1;
    } else {
        dir5::reset();
        motor.direction = -1;
        delta = -delta; // work with positive steps
    }

    motor.target_position = target;
    motor.steps_remaining = delta;
    motor.accel = accel;
    motor.vmax = vmax;
    motor.current_speed = 0;
    motor.state = STEP_ACCEL;
    motor.next_step_delay_us = 1e6f * sqrtf(2.0f / motor.accel);

    float accel_steps = (vmax * vmax) / (2.0f * accel);

    MODM_LOG_INFO << "next_step_delay_us:" << motor.next_step_delay_us << modm::endl;
    MODM_LOG_INFO << "accel_steps:" << accel_steps << modm::endl;

    if (2.0f * accel_steps > delta) {
        // Triangle profile
        MODM_LOG_INFO << "====Triangle profile====" << modm::endl;
        motor.target_speed = sqrtf(accel * delta);
        motor.accel_steps = delta / 2;
        motor.decel_steps = delta - motor.accel_steps;
        motor.cruise_steps = 0;
    } else {
        // Trapezoidal profile
        MODM_LOG_INFO << "====Trapezoidal profile====" << modm::endl;
        motor.target_speed = vmax;
        motor.accel_steps = accel_steps;
        motor.decel_steps = accel_steps;
        motor.cruise_steps = delta - motor.accel_steps - motor.decel_steps;
    }

    MODM_LOG_INFO << "target_speed:" << motor.target_speed << modm::endl;
    MODM_LOG_INFO << "accel_steps:" << motor.accel_steps << modm::endl;
    MODM_LOG_INFO << "cruise_steps:" << motor.cruise_steps << modm::endl;
    MODM_LOG_INFO << "decel_steps:" << motor.decel_steps << modm::endl;
}

void stepper_update() {
    static uint32_t last_step_time = 0;

    if (motor.steps_remaining <= 0) {
        motor.state = STEP_IDLE;
        return;
    }

    uint32_t micros = modm::PreciseClock::now().time_since_epoch().count();
    if (micros - last_step_time < motor.next_step_delay_us) {
        return;
    }
    last_step_time = micros;

    switch (motor.state) {
        case STEP_ACCEL:
            motor.current_speed += motor.accel * (motor.next_step_delay_us / 1e6f);

            if (motor.current_speed >= motor.target_speed || motor.steps_remaining <= motor.cruise_steps + motor.decel_steps) {
                motor.current_speed = motor.target_speed;
                if (motor.cruise_steps > 0)
                    motor.state = STEP_CRUISE;
                else
                    motor.state = STEP_DECEL;
            }
            break;

        case STEP_CRUISE:
            if (motor.steps_remaining <= motor.decel_steps) {
                motor.state = STEP_DECEL;
            }
            break;

        case STEP_DECEL:
            motor.current_speed -= motor.accel * (motor.next_step_delay_us / 1e6f);
            if (motor.current_speed < 0) {
                motor.current_speed = 0;
            }
            break;

        default:
            break;
    }

    // Issue a step pulse
    step5::set();
    modm::delay_ns(500);
    step5::reset();

    // Update the logical current position
    motor.current_position += motor.direction;

    // Compute delay for next step
    motor.next_step_delay_us = 1e6f / motor.current_speed;

    motor.steps_remaining--;
}

int main()
{
    SystemClock::enable();
    SysTickTimer::initialize<SystemClock>();

    Board::stlink::Uart::connect<Board::stlink::Tx::Tx, Board::stlink::Rx::Rx>();
    Board::stlink::Uart::initialize<SystemClock, 115200_Bd>();

    MODM_LOG_INFO << "starting ioboard date:" << __DATE__ << " time:" __TIME__ << modm::endl;

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

    Can1::connect<GpioInputB8::Rx, GpioOutputB9::Tx>(Gpio::InputType::PullUp);
    Can1::initialize<SystemClock, 1_Mbps>(9);
    // filter 0x200 to 0x2FF
    CanFilter::setFilter(0, CanFilter::FIFO0, CanFilter::ExtendedIdentifier(0x200), CanFilter::ExtendedFilterMask(0x700));
    
    step_en::set();

	// Timer2::enable();
	// Timer2::setMode(Timer2::Mode::UpCounter, Timer2::SlaveMode::Disabled, Timer2::SlaveModeTrigger::Internal0, Timer2::MasterMode::Reset, true);
	// Timer2::setPeriod<Board::SystemClock>(std::chrono::microseconds(current_period));
	// Timer2::enableInterrupt(Timer2::Interrupt::Update);
	// Timer2::enableInterruptVector(true, 1);
	// Timer2::applyAndReset();
	// Timer2::start();

    uint32_t steps_per_rev = 200*8;
    stepper_start_move(steps_per_rev*10, steps_per_rev*20, steps_per_rev*4);
    uint32_t last_step_time = 0;

    while (true) {
        stepper_update();
    }

    modm::PeriodicTimer blinker { 50ms };
    
    bool first_can_alive = true;
    modm::PeriodicTimer can_alive_timer { 1s };
    
    modm::PeriodicTimer status_timer { 20ms };

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

        if (status_timer.execute()) {
            uint16_t tors = read_tors();

            modm::can::Message status(CANID_IO_STATUS, 2);
            status.data[0] = tors >> 8;
            status.data[1] = tors & 0xFF;
            Can1::sendMessage(status);
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

        if (message.identifier == CANID_IO_STEPPER_ENABLE && message.length == 1) {
            bool en = message.data[0] & 1;

            step_en::set(en);
        }

        else if (message.identifier == CANID_IO_STEPPER_HOME && message.length == 1) {
            uint8_t id = message.data[0];

            if (id > 4) {
                continue;
            }

            //start homing of stepper id
        }

        else if (message.identifier == CANID_IO_STEPPER_GOTO && message.length == 2) {
            uint8_t id = message.data[0];
            int16_t steps = (message.data[1] << 8) | message.data[2];

            if (id > 4) {
                continue;
            }

            //start move of stepper id
        }
    }

    return 0;
}
