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
    // disabling interrupts there, steppers timer isr could be called during this function wich also utilize read_tors() when homing
    modm::atomic::Lock lock;

    uint16_t out = 0;

    const uint8_t tors_mapping[16] = { 12, 13, 14, 15, 0, 9, 10, 11, 5, 6, 7, 8, 1, 2, 3, 4 };

    hc165_latch::reset();
    modm::delay_ns(20);
    hc165_latch::set();

    for (uint8_t i = 0; i < 16; ++i) {
        out |= hc165_data::read() << tors_mapping[i];

        hc165_clk::set();
        modm::delay_ns(16);
        hc165_clk::reset();
    }
    return out;
}

enum class MotionStatus : uint8_t {
    Idle,
    IsDoingHome,
    IsDoingGoto,
    FlagHomeSucceeded,
    FlagHomeFailed,
    FlagGotoFinished
};

enum class MotionStartResult : uint8_t {
    Error_InvalidParams,
    Error_AlreadyInMotion,
    AlreadyDone, // when goto() or home() with zero steps
    Started
};

class IAccelStepper {
public:
    virtual ~IAccelStepper() = default;
    virtual MotionStartResult home(int32_t max_relative_steps_before_error, uint32_t acceleration, uint32_t max_velocity, uint8_t homing_tor_index, bool tor_state_to_end_homing) = 0;
    virtual MotionStartResult goto_absolute(int32_t absolute_steps, uint32_t acceleration, uint32_t max_velocity) = 0;
    virtual void cancel_current_motion() = 0;
    virtual void isr_step() = 0;
    virtual MotionStatus get_status() = 0;
    virtual bool clear_status_flag() = 0;
};

template <typename Step, typename Dir, typename Timer>
class AccelStepper : public IAccelStepper {
public:
    AccelStepper() : IAccelStepper() {
        Step::setOutput();
        Dir::setOutput();
        Step::reset();
        Dir::reset();
        m_status = MotionStatus::Idle;
        m_current_position = 0;
        m_steps_remaining = 0;
    }

    MotionStartResult home(int32_t max_relative_steps_before_error, uint32_t acceleration, uint32_t max_velocity, uint8_t homing_tor_index, bool tor_state_to_end_homing) {
        if (m_status != MotionStatus::Idle) {
            return MotionStartResult::Error_AlreadyInMotion;
        }

        if (max_relative_steps_before_error == 0) {
            // no need change m_status there, it is handled immediately
            return MotionStartResult::AlreadyDone;
        }

        if (acceleration == 0 || max_velocity == 0) {
            return MotionStartResult::Error_InvalidParams;
        }

        uint32_t total_steps = abs(max_relative_steps_before_error);

        if (max_relative_steps_before_error > 0) {
            Dir::set();
            m_direction = 1;
        } else {
            Dir::reset();
            m_direction = -1;
        }

        m_homing_tor_index = homing_tor_index;
        m_homing_tor_state_to_end = tor_state_to_end_homing;

        m_steps_remaining = total_steps;
        m_acceleration = acceleration;
        m_max_velocity = max_velocity;
        m_current_velocity = 0;
        m_current_step_period_us = 1000000.0f * sqrtf(2.0f / m_acceleration);
    
        m_accel_steps = (m_max_velocity * m_max_velocity) / (2.0f * m_acceleration);  

        if (m_accel_steps > m_steps_remaining) {
            m_accel_steps = m_steps_remaining;
            m_max_velocity = sqrtf(m_acceleration * m_steps_remaining);
        }

        m_const_steps = m_steps_remaining - m_accel_steps;
        m_decel_steps = 0;
        
        m_acceleration /= 1000000.0f; //convert motor accel to steps/microseconds^2
        m_remaining_steps_decel_phase = m_decel_steps;
        m_remaining_steps_const_phase = m_decel_steps + m_const_steps;
        m_remaining_steps_accel_phase = m_decel_steps + m_const_steps + m_accel_steps;

        Timer::enable();
        Timer::setMode(Timer::Mode::OneShotUpCounter, Timer::SlaveMode::Disabled, Timer::SlaveModeTrigger::Internal0, Timer::MasterMode::Reset);
        Timer::enableInterrupt(Timer::Interrupt::Update);
        Timer::enableInterruptVector(true, 1);
        Timer::template setPeriod<SystemClock>(std::chrono::microseconds(m_current_step_period_us));
    
        m_status = MotionStatus::IsDoingHome;
    
        Timer::applyAndReset();
        Timer::start();
    
        return MotionStartResult::Started;
    }

    MotionStartResult goto_absolute(int32_t absolute_steps, uint32_t acceleration, uint32_t max_velocity) {
        if (m_status != MotionStatus::Idle) {
            return MotionStartResult::Error_AlreadyInMotion;
        }

        if (acceleration == 0 || max_velocity == 0) {
            return MotionStartResult::Error_InvalidParams;
        }

        int32_t delta = absolute_steps - m_current_position;
        uint32_t total_steps = abs(delta);

        if (total_steps == 0) {
            // no need change m_status there, it is handled immediately
            return MotionStartResult::AlreadyDone;
        }
    
        if (delta > 0) {
            Dir::set();
            m_direction = 1;
        } else {
            Dir::reset();
            m_direction = -1;
        }

        m_steps_remaining = total_steps;
        m_acceleration = acceleration;
        m_max_velocity = max_velocity;
        m_current_velocity = 0;
        m_current_step_period_us = 1000000.0f * sqrtf(2.0f / m_acceleration);
        
        // first do calculations for a trapzoidal profile
        m_accel_steps = (m_max_velocity * m_max_velocity) / (2.0f * m_acceleration);
        m_decel_steps = m_accel_steps;
        m_const_steps = total_steps - m_accel_steps - m_decel_steps;
        
        // if the profile is a triangle, recalculate steps
        if (2.0f * m_accel_steps > total_steps) {
            m_max_velocity = sqrtf(m_acceleration * total_steps);
            m_accel_steps = total_steps / 2;
            m_decel_steps = total_steps - m_accel_steps;
            m_const_steps = 0;
        }
    
        // precompute as much as we can, to shorten isr_step() execution time
        m_acceleration /= 1000000.0f; //convert motor accel to steps/microseconds^2
        m_remaining_steps_decel_phase = m_decel_steps;
        m_remaining_steps_const_phase = m_decel_steps + m_const_steps;
        m_remaining_steps_accel_phase = m_decel_steps + m_const_steps + m_accel_steps;
    
        Timer::enable();
        Timer::setMode(Timer::Mode::OneShotUpCounter, Timer::SlaveMode::Disabled, Timer::SlaveModeTrigger::Internal0, Timer::MasterMode::Reset);
        Timer::enableInterrupt(Timer::Interrupt::Update);
        Timer::enableInterruptVector(true, 1);
        Timer::template setPeriod<SystemClock>(std::chrono::microseconds(m_current_step_period_us));

        m_status = MotionStatus::IsDoingGoto;
    
        Timer::applyAndReset();
        Timer::start();
    
        return MotionStartResult::Started;
    }

    void cancel_current_motion() {
        modm::atomic::Lock lock;
        Timer::pause();
        m_status = MotionStatus::Idle;
        m_steps_remaining = 0;
    }

    void isr_step() {
        if (m_status == MotionStatus::IsDoingHome) {
            // uint16_t tors = read_tors(); // I measured 8527ns for this line, good enough i guess
            bool tor_state = (read_tors() >> m_homing_tor_index) & 1;
            if (m_homing_tor_state_to_end == tor_state) {
                m_current_position = 0;
                m_steps_remaining = 0;
                m_status = MotionStatus::FlagHomeSucceeded;
                return;
            }
        }
    
        m_steps_remaining--;
        m_current_position += m_direction;
    
        Step::set();
        // according to datasheet, step high time should be at least 1.9us ish if i understood correctly, another timer could be used to create this delay
        // but with my testings, this is working fine with 100ns
        modm::delay_ns(100);
        Step::reset();
    
        if (m_steps_remaining == 0) {
            if (m_status == MotionStatus::IsDoingGoto) {
                m_status = MotionStatus::FlagGotoFinished;
                return;
            }
            
            if (m_status == MotionStatus::IsDoingHome) {
                m_status = MotionStatus::FlagHomeFailed;
                return;
            }
        }
    
        if (m_steps_remaining <= m_remaining_steps_decel_phase) {
            m_current_velocity -= m_acceleration * m_current_step_period_us;
            if (m_current_velocity <= 0) {
                // i'm unsure this case can be possible, but just in case let's handle this correctly
                if (m_status == MotionStatus::IsDoingGoto) {
                    m_status = MotionStatus::FlagGotoFinished;
                    m_steps_remaining = 0;
                    return;
                }
                
                if (m_status == MotionStatus::IsDoingHome) {
                    m_status = MotionStatus::FlagHomeFailed;
                    m_steps_remaining = 0;
                    return;
                }
            }
        }
        else if (m_steps_remaining <= m_remaining_steps_const_phase) {
            m_current_velocity = m_max_velocity;
        }
        else if (m_steps_remaining <= m_remaining_steps_accel_phase) {
            m_current_velocity += m_acceleration * m_current_step_period_us;
            if (m_current_velocity > m_max_velocity) {
                m_current_velocity = m_max_velocity;
            }
        }
    
        // current_step_period_us doesn't account for the time to get there on this ISR, therefore it is a little bit more than it should be but not by much (TODO measure)
        m_current_step_period_us = 1000000.0f / m_current_velocity;
        
        Timer::template setPeriod<SystemClock>(std::chrono::microseconds(m_current_step_period_us));
        Timer::start();
    }

    bool is_doing_goto() {
        return m_status == MotionStatus::IsDoingGoto;
    }

    bool is_doing_home() {
        return m_status == MotionStatus::IsDoingHome;
    }
    
    MotionStatus get_status() {
        return m_status;
    }

    bool clear_status_flag() {
        switch (m_status) {
            case MotionStatus::FlagGotoFinished:
            case MotionStatus::FlagHomeSucceeded:
            case MotionStatus::FlagHomeFailed:
                m_status = MotionStatus::Idle;
                return true;
            default:
                return false;
        }
    }
private:
    volatile MotionStatus m_status;

    uint8_t m_homing_tor_index;
    bool m_homing_tor_state_to_end;

    int32_t m_current_position;
    uint32_t m_steps_remaining;
    float m_current_velocity;
    uint32_t m_current_step_period_us;

    float m_acceleration;
    float m_max_velocity;
    uint32_t m_accel_steps;
    uint32_t m_const_steps;
    uint32_t m_decel_steps;
    uint32_t m_remaining_steps_accel_phase;
    uint32_t m_remaining_steps_const_phase;
    uint32_t m_remaining_steps_decel_phase;
    int8_t m_direction; // +1 or -1
};

AccelStepper<step1, dir1, Timer13> stepper1;
AccelStepper<step2, dir2, Timer12> stepper2;
AccelStepper<step3, dir3, Timer11> stepper3;
AccelStepper<step4, dir4, Timer10> stepper4;
AccelStepper<step5, dir5, Timer9> stepper5;
IAccelStepper* steppers[5] = { &stepper1, &stepper2, &stepper3, &stepper4, &stepper5 };

MODM_ISR(TIM8_UP_TIM13)
{
    modm::atomic::Lock lock;

    if (Timer13::getInterruptFlags() & Timer13::InterruptFlag::Update) {
        Timer13::acknowledgeInterruptFlags(Timer13::InterruptFlag::Update);
        steppers[0]->isr_step();
    }
}

MODM_ISR(TIM8_BRK_TIM12)
{
    modm::atomic::Lock lock;

    if (Timer12::getInterruptFlags() & Timer12::InterruptFlag::Update) {
        Timer12::acknowledgeInterruptFlags(Timer12::InterruptFlag::Update);
        steppers[1]->isr_step();
    }
}

MODM_ISR(TIM1_TRG_COM_TIM11)
{
    modm::atomic::Lock lock;

    if (Timer11::getInterruptFlags() & Timer11::InterruptFlag::Update) {
        Timer11::acknowledgeInterruptFlags(Timer11::InterruptFlag::Update);
        steppers[2]->isr_step();
    }
}

MODM_ISR(TIM1_UP_TIM10)
{
    modm::atomic::Lock lock;

    if (Timer10::getInterruptFlags() & Timer10::InterruptFlag::Update) {
        Timer10::acknowledgeInterruptFlags(Timer10::InterruptFlag::Update);
        steppers[3]->isr_step();
    }
}

MODM_ISR(TIM1_BRK_TIM9)
{
    modm::atomic::Lock lock;

    if (Timer9::getInterruptFlags() & Timer9::InterruptFlag::Update) {
        Timer9::acknowledgeInterruptFlags(Timer9::InterruptFlag::Update);
        steppers[4]->isr_step();
    }
}




void handle_can_messages() {
    if (!Can1::isMessageAvailable()) {
        return;
    }

    modm::can::Message message;
    uint8_t filter_id;
    Can1::getMessage(message, &filter_id);

    if (message.identifier == CANID_IO_REBOOT && message.length == 0) {
        NVIC_SystemReset();
    }

    else if (message.identifier == CANID_IO_STEPPER_ENABLE && message.length == 1) {
        bool en = message.data[0] & 1;

        uint8_t steppers_that_were_active = 0;
        for (uint8_t stepper_id = 0 ; stepper_id < 5 ; ++stepper_id) {
            MotionStatus status = steppers[stepper_id]->get_status();
            if (status == MotionStatus::IsDoingHome || status == MotionStatus::IsDoingGoto) {
                steppers[stepper_id]->cancel_current_motion();
                steppers_that_were_active |= 1 << stepper_id;
            }
        }

        if (steppers_that_were_active != 0) {
            modm::can::Message err(CANID_IO_STEPPER_ERROR_DISABLED_DURING_MOTION, 1);
            err.setExtended(false);
            err.data[0] = steppers_that_were_active;
            Can1::sendMessage(err);
            return;
        }

        step_en::set(en);
    }

    else if (message.identifier == CANID_IO_STEPPER_HOME && message.length == 5) {
        uint8_t stepper_id = message.data[0];
        int16_t max_relative_steps_before_error = message.data[1] << 8 | message.data[2];
        uint8_t tor_id = message.data[3];
        bool tor_state_to_end_homing = message.data[4] & 1;

        if (stepper_id >= 5 || tor_id >= 16) {
            modm::can::Message err(CANID_IO_STEPPER_ERROR_INVALID_PARAMS, 1);
            err.setExtended(false);
            err.data[0] = stepper_id;
            Can1::sendMessage(err);
            return;
        }

        if (!step_en::read()) {
            modm::can::Message err(CANID_IO_STEPPER_ERROR_NOT_ENABLED, 0);
            err.setExtended(false);
            Can1::sendMessage(err);
            return;
        }

        // TODO make these configurable ? They doesn't fit in standard 8 bytes can frame
        const uint16_t homing_acceleration = 200*8*40;
        const uint16_t homing_max_velocity = 200*8*2;
        
        MotionStartResult res = steppers[stepper_id]->home(max_relative_steps_before_error, homing_acceleration, homing_max_velocity, tor_id, tor_state_to_end_homing);

        if (res == MotionStartResult::Error_InvalidParams) {
            modm::can::Message err(CANID_IO_STEPPER_ERROR_INVALID_PARAMS, 1);
            err.setExtended(false);
            err.data[0] = stepper_id;
            Can1::sendMessage(err);
        } else if (res == MotionStartResult::Error_AlreadyInMotion) {
            modm::can::Message err(CANID_IO_STEPPER_ERROR_MOTION_IN_PROGRESS, 1);
            err.setExtended(false);
            err.data[0] = stepper_id;
            Can1::sendMessage(err);
        } else if (res == MotionStartResult::AlreadyDone) {
            modm::can::Message starting(CANID_IO_STEPPER_HOME_STARTING, 1);
            starting.setExtended(false);
            starting.data[0] = stepper_id;
            Can1::sendMessage(starting);

            modm::can::Message succeeded(CANID_IO_STEPPER_HOME_SUCCEEDED, 1);
            succeeded.setExtended(false);
            succeeded.data[0] = stepper_id;
            Can1::sendMessage(succeeded);
        } else if (res == MotionStartResult::Started) {
            modm::can::Message starting(CANID_IO_STEPPER_HOME_STARTING, 1);
            starting.setExtended(false);
            starting.data[0] = stepper_id;
            Can1::sendMessage(starting);
        }
    }

    else if (message.identifier == CANID_IO_STEPPER_GOTO && message.length == 8) {
        uint8_t stepper_id = message.data[0];
        int16_t absolute_steps = (message.data[1] << 8) | message.data[2];
        uint32_t acceleleration = (message.data[3] << 16) | (message.data[4] << 8) | message.data[5];
        uint16_t max_velocity = (message.data[6] << 8) | message.data[7];

        if (stepper_id >= 5) {
            modm::can::Message err(CANID_IO_STEPPER_ERROR_INVALID_PARAMS, 1);
            err.setExtended(false);
            err.data[0] = stepper_id;
            Can1::sendMessage(err);
            return;
        }

        if (!step_en::read()) {
            modm::can::Message err(CANID_IO_STEPPER_ERROR_NOT_ENABLED, 0);
            err.setExtended(false);
            Can1::sendMessage(err);
            return;
        }

        MotionStartResult res = steppers[stepper_id]->goto_absolute(absolute_steps, acceleleration, max_velocity);

        if (res == MotionStartResult::Error_InvalidParams) {
            modm::can::Message err(CANID_IO_STEPPER_ERROR_INVALID_PARAMS, 1);
            err.setExtended(false);
            err.data[0] = stepper_id;
            Can1::sendMessage(err);
        } else if (res == MotionStartResult::Error_AlreadyInMotion) {
            modm::can::Message err(CANID_IO_STEPPER_ERROR_MOTION_IN_PROGRESS, 1);
            err.setExtended(false);
            err.data[0] = stepper_id;
            Can1::sendMessage(err);
        } else if (res == MotionStartResult::AlreadyDone) {
            modm::can::Message starting(CANID_IO_STEPPER_GOTO_STARTING, 1);
            starting.setExtended(false);
            starting.data[0] = stepper_id;
            Can1::sendMessage(starting);

            modm::can::Message finished(CANID_IO_STEPPER_GOTO_FINISHED, 1);
            finished.setExtended(false);
            finished.data[0] = stepper_id;
            Can1::sendMessage(finished);
        } else if (res == MotionStartResult::Started) {
            modm::can::Message starting(CANID_IO_STEPPER_GOTO_STARTING, 1);
            starting.setExtended(false);
            starting.data[0] = stepper_id;
            Can1::sendMessage(starting);
        }
    }
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
    step_en::reset();

    Can1::connect<GpioInputB8::Rx, GpioOutputB9::Tx>(Gpio::InputType::PullUp);
    Can1::initialize<SystemClock, 1_Mbps>(9);
    // filter 0x200 to 0x2FF
    CanFilter::setFilter(0, CanFilter::FIFO0, CanFilter::StandardIdentifier(0x200), CanFilter::StandardFilterMask(0x700));

    modm::PeriodicTimer blinker { 50ms };
    
    bool first_can_alive = true;
    modm::PeriodicTimer can_alive_timer { 1s };
    
    modm::PeriodicTimer status_timer { 200ms };

    while (true) {
        if (blinker.execute()) {
            Board::LedD13::toggle();
        }

        if (can_alive_timer.execute()) {
            modm::can::Message alive(CANID_IO_ALIVE, 1);
            alive.setExtended(false);
            alive.data[0] = first_can_alive;
            Can1::sendMessage(alive);

            first_can_alive = false;
        }

        if (status_timer.execute()) {
            uint16_t tors = read_tors();

            modm::can::Message status(CANID_IO_STATUS, 2);
            status.setExtended(false);
            status.data[0] = tors >> 8;
            status.data[1] = tors & 0xFF;
            Can1::sendMessage(status);
        }

        for (uint8_t stepper_id = 0 ; stepper_id < 5 ; ++stepper_id) {
            MotionStatus status = steppers[stepper_id]->get_status();
            
            if (status == MotionStatus::FlagHomeSucceeded) {
                steppers[stepper_id]->clear_status_flag();

                modm::can::Message msg(CANID_IO_STEPPER_HOME_SUCCEEDED, 1);
                msg.setExtended(false);
                msg.data[0] = stepper_id;
                Can1::sendMessage(msg);
            }

            else if (status == MotionStatus::FlagHomeFailed) {
                steppers[stepper_id]->clear_status_flag();

                modm::can::Message msg(CANID_IO_STEPPER_HOME_FAILED, 1);
                msg.setExtended(false);
                msg.data[0] = stepper_id;
                Can1::sendMessage(msg);
            }

            else if (status == MotionStatus::FlagGotoFinished) {
                steppers[stepper_id]->clear_status_flag();

                modm::can::Message msg(CANID_IO_STEPPER_GOTO_FINISHED, 1);
                msg.setExtended(false);
                msg.data[0] = stepper_id;
                Can1::sendMessage(msg);
            }
        }

        handle_can_messages();
    }

    return 0;
}
