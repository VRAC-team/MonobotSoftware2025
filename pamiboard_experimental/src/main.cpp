#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <cmath> 
// #include "esp_log.h"

#include "tof.hpp"

#define TIMER_DIVIDER 240
#define TIMER_GROUP TIMER_GROUP_0
#define TIMER_INDEX TIMER_0

#define GPIO_ID1 7
#define GPIO_ID2 6
#define GPIO_ID3 42

#define GPIO_LED_ERR 2
#define GPIO_LED_OK 1
#define GPIO_LED_RGB 5

#define GPIO_NEN 8 // steppers not enable
// left stepper
#define GPIO_STEP1 48
#define GPIO_DIR1 16
#define GPIO_DIAG1 47
// right stepper
#define GPIO_STEP2 38
#define GPIO_DIR2 39
#define GPIO_DIAG2 15

#define GPIO_TOR1 41 // right
#define GPIO_TOR2 21 // left

#define GPIO_SERVO1 17
#define GPIO_SERVO2 18

#define GPIO_STARTER 9
#define GPIO_BUTTON 4
#define GPIO_LED_DANCE 14

constexpr float WHEEL_CIRCUMFERENCE = 43.15f * PI; // wheel diameter in mm
constexpr float WHEEL_SPACING = 111.2f; // distance between wheels in mm
constexpr int STEPS_PER_REVOLUTION = 200 * 16;

uint8_t g_board_id = 255;
bool g_debug_skip_wait = false;
bool g_stop_on_sick = true;
bool g_stop_on_tof = true;
uint16_t g_stop_tof_distance = 200;

enum TeamColor {
    BLUE = -1,
    YELLOW = 1,
};
enum TeamColor g_team = TeamColor::BLUE;
long g_start_time = 0;

int distance_to_steps(float distance_mm)
{
    float revolutions = distance_mm / WHEEL_CIRCUMFERENCE;
    int steps = (int)(revolutions * STEPS_PER_REVOLUTION + 0.5);
    return steps;
}

float steps_to_distance(int steps)
{
    float revolutions = static_cast<float>(steps) / STEPS_PER_REVOLUTION;
    float distance_mm = revolutions * WHEEL_CIRCUMFERENCE;
    return distance_mm;
}

int rotation_deg_to_steps(double angle_deg)
{
    float arc_length = M_PI * WHEEL_SPACING * angle_deg / 360.0;
    float revolutions = arc_length / WHEEL_CIRCUMFERENCE;
    int steps = (int)(revolutions * STEPS_PER_REVOLUTION + 0.5);
    return steps;
}

int rotation_rad_to_steps(double angle_rad)
{
    double arc_length = (WHEEL_SPACING / 2.0) * angle_rad;
    double revolutions = arc_length / WHEEL_CIRCUMFERENCE;
    int steps = static_cast<int>(revolutions * STEPS_PER_REVOLUTION + 0.5);
    return steps;
}

float normalize_theta_deg(float theta_deg) {
    while (theta_deg > 180.0f) {
        theta_deg -= 360.0f;
    }

    while (theta_deg < -180.0f) {
        theta_deg += 360.0f;
    }

    return theta_deg;
}

float normalize_theta_rad(float theta_rad) {
    while (theta_rad > M_PI) {
        theta_rad -= M_TWOPI;
    }

    while (theta_rad < -M_PI) {
        theta_rad += M_TWOPI;
    }

    return theta_rad;
}

void IRAM_ATTR timer_isr(void* arg);

enum class MotionState : uint8_t {
    Idle,
    IsDoingLine,
    IsDoingRotate,
    IsDoingEmergencyBrake,
    FlagRotateFinished,
    FlagLineFinished,
    FlagEmergencyBrakeFinished,
};

class DifferentialStepperRobot {
public:
    DifferentialStepperRobot(uint8_t pin_dir_right, uint8_t pin_dir_left, uint8_t pin_step_right, uint8_t pin_step_left)
    {
        m_pin_dir_right = pin_dir_right;
        m_pin_dir_left = pin_dir_left;
        m_pin_step_right = pin_step_right;
        m_pin_step_left = pin_step_left;

        pinMode(m_pin_dir_right, OUTPUT);
        pinMode(m_pin_dir_left, OUTPUT);
        pinMode(m_pin_step_right, OUTPUT);
        pinMode(m_pin_step_left, OUTPUT);
        digitalWrite(m_pin_dir_right, LOW);
        digitalWrite(m_pin_dir_left, LOW);
        digitalWrite(m_pin_step_right, LOW);
        digitalWrite(m_pin_step_left, LOW);

        m_state = MotionState::Idle;
        m_total_steps_counter = 0;
        m_steps_remaining = 0;

        m_brake_deceleration = STEPS_PER_REVOLUTION*300;
        m_brake_deceleration /= 1000000.0f; // convert this to steps/us

        m_emergency_on_sick = false;
    }

    MotionState goto_xy(int32_t x, int32_t y, enum TeamColor team, bool emergency_on_sick, uint32_t acceleration = STEPS_PER_REVOLUTION*60, uint32_t max_velocity = STEPS_PER_REVOLUTION*30) {
        if (m_state != MotionState::Idle) {
            return m_state;
        }

        if (team == TeamColor::YELLOW) {
            x = -x;
        }

        int32_t dx = x - m_odo_x;
        int32_t dy = y - m_odo_y;
        float theta_err_rad = normalize_theta_rad(std::atan2(dy, dx)) - m_odo_theta_rad;
        float dist_error = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

        // look at xy
        rotate_relative(theta_err_rad, acceleration, max_velocity);
        wait_rotate_finished();

        // line to xy
        line(dist_error, emergency_on_sick, acceleration, max_velocity);
        return wait_line_finished();
    }

    bool rotate_relative(double rad, uint32_t acceleration = STEPS_PER_REVOLUTION*60, uint32_t max_velocity = STEPS_PER_REVOLUTION*30)
    {
        m_total_steps_counter = 0;
        m_setpoint_theta_rad = rad;

        if (m_state != MotionState::Idle) {
            return false;
        }

        if (rad == 0) {
            m_state = MotionState::FlagRotateFinished;
            return true;
        }

        if (rad > 0) {
            digitalWrite(m_pin_dir_right, HIGH);
            digitalWrite(m_pin_dir_left, HIGH);
        } else {
            digitalWrite(m_pin_dir_right, LOW);
            digitalWrite(m_pin_dir_left, LOW);
        }

        m_state = MotionState::IsDoingRotate;
        int32_t steps = rotation_rad_to_steps(rad);
        return compute_trapezoid_profile_and_setup_timer(steps, acceleration, max_velocity);
    }

    bool line(double distance, bool emergency_on_sick, uint32_t acceleration = STEPS_PER_REVOLUTION*60, uint32_t max_velocity = STEPS_PER_REVOLUTION*30)
    {
        m_total_steps_counter = 0;
        m_setpoint_distance = distance;

        if (m_state != MotionState::Idle) {
            return false;
        }

        if (distance == 0) {
            m_state = MotionState::FlagLineFinished;
            return true;
        }

        if (distance > 0) {
            digitalWrite(m_pin_dir_right, LOW);
            digitalWrite(m_pin_dir_left, HIGH);
        } else {
            digitalWrite(m_pin_dir_right, HIGH);
            digitalWrite(m_pin_dir_left, LOW);
        }

        m_state = MotionState::IsDoingLine;
        m_emergency_on_sick = emergency_on_sick;

        int steps = distance_to_steps(distance);
        return compute_trapezoid_profile_and_setup_timer(steps, acceleration, max_velocity);
    }

    void wait_rotate_finished()
    {
        while (true) {
            if (m_state == MotionState::FlagRotateFinished) {
                m_state = MotionState::Idle;
                Serial.println("rotate finished");
                return;
            }
        }
    }

    MotionState wait_line_finished()
    {
        while (true) {
            if (m_state == MotionState::FlagLineFinished) {
                m_state = MotionState::Idle;
                Serial.println("line finished");
                return MotionState::FlagLineFinished;
            }
            else if (m_state == MotionState::FlagEmergencyBrakeFinished) {
                m_state = MotionState::Idle;
                Serial.println("emergency brake finished");
                return MotionState::FlagEmergencyBrakeFinished;
            }
        }
    }

    void isr_step()
    {
        if (m_state == MotionState::IsDoingLine) {
            if (m_emergency_on_sick && (!digitalRead(GPIO_TOR1) || !digitalRead(GPIO_TOR2))) {
                emergency_brake();
            }
        }

        m_steps_remaining--;
        m_total_steps_counter += m_direction;

        digitalWrite(m_pin_step_left, HIGH);
        digitalWrite(m_pin_step_right, HIGH);
        // according to datasheet, step high time should be at least 1.9us ish if i understood correctly, another timer could be used to create this delay
        // but with my testings, this is working fine without any delay
        digitalWrite(m_pin_step_left, LOW);
        digitalWrite(m_pin_step_right, LOW);

        if (m_steps_remaining == 0) {
            if (m_state == MotionState::IsDoingRotate) {
                m_state = MotionState::FlagRotateFinished;
                m_odo_theta_rad += m_setpoint_theta_rad;
                Serial.println("ISR FlagRotateFinished");
                Serial.print("theta_rad:");
                Serial.println(m_odo_theta_rad);
                return;
            }
            if (m_state == MotionState::IsDoingLine) {
                m_state = MotionState::FlagLineFinished;
                m_odo_x += m_setpoint_distance * std::cos(m_odo_theta_rad);
                m_odo_y += m_setpoint_distance * std::sin(m_odo_theta_rad);
                Serial.println("ISR FlagLineFinished");
                Serial.print("x:");
                Serial.println(m_odo_x);
                Serial.print("y:");
                Serial.println(m_odo_y);
                Serial.print("theta_rad:");
                Serial.println(m_odo_theta_rad);
                return;
            }
        }

        if (m_state == MotionState::IsDoingEmergencyBrake) {
            m_current_velocity -= m_brake_deceleration * m_current_step_period_us;
            if (m_current_velocity <= 0) {
                m_state = MotionState::FlagEmergencyBrakeFinished;
                double distance = steps_to_distance(m_total_steps_counter);
                m_odo_x += distance * std::cos(m_odo_theta_rad);
                m_odo_y += distance * std::sin(m_odo_theta_rad);
            }
        } else {
            if (m_steps_remaining <= m_remaining_steps_decel_phase) {
                m_current_velocity -= m_acceleration * m_current_step_period_us;
            } else if (m_steps_remaining <= m_remaining_steps_const_phase) {
                m_current_velocity = m_max_velocity;
            } else if (m_steps_remaining <= m_remaining_steps_accel_phase) {
                m_current_velocity += m_acceleration * m_current_step_period_us;
                if (m_current_velocity > m_max_velocity) {
                    m_current_velocity = m_max_velocity;
                }
            }
        }

        // current_step_period_us doesn't account for the time to get there on this ISR, therefore it is a little bit more than it should be but not by much (TODO measure)
        m_current_step_period_us = 1000000.0f / m_current_velocity;

        uint64_t timer_counter_value;
        timer_set_counter_value(TIMER_GROUP, TIMER_INDEX, 0);
        timer_set_alarm_value(TIMER_GROUP, TIMER_INDEX, m_current_step_period_us);
        TIMERG0.hw_timer[TIMER_INDEX].config.tn_alarm_en = TIMER_ALARM_EN;
    }

    bool emergency_brake()
    {
        if (m_state != MotionState::IsDoingLine) {
            return false;
        }
        m_state = MotionState::IsDoingEmergencyBrake;
        m_emergency_brake_steps = (m_current_velocity * m_current_velocity) / (2.0f * m_brake_deceleration);
        m_steps_remaining = m_emergency_brake_steps;
        return true;
    }

private:
    uint8_t m_pin_dir_right;
    uint8_t m_pin_dir_left;
    uint8_t m_pin_step_right;
    uint8_t m_pin_step_left;

    // this odo is the last known position before a motion, and is updated when a motion is completed or emergency brake
    double m_odo_x;
    double m_odo_y;
    double m_odo_theta_rad;

    float m_setpoint_theta_rad;
    float m_setpoint_distance;

    // internal state
    volatile MotionState m_state;
    volatile int32_t m_total_steps_counter; // this can differ from steps_remaining, because if a line got emergency brake the steps_remaining is recalculated
    volatile uint32_t m_steps_remaining;
    volatile float m_current_velocity;
    volatile uint32_t m_current_step_period_us;
    float m_max_velocity;
    float m_acceleration;
    uint32_t m_accel_steps;
    uint32_t m_const_steps;
    uint32_t m_decel_steps;
    uint32_t m_remaining_steps_accel_phase;
    uint32_t m_remaining_steps_const_phase;
    uint32_t m_remaining_steps_decel_phase;
    int8_t m_direction; // +1 or -1

    float m_brake_deceleration;
    volatile bool m_emergency_on_sick;
    volatile uint32_t m_emergency_brake_steps;


    bool compute_trapezoid_profile_and_setup_timer(int32_t steps, uint32_t acceleration, uint32_t max_velocity)
    {
        if (steps == 0 || acceleration == 0 || max_velocity == 0) {
            return false;
        }

        int32_t delta = steps;
        if (delta > 0) {
            m_direction = 1;
        } else {
            m_direction = -1;
        }

        uint32_t total_steps = abs(delta);

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
        m_acceleration /= 1000000.0f; // convert motor accel to steps/microseconds^2
        m_remaining_steps_decel_phase = m_decel_steps;
        m_remaining_steps_const_phase = m_decel_steps + m_const_steps;
        m_remaining_steps_accel_phase = m_decel_steps + m_const_steps + m_accel_steps;

        timer_config_t config = {
            .alarm_en = TIMER_ALARM_EN,
            .counter_en = TIMER_PAUSE,
            .intr_type = TIMER_INTR_LEVEL,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = TIMER_AUTORELOAD_DIS,
            .divider = TIMER_DIVIDER,
        };
        timer_init(TIMER_GROUP, TIMER_INDEX, &config);
        timer_set_counter_value(TIMER_GROUP, TIMER_INDEX, 0);
        timer_set_alarm_value(TIMER_GROUP, TIMER_INDEX, m_current_step_period_us);
        timer_enable_intr(TIMER_GROUP, TIMER_INDEX);
        timer_isr_register(TIMER_GROUP, TIMER_INDEX, timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
        timer_start(TIMER_GROUP, TIMER_INDEX);

        return true;
    }
};

DifferentialStepperRobot robot(GPIO_DIR1, GPIO_DIR2, GPIO_STEP1, GPIO_STEP2);

void IRAM_ATTR timer_isr(void* arg)
{
    TIMERG0.int_clr_timers.t0_int_clr = 1;
    robot.isr_step();
}

void setup()
{
    // init leds
    pinMode(GPIO_LED_ERR, OUTPUT);
    pinMode(GPIO_LED_OK, OUTPUT);
    pinMode(GPIO_LED_RGB, OUTPUT);
    digitalWrite(GPIO_LED_OK, LOW);
    digitalWrite(GPIO_LED_ERR, HIGH);
    neopixelWrite(GPIO_LED_RGB, 0, 0, 0);

    pinMode(GPIO_TOR1, INPUT);
    pinMode(GPIO_TOR2, INPUT);
    pinMode(GPIO_STARTER, INPUT);
    pinMode(GPIO_BUTTON, INPUT_PULLUP);
    pinMode(GPIO_LED_DANCE, OUTPUT);

    // init boardid
    pinMode(GPIO_ID1, INPUT_PULLUP);
    pinMode(GPIO_ID2, INPUT_PULLUP);
    pinMode(GPIO_ID3, INPUT_PULLUP);
    uint8_t id1 = !digitalRead(GPIO_ID1);
    uint8_t id2 = !digitalRead(GPIO_ID2);
    uint8_t id3 = !digitalRead(GPIO_ID3);
    g_board_id = id1 << 0 | id2 << 1 | id3 << 2;

    // init steppers
    pinMode(GPIO_STEP1, OUTPUT);
    pinMode(GPIO_STEP2, OUTPUT);
    pinMode(GPIO_DIR1, OUTPUT);
    pinMode(GPIO_DIR2, OUTPUT);
    pinMode(GPIO_NEN, OUTPUT);
    digitalWrite(GPIO_NEN, HIGH);

    Serial.begin(115200);

    // PROCEDURE FOR SERIAL DEBUGGING
    // 1/ Upload and Monitor
    // 2/ Ctrl-C on the Monitor, it should try to reconnect
    // 3/ Press reboot button on board
    
    // while (!Serial) {}
    // digitalWrite(GPIO_LED_ERR, LOW);

    tof_init();

    digitalWrite(GPIO_NEN, LOW);

    const uint32_t ACCEL = STEPS_PER_REVOLUTION*80;
    const uint32_t SPEED = STEPS_PER_REVOLUTION*40;

    robot.line(200, false, ACCEL, SPEED);
    robot.wait_line_finished();
    robot.rotate_relative(50.0 * DEG_TO_RAD, ACCEL, SPEED);
    robot.wait_rotate_finished();
    robot.line(350, false, ACCEL, SPEED);
    robot.wait_line_finished();
    robot.rotate_relative(-50.0 * DEG_TO_RAD, ACCEL, SPEED);
    robot.wait_rotate_finished();

    MotionState res = robot.goto_xy(1500, 300, g_team, true, STEPS_PER_REVOLUTION*100, STEPS_PER_REVOLUTION*30);

    if (res == MotionState::FlagEmergencyBrakeFinished) {
        neopixelWrite(GPIO_LED_RGB, 255, 0, 0);
        robot.rotate_relative(M_PI_2);
        robot.wait_rotate_finished();
        robot.line(200, false);
        robot.wait_line_finished();

        MotionState res = robot.goto_xy(1500, 300, g_team, false, STEPS_PER_REVOLUTION*50, STEPS_PER_REVOLUTION*25);
    }

    digitalWrite(GPIO_NEN, HIGH);
}

void loop()
{
}