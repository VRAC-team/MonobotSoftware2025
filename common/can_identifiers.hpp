#ifndef CAN_IDENTIFIERS
#define CAN_IDENTIFIERS

// ============= MOTOR BOARD =============
#define CANID_MOTOR_REBOOT 0x000
// reboot the board

#define CANID_MOTOR_SETPOINT 0x001
// this must be sent periodically at 200Hz, if the motorboard doesn't receive a setpoint in 10ms then it will change state to error
// <i16 pwm_right> <i16 pwm_left>

#define CANID_MOTOR_SETPOINT_RESPONSE 0x002
// sent by motorboard in response to setpoint
// <u16 enc1> <u16 enc2> <i16 us_since_last_setpoint>

#define CANID_MOTOR_SETPOINT_ERROR 0x003
// sent by motorboard when no setpoint received on the last 10ms

#define CANID_MOTOR_RESET_SETPOINT_ERROR 0x004
// reset setpoint error, next setpoint must be sent in less than 10ms or else it will change state to error

#define CANID_MOTOR_STATUS 0x0EE
// sent periodically by motorboard
// <u16 motor1_adc> <u16 motor2_adc>

#define CANID_MOTOR_ALIVE 0x0FF
// sent periodically by motorboard
// <u8 first_alive_since_reboot(bool)> <u8 is_state_in_error(bool)>



// ============= SERVO BOARD =============
#define CANID_SERVO_REBOOT 0x100
// reboot the board

#define CANID_SERVO_ENABLE_POWER 0x101
// enable power supplies: power1 is servos[0-7], power2 is servo[8-15], power3 is leds[0-3]
// <u8 power1_en(bool)> <u8 power2_en(bool)> <u8 power3_en(bool)>

#define CANID_SERVO_WRITE_US 0x102
// <u8 id(0-15)> <u16 microseconds(500-2500)>

#define CANID_SERVO_SET_LED_PATTERN 0x103
// <u8 id(0-3)> <u8 pattern>

#define CANID_SERVO_STATUS 0x1EE
// sent periodically by servoboard
// <u16 power1_adc> <u16 power2_adc> <u16 power3_adc> <u8 bit0:power1_en bit1:power2_en bit2:power3_en>

#define CANID_SERVO_ALIVE 0x1FF
//sent periodically by servoboard
// <u8 first_alive_since_reboot(bool)>


// ============= IO BOARD =============
#define CANID_IO_REBOOT 0x200
// reboot the board

#define CANID_IO_STEPPER_ENABLE 0x201
// enable/disable all steppers
// <u8 enable(bool)>

#define CANID_IO_STEPPER_HOME 0x202
// home a stepper
// <u8 stepper_id(0-4)>
// <i16 max_relative_steps_before_error>
// <u8 tor_id(0-15)>
// <u8 tor_state_to_end_homing(0-1)>

#define CANID_IO_STEPPER_HOME_FAILED 0x203
// sent by ioboard, when HOME could not be done because max_steps is reached and the tor state was not changed to the desired state
// <u8 stepper_id(0-4)>

#define CANID_IO_STEPPER_HOME_SUCCEEDED 0x204
// sent by ioboard, when HOME is done
// <u8 stepper_id(0-4)>

#define CANID_IO_STEPPER_GOTO 0x205
// move a stepper to absolute steps
// <u8 stepper_id(0-4)>
// <i16 absolute_steps>
// <u24 acceleleration>
// <u16 max_velocity>

#define CANID_IO_STEPPER_GOTO_ERROR_MOTION_IN_PROGRESS 0x206
// sent by ioboard, when a stepper is already moving it cannot accept another GOTO
// <u8 stepper_id(0-4)>

#define CANID_IO_STEPPER_GOTO_FINISHED 0x207
// sent by ioboard, when GOTO is done
// <u8 stepper_id(0-4)>

#define CANID_IO_STATUS 0x2EE
// sent periodically by ioboard
// <u16 tors(bitset)>

#define CANID_IO_ALIVE 0x2FF
// sent periodically by ioboard
// <u8 first_alive_since_reboot(bool)>


// ============= VACUUMPUMP BOARD =============
#define CANID_PUMP_REBOOT 0x300
// reboot the board

#define CANID_PUMP_SET 0x301
// <u8 pumps_states(bitset 0-5)>

#define CANID_PUMP_STATUS 0x3EE
// sent periodically by vacuumpumpboard
// <u8 pumps_states(bitset 0-5)> <u8 vacuum_states(bitset 0-5)>

#define CANID_PUMP_ALIVE 0x3FF
// sent periodically by vacuumpumpboard
// <u8 first_alive_since_reboot(bool)>


// ============= RASPI BOARD =============
#define CANID_RASPI_ALIVE = 0x4FF
// sent periodically by raspiboard
// <u8 first_alive_since_reboot(bool)>

#endif