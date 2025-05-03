#ifndef CAN_IDENTIFIERS
#define CAN_IDENTIFIERS

// ============= MOTOR BOARD =============
#define CANID_MOTOR_REBOOT 0x000
// reboot the board

#define CANID_MOTOR_STATE_ERROR 0x001
// sent by motorboard when no PWM_WRITE is received for 10ms

#define CANID_MOTOR_PWM_WRITE 0x002
// must be sent periodically at 200Hz, if the motorboard doesn't receive a PWM_WRITE for 10ms then it will change state to ERROR
// <i16 pwm_right>
// <i16 pwm_left>

#define CANID_MOTOR_STATUS 0x003
// sent by motorboard
// if the state is nominal, then it will sent immediately after received the PWM_WRITE (this can be used to measure latency)
// if the state is in error, then it will be sent periodically at 200Hz
// <u8 state_error(bool)>
// <u16 enc1>
// <u16 enc2>

#define CANID_MOTOR_RESET_STATE_ERROR 0x004
// reset error, next PWM_WRITE must be sent in less than 10ms or else it will change state to ERROR

#define CANID_MOTOR_ALIVE 0x0FF
// sent periodically by motorboard
// <u8 first_alive_since_reboot(bool)>



// ============= SERVO BOARD =============
#define CANID_SERVO_REBOOT 0x100
// reboot the board

#define CANID_SERVO_ERROR_INVALID_PARAMS 0x101
// send by the servoboard when invalid: stepper_id, led_id

#define CANID_SERVO_ERROR_NOT_ENABLED 0x102
// send by the servoboard when WRITE_US or SET_LED_PATTERN but pwoer is not enabled for the current servo_id/led_id
// <u8 invalid_servo_id/invalid_led_id>

#define CANID_SERVO_ENABLE_POWER 0x103
// enable/disable power supplies
// power1 is servos[0-7], power2 is servo[8-15], power3 is leds[0-3]
// <u8 power1_en(bool)>
// <u8 power2_en(bool)>
// <u8 power3_en(bool)>

#define CANID_SERVO_WRITE_US 0x104
// <u8 servo_id(0-15)>
// <u16 servo_us(500-2500)>

#define CANID_SERVO_SET_LED_PATTERN 0x105
// <u8 led_id(0-3)>
// <u8 led_pattern>

#define CANID_SERVO_STATUS 0x1EE
// sent periodically by servoboard
// <u16 power1_adc>
// <u16 power2_adc>
// <u16 power3_adc>
// <u8 bit0:power1_en bit1:power2_en bit2:power3_en>

#define CANID_SERVO_ALIVE 0x1FF
//sent periodically by servoboard
// <u8 first_alive_since_reboot(bool)>


// ============= IO BOARD =============
#define CANID_IO_REBOOT 0x200
// reboot the board

#define CANID_IO_STEPPER_ERROR_DISABLED_DURING_MOTION 0x201
// sent by ioboard, when there was a HOME or GOTO in progress but then received a DISABLE. All motions were canceled
// <u8 steppers_that_were_doing_motion(bitset)>

#define CANID_IO_STEPPER_ERROR_NOT_ENABLED 0x202
// sent by ioboard, when a new HOME or GOTO is received but the steppers are not enabled

#define CANID_IO_STEPPER_ERROR_MOTION_IN_PROGRESS 0x203
// sent by ioboard, when a new HOME or GOTO is received but the stepper_id is already doin HOME or GOTO
// <u8 stepper_id(0-4)>

#define CANID_IO_STEPPER_ERROR_INVALID_PARAMS 0x204
// send by ioboard, when a new HOME or GOTO is received with invalid parameters
// possible invalid parameters: acceleration=0, maxvelocity=0, stepper_id>5, tor_id>15

#define CANID_IO_STEPPER_ENABLE 0x205
// enable/disable all steppers drivers
// if disabling and there is a HOME or GOTO in progress, cancel all motions. ioboard will send a ERROR_DISABLED_DURING_MOTION
// <u8 enable(bool)>

#define CANID_IO_STEPPER_HOME 0x206
// home a stepper
// if the steppers are not enabled, ioboard will send a ERROR_NOT_ENABLED
// if the stepper_id is already doin HOME or GOTO, then ioboard will send a ERROR_MOTION_IN_PROGRESS
// if the parameters are invalid (acceleration=0 or maxvelocity=0), then ioboard will send a ERROR_INVALID_PARAMS
// when starting, ioboard will send HOME_STARTING
// if max_relative_steps_before_error, ioboard will send HOME_SUCCEEDED
// when tor_state_to_end_homing state is matched, current_position is reset to 0 and ioboard will send HOME_SUCCEEDED
// when max_relative_steps_before_error is reached without tor state, ioboard will send HOME_FAILED
// <u8 stepper_id(0-4)>
// <i16 max_relative_steps_before_error>
// <u8 tor_id(0-15)>
// <u8 tor_state_to_end_homing(0-1)>

#define CANID_IO_STEPPER_HOME_STARTING 0x207
// send by ioboard, when a new HOME is received and starting succesfully
// <u8 stepper_id(0-4)>

#define CANID_IO_STEPPER_HOME_FAILED 0x208
// sent by ioboard, when HOME could not be done because max_relative_steps_before_error is reached and the tor state was not changed to the desired state
// <u8 stepper_id(0-4)>

#define CANID_IO_STEPPER_HOME_SUCCEEDED 0x209
// sent by ioboard, when HOME is done
// <u8 stepper_id(0-4)>

#define CANID_IO_STEPPER_GOTO 0x20A
// move a stepper to absolute steps
// if the stepper_id is already doin HOME or GOTO, then ioboard will send a ERROR_MOTION_IN_PROGRESS
// if the steppers are not enabled, ioboard will send a ERROR_NOT_ENABLED
// if the parameters are invalid (acceleration=0 or maxvelocity=0), then ioboard will send a ERROR_INVALID_PARAMS
// when the motion starts, ioboard will send a GOTO_STARTING
// when the motion is done, ioboard will send a GOTO_FINISHED
// <u8 stepper_id(0-4)>
// <i16 absolute_steps>
// <u24 acceleleration>
// <u16 max_velocity>

#define CANID_IO_STEPPER_GOTO_STARTING 0x20B
// send by ioboard, when a new GOTO is received and starting succesfully
// <u8 stepper_id(0-4)>

#define CANID_IO_STEPPER_GOTO_FINISHED 0x20C
// sent by ioboard, when GOTO has finished
// <u8 stepper_id(0-4)>

#define CANID_IO_STATUS 0x2EE
// sent periodically by ioboard
// <u8 enable(bool)>
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
#define CANID_RASPI_ALIVE 0x4FF
// sent periodically by raspiboard
// <u8 first_alive_since_reboot(bool)>
// <u8 error(bool)>

#endif