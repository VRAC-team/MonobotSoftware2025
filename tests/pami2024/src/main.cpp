#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <AccelStepper.h>
#include "tof.hpp"
#include "boardid.hpp"

#define WHEEL_DIAMETER_MM 39.5f
#define WHEEL_DISTANCE_MM 111.2f // distance between wheels
// #define WHEEL_LEFT_COEF 0.995f
#define STEPS_PER_REVOLUTION 200
const float circumferenceMM = WHEEL_DIAMETER_MM * PI;

float wheel_distance_correction[6] = {111.2f, 111.2f * 0.9955f, 111.2f * 1.01, 111.2f, 111.2f, 111.2f};
float max_speed_right_correction[6] = {1.0f, 1.0f, 0.97f, 1.0f, 1.0f, 1.0f}; //NO NEGATIVE NAVLUES THERE
float max_speed_left_correction[6] = {1.0f, 1.0f, 1.04f, 1.0f, 1.0f, 1.0f}; //NO NEGATIVE VALUES THERE

uint8_t current_microstep = 8; // board id dependent
int8_t current_direction = 1;  // 1 for forward, -1 for backward

int8_t current_side = 1; // 1 for blue, -1 for yellow
bool avoidance_active = true;
long start_time = 0;
bool debug_mode = false;

float max_speed[6] = {6000, 6000, 25000, 6000, 8000, 10000};
float max_accel[6] = {8000, 8000, 15000, 8000, 10000, 12000};

#define MAX_SPEED 6000 // MAX 6000 in step/s
#define MAX_ACCEL 8000 // MAX 8000 in step/s

#define RGB_BRIGHTNESS 32
#define START_ADC_THRESHOLD 4040
#define TOF_STOP_DISTANCE 150

// The board contains these hardware fix (IO35, IO36, and IO37 not usable because of ESP32-S3-WROOM-1-N16R8 Octal SPI PSRAM)
// STRAP OLD TOF_SDA (IO16) TO M1_DIR
// STRAP OLD TOF_SCL (IO15) TO M2_DIAG
// STRAP OLD SERVO2 (IO18) TO M_EN
// STRAP OLD IR_TX1 (IO9) TO STARTER
// Using old IR_RX MicroMatch connector as TOF connector

#define PIN_RED 2
#define PIN_GREEN 1
#define PIN_RGB 5

// Board Id 1 is using TMC2226
// Board Id [2:3] are using A4988
// Board Id [4:6] are using MP6500
// Stepper 14HS13-0804S, Imax = 0.8A
// Vref for A4988 = Imax * 8 * 0.1 = 0.64V
// Vref for MP6500 = Imax / 3.5 = 0.23V
// Vref for TMC2226 = Imax = 0.8V
#define PIN_STEP1 48 // left
#define PIN_STEP2 38 // right
#define PIN_DIR1 16  // left
#define PIN_DIR2 39  // right
#define PIN_DIAG1 47 // left
#define PIN_DIAG2 15 // right
#define PIN_NEN 18   // not enable (active at 0), common to right and left

#define PIN_TOR1 41 // right
#define PIN_TOR2 21 // left

#define PIN_SERVO 17

#define PIN_STARTER 9

#define PIN_BUTTON 4

#define PIN_KABOOM 14

Servo servo;
AccelStepper left(AccelStepper::DRIVER, PIN_STEP1, PIN_DIR1);
AccelStepper right(AccelStepper::DRIVER, PIN_STEP2, PIN_DIR2);

void init_steppers()
{
  pinMode(PIN_STEP1, OUTPUT);
  pinMode(PIN_STEP2, OUTPUT);
  pinMode(PIN_DIR1, OUTPUT);
  pinMode(PIN_DIR2, OUTPUT);
  pinMode(PIN_DIAG1, INPUT);
  pinMode(PIN_DIAG2, INPUT);
  pinMode(PIN_NEN, OUTPUT);

  digitalWrite(PIN_NEN, HIGH);

  left.setMaxSpeed(MAX_SPEED);
  left.setAcceleration(MAX_ACCEL);
  right.setMaxSpeed(MAX_SPEED);
  right.setAcceleration(MAX_ACCEL);

  current_microstep = 8;
  switch (boardid_get())
  {
  case 3:
    current_direction = -1; // TMC2226 dir is inverted
    break;
  default:
    current_direction = 1;
    break;
  }
}

void init_servo()
{
  pinMode(PIN_SERVO, OUTPUT);
  servo.attach(PIN_SERVO);
  servo.write(0);
  delay(200);
  servo.detach(); // detach servo because one arm is drawing current because going into the body C
}

void arm_expand(int deg)
{
  servo.attach(PIN_SERVO);
  servo.write(deg);
}

void init_leds()
{
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_RGB, OUTPUT);

  digitalWrite(PIN_GREEN, LOW);
  digitalWrite(PIN_RED, HIGH);
  neopixelWrite(PIN_RGB, 0, 0, 0);
}

bool is_side_pressed()
{
  return !digitalRead(PIN_BUTTON);
}

bool is_starter_present()
{
  return analogRead(PIN_STARTER) > START_ADC_THRESHOLD;
}

long convertDistToStep(float _dist)
{
  float revolutions = _dist / circumferenceMM;
  return static_cast<long>(revolutions * STEPS_PER_REVOLUTION * current_microstep * current_direction);
}

long convertAngleToStep(float _angle)
{
  float angleRadians = _angle * (PI / 180.0f);
  float arcLength = (wheel_distance_correction[boardid_get() - 1] / 2.0f) * angleRadians;
  float revolutions = arcLength / circumferenceMM;
  return static_cast<long>(revolutions * STEPS_PER_REVOLUTION * current_microstep * current_direction);
}

void end_match()
{
  digitalWrite(PIN_NEN, HIGH);

  while (true)
  {
    digitalWrite(PIN_GREEN, LOW);
    delay(300);
    digitalWrite(PIN_GREEN, HIGH);
    delay(300);
  }
}

bool has_match_ended()
{
  if (debug_mode)
  {
    return false;
  }

  return (millis() - start_time) >= (100 * 1000);
}

void rotate(float deg)
{
  left.setMaxSpeed(max_speed[boardid_get() - 1] / 2.0);
  left.setAcceleration(max_accel[boardid_get() - 1] / 2.0);
  right.setMaxSpeed(max_speed[boardid_get() - 1] / 2.0);
  right.setAcceleration(max_accel[boardid_get() - 1] / 2.0);

  left.move(convertAngleToStep(deg) * current_side);
  right.move(convertAngleToStep(deg) * current_side);
  while (left.isRunning() || right.isRunning())
  {
    if (has_match_ended())
    {
      end_match();
    }

    left.run();
    right.run();
  }
}
long tempDistance_right, tempDistance_left;

void emergency_stop()
{
  tempDistance_right = right.distanceToGo();
  tempDistance_left = left.distanceToGo();

  right.move(100);
  left.move(-100);

  while (right.isRunning() || left.isRunning())
  {
    left.run();
    right.run();
  }
}

void line(float mm)
{
  left.setMaxSpeed(max_speed[boardid_get() - 1] * max_speed_left_correction[boardid_get() - 1]);
  left.setAcceleration(max_accel[boardid_get() - 1]);
  right.setMaxSpeed(max_speed[boardid_get() - 1] * max_speed_right_correction[boardid_get() - 1]);
  right.setAcceleration(max_accel[boardid_get() - 1]);

  left.move(convertDistToStep(mm));
  right.move(-convertDistToStep(mm));
  while (left.isRunning() || right.isRunning())
  {
    tof_update();

    if (has_match_ended())
    {
      end_match();
    }

    // BLOCKING IF ROBOT DETECTED
    if (avoidance_active)
    {
      if (!digitalRead(PIN_TOR1) || !digitalRead(PIN_TOR2) || tof_get() < TOF_STOP_DISTANCE)
      {
        // emergency_stop();
        // delay(1000);
        continue;
      }
    }

    left.run();
    right.run();
  }
}

void wait_for_starter_insertion()
{
  uint32_t i = 0;
  uint32_t last = millis();
  while (true)
  {
    uint32_t current = millis();
    if (current - last > 300)
    {
      if (i % 2 == 0)
      {
        neopixelWrite(PIN_RGB, 255, 0, 255);
      }
      else
      {
        neopixelWrite(PIN_RGB, 0, 0, 0);
      }
      i++;
      last = current;
    }

    if (is_starter_present())
    {
      delay(1000);
      break;
    }
  }
}

void test_roomba()
{
  neopixelWrite(PIN_RGB, 0, RGB_BRIGHTNESS, 0);
  digitalWrite(PIN_NEN, LOW);

  bool inverted = true;

  long last_dir_change = millis();
  while (true)
  {
    long diff = millis() - last_dir_change;
    bool right = !digitalRead(PIN_TOR1);
    bool left = !digitalRead(PIN_TOR2);

    if (!right && !left)
    { // AVANCE
      digitalWrite(PIN_DIR1, LOW);
      digitalWrite(PIN_DIR2, HIGH);
    }
    else if (right && !left && diff > 300)
    { // TOURNE A GAUCHE
      digitalWrite(PIN_DIR1, HIGH);
      digitalWrite(PIN_DIR2, HIGH);
      last_dir_change = millis();
    }
    else if (!right && left && diff > 300)
    { // TOURNE A DROITE
      digitalWrite(PIN_DIR1, LOW);
      digitalWrite(PIN_DIR2, LOW);
      last_dir_change = millis();
    }
    else if (right && left && diff > 300)
    {
      digitalWrite(PIN_DIR1, LOW);
      digitalWrite(PIN_DIR2, LOW);
    }

    digitalWrite(PIN_STEP1, LOW);
    digitalWrite(PIN_STEP2, LOW);
    digitalWrite(PIN_STEP1, HIGH);
    digitalWrite(PIN_STEP2, HIGH);
    delayMicroseconds(300);
  }
}

//HACK FOR DEMO, REVERT ME FOR IDF EPITA
extern uint8_t board_id;

void setup()
{
  Serial.begin(115200);

  pinMode(PIN_TOR1, INPUT);
  pinMode(PIN_TOR2, INPUT);
  pinMode(PIN_STARTER, INPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  if (is_side_pressed())
  {
    debug_mode = true;
  }

  init_leds();
  boardid_init();
  if (board_id == 5) board_id = 6;
  if (board_id == 4) board_id = 5;
  if (board_id == 2) board_id = 4;
  init_steppers();
  init_servo();

  delay(1000); // some time needed for platformio monitor
  Serial.printf("-------------START-------------\n");
  Serial.printf("PAMIBoard build %s %s\n", __DATE__, __TIME__);
  Serial.printf("boardid: %d\n", boardid_get());

  tof_init();

  //test_roomba();

  wait_for_starter_insertion();

  // 10 ROTATE TEST FOR WHEEL DISTANCE PARAMETER
  // digitalWrite(PIN_NEN, LOW);
  // rotate(360 * 100);
  // digitalWrite(PIN_NEN, HIGH);
  // while(true) {}

  // 2METER LINE TEST FOR WHEEL CORRECTION PARAMETER

  // digitalWrite(PIN_NEN, LOW);
  // line(2000);
  // delay(300);
  // rotate(2.5); //HACK TO FIX WHEEL DIFFERENCE SPEED
  // line(-2000);
  // digitalWrite(PIN_NEN, HIGH);
  // while (true)
  // {
  //}

  neopixelWrite(PIN_RGB, 0, 0, 255);

  while (true)
  {
    if (is_side_pressed())
    {
      current_side *= -1; // invert color side

      if (current_side == 1)
      {
        neopixelWrite(PIN_RGB, 0, 0, 255);
      }
      else
      {
        neopixelWrite(PIN_RGB, 255, 255, 0);
      }

      delay(300); // debounce
    }

    if (!is_starter_present())
    {
      start_time = millis();
      digitalWrite(PIN_GREEN, HIGH);
      digitalWrite(PIN_RED, LOW);

      uint32_t i = 0;
      while (true)
      {
        unsigned long current = millis();

        if (current % 100 == 0)
        {
          digitalWrite(PIN_RED, !digitalRead(PIN_RED));
        }

        if (current % 600 == 0)
        {
          neopixelWrite(PIN_RGB, 0, 0, 0);
        }
        else if (current % 300 == 0)
        {
          if (current_side == 1)
          {
            neopixelWrite(PIN_RGB, 0, 0, 255);
          }
          else
          {
            neopixelWrite(PIN_RGB, 255, 255, 0);
          }
        }

        if (millis() - start_time > (90 * 1000) || debug_mode)
        {
          break;
        }

        delay(1);
      }

      break;
    }
  }
}

void print_debug()
{
  Serial.printf("-----\n");
  Serial.printf("STARTER:%d (ADC:%d)\n", is_starter_present(), analogRead(PIN_STARTER));
  Serial.printf("TOR1:%d TOR2:%d\n", digitalRead(PIN_TOR1), digitalRead(PIN_TOR2));
  Serial.printf("SIDE:%d\n", is_side_pressed());
  Serial.printf("TOF:%d\n", tof_get());
}

void test_line_rot_line()
{
  neopixelWrite(PIN_RGB, RGB_BRIGHTNESS, 0, 0);
  line(1000);
  neopixelWrite(PIN_RGB, 0, RGB_BRIGHTNESS, 0);
  rotate(180);
  neopixelWrite(PIN_RGB, RGB_BRIGHTNESS, 0, RGB_BRIGHTNESS);
  line(1000);
  neopixelWrite(PIN_RGB, 0, 0, RGB_BRIGHTNESS);
  rotate(180);
  neopixelWrite(PIN_RGB, 0, 0, 0);
}



void loop()
{
  // tof_update();
  // print_debug();
  digitalWrite(PIN_NEN, LOW);

  switch (boardid_get())
  {
  case 6:
    avoidance_active = false;
    line(950);
    break;
  case 5:
    delay(800);
    avoidance_active = false;
    line(380);
    arm_expand(75);
    rotate(90);
    line(200); // SICK DETECTION DISTANCE
    break;
  case 4:
    delay(1600);
    avoidance_active = false;
    line(760);
    rotate(45);
    line(400);
    arm_expand(45);
    line(500); // SICK DETECTION DISTANCE
    break;
  case 3:
    line(1620);
    rotate(90);
    line(1250);
    arm_expand(90);
    break;
  case 2:
    line(1000);
    if (current_side == 1) { // IF BLUE
      rotate(-105);
    }
    else { // IF YELLOW
      rotate(-98);
    }
    line(1500);
    avoidance_active = false;
    arm_expand(60);
    line(500); // SICK DETECTION DISTANCE
    break;
  }

  end_match();
}