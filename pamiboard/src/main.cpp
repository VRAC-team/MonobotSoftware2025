#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <AccelStepper.h>
#include "tof.hpp"

#define GPIO_ID1 7
#define GPIO_ID2 6
#define GPIO_ID3 42

#define GPIO_LED_ERR 2
#define GPIO_LED_OK 1
#define GPIO_LED_RGB 5 

#define GPIO_NEN 8 //steppers not enable
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


const float WHEEL_CIRCUMFERENCE = 43.15f * PI; // wheel diameter in mm
const float WHEEL_SPACING = 111.2f; // distance between wheels in mm
const int STEPS_PER_REVOLUTION = 200*16;

const int MAX_SPEED = 6000; // MAX 6000 in step/s
const int MAX_ACCEL = 8000; // MAX 8000 in step/s


uint8_t g_board_id = 255;
bool g_debug_skip_wait = false;
bool g_stop_on_sick = true;
bool g_stop_on_tof = true;
uint16_t g_stop_tof_distance = 200;


int8_t g_team = 1; // 1 for yellow, -1 for blue
long g_start_time = 0;

Servo servo1, servo2;
AccelStepper stepper_left(AccelStepper::DRIVER, GPIO_STEP1, GPIO_DIR1);
AccelStepper stepper_right(AccelStepper::DRIVER, GPIO_STEP2, GPIO_DIR2);


bool has_match_ended() {
  unsigned long time_elapsed = millis() - g_start_time;

  if (g_debug_skip_wait) {
    return time_elapsed >= 15000;
  }
  else {
    return time_elapsed >= 100000;
  }
}

bool is_team_side_pressed() {
  return !digitalRead(GPIO_BUTTON);
}

bool is_starter_present() {
  return analogRead(GPIO_STARTER) < 10;
}

int distance_to_steps(float distance_mm) {
  float revolutions = distance_mm / WHEEL_CIRCUMFERENCE;
  int steps = (int)(revolutions * STEPS_PER_REVOLUTION + 0.5);
  return steps;
}

int rotation_to_steps(double angle_deg) {
  float arc_length = M_PI * WHEEL_SPACING * angle_deg / 360.0;
  float revolutions = arc_length / WHEEL_CIRCUMFERENCE;
  int steps = (int)(revolutions * STEPS_PER_REVOLUTION + 0.5);
  return steps;
}

void wait_first_starter_insertion() {
  long last_insertion = 0;

  while (true) {
    digitalWrite(GPIO_LED_ERR, HIGH);
    delay(50);

    // check if starter is present 10 times, separated by 100ms for each check
    if (is_starter_present()) {
      for (uint8_t i = 0 ; i < 10 ; ++i) {
        if (!is_starter_present()) {
          break;
        }
        digitalWrite(GPIO_LED_ERR, HIGH);
        digitalWrite(GPIO_LED_OK, HIGH);
        delay(50);
      }

      if (is_starter_present()) {
        digitalWrite(GPIO_LED_ERR, LOW);
        return;
      }
    }

    digitalWrite(GPIO_LED_ERR, LOW);
    delay(50);
  }
}

void wait_start_or_select_team() {
  neopixelWrite(GPIO_LED_RGB, 200, 170, 0); //default yellow team

  while (true) {
    if (!is_starter_present()) {
      return;
    }

    if (is_team_side_pressed()) {
      if (g_team == 1) {
        g_team = -1;
        neopixelWrite(GPIO_LED_RGB, 0, 0, 200);
      }
      else {
        g_team = 1;
        neopixelWrite(GPIO_LED_RGB, 200, 170, 0);
      }
      delay(200); //debounce
    }

    if (g_debug_skip_wait) {
      digitalWrite(GPIO_LED_OK, LOW);
      digitalWrite(GPIO_LED_ERR, !digitalRead(GPIO_LED_ERR));
    } else {
      digitalWrite(GPIO_LED_OK, !digitalRead(GPIO_LED_OK));
    }
    delay(30);
  }
}

void wait_last_fifteen_seconds() {
  if (g_debug_skip_wait)
    return;

  while (true) {
    if (millis() - g_start_time > 85000) {
      return;
    }

    digitalWrite(GPIO_LED_OK, !digitalRead(GPIO_LED_OK));
    delay(100);
  }
}

void end_match() {
  digitalWrite(GPIO_NEN, HIGH);

  digitalWrite(GPIO_LED_DANCE, HIGH);
  servo1.write(180); //sorti
  servo2.write(0); //sorti
  delay(400);

  // dance
  while (true) {
    digitalWrite(GPIO_LED_DANCE, HIGH);
    servo1.write(0+45);
    servo2.write(180-45);
    delay(1000);

    digitalWrite(GPIO_LED_DANCE, LOW);
    servo1.write(180-45);
    servo2.write(0+45);
    delay(1000);
  }
}

void do_steps(int left_steps, int right_steps) {
  stepper_left.move(left_steps);
  stepper_right.move(right_steps);

  while (stepper_left.isRunning() || stepper_right.isRunning())
  {
    if (has_match_ended()) {
      end_match();
    }

    if (g_stop_on_sick && (!digitalRead(GPIO_TOR1) || !digitalRead(GPIO_TOR2))) {
      continue;
    }

    if (g_stop_on_tof && tof_get_last_measure() < g_stop_tof_distance) {
      continue;
    }

    stepper_left.run();
    stepper_right.run();
  }
}

void do_line(float distance_mm) {
  int steps = distance_to_steps(distance_mm);
  do_steps(steps, -steps);
}

void do_rotate(float angle_deg) {
  bool last_stop_on_sick = g_stop_on_sick;
  bool last_stop_on_tof = g_stop_on_tof;
  g_stop_on_sick = false;
  g_stop_on_tof = false;

  int steps = rotation_to_steps(angle_deg) * g_team;
  do_steps(steps, steps);

  g_stop_on_sick = last_stop_on_sick;
  g_stop_on_tof = last_stop_on_tof;
}

void print_debug_infos()
{
  Serial.printf("----- millis:%ld\n", millis());
  Serial.printf("PAMIBoard build %s %s\n", __DATE__, __TIME__);
  Serial.printf("BOARDID: %d\n", g_board_id);
  Serial.printf("STARTER:%d (ADC:%d)\n", is_starter_present(), analogRead(GPIO_STARTER));
  Serial.printf("TOR1:%d TOR2:%d\n", digitalRead(GPIO_TOR1), digitalRead(GPIO_TOR2));
  Serial.printf("SIDE:%d\n", is_team_side_pressed());
  Serial.printf("TOF:%d\n", tof_get_last_measure());
}

void test_board() {
  while (1) {
    print_debug_infos();

    servo2.write(0);
    servo1.write(0);

    neopixelWrite(GPIO_LED_RGB, 255, 0, 0);
    delay(200);
    neopixelWrite(GPIO_LED_RGB, 0, 255, 0);
    delay(200);
    neopixelWrite(GPIO_LED_RGB, 0, 0, 255);
    delay(200);
    neopixelWrite(GPIO_LED_RGB, 0, 0, 0);
    delay(200);
  
    digitalWrite(GPIO_LED_DANCE, HIGH);
    delay(200);
    digitalWrite(GPIO_LED_DANCE, LOW);
    delay(200);
  
    servo2.write(180);
    servo1.write(180);

    digitalWrite(GPIO_LED_OK, HIGH);
    digitalWrite(GPIO_NEN, LOW);
    stepper_left.move(STEPS_PER_REVOLUTION);
    stepper_right.move(-STEPS_PER_REVOLUTION);
    while (stepper_left.isRunning() || stepper_right.isRunning()) {
      stepper_left.run();
      stepper_right.run();
    }
    stepper_left.move(-STEPS_PER_REVOLUTION);
    stepper_right.move(STEPS_PER_REVOLUTION);
    while (stepper_left.isRunning() || stepper_right.isRunning()) {
      stepper_left.run();
      stepper_right.run();
    }
    digitalWrite(GPIO_NEN, HIGH);
    digitalWrite(GPIO_LED_OK, LOW);
  }

}

void strat_superstar() {
  g_stop_on_sick = false;
  g_stop_on_tof = true;
  do_line(-1250);
  g_stop_on_tof = false;
  g_stop_on_sick = true;
  do_rotate(-90);
  do_line(-130); // homing vers la zone de calcul
  do_line(350);
}

void setup()
{
  Serial.begin(115200);

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

  // init leds
  pinMode(GPIO_LED_ERR, OUTPUT);
  pinMode(GPIO_LED_OK, OUTPUT);
  pinMode(GPIO_LED_RGB, OUTPUT);
  digitalWrite(GPIO_LED_OK, LOW);
  digitalWrite(GPIO_LED_ERR, HIGH);
  neopixelWrite(GPIO_LED_RGB, 0, 0, 0);

  // init steppers
  pinMode(GPIO_STEP1, OUTPUT);
  pinMode(GPIO_STEP2, OUTPUT);
  pinMode(GPIO_DIR1, OUTPUT);
  pinMode(GPIO_DIR2, OUTPUT);
  //pinMode(GPIO_DIAG1, INPUT);
  //pinMode(GPIO_DIAG2, INPUT);
  pinMode(GPIO_NEN, OUTPUT);
  digitalWrite(GPIO_NEN, HIGH);
  stepper_right.setPinsInverted(true);
  stepper_left.setPinsInverted(true);
  
  stepper_left.setMaxSpeed(MAX_SPEED);
  stepper_left.setAcceleration(MAX_ACCEL);
  stepper_right.setMaxSpeed(MAX_SPEED);
  stepper_right.setAcceleration(MAX_ACCEL);
  

  // init servos
  pinMode(GPIO_SERVO1, OUTPUT);
  pinMode(GPIO_SERVO2, OUTPUT);
  servo1.attach(GPIO_SERVO1);
  servo2.attach(GPIO_SERVO2);
  servo1.write(0); //range
  servo2.write(180); //range
  //servo1.write(180); //sorti
  //servo2.write(0); //sorti

  tof_init();

  // test_board();

  g_debug_skip_wait = is_team_side_pressed();

  wait_first_starter_insertion();

  digitalWrite(GPIO_NEN, LOW);

  wait_start_or_select_team();

  g_start_time = millis();
  
  wait_last_fifteen_seconds();

  switch (g_board_id)
  {
    case 1:
      strat_superstar();
      break;
  }
  
  end_match();
}

void loop()
{
  print_debug_infos();

  digitalWrite(GPIO_LED_ERR, HIGH);
  delay(200);

  digitalWrite(GPIO_LED_ERR, LOW);
  delay(200);
}