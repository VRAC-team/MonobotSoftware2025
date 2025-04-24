#include "tof.hpp"
#include "VL53L0X.h"
#include <Arduino.h>

static VL53L0X tof;
static uint16_t sensor1 = 0;
static long previousTime = 0;

bool tof_init()
{
  Wire.setPins(PIN_TOF_SDA, PIN_TOF_SCL);
  Wire.begin();

  // Disable/reset all sensors by driving their XSHUT pins low.
  pinMode(PIN_TOF_XSHUT, OUTPUT);
  digitalWrite(PIN_TOF_XSHUT, LOW);
  delay(10);

  // Stop driving this sensor's XSHUT low. This should allow the carrier
  // board to pull it high. (We do NOT want to drive XSHUT high since it is
  // not level shifted.) Then wait a bit for the sensor to start up.
  pinMode(PIN_TOF_XSHUT, INPUT);
  delay(10);

  tof.setBus(&Wire);
  tof.setTimeout(500);
  if (!tof.init())
  {
    Serial.println("Failed to detect and initialize tof");
    return false;
  }

  // Each sensor must have its address changed to a unique value other than
  // the default of 0x29 (except for the last one, which could be left at
  // the default). To make it simple, we'll just count up from 0x2A.
  tof.setAddress(0x2A);

  tof.startContinuous();
  tof.setMeasurementTimingBudget(2000); // reduce timing budget to 20 ms (default is about 33 ms)
  previousTime = millis();
  
  return true;
}

bool tof_update()
{
  if (millis() - previousTime > READ_TIME_PERIOD_MS)
  {
    previousTime = millis();
    sensor1 = tof.readRangeContinuousMillimeters();

    if (tof.timeoutOccurred())
      return false;
  }
  return true;
}

uint16_t tof_get() {
    return sensor1;
}