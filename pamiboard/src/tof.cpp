#include "tof.hpp"
#include "VL53L0X.h"
#include <Arduino.h>

static VL53L0X tof;
static volatile uint16_t distance_mm = 0;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t tofTaskHandle;

void tof_task(void *params) {
  while (true) {
    uint16_t dist = tof.readRangeContinuousMillimeters();
    if (!tof.timeoutOccurred()) {
      portENTER_CRITICAL(&mux);
      distance_mm = dist;
      portEXIT_CRITICAL(&mux);
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

bool tof_init()
{
  Wire.setPins(PIN_TOF_SDA, PIN_TOF_SCL);
  Wire.begin();

  pinMode(PIN_TOF_XSHUT, OUTPUT);
  digitalWrite(PIN_TOF_XSHUT, LOW);
  delay(10);

  pinMode(PIN_TOF_XSHUT, INPUT);
  delay(10);

  tof.setBus(&Wire);
  tof.setTimeout(500);
  if (!tof.init())
  {
    Serial.println("Failed to detect and initialize tof");
    return false;
  }

  tof.setAddress(0x2A);

  tof.startContinuous();
  // tof.setMeasurementTimingBudget(2000); // reduce timing budget to 20 ms (default is about 33 ms)
  
  xTaskCreatePinnedToCore(tof_task, "VL53L0X Task", 4096, NULL, 1, &tofTaskHandle, 0);

  return true;
}

uint16_t tof_get_last_measure() {
  uint16_t value;
  portENTER_CRITICAL(&mux);
  value = distance_mm;
  portEXIT_CRITICAL(&mux);
  return value;
}