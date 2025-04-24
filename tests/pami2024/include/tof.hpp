#ifndef TOF_HEADER
#define TOF_HEADER

#include <cstdint>

#define PIN_TOF_SDA 11   // RX3
#define PIN_TOF_SCL 12   // RX2
#define PIN_TOF_GPIO 10  // RX1
#define PIN_TOF_XSHUT 13 // RX4

#define READ_TIME_PERIOD_MS 40

bool tof_init();
bool tof_update();
uint16_t tof_get();

#endif