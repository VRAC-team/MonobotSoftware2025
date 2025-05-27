#ifndef TOF_HEADER
#define TOF_HEADER

#include <cstdint>

#define PIN_TOF_SDA 11   // RX3
#define PIN_TOF_SCL 12   // RX2
#define PIN_TOF_GPIO 10  // RX1
#define PIN_TOF_XSHUT 13 // RX4

bool tof_init();
uint16_t tof_get_last_measure();

#endif