#ifndef BOARDID_HEADER
#define BOARDID_HEADER

#include <cstdint>

#define PIN_ID1 7
#define PIN_ID2 6
#define PIN_ID3 42

void boardid_init();
uint8_t boardid_get();

#endif