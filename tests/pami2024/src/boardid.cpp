#include "boardid.hpp"
#include <Arduino.h>

uint8_t board_id = 255;

void boardid_init() {
  pinMode(PIN_ID1, INPUT_PULLUP);
  pinMode(PIN_ID2, INPUT_PULLUP);
  pinMode(PIN_ID3, INPUT_PULLUP);

  uint8_t id1 = !digitalRead(PIN_ID1);
  uint8_t id2 = !digitalRead(PIN_ID2);
  uint8_t id3 = !digitalRead(PIN_ID3);
  board_id = id1 << 0 | id2 << 1 | id3 << 2;
}

uint8_t boardid_get() {
  return board_id;
}