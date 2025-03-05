#include "Door.h"

void DOOR::KhoiTao(void) {
  pinMode(PIN_DOOR, INPUT);
  // _CoDongMoCua = TrangThai();
}

bool DOOR::TrangThai(void) {
  return digitalRead(PIN_DOOR) ? DOOR_CLOSE:DOOR_OPEN;  // Return trạng thái cửa
}