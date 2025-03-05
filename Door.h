
#ifndef _DOOR_H_
#define _DOOR_H_
#include <Arduino.h>

// #define PIN_DOOR 36
#define PIN_DOOR    2
#define DOOR_OPEN   1
#define DOOR_CLOSE  0


class DOOR {
public:
  void KhoiTao(void);
  bool TrangThai(void);
  bool CoDongMoCua = false;
};
#endif
