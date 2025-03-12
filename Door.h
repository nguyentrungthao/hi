
#ifndef _DOOR_H_
#define _DOOR_H_
#include <Arduino.h>
#include "AnhLABV01HardWare.h"

#define DOOR_OPEN   1
#define DOOR_CLOSE  0

typedef void (*FuncCallBack)(void*);

class DOOR {
public:
  void KhoiTao(void);
  void addOpenFuncCallBack(FuncCallBack pOpenFuncCallBack, void* ptr);
  void addCloseFuncCallBack(FuncCallBack pCloseFuncCallBack, void* ptr);
  bool TrangThai(void);
private:
  void* pOpenPrameter;
  void* pClosePrameter;

  FuncCallBack m_pOpenFuncCallBack;
  FuncCallBack m_pCloseFuncCallBack;

  TaskHandle_t taskHandleDoor;
  static void taskDoor(void* ptr);
  static void IRAM_ATTR interupt(void* ptr);
};
#endif
