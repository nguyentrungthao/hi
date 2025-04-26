#include "Door.h"

void DOOR::KhoiTao(void) {
  pinMode(PIN_DOOR, INPUT_PULLUP);
  xTaskCreate(taskDoor, "DoorTask", 4096, this, (configMAX_PRIORITIES - 3), &taskHandleDoor);
  attachInterruptArg(PIN_DOOR, interupt, this, CHANGE);
}

bool DOOR::TrangThai(void) {
  return (digitalRead(PIN_DOOR) == DOOR_CLOSE) ? DOOR_CLOSE : DOOR_OPEN;  // Return trạng thái cửa
}

void DOOR::addOpenFuncCallBack(FuncCallBack pOpenFuncCallBack, void* ptr) {
  if (pOpenFuncCallBack == NULL) return;

  m_pOpenFuncCallBack = pOpenFuncCallBack;

  if (ptr == NULL) return;

  pOpenPrameter = ptr;
}
void DOOR::addCloseFuncCallBack(FuncCallBack pCloseFuncCallBack, void* ptr) {
  if (pCloseFuncCallBack == NULL) return;

  m_pCloseFuncCallBack = pCloseFuncCallBack;

  if (ptr == NULL) return;

  pClosePrameter = ptr;
}

void DOOR::taskDoor(void* ptr) {
  DOOR* pDoor = (DOOR*)ptr;
  uint32_t notifyNum;
  while (1) {
    xTaskNotifyWait(pdFALSE, pdTRUE, &notifyNum, portMAX_DELAY);
    delay(100);
    if (pDoor->TrangThai() == DOOR_CLOSE && pDoor->m_pCloseFuncCallBack != NULL) {
      pDoor->m_pCloseFuncCallBack(pDoor->pClosePrameter);
    }
    else if (pDoor->TrangThai() == DOOR_OPEN && pDoor->m_pOpenFuncCallBack != NULL) {
      pDoor->m_pOpenFuncCallBack(pDoor->pOpenPrameter);
    }
  }
}

// void IRAM_ATTR DOOR::interupt(void* ptr) {
void DOOR::interupt(void* ptr) {
  if (ptr == NULL) return;
  DOOR* pDoor = (DOOR*)ptr;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(pDoor->taskHandleDoor, 0x01, eSetBits, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}