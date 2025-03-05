#ifndef __SERVO_H__
#define __SERVO_H__
#include <stdint.h>
#include <Arduino.h>

// Khai báo chân servo
// #define SERVO_PIN 13                               
#define SERVO_PIN 1        

#define CuaXaMo true                                   
#define CuaXaDong false
#define TanSoServo       50
#define DutyMinServo     30
#define DutyMaxServo     80
#define KenhPWMServo     0
#define DoPhanGiaiServo  10
class SERVO
{
private:
  uint8_t PhanTramCuaXa;                    // Phần trăm cửa xả
public:

  void KhoiTao(void);

  void DongCuaXa(void);             

  void MoCuaXa(uint8_t); 

  uint8_t PhanTramCuaXaHienTai(void);              

};
#endif /* __SERVO_H__ */