#include "esp32-hal-ledc.h"
#include "11_Servo.h"


//-------------------------------Hàm Khởi tạo---------------------------------
void SERVO::KhoiTao(void)
{
  pinMode(SERVO_PIN, OUTPUT);
  // ledcSetup(KenhPWMServo, TanSoServo, DoPhanGiaiServo);
  ledcAttachChannel(SERVO_PIN, TanSoServo, DoPhanGiaiServo, KenhPWMServo);
  ledcWriteChannel(KenhPWMServo, DutyMinServo);  
}
//----------------------------------------------------------------------------


//-------------------------Hàm điều khiển servo đóng cửa xả-------------------
void SERVO::DongCuaXa(void)
{
  ledcWriteChannel(KenhPWMServo, DutyMinServo);  
}
//----------------------------------------------------------------------------


//-----------------------Hàm điều khiển servo đi vào--------------------------
void SERVO::MoCuaXa(uint8_t PhanTram)
{
  this -> PhanTramCuaXa = PhanTram;
  ledcWriteChannel(KenhPWMServo, map(PhanTram, 100, 0, DutyMinServo, DutyMaxServo));
}
//----------------------------------------------------------------------------


//---------------------Hàm trả về phần trăm mở hiện tại-----------------------
uint8_t SERVO::PhanTramCuaXaHienTai(void)
{
    return this->PhanTramCuaXa;
}
//----------------------------------------------------------------------------
