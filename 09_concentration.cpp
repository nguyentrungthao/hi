#include "09_concentration.h"

/*
    @brief:     khởi tạo các  constructor
*/
Concentration::Concentration(const char *name)
    : IRCO2(), HRTOnOffPin(), PID(), EEPROMClass(name)
{
  xControlParamaterCO2 = userCO2_DEFAUT_CONTROL_PARAMETER();
  vSetParam(xControlParamaterCO2.xPID);
};

/*
    @brief:     khởi tạo cảm biến CO2 và khởi tạo cả 2 van khí
*/
void Concentration::KhoiTaoCO2(const uint16_t u16SizeEEprom)
{

  // khởi tạo cảm biến CO2
  if (IRCO2::init() != IRCO2_OK)
  {
    Serial.printf("init IRCO2 failed\n");
  }
  nongDoThucCO2 = this->LayGiaTriTuCamBien();

  HRTOnOffPin::init();
  khoiTaoEEpprom(u16SizeEEprom);

  this->TatDieuKhienCO2();
  // khởi tạo task tính toán thời gian điều khiển van khí
  xTaskCreate(taskTinhToanCO2, "tính CO2", 8096, (void *)this, (configMAX_PRIORITIES - 3), &taskTinhToan);
  delay(1000);
}

void Concentration::vSuspend()
{
  if (taskTinhToan)
  {
    vTaskSuspend(taskTinhToan);
  }
}
void Concentration::vResume()
{
  if (taskTinhToan)
  {
    vTaskResume(taskTinhToan);
  }
}

//* cài giá trị setpoint cho CO2
void Concentration::CaiNongDoCO2(float giaTriDat)
{
  // giới hạn điều khiển từ 0 đến 20%
  if (0.0f <= giaTriDat && giaTriDat <= 20.0f)
  {
    this->nongDoDat = giaTriDat;
  }
}

//*lấy giá trị setpoint của CO2
float Concentration::LayNongDoDatCO2()
{
  return this->nongDoDat;
}
//*lấy trạng thái valve
bool Concentration::LayTrangThaiVan()
{
  return this->getStatusPin(); // lấy từ class Valve
}
void Concentration::BatDieuKhienCO2()
{
  this->coChayBoDieuKhienCO2 = BAT_CO2;
  vResetPID();

  step = 1;
}
void Concentration::TatDieuKhienCO2()
{
  this->coChayBoDieuKhienCO2 = TAT_CO2;
  this->turnOffPin();
  step = 0;
}
void Concentration::SetEventDOOR()
{
  if (coChayBoDieuKhienCO2 == BAT_CO2)
  {
    step = 2;
  }
}
void Concentration::ResetEventDOOR()
{
  if (coChayBoDieuKhienCO2 == BAT_CO2)
  {
    step = 3;
    preCloseDoor = millis();
  }
}

void Concentration::taskTinhToanCO2(void *ptr)
{
  Concentration *pClass = static_cast<Concentration *>(ptr);
  constexpr uint16_t u16CYCLE = CO2_SAMPLE_TIME / 1000;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  float SaiSo = 0;
  int i = 0;
  uint64_t thoiGianMoVan;
  while (1)
  {
    pClass->nongDoThucCO2 = pClass->LayGiaTriTuCamBien();
    //! kiểm tra giá trị đọc về từ cảm biến
    switch (pClass->step)
    {
    case 0:
      i = 0;
      break;
    case 1:
      if (i >= u16CYCLE && (pClass->nongDoThucCO2 >= -0.5f && pClass->nongDoThucCO2 <= 30.0f))
      {
        SaiSo = pClass->nongDoDat - pClass->nongDoThucCO2;
        thoiGianMoVan = (uint64_t)pClass->getPIDcompute(SaiSo);
        if (concenTHOI_GIAN_MO_VAN_TOI_THIEU < thoiGianMoVan)
        {
          pClass->turnOnPinAndDelayOff(thoiGianMoVan);
        }
        i = 0;
      }
      break;
    case 2:
      break;
    case 3:
      if (millis() - pClass->preCloseDoor >= 10 * 1000)
      { // theo dõi CO2 20 trước khi quay lại tính toán
        pClass->step = 1;
      }
      break;
    default:
      Serial.printf("case error. Line %d, value %d", __LINE__, pClass->step);
      break;
    }
    i++;
    // Serial.printf("CO2: %.2f, SaiSo: %.2f, output: %llu\n", pClass->nongDoThucCO2, SaiSo, thoiGianMoVan);
    thoiGianMoVan = 0;

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
  }
}

/*
    @brief:     lấy giá trị thực tế từ cảm biến CO2

    @return:    nồng độ CO2 (đơn vị %)
*/
float Concentration::LayGiaTriTuCamBien()
{
  if (IRCO2::ReadData() == IRCO2_OK)
  {
    return IRCO2::GetCO2Concentration();
  }
  else
  {
    IRCO2::init(); // khởi tạo lại cảm biến
  }
  return LOI_GIAO_TIEP; // giá trị báo lỗi giao tiếp
}

float Concentration::LayNongDoCO2Thuc()
{
  return this->nongDoThucCO2;
}
//* lấy thời gian kích van được tính toán
uint32_t Concentration::LayThoiGianKichVan()
{
  return static_cast<uint32_t>(Concentration::thoiGianMoVan);
}

//* khởi động lại cảm biến
//* sau khi khởi động lại cảm biến trả về giá trị âm -2
void Concentration::KhoiDongLaiCamBien()
{
  this->SensorReset();
}
//* xóa hết các giá trị calib, đưa các giá trị calib về gốc
IRCO2_StatusTydef Concentration::XoaToanBoGiaTriCalib()
{
  return this->SetFactoryDefault();
}
//* gửi giá trị để cảm biến tự động điều chỉnh
IRCO2_StatusTydef Concentration::CalibGiaTriThuc(float giaTriChuan)
{
  if (giaTriChuan < -0.5f && giaTriChuan > 20.0f)
  {
    return IRCO2_ERR_VAL;
  }
  uint32_t _giaTriChuan = giaTriChuan * 1000.0f;

  for (uint8_t i = 0; i < 5; i++)
  {
    // giá trị gửi từ 0.5 đến 20 tương ứng 500 đến 20000
    if (this->SpanPointAdjustment(_giaTriChuan) == IRCO2_OK)
    {
      Serial.println("CALIB SPAN CO2 OKE");
      return IRCO2_OK;
    }
    delay(50);
  }

  return IRCO2_ERR_VAL;
}
//* gửi giá trị để cảm biến tự động điều chỉnh điểm Zero vì lượng CO2 mỗi nơi có thể khác nhau giá trị từ
IRCO2_StatusTydef Concentration::CalibDiem0(float giaTri0Chuan)
{
  if (giaTri0Chuan < 0.0f && giaTri0Chuan > 0.5f)
  {
    return IRCO2_ERR_VAL;
  }
  uint32_t _giaTriChuan = giaTri0Chuan * 1000.0f;

  for (uint8_t i = 0; i < 5; i++)
  {
    // giá trị gửi từ 0 đến 0.5 tương ứng 0 đến 500
    if (this->ZeroPointAdjustment(_giaTriChuan) == IRCO2_OK)
    {
      Serial.println("CALIB ZERO CO2 OKE");
      return IRCO2_OK;
    }
    delay(50);
  }
  return IRCO2_ERR_VAL;
}
// calib áp suất khi xả khí là 0.1Mpa lưu lượng đi vào 1 (lít/min)
void Concentration::CalibApSuat(uint32_t thoiGianMoVan)
{
  if (thoiGianMoVan < 10)
    return;
  this->turnOnPinAndDelayOff(thoiGianMoVan);
}

CO2CalibStruct_t Concentration::xGetCalibParamater() const
{
  return xCO2CalibStruct;
}

ControlParamaterCO2 Concentration::xGetControlParamater()
{
  xControlParamaterCO2.xPID = xGetParam();
  return xControlParamaterCO2;
}
void Concentration::vSetControlParamater(ControlParamaterCO2 xCtrParamter)
{
  this->xControlParamaterCO2 = xCtrParamter;
  strncpy(xControlParamaterCO2.pcConfim, userEEPROM_CONFIRM_DATA_STRING, co2EEPROM_LENGTH_CONFRIM);
  vSetParam(xControlParamaterCO2.xPID);
  LuuDuwLieuvaoEEprom();
}

void Concentration::khoiTaoEEpprom(const uint16_t u16SizeEEprom)
{
  if (!EEPROMClass::begin(u16SizeEEprom))
  {
    Serial.printf("%s Failed khoiTaoEEPROM\n", __FILE__);
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  EEPROMClass::readBytes(0, (uint8_t *)&xControlParamaterCO2, sizeof(xControlParamaterCO2));
  delay(10);
  if (strncmp(userEEPROM_CONFIRM_DATA_STRING, xControlParamaterCO2.pcConfim, strlen(userEEPROM_CONFIRM_DATA_STRING)) != 0)
  {
    Serial.printf("init control CO2\n");
    ControlParamaterCO2 xControlParamaterCO2 = userCO2_DEFAUT_CONTROL_PARAMETER();
    this->xControlParamaterCO2 = xControlParamaterCO2;
    EEPROMClass::writeBytes(0, (uint8_t *)(&xControlParamaterCO2), sizeof(xControlParamaterCO2));
    EEPROMClass::commit();
    delay(10);
  }
  vSetParam(xControlParamaterCO2.xPID);
  Serial.printf("P%f, I%f, D%f, Ia%f, Ii%f, Oa%f, Oi%d",
                xControlParamaterCO2.xPID.Kp,        // P%f
                xControlParamaterCO2.xPID.Ki,        // I%f
                xControlParamaterCO2.xPID.Kd,        // D%f  (như bạn đã bắt đầu)
                xControlParamaterCO2.xPID.WindupMax, // Ia%f (Giả định "Ia" là Integral Anti-windup Max)
                xControlParamaterCO2.xPID.WindupMin, // Ii%f (Giả định "Ii" là Integral Anti-windup Min)
                xControlParamaterCO2.xPID.OutMax,    // Oa%f (Giả định "Oa" là Output Max)
                xControlParamaterCO2.xPID.OutMin     // Oi%f (Giả định "Oi" là Output Min)
  );
}
void Concentration::LayDuLieuTuEEprom()
{
  EEPROMClass::readBytes(0, (uint8_t *)(&xControlParamaterCO2), sizeof(xControlParamaterCO2));
}
void Concentration::LuuDuwLieuvaoEEprom()
{
  EEPROMClass::writeBytes(0, (uint8_t *)(&xControlParamaterCO2), sizeof(xControlParamaterCO2));
  EEPROMClass::commit();
}

void Concentration::LayDuLieuLuuTru(ControlParamaterCO2 *pxControlParamaterCO2)
{
  if (pxControlParamaterCO2)
  {
    EEPROMClass::readBytes(0, (uint8_t *)(&pxControlParamaterCO2), sizeof(pxControlParamaterCO2));
  }
}