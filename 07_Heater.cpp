#include "07_Heater.h"

HEATER::HEATER(const char *name)
    : PID(), triac((gpio_num_t)TRIAC4_PIN), EEPROMClass(name)
{
}

void HEATER::KhoiTao(TempCalibStruct_t xTempCalibStruct, const uint16_t u16SizeEEprom)
{
  if (userCHECK_FLAG(u8SysFlagGroup, heaterFLAG_INITED))
  {
    Serial.printf("%s đã khởi tạo rồi\n", __FILE__);
    return;
  }
  userSET_FLAG(u8SysFlagGroup, heaterFLAG_INITED);



  KhoiTaoCamBien(xTempCalibStruct);
  userSET_FLAG(u8SysFlagGroup, heaterFLAG_INIT_SENSOR);

  KhoiTaoTriac();
  userSET_FLAG(u8SysFlagGroup, heaterFLAG_INTI_ACTUATOR);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  khoiTaoEEpprom(u16SizeEEprom);
  userSET_FLAG(u8SysFlagGroup, heaterFLAG_INIT_EEPROM);

  xTaskCreate(TaskDieuKhienNhiet, "TempCalcu", 4096, (void *)this, (configMAX_PRIORITIES - 3), &taskHandleDieuKhienNhiet);
}

void HEATER::vSuspend()
{
  if (taskHandleDieuKhienNhiet)
  {
    vTaskSuspend(taskHandleDieuKhienNhiet);
  }
}
void HEATER::vResume()
{
  if (taskHandleDieuKhienNhiet)
  {
    vTaskResume(taskHandleDieuKhienNhiet);
  }
}

void HEATER::CaiDatNhietDo(float setpoint)
{
  NhietDoCaiDat = setpoint;
}
void HEATER::CaiGiaTriOfset(TempCalibStruct_t calibOffset)
{
  if (fabs(calibOffset.point1.Setpoint - calibOffset.point2.Setpoint) < 1e-6)
  {
    ESP_LOGE(__FILE__, "trùng điểm calib 1:%0.1f 2:%0.1f", calibOffset.point1.Setpoint, calibOffset.point2.Setpoint);
    return;
  }
  if (fabs(calibOffset.point2.Setpoint - calibOffset.point3.Setpoint) < 1e-6)
  {
    ESP_LOGE(__FILE__, "trùng điểm calib 2:%0.1f 3:%0.1f", calibOffset.point2.Setpoint, calibOffset.point3.Setpoint);
    return;
  }
  if (fabs(calibOffset.point3.Setpoint - calibOffset.point1.Setpoint) < 1e-6)
  {
    ESP_LOGE(__FILE__, "trùng điểm calib 3:%0.1f 1:%0.1f", calibOffset.point3.Setpoint, calibOffset.point1.Setpoint);
    return;
  }

  xTempCalibStruct = calibOffset;
  vTinhGiaTriABchoDuongCamBien(xTempCalibStruct.point1, xTempCalibStruct.point2, linePoint1Point2);
  vTinhGiaTriABchoDuongCamBien(xTempCalibStruct.point2, xTempCalibStruct.point3, linePoint2Point3);
}

float HEATER::LayNhietDoLoc(void)
{
  return NhietDoLocBuong;
}

void HEATER::BatDieuKhienNhietDo(void)
{
  userSET_FLAG(u8SysFlagGroup, heaterFLAG_ON_OFF);
  digitalWrite(RELAY_PIN, HIGH);
  int8_t i8Try = 15;
  do
  {
    delay(10); // chờ cho tiếp điểm relay đóng và có tín hiệu ACDET
    if (CheckNguonCongSuat())
    {
      break;
    }
  } while (i8Try--);

  if (NhietDoCaiDat > userSETPOINT_TEMP_MAX)
  {
    u16MaxThoiGianBatVanh = OUT_MAX_POWER;
    u16MaxThoiGianBatCua = OUT_MAX_POWER;
    setOutput(0, OUT_MAX_POWER);
  }
  else
  {
    u16MaxThoiGianBatVanh = xControlParamaterTEMP.Perimter;
    u16MaxThoiGianBatCua = xControlParamaterTEMP.Door;
    setOutput(0, xControlParamaterTEMP.xPID.OutMax);
  }
  u16ThoiGianBatVanh = u16MaxThoiGianBatVanh;
  u16ThoiGianBatCua = u16MaxThoiGianBatCua;
  u16ThoiGianBatBuong = 0;
  vResetPID();
  TurnOnTriac();

  preOpenDoor = millis();
  step = eWAIT_BEFORE_RUN;
}
void HEATER::TatDieuKhienNhietDo(void)
{
  userRESET_FLAG(u8SysFlagGroup, heaterFLAG_ON_OFF);

  digitalWrite(RELAY_PIN, LOW);
  TurnOffTriac();
  triacBuong.turnOffPin();
  triacCua.turnOffPin();
  triacVanh.turnOffPin();
  u16ThoiGianBatBuong = 0;
  u16ThoiGianBatVanh = 0;
  u16ThoiGianBatCua = 0;

  step = eOFF;
}
void HEATER::SetEventDOOR()
{
  if (userCHECK_FLAG(u8SysFlagGroup, heaterFLAG_ON_OFF))
  {
    step = eOPEN_DOOR;
    preOpenDoor = millis();
  }
}
void HEATER::ResetEventDOOR()
{
  if (userCHECK_FLAG(u8SysFlagGroup, heaterFLAG_ON_OFF))
  {
    step = eCLOSE_DOOR;
    preOpenDoor = millis();
  }
}

void HEATER::ResetCamBienNhiet(void)
{
  PT100_buong.begin(MAX31865_4WIRE);
  PT100_buong.enable50Hz(true);
  delay(10);
  PT100_2.begin(MAX31865_4WIRE);
  PT100_2.enable50Hz(true);
  delay(10);
  PT100_3.begin(MAX31865_4WIRE);
  PT100_3.enable50Hz(true);
  delay(10);
}
void HEATER::CaiTocDoQuat(uint8_t value)
{
  if (value < 10)
    return;
  uint16_t gocKichTriac = map(value, 10, 100, 5000, 4000);
  triac::SetTimeOverFlow(gocKichTriac);
}
void HEATER::KhoiDongQuat(uint16_t thoiGianKhoiDong)
{
}
bool HEATER::TrangThaiThanhGiaNhiet(void)
{
  return triacBuong.getStatusPin();
}
bool HEATER::TrangThaiQuat(void)
{
  return RunStatus;
}
bool HEATER::CheckNguonCongSuat()
{
  return isACDET;
}
void HEATER::addCallBackACDET(CallBackACDET_t pCallBack, void *pArg)
{
  if (pCallBack != NULL)
  {
    m_pCallBackACDET = pCallBack;
  }
  else
  {
    Serial.println("ERORR: m_pCallBackACDET is NULL");
    return;
  }
  if (pArg != NULL)
  {
    m_pArgACDET = pArg;
  }
  else
  {
    Serial.println("Warning: m_pArgACDET is NULL");
  }
}

ControlParamaterTEMP HEATER::xGetControlParamater()
{
  xControlParamaterTEMP.xPID = xGetParam();
  return xControlParamaterTEMP;
}
TempCalibStruct_t HEATER::xGetCalibParamater() const
{
  return xTempCalibStruct;
}
void HEATER::vSetControlParamater(ControlParamaterTEMP xCtrParamter)
{
  this->xControlParamaterTEMP = xCtrParamter;
  strncpy(xControlParamaterTEMP.pcConfim, userEEPROM_CONFIRM_DATA_STRING, heaterEEPROM_LENGTH_CONFRIM);
  vSetParam(xControlParamaterTEMP.xPID);
  LuuDuwLieuvaoEEprom();
}

// private
void HEATER::TaskDieuKhienNhiet(void *ptr)
{
  if (ptr == NULL)
    return;
  HEATER *pHeater = (HEATER *)ptr;

  portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  float saiSo = 0;
  uint8_t chuKyTheoDoi = 0;

  while (1)
  {
    pHeater->NhietDoLocBuong = pHeater->fTinhGiaTriDauRaCamBien(pHeater->PT100_buong);

    switch (pHeater->step)
    {
    case eOFF:
      pHeater->u16ThoiGianBatBuong = 0;
      pHeater->u16ThoiGianBatVanh = 0;
      pHeater->u16ThoiGianBatCua = 0;
      break;
    case eWAIT_BEFORE_RUN:
      if (millis() - pHeater->preOpenDoor > 60000)
      {
        pHeater->step = eRUN;
      }
      break;
    case eRUN:
      saiSo = pHeater->NhietDoCaiDat - pHeater->NhietDoLocBuong;
      pHeater->u16ThoiGianBatBuong = (uint16_t)round(pHeater->getPIDcompute(saiSo));
      pHeater->u16ThoiGianBatBuong *= 10; // chuyển số chu kì sang mS

      chuKyTheoDoi++;
      if (chuKyTheoDoi >= 120)
      { // 120 chu kỳ với delay 1s => theo dõi 120s
        chuKyTheoDoi = 0;
        if (saiSo < -0.1)
        {
          pHeater->u16ThoiGianBatVanh -= 10;
          pHeater->u16ThoiGianBatCua -= 10;
        }
        else
        {
          pHeater->u16ThoiGianBatVanh += 10;
          pHeater->u16ThoiGianBatCua += 10;
        }

        // khống chế giá trị
        if (pHeater->u16ThoiGianBatVanh < 0)
        {
          pHeater->u16ThoiGianBatVanh = 0;
        }
        else if (pHeater->u16ThoiGianBatVanh > pHeater->u16MaxThoiGianBatVanh)
        {
          pHeater->u16ThoiGianBatVanh = pHeater->u16MaxThoiGianBatVanh;
        }
        if (pHeater->u16ThoiGianBatCua < 0)
        {
          pHeater->u16ThoiGianBatCua = 0;
        }
        else if (pHeater->u16ThoiGianBatCua > pHeater->u16MaxThoiGianBatCua)
        {
          pHeater->u16ThoiGianBatCua = pHeater->u16MaxThoiGianBatCua;
        }
      }
      break;

    case eOPEN_DOOR:
      if (millis() - pHeater->preOpenDoor >= 30000ul)
      { // mở cửa lâu hơn 30s
        pHeater->triacBuong.turnOffPin();
        pHeater->u16ThoiGianBatBuong = 0u;
        pHeater->TurnOffTriac();
      }
      pHeater->u16ThoiGianBatCua = OUT_MAX_POWER;
      pHeater->u16ThoiGianBatVanh = OUT_MAX_POWER;
      break;

    case eCLOSE_DOOR:
      if (millis() - pHeater->preOpenDoor >= 60000ul)
      {
        pHeater->step = eRUN;
      }
      else
      {
        pHeater->TurnOnTriac();
        pHeater->KhoiDongQuat(5);
        pHeater->u16ThoiGianBatCua = OUT_MAX_POWER;
        pHeater->u16ThoiGianBatVanh = OUT_MAX_POWER;
      }
      break;
    default:
      Serial.printf("case error. Line %d, value %d", __LINE__, pHeater->step);
      break;
    }

    taskENTER_CRITICAL(&my_spinlock);
    pHeater->pArru16ThoiGianKichTriac[eTriac1] = pHeater->u16ThoiGianBatBuong;
    pHeater->pArru16ThoiGianKichTriac[eTriac3] = pHeater->u16ThoiGianBatCua;
    pHeater->pArru16ThoiGianKichTriac[eTriac4] = pHeater->u16ThoiGianBatVanh;
    taskEXIT_CRITICAL(&my_spinlock);

    // Serial.printf("cb1: %0.2f T1:%d T3:%d T4:%d\n",
    //               pHeater->NhietDoLocBuong, pHeater->u16ThoiGianBatBuong, pHeater->u16ThoiGianBatCua, pHeater->u16ThoiGianBatVanh);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
  }
}
void HEATER::interuptACDET(void *ptr)
{
  if (ptr == NULL)
    return;
  HEATER *pHeater = (HEATER *)ptr;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(pHeater->taskHandleNgatACDET, 0x01, eSetBits, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
void HEATER::TaskNgatACDET(void *ptr)
{
  if (ptr == NULL)
    return;
  HEATER *pHeater = (HEATER *)ptr;
  uint32_t notifyNum;
  while (1)
  {
    if (xTaskNotifyWait(pdFALSE, pdTRUE, &notifyNum, pdMS_TO_TICKS(50)) == pdFALSE)
    {
      pHeater->isACDET = false;
      continue;
    }
    pHeater->acdet_intr_handler_in_task();
    pHeater->isACDET = true;
    if (pHeater->m_pCallBackACDET != NULL)
    {
      pHeater->m_pCallBackACDET(pHeater->m_pArgACDET);
    }
    for (int8_t i = 0; i < eTriacMax; i++)
    {
      if (!pHeater->pArrTriac[i]->isTimerRunning() && pHeater->pArru16ThoiGianKichTriac[i] > 0)
      {
        pHeater->pArrTriac[i]->turnOnPinAndDelayOff(pHeater->pArru16ThoiGianKichTriac[i]);
        pHeater->pArru16ThoiGianKichTriac[i] = 0;
      }
    }
  }
}

void HEATER::KhoiTaoCamBien(TempCalibStruct_t xTempCalibStruct)
{
  pinMode(PT100_CS1_PIN, OUTPUT);
  pinMode(PT100_CS2_PIN, OUTPUT);
  pinMode(PT100_CS3_PIN, OUTPUT);
  digitalWrite(PT100_CS1_PIN, 1);
  digitalWrite(PT100_CS2_PIN, 1);  
  digitalWrite(PT100_CS3_PIN, 1);
  PT100_buong.begin(MAX31865_4WIRE);
  PT100_buong.enable50Hz(true);
  delay(10);
  PT100_2.begin(MAX31865_4WIRE);
  PT100_2.enable50Hz(true);
  delay(10);
  PT100_3.begin(MAX31865_4WIRE);
  PT100_3.enable50Hz(true);
  delay(10);

  CaiGiaTriOfset(xTempCalibStruct);

  for (uint8_t i = 0; i < 20; i++)
  {
    NhietDoLocBuong = LocCamBienBuong.updateEstimate(fTinhGiaTriDauRaCamBien(PT100_buong)); //* lọc giá trị đọc
  }
}
float HEATER::DocGiaTriCamBien(Adafruit_MAX31865 &xPt100)
{
  float fNhietDo = 999;
  uint8_t u8Try = 5;
  while (u8Try--)
  {
    fNhietDo = xPt100.temperature(MAX31865_DIEN_TRO_CAM_BIEN, MAX31865_DIEN_TRO_THAM_CHIEU);
    if (abs(fNhietDo) < MAX31865_NHIET_DO_TOI_DA)
    {
      break;
    }
    fNhietDo = 999;
    xPt100.begin(MAX31865_4WIRE);
    xPt100.enable50Hz(true);
    delay(100);
  }
  return fNhietDo;
}
float HEATER::fTinhGiaTriDauRaCamBien(Adafruit_MAX31865 &xPt100)
{
  float fNhietDo = DocGiaTriCamBien(xPt100);
  // Serial.printf("nhiet cam bien %f\t", fNhietDo);
  // chọn hệ số
  if (fNhietDo <= xTempCalibStruct.point1.Setpoint)
  {
    fNhietDo = fNhietDo + xTempCalibStruct.point1.value;
  }
  else if (xTempCalibStruct.point1.Setpoint < fNhietDo && fNhietDo <= xTempCalibStruct.point2.Setpoint)
  {
    fNhietDo = linePoint1Point2.a * fNhietDo + linePoint1Point2.b;
  }
  else if (xTempCalibStruct.point2.Setpoint < fNhietDo && fNhietDo <= xTempCalibStruct.point3.Setpoint)
  {
    fNhietDo = linePoint2Point3.a * fNhietDo + linePoint2Point3.b;
  }
  else
  {
    fNhietDo = fNhietDo + xTempCalibStruct.point3.value;
  }
  // Serial.printf("nhiet calib %f\n", fNhietDo);

  return fNhietDo;
}
void HEATER::vTinhGiaTriABchoDuongCamBien(CalibData_t point1, CalibData_t point2, ParamterSensorlineOutput_t &ABParameter)
{
  if (fabs(point1.Setpoint - point2.Setpoint) < 1e-3)
    return;

  float y1 = point1.Setpoint + point1.value;
  float y2 = point2.Setpoint + point2.value;
  float x1 = point1.Setpoint;
  float x2 = point2.Setpoint;
  ABParameter.a = (y2 - y1) / (x2 - x1);
  ABParameter.b = y1 - ABParameter.a * x1;
  // Serial.printf("x1:%.1f, x2:%0.1f, a:%0.3f, b:%0.3f\n", x1, x2, ABParameter.a, ABParameter.b);
}

void HEATER::KhoiTaoTriac(void)
{
  triacBuong.init();
  triacVanh.init();
  triacCua.init();
  triac::init(); // khởi tạo triac tốc độ quạt
  //! triac::configACDETPIN(ACDET_PIN);
  //! không trực tiếp gọi vì heater cần tín hiệu từ ACDET điều khiển số chu kỳ dẫn của triac
  pinMode(ACDET_PIN, INPUT);
  xTaskCreate(TaskNgatACDET, "ngatACDET", 2048, this, (configMAX_PRIORITIES - 2), &taskHandleNgatACDET);
  attachInterruptArg(ACDET_PIN, interuptACDET, this, FALLING);
}

void HEATER::khoiTaoEEpprom(const uint16_t u16SizeEEprom)
{
  if (!EEPROMClass::begin(u16SizeEEprom))
  {
    Serial.printf("%s Failed khoiTaoEEPROM\n", __FILE__);
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  EEPROMClass::readBytes(0, (uint8_t *)&xControlParamaterTEMP, sizeof(xControlParamaterTEMP));
  delay(10);
  if (strncmp(userEEPROM_CONFIRM_DATA_STRING, xControlParamaterTEMP.pcConfim, strlen(userEEPROM_CONFIRM_DATA_STRING)) != 0)
  {
    Serial.printf("init control temprature\n");
    ControlParamaterTEMP xControlParamaterTEMP = userTEMP_DEFAUT_CONTROL_PARAMETER();
    this->xControlParamaterTEMP = xControlParamaterTEMP;
    EEPROMClass::writeBytes(0, (uint8_t *)(&xControlParamaterTEMP), sizeof(xControlParamaterTEMP));
    EEPROMClass::commit();
    delay(10);
  }
  vSetParam(xControlParamaterTEMP.xPID);
  Serial.printf("P%f, I%f, D%f, Ia%f, Ii%f, Oa%f, Oi%f, Pe%u, Do%u",
                xControlParamaterTEMP.xPID.Kp,        // P%f
                xControlParamaterTEMP.xPID.Ki,        // I%f
                xControlParamaterTEMP.xPID.Kd,        // D%f  (như bạn đã bắt đầu)
                xControlParamaterTEMP.xPID.WindupMax, // Ia%f (Giả định "Ia" là Integral Anti-windup Max)
                xControlParamaterTEMP.xPID.WindupMin, // Ii%f (Giả định "Ii" là Integral Anti-windup Min)
                xControlParamaterTEMP.xPID.OutMax,    // Oa%f (Giả định "Oa" là Output Max)
                xControlParamaterTEMP.xPID.OutMin,    // Oi%f (Giả định "Oi" là Output Min)
                xControlParamaterTEMP.Perimter,       // Pe%u
                xControlParamaterTEMP.Door            // Do%u
  );
}
void HEATER::LayDuLieuTuEEprom()
{
  EEPROMClass::readBytes(0, (uint8_t *)(&xControlParamaterTEMP), sizeof(xControlParamaterTEMP));
}
void HEATER::LuuDuwLieuvaoEEprom()
{
  EEPROMClass::writeBytes(0, (uint8_t *)(&xControlParamaterTEMP), sizeof(xControlParamaterTEMP));
  EEPROMClass::commit();
  delay(10);
}

void HEATER::LayDuLieuLuuTru(ControlParamaterTEMP *pxControlParamaterTEMP)
{
  if (pxControlParamaterTEMP)
  {
    EEPROMClass::readBytes(0, (uint8_t *)(&pxControlParamaterTEMP), sizeof(pxControlParamaterTEMP));
  }
}
