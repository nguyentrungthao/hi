#include "07_Heater.h"

HEATER::HEATER()
  : PID(), triac((gpio_num_t)TRIAC4_PIN) {
}

void HEATER::KhoiTao(void) {
  KhoiTaoCamBien();
  KhoiTaoThongSoPID();
  KhoiTaoTriac();
  xTaskCreate(TaskDieuKhienNhiet, "tính Nhiệt", 4096, (void*)this, (configMAX_PRIORITIES - 3), &taskHandleDieuKhienNhiet);
}

void HEATER::CaiDatNhietDo(float setpoint) {
  NhietDoCaiDat = setpoint;
}
void HEATER::CaiGiaTriOfset(float calibOffset) {
  HeSoCalib = calibOffset;
}


float HEATER::LayNhietDoLoc(void) {
  return NhietDoLocBuong;
}
float HEATER::LayGiaTriPT100Ofset(void) {
  return HeSoCalib;
}

void HEATER::BatDieuKhienNhietDo(void) {
  TrangThaiDieuKhienNhiet = HEATER_ON;
  step = 1;
  TurnOnTriac();
  u16ThoiGianBatBuong = 0;
  u16ThoiGianBatVanh = THOI_GIAN_BAT_TRIAC_VANH;
  u16ThoiGianBatCua = THOI_GIAN_BAT_TRIAC_CUA;
}
void HEATER::TatDieuKhienNhietDo(void) {
  TrangThaiDieuKhienNhiet = HEATER_OFF;
  step = 0;
  TurnOffTriac();
  triacBuong.turnOffPin();
  triacCua.turnOffPin();
  triacVanh.turnOffPin();
  u16ThoiGianBatBuong = 0;
  u16ThoiGianBatVanh = 0;
  u16ThoiGianBatCua = 0;
}
void HEATER::SetEventDOOR() {
  if (TrangThaiDieuKhienNhiet == HEATER_ON) {
    step = 2;
    preOpenDoor = millis();
  }
}
void HEATER::ResetEventDOOR() {
  if (TrangThaiDieuKhienNhiet == HEATER_ON) {
    step = 3;
    preCloseDoor = millis();
  }
}

void HEATER::CalibNhietDoPT100(float, float) {
}
void HEATER::ResetCamBienNhiet(void) {
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
void HEATER::CaiTocDoQuat(uint8_t value) {
  if (value < 10) return;
  uint16_t gocKichTriac = map(value, 10, 100, 5000, 4000);
  triac::SetTimeOverFlow(gocKichTriac);
}
void HEATER::KhoiDongQuat(uint16_t thoiGianKhoiDong) {
}
bool HEATER::TrangThaiThanhGiaNhiet(void) {
  return triacBuong.getStatusPin();
}
bool HEATER::TrangThaiQuat(void) {
  return RunStatus;
}
bool HEATER::CheckNguonCongSuat() {
  return isACDET;
}
void HEATER::addCallBackACDET(CallBackACDET_t pCallBack, void* pArg) {
  if (pCallBack != NULL) {
    m_pCallBackACDET = pCallBack;
  }
  else {
    Serial.println("ERORR: m_pCallBackACDET is NULL");
    return;
  }
  if (pArg != NULL) {
    m_pArgACDET = pArg;
  }
  else {
    Serial.println("Warning: m_pArgACDET is NULL");
  }
}
void HEATER::TaskDieuKhienNhiet(void* ptr) {
  if (ptr == NULL) return;
  HEATER* pHeater = (HEATER*)ptr;

  portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  float saiSo = 0;
  uint8_t chuKyTheoDoi = 0;  // đếm số lần số để trừ dần cửa và vành

  while (1) {

    pHeater->NhietDoLocBuong = pHeater->LocCamBienBuong.updateEstimate(pHeater->DocGiaTriCamBien(pHeater->PT100_buong)) + pHeater->HeSoCalib;

    switch (pHeater->step) {
    case 0:  //tắt máy
      pHeater->u16ThoiGianBatBuong = 0;
      pHeater->u16ThoiGianBatVanh = 0;
      pHeater->u16ThoiGianBatCua = 0;
      // Serial.print(" - step 0 - ");
      break;

    case 1:  // chạy máy bình thường
      // tính toán
      saiSo = pHeater->NhietDoCaiDat - pHeater->NhietDoLocBuong;
      pHeater->u16ThoiGianBatBuong = (uint16_t)round(pHeater->getPIDcompute(saiSo));
      pHeater->u16ThoiGianBatBuong *= 10;  // chuyển số chu kì sang mS

      chuKyTheoDoi++;
      if (chuKyTheoDoi >= 120) {  // 60 chu kỳ với delay 1s => theo dõi 60s
        chuKyTheoDoi = 0;
        if (saiSo < -0.1) {
          pHeater->u16ThoiGianBatVanh -= 10;
          pHeater->u16ThoiGianBatCua -= 10;
        }
        else {
          pHeater->u16ThoiGianBatVanh += 10;
          pHeater->u16ThoiGianBatCua += 10;
        }

        // khống chế giá trị
        if (pHeater->u16ThoiGianBatVanh < 0) {
          pHeater->u16ThoiGianBatVanh = 0;
        }
        else if (pHeater->u16ThoiGianBatVanh > THOI_GIAN_BAT_TRIAC_VANH) {
          pHeater->u16ThoiGianBatVanh = THOI_GIAN_BAT_TRIAC_VANH;
        }
        if (pHeater->u16ThoiGianBatCua < 0) {
          pHeater->u16ThoiGianBatCua = 0;
        }
        else if (pHeater->u16ThoiGianBatCua > THOI_GIAN_BAT_TRIAC_CUA) {
          pHeater->u16ThoiGianBatCua = THOI_GIAN_BAT_TRIAC_CUA;
        }
      }
      // Serial.print(" - step 1 - ");
      break;

    case 2:                                       // chạy mở cửa
      if (millis() - pHeater->preOpenDoor >= 30 * 1000) {  // mở cửa lâu hơn 30s
        pHeater->triacBuong.turnOffPin();
        pHeater->u16ThoiGianBatBuong = 0;
        //tắt quạt
        pHeater->TurnOffTriac();
      }
      // kích bật công suất cao cho cửa và vành
      pHeater->u16ThoiGianBatCua = OUT_MAX_POWER;
      pHeater->u16ThoiGianBatVanh = OUT_MAX_POWER;
      // Serial.printf(" - step 2 %ld %ld - ", millis(), pHeater->preOpenDoor);
      break;

    case 3:                                        // chạy đóng cửa
      if (millis() - pHeater->preCloseDoor >= 60 * 1000) {  // theo dõi nhiệt 30s trước khi quay lại tính toán
        pHeater->u16ThoiGianBatCua = THOI_GIAN_BAT_TRIAC_CUA;
        pHeater->u16ThoiGianBatVanh = THOI_GIAN_BAT_TRIAC_VANH;
        pHeater->step = 1;
      }
      else {
        pHeater->TurnOnTriac();
        pHeater->KhoiDongQuat(5);
        // kích bật công xuất cao cho cửa và vành
        pHeater->u16ThoiGianBatCua = OUT_MAX_POWER;
        pHeater->u16ThoiGianBatVanh = OUT_MAX_POWER;
        // Serial.print(" - đợi - ");
      }
      // Serial.printf(" - step 3 %ld %ld - ", millis(), pHeater->preCloseDoor);
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
    //   pHeater->NhietDoLocBuong, pHeater->u16ThoiGianBatBuong, pHeater->u16ThoiGianBatCua, pHeater->u16ThoiGianBatVanh);
    
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
  }
}
// void IRAM_ATTR HEATER::interupt(void* ptr) {
void HEATER::interupt(void* ptr) {
  if (ptr == NULL) return;
  HEATER* pHeater = (HEATER*)ptr;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(pHeater->taskHandleNgatACDET, 0x01, eSetBits, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
void HEATER::TaskNgatACDET(void* ptr) {
  if (ptr == NULL) return;
  HEATER* pHeater = (HEATER*)ptr;
  uint32_t notifyNum;
  while (1) {
    if (xTaskNotifyWait(pdFALSE, pdTRUE, &notifyNum, pdMS_TO_TICKS(100)) == pdFALSE) {
      pHeater->isACDET = false;
      continue;
    }
    pHeater->acdet_intr_handler_in_task();
    pHeater->isACDET = true;
    if (pHeater->m_pCallBackACDET != NULL) {
      pHeater->m_pCallBackACDET(pHeater->m_pArgACDET);
    }
    for (int8_t i = 0; i < eTriacMax; i++) {
      if (!pHeater->pArrTriac[i]->isTimerRunning() && pHeater->pArru16ThoiGianKichTriac[i] > 0) {
        pHeater->pArrTriac[i]->turnOnPinAndDelayOff(pHeater->pArru16ThoiGianKichTriac[i]);
        pHeater->pArru16ThoiGianKichTriac[i] = 0;

      }
    }
  }
}

void HEATER::KhoiTaoThongSoPID(void) {
  setPIDparamters(KP, KI, KD);
  setWindup(WIN_MIN, WIN_MAX, KW);
  setOutput(OUT_MIN, OUT_MAX);
  setSampleTime(SAMPLE_TIME);
}
void HEATER::KhoiTaoCamBien() {
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
  for (uint8_t i = 0; i < 20; i++) {
    NhietDoLocBuong = LocCamBienBuong.updateEstimate(PT100_buong.temperature(MAX31865_DIEN_TRO_CAM_BIEN, MAX31865_DIEN_TRO_THAM_CHIEU)) + HeSoCalib ;  //* lọc giá trị đọc
  }
}
float HEATER::DocGiaTriCamBien(Adafruit_MAX31865& xPt100) {
  float fNhietDo = 999;
  uint8_t u8Try = 5;
  while (u8Try--) {
    fNhietDo = xPt100.temperature(MAX31865_DIEN_TRO_CAM_BIEN, MAX31865_DIEN_TRO_THAM_CHIEU);
    if (abs(fNhietDo) < MAX31865_NHIET_DO_TOI_DA) {
      break;
    }
    fNhietDo = 999;
    xPt100.begin(MAX31865_4WIRE);
    xPt100.enable50Hz(true);
    delay(100);
  }
  return fNhietDo;
}
void HEATER::KhoiTaoTriac(void) {
  triacBuong.init();
  triacVanh.init();
  triacCua.init();
  triac::init();  // khởi tạo triac tốc độ quạt
  //! triac::configACDETPIN(ACDET_PIN); 
  //!không trực tiếp gọi vì heater cần tín hiệu từ ACDET điều khiển số chu kỳ dẫn của triac
  pinMode(ACDET_PIN, INPUT);
  xTaskCreate(TaskNgatACDET, "ngatACDET", 8196, this, (configMAX_PRIORITIES - 2), &taskHandleNgatACDET);
  attachInterruptArg(ACDET_PIN, interupt, this, FALLING);
}
