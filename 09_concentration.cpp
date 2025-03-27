#include "09_concentration.h"

/*
    @brief:     khởi tạo các  constructor
*/
Concentration::Concentration(void)
  : IRCO2(), HRTOnOffPin(), PID() {
};


/*
    @brief:     khởi tạo cảm biến CO2 và khởi tạo cả 2 van khí
*/
void Concentration::KhoiTaoCO2() {

  // khởi tạo cảm biến CO2
  if (IRCO2::init() != IRCO2_OK) {
    Serial.printf("init IRCO2 failed\n");
  }

  nongDoThucCO2 = this->LayGiaTriTuCamBien();

  HRTOnOffPin::init();
  PID::setPIDparamters(CO2_KP, CO2_KI, CO2_KD);
  PID::setWindup(CO2_WIN_MIN, CO2_WIN_MAX, CO2_KW);
  PID::setOutput(CO2_OUT_MIN, CO2_OUT_MAX);
  PID::setSampleTime(CO2_SAMPLE_TIME);

  this->TatDieuKhienCO2();
  //khởi tạo task tính toán thời gian điều khiển van khí
  xTaskCreate(taskTinhToanCO2, "tính CO2", 8096, (void*)this, (configMAX_PRIORITIES - 3), &taskTinhToan);
  delay(1000);
}

//* cài giá trị setpoint cho CO2
void Concentration::CaiNongDoCO2(float giaTriDat) {
  // giới hạn điều khiển từ 0 đến 20%
  if (0.0f <= giaTriDat && giaTriDat <= 20.0f) {
    this->nongDoDat = giaTriDat;
  }
}

//*lấy giá trị setpoint của CO2
float Concentration::LayNongDoDatCO2() {
  return this->nongDoDat;
}
//*lấy trạng thái valve
bool Concentration::LayTrangThaiVan() {
  return this->getStatusPin();  // lấy từ class Valve
}
void Concentration::BatDieuKhienCO2() {
  this->coChayBoDieuKhienCO2 = BAT_CO2;
  step = 1;
}
void Concentration::TatDieuKhienCO2() {
  this->coChayBoDieuKhienCO2 = TAT_CO2;
  step = 0;
  this->turnOffPin();
}
void Concentration::SetEventDOOR() {
  if (coChayBoDieuKhienCO2 == BAT_CO2) {
    step = 2;
  }
}
void Concentration::ResetEventDOOR() {
  if (coChayBoDieuKhienCO2 == BAT_CO2) {
    step = 3;
    preCloseDoor = millis();
  }
}

void Concentration::taskTinhToanCO2(void* ptr) {
  Concentration* pClass = static_cast<Concentration*>(ptr);
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  float SaiSo = 0;
  int i = 0;
  uint64_t thoiGianMoVan;
  constexpr uin16_t p
  while (1) {
    pClass->nongDoThucCO2 = pClass->LayGiaTriTuCamBien();
    //! kiểm tra giá trị đọc về từ cảm biến
    switch (pClass->step) {
    case 0:
      i = 0;
      break;
    case 1:
      if (i >= (CO2_SAMPLE_TIME / 1000) && (pClass->nongDoThucCO2 >= 0.0f && pClass->nongDoThucCO2 <= 30.0f)) {
        SaiSo = pClass->nongDoDat - pClass->nongDoThucCO2;
        thoiGianMoVan = (uint64_t)pClass->getPIDcompute(SaiSo);
        pClass->turnOnPinAndDelayOff(thoiGianMoVan);
        i = 0;
      }
      break;
    case 2:
      break;
    case 3:
      if (millis() - pClass->preCloseDoor >= 10 * 1000) {  // theo dõi CO2 20 trước khi quay lại tính toán
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
float Concentration::LayGiaTriTuCamBien() {
  if (IRCO2::ReadData() == IRCO2_OK) {
    return IRCO2::GetCO2Concentration();
  }
  else {
    IRCO2::init(); // khởi tạo lại cảm biến
  }
  return LOI_GIAO_TIEP;  // giá trị báo lỗi giao tiếp
}


float Concentration::LayNongDoCO2Thuc() {
  return this->nongDoThucCO2;
}
//* lấy thời gian kích van được tính toán
uint32_t Concentration::LayThoiGianKichVan() {
  return static_cast<uint32_t>(Concentration::thoiGianMoVan);
}

//* khởi động lại cảm biến
//* sau khi khởi động lại cảm biến trả về giá trị âm -2
void Concentration::KhoiDongLaiCamBien() {
  this->SensorReset();
}
//* xóa hết các giá trị calib, đưa các giá trị calib về gốc
IRCO2_StatusTydef Concentration::XoaToanBoGiaTriCalib() {
  return this->SetFactoryDefault();
}
//* gửi giá trị để cảm biến tự động điều chỉnh
IRCO2_StatusTydef Concentration::CalibGiaTriThuc(float giaTriChuan) {
  if (giaTriChuan < -0.5f && giaTriChuan > 20.0f) {
    return IRCO2_ERR_VAL;
  }
  uint32_t _giaTriChuan = giaTriChuan * 1000.0f;

  for (uint8_t i = 0; i < 5; i++) {
    // giá trị gửi từ 0.5 đến 20 tương ứng 500 đến 20000
    if (this->SpanPointAdjustment(_giaTriChuan) == IRCO2_OK) {
      Serial.println("CALIB SPAN CO2 OKE");
      return IRCO2_OK;
    }
    delay(50);
  }

  return IRCO2_ERR_VAL;
}
//* gửi giá trị để cảm biến tự động điều chỉnh điểm Zero vì lượng CO2 mỗi nơi có thể khác nhau giá trị từ
IRCO2_StatusTydef Concentration::CalibDiem0(float giaTri0Chuan) {
  if (giaTri0Chuan < 0.0f && giaTri0Chuan > 0.5f) {
    return IRCO2_ERR_VAL;
  }
  uint32_t _giaTriChuan = giaTri0Chuan * 1000.0f;

  for (uint8_t i = 0; i < 5; i++) {
    // giá trị gửi từ 0 đến 0.5 tương ứng 0 đến 500
    if (this->ZeroPointAdjustment(_giaTriChuan) == IRCO2_OK) {
      Serial.println("CALIB ZERO CO2 OKE");
      return IRCO2_OK;
    }
    delay(50);
  }
  return IRCO2_ERR_VAL;
}
//calib áp suất khi xả khí là 0.1Mpa lưu lượng đi vào 1 (lít/min)
void Concentration::CalibApSuat(uint32_t thoiGianMoVan) {
  if (thoiGianMoVan < 10) return;
  this->turnOnPinAndDelayOff(thoiGianMoVan);
}
