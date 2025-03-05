
#include "esp32-hal.h"
#include "07_Heater.h"
#include "userdef.h"

/*-----------------------KHỞI TẠO CHUNG--------------------------*/
HEATER::HEATER(void)
  : Adafruit_MAX31865(spiCS1, spiMOSI, spiMISO, spiCLK),
    SimpleKalmanFilter(SAI_SO_DO, SAI_SO_UOC_LUONG, HE_SO_NHIEU),
    triacFan((gpio_num_t)pinFAN, TIMER_GROUP_0, TIMER_0),
    triacHeater((gpio_num_t)pinHEATER, TIMER_GROUP_0, TIMER_1){};

void HEATER::KhoiTaoThongSoPID(void) {
  this->MangCacThongSoPID[0] = {
    .Setpoint = 19,
    .Kp = 25,
    .Ki = 0.05,
    .Kd = 0,
    .OutMax = 45,
    .WindupMax = 35,
    .ThoiGianOnDinhSauMoCua = 80,
    .ThoiGianGiaNhietSauMoCua = 50,
  };
  this->MangCacThongSoPID[1] = {
    .Setpoint = 30,
    .Kp = 30,
    .Ki = 0.1,
    .Kd = 0,
    .OutMax = 55,
    .WindupMax = 40,
    .ThoiGianOnDinhSauMoCua = 80,
    .ThoiGianGiaNhietSauMoCua = 50,
  };
  this->MangCacThongSoPID[2] = {
    .Setpoint = 40,
    .Kp = 45,
    .Ki = 0.2,
    .Kd = 0,
    .OutMax = 70,
    .WindupMax = 60,
    .ThoiGianOnDinhSauMoCua = 80,
    .ThoiGianGiaNhietSauMoCua = 50,
  };
  this->MangCacThongSoPID[3] = {
    .Setpoint = 60,
    .Kp = 50,
    .Ki = 0.3,
    .Kd = 0,
    .OutMax = 90,
    .WindupMax = 75,
    .ThoiGianOnDinhSauMoCua = 60,
    .ThoiGianGiaNhietSauMoCua = 50,
  };
  this->MangCacThongSoPID[4] = {
    .Setpoint = 80,
    .Kp = 60,
    .Ki = 0.4,
    .Kd = 0,
    .OutMax = 100,
    .WindupMax = 100,
    .ThoiGianOnDinhSauMoCua = 60,
    .ThoiGianGiaNhietSauMoCua = 60,
  };
  this->MangCacThongSoPID[5] = {
    .Setpoint = 100,
    .Kp = 80,
    .Ki = 0.8,
    .Kd = 0,
    .OutMax = 120,
    .WindupMax = 150,
    .ThoiGianOnDinhSauMoCua = 50,
    .ThoiGianGiaNhietSauMoCua = 50,
  };
  this->MangCacThongSoPID[6] = {
    .Setpoint = 200,
    .Kp = 100,
    .Ki = 1.0,
    .Kd = 0,
    .OutMax = 150,
    .WindupMax = 180,
    .ThoiGianOnDinhSauMoCua = 40,
    .ThoiGianGiaNhietSauMoCua = 40,
  };
}
void HEATER::KhoiTao(void) {
  // Kéo tất cả các chân CS của cảm biến lên mức cao
  // Tránh lỗi giao tiếp khi không sử dụng các cảm biến khác
  pinMode(spiCS1, OUTPUT);
  pinMode(spiCS2, OUTPUT);
  pinMode(spiCS3, OUTPUT);
  digitalWrite(spiCS1, HIGH);
  digitalWrite(spiCS2, HIGH);
  digitalWrite(spiCS3, HIGH);

  Adafruit_MAX31865::begin(MAX31865_4WIRE);
  Adafruit_MAX31865::enable50Hz(true);
  PID::setOutput(0, 210);
  PID::setWindup(0, 210, 1);
  PID::setSampleTime(1000.0);
  PID::setPIDparamters(50, 1, 10);
  this->KhoiTaoThongSoPID();
  this->TatDieuKhienNhietDo();

  DOOR::KhoiTao();

  pinMode(pinACDET, INPUT);

  triac::configACDETPIN((gpio_num_t)pinACDET);

  triacFan.init();
  triacHeater.init();
  CaiTocDoQuat(1);
  this->TatDieuKhienNhietDo();
  xTaskCreateUniversal(this->TaskDieuKhienNhiet,
                       "Task Incu PID",
                       5120,
                       (void *)this,
                       15,
                       &this->taskPIDHandle,
                       -1);
}

float HEATER::LayThoiGianKichTriac(void)  // Hàm lấy thời gian kích triac
{
  return this->ThoiGianKichTriac;
}

void HEATER::BatDieuKhienNhietDo(void)  // Hàm bật điều khiển nhiệt độ
{
  if (this->TrangThaiDieuKhienNhiet == HEATER_OFF) {
    // digitalWrite(pinFAN, HIGH); // Bật quạt
    this->triacFan.TurnOnTriac();
    this->KhoiDongQuat(TG_KHOI_DONG_QUAT);  // Set dien ap kich toi da cho quat de khoi dong trong khoảng thời gian được truyền vào (ms)
    this->CaiTocDoQuat(this->TocDoQuat);
    this->triacHeater.TurnOnTriac();
    // Serial.println("Batdk");
    this->TrangThaiDieuKhienNhiet = HEATER_ON;  // Chuyển trạng thái điều khiển nhiệt sang ON
  }
}

void HEATER::TatDieuKhienNhietDo(void) {
  this->TrangThaiDieuKhienNhiet = HEATER_OFF;  // Chuyển trạng thái điều khiển nhiệt sang OFF
  // digitalWrite(pinFAN, LOW); // Tắt quạt
  this->triacFan.TurnOnTriac();
  this->triacHeater.TurnOnTriac();
  this->triacHeater.SetTimeOverFlow(9600);
  this->triacFan.SetTimeOverFlow(9600);
  this->triacFan.TurnOffTriac();
  this->triacHeater.TurnOffTriac();
}

void HEATER::TaskDieuKhienNhiet(void *ClassPtr) {
  HEATER *heater_ptr = static_cast<HEATER *>(ClassPtr);  // Ép kiểu con trỏ void về con trỏ class HEATER
  static bool pidActive = true;
  static uint8_t DemThoiGianMoCua = 0;
  static bool TrangThaiCua;
  static unsigned long lastDoorCloseMillis = 0;  // Lưu thời điểm cửa đóng
  static unsigned long lastDoorOpenMillis = 0;

  while (1) {
    heater_ptr->temperature = heater_ptr->LayNhietDoThuc();                        // Cập nhật nhiệt độ thực
    heater_ptr->NhietDoLoc = heater_ptr->updateEstimate(heater_ptr->temperature);  // Cập nhật nhiệt độ lọc
    TrangThaiCua = heater_ptr->TrangThai();                                        // Kiểm tra trạng thái cửa

    // Nếu nhiệt độ nằm trong khoảng +- HE_SO_CHO_PHEP_DAO_DONG_NHIET thì nhiệt sau khi lọc = NhietDoCaiDat
    if (fabs(heater_ptr->NhietDoLoc - heater_ptr->NhietDoCaiDat) <= HE_SO_CHO_PHEP_DAO_DONG_NHIET) {
      heater_ptr->NhietDoLoc = heater_ptr->NhietDoCaiDat;
    }

    switch (TrangThaiCua) {
      case DOOR_CLOSE:
        if (heater_ptr->TrangThaiDieuKhienNhiet != HEATER_ON) {
          break;
        }

        if (pidActive) {
          heater_ptr->TinhToanDieuKhienNhiet(heater_ptr->NhietDoCaiDat);
        } else {
          unsigned long currentMillis = millis();

          // Kiểm tra nếu mới đóng cửa
          if (lastDoorCloseMillis == 0) {
            lastDoorCloseMillis = currentMillis;
            heater_ptr->triacFan.TurnOnTriac();
            heater_ptr->triacHeater.TurnOnTriac();
            heater_ptr->KhoiDongQuat(TG_KHOI_DONG_QUAT);
          }

          unsigned long elapsedTime = (currentMillis - lastDoorCloseMillis) / 1000;

          if (elapsedTime >= heater_ptr->ThoiGianOnDinhSauKhiMoCua) {
            lastDoorCloseMillis = 0;  // Reset thời điểm đóng cửa
            DemThoiGianMoCua = 0;     // Reset thời gian mở cửa
            lastDoorOpenMillis = 0;
            pidActive = true;  // Cho phép bộ điều khiên PID hoạt động
          }

          if (elapsedTime < (DemThoiGianMoCua / 5) && heater_ptr->NhietDoCaiDat > heater_ptr->temperature) {
            heater_ptr->ThoiGianKichTriac = TRIAC_TIME_MIN;
            Serial.println("TriacFull");
          } else if (elapsedTime < heater_ptr->ThoiGianGiaNhietSauKhiMoCua) {
            heater_ptr->ThoiGianKichTriac = 9000;
            Serial.println("Triac-9000");
          }
          heater_ptr->triacHeater.SetTimeOverFlow(heater_ptr->ThoiGianKichTriac);
        }
        break;

      case DOOR_OPEN:
        pidActive = false;
        lastDoorCloseMillis = 0;  // Reset thời điểm cửa đóng
        if (lastDoorOpenMillis == 0) {
          lastDoorOpenMillis = millis();
        } else {
          DemThoiGianMoCua = (millis() - lastDoorOpenMillis) / 1000;
          if (DemThoiGianMoCua >= 30) {
            DemThoiGianMoCua = 30;
          }
        }
        Serial.printf("TgMoCua:%d\n", DemThoiGianMoCua);
        heater_ptr->triacFan.TurnOffTriac();
        heater_ptr->triacHeater.TurnOffTriac();
        heater_ptr->ThoiGianKichTriac = TRIAC_TIME_MAX;
        break;

      default:
        break;
    }
    delay(1000);
  }
}

void HEATER::CaiDatNhietDo(float NhietCai)  // Gọi hàm truyền vào nhiệt độ cài đặt
{
  this->NhietDoCaiDat = NhietCai;
}

float HEATER::LayNhietDoThuc(void)  // Hàm lấy nhiệt độ thực, bao gồm xử lý khi đọc nhiệt độ bị lỗi
{
  static float NhietHienTai, NhietTruocDo;
  static bool CoDocNhietLanDau = 1;
  static uint8_t count = 0;

  NhietHienTai = Adafruit_MAX31865::temperature(RNOMINAL, RREF) + this->HeSoCalib;
  if (NhietHienTai >= 350 || NhietHienTai < 0) {
    this->ResetCamBienNhiet();
    delay(20);
    if (count <= 3) {
      count++;
      return LayNhietDoThuc();
    } else {
      count = 0;
      return 999;
    }
  } else {
    count = 0;
  }
  // Serial.printf("Nhiet:%.2f, Heso:%.2f\n", NhietHienTai, this->HeSoCalib);
  return NhietHienTai;
}

void HEATER::TinhToanPID(float Error)  // hàm tính toán PID
{
  // Tính toán PID rồi sau đó chuyển qua thời gian kích triac trước, để không bị lỗi reset khi sử dụng timer
  this->PIDResult = PID::getPIDcompute(Error);
  // this->ThoiGianKichTriac = map(constrain(this->PIDResult, 0, 9600), 0, 9600, TRIAC_TIME_MAX, TRIAC_TIME_MIN);
  double alpha = acos(2 * pow(this->PIDResult / 220.0, 2) - 1);
  // Kết quả trả về là radian
  this->ThoiGianKichTriac = 10000 * alpha / PI;
}
void HEATER::TinhToanDieuKhienNhiet(float NhietDoCaiDat)  // Hàm chính để tính toán điều khiển nhiệt độ
{
  static bool flag = 0;
  static float temp_old;
  static float NhietDoCaiDatTruocDo = 0;
  static float SaiSo = 0;
  static PID_param_t ThongSoPID;
  if (this->TrangThaiDieuKhienNhiet == HEATER_ON)  // Nếu trạng thái điều khiển nhiệt là ON
  {
    if (NhietDoCaiDat != NhietDoCaiDatTruocDo) {
      // NhietDoCaiDatTruocDo = NhietDoCaiDat;
      ThongSoPID = this->TimThongSoPID(NhietDoCaiDat, this->MangCacThongSoPID, sizeof(this->MangCacThongSoPID) / sizeof(this->MangCacThongSoPID[0]));
      PID::setPIDparamters(ThongSoPID.Kp, ThongSoPID.Ki, ThongSoPID.Kd);  // Thay đổi thông Số PID
      PID::setOutput(0, ThongSoPID.OutMax);
      PID::setWindup(0, ThongSoPID.WindupMax, 2);
      this->ThoiGianOnDinhSauKhiMoCua = ThongSoPID.ThoiGianOnDinhSauMoCua;
      this->ThoiGianGiaNhietSauKhiMoCua = ThongSoPID.ThoiGianGiaNhietSauMoCua;
    }
    SaiSo = NhietDoCaiDat - this->temperature;

    float OutMax = constrain(ThongSoPID.OutMax * (float)map(this->TocDoQuat, 5, 100, 100, 130) / 100.0, 0.0, 210.0);
    float WindupMax = constrain(ThongSoPID.WindupMax * (float)map(this->TocDoQuat, 5, 100, 100, 130) / 100.0, 0.0, 210.0);
    float Kd = ThongSoPID.Kd;
    float Ki = ThongSoPID.Ki * constrain((float)map(this->TocDoQuat, 20, 100, 10, 45) / 10.0, 1.1, 4.2);
    float Kp = OutMax / constrain((float)map(this->TocDoQuat, 10, 100, 18, 25) / 10.0, 1.8, 2.5);
    // Đã điều chỉnh trên tủ 55L 550W
    if (SaiSo > (float)map(this->TocDoQuat, 5, 100, 80, 50) / 10.0) {
      PID::setPIDparamters(200.0 / ((float)map(this->TocDoQuat, 5, 100, 80, 50) / 10.0), 0, 500);  // Thay đổi thông Số PID
      PID::setOutput(0, 200);
      PID::setWindup(0, 80, 2);
    } else if (SaiSo > (float)map(this->TocDoQuat, 5, 100, 45, 20) / 10.0) {
      PID::setPIDparamters(OutMax / ((float)map(this->TocDoQuat, 5, 100, 45, 20) / 10.0), 0, (float)map(this->TocDoQuat, 5, 100, 1000, 300));  // Thay đổi thông Số PID
      PID::setOutput(0, (float)map(this->TocDoQuat, 5, 100, 140, 180));
      PID::setWindup(0, 80, 1);
    } else {
      PID::setPIDparamters(Kp, Ki, Kd);  // Thay đổi thông Số PID
      PID::setOutput(0, OutMax);
      PID::setWindup(0, WindupMax, 0.5);
    }
    this->TinhToanPID(SaiSo);

    // Điều kiện dừng kich triac khi nhiet lố
    if (SaiSo >= -0.1f && SaiSo < 0.0f && NhietDoCaiDat <= 45) {
      this->ThoiGianKichTriac = 9500;
    }
    else if (SaiSo < -0.1f && NhietDoCaiDat <= 45) {
      this->ThoiGianKichTriac = 9600;
    } else if (SaiSo <= -0.1f)  // Nếu nhiệt độ lố 0.1f thì tắt kích triac
    {
      this->ThoiGianKichTriac = 9600;
    }
    else if (SaiSo < -0.2f){
      this->ThoiGianKichTriac = 9600;
    }
  } else {
    this->ThoiGianKichTriac = TRIAC_TIME_MAX;  // Kết quả
  }
  this->triacHeater.SetTimeOverFlow((uint16_t)this->ThoiGianKichTriac);

  // POST thông số về server
  // extern QueueHandle_t dataQueue;
  // if (dataQueue != NULL)
  // {
  //   PIDData exampleData = {NhietDoCaiDat, this->temperature, PID::Kp, PID::Ki, PID::Kd, PID::PTerm, PID::ITerm, PID::DTerm, PID::Output, this->ThoiGianKichTriac, PID::WindupMax, PID::OutMax, ThongSoPID.ThoiGianOnDinhSauMoCua, ThongSoPID.ThoiGianGiaNhietSauMoCua};
  //   xQueueSend(dataQueue, &exampleData, portMAX_DELAY);
  // }
}
void HEATER::CaiGiaTriOfset(float value)  // Cài giá trị Ofset nhiệt độ
{
  this->HeSoCalib = value;
}
float HEATER::LayGiaTriPT100Ofset(void)  // Lấy giá trị ofset nhiệt độ PT100
{
  return this->HeSoCalib;
}
void HEATER::CalibNhietDoPT100(float setTemp, float calibTemp)  // Calib nhiệt độ pt100
{
  this->HeSoCalib = calibTemp - (setTemp - this->HeSoCalib);
}

void HEATER::ResetCamBienNhiet(void) {
  Adafruit_MAX31865::begin(MAX31865_4WIRE);
  Adafruit_MAX31865::enable50Hz(true);
}
float HEATER::LayNhietDoLoc(void) {
  return this->NhietDoLoc;
}
void HEATER::CaiTocDoQuat(uint8_t value) {
  static float DienApQuat = 0;
  if (value == 0) {
    triacFan.SetTimeOverFlow(9600);
  } else {
    this->TocDoQuat = value > 100 ? 100 : value < 5 ? 5
                                                    : value;
    // Tinh goc kich dua tren dien ap

    DienApQuat = map(this->TocDoQuat, 5, 100, 60, 150);

    if (DienApQuat >= 200) {
      triacFan.SetTimeOverFlow((((((-0.000000313052501 * DienApQuat + 0.000233117151) * DienApQuat - 0.0602810289) * DienApQuat + 5.54206941) * DienApQuat + 0.070522428) * DienApQuat + 0.000387562703) * DienApQuat - 2448040.2);
    } else {

      triacFan.SetTimeOverFlow((((((-0.000000000161804792 * DienApQuat + 0.0000000710158223) * DienApQuat - 0.00000978263633) * DienApQuat - 0.000247417252) * DienApQuat + 0.179451126) * DienApQuat - 42.0382603) * DienApQuat + 10331.4366);
    }
  }
}

void HEATER::KhoiDongQuat(uint16_t ThoiGianKhoiDong) {
  uint8_t TocDoQuatTruocDo = this->TocDoQuat;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  CaiTocDoQuat(80);
  // vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ThoiGianKhoiDong));
  delay(ThoiGianKhoiDong);
  CaiTocDoQuat(TocDoQuatTruocDo);
}

bool HEATER::TrangThaiThanhGiaNhiet(void) {
  if (this->ThoiGianKichTriac >= 1900 && this->ThoiGianKichTriac < 9600 && TrangThaiDieuKhienNhiet == HEATER_ON) {
    return true;
  } else {
    return false;
  }
}

float HEATER::TimYc(float x_A, float y_A, float x_B, float y_B, float x_C) {
  float m = (y_B - y_A) / (x_B - x_A);  // Hệ số góc
  float y_C = y_A + m * (x_C - x_A);    // Tính tung độ y_C
  return y_C;
}

PID_param_t HEATER::TimThongSoPID(float sp, PID_param_t *param, int8_t size) {
  PID_param_t pid_param;
  for (int8_t i = size - 1; i > 0; i--) {
    if (sp > param[i - 1].Setpoint && sp < param[i].Setpoint) {
      Serial.printf("sp1: %.2f, sp2: %.2f\n", param[i - 1].Setpoint, param[i].Setpoint);
      pid_param.Kp = TimYc(param[i - 1].Setpoint, param[i - 1].Kp, param[i].Setpoint, param[i].Kp, sp);
      pid_param.Ki = TimYc(param[i - 1].Setpoint, param[i - 1].Ki, param[i].Setpoint, param[i].Ki, sp);
      pid_param.Kd = TimYc(param[i - 1].Setpoint, param[i - 1].Kd, param[i].Setpoint, param[i].Kd, sp);
      pid_param.OutMax = TimYc(param[i - 1].Setpoint, param[i - 1].OutMax, param[i].Setpoint, param[i].OutMax, sp);
      pid_param.WindupMax = TimYc(param[i - 1].Setpoint, param[i - 1].WindupMax, param[i].Setpoint, param[i].WindupMax, sp);
      pid_param.ThoiGianGiaNhietSauMoCua = TimYc(param[i - 1].Setpoint, param[i - 1].ThoiGianGiaNhietSauMoCua, param[i].Setpoint, param[i].ThoiGianGiaNhietSauMoCua, sp);
      pid_param.ThoiGianOnDinhSauMoCua = TimYc(param[i - 1].Setpoint, param[i - 1].ThoiGianOnDinhSauMoCua, param[i].Setpoint, param[i].ThoiGianOnDinhSauMoCua, sp);
      return pid_param;
    } else if (sp == param[i].Setpoint) {
      Serial.printf("sp: %.2f\n", param[i].Setpoint);
      return param[i];
    }
  }
  pid_param.Kp = 0;
  pid_param.Ki = 0;
  pid_param.Kd = 0;
  pid_param.ThoiGianGiaNhietSauMoCua = 0;
  pid_param.ThoiGianOnDinhSauMoCua = 0;
  Serial.println("Chua co he so PID");
  return pid_param;
}