#ifndef _HEATER_H_
#define _HEATER_H_

#ifndef _CHE_DO_MO_CUA_
#define _CHE_DO_MO_CUA_
#endif

#include <Adafruit_MAX31865.h>
#include <math.h>
#include <SimpleKalmanFilter.h>
#include "08_PID.h"
#ifdef _CHE_DO_MO_CUA_
#include "Door.h"
#include "12_triac.h"

#endif

#define RNOMINAL  100
#define RREF      390 // 430


// #define pinACDET  (gpio_num_t)39
// #define pinHEATER 2
// #define pinFAN    15

// #define spiCS     14  //SS1
// // #define spiCS     27   //ss2
// #define spiMOSI   23
// #define spiMISO   19
// #define spiCLK    18

#define pinACDET  (gpio_num_t)12
#define pinHEATER 45
#define pinFAN    48

#define spiCS1    39  //SS1
#define spiCS2    38  //SS2
#define spiCS3    37  //SS3
#define spiMOSI   42
#define spiMISO   41
#define spiCLK    40


#define HEATER_ON   true
#define HEATER_OFF  false

#define TRIAC_TIME_MAX  9600    
#define TRIAC_TIME_MIN  1900 
#define TG_KHOI_DONG_QUAT 300

#define SAI_SO_DO           0.3
#define SAI_SO_UOC_LUONG    0.1
#define HE_SO_NHIEU         1
#define HE_SO_CHO_PHEP_DAO_DONG_NHIET 0.1f

typedef struct 
{
    float Setpoint;
    float Kp;
    float Ki;
    float Kd;
    float OutMax;
    float WindupMax;
    uint8_t ThoiGianOnDinhSauMoCua;
    uint8_t ThoiGianGiaNhietSauMoCua;
} PID_param_t;

#ifdef _CHE_DO_MO_CUA_
class HEATER : public Adafruit_MAX31865, public PID, public SimpleKalmanFilter, public DOOR, public triac
#else 
class HEATER : public Adafruit_MAX31865, public PID, public SimpleKalmanFilter, public triac
#endif
{
private:
// public:
  uint8_t TocDoQuat = 1;
  triac triacFan;
  triac triacHeater;
  float PIDResult;
  float ThoiGianKichTriac = 9600;
  bool  TrangThaiDieuKhienNhiet;
  float NhietDoCaiDat;
  float temperature;
  float NhietDoLoc;
  float HeSoCalib = 0;
  uint8_t CoBaoDocNhiet = 1;
  uint8_t ThoiGianOnDinhSauKhiMoCua = 30;
  uint8_t ThoiGianGiaNhietSauKhiMoCua = 60;
  PID_param_t MangCacThongSoPID[7];

  //Mới thêm
  float SaiSoNhietChoPhep;

  TimerHandle_t timerIncu = NULL;
  esp_timer_handle_t tTriacHandle;
  TaskHandle_t taskTriacHandle;
  TaskHandle_t taskPIDHandle;
  esp_timer_create_args_t timer_arg;

  void TinhToanPID(float);
  void TinhToanDieuKhienNhiet(float);
  static void TaskDieuKhienNhiet(void*);
  float LayThoiGianKichTriac(void);
  float TimYc(float x_A, float y_A, float x_B, float y_B, float x_C); // de tim thong so PID tuyen tinh hoa 2 diem
  PID_param_t TimThongSoPID(float sp, PID_param_t *param, int8_t size);
public:
  HEATER();
  void KhoiTao(void);                 // Khởi tạo trong setup
  void KhoiTaoThongSoPID(void);

  void CaiDatNhietDo(float);          // Set nhiệt độ setpoint
  void CaiGiaTriOfset(float);         // Gọi trong setup sau khởi tạo, truyền vào giá trị ofset đã được lưu vào epprom

  float LayNhietDoThuc(void);         // Đọc nhiệt độ thực tế
  float LayNhietDoLoc(void);          // Đọc nhiệt độ sau khi lọc dùng để hiển thị
  float LayGiaTriPT100Ofset(void);    // Hàm trả về giá trị ofset nhiệt độ sau khi calib, giá trị này dùng để lưu vào epprom

  void BatDieuKhienNhietDo(void);     // Bật gia nhiệt
  void TatDieuKhienNhietDo(void);     // Tắt gia nhiệt

  void CalibNhietDoPT100(float, float);      // hàm calib tham số đầu tiên là nhiệt độ cài, tham số thứ 2 là nhiệt độ được đo bằng máy chuẩn
  void ResetCamBienNhiet(void);              // khởi động lại PT100 khi đọc nhiệt độ bị lỗi
  void CaiTocDoQuat(uint8_t value = 10);
  void KhoiDongQuat(uint16_t);
  bool TrangThaiThanhGiaNhiet(void);
};
#endif
