#ifndef _HEATER_H_
#define _HEATER_H_

#ifndef _CHE_DO_MO_CUA_
#define _CHE_DO_MO_CUA_
#endif

#include <Adafruit_MAX31865.h>
#include <math.h>
#include <SimpleKalmanFilter.h>
#include "08_PID.h"
#include "Door.h"
#include "12_triac.h"
#include "AnhLABV01HardWare.h"
#include "OnOffHighResolutionTimer.h"

#define MAX31865_DIEN_TRO_THAM_CHIEU 390.0
#define MAX31865_DIEN_TRO_CAM_BIEN 100.0
#define MAX31865_NHIET_DO_TOI_DA 500.0

#define KP 5.0f     //8; //8.5
#define KI 0.001     // 0.01
#define KD 0        //30.0f
#define KW 0
#define OUT_MAX 17  // số chu kỳ kích
#define OUT_MIN 0
#define WIN_MAX 10
#define WIN_MIN 0
#define SAMPLE_TIME 1000
#define OUT_MAX_POWER 1000
#define THOI_GIAN_BAT_TRIAC_CUA 40 // 170
#define THOI_GIAN_BAT_TRIAC_VANH 200 // 310

#define HEATER_ON   true
#define HEATER_OFF  false

#define TRIAC_TIME_MAX  9600    
#define TRIAC_TIME_MIN  1900 
#define TG_KHOI_DONG_QUAT 300

#define SAI_SO_DO           0.3
#define SAI_SO_UOC_LUONG    0.1
#define HE_SO_NHIEU         1
#define HE_SO_CHO_PHEP_DAO_DONG_NHIET 0.1f

enum triacName {
  eTriac1 = 0,
  eTriac3,
  eTriac4,

  eTriacMax,
};

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

typedef void (*CallBackACDET_t)(void*);

class HEATER : public PID, public triac
{

public:
  HEATER();
  void KhoiTao(void);                 // Khởi tạo trong setup

  void CaiDatNhietDo(float);          // Set nhiệt độ setpoint
  void CaiGiaTriOfset(float);         // Gọi trong setup sau khởi tạo, truyền vào giá trị ofset đã được lưu vào epprom

  float LayNhietDoLoc(void);          // Đọc nhiệt độ sau khi lọc dùng để hiển thị
  float LayGiaTriPT100Ofset(void);    // Hàm trả về giá trị ofset nhiệt độ sau khi calib, giá trị này dùng để lưu vào epprom

  void BatDieuKhienNhietDo(void);     // Bật gia nhiệt
  void TatDieuKhienNhietDo(void);     // Tắt gia nhiệt

  void CalibNhietDoPT100(float, float);      // hàm calib tham số đầu tiên là nhiệt độ cài, tham số thứ 2 là nhiệt độ được đo bằng máy chuẩn
  void ResetCamBienNhiet(void);              // khởi động lại PT100 khi đọc nhiệt độ bị lỗi
  void CaiTocDoQuat(uint8_t value = 10);
  void KhoiDongQuat(uint16_t);

  bool TrangThaiThanhGiaNhiet(void);
  bool TrangThaiQuat(void);
  bool CheckNguonCongSuat();
  void addCallBackACDET(CallBackACDET_t pCallBack, void* pArg = NULL);
  void addCallBackTimeOutTriacBuong(CallBack_t pCallBack, void* pArg = NULL) {
    triacBuong.addCallBackTimeout(pCallBack, pArg);
  }
  void addCallBackWritePinTriacBuong(CallBack_t pCallBack, void* pArg = NULL) {
    triacBuong.addCallBackWritePin(pCallBack, pArg);
  }
private:
  uint8_t step = 0;
  uint32_t preOpenDoor = 0;
  uint32_t preCloseDoor = 0;

  HRTOnOffPin triacBuong = HRTOnOffPin(TRIAC1_PIN);
  HRTOnOffPin triacCua = HRTOnOffPin(TRIAC3_PIN);
  HRTOnOffPin triacVanh = HRTOnOffPin(TRIAC4_PIN);
  Adafruit_MAX31865 PT100_buong = Adafruit_MAX31865(PT100_CS1_PIN, PT100_MOSI_PIN, PT100_MISO_PIN, PT100_SCK_PIN);
  Adafruit_MAX31865 PT100_2 = Adafruit_MAX31865(PT100_CS2_PIN, PT100_MOSI_PIN, PT100_MISO_PIN, PT100_SCK_PIN);
  Adafruit_MAX31865 PT100_3 = Adafruit_MAX31865(PT100_CS3_PIN, PT100_MOSI_PIN, PT100_MISO_PIN, PT100_SCK_PIN);
  SimpleKalmanFilter LocCamBienBuong = SimpleKalmanFilter(SAI_SO_DO, SAI_SO_UOC_LUONG, HE_SO_NHIEU);

  volatile bool isACDET;
  uint8_t TocDoQuat = 1;
  bool  TrangThaiDieuKhienNhiet;
  float NhietDoCaiDat;
  float temperature;
  float NhietDoLocBuong;
  float HeSoCalib = 0;
  uint8_t CoBaoDocNhiet = 1;
  uint8_t ThoiGianOnDinhSauKhiMoCua = 30;
  uint8_t ThoiGianGiaNhietSauKhiMoCua = 60;
  volatile uint16_t pArru16ThoiGianKichTriac[eTriacMax];
  HRTOnOffPin* pArrTriac[eTriacMax] = { &triacBuong, &triacCua, &triacVanh };

  //Mới thêm
  float SaiSoNhietChoPhep;

  CallBackACDET_t m_pCallBackACDET;
  void *m_pArgACDET;

  TaskHandle_t taskHandleDieuKhienNhiet;
  TaskHandle_t taskHandleNgatACDET;

  static void TaskDieuKhienNhiet(void* ptr);
  static void IRAM_ATTR interupt(void* ptr);
  static void TaskNgatACDET(void* ptr);

  void KhoiTaoThongSoPID(void);
  void KhoiTaoCamBien();
  float DocGiaTriCamBien(Adafruit_MAX31865& xPt100);
  void KhoiTaoTriac(void);

};
#endif
