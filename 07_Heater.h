#ifndef _HEATER_H_
#define _HEATER_H_

#ifndef _CHE_DO_MO_CUA_
#define _CHE_DO_MO_CUA_
#endif

#include <Arduino.h>
#include <Adafruit_MAX31865.h>
#include <EEPROM.h>
#include <math.h>
#include <SimpleKalmanFilter.h>
#include "08_PID.h"
#include "Door.h"
#include "12_triac.h"
#include "AnhLABV01HardWare.h"
#include "OnOffHighResolutionTimer.h"
#include "userdef.h"

#define MAX31865_DIEN_TRO_THAM_CHIEU 390.0
#define MAX31865_DIEN_TRO_CAM_BIEN 100.0
#define MAX31865_NHIET_DO_TOI_DA 500.0

#define OUT_MAX_POWER 1000
#define SAMPLE_TIME 1000

#define HEATER_ON   true
#define HEATER_OFF  false

#define TRIAC_TIME_MAX  9600    
#define TRIAC_TIME_MIN  1900 
#define TG_KHOI_DONG_QUAT 300

#define SAI_SO_DO           0.3
#define SAI_SO_UOC_LUONG    0.1
#define HE_SO_NHIEU         1
#define HE_SO_CHO_PHEP_DAO_DONG_NHIET 0.1f

#define heaterFLAG_ON_OFF (1<<1)
#define heaterFLAG_INITED (1<<0)
#define heaterFLAG_INIT_EEPROM (1<<2)
#define heaterFLAG_INIT_SENSOR (1<<3)
#define heaterFLAG_INTI_ACTUATOR (1<<4)

#define heaterEEPROM_SIZE 100
#define heaterEEPROM_LENGTH_CONFRIM 8

enum triacName {
  eTriac1 = 0,
  eTriac3,
  eTriac4,

  eTriacMax,
};

enum StepControl_t {
  eOFF = 0,
  eWAIT_BEFORE_RUN,
  eRUN,
  eOPEN_DOOR,
  eCLOSE_DOOR,
};

typedef struct {
  PIDParam_t xPID;
  uint16_t Perimter;
  uint16_t Door;
  char pcConfim[heaterEEPROM_LENGTH_CONFRIM];
}ControlParamaterTEMP;

typedef void (*CallBackACDET_t)(void*);

class HEATER : public PID, public triac, public EEPROMClass
{

public:
  
  HEATER(const char* name = "HeaterParameter");
  ~HEATER() { LuuDuwLieuvaoEEprom(); }

  //điều khiển nhiệt 
  void KhoiTao(const uint16_t u16SizeEEprom = heaterEEPROM_SIZE);
  void BatDieuKhienNhietDo(void);
  void TatDieuKhienNhietDo(void);
  void vSuspend();
  void vResume();
  void SetEventDOOR();
  void ResetEventDOOR();

  // lưu trữ 
  void LayDuLieuLuuTru(ControlParamaterTEMP* pxControlParamaterTEMP);

  // giải thuật
  void CaiDatNhietDo(float);
  void vSetControlParamater(ControlParamaterTEMP);

  // cảm biến 
  void CaiGiaTriOfset(float);
  void ResetCamBienNhiet(void);

  // cơ cấu chấp hành 
  void CaiTocDoQuat(uint8_t value = 10);
  void KhoiDongQuat(uint16_t);
  void addCallBackACDET(CallBackACDET_t pCallBack, void* pArg = NULL);
  void addCallBackTimeOutTriacBuong(CallBack_t pCallBack, void* pArg = NULL) {
    triacBuong.addCallBackTimeout(pCallBack, pArg);
  }
  void addCallBackWritePinTriacBuong(CallBack_t pCallBack, void* pArg = NULL) {
    triacBuong.addCallBackWritePin(pCallBack, pArg);
  }

  // truy xuất dữ liệu
  float LayNhietDoLoc(void);
  float LayGiaTriDieuKhienCua() const {
    return u16MaxThoiGianBatCua;
  }
  float LayGiaTriDieuKhienVanh() const {
    return u16MaxThoiGianBatVanh;
  }
  bool TrangThaiThanhGiaNhiet(void);
  bool TrangThaiQuat(void);
  bool CheckNguonCongSuat();
  ControlParamaterTEMP xGetControlParamater();

private:
  // điều khiển nhiệt
  uint8_t u8SysFlagGroup = 0;
  uint8_t step = 0;
  TaskHandle_t taskHandleDieuKhienNhiet;
  static void TaskDieuKhienNhiet(void* ptr);
  void KhoiTaoCamBien();
  float DocGiaTriCamBien(Adafruit_MAX31865& xPt100);
  void KhoiTaoTriac(void);
  void khoiTaoEEpprom(const uint16_t u16SizeEEprom = heaterEEPROM_SIZE);

  // lưu trữ 
  void LayDuLieuTuEEprom();
  void LuuDuwLieuvaoEEprom();

  // giải thuật 
  float NhietDoCaiDat;
  float NhietDoLocBuong;
  ControlParamaterTEMP xControlParamaterTEMP;
  uint32_t preOpenDoor = 0;
  uint32_t preCloseDoor = 0;
  int16_t u16ThoiGianBatBuong = 0;
  int16_t u16ThoiGianBatVanh = 0;
  int16_t u16ThoiGianBatCua = 0;
  int16_t u16MaxThoiGianBatVanh = xControlParamaterTEMP.Perimter;
  int16_t u16MaxThoiGianBatCua = xControlParamaterTEMP.Door;

  // cảm biến 
  Adafruit_MAX31865 PT100_buong = Adafruit_MAX31865(PT100_CS1_PIN, PT100_MOSI_PIN, PT100_MISO_PIN, PT100_SCK_PIN);
  SimpleKalmanFilter LocCamBienBuong = SimpleKalmanFilter(SAI_SO_DO, SAI_SO_UOC_LUONG, HE_SO_NHIEU);
  Adafruit_MAX31865 PT100_2 = Adafruit_MAX31865(PT100_CS2_PIN, PT100_MOSI_PIN, PT100_MISO_PIN, PT100_SCK_PIN);
  Adafruit_MAX31865 PT100_3 = Adafruit_MAX31865(PT100_CS3_PIN, PT100_MOSI_PIN, PT100_MISO_PIN, PT100_SCK_PIN);
  float HeSoCalib = 0;

  // cơ cấu chấp hành 
  volatile bool isACDET;
  void* m_pArgACDET;
  TaskHandle_t taskHandleNgatACDET;
  CallBackACDET_t m_pCallBackACDET;
  uint8_t TocDoQuat = 10;
  HRTOnOffPin triacBuong = HRTOnOffPin(TRIAC1_PIN);
  HRTOnOffPin triacCua = HRTOnOffPin(TRIAC2_PIN);
  HRTOnOffPin triacVanh = HRTOnOffPin(TRIAC3_PIN);
  HRTOnOffPin* pArrTriac[eTriacMax] = { &triacBuong, &triacCua, &triacVanh };
  volatile uint16_t pArru16ThoiGianKichTriac[eTriacMax];
  static void TaskNgatACDET(void* ptr);
  static void interupt(void* ptr);
};
#endif
