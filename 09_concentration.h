/*
    TODO: Điều khiển nồng độ CO2

    Người viết Nguyễn Trung Thảo
*/

#ifndef _CONTROLCO2_H_
#define _CONTROLCO2_H_

#include "userdef.h"
#include "06_SenSorCO2.h"
#include "OnOffHighResolutionTimer.h"
#include "08_PID.h"
#include <EEPROM.h>

#define CO2_SAMPLE_TIME 10000

#define TAT_CO2 false                           // trạng thái TẮT điều khiển CO2
#define BAT_CO2 true                            //trạng thái cho phép điều khiển CO2

//* các giá trị trả về của cảm biến để báo lỗi từ manual
#define CAM_BIEN_LOI -1.0f
#define CAM_BIEN_DANG_KHOI_TAO -2.0f
#define VUOT_NGUONG_GIA_TRI_DO_CAM_BIEN -3.0f
//* giá trị báo lỗi giao tiếp. Tự quy định
#define LOI_GIAO_TIEP 999.0f

#define co2EEPROM_SIZE 100
#define co2EEPROM_LENGTH_CONFRIM 8

#define concenTHOI_GIAN_MO_VAN_TOI_THIEU 10.0f

typedef struct {
  PIDParam_t xPID;
  char pcConfim[co2EEPROM_LENGTH_CONFRIM];
}ControlParamaterCO2;

class Concentration : public IRCO2, public HRTOnOffPin, public PID, public EEPROMClass {
public:

  Concentration(const char* name = "CO2Parameter");
  ~Concentration() { LuuDuwLieuvaoEEprom(); }

  // điều khiển 
  void KhoiTaoCO2(const uint16_t u16SizeEEprom = co2EEPROM_SIZE);
  void BatDieuKhienCO2();
  void TatDieuKhienCO2();
  void vSuspend();
  void vResume();
  void SetEventDOOR();
  void ResetEventDOOR();

  // lưu trữ 
  void LayDuLieuLuuTru(ControlParamaterCO2* pxControlParamaterCO2);

  // giải thuật
  void CaiNongDoCO2(float giaTriDat = -0.5);
  void vSetControlParamater(ControlParamaterCO2 );

  // cảm biến
  float LayNongDoDatCO2();
  float LayNongDoCO2Thuc();//
  uint32_t LayThoiGianKichVan();//
  bool LayTrangThaiVan();
  void KhoiDongLaiCamBien();
  IRCO2_StatusTydef XoaToanBoGiaTriCalib();
  IRCO2_StatusTydef CalibGiaTriThuc(float giaTriChuan);
  IRCO2_StatusTydef CalibDiem0(float giaTri0Chuan);

  // cơ cấu chấp hành 
  void CalibApSuat(uint32_t thoiGianMoVan);

  // lấy thông số 
  ControlParamaterCO2 xGetControlParamater();

private:
  // điều khiển 
  bool coChayBoDieuKhienCO2;
  uint8_t step = 0;
  uint32_t preCloseDoor = 0;
  TaskHandle_t taskTinhToan;
  static void taskTinhToanCO2(void*);
  void khoiTaoEEpprom(const uint16_t u16SizeEEprom = co2EEPROM_SIZE);

  // lưu trữ 
  void LayDuLieuTuEEprom();
  void LuuDuwLieuvaoEEprom();

  // giải thuật
  float nongDoDat;
  float nongDoThucCO2;
  float nongDoLocCO2;
  ControlParamaterCO2 xControlParamaterCO2;

  // cảm biến
  float thoiGianMoVan;
  float LayGiaTriTuCamBien();

  // cơ cấu chấp hành 
};


#endif // kết thúc _CONTROLCO2_H_