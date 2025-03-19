/*
    TODO: Điều khiển nồng độ CO2

    Người viết Nguyễn Trung Thảo
*/

#ifndef _CONTROLCO2_H_
#define _CONTROLCO2_H_

#include "Config.h"
#include "06_SenSorCO2.h"
#include "OnOffHighResolutionTimer.h"
#include "08_PID.h"

#define SETPONIT_CO2 5
#define CO2_KP 1000
#define CO2_KI 0.005
#define CO2_KD 0
#define CO2_KW 0  //0.05
#define CO2_OUT_MAX 4000
#define CO2_OUT_MIN 0
#define CO2_WIN_MAX 1000
#define CO2_WIN_MIN 0
#define CO2_SAMPLE_TIME 10000

// #define debug
#define TU_CO2_CO_QUAT

#define TAT_CO2 false                           // trạng thái TẮT điều khiển CO2
#define BAT_CO2 true                            //trạng thái cho phép điều khiển CO2

//* các giá trị trả về của cảm biến để báo lỗi từ manual
#define CAM_BIEN_LOI -1.0f
#define CAM_BIEN_DANG_KHOI_TAO -2.0f
#define VUOT_NGUONG_GIA_TRI_DO_CAM_BIEN -3.0f
//* giá trị báo lỗi giao tiếp. Tự quy định
#define LOI_GIAO_TIEP 999.0f
//* giá trị sai số mong muốn của bộ điều khiển 
#define SAI_SO_MONG_MUON 0.1f

#define THOI_GIAN_MO_VAN_TOI_THIEU 10.0f


#if defined(ON_OFF_DEBUG) && defined(debug) && defined(DEBUG_CO2)
#define CO2_SerialPrintf(flag, string, ...) \
  do { \
    if (flag) \
      Serial.printf(string, ##__VA_ARGS__); \
  } while (0)

#define CO2_SerialPrintln(flag, string, ...) \
  do { \
    if (flag) \
      Serial.println(string, ##__VA_ARGS__); \
  } while (0)
#define CO2_SerialPrint(flag, string, ...) \
  do { \
    if (flag) \
      Serial.print(string, ##__VA_ARGS__); \
  } while (0)

#elif !defined(ON_OFF_DEBUG) && defined(debug) && defined(DEBUG_CO2)

#define CO2_SerialPrintf(flag, string, ...) Serial.printf(string, ##__VA_ARGS__)
#define CO2_SerialPrintln(flag, string, ...) Serial.println(string, ##__VA_ARGS__)
#define CO2_SerialPrint(flag, string, ...) Serial.print(string, ##__VA_ARGS__)

#else 

#define CO2_SerialPrintf(flag, string, ...) (void*)0
#define CO2_SerialPrintln(flag, string, ...) (void*)0
#define CO2_SerialPrint(flag, string, ...) (void*)0

#endif

class Concentration : public IRCO2, public HRTOnOffPin, public PID {
public:

  Concentration(void);

    void KhoiTaoCO2();

    void CaiNongDoCO2(float giaTriDat = -0.5);//
    
    float LayNongDoDatCO2();
    float LayNongDoCO2Thuc();//
    uint32_t LayThoiGianKichVan();//
    bool LayTrangThaiVan();

    void BatDieuKhienCO2();//
    void TatDieuKhienCO2();//
    void SetEventDOOR();
    void ResetEventDOOR();
    
    // void logDEBUG();
    
    void KhoiDongLaiCamBien();
    IRCO2_StatusTydef XoaToanBoGiaTriCalib();
    IRCO2_StatusTydef CalibGiaTriThuc(float giaTriChuan);
    IRCO2_StatusTydef CalibDiem0(float giaTri0Chuan);
    void CalibApSuat(uint32_t thoiGianMoVan);
private:
    float LayGiaTriTuCamBien();
    float nongDoDat;
    uint32_t thoiGianLayMau;
    float nongDoThucCO2;
    float nongDoLocCO2;
    float thoiGianMoVan;
    float ketQuaDocCamBien;
    uint16_t thoiGianCho = 0; // đếm thời gian lấy mẫu, đơn vị 1s 
    uint8_t step = 0;
    uint32_t preCloseDoor = 0;

    // float saiSo, saiSoTruocDo;
    bool coChayBoDieuKhienCO2;
    TaskHandle_t taskTinhToan;
    TimerHandle_t TimerLayMauCO2;
    static void taskTinhToanCO2(void*);

};


#endif // kết thúc _CONTROLCO2_H_