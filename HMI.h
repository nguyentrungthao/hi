#ifndef _HMI_H_
#define _HMI_H_

#include "DWIN_custom.h"
#include "freertos/queue.h"
#include <vector>
#include <TimeLib.h>
#include <userdef.h>
#include "HMIparam.h"
#include <Arduino.h>

#define MANUFACTURER_PASSWORD "LABone2025"

typedef enum
{
    UNDEFINED,
    HMI_SET_RUN_ONOFF,
    HMI_SET_FAN,
    HMI_SET_SETPOINT_TEMP,
    HMI_SET_SETPOINT_CO2,
    HMI_SET_CALIB_NHIET,
    HMI_SET_CALIB_SPAN_CO2,
    HMI_SET_CALIB_ZERO_CO2,
    HMI_RESET_CALIB_NHIET,
    HMI_RESET_CALIB_CO2,
    // HMI_SET_FLAP,
    HMI_SET_DELAYOFF,
    HMI_SET_DELAYOFF_ONOFF,
    HMI_SET_TEMPMAX,
    HMI_SET_TEMPMIN,
    HMI_SET_RTC,
    HMI_SET_ALARM_TEMP_BELOW,
    HMI_SET_ALARM_TEMP_ABOVE,
    HMI_SET_ALARM_CO2_BELOW,
    HMI_SET_ALARM_CO2_ABOVE,
    HMI_SET_TEXT,
    HMI_EDIT_SEG_SETPOINT_TEMP,
    HMI_EDIT_SEG_SETPOINT_CO2,
    HMI_EDIT_SEG_FANSPEED,
    HMI_EDIT_SEG_AIRFLAP,
    HMI_EDIT_SEG_TEMPMIN,
    HMI_EDIT_SEG_TEMPMAX,
    HMI_EDIT_SEG_CO2MIN,
    HMI_EDIT_SEG_CO2MAX,
    HMI_EDIT_SEG_DELAYOFF_DAY,
    HMI_EDIT_SEG_DELAYOFF_HOUR,
    HMI_EDIT_SEG_DELAYOFF_MINUTE,
    HMI_SELECT_SEGMENT,
    HMI_ADD_SEG,
    HMI_SUB_SEG,
    HMI_EDIT_SEG_DAY,
    HMI_EDIT_SEG_HOUR,
    HMI_EDIT_SEG_MINUTE,
    HMI_SAVE_SEG,
    HMI_ADD_PROGRAM,
    HMI_DELETE_PROGRAM,
    HMI_SELECT_PROGRAM,
    HMI_SELECT_PROGRAMNAME,
    HMI_RUN_PROGRAM_MODE,
    HMI_RUN_QUICK_MODE,
    HMI_RESET_STER,
    HMI_SET_STER_TEMP,
    HMI_SET_STER_TIME,
    HMI_SET_ICON1,
    HMI_SET_ICON2,
    HMI_EXPORT_DATA,
    HMI_FIRMWARE_USB,
    HMI_FIRMWARE_FOTA,
    HMI_SET_SSID,
    HMI_SET_PASSWORD,
    HMI_SET_SCROLLCHART,
    HMI_CONNECT_OR_DISCONNECT_WIFI, // truc them
    HMI_CHANGE_ADMIN_PASSWORD,
    eHMI_SET_EVENT_WAKEUP,

    HMI_SET_MAX_ENUM
} hmi_set_type_t;
typedef enum
{
    HMI_GET_SEGMENT_LIST,
    HMI_GET_NEXT_SEGMENT_LIST,
    HMI_GET_BACK_SEGMENT_LIST,
    HMI_REFRESH_SEGMENT_LIST,
    HMI_GET_SEGMENT_DELAYOFF,
    HMI_GET_FLAP,
    HMI_GET_RTC,
    HMI_GET_ALARM,
    HMI_GET_PROGRAM_LIST,
    HMI_GET_NEXT_PROGRAM_LIST,
    HMI_GET_BACK_PROGRAM_LIST,
    HMI_GET_DELAYOFF,
    HMI_GET_CALIB,
    HMI_CHECK_LIST,
    HMI_GET_SCAN_SSID_WIFI,

    eHMI_EVENT_ICON_NHIET,
    eHMI_EVENT_ICON_CO2,
    eHMI_EVENT_ICON_CUA,
    eHMI_EVENT_ICON_FAN,

    eHMI_EVENT_HIEN_THI_GIA_TRI_CAM_BIEN,
    eHMI_EVENT_HIEN_THI_THOI_GIAN,
    eHMI_EVENT_ICON_USB,
    eHMI_EVENT_ICON_WIFI,
    eHMI_EVENT_VE_DO_THI,
    eHMI_EVENT_WARNING,
    eHMI_EVENT_REFRESH,

    eHMI_EVENT_TIMEROUT_OFF,

    HMI_GET_MAX_ENUM
} hmi_get_type_t;

typedef enum
{
    HMI_FLOAT,
    HMI_INT,
    HMI_DATE,
    HMI_TEXT,
    HMI_PASSWORD,
    HMI_EXTERNAL_PASSWORD, // truc them
} hmi_data_type_t;

typedef struct
{
    hmi_set_type_t type;
    union
    {
        float f_value;
        int32_t i32_value;
        uint32_t u32_value;
    };
    float maxValue;
    float minValue;
    uint16_t pageAfterEnter;
    uint16_t pageAfterReturn;
    uint16_t VPTextDisplayAfterEnter;
    uint16_t VPTextDisplayWhenInput;
    hmi_data_type_t displayType;
    uint8_t indexList;
    String text;
    int8_t textLen;
} hmi_set_event_t;

typedef std::function<void(const hmi_set_event_t& event)> hmiSetData_t;
typedef std::function<bool(hmi_get_type_t get, void* args)> hmiGetData_t;
// typedef std::function<void(hmi_get_type_t get, int start, int length)> hmiGetDataList_t;

class HMI : public DWIN
{
public:
    HMI(HardwareSerial& port, uint8_t receivePin, uint8_t transmitPin, long baud = DWIN_DEFAULT_BAUD_RATE);
    void KhoiTao(void);
    void DangKyHamSetCallback(hmiSetData_t function);
    void DangKyHamGetCallback(hmiGetData_t function);

    void HienThiDuLieuSegmentTrenHang(uint8_t row, uint8_t index, float setpoint, float setpointCO2, int delayOffDay, int delayOffHour, int delayOffMinute,
        int fanSpeed, float tempMin, float tempMax, float CO2Min, float CO2Max);
    void XoaDuLieuHienThiSegmentTrenHang(uint8_t row);

    void HienThiNgay(int ngay);
    void HienThiThang(int thang);
    void HienThiNam(int nam);
    void HienThiGio(int gio);
    void HienThiPhut(int phut);

    void HienThiNhietDoCanhBao(float NhietDuoi, float NhietTren);
    void HienThiCO2CanhBao(float CO2Duoi, float CO2Tren);

    void HienThiTenChuongTrinhTrenHang(uint8_t row, uint8_t index, String name, uint8_t TotalSeg, const char* func = "");
    void XoaDuLieuHienThiTenChuongTrinhTrenHang(uint8_t row);

    void VeDoThi(float value, time_t time, int offset);
    void VeDoThiNhiet(float value, time_t time, int offset);
    void VeDoThiCO2(float value, time_t time, int offset);
    void VeDoThi(BaseProgram_t data);
    void setupDoThiDoiSetpoint(BaseProgram_t data);
    void XoaDoThi(void);

    void HienThiNhietDo(float GiaTri);
    void HienThiNhietDo(String text);
    void HienThiCO2(float GiaTri);
    void HienThiCO2(String text);
    void HienThiSetpointTemp(float GiaTri);
    void HienThiSetpointCO2(float GiaTri);
    void HienThiTocDoQuat(int GiaTri);
    // void HienThiGocFlap(int GiaTri);
    void HienThiThoiGianChay(int ngay, int gio, int phut, int giay);
    void HienThiThoiGianChay(String text);
    void HienThiThoiGianRTC(int ngay, int thang, int nam, int gio, int phut, int giay);
    void HienThiIconQuat(bool TrangThai);
    void HienThiIconCua(bool TrangThai);
    void HienThiIconGiaNhiet(bool TrangThai);
    void HienThiIconVanCO2(bool TrangThai);
    void HienThiIconTrangThaiRun(bool TrangThai);
    void HienThiIconOnOffDelayOff(bool TrangThai);
    void HienThiChuongTrinhDangChay(String text);
    void HienThiTenProgramDangEdit(String text);
    void HienThiSegmentDangChay(int SegmentDangChay, int TongSoSegment);
    void HienThiSegmentDangChay(String text);
    void HienThiIconSegment(bool TrangThai);
    void HienThiVongLapChuongTrinhConLai(int GiaTri, int Tong);
    void HienThiVongLapChuongTrinhConLai(String text);
    void HienThiHeSoCalib(float GiaTri);
    void HienThiWarning(std::vector<String> warningVector, uint8_t TrangSauKhiReturn);
    void HienThiWarning(String text, uint8_t TrangSauKhiReturn);
    void HienThiIconRemoveWater(bool TrangThai);
    void HienThiIconRemoveSample(bool TrangThai);
    void HienThiIconConfirm1(bool TrangThai);
    void HienThiIconConfirm2(bool TrangThai);
    void Buzzer(int value);
    void HienThiThanhLoading(uint8_t PhanTram);
    void HienThiPhanTramThanhLoading(uint8_t PhanTram);
    void HienThiPhanTramThanhLoading(String text);
    void HienThiThongTinVersion(String text);
    void HienThiThongTinUpdate(String text);
    void HienThiTrangThaiKetNoiWiFi(String text); // truc them
    void HienThiSSIDWiFi(String text);
    void HienThiPasswordWiFi(String text);
    void HienThiListSSIDWifi(std::vector<String> vectorSSID);
    void HienThiIconUSB(bool TrangThai);
    bool SoSanhPassWord(String EnteredPassword);
    void ThayDoiUserAdminPassword(String EnteredPassword);
    void HienThiCheckAdminPasswordState(String text);
protected:
    HardwareSerial* _hmiSerial;
    hmiSetData_t _hmiSetDataCallback;
    hmiGetData_t _hmiGetDataCallback;
    SemaphoreHandle_t _lock;

    String _ChuoiBanPhimDangNhap;
    bool _CapslockEnable;
    hmi_set_event_t _set_event;

    String _TenChuongTrinh;
    String _TenChuongTrinhTruocDo;
    // int _currentProgram;
    int32_t _ChiMucChuongTrinhTruocDo;

    int32_t _ChiMucPhanDoanTruocDo;

    uint32_t _GioTietTrung;
    uint32_t _PhutTietTrung;
    float _NhietDoTietTrung;
    int8_t _DemTrangThaiNext;
    String _ChuoiPassword;
    uint8_t _TrangSauKhiNhanReturnTrenWarning;

    struct DuLieuDoThi_t
    {
        int YCentral;
        int VDCentral;
        int MulY;
        float maxValue = 400;
        float minValue = 200;
        float valueStep = 0;
        time_t timeArr[9];
        int32_t valueArr[6];
        int16_t count = 0;
        bool flag = 0;
    };
    DuLieuDoThi_t _DuLieuDoThiNhietDo;
    DuLieuDoThi_t _DuLieuDoThiCO2;

    void _createHmiListenTask(void* args);
    static void _hmiListenTask(void* args);
    static void _hmiUartEvent(void);

    static void _NutVaoChucNangChonCalib_(int32_t lastBytes, void* args);
    static void _NutVaoChucNangCalibNhiet_(int32_t lastBytes, void* args);
    static void _NutVaoChucNangCalibCO2_(int32_t lastBytes, void* args);
    static void _NutEditCalibTemp_(int32_t lastBytes, void* args);
    static void _NutEditCalibSpanCO2_(int32_t lastBytes, void* args);
    static void _NutEditCalibZeroCO2_(int32_t lastBytes, void* args);
    static void _NutEnterTrangCalibNhiet_(int32_t lastBytes, void* args);
    static void _NutEnterTrangCalibCO2_(int32_t lastBytes, void* args);
    static void _NutResetHeSoCalibNhiet_(int32_t lastBytes, void* args);
    static void _NutResetHeSoCalibCO2_(int32_t lastBytes, void* args);

    static void _NutThucDay_(int32_t lastBytes, void* args);

    static void _NutCaiDatThoiGianRTC_(int32_t lastBytes, void* args);
    static void _NutEnterTrangCaiRTC_(int32_t lastBytes, void* args);

    static void _NutCaiCanhBaoNhietDo_(int32_t lastBytes, void* args);
    static void _NutEnterTrangCaiCanhBaoNhietDo_(int32_t lastBytes, void* args);
    static void _NutCaiCanhBaoNhietDoThap_(int32_t lastBytes, void* args);
    static void _NutCaiCanhBaoNhietDoCao_(int32_t lastBytes, void* args);
    static void _NutCaiCanhBaoCO2Thap_(int32_t lastBytes, void* args);
    static void _NutCaiCanhBaoCO2Cao_(int32_t lastBytes, void* args);

    static void _NutVaoChucNangProgram_(int32_t lastBytes, void* args);
    static void _NutChonProgram_(int32_t lastBytes, void* args);
    static void _CacNutThaoTacTrongTrangProgram_(int32_t lastBytes, void* args);
    static void _NutChonSegment_(int32_t lastBytes, void* args);

    static void _NutCaiNhietDoSetpoint_(int32_t lastBytes, void* args);
    static void _NutCaiCO2Setpoint_(int32_t lastBytes, void* args);
    static void _NutCaiTocDoQuat_(int32_t lastBytes, void* args);
    static void _NutCaiThoiGianTatMay_(int32_t lastBytes, void* args);
    static void _NutEnterCaiThoiGianTatMay_(int32_t lastBytes, void* args);

    static void _XuLyBanPhim_(int32_t lastBytes, void* args);
    // static void _SegmentButton_(int32_t lastBytes, void *args);

    static void _NutEditSetpointTrangSegment_(int32_t lastBytes, void* args);
    static void _NutEditSetpointCO2TrangSegment_(int32_t lastBytes, void* args);

    static void _NutEditTocDoQuatTrangSegment_(int32_t lastBytes, void* args);
    // static void _NutEditFlapTrangSegment_(int32_t lastBytes, void *args);
    static void _NutEditTempMinTrangSegment_(int32_t lastBytes, void* args);
    static void _NutEditTempMaxTrangSegment_(int32_t lastBytes, void* args);
    static void _NutEditCO2MinTrangSegment_(int32_t lastBytes, void* args);
    static void _NutEditCO2MaxTrangSegment_(int32_t lastBytes, void* args);

    static void _NutEditThoiGianTatTrangSegment_(int32_t lastBytes, void* args);
    static void _NutEnterTrongCaiDatThoiGianTatCuaSegment_(int32_t lastBytes, void* args);
    static void _CacNutThaoTacTrongTrangEditSegment_(int32_t lastBytes, void* args);

    // static void _SegmentDownButton_(int32_t lastBytes, void *args);
    // static void _SegmentUpButton_(int32_t lastBytes, void *args);
    // static void _AddSegment_(int32_t lastBytes, void *args);
    // static void _SubSegment_(int32_t lastBytes, void *args);

    static void _NutVaoTrangNhapNgay_(int32_t lastBytes, void* args);
    static void _NutVaoTrangNhapThang_(int32_t lastBytes, void* args);
    static void _NutVaoTrangNhapNam_(int32_t lastBytes, void* args);
    static void _NutVaoTrangNhapGio_(int32_t lastBytes, void* args);
    static void _NutVaoTrangNhapPhut_(int32_t lastBytes, void* args);

    static void _NutRun_(int32_t lastBytes, void* args);
    static void _NutThayDoiTrangThaiChucNangHenGioTat_(int32_t lastBytes, void* args);

    static void _NutEnterTrongTrangProgram_(int32_t lastBytes, void* args);
    static void _NutEnterChayProgramVoiChuKyDuocChon_(int32_t lastBytes, void* args);
    static void _NutInfTrongTrangChonChuKyChayProgram_(int32_t lastBytes, void* args);
    static void _NutCaiChuKyChayProgram_(int32_t lastBytes, void* args);

    static void _NutVaoChucNangTietTrung_(int32_t lastBytes, void* args);
    static void _NutCaiThoiGianTietTrung_(int32_t lastBytes, void* args);
    static void _NutCaiNhietDoTietTrung_(int32_t lastBytes, void* args);
    static void _NutNextTrongCaiTietTrung_(int32_t lastBytes, void* args);
    static void _NutEnterCaiTietTrung_(int32_t lastBytes, void* args);
    static void _NutEnterCaiThoiGianTietTrung_(int32_t lastBytes, void* args);
    static void _NutTroVeTrongTrangWarning_(int32_t lastBytes, void* args);
    static void _NutXoaDoThi_(int32_t lastBytes, void* args);

    static void _NutDataRecord_(int32_t lastBytes, void* args);
    static void _NutExportData_(int32_t lastBytes, void* args);

    static void _NutVaoChucNangUpdate_(int32_t lastBytes, void* args);
    static void _CacNutChonPhuongThucUpdate_(int32_t lastBytes, void* args);
    static void _CacNutTrangFOTA_(int32_t lastBytes, void* args);

    static void _ThanhCuonDoThi_(int32_t lastBytes, void* args);

    static void _NutVaoChucNangWiFi_(int32_t lastBytes, void* args); // truc them
    static void _CacNutTrangWiFi_(int32_t lastBytes, void* args);
    static void _NutVaoChucNangThayDoiAdminPassword_(int32_t lastBytes, void* args);
    static void _CacNutTrangThayDoiAdminPassword_(int32_t lastBytes, void* args);
};
#endif
