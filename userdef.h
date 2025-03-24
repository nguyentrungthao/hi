#ifndef _UserDef_H_
#define _UserDef_H_

#include <Arduino.h>

#define PATH_CALIB_DATA "/CalibData.data"
#define PATH_BASEPROGRAM_DATA "/BaseProgram.data"
#define PATH_RECORD "/record.dat"
#define PATH_LOG "/log"

typedef enum 
{
    PROGRAM_MODE,
    QUICK_MODE,
    STERILIZATION_MODE,
} RunMode_t;

typedef struct 
{
    float setPointTemp;
    float tempMax;
    float tempMin;
    float setPointCO2;
    float CO2Max;
    float CO2Min;
    int8_t fanSpeed;
    // int8_t flap;
    int8_t delayOffDay;
    int8_t delayOffHour;
    int8_t delayOffMinute;
} Program_t;

typedef struct 
{
    Program_t programData;
    float temperature;
    float CO2;
    bool machineState;
    bool delayOffState;
} BaseProgram_t;

typedef struct 
{
    float Setpoint;
    float value;
} CalibData_t;

typedef struct 
{
    float setpointTemp;
    float valueTemp;
    float setpointCO2;
    float valueCO2;
    uint8_t fan;
    // uint8_t flap;
    uint8_t day;
    uint8_t month;
    uint8_t year;
    uint8_t hour;
    uint8_t minute;
    uint8_t second; 
} RecordData_t;

typedef struct {
    char ssid[30];
    char password[30];
    bool state;
} WiFiConfig_t; 

// Program structure definition
typedef struct
{
    float setPoint;
    float temp;
    int8_t fanSpeed;
    int8_t flap;
    bool runState;
} MQTTData_t;

struct PIDData {
    float Setpoint;
    float Temperature;
    float Kp;
    float Ki;
    float Kd;
    float PTerm;
    float ITerm;
    float DTerm;
    float Output;
    int Triac;
    int IMax;
    int UMax;
    int TGondinh;
    int TGgianhiet;
};

enum EventTaskHMI_t {
    eEVENT_ICON_NHIET = 0,
    eEVENT_ICON_CO2,
    eEVENT_ICON_CUA,
    
    eEVENT_ICON_FAN,
    eEVENT_ICON_USB,
    eEVENT_ICON_WIFI,
    eEVENT_HIEN_THI_GIA_TRI_CAM_BIEN,
    eEVENT_HIEN_THI_THOI_GIAN,
    eEVENT_VE_DO_THI,
    eEVENT_WARNING,
    eEVENT_REFRESH,
};

struct FrameDataQueue_t {
    int32_t event;
    void* pvData;   
};

typedef enum {
    MAIN_UPDATE_USB,
    MAIN_UPDATE_FOTA,
} MethodUpdates_t;



#define _CHECK_AND_WARNING_PAGE(condition, message, returnPage) do { \
    if (condition) { \
        _dwin.HienThiWarning(message, returnPage); \
        _dwin.Buzzer(160); \
    } \
} while (0)


#endif