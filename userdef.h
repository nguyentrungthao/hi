#ifndef _UserDef_H_
#define _UserDef_H_

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
    float setPoint;
    float tempMax;
    float tempMin;
    int8_t fanSpeed;
    int8_t flap;
    int8_t delayOffDay;
    int8_t delayOffHour;
    int8_t delayOffMinute;
} Program_t;

typedef struct 
{
    Program_t programData;
    float temperature;
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
    float setpoint;
    float value;
    uint8_t fan;
    uint8_t flap;
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

#endif