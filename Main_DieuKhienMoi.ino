// Đường dẫn SDKconfig thay đổi độ ưu tiên task Timer lên 20
// C:\Users\minht\AppData\Local\Arduino15\packages\esp32\tools\esp32-arduino-libs\idf-release_v5.1-33fbade6\esp32s3\qio_qspi\include
#include <iostream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <type_traits>
#include <TimeLib.h>
#include <Wire.h>
#include <Arduino.h>

#include "FileHandle.h"
#include "userdef.h"
#include "RTClib.h"
#include "HMI.h"
#include "HMIparam.h"
#include "src/usb_msc_host.h"

#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#define ARDUINOJSON_USE_LONG_LONG 1
#include <ArduinoJson.h>
#include <SD_MMC.h>
#include <SPI.h>
#include "freertos/FreeRTOSConfig.h"
#include "driver/temperature_sensor.h"

#include "HeaterControl.h"

#include "RTC_Timer.h"
RTC_Timer _time;

int clk = 5;
int cmd = 4;
int d0 = 6;
bool onebit = true;

// Thông tin Wi-Fi
const char* ssid = "LABone:KingLab-HUYlab";
const char* password = "66668888";

const uint8_t RxPin = 14;  // Chân Rx1 trên ESP32
const uint8_t TxPin = 13;  // Chân Tx1 trên ESP32

// Thêm điều kiện xài thư viện cũ
#define DGUS_SERIAL Serial2  // Cổng UART giao tiếp giữa MCU với HMI
#define DGUS_BAUD 115200     // Baundrate giao tiếp UART giữa MCU và HMI

#define DEM_XUONG true
#define DEM_LEN false

#define WIFICONFIG_PATH "/WiFiConfig.txt"
#define BUOC_NHAY_DO_THI 20

#define ADMIN_PASSWORD "/AdminPassword.txt"

HMI _dwin(Serial2, RxPin, TxPin, DGUS_BAUD);
FileHandle UsbFile(USB_MSC_HOST);
FileHandle SDMMCFile(SD_MMC);

BaseProgram_t BaseProgram;
std::vector<Program_t> ProgramList(30);
std::vector<Program_t> CurrentProgram(30);
std::vector<CalibData_t> DanhSachHeSoCalib;
DateTime RTCnow;

RunMode_t RunMode = QUICK_MODE;
static int listPageStartPosition = 0;
static int listLength = 0;
// static int programIndex = 0;
static int programListIndex = 0;
static int RunningSegmentIndex = 0;
static int programLoop = 0;
static int programLoopCount = 0;
static bool programInf = false;
static bool FlagNhietDoXacLap = false;
static bool FlagCO2XacLap = false;
static bool SwitchSegment = false;
static bool FlagVeDoThi = true;

TaskHandle_t TaskEspnowHdl = NULL;
TaskHandle_t TaskVeDoThiHdl = NULL;
TaskHandle_t TaskCanhBaoHdl = NULL;
TaskHandle_t TaskExportDataHdl = NULL;
TaskHandle_t TaskUpdateFirmwareHdl = NULL;
TaskHandle_t TaskMainHdl;
TaskHandle_t TaskKetNoiWiFiHdl;  // truc them

SemaphoreHandle_t SemaCaiDatHeater;
SemaphoreHandle_t SemaExportData;
SemaphoreHandle_t SemaKetNoiWiFi;  // truc them

QueueHandle_t QueueUpdateFirmware;

RecordData_t DuLieuDoThi[500];

typedef enum {
    MAIN_UPDATE_USB,
    MAIN_UPDATE_FOTA,
} MethodUpdates_t;

String WifiSSID = "";
String WifiPassword = "";
WiFiConfig_t WiFiConfig;

uint8_t countTest = 0;
int8_t GiaTriThanhCuon = 0;
int8_t GiaTriThanhCuonTruocDo = 0;
bool TrangThaiThanhCuon = false;

// Queue handle -> POST thong so dieu khien ve server
QueueHandle_t dataQueue;

bool DemThoiGianChay(bool reset, bool DemXuong = false);
void sendData(PIDData* data);

void setTimeFromRTC(int year, int month, int day, int hour, int minute, int second) {
    struct tm timeinfo;
    timeinfo.tm_year = year - 1900;  // Năm tính từ 1900
    timeinfo.tm_mon = month - 1;     // Tháng bắt đầu từ 0
    timeinfo.tm_mday = day;
    timeinfo.tm_hour = hour;
    timeinfo.tm_min = minute;
    timeinfo.tm_sec = second;

    time_t t = mktime(&timeinfo);  // Chuyển đổi struct tm thành time_t
    struct timeval now_tv = { .tv_sec = t };
    settimeofday(&now_tv, NULL);  // Đặt thời gian hệ thống ESP32
}
// Liệt kê và in ra tên các tệp tin trong thư mục "program"
void listFilesInProgramDirectory() {
    File root = SD_MMC.open("/program");
    if (!root) {
        Serial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory()) {
        Serial.println("/program is not a directory");
        return;
    }
    time_t thoigianluufile;
    File file = root.openNextFile();
    while (file) {
        Serial.print("File: ");
        Serial.print(file.name());
        thoigianluufile = file.getLastWrite();
        Serial.printf(" %u/%u/%u-%02u:%02u:%02u\n", day(thoigianluufile), month(thoigianluufile), year(thoigianluufile),
            hour(thoigianluufile), minute(thoigianluufile), second(thoigianluufile));
        file.close();
        file = root.openNextFile();
    }
}

void hmiSetEvent(const hmi_set_event_t& event) {
    char filePath[30];
    static int index = 0;
    static int ngayRTC, thangRTC, namRTC, gioRTC, phutRTC;
    static String currentProgramName;
    Program_t newProgram;
    static int8_t XacNhanTietTrung = 0;
    static int32_t ThoiGian2LanChamThanhCuon = millis();
    switch (event.type) {
    case HMI_SET_RUN_ONOFF:
        if (BaseProgram.machineState == false) {
            BaseProgram.machineState = true;
            FlagNhietDoXacLap = false;
            SwitchSegment = true;
            RunningSegmentIndex = 0;
            programLoopCount = 0;
            if (RunMode == PROGRAM_MODE) {
                _dwin.HienThiVongLapChuongTrinhConLai(programLoopCount + 1, programLoop);
            }
            else {
                _dwin.HienThiVongLapChuongTrinhConLai("");
            }
            if (BaseProgram.delayOffState) {
                DemThoiGianChay(true, DEM_XUONG);
                // _time.reset();
            }
            else {
                DemThoiGianChay(true, DEM_LEN);
                // _time.reset();
            }
            _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);
            xSemaphoreGive(SemaCaiDatHeater);
        }
        else {
            BaseProgram.machineState = false;
            if (BaseProgram.delayOffState) {
                _dwin.HienThiThoiGianChay(BaseProgram.programData.delayOffDay, BaseProgram.programData.delayOffHour, BaseProgram.programData.delayOffMinute, 0);
            }
            else {
                _dwin.HienThiThoiGianChay("Inf");
            }
            _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);
            xSemaphoreGive(SemaCaiDatHeater);
        }
        break;
    case HMI_SET_SETPOINT_TEMP:
        Serial.printf("Set setpoint temp: %.1f\n", event.f_value);
        BaseProgram.programData.setPointTemp = event.f_value;
        if (RunMode == QUICK_MODE) {
            SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
        }
        FlagNhietDoXacLap = false;
        xSemaphoreGive(SemaCaiDatHeater);
        break;
    case HMI_SET_SETPOINT_CO2:
        Serial.printf("Set setpoint CO2: %.1f\n", event.f_value);
        BaseProgram.programData.setPointCO2 = event.f_value;
        if (RunMode == QUICK_MODE) {
            SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
        }
        FlagNhietDoXacLap = false;
        xSemaphoreGive(SemaCaiDatHeater);
        break;
    case HMI_SET_FAN:
        Serial.printf("Set fanspeed: %.1f\n", event.f_value);
        BaseProgram.programData.fanSpeed = (int8_t)event.f_value;
        if (RunMode == QUICK_MODE) {
            SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
        }
        xSemaphoreGive(SemaCaiDatHeater);
        break;
    case HMI_SET_CALIB:
    {
        Serial.printf("Set calib: %.1f\n", event.f_value);
        // Kiểm tra đã có hệ số calib cũ chưa nếu có thì cập nhật lại hệ số calib
        CalibData_t HeSoCalib;
        for (int i = 0; i < DanhSachHeSoCalib.size(); i++) {
            if (fabs(DanhSachHeSoCalib.at(i).Setpoint - BaseProgram.programData.setPointTemp) < 0.01) {
                DanhSachHeSoCalib[i].value = event.f_value - (BaseProgram.temperature - DanhSachHeSoCalib[i].value);
                SDMMCFile.writeFile(PATH_CALIB_DATA, (uint8_t*)DanhSachHeSoCalib.data(), DanhSachHeSoCalib.size() * sizeof(CalibData_t));
                Serial.println("Thay Doi he so calib da co");
                xSemaphoreGive(SemaCaiDatHeater);
                return;
            }
        }
        HeSoCalib.Setpoint = BaseProgram.programData.setPointTemp;
        HeSoCalib.value = event.f_value - BaseProgram.temperature;
        DanhSachHeSoCalib.push_back(HeSoCalib);
        SDMMCFile.writeFile(PATH_CALIB_DATA, (uint8_t*)DanhSachHeSoCalib.data(), DanhSachHeSoCalib.size() * sizeof(CalibData_t));
        xSemaphoreGive(SemaCaiDatHeater);
        break;
    }
    case HMI_RESET_CALIB:
    {
        Serial.println("Reset calib");
        CalibData_t HeSoCalib;
        for (int i = 0; i < DanhSachHeSoCalib.size(); i++) {
            if (fabs(DanhSachHeSoCalib.at(i).Setpoint - BaseProgram.programData.setPointTemp) < 0.01) {
                DanhSachHeSoCalib[i].value = 0;
                SDMMCFile.writeFile(PATH_CALIB_DATA, (uint8_t*)DanhSachHeSoCalib.data(), DanhSachHeSoCalib.size() * sizeof(CalibData_t));
                xSemaphoreGive(SemaCaiDatHeater);
                return;
            }
        }
        break;
    }
    // case HMI_SET_FLAP:
    //     Serial.printf("Set flap: %.1f\n", event.f_value);
    //     BaseProgram.programData.flap = (int8_t)event.f_value;
    //     if (RunMode == QUICK_MODE)
    //     {
    //         SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
    //     }
    //     xSemaphoreGive(SemaCaiDatHeater);
    //     break;
    case HMI_SET_DELAYOFF:
        if (RunMode == STERILIZATION_MODE) {
            Serial.println("HMI_SET_DELAYOFF, Fail -> STERILIZATION_MODE");
            return;
        }
        Serial.printf("Set time delayoff: %d\n", event.u32_value);
        BaseProgram.programData.delayOffDay = event.u32_value / 86400;
        BaseProgram.programData.delayOffHour = event.u32_value % 86400 / 3600;
        BaseProgram.programData.delayOffMinute = event.u32_value % 86400 % 3600 / 60;
        if (RunMode == QUICK_MODE) {
            SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
        }
        Serial.printf("DelayOff Day: %d\n", BaseProgram.programData.delayOffDay);
        Serial.printf("DelayOff Hour: %d\n", BaseProgram.programData.delayOffHour);
        Serial.printf("DelayOff Minute: %d\n", BaseProgram.programData.delayOffMinute);
        if (BaseProgram.delayOffState) {
            DemThoiGianChay(true, DEM_XUONG);
            _dwin.HienThiThoiGianChay(BaseProgram.programData.delayOffDay, BaseProgram.programData.delayOffHour, BaseProgram.programData.delayOffMinute, 0);
        }
        break;
    case HMI_SET_RTC:
    {
        DateTime RTCset((uint32_t)event.u32_value);
        _time.updateRTC(RTCset);
        setTimeFromRTC(RTCset.year(), RTCset.month(), RTCset.day(), RTCset.hour(), RTCset.minute(), RTCset.second());
    }
    Serial.printf("Unix time: %d\n", (uint32_t)event.u32_value);
    break;
    case HMI_SET_ALARM_BELOW:
        Serial.printf("HMI_SET_ALARM_BELOW: %.1f\n", event.f_value);
        BaseProgram.programData.tempMin = event.f_value;
        if (RunMode == QUICK_MODE) {
            SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
        }
        break;
    case HMI_SET_ALARM_ABOVE:
        Serial.printf("HMI_SET_ALARM_ABOVE: %.1f\n", event.f_value);
        BaseProgram.programData.tempMax = event.f_value;
        if (RunMode == QUICK_MODE) {
            SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
        }
        break;
    case HMI_ADD_PROGRAM:
        Serial.println("HMI_ADD_PROGRAM");
        if (SDMMCFile.countFiles("/program") >= 20) {
            _dwin.HienThiWarning("Max: 20 programs", _ProgramPage);
            break;
        }
        sprintf(filePath, "/program/%s", event.text.c_str());
        if (SDMMCFile.exists(filePath)) {
            _dwin.HienThiWarning(event.text + " already exists", _ProgramPage);
            break;
        }
        currentProgramName = event.text;
        ProgramList.clear();
        newProgram.setPointTemp = 37;
        newProgram.setPointCO2 = 5;
        newProgram.fanSpeed = 50;
        newProgram.tempMin = -3;
        newProgram.tempMax = 3;
        newProgram.CO2Min = -3;
        newProgram.CO2Max = 3;
        newProgram.delayOffDay = 0;
        newProgram.delayOffHour = 1;
        newProgram.delayOffMinute = 1;
        // newProgram.flap = 10;
        ProgramList.push_back(newProgram);
        SDMMCFile.writeFile((const char*)filePath, (uint8_t*)ProgramList.data(), ProgramList.size() * sizeof(Program_t));
        hmiGetEvent(HMI_GET_PROGRAM_LIST, NULL);
        hmiGetEvent(HMI_GET_SEGMENT_LIST, NULL);
        _dwin.setPage(_SegmentAdjPage);
        break;
    case HMI_DELETE_PROGRAM:
        Serial.println("HMI_DELETE_PROGRAM");
        sprintf(filePath, "/program/%s", event.text.c_str());
        SDMMCFile.deleteFile(filePath);
        break;
    case HMI_EDIT_SEG_SETPOINT:
        if (ProgramList.size() > event.indexList + listPageStartPosition) {
            ProgramList.at(event.indexList + listPageStartPosition).setPointTemp = event.f_value;
            Serial.printf("Set segment setpoint %u: %.1f\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].setPointTemp);
        }
        break;
        // case HMI_EDIT_SEG_SETPOINT:
        //     if (ProgramList.size() > event.indexList + listPageStartPosition)
        //     {
        //         ProgramList.at(event.indexList + listPageStartPosition).setPointCO2 = event.f_value;
        //         Serial.printf("Set segment setpoint %u: %.1f\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].setPointCO2);
        //     }
        //     break;
    case HMI_EDIT_SEG_FANSPEED:
        if (ProgramList.size() > event.indexList + listPageStartPosition) {
            ProgramList.at(event.indexList + listPageStartPosition).fanSpeed = event.f_value;
            Serial.printf("Set segment fanspeed %u: %u\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].fanSpeed);
        }
        break;
        // case HMI_EDIT_SEG_AIRFLAP:
        //     if (ProgramList.size() > event.indexList + listPageStartPosition)
        //     {
        //         ProgramList.at(event.indexList + listPageStartPosition).flap = event.f_value;
        //         Serial.printf("Set segment air flap %u: %u\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].flap);
        //     }
        //     break;
    case HMI_EDIT_SEG_TEMPMIN:
        if (ProgramList.size() > event.indexList + listPageStartPosition) {
            ProgramList.at(event.indexList + listPageStartPosition).tempMin = event.f_value;
            Serial.printf("Set segment temp pro min %u: %.1f\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].tempMin);
        }
        break;
    case HMI_EDIT_SEG_TEMPMAX:
        if (ProgramList.size() > event.indexList + listPageStartPosition) {
            ProgramList.at(event.indexList + listPageStartPosition).tempMax = event.f_value;
            Serial.printf("Set segment temp pro max %u: %.1f\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].tempMax);
        }
        break;
    case HMI_ADD_SEG:
        Serial.println("HMI_ADD_SEG");
        if (ProgramList.size() >= 30) {
            return;
        }
        newProgram.setPointTemp = 37;
        newProgram.setPointCO2 = 5;
        newProgram.fanSpeed = 50;
        newProgram.tempMin = -3;
        newProgram.tempMax = 3;
        newProgram.CO2Min = -3;
        newProgram.CO2Max = 3;
        newProgram.delayOffDay = 0;
        newProgram.delayOffHour = 1;
        newProgram.delayOffMinute = 1;
        // newProgram.flap = 10;
        if (ProgramList.size() < (int)event.indexList + listPageStartPosition) {
            ProgramList.push_back(newProgram);
            if (ProgramList.size() >= 5) {
                listPageStartPosition = ProgramList.size() - 5;
            }
        }
        else {
            ProgramList.insert(ProgramList.begin() + (int)event.indexList + listPageStartPosition, newProgram);
        }
        break;
    case HMI_SUB_SEG:
        Serial.println("HMI_SUB_SEG");
        if (ProgramList.size() > 1) {
            if ((int)event.indexList < 5 && ProgramList.size() >(int)event.indexList + listPageStartPosition) {
                ProgramList.erase(ProgramList.begin() + (int)event.indexList + listPageStartPosition);
            }
            else {
                ProgramList.erase(ProgramList.end() - 1);
            }
            // ProgramList.shrink_to_fit();
        }
        break;
    case HMI_EDIT_SEG_DELAYOFF_DAY:
        Serial.println("HMI_EDIT_SEG_DELAYOFF_DAY");
        if (ProgramList.size() > event.indexList + listPageStartPosition) {
            ProgramList.at(event.indexList + listPageStartPosition).delayOffDay = event.u32_value;
            Serial.printf("Delay off Day%u: %d\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].delayOffDay);
        }
        break;
    case HMI_EDIT_SEG_DELAYOFF_HOUR:
        Serial.println("HMI_EDIT_SEG_DELAYOFF_HOUR");
        if (ProgramList.size() > event.indexList + listPageStartPosition) {
            ProgramList.at(event.indexList + listPageStartPosition).delayOffHour = event.u32_value;
            Serial.printf("Delay off Hour%u: %d\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].delayOffHour);
        }
        break;
    case HMI_EDIT_SEG_DELAYOFF_MINUTE:
        Serial.println("HMI_EDIT_SEG_DELAYOFF_MINUTE");
        if (ProgramList.size() > event.indexList + listPageStartPosition) {
            ProgramList.at(event.indexList + listPageStartPosition).delayOffMinute = event.u32_value;
            Serial.printf("Delay off Minute%u: %d\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].delayOffMinute);
        }
        break;
    case HMI_SAVE_SEG:
        sprintf(filePath, "/program/%s", currentProgramName);
        SDMMCFile.writeFile((const char*)filePath, (uint8_t*)ProgramList.data(), ProgramList.size() * sizeof(Program_t));
        Serial.printf("Store %s\n", filePath);
        if (currentProgramName != event.text) {
            char* newName = new char[30];
            sprintf(newName, "/program/%s", event.text.c_str());
            SDMMCFile.renameFile(filePath, newName);
            delete[] newName;
        }
        break;
    case HMI_SELECT_PROGRAM:
        ProgramList.clear();
        // ProgramList.shrink_to_fit();
        sprintf(filePath, "/program/%s", event.text.c_str());
        ProgramList.resize(SDMMCFile.sizeFile(filePath) / sizeof(Program_t));
        SDMMCFile.readFile((const char*)filePath, (uint8_t*)ProgramList.data(), SDMMCFile.sizeFile(filePath));
        _dwin.HienThiTenProgramDangEdit((String&)event.text);
        currentProgramName = event.text;
        Serial.printf("Restore %s\n", filePath);
        Serial.printf("list size: %d\n", ProgramList.size());
        break;
    case HMI_SET_DELAYOFF_ONOFF:
        Serial.println("HMI_SET_DELAYOFF_ON");
        if (RunMode != STERILIZATION_MODE) {
            if (BaseProgram.delayOffState == false) {
                BaseProgram.delayOffState = true;
                DemThoiGianChay(true, DEM_XUONG);
            }
            else {
                BaseProgram.delayOffState = false;
                DemThoiGianChay(true, DEM_LEN);
                _dwin.HienThiThoiGianChay("Inf");
            }
            _dwin.HienThiIconOnOffDelayOff(BaseProgram.delayOffState);
        }
        break;
    case HMI_RUN_PROGRAM_MODE:
        CurrentProgram.clear();
        // CurrentProgram.shrink_to_fit();
        Serial.printf("list size: %d\n", CurrentProgram.size());
        sprintf(filePath, "/program/%s", event.text.c_str());
        if (SDMMCFile.sizeFile(filePath) / sizeof(Program_t) == 0) {
            _dwin.HienThiWarning(event.text + ": 0 segment", _ProgramPage);
            break;
        }
        CurrentProgram.resize(SDMMCFile.sizeFile(filePath) / sizeof(Program_t));
        SDMMCFile.readFile((const char*)filePath, (uint8_t*)CurrentProgram.data(), SDMMCFile.sizeFile(filePath));
        if (CurrentProgram.size() > 0) {
            RunMode = PROGRAM_MODE;
            SwitchSegment = true;
            RunningSegmentIndex = 0;
            _dwin.HienThiChuongTrinhDangChay(event.text);
            _dwin.HienThiIconSegment(false);
            _dwin.HienThiSegmentDangChay("");
            _dwin.setPage(_HomePage);
            if ((uint8_t)event.f_value > 0) {
                programInf = false;
                programLoop = (uint8_t)event.f_value;
                _dwin.HienThiVongLapChuongTrinhConLai(0, programLoop);
            }
            else {
                programInf = true;
                _dwin.HienThiVongLapChuongTrinhConLai("Inf");
            }
            Serial.println("HMI_RUN_PROGRAM_MODE");
        }
        break;
    case HMI_RUN_QUICK_MODE:
        RunMode = QUICK_MODE;
        SDMMCFile.readFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
        BaseProgram.machineState = false;
        BaseProgram.delayOffState = false;
        _dwin.HienThiChuongTrinhDangChay("Quick");
        _dwin.HienThiSetpointTemp(BaseProgram.programData.setPointTemp);
        _dwin.HienThiSetpointCO2(BaseProgram.programData.setPointCO2);
        _dwin.HienThiTocDoQuat(BaseProgram.programData.fanSpeed);
        _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);
        // _dwin.HienThiGocFlap(BaseProgram.programData.flap);
        _dwin.HienThiThoiGianChay(BaseProgram.programData.delayOffDay, BaseProgram.programData.delayOffHour, BaseProgram.programData.delayOffMinute, 0);
        _dwin.HienThiSegmentDangChay("");
        _dwin.HienThiIconSegment(false);
        _dwin.HienThiIconOnOffDelayOff(false);
        _dwin.HienThiThoiGianChay("Inf");
        _dwin.HienThiVongLapChuongTrinhConLai("Inf");
        xSemaphoreGive(SemaCaiDatHeater);
        break;
    case HMI_RESET_STER:
        XacNhanTietTrung = 0;
        break;
    case HMI_SET_ICON1:
        _dwin.HienThiIconConfirm1(true);
        // _dwin.setPage(_SterilizationPage);
        break;
    case HMI_SET_ICON2:
        _dwin.HienThiIconConfirm2(true);
        // _dwin.setPage(_SterilizationPage);
        XacNhanTietTrung = 1;
        break;
    case HMI_SET_STER_TEMP:
        if (XacNhanTietTrung) {
            Serial.printf("tiet trung temp: %.1f\n", (float)event.f_value);
            BaseProgram.programData.setPointTemp = (float)event.f_value;
            BaseProgram.programData.fanSpeed = 100;
            // BaseProgram.programData.flap = 0;
            _dwin.HienThiSetpointTemp(BaseProgram.programData.setPointTemp);
            _dwin.HienThiTocDoQuat(BaseProgram.programData.fanSpeed);
            // _dwin.HienThiGocFlap(BaseProgram.programData.flap);
        }
        break;
    case HMI_SET_STER_TIME:
        if (XacNhanTietTrung) {
            Serial.printf("tiet trung time: %d:%d\n", event.u32_value / 3600, event.u32_value % 3600 / 60);
            BaseProgram.programData.delayOffDay = 0;
            BaseProgram.programData.delayOffHour = event.u32_value / 3600;
            BaseProgram.programData.delayOffMinute = event.u32_value % 3600 / 60;
            BaseProgram.delayOffState = true;
            RunMode = STERILIZATION_MODE;
            _dwin.HienThiChuongTrinhDangChay("Sterilization");
            _dwin.HienThiVongLapChuongTrinhConLai("");
            _dwin.HienThiIconSegment(false);
            _dwin.HienThiSegmentDangChay("");
            _dwin.HienThiIconOnOffDelayOff(BaseProgram.delayOffState);
            DemThoiGianChay(true, DEM_XUONG);
        }
        break;
    case HMI_EXPORT_DATA:
        xSemaphoreGive(SemaExportData);
        Serial.println("Copy");
        break;
    case HMI_SET_SSID:
        Serial.print("SSID: ");
        Serial.println(event.text);
        WifiSSID = event.text;
        break;
    case HMI_SET_PASSWORD:
        Serial.print("PASSWORD: ");
        Serial.println(event.text);
        WifiPassword = event.text;
        break;
    case HMI_FIRMWARE_USB:
        Serial.println("->>> HMI_FIRMWARE_USB");
        if (QueueUpdateFirmware != NULL) {
            MethodUpdates_t method = MAIN_UPDATE_USB;
            xQueueSend(QueueUpdateFirmware, &method, (TickType_t)0);
            if (TaskUpdateFirmwareHdl == NULL) {
                if (esp_get_free_heap_size() > 60000 && USB_MSC_HOST.isConnected()) {
                    xTaskCreateUniversal(TaskUpdateFirmware, "Task update firmware", 8192, NULL, 12, &TaskUpdateFirmwareHdl, -1);
                }
            }
        }
        break;
    case HMI_FIRMWARE_FOTA:
        Serial.println("->>> FOTA");
        if (QueueUpdateFirmware != NULL) {
            MethodUpdates_t method = MAIN_UPDATE_FOTA;
            xQueueSend(QueueUpdateFirmware, &method, (TickType_t)0);
            if (TaskUpdateFirmwareHdl == NULL) {
                if (esp_get_free_heap_size() > 60000) {
                    xTaskCreateUniversal(TaskUpdateFirmware, "Task update firmware", 8192, NULL, 12, &TaskUpdateFirmwareHdl, -1);
                }
            }
        }
        break;
    case HMI_SET_SCROLLCHART:
        countTest++;
        Serial.println("Scroll: " + String((int)event.f_value));
        GiaTriThanhCuon = (int)event.f_value;
        // if(TrangThaiThanhCuon == false && millis() - ThoiGian2LanChamThanhCuon > 200) {
        //     GiaTriThanhCuonTruocDo = GiaTriThanhCuon;
        //     TrangThaiThanhCuon = true;
        //     ThoiGian2LanChamThanhCuon = millis();
        // }
        if (millis() - ThoiGian2LanChamThanhCuon > 200) {
            GiaTriThanhCuonTruocDo = GiaTriThanhCuon;
            TrangThaiThanhCuon = false;
        }
        else if (GiaTriThanhCuonTruocDo != GiaTriThanhCuon) {
            TrangThaiThanhCuon = true;
        }
        ThoiGian2LanChamThanhCuon = millis();
        break;
    case HMI_CONNECT_OR_DISCONNECT_WIFI:
        xSemaphoreGive(SemaKetNoiWiFi);
        break;
    case HMI_CHANGE_ADMIN_PASSWORD:
        Serial.print("Password: ");
        Serial.println(event.text);
        SDMMCFile.writeFile(ADMIN_PASSWORD, event.text.c_str());
        break;
    default:
        Serial.println("UNDEFINE");
        break;
    }
}
int programStart = 0, programEnd = 0;
bool hmiGetEvent(hmi_get_type_t event, void* args) {
    int i = 0;
    time_t timeNow;
    File file, root;
    switch (event) {
    case HMI_CHECK_LIST:
        if (*((uint8_t*)args) + listPageStartPosition >= ProgramList.size()) {
            return 0;
        }
        break;
        // case HMI_GET_FLAP:
        //     // _dwin.HienThiGocFlap(BaseProgram.programData.flap);
        //     break;
    case HMI_GET_RTC:
        // timeNow = now();
        _dwin.HienThiNgay(_time.getDay());
        _dwin.HienThiThang(_time.getMonth());
        _dwin.HienThiNam(_time.getYear());
        _dwin.HienThiGio(_time.getHour());
        _dwin.HienThiPhut(_time.getMinute());
        break;
    case HMI_GET_DELAYOFF:
        _dwin.HienThiNgay(BaseProgram.programData.delayOffDay);
        _dwin.HienThiGio(BaseProgram.programData.delayOffHour);
        _dwin.HienThiPhut(BaseProgram.programData.delayOffMinute);
        break;
    case HMI_GET_ALARM:
        _dwin.HienThiNhietDoCanhBao(BaseProgram.programData.tempMin, BaseProgram.programData.tempMax);
        break;
    case HMI_GET_PROGRAM_LIST:
        programListIndex = 0;
        root = SD_MMC.open("/program");
        if (!root) {
            Serial.println("Failed to open directory");
            return 0;
        }
        if (!root.isDirectory()) {
            Serial.println("/program is not a directory");
            return 0;
        }
        file = root.openNextFile();
        programStart = 0;
        while (file) {
            Serial.print("File: ");
            Serial.println(file.name());
            _dwin.HienThiTenChuongTrinhTrenHang(programListIndex, programListIndex + 1, file.name(), file.size() / sizeof(Program_t));
            file = root.openNextFile();
            programListIndex++;
            if (programListIndex % 4 == 0) {
                break;
            }
        }
        if (programListIndex < 4) {
            for (int8_t i = programListIndex; i < 4; i++) {
                _dwin.XoaDuLieuHienThiTenChuongTrinhTrenHang(i);
            }
        }
        Serial.println("HMI_GET_PROGRAM_LIST");
        break;
    case HMI_GET_NEXT_PROGRAM_LIST:
        if (programListIndex - programStart == 0) {
            programStart -= 4;
        }
        else {
            programStart = (programListIndex / 4) * 4;
        }
        programListIndex = 0;
        Serial.printf("programStart: %d\n", programStart);
        Serial.printf("programListIndex: %d\n", programListIndex);
        root = SD_MMC.open("/program");
        if (!root) {
            Serial.println("Failed to open directory");
            return 0;
        }
        if (!root.isDirectory()) {
            Serial.println("/program is not a directory");
            return 0;
        }
        file = root.openNextFile();
        while (file) {
            Serial.print("File: ");
            Serial.println(file.name());
            if (programListIndex >= programStart) {
                _dwin.HienThiTenChuongTrinhTrenHang(programListIndex - programStart, programListIndex + 1, file.name(), file.size() / sizeof(Program_t));
                file.close();
            }
            file = root.openNextFile();
            programListIndex++;
            if (programListIndex % 4 == 0 && programListIndex > programStart) {
                file.close();
                break;
            }
        }
        if (programListIndex % 4 != 0) {
            for (int8_t i = programListIndex - programStart; i < 4; i++) {
                _dwin.XoaDuLieuHienThiTenChuongTrinhTrenHang(i);
            }
        }
        break;
    case HMI_GET_BACK_PROGRAM_LIST:
        if (programStart >= 4) {
            programStart -= 4;
        }
        else if (programStart == 0) {
            return 0;
        }
        else {
            programStart = 0;
        }
        programListIndex = 0;
        root = SD_MMC.open("/program");
        if (!root) {
            Serial.println("Failed to open directory");
            return 0;
        }
        if (!root.isDirectory()) {
            Serial.println("/program is not a directory");
            return 0;
        }
        file = root.openNextFile();
        while (file) {
            Serial.print("File: ");
            Serial.println(file.name());
            if (programListIndex >= programStart) {
                _dwin.HienThiTenChuongTrinhTrenHang(programListIndex - programStart, programListIndex + 1, file.name(), file.size() / sizeof(Program_t));
                file.close();
            }
            file = root.openNextFile();
            programListIndex++;
            if (programListIndex % 4 == 0 && programListIndex > programStart) {
                file.close();
                break;
            }
        }
        if (programListIndex % 4 != 0) {
            for (int8_t i = programListIndex - programStart; i < 4; i++) {
                _dwin.XoaDuLieuHienThiTenChuongTrinhTrenHang(i);
            }
        }

        break;
    case HMI_GET_SEGMENT_LIST:
        Serial.println("List_segment");
        listPageStartPosition = 0;
        if (ProgramList.size() >= 5) {
            listLength = 5;
        }
        else {
            listLength = ProgramList.size();
        }
        goto HienThiSegmentList;
        break;
    case HMI_GET_NEXT_SEGMENT_LIST:
        if (ProgramList.size() > listPageStartPosition + 5) {
            listPageStartPosition += 5;
            listLength = ProgramList.size() - listPageStartPosition;
            if (listLength > 5) {
                listLength = 5;
            }
        }
        else if (ProgramList.size() <= 5) {
            listLength = ProgramList.size();
        }
        else if (ProgramList.size() <= listPageStartPosition + 5) {
            listLength = ProgramList.size() - listPageStartPosition;
        }
        else {
            Serial.println("Segment List is full");
        }
        goto HienThiSegmentList;
        break;
    case HMI_GET_BACK_SEGMENT_LIST:
        if (listPageStartPosition - 5 >= 0) {
            listPageStartPosition -= 5;
            listLength = 5;
        }
        else if (ProgramList.size() <= 5) {
            listPageStartPosition = 0;
            listLength = ProgramList.size();
        }
        else if (listPageStartPosition - 5 < 0) {
            listPageStartPosition = 0;
            listLength = 5;
        }
        goto HienThiSegmentList;
        break;
    case HMI_REFRESH_SEGMENT_LIST:
        if (ProgramList.size() <= 5) {
            listLength = ProgramList.size();
        }
        else if (ProgramList.size() <= listPageStartPosition + 5) {
            listLength = ProgramList.size() - listPageStartPosition;
        }
        goto HienThiSegmentList;
        break;
    case HMI_GET_SEGMENT_DELAYOFF:
        _dwin.HienThiNgay((ProgramList[*((uint8_t*)args) + listPageStartPosition].delayOffDay));
        _dwin.HienThiGio((ProgramList[*((uint8_t*)args) + listPageStartPosition].delayOffHour));
        _dwin.HienThiPhut((ProgramList[*((uint8_t*)args) + listPageStartPosition].delayOffMinute));
        break;
    case HMI_GET_CALIB:
        _dwin.HienThiHeSoCalib(GetCalib(BaseProgram.programData.setPointTemp));
        break;
    default:
        break;
    }
    return 1;
HienThiSegmentList:
    for (i = listPageStartPosition; i < 5 + listPageStartPosition; i++) {
        if (i < listLength + listPageStartPosition) {
            Program_t programData = ProgramList[i];
            _dwin.HienThiDuLieuSegmentTrenHang((i - listPageStartPosition),
                i + 1,
                programData.setPointTemp,
                programData.delayOffDay,
                programData.delayOffHour,
                programData.delayOffMinute,
                programData.fanSpeed,
                // programData.flap,
                programData.tempMin,
                programData.tempMax);
        }
        else {
            _dwin.XoaDuLieuHienThiSegmentTrenHang(i - listPageStartPosition);
        }
    }
    return 1;
}

bool DemThoiGianChay(bool reset, bool DemXuong) {
    static time_t timeRun;
    // Serial.println("Gửi thời gian đếm");
    if (reset) {
        _time.reset();
        if (DemXuong) {
            _time.start(BaseProgram.programData.delayOffDay * 86400 + BaseProgram.programData.delayOffHour * 3600 + BaseProgram.programData.delayOffMinute * 60, true);
            _dwin.HienThiThoiGianChay(BaseProgram.programData.delayOffDay, BaseProgram.programData.delayOffHour, BaseProgram.programData.delayOffMinute, 0);
            _dwin.HienThiIconOnOffDelayOff(true);
        }
        else {
            _time.start(BaseProgram.programData.delayOffDay * 86400 + BaseProgram.programData.delayOffHour * 3600 + BaseProgram.programData.delayOffMinute * 60);
            _dwin.HienThiIconOnOffDelayOff(false);
        }
    }
    else {
        if (DemXuong) {
            timeRun = _time.getRemainingTime();  // Lấy thời gian còn lại
        }
        else {
            timeRun = _time.getElapsedTime();  // Lấy thời gian đang đếm
        }
        _dwin.HienThiThoiGianChay((int)timeRun / 86400, (int)timeRun % 86400 / 3600, (int)timeRun % 86400 % 3600 / 60, (int)timeRun % 86400 % 3600 % 60);
    }
    if (_time.isFinished()) {
        return false;
    }
    return true;
}

// float GetCalib(float value)
// {
//     for (CalibData_t HeSoCalib : DanhSachHeSoCalib)
//     {
//         if (fabs(HeSoCalib.Setpoint - value) < 0.02)
//         {
//             // Serial.printf("Setpoint: %.1f, value: %.2f\n", HeSoCalib.Setpoint, HeSoCalib.value);
//             return HeSoCalib.value;
//         }
//     }
//     return 0;
// }

void TaskGuiDuLieuEspNow(void*) {
    for (;;) {
        xSemaphoreTake(SemaCaiDatHeater, 10000 / portTICK_PERIOD_MS);
        CaiDatHeater();
    }
}

int values[500];
void TaskHienThiNhietDoVaVeDoThi(void*) {
    TickType_t xLastWakeTime;
    // const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    int countTime = 0;
    int32_t preTime = millis();

    static int valueIdx = 0;
    char timeText[10];
    int length = 0;
    int LogFileSize;
    RecordData_t data;

    for (;;) {
        if (millis() - preTime >= 1000) {
            preTime = millis();
            RTCnow = _time.getCurrentTime();
            _dwin.HienThiThoiGianRTC(RTCnow.day(), RTCnow.month(), RTCnow.year() % 1000, RTCnow.hour(), RTCnow.minute(), RTCnow.second());
            if (countTime % BUOC_NHAY_DO_THI == 0) {
                // BaseProgram.temperature = random(50, 51);
                if (FlagVeDoThi) {
                    _dwin.VeDoThi(BaseProgram.temperature, RTCnow.unixtime(), BUOC_NHAY_DO_THI);
                    //     valueIdx++;
                    //     if (valueIdx >= 270)
                    //     {
                    //         for (int i = 0; i < 9; i++)
                    //         {
                    //             SDMMCFile.readFile("/TestDothi1.dat", (uint8_t *)&data, sizeof(RecordData_t), (i * 30 + valueIdx - 270) * sizeof(RecordData_t));
                    //             snprintf(timeText, sizeof(timeText), "%02u:%02u:%02u", data.hour, data.minute, data.second);
                    //             _dwin.setText(_VPAddressGraphXValueText1 + 10 * i, timeText);
                    //         }
                    //     }
                    //     else
                    //     {
                    //         for (int i = 0; i < valueIdx / 30; i++)
                    //         {
                    //             SDMMCFile.readFile("/TestDothi1.dat", (uint8_t *)&data, sizeof(RecordData_t), (i * 30) * sizeof(RecordData_t));
                    //             snprintf(timeText, sizeof(timeText), "%02u:%02u:%02u", data.hour, data.minute, data.second);
                    //             _dwin.setText(_VPAddressGraphXValueText1 + 10 * i, timeText);
                    //         }
                    //     }
                }
                // // countTime = 0;
                // RecordData_t record;
                // record.setpointTemp = BaseProgram.programData.setPointTemp;
                // record.valueTemp = BaseProgram.temperature;
                // record.fan = BaseProgram.programData.fanSpeed;
                // record.flap = BaseProgram.programData.flap;
                // record.day = RTCnow.day();
                // record.month = RTCnow.month();
                // record.year = RTCnow.year() % 2000;
                // record.hour = RTCnow.hour();
                // record.minute = RTCnow.minute();
                // record.second = RTCnow.second();
                // writeRecord(SD_MMC, record); // Test
                // SDMMCFile.appendFile("/TestDothi1.dat", (uint8_t *)&record, sizeof(record));
            }
            if (countTime % 60 == 0) {
                countTime = 0;
                RecordData_t record;
                record.setpointTemp = BaseProgram.programData.setPointTemp;
                record.setpointCO2 = BaseProgram.programData.setPointCO2;
                record.valueTemp = BaseProgram.temperature;
                record.fan = BaseProgram.programData.fanSpeed;
                // record.flap = BaseProgram.programData.flap;
                record.day = RTCnow.day();
                record.month = RTCnow.month();
                record.year = RTCnow.year() % 2000;
                record.hour = RTCnow.hour();
                record.minute = RTCnow.minute();
                record.second = RTCnow.second();
                writeRecord(SD_MMC, record);  // Test
                // SDMMCFile.appendFile("/TestDothi1.dat", (uint8_t *)&record, sizeof(record));
            }
            countTime++;
        }

        // if (countTest == 0 && TrangThaiThanhCuon)
        // {
        //     TrangThaiThanhCuon = false;
        //     delay(10);
        //     if (countTest > 0)
        //     {
        //         continue;
        //     }
        //     Serial.println("Nguoi dung da buong nut: " + String(GiaTriThanhCuon - GiaTriThanhCuonTruocDo));
        //     FlagVeDoThi = false;
        //     LogFileSize = SDMMCFile.sizeFile("/TestDothi1.dat");
        //     if (GiaTriThanhCuon - GiaTriThanhCuonTruocDo > 0)
        //     {
        //         length = GiaTriThanhCuon - GiaTriThanhCuonTruocDo;
        //         GiaTriThanhCuonTruocDo = GiaTriThanhCuon;
        //         if (valueIdx == LogFileSize / sizeof(RecordData_t))
        //         {
        //             FlagVeDoThi = true;
        //             continue;
        //         }

        //         if (valueIdx + length > LogFileSize / sizeof(RecordData_t))
        //         {
        //             length = LogFileSize / sizeof(RecordData_t) - valueIdx;
        //         }

        //         if (valueIdx + length >= 270)
        //         {
        //             for (int i = 0; i < 9; i++)
        //             {
        //                 SDMMCFile.readFile("/TestDothi1.dat", (uint8_t *)&data, sizeof(RecordData_t), (i * 30 + valueIdx - 270 + length) * sizeof(RecordData_t));
        //                 snprintf(timeText, sizeof(timeText), "%02u:%02u:%02u", data.hour, data.minute, data.second);
        //                 _dwin.setText(_VPAddressGraphXValueText1 + 10 * i, timeText);
        //             }
        //         }
        //         else
        //         {
        //             for (int i = 0; i < (valueIdx + length) / 30; i++)
        //             {
        //                 SDMMCFile.readFile("/TestDothi1.dat", (uint8_t *)&data, sizeof(RecordData_t), (i * 30) * sizeof(RecordData_t));
        //                 snprintf(timeText, sizeof(timeText), "%02u:%02u:%02u", data.hour, data.minute, data.second);
        //                 _dwin.setText(_VPAddressGraphXValueText1 + 10 * i, timeText);
        //             }
        //         }

        //         delay(1);
        //         _dwin.sendGraphValue(0, values, length);
        //         valueIdx += length;

        //         float maxValue, minValue;
        //         if (valueIdx >= 270)
        //         {
        //             length = 270;
        //             SDMMCFile.readFile("/TestDothi1.dat", (uint8_t *)DuLieuDoThi, length * sizeof(RecordData_t), (valueIdx - 270) * sizeof(RecordData_t));
        //         }
        //         else
        //         {
        //             length = valueIdx;
        //             SDMMCFile.readFile("/TestDothi1.dat", (uint8_t *)DuLieuDoThi, length * sizeof(RecordData_t));
        //         }
        //         minValue = DuLieuDoThi[0].value * 10;
        //         maxValue = minValue + 10;
        //         for (int i = 0; i < length; i++)
        //         {
        //             values[i] = DuLieuDoThi[i].value * 10;
        //             if (maxValue < values[i])
        //             {
        //                 maxValue = values[i] + 10;
        //             }
        //             else if (minValue > values[i])
        //             {
        //                 minValue = values[i];
        //             }
        //         }

        //         int VDCentral = (minValue + maxValue) / 2;
        //         int MulY = 150 * 256 / (maxValue - minValue);
        //         _dwin.setGraphVDCentral(_SPAddressSmallGraph1, VDCentral);
        //         _dwin.setGraphMulY(_SPAddressSmallGraph1, MulY);

        //         MulY = 190 * 256 / (maxValue - minValue);
        //         _dwin.setGraphVDCentral(_SPAddressLargeGraph, VDCentral);
        //         _dwin.setGraphMulY(_SPAddressLargeGraph, MulY);

        //         float valueStep = (maxValue - minValue) / 5;
        //         _dwin.setText(_VPAddressGraphYValueText1, String(minValue / 10, 1));
        //         for (float i = 1; i < 6; i++)
        //         {
        //             _dwin.setText(_VPAddressGraphYValueText1 + i * 5, String((minValue + i * valueStep) / 100, 1));
        //         }
        //     }
        //     else if (GiaTriThanhCuon - GiaTriThanhCuonTruocDo < 0)
        //     {
        //         if (valueIdx == 271)
        //         {
        //             continue;
        //         }
        //         length = (GiaTriThanhCuonTruocDo - GiaTriThanhCuon) * 2;
        //         GiaTriThanhCuonTruocDo = GiaTriThanhCuon;
        //         valueIdx = valueIdx - 271 - length;
        //         if (valueIdx < 0)
        //         {
        //             valueIdx = 0;
        //         }

        //         _dwin.resetGraph(0);
        //         delay(1);
        //         length = SDMMCFile.readFile("/TestDothi1.dat", (uint8_t *)DuLieuDoThi, 271 * sizeof(RecordData_t), valueIdx * sizeof(RecordData_t)) / sizeof(RecordData_t);
        //         for (int i = 0; i < length / 30; i++)
        //         {
        //             SDMMCFile.readFile("/TestDothi1.dat", (uint8_t *)&data, sizeof(RecordData_t), (i * 30 + valueIdx) * sizeof(RecordData_t));
        //             snprintf(timeText, sizeof(timeText), "%02u:%02u:%02u", data.hour, data.minute, data.second);
        //             _dwin.setText(_VPAddressGraphXValueText1 + 10 * i, timeText);
        //         }
        //         delay(5);
        //         int j;
        //         for (j = 0; j < length / 100; j++)
        //         {
        //             for (int i = valueIdx; i < 100 + valueIdx; i++)
        //             {
        //                 values[i - valueIdx] = DuLieuDoThi[i - valueIdx + j * 100].value * 10;
        //             }
        //             Serial.println("Ve do thi");
        //             _dwin.sendGraphValue(0, values, 100);
        //             valueIdx += 100;
        //             delay(5);
        //         }

        //         if (length % 100 == 0)
        //         {
        //             continue;
        //         }
        //         for (int i = valueIdx; i < (length % 100 + valueIdx); i++)
        //         {
        //             // SDMMCFile.readFile("/TestDothi1.dat", (uint8_t*)&data, sizeof(RecordData_t), i*sizeof(RecordData_t));
        //             values[i - valueIdx] = DuLieuDoThi[i - valueIdx + j * 100].value * 10;
        //         }
        //         Serial.println("Ve do thi");
        //         _dwin.sendGraphValue(0, values, length % 100);
        //         Serial.println("length: " + String(length));
        //         valueIdx += length % 100;
        //         delay(5);
        //     }
        // }
        // else
        // {
        //     countTest = 0;
        // }
        // delay(100);
        delay(1);
    }
}

void TaskXuLyCanhBao(void*) {
    bool TT_CanhBaoNhietDoCao = false;
    bool TT_CanhBaoNhietDoThap = false;
    bool TT_CanhBaoLoiCamBien = false;

    for (;;) {
        if (FlagNhietDoXacLap && BaseProgram.machineState) {
            if (BaseProgram.temperature > 350 && TT_CanhBaoLoiCamBien == false) {
                _dwin.HienThiWarning("Err: Temperature sensor", _HomePage);
                _dwin.Buzzer(160);
                TT_CanhBaoLoiCamBien = true;
            }
            else if (BaseProgram.temperature < 350) {
                TT_CanhBaoLoiCamBien = false;  // Reset lại khi cảm biến bình thường
            }

            if (BaseProgram.temperature >= BaseProgram.programData.setPointTemp + BaseProgram.programData.tempMax && TT_CanhBaoNhietDoCao == false && TT_CanhBaoLoiCamBien == false && FlagNhietDoXacLap == true) {
                TT_CanhBaoNhietDoCao = true;
                _dwin.HienThiWarning("Alarm: Overheat", _HomePage);
                _dwin.Buzzer(160);
            }
            else if (BaseProgram.temperature <= BaseProgram.programData.setPointTemp + BaseProgram.programData.tempMin && TT_CanhBaoNhietDoThap == false && TT_CanhBaoLoiCamBien == false && FlagNhietDoXacLap == true) {
                TT_CanhBaoNhietDoThap = true;
                _dwin.HienThiWarning("Alarm: Underheat", _HomePage);
                _dwin.Buzzer(160);
            }
            else if (BaseProgram.temperature < BaseProgram.programData.setPointTemp + BaseProgram.programData.tempMax && BaseProgram.temperature > BaseProgram.programData.setPointTemp + BaseProgram.programData.tempMin && TT_CanhBaoLoiCamBien == false) {
                TT_CanhBaoNhietDoCao = false;   // Reset cảnh báo nhiệt độ cao
                TT_CanhBaoNhietDoThap = false;  // Reset cảnh báo nhiệt độ thấp
            }
        }
        CapNhatTrangThaiHeater();
        delay(100);
    }
}

void TaskExportData(void*) {
    for (;;) {
        if (xSemaphoreTake(SemaExportData, portMAX_DELAY) == pdTRUE) {
            if (!USB_MSC_HOST.isConnected()) {
                _dwin.HienThiPhanTramThanhLoading("Fail");
                _dwin.HienThiThanhLoading(0);
                continue;
            }
            copyFiles(USB_MSC_HOST, SD_MMC);  // Test
        }
        // UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // Serial.printf("%s: Task's stack high water mark: %lu words\n", __func__, stackHighWaterMark);
    }
}

void TaskUpdateFirmware(void*) {
    for (;;) {
        MethodUpdates_t method;
        if (QueueUpdateFirmware != NULL) {
            if (xQueueReceive(QueueUpdateFirmware, &method, portMAX_DELAY) == pdPASS) {
                if (TaskCanhBaoHdl != NULL) {
                    Serial.println("Suspend: TaskCanhBaoHdl");
                    vTaskSuspend(TaskCanhBaoHdl);
                }
                if (TaskEspnowHdl != NULL) {
                    Serial.println("Suspend: TaskEspnowHdl");
                    vTaskSuspend(TaskEspnowHdl);
                }
                if (TaskVeDoThiHdl != NULL) {
                    Serial.println("Suspend: TaskVeDoThiHdl");
                    vTaskSuspend(TaskVeDoThiHdl);
                }
                if (TaskExportDataHdl != NULL) {
                    Serial.println("Suspend: TaskExportDataHdl");
                    vTaskSuspend(TaskExportDataHdl);
                }

                if (method == MAIN_UPDATE_USB) {
                    if (USB_MSC_HOST.isConnected()) {
                        // Kết thúc SD_MMC trước khi cập nhật để tránh lỗi
                        SD_MMC.end();

                        // Update firmware
                        updateFirmware(USB_MSC_HOST);

                        // Update HMI
                        updateHMI(USB_MSC_HOST);

                        _dwin.HienThiWarning("Done", _WarningPage);
                        delay(1000);
                        _dwin.HienThiWarning("Restart after 2s", _WarningPage);
                        delay(1000);
                        _dwin.HienThiWarning("Restart after 1s", _WarningPage);
                        delay(1000);
                        _dwin.HienThiWarning("Restarting...", _WarningPage);
                        ESP.restart();
                    }
                }
                else if (method == MAIN_UPDATE_FOTA) {
                    Serial.println("Current Version: " + convertDateToVersion(__DATE__));
                    if (!downloadUpdatesFromJson(SD_MMC)) {
                        Serial.println("Error downloading updates or no new version available");
                        _dwin.HienThiWarning("No new version available", _UpdatePage);
                        // _dwin.setPage(_HomePage);
                        vTaskDelete(NULL);
                        // continue;
                    }
                    else {

                        // // Kết thúc SD_MMC trước khi cập nhật để tránh lỗi
                        // SD_MMC.end();

                        // Update firmware
                        updateFirmware(SD_MMC);

                        // Update HMI
                        updateHMI(SD_MMC);
                        SD_MMC.end();

                        _dwin.HienThiWarning("Done", _WarningPage);
                        delay(1000);
                        _dwin.HienThiWarning("Restart after 2s", _WarningPage);
                        delay(1000);
                        _dwin.HienThiWarning("Restart after 1s", _WarningPage);
                        delay(1000);
                        _dwin.HienThiWarning("Restarting...", _WarningPage);
                        ESP.restart();
                    }
                }

                if (TaskEspnowHdl != NULL) {
                    vTaskResume(TaskEspnowHdl);
                }
                else {
                    ESP.restart();
                }
                if (TaskVeDoThiHdl != NULL) {
                    vTaskResume(TaskVeDoThiHdl);
                }
                else {
                    ESP.restart();
                }
                if (TaskCanhBaoHdl != NULL) {
                    vTaskResume(TaskCanhBaoHdl);
                }
                else {
                    ESP.restart();
                }
                if (TaskExportDataHdl != NULL) {
                    vTaskResume(TaskExportDataHdl);
                }
                else {
                    ESP.restart();
                }
            }
        }
        delay(100);
    }
}

// truc them
// Sửa lại các hàm icon, hiển thị gọi bên HMI.h
void TaskKetNoiWiFi(void*) {
    for (;;) {
        if (xSemaphoreTake(SemaKetNoiWiFi, 5000 / portTICK_PERIOD_MS) == pdTRUE) {
            _dwin.HienThiTrangThaiKetNoiWiFi("");
            if (WiFi.status() != WL_CONNECTED) {
                // Kết nối Wi-Fi
                WiFi.begin(WifiSSID.c_str(), WifiPassword.c_str());
                Serial.println("Connecting to " + WifiSSID);
                _dwin.HienThiTrangThaiKetNoiWiFi("Connecting to " + WifiSSID);
                for (uint8_t i = 0; i < 15; i++) {
                    if (WiFi.status() == WL_CONNECTED) {
                        _dwin.setVP(_VPAddressIonConnect, 2);
                        _dwin.setVP(_VPAddressIconWiFi, map(constrain(WiFi.RSSI(), -100, -40), -100, -40, 1, 4));

                        strcpy(WiFiConfig.ssid, WifiSSID.c_str());
                        strcpy(WiFiConfig.password, WifiPassword.c_str());
                        WiFiConfig.state = true;
                        SDMMCFile.writeFile("/WiFiConfig.txt", (uint8_t*)&WiFiConfig, sizeof(WiFiConfig));
                        _dwin.HienThiTrangThaiKetNoiWiFi("Connected, MAC: " + String(WiFi.macAddress()));
                        // setup_PostGet();
                        // setupMQTT();
                        // Create queue
                        dataQueue = xQueueCreate(10, sizeof(PIDData));
                        if (dataQueue == NULL) {
                            Serial.println("Failed to create queue");
                        }
                        break;
                    }
                    delay(1000);
                }
                if (WiFi.status() != WL_CONNECTED) {
                    _dwin.setVP(_VPAddressIonConnect, 0);
                    _dwin.HienThiTrangThaiKetNoiWiFi("Failed to connect");
                    continue;
                }
            }
            else {
                WiFi.disconnect();
                delay(100);
                if (WiFi.status() != WL_CONNECTED) {
                    _dwin.setVP(_VPAddressIonConnect, 0);
                    _dwin.setVP(_VPAddressIconWiFi, 0);
                    WiFiConfig.state = false;
                    SDMMCFile.writeFile(WIFICONFIG_PATH, (uint8_t*)&WiFiConfig, sizeof(WiFiConfig));
                }
            }
            UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
            Serial.printf("%s: Task's stack high water mark: %lu words\n", __func__, stackHighWaterMark);
        }
        else {
            if (WiFi.status() == WL_CONNECTED) {
                _dwin.setVP(_VPAddressIconWiFi, map(constrain(WiFi.RSSI(), -100, -40), -100, -40, 1, 4));
            }
            else {
                _dwin.setVP(_VPAddressIconWiFi, 0);
            }
        }
    }
}

// Task này dùng để xử lý trong quá trình tủ chạy như là chuyển segment của program,
// Tắt tủ khi hết thời
// Hiển thị Icon xự kiện USB
// Chạy các chế độ tiệt trùng, program, quick mode
void TaskMain(void*) {
    bool FlagUSB = false;
    for (;;) {

        // Phần xử lý hiển thị icon usb
        if (FlagUSB == false && USB_MSC_HOST.isConnected()) {
            FlagUSB = true;
            _dwin.HienThiIconUSB(FlagUSB);
        }
        else if (FlagUSB == true && !USB_MSC_HOST.isConnected()) {
            FlagUSB = false;
            _dwin.HienThiIconUSB(FlagUSB);
        }

        // Phần xác định nhiệt độ đã xác lập hay chưa
        if (fabs(BaseProgram.temperature - BaseProgram.programData.setPointTemp) <= 0.15) {
            if (FlagNhietDoXacLap == false) {
                if (BaseProgram.delayOffState) {
                    DemThoiGianChay(true, DEM_XUONG);
                }
            }
            FlagNhietDoXacLap = true;
        }

        // Phần kiểm tra và cập nhật thời gian đếm xuống / đếm lên và set cờ chuyển segment = true nếu thời gian đềm ngược về 0
        if (BaseProgram.machineState == true) {
            // Nhiệt độ xác lập và trạng thái hẹn giờ là đếm xuống thì cho chạy đếm xuống
            if (BaseProgram.delayOffState && FlagNhietDoXacLap) {
                // Kiểm tra đã đếm thời gian về 00:00:00 chưa
                if (DemThoiGianChay(false, DEM_XUONG) == false) {
                    Serial.println("Chuyen segment");
                    SwitchSegment = true;
                    FlagNhietDoXacLap = false;
                    _dwin.Buzzer(400);
                    if (RunMode == STERILIZATION_MODE) {
                        // tắt máy
                        BaseProgram.machineState = false;
                        _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);
                        _dwin.HienThiThoiGianChay("FINISH");
                        xSemaphoreGive(SemaCaiDatHeater);
                        _dwin.Buzzer(800);
                    }
                }
            }
            else if (BaseProgram.delayOffState == false) {
                DemThoiGianChay(false, DEM_LEN);
            }
        }

        // Phần xử lý chuyển segment và tắt máy
        // if(RunMode == PROGRAM_MODE && SwitchSegment == true && BaseProgram.machineState == true)
        // if (RunMode == PROGRAM_MODE && SwitchSegment == true)
        if (SwitchSegment == true) {
            SwitchSegment = false;
            if (RunMode == PROGRAM_MODE) {
                if (CurrentProgram.size() > RunningSegmentIndex) {
                    BaseProgram.programData = CurrentProgram.at(RunningSegmentIndex);
                    // BaseProgram.machineState = true;
                    BaseProgram.delayOffState = true;
                    FlagNhietDoXacLap = false;
                    xSemaphoreGive(SemaCaiDatHeater);
                    // _dwin.HienThiIconTrangThaiRun(true);
                    _dwin.HienThiSetpointTemp(BaseProgram.programData.setPointTemp);
                    _dwin.HienThiSetpointCO2(BaseProgram.programData.setPointCO2);
                    _dwin.HienThiTocDoQuat(BaseProgram.programData.fanSpeed);
                    _dwin.HienThiThoiGianChay(BaseProgram.programData.delayOffDay, BaseProgram.programData.delayOffHour, BaseProgram.programData.delayOffMinute, 0);
                    // _dwin.HienThiGocFlap(BaseProgram.programData.flap);
                    _dwin.HienThiSegmentDangChay(RunningSegmentIndex + 1, CurrentProgram.size());
                    _dwin.HienThiIconSegment(true);
                    if (BaseProgram.programData.delayOffDay == 0 && BaseProgram.programData.delayOffHour == 0 && BaseProgram.programData.delayOffMinute == 0) {
                        _dwin.HienThiIconOnOffDelayOff(false);
                    }
                    else {
                        _dwin.HienThiIconOnOffDelayOff(true);
                    }
                    RunningSegmentIndex++;
                }
                else {
                    programLoopCount++;
                    if (programLoopCount < programLoop && programInf == false) {
                        // BaseProgram.machineState = true;
                        FlagNhietDoXacLap = false;
                        SwitchSegment = true;
                        RunningSegmentIndex = 0;
                        _dwin.HienThiVongLapChuongTrinhConLai(programLoopCount + 1, programLoop);
                        if (BaseProgram.delayOffState) {
                            DemThoiGianChay(true, DEM_XUONG);
                        }
                        else {
                            DemThoiGianChay(true, DEM_LEN);
                        }
                    }
                    else if (programInf) {
                        // BaseProgram.machineState = true;
                        FlagNhietDoXacLap = false;
                        SwitchSegment = true;
                        RunningSegmentIndex = 0;
                        if (BaseProgram.delayOffState) {
                            DemThoiGianChay(true, DEM_XUONG);
                        }
                        else {
                            DemThoiGianChay(true, DEM_LEN);
                        }
                    }
                    else {
                        // tắt máy
                        BaseProgram.machineState = false;
                        _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);
                        _dwin.HienThiThoiGianChay("FINISH");
                        xSemaphoreGive(SemaCaiDatHeater);
                        _dwin.Buzzer(800);
                    }
                }
            }
        }

        delay(999);
        // UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // Serial.printf("%s: Task's stack high water mark: %lu words\n", __func__, stackHighWaterMark);
    }
}

void TaoCacThuMucHeThongTrenSD(void) {
    // program, log
    SDMMCFile.createDir("/program");
    SDMMCFile.createDir("/log");
    SDMMCFile.createDir("/Update");
    SDMMCFile.createDir("/Update/Firmware");
    SDMMCFile.createDir("/Update/HMI");
    SDMMCFile.createDir("/UpdateStatus");
}

void setup() {
    Serial.begin(115200);

    // pinMode(_HMIPin, OUTPUT);
    // digitalWrite(_HMIPin, HIGH);
    delay(1000);

    /*------------------------------------RTC-----------------------------------*/
    // Kiểm tra kết nối DS3231
    Wire.begin(10, 11);

    // Wire.begin(8, 9);

    _time.begin();
    _time.getCurrentTime();  // Gọi hàm này trước khi get thời gian
    setTimeFromRTC(_time.getYear(), _time.getMonth(), _time.getDay(), _time.getHour(), _time.getMinute(), _time.getSecond());
    struct tm tmstruct;
    delay(1000);
    tmstruct.tm_year = 0;
    getLocalTime(&tmstruct, 5000);
    Serial.printf(
        "\nNow is : %d-%02d-%02d %02d:%02d:%02d\n", (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min,
        tmstruct.tm_sec);
    Serial.println("");

    _dwin.KhoiTao();
    _dwin.CRCEnabled();
    _dwin.DangKyHamSetCallback(hmiSetEvent);
    _dwin.DangKyHamGetCallback(hmiGetEvent);

    // SDMMCFile.begin();
    SD_MMC.setPins(clk, cmd, d0);
    if (!SD_MMC.begin("/sdcard", onebit, true, 4000000)) {
        Serial.println("Mount Failed");
    }
    else {
        Serial.println("SD_MMC initialized successfully");
        checkAndResumeUpdates(SD_MMC);
        SDMMCFile.setFileSystem(SD_MMC);
        TaoCacThuMucHeThongTrenSD();
    }
    delay(1000);
    listFilesInProgramDirectory();

    USB_MSC_HOST.begin();
    UsbFile.setFileSystem(USB_MSC_HOST);
    delay(1000);
    if (USB_MSC_HOST.isConnected()) {
        checkAndResumeUpdates(USB_MSC_HOST);
    }

    // Khởi tạo SD_MMC
    // Serial.println("Mounting SDcard");
    // SPI.begin(sck, miso, mosi, cs);
    // if (!SD.begin(cs, SPI, 8000000)) {
    //     Serial.println("Card Mount Failed");
    //     // return;
    // }
    // else {
    //     checkAndResumeUpdates(SD);
    // }

    KhoiTaoHeater();
    delay(10);

    // Khôi phục thông số hệ thống
    // Khôi phục danh sách hệ số calib.
    if (SDMMCFile.exists(PATH_CALIB_DATA)) {
        DanhSachHeSoCalib.resize(SDMMCFile.sizeFile(PATH_CALIB_DATA) / sizeof(CalibData_t));
        if (SDMMCFile.readFile(PATH_CALIB_DATA, (uint8_t*)DanhSachHeSoCalib.data(), SDMMCFile.sizeFile(PATH_CALIB_DATA))) {
            for (CalibData_t HeSoCalib : DanhSachHeSoCalib) {
                Serial.printf("Setpoint: %.1f, value: %.2f\n", HeSoCalib.Setpoint, HeSoCalib.value);
            }
        }
    }

    // Khôi phục thông số chương trình cơ sở.
    if (SDMMCFile.exists(PATH_BASEPROGRAM_DATA)) {
        SDMMCFile.readFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
    }
    else {
        BaseProgram.programData.setPointTemp = 37;
        BaseProgram.programData.setPointCO2 = 5;
        BaseProgram.programData.fanSpeed = 50;
        BaseProgram.programData.delayOffDay = 0;
        BaseProgram.programData.delayOffHour = 1;
        BaseProgram.programData.delayOffMinute = 0;
        // BaseProgram.programData.flap = 10;
        BaseProgram.programData.tempMax = 1;
        BaseProgram.programData.tempMin = -1;
        BaseProgram.programData.CO2Max = 1.1;
        BaseProgram.programData.CO2Min = -1.1;
        BaseProgram.machineState = false;
        BaseProgram.delayOffState = false;
        SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
    }

    if (SDMMCFile.exists(PATH_RECORD)) {
        RecordData_t data;
        int filesize = SDMMCFile.sizeFile(PATH_RECORD);
        for (int i = 0; i < filesize / sizeof(data); i++) {
            SDMMCFile.readFile(PATH_RECORD, (uint8_t*)&data, sizeof(data), i * sizeof(data));
            // Serial.printf("SP: %.1f, Value: %.1f, Fan: %u, Flap: %u\n", data.setpoint, data.value, data.fan, data.flap);
            Serial.printf("SP: %.1f, Value: %.1f, Fan: %u\n", data.setpointTemp, data.valueTemp, data.fan);
        }
    }

    if (!SDMMCFile.exists("/usbmode.txt")) {
        uint8_t mode = 0;
        SDMMCFile.writeFile("/usbmode.txt", (uint8_t*)&mode, sizeof(mode));
    }
    else {
        uint8_t mode;
        SDMMCFile.readFile("/usbmode.txt", (uint8_t*)&mode, sizeof(mode));
        if (mode == 0) {
        }
        else if (mode == 1) {
            USB_MSC_HOST.end();
            delay(1000);
            sd2usbmsc_init();
        }
    }

    if (SDMMCFile.exists(ADMIN_PASSWORD)) {
        Serial.print("Khoi phuc mat khau admin: ");
        String admin_password = SDMMCFile.readFile(ADMIN_PASSWORD);
        _dwin.ThayDoiUserAdminPassword(admin_password);
        Serial.println(admin_password);
    }

    SemaCaiDatHeater = xSemaphoreCreateBinary();
    SemaExportData = xSemaphoreCreateBinary();
    SemaKetNoiWiFi = xSemaphoreCreateBinary();  // truc them

    QueueUpdateFirmware = xQueueCreate(2, sizeof(MethodUpdates_t));

    RunMode = QUICK_MODE;
    BaseProgram.machineState = false;
    BaseProgram.delayOffState = false;
    _dwin.HienThiSetpointTemp(BaseProgram.programData.setPointTemp);
    _dwin.HienThiSetpointCO2(BaseProgram.programData.setPointCO2);
    _dwin.HienThiTocDoQuat(BaseProgram.programData.fanSpeed);
    // _dwin.HienThiGocFlap(BaseProgram.programData.flap);
    _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);
    _dwin.HienThiIconGiaNhiet(false);
    _dwin.HienThiChuongTrinhDangChay("Quick");
    _dwin.HienThiThoiGianChay("Inf");
    _dwin.HienThiSegmentDangChay("");
    _dwin.HienThiVongLapChuongTrinhConLai("");
    _dwin.HienThiIconSegment(false);
    _dwin.setBrightness(40);
    _dwin.XoaDoThi();
    xSemaphoreGive(SemaCaiDatHeater);

    // Test log
    initLogFileState(SD_MMC);
    _dwin.HienThiThongTinVersion(__TIME__);

    BaseType_t xTaskEspNowReturn = xTaskCreateUniversal(TaskGuiDuLieuEspNow, "Task gui du lieu", 3072, NULL, 5, &TaskEspnowHdl, -1);
    delay(5);
    BaseType_t xTaskExportData = xTaskCreateUniversal(TaskExportData, "Task Export data", 5120, NULL, 6, &TaskCanhBaoHdl, -1);
    delay(5);
    // BaseType_t xTaskUpdateFirmware = xTaskCreateUniversal(TaskUpdateFirmware, "Task update firmware", 8192, NULL, 4, &TaskUpdateFirmwareHdl, -1);
    // delay(5);
    BaseType_t xTaskKetNoiWiFi = xTaskCreateUniversal(TaskKetNoiWiFi, "Task ket noi wifi", 4096, NULL, 1, &TaskKetNoiWiFiHdl, -1);  // truc them
    delay(5);
    BaseType_t xTaskMain = xTaskCreateUniversal(TaskMain, "Task main", 3072, NULL, 9, &TaskMainHdl, -1);
    delay(5);
    BaseType_t xTaskVeDoThiReturn = xTaskCreateUniversal(TaskHienThiNhietDoVaVeDoThi, "Task hien thi nhiet va ve do thi", 5120, NULL, 8, &TaskVeDoThiHdl, -1);
    delay(5);
    _dwin.setPage(_HomePage);
    BaseType_t xTaskXuLyCanhBaoReturn = xTaskCreateUniversal(TaskXuLyCanhBao, "Task xu ly canh bao", 3072, NULL, 7, &TaskCanhBaoHdl, -1);
    delay(5);

    if (SDMMCFile.exists(WIFICONFIG_PATH)) {
        SDMMCFile.readFile(WIFICONFIG_PATH, (uint8_t*)&WiFiConfig, sizeof(WiFiConfig));
        WifiSSID = String(WiFiConfig.ssid);
        WifiPassword = String(WiFiConfig.password);
        _dwin.HienThiSSIDWiFi(WifiSSID);
        _dwin.HienThiPasswordWiFi(WifiPassword);
        Serial.print("Thong tin Wifi");
        Serial.println(WifiSSID);
        Serial.println(WifiPassword);
        if (WiFiConfig.state) {
            xSemaphoreGive(SemaKetNoiWiFi);
        }
    }
    // _dwin.echoEnabled(true);
}

void loop() {
    // Nếu RAM còn trống 70000 byte thì mới thực hiện GET POST
    // if (esp_get_free_heap_size() > 20000)
    // {
    // loop_PostGet();
    // loopMQTT();
    // if(WiFi.status() == WL_CONNECTED) {
    //     http.begin()
    // }
    // delay(1000);
    //     if (dataQueue != NULL)
    //     {
    //         PIDData data;
    //         if (xQueueReceive(dataQueue, &data, 0) == pdTRUE)
    //         {
    //             sendData(&data);
    //         }
    //     }
    // }
    // else {
    //     delay(10);
    // }
}

// void sendData(PIDData *data)
// {
//     HTTPClient http;
//     http.begin("https://app.iottest.io.vn/pidparam/data");
//     http.addHeader("Content-Type", "application/json");

//     // Create JSON object
//     StaticJsonDocument<512> jsonDoc;
//     jsonDoc["Setpoint"] = data->Setpoint;
//     jsonDoc["Temperature"] = data->Temperature;
//     jsonDoc["Kp"] = data->Kp;
//     jsonDoc["Ki"] = data->Ki;
//     jsonDoc["Kd"] = data->Kd;
//     jsonDoc["PTerm"] = data->PTerm;
//     jsonDoc["ITerm"] = data->ITerm;
//     jsonDoc["DTerm"] = data->DTerm;
//     jsonDoc["Output"] = data->Output;
//     jsonDoc["Triac"] = data->Triac;
//     jsonDoc["IMax"] = data->IMax;
//     jsonDoc["UMax"] = data->UMax;
//     jsonDoc["TGondinh"] = data->TGondinh;
//     jsonDoc["TGgianhiet"] = data->TGgianhiet;
//     jsonDoc["MAC"] = String(WiFi.macAddress());

//     String jsonString;
//     serializeJson(jsonDoc, jsonString);
//     Serial.print(jsonString);

//     // Send HTTP POST request
//     int httpResponseCode = http.POST("[" + jsonString + "]");
//     if (httpResponseCode > 0)
//     {
//         Serial.printf("HTTP Response code: %d\n", httpResponseCode);
//     }
//     else
//     {
//         Serial.printf("Error in HTTP request: %s\n", http.errorToString(httpResponseCode).c_str());
//     }
//     http.end();
// }
