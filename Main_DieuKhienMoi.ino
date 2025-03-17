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
#include <esp_task_wdt.h>


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

#include "09_concentration.h"
#include "07_Heater.h"
#include "Door.h"

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
Concentration _CO2;
HEATER _Heater;
DOOR _Door;

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

TaskHandle_t TaskHMIHdl = NULL;
TaskHandle_t TaskExportDataHdl = NULL;
TaskHandle_t TaskUpdateFirmwareHdl = NULL;
TaskHandle_t TaskMainHdl;
TaskHandle_t TaskKetNoiWiFiHdl;  // truc them

SemaphoreHandle_t SemaKetNoiWiFi;  // truc them

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

QueueHandle_t recvHMIQueue;
QueueHandle_t QueueUpdateFirmware;


// các hàm khởi tạo và hỗ trợ khởi tạo
void khoiTaoWDT(uint32_t miliSecond);
void khoiTaoDWIN();
void khoiTaoSDCARD();
void khoiTaoRTC();
void khoiTaoHeater();
void khoiTaoCO2();
void khoiTaoCua();
void listFilesInProgramDirectory(void);
void TaoCacThuMucHeThongTrenSD(void);

//các hàm chạy run time hoặc call back
void BatMay(const char* funcCall);
void TatMay(const char* funcCall);
float GetCalib(float value);
bool DemThoiGianChay(bool reset, bool DemXuong);
void setTimeFromRTC(int year, int month, int day, int hour, int minute, int second);
void hmiSetEvent(const hmi_set_event_t& event);
bool hmiGetEvent(hmi_get_type_t event, void* args);
void callBackOpenDoor(void* ptr);
void callBackCloseDoor(void* ptr);
static void triggeONICONNhiet(void*);
static void triggeOFFICONNhiet(void*);

//các task
void TaskHMI(void*);
void TaskExportData(void*);
void TaskUpdateFirmware(void*);
void TaskKetNoiWiFi(void*);
void TaskMain(void*);


void setup() {
  // khoiTaoWDT(10000); // timeout watch dog 10s
  Serial.begin(115200);
  Wire.begin(10, 11);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  SemaKetNoiWiFi = xSemaphoreCreateBinary();  // truc them
  QueueUpdateFirmware = xQueueCreate(2, sizeof(MethodUpdates_t));
  recvHMIQueue = xQueueCreate(10, sizeof(FrameDataQueue_t));

  khoiTaoRTC();
  delay(10);

  khoiTaoDWIN();
  delay(10);

  khoiTaoSDCARD();
  delay(10);

  khoiTaoCua();
  delay(10);

  khoiTaoHeater();
  delay(10);

  khoiTaoCO2();
  delay(10);

  delay(2000);
  _dwin.echoEnabled(true);
  RunMode = QUICK_MODE;
  BaseProgram.machineState = false;
  BaseProgram.delayOffState = false;
  Serial.println(BaseProgram.programData.setPointTemp);
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

  _dwin.HienThiThongTinVersion(__TIME__);
  _dwin.SetupDoThiNho(BaseProgram);
  _dwin.setPage(_HomePage);
  delay(1000);

  xTaskCreateUniversal(TaskExportData, "tskExport", 8192, NULL, 4, &TaskExportDataHdl, -1);
  delay(5);
  xTaskCreateUniversal(TaskUpdateFirmware, "tskFirmware", 8192, NULL, 5, &TaskUpdateFirmwareHdl, -1);
  delay(5);
  xTaskCreateUniversal(TaskKetNoiWiFi, "tskWifi", 8192, NULL, 1, &TaskKetNoiWiFiHdl, -1);  // truc them
  delay(5);
  xTaskCreateUniversal(TaskMain, "tskMain", 8192, NULL, 3, &TaskMainHdl, -1);
  delay(5);
  xTaskCreateUniversal(TaskHMI, "tskHMI", 8192, NULL, 2, &TaskHMIHdl, -1);
  delay(5);
}

void loop() {

  // Nếu RAM còn trống 70000 byte thì mới thực hiện GET POST
  if (esp_get_free_heap_size() > 20000) {
    loop_PostGet();
    // loopMQTT();
    // if (WiFi.status() == WL_CONNECTED) {
    //     http.begin();
    // }
  }
  delay(1);
}

void khoiTaoWDT(uint32_t miliSecond) {
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = miliSecond,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // Bitmask of all cores
    .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&twdt_config);
  esp_task_wdt_add(NULL);
}

void khoiTaoRTC() {
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
}
void khoiTaoDWIN() {
  _dwin.KhoiTao();
  _dwin.CRCEnabled();
  _dwin.DangKyHamSetCallback(hmiSetEvent);
  _dwin.DangKyHamGetCallback(hmiGetEvent);
}
void khoiTaoSDCARD() {
  // SDMMCFile.begin();
  SD_MMC.setPins(clk, cmd, d0);
  if (!SD_MMC.begin("/sdcard", onebit, true, 4000000)) {
    Serial.println("Mount Failed");
  } else {
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
    if (BaseProgram.machineState == true) {
    }
  } else {
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
  } else {
    uint8_t mode;
    SDMMCFile.readFile("/usbmode.txt", (uint8_t*)&mode, sizeof(mode));
    if (mode == 0) {
    } else if (mode == 1) {
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

  // Test log
  initLogFileState(SD_MMC);

  if (SDMMCFile.exists(WIFICONFIG_PATH)) {
    SDMMCFile.readFile(WIFICONFIG_PATH, (uint8_t*)&WiFiConfig, sizeof(WiFiConfig));
    WifiSSID = String(WiFiConfig.ssid);
    WifiPassword = String(WiFiConfig.password);
    _dwin.HienThiSSIDWiFi(WifiSSID);
    _dwin.HienThiPasswordWiFi(WifiPassword);
    Serial.print("Thong tin Wifi: ");
    Serial.print(WifiSSID + " ");
    Serial.println(WifiPassword);
    if (WiFiConfig.state) {
      xSemaphoreGive(SemaKetNoiWiFi);
    }
  }
}
void khoiTaoHeater() {
  _Heater.KhoiTao();
  // bật quạt
  _Heater.CaiTocDoQuat(BaseProgram.programData.fanSpeed);
  _Heater.TurnOnTriac();
  _Heater.addCallBackWritePinTriacBuong(triggeONICONNhiet, NULL);  //trigger on ICON nhiệt
  _Heater.addCallBackTimeOutTriacBuong(triggeOFFICONNhiet, NULL);  //trigger off ICON nhiệt
  if (recvHMIQueue == NULL) {
    return;
  }
  static FrameDataQueue_t data;
  data.event = eEVENT_ICON_NHIET;
  xQueueSend(recvHMIQueue, &data, 10);
  data.event = eEVENT_ICON_FAN;
  xQueueSend(recvHMIQueue, &data, 10);
  data.event = eEVENT_WARNING;
  xQueueSend(recvHMIQueue, &data, 10);
}
void khoiTaoCO2() {
  _CO2.KhoiTaoCO2();
  _CO2.addCallBackWritePin(triggeONICONCO2, NULL);  //trigger on ICON nhiệt
  _CO2.addCallBackTimeout(triggeOFFICONCO2, NULL);  //trigger off ICON nhiệt
  if (recvHMIQueue == NULL) {
    return;
  }
  static FrameDataQueue_t data;
  data.event = eEVENT_WARNING;
  xQueueSend(recvHMIQueue, &data, 10);
  data.event = eEVENT_ICON_CO2;
  xQueueSend(recvHMIQueue, &data, 10);
}
void khoiTaoCua() {
  _Door.KhoiTao();
  _Door.addCloseFuncCallBack(callBackCloseDoor, NULL);
  _Door.addOpenFuncCallBack(callBackOpenDoor, NULL);
  if (recvHMIQueue == NULL) {
    return;
  }
  static FrameDataQueue_t data;
  data.event = eEVENT_ICON_CUA;
  xQueueSend(recvHMIQueue, &data, 0);
}
void callBackOpenDoor(void* ptr) {
  //!check ptr trước khi dùng nhá
  Serial.printf("Open door\n");
  static FrameDataQueue_t data;
  data.event = eEVENT_ICON_CUA;
  if (recvHMIQueue == NULL) {
    return;
  }
  xQueueSend(recvHMIQueue, &data, 0);
  digitalWrite(RELAY_PIN, LOW);
  _Heater.SetEventDOOR();
  _CO2.SetEventDOOR();
}
void callBackCloseDoor(void* ptr) {
  //!check ptr trước khi dùng nhá
  Serial.printf("Close door\n");
  FlagNhietDoXacLap = false;
  static FrameDataQueue_t data;
  data.event = eEVENT_ICON_CUA;
  if (recvHMIQueue == NULL) {
    return;
  }
  xQueueSend(recvHMIQueue, &data, 0);
  digitalWrite(RELAY_PIN, HIGH);
  _Heater.ResetEventDOOR();
  _CO2.ResetEventDOOR();
}

void BatMay(const char* funcCall) {
  Serial.printf("\t\t\tBat may: call from %s\n", funcCall ? funcCall : "NULL");
  _Heater.BatDieuKhienNhietDo();
  digitalWrite(RELAY_PIN, HIGH);
  _CO2.BatDieuKhienCO2();
  SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
  if (_Door.TrangThai() == DOOR_OPEN) {
    _CO2.SetEventDOOR();
    _Heater.SetEventDOOR();
  }
}
void TatMay(const char* funcCall) {
  Serial.printf("\t\t\tTat may: call from %s\n", funcCall ? funcCall : "NULL");
  _Heater.TatDieuKhienNhietDo();
  digitalWrite(RELAY_PIN, LOW);
  _CO2.TatDieuKhienCO2();
  SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
}

float GetCalib(float value) {
  for (CalibData_t HeSoCalib : DanhSachHeSoCalib) {
    if (fabs(HeSoCalib.Setpoint - value) < 0.02) {
      // Serial.printf("Setpoint: %.1f, value: %.2f\n", HeSoCalib.Setpoint, HeSoCalib.value);
      return HeSoCalib.value;
    }
  }
  return 0;
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
    } else {
      _time.start(BaseProgram.programData.delayOffDay * 86400 + BaseProgram.programData.delayOffHour * 3600 + BaseProgram.programData.delayOffMinute * 60);
      _dwin.HienThiIconOnOffDelayOff(false);
    }
  } else {
    if (DemXuong) {
      timeRun = _time.getRemainingTime();  // Lấy thời gian còn lại
    } else {
      timeRun = _time.getElapsedTime();  // Lấy thời gian đang đếm
    }
    _dwin.HienThiThoiGianChay((int)timeRun / 86400, (int)timeRun % 86400 / 3600, (int)timeRun % 86400 % 3600 / 60, (int)timeRun % 86400 % 3600 % 60);
  }
  if (_time.isFinished()) {
    return false;
  }
  return true;
}

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
        } else {
          _dwin.HienThiVongLapChuongTrinhConLai("");
        }
        if (BaseProgram.delayOffState) {
          DemThoiGianChay(true, DEM_XUONG);
          // _time.reset();
        } else {
          DemThoiGianChay(true, DEM_LEN);
          // _time.reset();
        }
        _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);

      } else {
        BaseProgram.machineState = false;
        if (BaseProgram.delayOffState) {
          _dwin.HienThiThoiGianChay(BaseProgram.programData.delayOffDay, BaseProgram.programData.delayOffHour, BaseProgram.programData.delayOffMinute, 0);
        } else {
          _dwin.HienThiThoiGianChay("Inf");
        }
        _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);
      }
      Serial.printf("HMI_SET_RUN_ONOFF\n");
      break;
    case HMI_SET_SETPOINT_TEMP:
      Serial.printf("Set setpoint temp: %.1f\n", event.f_value);
      BaseProgram.programData.setPointTemp = event.f_value;
      if (RunMode == QUICK_MODE) {
        SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
      }
      FlagNhietDoXacLap = false;

      break;
    case HMI_SET_SETPOINT_CO2:
      Serial.printf("Set setpoint CO2: %.1f\n", event.f_value);
      BaseProgram.programData.setPointCO2 = event.f_value;
      if (RunMode == QUICK_MODE) {
        SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
      }
      FlagCO2XacLap = false;

      break;
    case HMI_SET_FAN:
      Serial.printf("Set fanspeed: %.1f\n", event.f_value);
      BaseProgram.programData.fanSpeed = (int8_t)event.f_value;
      if (RunMode == QUICK_MODE) {
        SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
      }

      break;
    case HMI_SET_CALIB_NHIET:
      {
        Serial.printf("Set calib: %.1f\n", event.f_value);
        // Kiểm tra đã có hệ số calib cũ chưa nếu có thì cập nhật lại hệ số calib
        CalibData_t HeSoCalib;
        for (int i = 0; i < DanhSachHeSoCalib.size(); i++) {
          if (fabs(DanhSachHeSoCalib.at(i).Setpoint - BaseProgram.programData.setPointTemp) < 0.01) {
            DanhSachHeSoCalib[i].value = event.f_value - (BaseProgram.temperature - DanhSachHeSoCalib[i].value);
            SDMMCFile.writeFile(PATH_CALIB_DATA, (uint8_t*)DanhSachHeSoCalib.data(), DanhSachHeSoCalib.size() * sizeof(CalibData_t));
            Serial.println("Thay Doi he so calib da co");

            return;
          }
        }
        HeSoCalib.Setpoint = BaseProgram.programData.setPointTemp;
        HeSoCalib.value = event.f_value - BaseProgram.temperature;
        DanhSachHeSoCalib.push_back(HeSoCalib);
        SDMMCFile.writeFile(PATH_CALIB_DATA, (uint8_t*)DanhSachHeSoCalib.data(), DanhSachHeSoCalib.size() * sizeof(CalibData_t));

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

            return;
          }
        }
        break;
      }
    case HMI_SET_CALIB_CO2:
      {
        Serial.printf("Set calib: %.1f\n", event.f_value);
        _CO2.CalibGiaTriThuc(event.f_value);
        break;
      }
    // case HMI_SET_FLAP:
    //     Serial.printf("Set flap: %.1f\n", event.f_value);
    //     BaseProgram.programData.flap = (int8_t)event.f_value;
    //     if (RunMode == QUICK_MODE)
    //     {
    //         SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
    //     }
    //
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
    case HMI_SET_ALARM_TEMP_BELOW:
      Serial.printf("HMI_SET_ALARM_TEMP_BELOW: %.1f\n", event.f_value);
      BaseProgram.programData.tempMin = event.f_value;
      if (RunMode == QUICK_MODE) {
        SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
      }
      break;
    case HMI_SET_ALARM_TEMP_ABOVE:
      Serial.printf("HMI_SET_ALARM_TEMP_ABOVE: %.1f\n", event.f_value);
      BaseProgram.programData.tempMax = event.f_value;
      if (RunMode == QUICK_MODE) {
        SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
      }
      break;
    case HMI_SET_ALARM_CO2_BELOW:
      Serial.printf("HMI_SET_ALARM_CO2_BELOW: %.1f\n", event.f_value);
      BaseProgram.programData.CO2Min = event.f_value;
      if (RunMode == QUICK_MODE) {
        SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
      }
      break;
    case HMI_SET_ALARM_CO2_ABOVE:
      Serial.printf("HMI_SET_ALARM_CO2_ABOVE: %.1f\n", event.f_value);
      BaseProgram.programData.CO2Max = event.f_value;
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
    case HMI_EDIT_SEG_SETPOINT_TEMP:
      if (ProgramList.size() > event.indexList + listPageStartPosition) {
        ProgramList.at(event.indexList + listPageStartPosition).setPointTemp = event.f_value;
        Serial.printf("Set segment setpoint %u: %.1f\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].setPointTemp);
      }
      break;
    case HMI_EDIT_SEG_SETPOINT_CO2:
      if (ProgramList.size() > event.indexList + listPageStartPosition) {
        ProgramList.at(event.indexList + listPageStartPosition).setPointCO2 = event.f_value;
        Serial.printf("Set segment setpoint CO2 %u: %.1f\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].setPointCO2);
      }
      break;
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
    case HMI_EDIT_SEG_CO2MIN:
      if (ProgramList.size() > event.indexList + listPageStartPosition) {
        ProgramList.at(event.indexList + listPageStartPosition).CO2Min = event.f_value;
        Serial.printf("Set segment CO2 pro min %u: %.1f\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].CO2Min);
      }
      break;
    case HMI_EDIT_SEG_CO2MAX:
      if (ProgramList.size() > event.indexList + listPageStartPosition) {
        ProgramList.at(event.indexList + listPageStartPosition).CO2Max = event.f_value;
        Serial.printf("Set segment CO2 pro max %u: %.1f\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].CO2Max);
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
      } else {
        ProgramList.insert(ProgramList.begin() + (int)event.indexList + listPageStartPosition, newProgram);
      }
      break;
    case HMI_SUB_SEG:
      Serial.println("HMI_SUB_SEG");
      if (ProgramList.size() > 1) {
        if ((int)event.indexList < 5 && ProgramList.size() > (int)event.indexList + listPageStartPosition) {
          ProgramList.erase(ProgramList.begin() + (int)event.indexList + listPageStartPosition);
        } else {
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
        } else {
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
        } else {
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
        BaseProgram.programData.setPointCO2 = 0;
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
      if (TaskExportDataHdl != NULL) {
        Serial.println("Resume: TaskExportDataHdl");
        vTaskResume(TaskExportDataHdl);
      }
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
      } else if (GiaTriThanhCuonTruocDo != GiaTriThanhCuon) {
        TrangThaiThanhCuon = true;
      }
      ThoiGian2LanChamThanhCuon = millis();
      break;
    case HMI_CONNECT_OR_DISCONNECT_WIFI:
      Serial.println("HMI_CONNECT_OR_DISCONNECT_WIFI");
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
      _dwin.HienThiCO2CanhBao(BaseProgram.programData.CO2Min, BaseProgram.programData.CO2Max);
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
        String name = file.name();
        Serial.println(name);
        _dwin.HienThiTenChuongTrinhTrenHang(programListIndex, programListIndex + 1, name, file.size() / sizeof(Program_t), __func__);
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
      } else {
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
          _dwin.HienThiTenChuongTrinhTrenHang(programListIndex - programStart, programListIndex + 1, file.name(), file.size() / sizeof(Program_t), __func__);
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
      } else if (programStart == 0) {
        return 0;
      } else {
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
          _dwin.HienThiTenChuongTrinhTrenHang(programListIndex - programStart, programListIndex + 1, file.name(), file.size() / sizeof(Program_t), __func__);
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
      } else {
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
      } else if (ProgramList.size() <= 5) {
        listLength = ProgramList.size();
      } else if (ProgramList.size() <= listPageStartPosition + 5) {
        listLength = ProgramList.size() - listPageStartPosition;
      } else {
        Serial.println("Segment List is full");
      }
      goto HienThiSegmentList;
      break;
    case HMI_GET_BACK_SEGMENT_LIST:
      if (listPageStartPosition - 5 >= 0) {
        listPageStartPosition -= 5;
        listLength = 5;
      } else if (ProgramList.size() <= 5) {
        listPageStartPosition = 0;
        listLength = ProgramList.size();
      } else if (listPageStartPosition - 5 < 0) {
        listPageStartPosition = 0;
        listLength = 5;
      }
      goto HienThiSegmentList;
      break;
    case HMI_REFRESH_SEGMENT_LIST:
      if (ProgramList.size() <= 5) {
        listLength = ProgramList.size();
      } else if (ProgramList.size() <= listPageStartPosition + 5) {
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
                                         programData.setPointCO2,
                                         programData.delayOffDay,
                                         programData.delayOffHour,
                                         programData.delayOffMinute,
                                         programData.fanSpeed,
                                         // programData.flap,
                                         programData.tempMin,
                                         programData.tempMax,
                                         programData.CO2Min,
                                         programData.CO2Max);
    } else {
      _dwin.XoaDuLieuHienThiSegmentTrenHang(i - listPageStartPosition);
    }
  }
  return 1;
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
static void triggeONICONNhiet(void*) {
  static FrameDataQueue_t data;
  data.event = eEVENT_ICON_NHIET;
  if (recvHMIQueue == NULL) {
    return;
  }
  xQueueSend(recvHMIQueue, &data, 0);
}
static void triggeOFFICONNhiet(void*) {
  static FrameDataQueue_t data;
  data.event = eEVENT_ICON_NHIET;
  if (recvHMIQueue == NULL) {
    return;
  }
  xQueueSend(recvHMIQueue, &data, 0);
}
static void triggeONICONCO2(void*) {
  static FrameDataQueue_t data;
  data.event = eEVENT_ICON_CO2;
  if (recvHMIQueue == NULL) {
    return;
  }
  xQueueSend(recvHMIQueue, &data, 0);
}
static void triggeOFFICONCO2(void*) {
  static FrameDataQueue_t data;
  data.event = eEVENT_ICON_CO2;
  if (recvHMIQueue == NULL) {
    return;
  }
  xQueueSend(recvHMIQueue, &data, 0);
}






void TaskExportData(void*) {
  vTaskSuspend(NULL);
  for (;;) {
    if (USB_MSC_HOST.isConnected()) {
      copyFiles(USB_MSC_HOST, SD_MMC);  // Test
      delay(100);
      Serial.println("Copy done");
      delay(1000);
    } else {
      _dwin.HienThiPhanTramThanhLoading("Fail");
      _dwin.HienThiThanhLoading(0);
    }
    vTaskSuspend(NULL);
    // UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.printf("%s: Task's stack high water mark: %lu words\n", __func__, stackHighWaterMark);
  }
}

void TaskUpdateFirmware(void*) {
  MethodUpdates_t method;
  for (;;) {
    xQueueReceive(QueueUpdateFirmware, &method, portMAX_DELAY);
    Serial.printf("%s nhận phương thức update: %s", __func__, method == MAIN_UPDATE_USB ? "USB" : "FOTA");

    // if (TaskMainHdl != NULL) {
    //     Serial.println("Suspend: TaskMainHdl");
    //     vTaskSuspend(TaskMainHdl);
    // }
    // if (TaskHMIHdl != NULL) {
    //     Serial.println("Suspend: TaskHMI");
    //     vTaskSuspend(TaskHMIHdl);
    // }
    // if (TaskExportDataHdl != NULL) {
    //     Serial.println("Suspend: TaskExportDataHdl");
    //     vTaskSuspend(TaskExportDataHdl);
    // }

    if (method == MAIN_UPDATE_USB) {
      if (USB_MSC_HOST.isConnected()) {
        // Kết thúc SD_MMC trước khi cập nhật để tránh lỗi
        SD_MMC.end();

        _dwin.HienThiWarning("updating firmware", _WarningPage);

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
      } else {
        _dwin.HienThiWarning("no USB device", _UpdatePage);
      }
    } else if (method == MAIN_UPDATE_FOTA) {
      Serial.println("Current Version: " + convertDateToVersion(__DATE__));
      if (!downloadUpdatesFromJson(SD_MMC)) {
        Serial.println("Error downloading updates or no new version available");
        _dwin.HienThiWarning("No new version available", _UpdatePage);
        // _dwin.setPage(_HomePage);
        // vTaskDelete(NULL);
        // continue;
      } else {

        // Kết thúc SD_MMC trước khi cập nhật để tránh lỗi
        // SD_MMC.end();
        _dwin.HienThiWarning("updating firmware", _WarningPage);
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


    // if (TaskMainHdl != NULL) {
    //     Serial.println("Resume: TaskMainHdl");
    //     vTaskResume(TaskMainHdl);
    // }
    // if (TaskHMIHdl != NULL) {
    //     Serial.println("Resume: TaskHMI");
    //     vTaskResume(TaskHMIHdl);
    // }
    // if (TaskExportDataHdl != NULL) {
    //     Serial.println("Resume: TaskExportDataHdl");
    //     vTaskResume(TaskExportDataHdl);
    // }
  }
}

// truc them
// Sửa lại các hàm icon, hiển thị gọi bên HMI.h
void TaskKetNoiWiFi(void*) {
  for (;;) {
    xSemaphoreTake(SemaKetNoiWiFi, portMAX_DELAY);
    Serial.println("TaskKetNoiWiFi RUN");
    _dwin.HienThiTrangThaiKetNoiWiFi("");

    if (WiFi.status() != WL_CONNECTED) {
      // Kết nối Wi-Fi
      Serial.println("Connecting to " + WifiSSID);
      _dwin.HienThiTrangThaiKetNoiWiFi("Connecting to " + WifiSSID);
      WiFi.begin(WifiSSID.c_str(), WifiPassword.c_str());

      uint8_t u8Try = 30;
      while (u8Try--) {
        if (WiFi.status() == WL_CONNECTED) {
          _dwin.setVP(_VPAddressIonConnect, 2);
          _dwin.HienThiTrangThaiKetNoiWiFi("Connected, MAC: " + String(WiFi.macAddress()));
          _dwin.setVP(_VPAddressIconWiFi, map(constrain(WiFi.RSSI(), -100, -40), -100, -40, 1, 4));
          strcpy(WiFiConfig.ssid, WifiSSID.c_str());
          strcpy(WiFiConfig.password, WifiPassword.c_str());
          WiFiConfig.state = true;
          setup_PostGet();
          // setupMQTT();
          SDMMCFile.writeFile(WIFICONFIG_PATH, (uint8_t*)&WiFiConfig, sizeof(WiFiConfig));
          break;
        }
        delay(500);
      }
      if (WiFi.status() != WL_CONNECTED) {
        _dwin.setVP(_VPAddressIonConnect, 0);
        _dwin.HienThiTrangThaiKetNoiWiFi("Failed to connect");
      }
    } else {
      _dwin.HienThiTrangThaiKetNoiWiFi("disconnect " + WifiSSID);
      Serial.println("Connecting to " + WifiSSID);
      _dwin.setVP(_VPAddressIonConnect, 0);
      _dwin.setVP(_VPAddressIconWiFi, 0);
      WiFiConfig.state = false;
      uint8_t u8Try = 15;
      while (WiFi.status() == WL_CONNECTED && u8Try--) {
        WiFi.disconnect();
        delay(1000);
      }
    }

    UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("%s: Task's stack high water mark: %lu words\n", __func__, stackHighWaterMark);
  }
}

// Task này dùng để xử lý trong quá trình tủ chạy như là chuyển segment của program,
// Tắt tủ khi hết thời
// Hiển thị Icon xự kiện USB
// Chạy các chế độ tiệt trùng, program, quick mode
// quyết định nội dụng hiển thị task HMI
void TaskMain(void*) {
  bool machineState = BaseProgram.machineState;
  bool FlagUSB = false;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  FrameDataQueue_t sendData = {
    .pvData = NULL,
  };

  uint16_t chuKyVeDoThi = 0;    // 10s
  uint16_t chuKyWarning = 0;    // 60s
  uint16_t chuKyRefresh = 0;    // 120s
  uint16_t chuKyRecord = 0;     // 60s
  uint16_t chuKyCheckWifi = 0;  // 60s

  while (1) {

    // *LỆNH CHO HMI
    // lệnh kiểm tra USB chu kỳ 1s 1 lần
    sendData.event = eEVENT_ICON_USB;
    xQueueSend(recvHMIQueue, &sendData, 10);
    // lệnh đọc cảm biến chu kỳ 1s 1 lần
    sendData.event = eEVENT_HIEN_THI_GIA_TRI_CAM_BIEN;
    xQueueSend(recvHMIQueue, &sendData, 10);
    // lệnh cho thời gian chu kỳ 1s 1 lần
    sendData.event = eEVENT_HIEN_THI_THOI_GIAN;
    xQueueSend(recvHMIQueue, &sendData, 10);
    // icon quạt
    sendData.event = eEVENT_ICON_FAN;
    xQueueSend(recvHMIQueue, &sendData, 10);
    // lệnh cho đồ thị chu kỳ riêng
    if (chuKyVeDoThi >= 10) {
      chuKyVeDoThi = 0;
      sendData.event = eEVENT_VE_DO_THI;
      xQueueSend(recvHMIQueue, &sendData, 10);
    }
    // lệnh cho đồ thị chu kỳ riêng
    if (chuKyCheckWifi >= 30) {
      chuKyCheckWifi = 0;
      sendData.event = eEVENT_ICON_WIFI;
      xQueueSend(recvHMIQueue, &sendData, 10);
    }
    // lệnh cho warning
    if (chuKyWarning >= 120) {
      chuKyWarning = 0;
      sendData.event = eEVENT_WARNING;
      xQueueSend(recvHMIQueue, &sendData, 10);
    }
    // lệnh cho refesh
    if (chuKyRefresh >= 600) {
      chuKyRefresh = 0;
      sendData.event = eEVENT_REFRESH;
      xQueueSend(recvHMIQueue, &sendData, 10);
    }
    // chu kỳ record
    if (chuKyRecord >= 60) {
      chuKyRecord = 0;
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
    chuKyRecord++;
    chuKyRefresh++;
    chuKyVeDoThi++;
    chuKyWarning++;
    chuKyCheckWifi++;

    // Phần xác định nhiệt độ đã xác lập hay chưa
    if (fabs(BaseProgram.temperature - BaseProgram.programData.setPointTemp) <= 0.15 && fabs(BaseProgram.CO2 - BaseProgram.programData.setPointCO2) <= 0.15) {
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

            _dwin.Buzzer(800);
          }
        }
      } else if (BaseProgram.delayOffState == false) {
        DemThoiGianChay(false, DEM_LEN);
      }
    }

    // Phần xử lý chuyển segment và tắt máy
    // if(RunMode == PROGRAM_MODE && SwitchSegment == true && BaseProgram.machineState == true)
    // if (RunMode == PROGRAM_MODE && SwitchSegment == true)
    if (SwitchSegment == true) {
      Serial.println("SwitchSegment");
      SwitchSegment = false;
      if (RunMode == PROGRAM_MODE) {
        if (CurrentProgram.size() > RunningSegmentIndex) {
          BaseProgram.programData = CurrentProgram.at(RunningSegmentIndex);
          // BaseProgram.machineState = true;
          BaseProgram.delayOffState = true;
          FlagNhietDoXacLap = false;

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
          } else {
            _dwin.HienThiIconOnOffDelayOff(true);
          }
          RunningSegmentIndex++;
        } else {
          programLoopCount++;
          if (programLoopCount < programLoop && programInf == false) {
            // BaseProgram.machineState = true;
            FlagNhietDoXacLap = false;
            SwitchSegment = true;
            RunningSegmentIndex = 0;
            _dwin.HienThiVongLapChuongTrinhConLai(programLoopCount + 1, programLoop);
            if (BaseProgram.delayOffState) {
              DemThoiGianChay(true, DEM_XUONG);
            } else {
              DemThoiGianChay(true, DEM_LEN);
            }
          } else if (programInf) {
            // BaseProgram.machineState = true;
            FlagNhietDoXacLap = false;
            SwitchSegment = true;
            RunningSegmentIndex = 0;
            if (BaseProgram.delayOffState) {
              DemThoiGianChay(true, DEM_XUONG);
            } else {
              DemThoiGianChay(true, DEM_LEN);
            }
          } else {
            // tắt máy
            BaseProgram.machineState = false;
            _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);
            _dwin.HienThiThoiGianChay("FINISH");

            _dwin.Buzzer(800);
          }
        }
      }
    }


    _Heater.CaiDatNhietDo(BaseProgram.programData.setPointTemp);
    _Heater.CaiTocDoQuat(BaseProgram.programData.fanSpeed);
    _Heater.CaiGiaTriOfset(GetCalib(BaseProgram.programData.setPointTemp));
    _CO2.CaiNongDoCO2(BaseProgram.programData.setPointCO2);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    // UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.printf("%s: Task's stack high water mark: %lu words\n", __func__, stackHighWaterMark);
    if (machineState != BaseProgram.machineState) {
      if (BaseProgram.machineState == true) {
        BatMay(__func__);
      } else if (BaseProgram.machineState == false) {
        TatMay(__func__);
      }
      machineState = BaseProgram.machineState;
    }
  }
}

void TaskHMI(void*) {
  FrameDataQueue_t data;
  while (1) {
    xQueueReceive(recvHMIQueue, &data, portMAX_DELAY);
    // Serial.printf("%s nhận event: %d, data nhận được %s\n", __func__, data.event, data.pvData == NULL ? "NULL" : "NOT NULL");

    switch ((EventTaskHMI_t)data.event) {
      case eEVENT_ICON_NHIET:
        if (_Heater.TrangThaiThanhGiaNhiet() == 1) {  // gia nhiet
          _dwin.HienThiIconGiaNhiet(1);
        } else if (_Heater.TrangThaiThanhGiaNhiet() == 0) {
          _dwin.HienThiIconGiaNhiet(0);
        }
        break;
      case eEVENT_ICON_CO2:
        if (_CO2.LayTrangThaiVan() == 1) {  // bat van khi
          _dwin.HienThiIconVanCO2(1);
        } else if (_CO2.LayTrangThaiVan() == 0) {
          _dwin.HienThiIconVanCO2(0);
        }
        break;
      case eEVENT_ICON_CUA:
        if (_Door.TrangThai() == DOOR_CLOSE) {
          _dwin.HienThiIconCua(0);
        } else if (_Door.TrangThai() == DOOR_OPEN) {
          _dwin.HienThiIconCua(1);
        }
        break;
      case eEVENT_ICON_FAN:
        if (_Heater.TrangThaiQuat() == 1) {
          _dwin.HienThiIconQuat(1);
        } else if (_Heater.TrangThaiQuat() == 0) {
          _dwin.HienThiIconQuat(0);
        }
        break;
      case eEVENT_ICON_USB:
        if (USB_MSC_HOST.isConnected()) {
          _dwin.HienThiIconUSB(1);
        } else {
          _dwin.HienThiIconUSB(0);
        }
        break;
      case eEVENT_ICON_WIFI:
        if (WiFi.status() == WL_CONNECTED) {
          _dwin.setVP(_VPAddressIconWiFi, map(constrain(WiFi.RSSI(), -100, -40), -100, -40, 1, 4));
        } else {
          _dwin.setVP(_VPAddressIconWiFi, 0);
        }
        break;
      case eEVENT_HIEN_THI_GIA_TRI_CAM_BIEN:  // cập nhật 1s
        BaseProgram.CO2 = _CO2.LayNongDoCO2Thuc();
        if (BaseProgram.CO2 >= -0.5f && BaseProgram.CO2 <= 30.0f) {
          _dwin.HienThiCO2(BaseProgram.CO2);
        } else {
          _dwin.HienThiCO2("err");
        }
        BaseProgram.temperature = _Heater.LayNhietDoLoc();
        if (BaseProgram.temperature > -10 && BaseProgram.temperature <= 350) {
          _dwin.HienThiNhietDo(BaseProgram.temperature);
        } else {
          _dwin.HienThiNhietDo("err");
        }
        break;
      case eEVENT_HIEN_THI_THOI_GIAN:  // cập nhật 1s
        RTCnow = _time.getCurrentTime();
        _dwin.HienThiThoiGianRTC(RTCnow.day(), RTCnow.month(), RTCnow.year() % 1000, RTCnow.hour(), RTCnow.minute(), RTCnow.second());
        break;
      case eEVENT_VE_DO_THI:  // cập nhật theo chu kỳ riêng (hiện tại 1s)
        _dwin.VeDoThi(BaseProgram);
        break;
      case eEVENT_WARNING:  // cập nhật chu kỳ hoặc có lỗi xuất hiện
        if (FlagNhietDoXacLap == true) {
          String warningText = "";
          if (BaseProgram.temperature >= BaseProgram.programData.setPointTemp + BaseProgram.programData.tempMax) {
            warningText += "Overheat ";
          } else if (BaseProgram.temperature <= BaseProgram.programData.setPointTemp + BaseProgram.programData.tempMin) {
            warningText += "Underheat ";
            _dwin.Buzzer(160);
          }

          if (BaseProgram.CO2 >= BaseProgram.programData.setPointCO2 + BaseProgram.programData.CO2Max) {
            warningText += "OverCO2 ";
          } else if (BaseProgram.CO2 <= BaseProgram.programData.setPointCO2 + BaseProgram.programData.CO2Min) {
            warningText += "UnderCO2 ";
          }

          if (warningText.length() > 2) {
            _dwin.HienThiWarning("Alarm: " + warningText, _HomePage);
            _dwin.Buzzer(160);
          }
        }
        {
          String warningText = "";
          if (BaseProgram.temperature > 350) {
            warningText += "Temprature ";
          }
          if (BaseProgram.CO2 > 30) {
            warningText += "CO2 ";
          }
          if (warningText.length() > 2) {
            _dwin.HienThiWarning("Err: " + warningText + "sensor", _HomePage);
            _dwin.Buzzer(160);
          }
        }
        break;
      case eEVENT_REFRESH:  // ghi lại màn hình sau 10p
        Serial.println(BaseProgram.programData.setPointTemp);
        _dwin.HienThiSetpointTemp(BaseProgram.programData.setPointTemp);
        _dwin.HienThiSetpointCO2(BaseProgram.programData.setPointCO2);
        _dwin.HienThiTocDoQuat(BaseProgram.programData.fanSpeed);
        _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);
        break;
      default:
        Serial.printf("%s nhận UNDEFINE event \n", __func__);
        break;
    }
  }
}
