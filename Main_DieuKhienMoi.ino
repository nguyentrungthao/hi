// Đường dẫn SDKconfig thay đổi độ ưu tiên task Timer lên 20
// C:\Users\minht\AppData\Local\Arduino15\packages\esp32\tools\esp32-arduino-libs\idf-release_v5.1-33fbade6\esp32s3\qio_qspi\include

// standard
#include <stdio.h>
#include <vector>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <type_traits>

// Arduino
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#define ARDUINOJSON_USE_LONG_LONG 1
#include <ArduinoJson.h>
#include <SD_MMC.h>
#include <SPI.h>
#include <TimeLib.h>

// ESP IDF
#include "esp_heap_caps.h"
#include <esp_task_wdt.h>
#include "freertos/FreeRTOSConfig.h"
#include "driver/temperature_sensor.h"

// user
#include "FileHandle.h"
#include "userdef.h"
#include "RTClib.h"
#include "HMI.h"
#include "HMIparam.h"
#include "src/usb_msc_host.h"

#include "09_concentration.h"
#include "07_Heater.h"
#include "Door.h"

#include "RTC_Timer.h"
#include "FifoBuffer.h"

RTC_Timer _time;

int clk = 5;
int cmd = 4;
int d0 = 6;
bool onebit = true;

// Thông tin Wi-Fi
const char *ssid = "LABone_Phong_Hoan_Thien";
const char *password = "66668888";

const uint8_t RxPin = 14; // Chân Rx1 trên ESP32
const uint8_t TxPin = 13; // Chân Tx1 trên ESP32

// Thêm điều kiện xài thư viện cũ
#define DGUS_SERIAL Serial2 // Cổng UART giao tiếp giữa MCU với HMI
#define DGUS_BAUD 115200    // Baundrate giao tiếp UART giữa MCU và HMI

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
TaskHandle_t TaskKetNoiWiFiHdl; // truc them
QueueHandle_t KetNoiWiFiQueue;  // truc them

String WifiSSID = "";
String WifiPassword = "";
WiFiConfig_t WiFiConfig;
bool bTrangThaiWifi = false;

uint8_t countTest = 0;
int8_t GiaTriThanhCuon = 0;
int8_t GiaTriThanhCuonTruocDo = 0;
bool TrangThaiThanhCuon = false;

// Queue handle -> POST thong so dieu khien ve server
QueueHandle_t dataQueue;

QueueHandle_t recvHMIQueue;
QueueHandle_t QueueUpdateFirmware;

const uint32_t pu32ArgTimerDWIN[][2] = {{eHMI_EVENT_HIEN_THI_GIA_TRI_CAM_BIEN, 1000},
                                        {eHMI_EVENT_HIEN_THI_THOI_GIAN, 1000},
                                        {eHMI_EVENT_ICON_USB, 1000},
                                        {eHMI_EVENT_ICON_FAN, 1000},
                                        {eHMI_EVENT_VE_DO_THI, 10000},
                                        {eHMI_EVENT_ICON_WIFI, 30000},
                                        {eHMI_EVENT_WARNING, 120000},
                                        {eHMI_EVENT_REFRESH, 120000}};
constexpr uint8_t u8NUMBER_OF_TIMER_DWIN = sizeof(pu32ArgTimerDWIN) / sizeof(pu32ArgTimerDWIN[0]);
TimerHandle_t *const pxTimerDWINhdl = (TimerHandle_t *)malloc(u8NUMBER_OF_TIMER_DWIN * sizeof(TimerHandle_t));
TimerHandle_t xTimerSleep;
TimerHandle_t xTimerPIDLog;

// các hàm khởi tạo và hỗ trợ khởi tạo
void khoiTaoDWIN();
void khoiTaoSDCARD();
void khoiTaoRTC();
void khoiTaoHeater();
void khoiTaoCO2();
void khoiTaoCua();
void listFilesInProgramDirectory(void);
void TaoCacThuMucHeThongTrenSD(void);
void KhoiTaoSoftTimerDinhThoi(void);

// các hàm chạy run time hoặc call back
void BatMay(const char *funcCall);
void TatMay(const char *funcCall);
// float GetCalib(float value);
bool DemThoiGianChay(bool reset, bool DemXuong);
void setTimeFromRTC(int year, int month, int day, int hour, int minute, int second);
void hmiSetEvent(const hmi_set_event_t &event);
bool hmiGetEvent(hmi_get_type_t event, void *args);
void callBackOpenDoor(void *ptr);
void callBackCloseDoor(void *ptr);
static void triggeONICONNhiet(void *);
static void triggeOFFICONNhiet(void *);
void callBackSoftTimerChoDWIN(TimerHandle_t xTimer);
void hienThiList(int i);
void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
void callBackSoftTimer(TimerHandle_t xTimer);

// các task
void TaskHMI(void *);
void TaskExportData(void *);
void TaskUpdateFirmware(void *);
void TaskKetNoiWiFi(void *);
void TaskMain(void *);
void TaskMonitorDebug(void *);
void TaskHardWareTesting(void *);

// đây là nhánh newCALIB
void setup()
{
  // esp_task_wdt_deinit();
  Serial.begin(115200);
  Wire.begin(10, 11);

  KetNoiWiFiQueue = xQueueCreate(2, sizeof(FrameDataQueue_t));
  QueueUpdateFirmware = xQueueCreate(2, sizeof(MethodUpdates_t));
  recvHMIQueue = xQueueCreate(20, sizeof(FrameDataQueue_t));

  khoiTaoRTC();
  delay(10);

  khoiTaoDWIN();
  delay(10);

  khoiTaoCua();
  delay(10);

  khoiTaoSDCARD();
  delay(10);

  khoiTaoHeater();
  delay(10);

  khoiTaoCO2();
  delay(10);

  delay(2000);
  // _dwin.echoEnabled(true);
  RunMode = QUICK_MODE;
  BaseProgram.machineState = false;
  BaseProgram.delayOffState = false;
  Serial.println(BaseProgram.programData.setPointTemp);
  _dwin.HienThiSetpointTemp(BaseProgram.programData.setPointTemp);
  _dwin.HienThiSetpointCO2(BaseProgram.programData.setPointCO2);
  _dwin.HienThiTocDoQuat(BaseProgram.programData.fanSpeed);
  _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);
  _dwin.HienThiIconGiaNhiet(false);
  _dwin.HienThiChuongTrinhDangChay("Quick");
  _dwin.HienThiThoiGianChay("Inf");
  _dwin.HienThiSegmentDangChay("");
  _dwin.HienThiVongLapChuongTrinhConLai("");
  _dwin.HienThiIconSegment(false);
  _dwin.setBrightness(50);
  _dwin.XoaDoThi(BaseProgram);

  _dwin.HienThiThongTinVersion(__TIME__);
  _dwin.setPage(_HomePage);

  xTaskCreateUniversal(TaskExportData, "tskExport", 8192, NULL, 5, &TaskExportDataHdl, -1);
  delay(5);
  xTaskCreateUniversal(TaskUpdateFirmware, "tskFirmware", 8192, NULL, 4, &TaskUpdateFirmwareHdl, -1);
  delay(5);
  xTaskCreateUniversal(TaskKetNoiWiFi, "tskWifi", 4096, NULL, 1, &TaskKetNoiWiFiHdl, -1); // truc them
  delay(5);
  xTaskCreateUniversal(TaskMain, "tskMain", 8192, NULL, 3, &TaskMainHdl, -1);
  delay(5);
  xTaskCreateUniversal(TaskHMI, "tskHMI", 8192, NULL, 5, &TaskHMIHdl, -1);
  delay(5);
  // xTaskCreateUniversal(TaskMonitorDebug, "Debug", 4096, NULL, 5, NULL, -1);
  delay(5);

  KhoiTaoSoftTimerDinhThoi();
}

void loop()
{
  loop_PostGet();
  delay(1);
}

void khoiTaoRTC()
{
  _time.begin();
  _time.getCurrentTime(); // Gọi hàm này trước khi get thời gian
  struct tm tmstruct;
  setTimeFromRTC(_time.getYear(), _time.getMonth(), _time.getDay(), _time.getHour(), _time.getMinute(), _time.getSecond());
  tmstruct.tm_year = 0;
  getLocalTime(&tmstruct, 5000);
  Serial.printf(
      "\nNow is : %d-%02d-%02d %02d:%02d:%02d\n", (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min,
      tmstruct.tm_sec);
  Serial.println("");
}
void khoiTaoDWIN()
{
  _dwin.KhoiTao();
  _dwin.CRCEnabled();
  _dwin.DangKyHamSetCallback(hmiSetEvent);
  _dwin.DangKyHamGetCallback(hmiGetEvent);
  FrameDataQueue_t data;
  data.event = eHMI_EVENT_VE_DO_THI;
  xQueueSend(recvHMIQueue, &data, 0);
}
void khoiTaoSDCARD()
{
  // SDMMCFile.begin();
  SD_MMC.setPins(clk, cmd, d0);
  if (!SD_MMC.begin("/sdcard", onebit, true, 4000000))
  {
    Serial.println("Mount Failed");
  }
  else
  {
    Serial.println("SD_MMC initialized successfully");
    checkAndResumeUpdates(SD_MMC);
    SDMMCFile.setFileSystem(SD_MMC);
  }
  TaoCacThuMucHeThongTrenSD();
  listFilesInProgramDirectory();

  UsbFile.setFileSystem(USB_MSC_HOST);
  USB_MSC_HOST.begin();
  if (USB_MSC_HOST.isConnected())
  {
    checkAndResumeUpdates(USB_MSC_HOST);
  }

  // Khôi phục thông số hệ thống
  // Khôi phục danh sách hệ số calib.
  if (SDMMCFile.exists(PATH_CALIB_DATA))
  {
    DanhSachHeSoCalib.resize(SDMMCFile.sizeFile(PATH_CALIB_DATA) / sizeof(CalibData_t));
    if (SDMMCFile.readFile(PATH_CALIB_DATA, (uint8_t *)DanhSachHeSoCalib.data(), SDMMCFile.sizeFile(PATH_CALIB_DATA)))
    {
      for (CalibData_t HeSoCalib : DanhSachHeSoCalib)
      {
        Serial.printf("Setpoint: %.1f, value: %.2f\n", HeSoCalib.Setpoint, HeSoCalib.value);
      }
    }
  }

  if (SDMMCFile.exists(PATH_CALIB_DATA))
  {
  }

  // Khôi phục thông số chương trình cơ sở.
  if (SDMMCFile.exists(PATH_BASEPROGRAM_DATA))
  {
    SDMMCFile.readFile(PATH_BASEPROGRAM_DATA, (uint8_t *)&BaseProgram, sizeof(BaseProgram));
  }
  else
  {
    BaseProgram.programData.setPointTemp = 37;
    BaseProgram.programData.setPointCO2 = 5;
    BaseProgram.programData.fanSpeed = 50;
    BaseProgram.programData.delayOffDay = 0;
    BaseProgram.programData.delayOffHour = 1;
    BaseProgram.programData.delayOffMinute = 0;
    BaseProgram.programData.tempMax = 0.3;
    BaseProgram.programData.tempMin = -0.3;
    BaseProgram.programData.CO2Max = 0.3;
    BaseProgram.programData.CO2Min = -0.3;
    BaseProgram.machineState = false;
    BaseProgram.delayOffState = false;
    SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t *)&BaseProgram, sizeof(BaseProgram));
  }

  if (SDMMCFile.exists(PATH_RECORD))
  {
    RecordData_t data;
    int filesize = SDMMCFile.sizeFile(PATH_RECORD);
    for (int i = 0; i < filesize / sizeof(data); i++)
    {
      SDMMCFile.readFile(PATH_RECORD, (uint8_t *)&data, sizeof(data), i * sizeof(data));
      // Serial.printf("SP: %.1f, Value: %.1f, Fan: %u, Flap: %u\n", data.setpoint, data.value, data.fan, data.flap);
      Serial.printf("TempSP: %.1f, TempValue: %.1f, CO2SP: %0.1f, CO2Value: %0.1f, Fan: %u\n", data.setpointTemp, data.valueTemp, data.fan);
    }
  }

  if (!SDMMCFile.exists("/usbmode.txt"))
  {
    uint8_t mode = 0;
    SDMMCFile.writeFile("/usbmode.txt", (uint8_t *)&mode, sizeof(mode));
  }
  else
  {
    uint8_t mode;
    SDMMCFile.readFile("/usbmode.txt", (uint8_t *)&mode, sizeof(mode));
    if (mode == 0)
    {
    }
    else if (mode == 1)
    {
      USB_MSC_HOST.end();
      delay(1000);
      sd2usbmsc_init();
    }
  }

  if (SDMMCFile.exists(ADMIN_PASSWORD))
  {
    Serial.print("Khoi phuc mat khau admin: ");
    String admin_password = SDMMCFile.readFile(ADMIN_PASSWORD);
    _dwin.ThayDoiUserAdminPassword(admin_password);
    Serial.println(admin_password);
  }

  // Test log
  initLogFileState(SD_MMC);

  if (SDMMCFile.exists(WIFICONFIG_PATH))
  {
    SDMMCFile.readFile(WIFICONFIG_PATH, (uint8_t *)&WiFiConfig, sizeof(WiFiConfig));
    WifiSSID = String(WiFiConfig.ssid);
    WifiPassword = String(WiFiConfig.password);
    _dwin.HienThiSSIDWiFi(WifiSSID);
    _dwin.HienThiPasswordWiFi(WifiPassword);
    Serial.print("Thong tin Wifi: ");
    Serial.print(WifiSSID + " ");
    Serial.println(WifiPassword);
    if (WiFiConfig.state && KetNoiWiFiQueue)
    {
      FrameDataQueue_t data = {
          .event = eEVENT_CONNECT_WIFI,
      };
      xQueueSend(KetNoiWiFiQueue, &data, 10);
    }
  }
}
void khoiTaoHeater()
{
  // _Heater.CaiGiaTriOfset(GetCalib(BaseProgram.programData.setPointTemp));
  _Heater.KhoiTao();
  BaseProgram.temperature = _Heater.LayNhietDoLoc();
  _Heater.addCallBackWritePinTriacBuong(triggeONICONNhiet, NULL); // trigger on ICON nhiệt
  _Heater.addCallBackTimeOutTriacBuong(triggeOFFICONNhiet, NULL); // trigger off ICON nhiệt
  if (recvHMIQueue == NULL)
  {
    return;
  }
  static FrameDataQueue_t data;
  data.event = eHMI_EVENT_ICON_NHIET;
  xQueueSend(recvHMIQueue, &data, 10);
  data.event = eHMI_EVENT_ICON_FAN;
  xQueueSend(recvHMIQueue, &data, 10);
  data.event = eHMI_EVENT_WARNING;
  xQueueSend(recvHMIQueue, &data, 10);
}
void khoiTaoCO2()
{
  _CO2.KhoiTaoCO2();
  BaseProgram.CO2 = _CO2.LayNongDoCO2Thuc();
  _CO2.addCallBackWritePin(triggeONICONCO2, NULL); // trigger on ICON nhiệt
  _CO2.addCallBackTimeout(triggeOFFICONCO2, NULL); // trigger off ICON nhiệt
  if (recvHMIQueue == NULL)
  {
    return;
  }
  static FrameDataQueue_t data;
  data.event = eHMI_EVENT_WARNING;
  xQueueSend(recvHMIQueue, &data, 10);
  data.event = eHMI_EVENT_ICON_CO2;
  xQueueSend(recvHMIQueue, &data, 10);
}
void khoiTaoCua()
{
  _Door.KhoiTao();
  _Door.addCloseFuncCallBack(callBackCloseDoor, NULL);
  _Door.addOpenFuncCallBack(callBackOpenDoor, NULL);
  if (recvHMIQueue == NULL)
  {
    return;
  }
  static FrameDataQueue_t data;
  data.event = eHMI_EVENT_ICON_CUA;
  xQueueSend(recvHMIQueue, &data, 0);
}
void KhoiTaoSoftTimerDinhThoi()
{

  for (uint8_t i = 0; i < u8NUMBER_OF_TIMER_DWIN; i++)
  {
    Serial.printf("create timer event HMI %lu period %lu\n", pu32ArgTimerDWIN[i][0], pu32ArgTimerDWIN[i][1]);

    String nameTimer = "timerDwin" + String(pu32ArgTimerDWIN[i][0]);

    pxTimerDWINhdl[i] = xTimerCreate(nameTimer.c_str(),
                                     pdMS_TO_TICKS(pu32ArgTimerDWIN[i][1]),
                                     pdTRUE,
                                     (void *)&(pu32ArgTimerDWIN[i][0]),
                                     callBackSoftTimerChoDWIN);
    if (pxTimerDWINhdl[i] == NULL)
    {
      Serial.printf("hết bộ nhớ để tạo timer => reset\n");
      abort();
    }
  }

  for (uint8_t i = 0; i < u8NUMBER_OF_TIMER_DWIN; i++)
  {
    xTimerStart(pxTimerDWINhdl[i], 1000);
  }

  xTimerSleep = xTimerCreate("SleepTimer", pdMS_TO_TICKS(900000), pdTRUE, (void *)eHMI_EVENT_TIMEROUT_OFF, callBackSoftTimer);
  if (xTimerSleep == NULL)
  {
    Serial.printf("hết bộ nhớ để tạo timer => reset\n");
    abort();
  }
  xTimerStart(xTimerSleep, 1000);

  xTimerPIDLog = xTimerCreate("PIDTimer", pdMS_TO_TICKS(1000), pdTRUE, (void *)eHMI_EVENT_HIEN_THI_PID, callBackSoftTimer);
  if (xTimerPIDLog == NULL)
  {
    Serial.printf("hết bộ nhớ để tạo timer => reset\n");
    abort();
  }
}
void callBackSoftTimerChoDWIN(TimerHandle_t xTimer)
{
  configASSERT(xTimer);
  FrameDataQueue_t dataFrameForDWIN;
  dataFrameForDWIN.event = *((uint32_t *)pvTimerGetTimerID(xTimer));
  if (recvHMIQueue == NULL)
    return;
  xQueueSend(recvHMIQueue, &dataFrameForDWIN, 10);
}
void callBackSoftTimer(TimerHandle_t xTimer)
{
  FrameDataQueue_t data;
  data.event = (int32_t)pvTimerGetTimerID(xTimer);
  if (recvHMIQueue)
  {
    xQueueSend(recvHMIQueue, &data, 0);
  }
}

void callBackOpenDoor(void *ptr)
{
  //! check ptr trước khi dùng nhá
  Serial.printf("Open door\n");
  static FrameDataQueue_t data;
  data.event = eHMI_EVENT_ICON_CUA;
  FlagNhietDoXacLap = false;
  if (recvHMIQueue == NULL)
  {
    return;
  }
  xQueueSend(recvHMIQueue, &data, 0);

  if (BaseProgram.machineState == false)
  {
    return;
  }
  _Heater.SetEventDOOR();
  _CO2.SetEventDOOR();
}
void callBackCloseDoor(void *ptr)
{
  //! check ptr trước khi dùng nhá
  Serial.printf("Close door\n");
  static FrameDataQueue_t data;
  data.event = eHMI_EVENT_ICON_CUA;
  if (recvHMIQueue == NULL)
  {
    return;
  }
  xQueueSend(recvHMIQueue, &data, 0);

  if (BaseProgram.machineState == false)
  {
    return;
  }
  _Heater.ResetEventDOOR();
  _CO2.ResetEventDOOR();
}

void BatMay(const char *funcCall)
{
  static FrameDataQueue_t data;
  Serial.printf("\t\t\tBat may: call from %s\n", funcCall ? funcCall : "NULL");
  _Heater.BatDieuKhienNhietDo();
  _CO2.BatDieuKhienCO2();
  // SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
  if (_Door.TrangThai() == DOOR_OPEN)
  {
    _CO2.SetEventDOOR();
    _Heater.SetEventDOOR();
  }

  data.event = eHMI_EVENT_WARNING;
  if (recvHMIQueue == NULL)
  {
    Serial.printf("\t\t\tQueue recvHMIQueue is NULL => Return\n");
    return;
  }
  xQueueSend(recvHMIQueue, &data, 100);
}
void TatMay(const char *funcCall)
{
  Serial.printf("\t\t\tTat may: call from %s\n", funcCall ? funcCall : "NULL");
  _Heater.TatDieuKhienNhietDo();
  _CO2.TatDieuKhienCO2();
  // SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t*)&BaseProgram, sizeof(BaseProgram));
}

// float GetCalib(float value)
// {
//   for (CalibData_t HeSoCalib : DanhSachHeSoCalib)
//   {
//     if (fabs(HeSoCalib.Setpoint - value) < 0.02)
//     {
//       // Serial.printf("Setpoint: %.1f, value: %.2f\n", HeSoCalib.Setpoint, HeSoCalib.value);
//       return HeSoCalib.value;
//     }
//   }
//   return 0;
// }

bool DemThoiGianChay(bool reset, bool DemXuong)
{
  static time_t timeRun;
  // Serial.println("Gửi thời gian đếm");
  if (reset)
  {
    _time.reset();
    if (DemXuong)
    {
      _time.start(BaseProgram.programData.delayOffDay * 86400 + BaseProgram.programData.delayOffHour * 3600 + BaseProgram.programData.delayOffMinute * 60, true);
      _dwin.HienThiThoiGianChay(BaseProgram.programData.delayOffDay, BaseProgram.programData.delayOffHour, BaseProgram.programData.delayOffMinute, 0);
      _dwin.HienThiIconOnOffDelayOff(true);
    }
    else
    {
      _time.start(BaseProgram.programData.delayOffDay * 86400 + BaseProgram.programData.delayOffHour * 3600 + BaseProgram.programData.delayOffMinute * 60);
      _dwin.HienThiIconOnOffDelayOff(false);
    }
  }
  else
  {
    if (DemXuong)
    {
      timeRun = _time.getRemainingTime(); // Lấy thời gian còn lại
    }
    else
    {
      timeRun = _time.getElapsedTime(); // Lấy thời gian đang đếm
    }
    _dwin.HienThiThoiGianChay((int)timeRun / 86400, (int)timeRun % 86400 / 3600, (int)timeRun % 86400 % 3600 / 60, (int)timeRun % 86400 % 3600 % 60);
  }
  if (_time.isFinished())
  {
    return false;
  }
  return true;
}

void setTimeFromRTC(int year, int month, int day, int hour, int minute, int second)
{
  struct tm timeinfo;
  timeinfo.tm_year = year - 1900; // Năm tính từ 1900
  timeinfo.tm_mon = month - 1;    // Tháng bắt đầu từ 0
  timeinfo.tm_mday = day;
  timeinfo.tm_hour = hour;
  timeinfo.tm_min = minute;
  timeinfo.tm_sec = second;

  time_t t = mktime(&timeinfo); // Chuyển đổi struct tm thành time_t
  struct timeval now_tv = {.tv_sec = t};
  settimeofday(&now_tv, NULL); // Đặt thời gian hệ thống ESP32
}
// Liệt kê và in ra tên các tệp tin trong thư mục "program"
void listFilesInProgramDirectory()
{
  File root = SD_MMC.open("/program");
  if (!root)
  {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory())
  {
    Serial.println("/program is not a directory");
    return;
  }
  time_t thoigianluufile;
  File file = root.openNextFile();
  while (file)
  {
    Serial.print("File: ");
    Serial.print(file.name());
    thoigianluufile = file.getLastWrite();
    Serial.printf(" %u/%u/%u-%02u:%02u:%02u\n", day(thoigianluufile), month(thoigianluufile), year(thoigianluufile),
                  hour(thoigianluufile), minute(thoigianluufile), second(thoigianluufile));
    file.close();
    file = root.openNextFile();
  }
}

void vXuLyCalibCamBienVaCuaVanh(const hmi_set_event_t &event)
{
  ParameterSaveInSDCard_t xParameterSaveInSDCard;
  xParameterSaveInSDCard.xCalibTemp = _Heater.xGetCalibParamater();
  xParameterSaveInSDCard.xCalibCO2 = _CO2.xGetCalibParamater();
  xParameterSaveInSDCard.i16Perimeter = _Heater.LayGiaTriDieuKhienVanh();
  xParameterSaveInSDCard.i16Door = _Heater.LayGiaTriDieuKhienCua();
  switch (event.type)
  {
  case HMI_SET_CALIB_NHIET:

    break;
  case eHMI_SET_PERIMETER:
    xParameterSaveInSDCard.i16Perimeter = event.f_value;
    break;
  case eHMI_SET_DOOR:
    xParameterSaveInSDCard.i16Door = event.f_value;
    break;
  case HMI_RESET_CALIB_NHIET:
    memset((uint8_t *)&xParameterSaveInSDCard.xCalibTemp, 0, sizeof(xParameterSaveInSDCard.xCalibTemp));
    break;
  case HMI_SET_CALIB_CO2:
    
  break;
  case HMI_RESET_CALIB_CO2:
    memset((uint8_t *)&xParameterSaveInSDCard.xCalibCO2, 0, sizeof(xParameterSaveInSDCard.xCalibCO2));
    break;
  }


  SDMMCFile.writeFile(PATH_CALIB_DATA, (uint8_t *)xParameterSaveInSDCard, sizeof(ParameterSaveInSDCard_t));
}

void hmiSetEvent(const hmi_set_event_t &event)
{
  char filePath[30];
  static int index = 0;
  static int ngayRTC, thangRTC, namRTC, gioRTC, phutRTC;
  static String currentProgramName;
  Program_t newProgram;
  static int8_t XacNhanTietTrung = 0;
  static int32_t ThoiGian2LanChamThanhCuon = millis();

  Serial.printf("%s call, event %d\n", __func__, event.type);
  xTimerReset(xTimerSleep, 10);

  switch (event.type)
  {
  case HMI_SET_RUN_ONOFF:
    if (BaseProgram.machineState == false)
    {
      BaseProgram.machineState = true;
      FlagNhietDoXacLap = false;
      SwitchSegment = true;
      RunningSegmentIndex = 0;
      programLoopCount = 0;
      if (RunMode == PROGRAM_MODE)
      {
        _dwin.HienThiVongLapChuongTrinhConLai(programLoopCount + 1, programLoop);
      }
      else
      {
        _dwin.HienThiVongLapChuongTrinhConLai("");
      }
      if (BaseProgram.delayOffState)
      {
        DemThoiGianChay(true, DEM_XUONG);
        // _time.reset();
      }
      else
      {
        DemThoiGianChay(true, DEM_LEN);
        // _time.reset();
      }
      _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);
    }
    else
    {
      BaseProgram.machineState = false;
      if (BaseProgram.delayOffState)
      {
        _dwin.HienThiThoiGianChay(BaseProgram.programData.delayOffDay, BaseProgram.programData.delayOffHour, BaseProgram.programData.delayOffMinute, 0);
      }
      else
      {
        _dwin.HienThiThoiGianChay("Inf");
      }
      _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);
    }
    Serial.printf("HMI_SET_RUN_ONOFF\n");
    break;
  case HMI_SET_SETPOINT_TEMP:
    Serial.printf("Set setpoint temp: %.1f\n", event.f_value);
    BaseProgram.programData.setPointTemp = event.f_value;
    if (RunMode == QUICK_MODE)
    {
      SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t *)&BaseProgram, sizeof(BaseProgram));
    }
    FlagNhietDoXacLap = false;
    break;
  case HMI_SET_SETPOINT_CO2:
    Serial.printf("Set setpoint CO2: %.1f\n", event.f_value);
    BaseProgram.programData.setPointCO2 = event.f_value;
    if (RunMode == QUICK_MODE)
    {
      SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t *)&BaseProgram, sizeof(BaseProgram));
    }
    FlagNhietDoXacLap = false;

    break;
  case HMI_SET_FAN:
    Serial.printf("Set fanspeed: %.1f\n", event.f_value);
    BaseProgram.programData.fanSpeed = (int8_t)event.f_value;
    if (RunMode == QUICK_MODE)
    {
      SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t *)&BaseProgram, sizeof(BaseProgram));
    }

    break;
  case HMI_SET_CALIB_NHIET:
  case eHMI_SET_PERIMETER:
  case eHMI_SET_DOOR:
  case HMI_RESET_CALIB_NHIET:
  case HMI_SET_CALIB_CO2:
  case HMI_RESET_CALIB_CO2:
    vXuLyCalibCamBienVaCuaVanh(event);
    break;
  case HMI_SET_DELAYOFF:
    if (RunMode == STERILIZATION_MODE)
    {
      Serial.println("HMI_SET_DELAYOFF, Fail -> STERILIZATION_MODE");
      return;
    }
    Serial.printf("Set time delayoff: %d\n", event.u32_value);
    BaseProgram.programData.delayOffDay = event.u32_value / 86400;
    BaseProgram.programData.delayOffHour = event.u32_value % 86400 / 3600;
    BaseProgram.programData.delayOffMinute = event.u32_value % 86400 % 3600 / 60;
    if (RunMode == QUICK_MODE)
    {
      SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t *)&BaseProgram, sizeof(BaseProgram));
    }
    Serial.printf("DelayOff Day: %d\n", BaseProgram.programData.delayOffDay);
    Serial.printf("DelayOff Hour: %d\n", BaseProgram.programData.delayOffHour);
    Serial.printf("DelayOff Minute: %d\n", BaseProgram.programData.delayOffMinute);
    if (BaseProgram.delayOffState)
    {
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
    if (RunMode == QUICK_MODE)
    {
      SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t *)&BaseProgram, sizeof(BaseProgram));
    }
    break;
  case HMI_SET_ALARM_TEMP_ABOVE:
    Serial.printf("HMI_SET_ALARM_TEMP_ABOVE: %.1f\n", event.f_value);
    BaseProgram.programData.tempMax = event.f_value;
    if (RunMode == QUICK_MODE)
    {
      SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t *)&BaseProgram, sizeof(BaseProgram));
    }
    break;
  case HMI_SET_ALARM_CO2_BELOW:
    Serial.printf("HMI_SET_ALARM_CO2_BELOW: %.1f\n", event.f_value);
    BaseProgram.programData.CO2Min = event.f_value;
    if (RunMode == QUICK_MODE)
    {
      SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t *)&BaseProgram, sizeof(BaseProgram));
    }
    break;
  case HMI_SET_ALARM_CO2_ABOVE:
    Serial.printf("HMI_SET_ALARM_CO2_ABOVE: %.1f\n", event.f_value);
    BaseProgram.programData.CO2Max = event.f_value;
    if (RunMode == QUICK_MODE)
    {
      SDMMCFile.writeFile(PATH_BASEPROGRAM_DATA, (uint8_t *)&BaseProgram, sizeof(BaseProgram));
    }
    break;
  case HMI_ADD_PROGRAM:
    Serial.println("HMI_ADD_PROGRAM");
    if (SDMMCFile.countFiles("/program") >= 20)
    {
      _dwin.HienThiWarning("Max: 20 programs", _ProgramPage);
      break;
    }
    sprintf(filePath, "/program/%s", event.text.c_str());
    if (SDMMCFile.exists(filePath))
    {
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
    SDMMCFile.writeFile((const char *)filePath, (uint8_t *)ProgramList.data(), ProgramList.size() * sizeof(Program_t));
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
    if (ProgramList.size() > event.indexList + listPageStartPosition)
    {
      ProgramList.at(event.indexList + listPageStartPosition).setPointTemp = event.f_value;
      Serial.printf("Set segment setpoint %u: %.1f\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].setPointTemp);
    }
    break;
  case HMI_EDIT_SEG_SETPOINT_CO2:
    if (ProgramList.size() > event.indexList + listPageStartPosition)
    {
      ProgramList.at(event.indexList + listPageStartPosition).setPointCO2 = event.f_value;
      Serial.printf("Set segment setpoint CO2 %u: %.1f\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].setPointCO2);
    }
    break;
  case HMI_EDIT_SEG_FANSPEED:
    if (ProgramList.size() > event.indexList + listPageStartPosition)
    {
      ProgramList.at(event.indexList + listPageStartPosition).fanSpeed = event.f_value;
      Serial.printf("Set segment fanspeed %u: %u\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].fanSpeed);
    }
    break;
  case HMI_EDIT_SEG_TEMPMIN:
    if (ProgramList.size() > event.indexList + listPageStartPosition)
    {
      ProgramList.at(event.indexList + listPageStartPosition).tempMin = event.f_value;
      Serial.printf("Set segment temp pro min %u: %.1f\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].tempMin);
    }
    break;
  case HMI_EDIT_SEG_TEMPMAX:
    if (ProgramList.size() > event.indexList + listPageStartPosition)
    {
      ProgramList.at(event.indexList + listPageStartPosition).tempMax = event.f_value;
      Serial.printf("Set segment temp pro max %u: %.1f\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].tempMax);
    }
    break;
  case HMI_EDIT_SEG_CO2MIN:
    if (ProgramList.size() > event.indexList + listPageStartPosition)
    {
      ProgramList.at(event.indexList + listPageStartPosition).CO2Min = event.f_value;
      Serial.printf("Set segment CO2 pro min %u: %.1f\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].CO2Min);
    }
    break;
  case HMI_EDIT_SEG_CO2MAX:
    if (ProgramList.size() > event.indexList + listPageStartPosition)
    {
      ProgramList.at(event.indexList + listPageStartPosition).CO2Max = event.f_value;
      Serial.printf("Set segment CO2 pro max %u: %.1f\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].CO2Max);
    }
    break;
  case HMI_ADD_SEG:
    Serial.println("HMI_ADD_SEG");
    if (ProgramList.size() >= 30)
    {
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
    if (ProgramList.size() < (int)event.indexList + listPageStartPosition)
    {
      ProgramList.push_back(newProgram);
      if (ProgramList.size() >= 5)
      {
        listPageStartPosition = ProgramList.size() - 5;
      }
    }
    else
    {
      ProgramList.insert(ProgramList.begin() + (int)event.indexList + listPageStartPosition, newProgram);
    }
    break;
  case HMI_SUB_SEG:
    Serial.println("HMI_SUB_SEG");
    if (ProgramList.size() > 1)
    {
      if ((int)event.indexList < 5 && ProgramList.size() > (int)event.indexList + listPageStartPosition)
      {
        ProgramList.erase(ProgramList.begin() + (int)event.indexList + listPageStartPosition);
      }
      else
      {
        ProgramList.erase(ProgramList.end() - 1);
      }
      // ProgramList.shrink_to_fit();
    }
    break;
  case HMI_EDIT_SEG_DELAYOFF_DAY:
    Serial.println("HMI_EDIT_SEG_DELAYOFF_DAY");
    if (ProgramList.size() > event.indexList + listPageStartPosition)
    {
      ProgramList.at(event.indexList + listPageStartPosition).delayOffDay = event.u32_value;
      Serial.printf("Delay off Day%u: %d\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].delayOffDay);
    }
    break;
  case HMI_EDIT_SEG_DELAYOFF_HOUR:
    Serial.println("HMI_EDIT_SEG_DELAYOFF_HOUR");
    if (ProgramList.size() > event.indexList + listPageStartPosition)
    {
      ProgramList.at(event.indexList + listPageStartPosition).delayOffHour = event.u32_value;
      Serial.printf("Delay off Hour%u: %d\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].delayOffHour);
    }
    break;
  case HMI_EDIT_SEG_DELAYOFF_MINUTE:
    Serial.println("HMI_EDIT_SEG_DELAYOFF_MINUTE");
    if (ProgramList.size() > event.indexList + listPageStartPosition)
    {
      ProgramList.at(event.indexList + listPageStartPosition).delayOffMinute = event.u32_value;
      Serial.printf("Delay off Minute%u: %d\n", event.indexList + listPageStartPosition, ProgramList[event.indexList + listPageStartPosition].delayOffMinute);
    }
    break;
  case HMI_SAVE_SEG:
    sprintf(filePath, "/program/%s", currentProgramName);
    SDMMCFile.writeFile((const char *)filePath, (uint8_t *)ProgramList.data(), ProgramList.size() * sizeof(Program_t));
    Serial.printf("Store %s\n", filePath);
    if (currentProgramName != event.text)
    {
      char *newName = new char[30];
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
    SDMMCFile.readFile((const char *)filePath, (uint8_t *)ProgramList.data(), SDMMCFile.sizeFile(filePath));
    _dwin.HienThiTenProgramDangEdit((String &)event.text);
    currentProgramName = event.text;
    Serial.printf("Restore %s\n", filePath);
    Serial.printf("list size: %d\n", ProgramList.size());
    break;
  case HMI_SET_DELAYOFF_ONOFF:
    Serial.println("HMI_SET_DELAYOFF_ON");
    if (RunMode != STERILIZATION_MODE)
    {
      if (BaseProgram.delayOffState == false)
      {
        BaseProgram.delayOffState = true;
        DemThoiGianChay(true, DEM_XUONG);
      }
      else
      {
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
    if (SDMMCFile.sizeFile(filePath) / sizeof(Program_t) == 0)
    {
      _dwin.HienThiWarning(event.text + ": 0 segment", _ProgramPage);
      break;
    }
    CurrentProgram.resize(SDMMCFile.sizeFile(filePath) / sizeof(Program_t));
    SDMMCFile.readFile((const char *)filePath, (uint8_t *)CurrentProgram.data(), SDMMCFile.sizeFile(filePath));
    if (CurrentProgram.size() > 0)
    {
      RunMode = PROGRAM_MODE;
      SwitchSegment = true;
      RunningSegmentIndex = 0;
      _dwin.HienThiChuongTrinhDangChay(event.text);
      _dwin.HienThiIconSegment(false);
      _dwin.HienThiSegmentDangChay("");
      _dwin.setPage(_HomePage);
      if ((uint8_t)event.f_value > 0)
      {
        programInf = false;
        programLoop = (uint8_t)event.f_value;
        _dwin.HienThiVongLapChuongTrinhConLai(0, programLoop);
      }
      else
      {
        programInf = true;
        _dwin.HienThiVongLapChuongTrinhConLai("Inf");
      }
      Serial.println("HMI_RUN_PROGRAM_MODE");
    }
    break;
  case HMI_RUN_QUICK_MODE:
    RunMode = QUICK_MODE;
    SDMMCFile.readFile(PATH_BASEPROGRAM_DATA, (uint8_t *)&BaseProgram, sizeof(BaseProgram));
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
    if (XacNhanTietTrung)
    {
      Serial.printf("tiet trung temp: %.1f\n", (float)event.f_value);
      BaseProgram.programData.setPointTemp = (float)event.f_value;
      BaseProgram.programData.fanSpeed = 100;
      BaseProgram.programData.setPointCO2 = 0;
      _dwin.HienThiSetpointTemp(BaseProgram.programData.setPointTemp);
      _dwin.HienThiTocDoQuat(BaseProgram.programData.fanSpeed);
      _dwin.HienThiSetpointCO2(BaseProgram.programData.setPointCO2);
    }
    break;
  case HMI_SET_STER_TIME:
    if (XacNhanTietTrung)
    {
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
    if (TaskExportDataHdl != NULL)
    {
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
    if (QueueUpdateFirmware != NULL)
    {
      MethodUpdates_t method = MAIN_UPDATE_USB;
      xQueueSend(QueueUpdateFirmware, &method, (TickType_t)0);
      if (TaskUpdateFirmwareHdl == NULL)
      {
        if (esp_get_free_heap_size() > 60000 && USB_MSC_HOST.isConnected())
        {
          xTaskCreateUniversal(TaskUpdateFirmware, "Task update firmware", 8192, NULL, 12, &TaskUpdateFirmwareHdl, -1);
        }
      }
    }
    break;
  case HMI_FIRMWARE_FOTA:
    Serial.println("->>> FOTA");
    if (QueueUpdateFirmware != NULL)
    {
      MethodUpdates_t method = MAIN_UPDATE_FOTA;
      xQueueSend(QueueUpdateFirmware, &method, (TickType_t)0);
      if (TaskUpdateFirmwareHdl == NULL)
      {
        if (esp_get_free_heap_size() > 60000)
        {
          xTaskCreateUniversal(TaskUpdateFirmware, "Task update firmware", 8192, NULL, 12, &TaskUpdateFirmwareHdl, -1);
        }
      }
    }
    break;
  case HMI_SET_SCROLLCHART:
    countTest++;
    Serial.println("Scroll: " + String((int)event.f_value));
    GiaTriThanhCuon = (int)event.f_value;
    if (millis() - ThoiGian2LanChamThanhCuon > 200)
    {
      GiaTriThanhCuonTruocDo = GiaTriThanhCuon;
      TrangThaiThanhCuon = false;
    }
    else if (GiaTriThanhCuonTruocDo != GiaTriThanhCuon)
    {
      TrangThaiThanhCuon = true;
    }
    ThoiGian2LanChamThanhCuon = millis();
    break;
  case HMI_CONNECT_OR_DISCONNECT_WIFI:
  {
    Serial.println("HMI_CONNECT_OR_DISCONNECT_WIFI");
    FrameDataQueue_t data;
    if (bTrangThaiWifi == true)
    {
      data.event = eEVENT_DISCONNECT_WIFI;
    }
    else
    {
      data.event = eEVENT_CONNECT_WIFI;
    }
    if (KetNoiWiFiQueue)
    {
      xQueueSend(KetNoiWiFiQueue, &data, 0);
    }
  }

  break;
  case HMI_CHANGE_ADMIN_PASSWORD:
    Serial.print("Password: ");
    Serial.println(event.text);
    SDMMCFile.writeFile(ADMIN_PASSWORD, event.text.c_str());
    break;
  case eHMI_SET_EVENT_WAKEUP:
    for (uint8_t i = 0; i < u8NUMBER_OF_TIMER_DWIN; i++)
    {
      xTimerStart(pxTimerDWINhdl[i], 1000);
    }
    break;
  case eHMI_SET_PID:
    Serial.printf("Start log PID\n");
    xTimerStart(xTimerPIDLog, 1000);
    break;
  case eHMI_EXIT_PID:
    Serial.printf("end log PID\n");
    xTimerStop(xTimerPIDLog, 1000);
    break;
  case eHMI_SET_PARAMTER_KP_TEMP_PID:
  {

    ControlParamaterTEMP pxParamter;
    pxParamter = _Heater.xGetControlParamater();
    pxParamter.xPID.Kp = (float)event.f_value;
    _Heater.vSetControlParamater(pxParamter);
  }
  break;
  case eHMI_SET_PARAMTER_KI_TEMP_PID:
  {
    ControlParamaterTEMP pxParamter;
    pxParamter = _Heater.xGetControlParamater();
    pxParamter.xPID.Ki = (float)event.f_value;
    _Heater.vSetControlParamater(pxParamter);
  }
  break;

  case eHMI_SET_PARAMTER_KD_TEMP_PID:
  {
    ControlParamaterTEMP pxParamter;
    pxParamter = _Heater.xGetControlParamater();
    pxParamter.xPID.Kd = (float)event.f_value;
    _Heater.vSetControlParamater(pxParamter);
  }
  break;

  case eHMI_SET_PARAMTER_KW_TEMP_PID:
  {
    ControlParamaterTEMP pxParamter;
    pxParamter = _Heater.xGetControlParamater();
    pxParamter.xPID.Kw = (float)event.f_value;
    _Heater.vSetControlParamater(pxParamter);
  }
  break;

  case eHMI_SET_PARAMTER_IMAX_TEMP_PID:
  {
    ControlParamaterTEMP pxParamter;
    pxParamter = _Heater.xGetControlParamater();
    pxParamter.xPID.WindupMax = (float)event.f_value;
    _Heater.vSetControlParamater(pxParamter);
  }
  break;

  case eHMI_SET_PARAMTER_IMIN_TEMP_PID:
  {
    ControlParamaterTEMP pxParamter;
    pxParamter = _Heater.xGetControlParamater();
    pxParamter.xPID.WindupMin = (float)event.f_value;
    _Heater.vSetControlParamater(pxParamter);
  }
  break;

  case eHMI_SET_PARAMTER_OUTMAX_TEMP_PID:
  {
    ControlParamaterTEMP pxParamter;
    pxParamter = _Heater.xGetControlParamater();
    pxParamter.xPID.OutMax = (float)event.f_value;
    _Heater.vSetControlParamater(pxParamter);
  }
  break;

  case eHMI_SET_PARAMTER_OUTMIN_TEMP_PID:
  {
    ControlParamaterTEMP pxParamter;
    pxParamter = _Heater.xGetControlParamater();
    pxParamter.xPID.OutMin = (float)event.f_value;
    _Heater.vSetControlParamater(pxParamter);
  }
  break;
  case eHMI_SET_PARAMTER_PERIMETER_TEMP:
  {
    ControlParamaterTEMP pxParamter;
    pxParamter = _Heater.xGetControlParamater();
    pxParamter.Perimter = (float)event.f_value;
    _Heater.vSetControlParamater(pxParamter);
  }
  break;
  case eHMI_SET_PARAMTER_DOOR_TEMP_PID:
  {
    ControlParamaterTEMP pxParamter;
    pxParamter = _Heater.xGetControlParamater();
    pxParamter.Door = (float)event.f_value;
    _Heater.vSetControlParamater(pxParamter);
  }
  break;
  case eHMI_SET_PARAMTER_KP_CO2_PID:
  {
    ControlParamaterCO2 pxParamter;
    pxParamter.xPID.Kp = (float)event.f_value;
    _CO2.vSetControlParamater(pxParamter);
  }
  break;

  case eHMI_SET_PARAMTER_KI_CO2_PID:
  {
    ControlParamaterCO2 pxParamter;
    pxParamter = _CO2.xGetControlParamater();
    pxParamter.xPID.Ki = (float)event.f_value;
    _CO2.vSetControlParamater(pxParamter);
  }
  break;

  case eHMI_SET_PARAMTER_KD_CO2_PID:
  {
    ControlParamaterCO2 pxParamter;
    pxParamter = _CO2.xGetControlParamater();
    pxParamter.xPID.Kd = (float)event.f_value;
    _CO2.vSetControlParamater(pxParamter);
  }
  break;

  case eHMI_SET_PARAMTER_KW_CO2_PID:
  {
    ControlParamaterCO2 pxParamter;
    pxParamter = _CO2.xGetControlParamater();
    pxParamter.xPID.Kw = (float)event.f_value;
    _CO2.vSetControlParamater(pxParamter);
  }
  break;

  case eHMI_SET_PARAMTER_IMAX_CO2_PID:
  {
    ControlParamaterCO2 pxParamter;
    pxParamter = _CO2.xGetControlParamater();
    pxParamter.xPID.WindupMax = (float)event.f_value;
    _CO2.vSetControlParamater(pxParamter);
  }
  break;

  case eHMI_SET_PARAMTER_IMIN_CO2_PID:
  {
    ControlParamaterCO2 pxParamter;
    pxParamter = _CO2.xGetControlParamater();
    pxParamter.xPID.WindupMin = (float)event.f_value;
    _CO2.vSetControlParamater(pxParamter);
  }
  break;

  case eHMI_SET_PARAMTER_OUTMAX_CO2_PID:
  {
    ControlParamaterCO2 pxParamter;
    pxParamter = _CO2.xGetControlParamater();
    pxParamter.xPID.OutMax = (float)event.f_value;
    _CO2.vSetControlParamater(pxParamter);
  }
  break;

  case eHMI_SET_PARAMTER_OUTMIN_CO2_PID:
  {
    ControlParamaterCO2 pxParamter;
    pxParamter = _CO2.xGetControlParamater();
    pxParamter.xPID.OutMin = (float)event.f_value;
    _CO2.vSetControlParamater(pxParamter);
  }
  break;
  default:
    Serial.printf("%s UNDEFINE\n", __func__);
    break;
  }
}
bool hmiGetEvent(hmi_get_type_t event, void *args)
{
  static FrameDataQueue_t xDataHMIGetEvent;
  if (recvHMIQueue == NULL)
    return 0;
  xDataHMIGetEvent.event = event;
  xDataHMIGetEvent.pvData = args;
  xQueueSend(recvHMIQueue, &xDataHMIGetEvent, 0);

  return 1;
}

void TaoCacThuMucHeThongTrenSD(void)
{
  // program, log
  SDMMCFile.createDir("/program");
  SDMMCFile.createDir("/log");
  SDMMCFile.createDir("/Update");
  SDMMCFile.createDir("/Update/Firmware");
  SDMMCFile.createDir("/Update/HMI");
  SDMMCFile.createDir("/UpdateStatus");
}
static void triggeONICONNhiet(void *)
{
  static FrameDataQueue_t data;
  data.event = eHMI_EVENT_ICON_NHIET;
  if (recvHMIQueue == NULL)
  {
    return;
  }
  xQueueSend(recvHMIQueue, &data, 0);
}
static void triggeOFFICONNhiet(void *)
{
  static FrameDataQueue_t data;
  data.event = eHMI_EVENT_ICON_NHIET;
  if (recvHMIQueue == NULL)
  {
    return;
  }
  xQueueSend(recvHMIQueue, &data, 0);
}
static void triggeONICONCO2(void *)
{
  static FrameDataQueue_t data;
  data.event = eHMI_EVENT_ICON_CO2;
  if (recvHMIQueue == NULL)
  {
    return;
  }
  xQueueSend(recvHMIQueue, &data, 0);
}
static void triggeOFFICONCO2(void *)
{
  static FrameDataQueue_t data;
  data.event = eHMI_EVENT_ICON_CO2;
  if (recvHMIQueue == NULL)
  {
    return;
  }
  xQueueSend(recvHMIQueue, &data, 0);
}

void hienThiList(int i)
{
  for (i = listPageStartPosition; i < 5 + listPageStartPosition; i++)
  {
    if (i < listLength + listPageStartPosition)
    {
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
    }
    else
    {
      _dwin.XoaDuLieuHienThiSegmentTrenHang(i - listPageStartPosition);
    }
  }
}
String xGetReasonDisconnect(WiFiEventInfo_t info)
{
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  String ErrMessage;
  switch (info.wifi_sta_disconnected.reason)
  {
  case WIFI_REASON_NO_AP_FOUND:
    ErrMessage = "No Wifi Name";
    break;
  case WIFI_REASON_AUTH_FAIL:
  case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    ErrMessage = "Wrong Password";
    break;
  case WIFI_REASON_ASSOC_LEAVE:
    ErrMessage = "Disconnected";
    break;
  default:
    ErrMessage = "lost - retrying";
    break;
  }
  return ErrMessage;
}

void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
  static FrameDataQueue_t data;
  static String ErrMessage;
  switch (event)
  {
  case ARDUINO_EVENT_WIFI_READY:
    Serial.println("WiFi interface ready");
    break;
  case ARDUINO_EVENT_WIFI_SCAN_DONE:
    Serial.println("Completed scan for access points");
    break;
  case ARDUINO_EVENT_WIFI_STA_CONNECTED:
  case ARDUINO_EVENT_WIFI_STA_GOT_IP:
    Serial.println("Connected to access point");
    data.event = eEVENT_CONNECT_WIFI_SUCCESS;
    if (KetNoiWiFiQueue)
      xQueueSend(KetNoiWiFiQueue, &data, 0);
    break;
  case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    ErrMessage = xGetReasonDisconnect(info);

    data.event = eEVENT_LOST_WIFI_CONNECTION;
    data.pvData = (void *)&ErrMessage;
    if (KetNoiWiFiQueue)
      xQueueSend(KetNoiWiFiQueue, &data, 0);

    data.event = eHMI_EVENT_ICON_WIFI;
    data.pvData = NULL;
    if (recvHMIQueue)
      xQueueSend(recvHMIQueue, &data, 0);
    break;
  default:
    Serial.printf("Err case [WiFi-event] event: %d\n", event);
    break;
  }
}

void TaskExportData(void *)
{
  vTaskSuspend(NULL);
  for (;;)
  {
    if (USB_MSC_HOST.isConnected())
    {
      copyFiles(USB_MSC_HOST, SD_MMC); // Test
      Serial.println("Copy done");
      delay(100);
    }
    else
    {
      _dwin.HienThiPhanTramThanhLoading("noUSB");
      _dwin.HienThiThanhLoading(0);
    }
    vTaskSuspend(NULL);
    // UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.printf("%s: Task's stack high water mark: %lu words\n", __func__, stackHighWaterMark);
  }
}

void TaskUpdateFirmware(void *)
{
  MethodUpdates_t method;
  for (;;)
  {
    xQueueReceive(QueueUpdateFirmware, &method, portMAX_DELAY);
    Serial.printf("%s nhận phương thức update: %s", __func__, method == MAIN_UPDATE_USB ? "USB" : "FOTA");

    if (method == MAIN_UPDATE_USB)
    {
      if (USB_MSC_HOST.isConnected())
      {
        // Kết thúc SD_MMC trước khi cập nhật để tránh lỗi
        SD_MMC.end();

        _dwin.HienThiWarning("updating firmware", _WarningPage);

        // Update firmware
        updateFirmware(USB_MSC_HOST);

        // Update HMI
        updateHMI(USB_MSC_HOST);
        _dwin.HienThiWarning("Done", _WarningPage);

        _dwin.HienThiWarning("Restart after 2s", _WarningPage);
        _dwin.HienThiWarning("Restart after 1s", _WarningPage);
        _dwin.HienThiWarning("Restarting...", _WarningPage);
        ESP.restart();
      }
      else
      {
        _dwin.HienThiWarning("no USB device", _UpdatePage);
      }
    }
    else if (method == MAIN_UPDATE_FOTA)
    {

      if (WiFi.status() != WL_CONNECTED)
      {
      }
      Serial.println("Current Version: " + convertDateToVersion(__DATE__));
      if (!downloadUpdatesFromJson(SD_MMC))
      {
        Serial.println("Error downloading updates or no new version available");
        _dwin.HienThiWarning("No new version available", _UpdatePage);
        // _dwin.setPage(_HomePage);
        // vTaskDelete(NULL);
        // continue;
      }
      else
      {

        // Kết thúc SD_MMC trước khi cập nhật để tránh lỗi
        // SD_MMC.end();
        _dwin.HienThiWarning("updating firmware", _WarningPage);
        // Update firmware
        updateFirmware(SD_MMC);

        // Update HMI
        updateHMI(SD_MMC);
        SD_MMC.end();
        _dwin.HienThiWarning("Done", _WarningPage);

        _dwin.HienThiWarning("Restart after 2s", _WarningPage);
        _dwin.HienThiWarning("Restart after 1s", _WarningPage);
        _dwin.HienThiWarning("Restarting...", _WarningPage);
        ESP.restart();
      }
    }
  }
}

void TaskKetNoiWiFi(void *)
{
  FrameDataQueue_t data;
  bool trangThaiKetNoi = false;
  WiFi.onEvent(WiFiEvent);

  for (;;)
  {
    xQueueReceive(KetNoiWiFiQueue, &data, portMAX_DELAY);
    Serial.printf("%s nhận event: %d, data nhận được %s\n", __func__, data.event, data.pvData == NULL ? "NULL" : "NOT NULL");
    // _dwin.HienThiTrangThaiKetNoiWiFi("");

    switch ((WifiEvent_t)data.event)
    {
    case eEVENT_CONNECT_WIFI:
      Serial.println("Connecting to " + WifiSSID);
      _dwin.HienThiTrangThaiKetNoiWiFi("Connecting");
      WiFi.begin(WifiSSID.c_str(), WifiPassword.c_str());
      WiFiConfig.state = true;
      strncpy(WiFiConfig.ssid, WifiSSID.c_str(), 30);
      strncpy(WiFiConfig.password, WifiPassword.c_str(), 30);
      _dwin.HienThiSSIDWiFi(WifiSSID);
      _dwin.HienThiPasswordWiFi(WifiPassword);
      SDMMCFile.writeFile(WIFICONFIG_PATH, (uint8_t *)&WiFiConfig, sizeof(WiFiConfig));
      bTrangThaiWifi = true;
      continue;
    case eEVENT_DISCONNECT_WIFI:
    {
      int8_t i8Try = 5;
      _dwin.HienThiTrangThaiKetNoiWiFi("Disconnected");
      Serial.println("disconnect to " + WifiSSID);
      _dwin.setVP(_VPAddressIonConnect, 0);
      _dwin.setVP(_VPAddressIconWiFi, 0);
      WiFiConfig.state = false;
      do
      {
        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);
        delay(100);
      } while (WiFi.status() == WL_CONNECTED && i8Try--);
      SDMMCFile.writeFile(WIFICONFIG_PATH, (uint8_t *)&WiFiConfig, sizeof(WiFiConfig));
      bTrangThaiWifi = false;
    }
    break;
    case eEVENT_CONNECT_WIFI_SUCCESS:
      _dwin.setVP(_VPAddressIonConnect, 2);
      _dwin.HienThiTrangThaiKetNoiWiFi("MAC: " + String(WiFi.macAddress()));
      _dwin.setVP(_VPAddressIconWiFi, map(constrain(WiFi.RSSI(), -100, -40), -100, -40, 1, 4));
      strcpy(WiFiConfig.ssid, WifiSSID.c_str());
      strcpy(WiFiConfig.password, WifiPassword.c_str());
      setup_PostGet();
      break;
    case eEVENT_LOST_WIFI_CONNECTION:
    {
      _dwin.setVP(_VPAddressIconWiFi, 0);
      if (!data.pvData || bTrangThaiWifi == false)
        continue;
      String message = *((String *)data.pvData);
      _dwin.HienThiTrangThaiKetNoiWiFi(message);
      WiFi.begin(WifiSSID.c_str(), WifiPassword.c_str());
    }
    break;
    default:
      break;
    }
    UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("%s: Task's stack high water mark: %lu words\n", __func__, stackHighWaterMark);
  }
}

// Task này dùng để xử lý trong quá trình tủ chạy như là chuyển segment của program,
// Tắt tủ khi hết thời
// Chạy các chế độ tiệt trùng, program, quick mode
void TaskMain(void *)
{
  bool machineState;
  bool FlagUSB = false;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  FrameDataQueue_t sendData = {
      .pvData = NULL,
  };
  uint16_t chuKyRecord = 0; // 60s

  while (1)
  {
    // chu kỳ record
    if (chuKyRecord >= 1)
    {
      chuKyRecord = 0;
      RecordData_t record = {
          .setpointTemp = BaseProgram.programData.setPointTemp,
          .valueTemp = BaseProgram.temperature,
          .setpointCO2 = BaseProgram.programData.setPointCO2,
          .valueCO2 = BaseProgram.CO2,
          .fan = BaseProgram.programData.fanSpeed,
          .day = RTCnow.day(),
          .month = RTCnow.month(),
          .year = RTCnow.year() % 2000,
          .hour = RTCnow.hour(),
          .minute = RTCnow.minute(),
          .second = RTCnow.second()};
      writeRecord(SD_MMC, record);
    }
    chuKyRecord++;

    // Phần xác định nhiệt độ đã xác lập hay chưa
    if (fabs(BaseProgram.temperature - BaseProgram.programData.setPointTemp) <= 0.15 &&
        fabs(BaseProgram.CO2 - BaseProgram.programData.setPointCO2) <= 0.15)
    {
      if (FlagNhietDoXacLap == false)
      {
        if (BaseProgram.delayOffState)
        {
          DemThoiGianChay(true, DEM_XUONG);
        }
      }
      FlagNhietDoXacLap = true;
    }

    // Phần kiểm tra và cập nhật thời gian đếm xuống / đếm lên và set cờ chuyển segment = true nếu thời gian đềm ngược về 0
    if (BaseProgram.machineState == true)
    {
      // Nhiệt độ xác lập và trạng thái hẹn giờ là đếm xuống thì cho chạy đếm xuống
      if (BaseProgram.delayOffState && FlagNhietDoXacLap)
      {
        // Kiểm tra đã đếm thời gian về 00:00:00 chưa
        if (DemThoiGianChay(false, DEM_XUONG) == false)
        {
          Serial.println("Chuyen segment");
          SwitchSegment = true;
          FlagNhietDoXacLap = false;
          _dwin.Buzzer(400);
          if (RunMode == STERILIZATION_MODE)
          {
            // tắt máy
            BaseProgram.machineState = false;
            _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);
            _dwin.HienThiThoiGianChay("FINISH");

            _dwin.Buzzer(800);
          }
        }
      }
      else if (BaseProgram.delayOffState == false)
      {
        DemThoiGianChay(false, DEM_LEN);
      }
    }

    // Phần xử lý chuyển segment và tắt máy
    // if(RunMode == PROGRAM_MODE && SwitchSegment == true && BaseProgram.machineState == true)
    // if (RunMode == PROGRAM_MODE && SwitchSegment == true)
    if (SwitchSegment == true)
    {
      Serial.println("SwitchSegment");
      SwitchSegment = false;
      if (RunMode == PROGRAM_MODE)
      {
        if (CurrentProgram.size() > RunningSegmentIndex)
        {
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
          if (BaseProgram.programData.delayOffDay == 0 && BaseProgram.programData.delayOffHour == 0 && BaseProgram.programData.delayOffMinute == 0)
          {
            _dwin.HienThiIconOnOffDelayOff(false);
          }
          else
          {
            _dwin.HienThiIconOnOffDelayOff(true);
          }
          RunningSegmentIndex++;
        }
        else
        {
          programLoopCount++;
          if (programLoopCount < programLoop && programInf == false)
          {
            // BaseProgram.machineState = true;
            FlagNhietDoXacLap = false;
            SwitchSegment = true;
            RunningSegmentIndex = 0;
            _dwin.HienThiVongLapChuongTrinhConLai(programLoopCount + 1, programLoop);
            if (BaseProgram.delayOffState)
            {
              DemThoiGianChay(true, DEM_XUONG);
            }
            else
            {
              DemThoiGianChay(true, DEM_LEN);
            }
          }
          else if (programInf)
          {
            // BaseProgram.machineState = true;
            FlagNhietDoXacLap = false;
            SwitchSegment = true;
            RunningSegmentIndex = 0;
            if (BaseProgram.delayOffState)
            {
              DemThoiGianChay(true, DEM_XUONG);
            }
            else
            {
              DemThoiGianChay(true, DEM_LEN);
            }
          }
          else
          {
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
    _CO2.CaiNongDoCO2(BaseProgram.programData.setPointCO2);
    // _Heater.CaiGiaTriOfset(GetCalib(BaseProgram.programData.setPointTemp));
    if (machineState != BaseProgram.machineState)
    {
      if (BaseProgram.machineState == true)
      {
        BatMay(__func__);
      }
      else if (BaseProgram.machineState == false)
      {
        TatMay(__func__);
      }
      machineState = BaseProgram.machineState;
    }
    if (BaseProgram.machineState == true)
    {
      xTimerReset(xTimerSleep, 10);
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
  }
}

void TaskHMI(void *)
{
  FrameDataQueue_t data;
  FrameDataQueue_t dataSent;
  int programStart = 0, programEnd = 0;

  while (1)
  {
    xQueueReceive(recvHMIQueue, &data, portMAX_DELAY);
    // Serial.printf("%s nhận event: %d, data nhận được %s\n", __func__, data.event, data.pvData == NULL ? "NULL" : "NOT NULL");
    int i = 0;
    time_t timeNow;
    File file, root;

    switch ((hmi_get_type_t)data.event)
    {
    case eHMI_EVENT_ICON_NHIET:
      if (_Heater.TrangThaiThanhGiaNhiet() == 1)
      { // gia nhiet
        _dwin.HienThiIconGiaNhiet(1);
      }
      else if (_Heater.TrangThaiThanhGiaNhiet() == 0)
      {
        _dwin.HienThiIconGiaNhiet(0);
      }
      break;
    case eHMI_EVENT_ICON_CO2:
      if (_CO2.LayTrangThaiVan() == 1)
      { // bat van khi
        _dwin.HienThiIconVanCO2(1);
      }
      else if (_CO2.LayTrangThaiVan() == 0)
      {
        _dwin.HienThiIconVanCO2(0);
      }
      break;
    case eHMI_EVENT_ICON_CUA:
      if (_Door.TrangThai() == DOOR_CLOSE)
      {
        _dwin.HienThiIconCua(0);
      }
      else if (_Door.TrangThai() == DOOR_OPEN)
      {
        _dwin.HienThiIconCua(1);
      }
      break;
    case eHMI_EVENT_ICON_FAN:
      if (_Heater.TrangThaiQuat() == 1)
      {
        _dwin.HienThiIconQuat(1);
      }
      else if (_Heater.TrangThaiQuat() == 0)
      {
        _dwin.HienThiIconQuat(0);
      }
      break;
    case eHMI_EVENT_ICON_USB:
      if (USB_MSC_HOST.isConnected())
      {
        _dwin.HienThiIconUSB(1);
      }
      else
      {
        _dwin.HienThiIconUSB(0);
      }
      break;
    case eHMI_EVENT_ICON_WIFI:
      if (WiFi.status() == WL_CONNECTED)
      {
        _dwin.setVP(_VPAddressIconWiFi, map(constrain(WiFi.RSSI(), -100, -40), -100, -40, 1, 4));
      }
      else
      {
        _dwin.setVP(_VPAddressIconWiFi, 0);
      }
      break;
    case eHMI_EVENT_HIEN_THI_GIA_TRI_CAM_BIEN: // cập nhật 1s
      BaseProgram.CO2 = _CO2.LayNongDoCO2Thuc();
      if (BaseProgram.CO2 >= -0.5f && BaseProgram.CO2 <= 30.0f)
      {
        _dwin.HienThiCO2(BaseProgram.CO2);
      }
      else if (BaseProgram.CO2 < -0.5f)
      {
        _dwin.HienThiCO2("wait");
      }
      else
      {
        _dwin.HienThiCO2("err");
      }
      BaseProgram.temperature = _Heater.LayNhietDoLoc();
      if (BaseProgram.temperature > -10 && BaseProgram.temperature <= 350)
      {
        _dwin.HienThiNhietDo(BaseProgram.temperature);
      }
      else
      {
        _dwin.HienThiNhietDo("err");
      }
      break;
    case eHMI_EVENT_HIEN_THI_PID:
    {
      PIDCalcu_t xPIDCaluTemp = _Heater.xGetCalcu();
      PIDParam_t xPIDParamTemp = _Heater.xGetParam();
      PIDCalcu_t xPIDCaluCO2 = _CO2.xGetCalcu();
      PIDParam_t xPIDParamCO2 = _CO2.xGetParam();
      float errTemp = BaseProgram.programData.setPointTemp - BaseProgram.temperature;
      float errCO2 = BaseProgram.programData.setPointCO2 - BaseProgram.CO2;
      float thoiGianBatCua = _Heater.LayGiaTriDieuKhienCua();
      float thoiGianBatVanh = _Heater.LayGiaTriDieuKhienVanh();
      _dwin.HienThiThongSoTrangPID(xPIDCaluTemp, xPIDParamTemp, errTemp, thoiGianBatCua, thoiGianBatVanh, xPIDCaluCO2, xPIDParamCO2, errCO2);
    }
    break;
    case eHMI_EVENT_HIEN_THI_THOI_GIAN: // cập nhật 1s
      RTCnow = _time.getCurrentTime();
      _dwin.HienThiThoiGianRTC(RTCnow.day(), RTCnow.month(), RTCnow.year() % 1000, RTCnow.hour(), RTCnow.minute(), RTCnow.second());
      break;
    case eHMI_EVENT_VE_DO_THI: // cập nhật theo chu kỳ riêng (hiện tại 1s)
      RTCnow = _time.getCurrentTime();
      _dwin.VeDoThi(BaseProgram, RTCnow.unixtime());
      break;
    case eHMI_EVENT_WARNING: // cập nhật chu kỳ hoặc có lỗi xuất hiện
    {
      std::vector<String> warningVector;
      String warningText = "";
      if (FlagNhietDoXacLap == true)
      {
        String text;
        if (BaseProgram.temperature >= BaseProgram.programData.setPointTemp + BaseProgram.programData.tempMax)
        {
          text = "Alarm: Overheat ";
          warningVector.push_back(text);
        }
        else if (BaseProgram.temperature <= BaseProgram.programData.setPointTemp + BaseProgram.programData.tempMin)
        {
          text = "Alarm: Underheat ";
          warningVector.push_back(text);
        }

        if (BaseProgram.CO2 >= BaseProgram.programData.setPointCO2 + BaseProgram.programData.CO2Max)
        {
          text += "Alarm: OverCO2 ";
          warningVector.push_back(text);
        }
        else if (BaseProgram.CO2 <= BaseProgram.programData.setPointCO2 + BaseProgram.programData.CO2Min)
        {
          text += "Alarm: UnderCO2 ";
          warningVector.push_back(text);
        }
      }

      if (BaseProgram.temperature < -10 || BaseProgram.temperature >= 350)
      {
        warningText = "Err: temprature sensor";
        warningVector.push_back(warningText);
      }
      if (BaseProgram.CO2 > 30)
      {
        warningText = "Err: CO2 sensor";
        warningVector.push_back(warningText);
      }

      if (BaseProgram.machineState == true && _Heater.CheckNguonCongSuat() == false)
      {
        warningVector.push_back("Alarm: Thermal relay actived");
      }

      if (!warningVector.empty())
      {
        _dwin.HienThiWarning(warningVector, _HomePage);
        _dwin.Buzzer(800);
      }
    }
    break;
    case eHMI_EVENT_REFRESH:
    {
      uint8_t page = _dwin.getPage();
      if (page == _EndIntroPage || page == _SleepPage)
      {
        Serial.println("\t\tDWIN REFRESH\n");
        _dwin.HienThiSetpointTemp(BaseProgram.programData.setPointTemp);
        _dwin.HienThiSetpointCO2(BaseProgram.programData.setPointCO2);
        _dwin.HienThiTocDoQuat(BaseProgram.programData.fanSpeed);
        _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);
        _dwin.setBrightness(50);
        _dwin.setPage(_HomePage);
      }
    }

    break;
    case HMI_CHECK_LIST:
      if (data.pvData == NULL)
        continue;
      if (*((uint8_t *)data.pvData) + listPageStartPosition >= ProgramList.size())
      {
        continue;
      }
      break;
    case eHMI_EVENT_TIMEROUT_OFF:
      if (_dwin.getPage() == _HomePage)
      {
        Serial.printf("\t\tDWIN OFF\n");
        //! chuyển hàm này vào demon task
        for (uint8_t i = 0; i < u8NUMBER_OF_TIMER_DWIN; i++)
        {
          xTimerStop(pxTimerDWINhdl[i], 1000);
        }
        _dwin.Buzzer(800);
        _dwin.setBrightness(1);
        _dwin.setPage(_SleepPage);
      }
    case HMI_GET_RTC:
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
      if (!root)
      {
        Serial.println("Failed to open directory");
        continue;
      }
      if (!root.isDirectory())
      {
        Serial.println("/program is not a directory");
        continue;
      }
      file = root.openNextFile();
      programStart = 0;
      while (file)
      {
        Serial.print("File: ");
        String name = file.name();
        Serial.println(name);
        _dwin.HienThiTenChuongTrinhTrenHang(programListIndex, programListIndex + 1, name, file.size() / sizeof(Program_t), __func__);
        file = root.openNextFile();
        programListIndex++;
        if (programListIndex % 6 == 0)
        {
          break;
        }
      }
      if (programListIndex < 6)
      {
        for (int8_t i = programListIndex; i < 6; i++)
        {
          _dwin.XoaDuLieuHienThiTenChuongTrinhTrenHang(i);
        }
      }
      Serial.println("HMI_GET_PROGRAM_LIST");
      break;
    case HMI_GET_NEXT_PROGRAM_LIST:
      if (programListIndex - programStart == 0)
      {
        programStart -= 6;
      }
      else
      {
        programStart = (programListIndex / 6) * 6;
      }
      programListIndex = 0;
      Serial.printf("programStart: %d\n", programStart);
      Serial.printf("programListIndex: %d\n", programListIndex);
      root = SD_MMC.open("/program");
      if (!root)
      {
        Serial.println("Failed to open directory");
        continue;
      }
      if (!root.isDirectory())
      {
        Serial.println("/program is not a directory");
        continue;
      }
      file = root.openNextFile();
      while (file)
      {
        Serial.print("File: ");
        Serial.println(file.name());
        if (programListIndex >= programStart)
        {
          _dwin.HienThiTenChuongTrinhTrenHang(programListIndex - programStart, programListIndex + 1, file.name(), file.size() / sizeof(Program_t), __func__);
          file.close();
        }
        file = root.openNextFile();
        programListIndex++;
        if (programListIndex % 6 == 0 && programListIndex > programStart)
        {
          file.close();
          break;
        }
      }
      if (programListIndex % 6 != 0)
      {
        for (int8_t i = programListIndex - programStart; i < 6; i++)
        {
          _dwin.XoaDuLieuHienThiTenChuongTrinhTrenHang(i);
        }
      }
      break;
    case HMI_GET_BACK_PROGRAM_LIST:
      if (programStart >= 6)
      {
        programStart -= 6;
      }
      else if (programStart == 0)
      {
        continue;
      }
      else
      {
        programStart = 0;
      }
      programListIndex = 0;
      root = SD_MMC.open("/program");
      if (!root)
      {
        Serial.println("Failed to open directory");
        continue;
      }
      if (!root.isDirectory())
      {
        Serial.println("/program is not a directory");
        continue;
      }
      file = root.openNextFile();
      while (file)
      {
        Serial.print("File: ");
        Serial.println(file.name());
        if (programListIndex >= programStart)
        {
          _dwin.HienThiTenChuongTrinhTrenHang(programListIndex - programStart, programListIndex + 1, file.name(), file.size() / sizeof(Program_t), __func__);
          file.close();
        }
        file = root.openNextFile();
        programListIndex++;
        if (programListIndex % 6 == 0 && programListIndex > programStart)
        {
          file.close();
          break;
        }
      }
      if (programListIndex % 6 != 0)
      {
        for (int8_t i = programListIndex - programStart; i < 6; i++)
        {
          _dwin.XoaDuLieuHienThiTenChuongTrinhTrenHang(i);
        }
      }

      break;
    case HMI_GET_SEGMENT_LIST:
      Serial.println("List_segment");
      listPageStartPosition = 0;
      if (ProgramList.size() >= 5)
      {
        listLength = 5;
      }
      else
      {
        listLength = ProgramList.size();
      }
      hienThiList(i);
      break;
    case HMI_GET_NEXT_SEGMENT_LIST:
      if (ProgramList.size() > listPageStartPosition + 5)
      {
        listPageStartPosition += 5;
        listLength = ProgramList.size() - listPageStartPosition;
        if (listLength > 5)
        {
          listLength = 5;
        }
      }
      else if (ProgramList.size() <= 5)
      {
        listLength = ProgramList.size();
      }
      else if (ProgramList.size() <= listPageStartPosition + 5)
      {
        listLength = ProgramList.size() - listPageStartPosition;
      }
      else
      {
        Serial.println("Segment List is full");
      }
      hienThiList(i);
      break;
    case HMI_GET_BACK_SEGMENT_LIST:
      if (listPageStartPosition - 5 >= 0)
      {
        listPageStartPosition -= 5;
        listLength = 5;
      }
      else if (ProgramList.size() <= 5)
      {
        listPageStartPosition = 0;
        listLength = ProgramList.size();
      }
      else if (listPageStartPosition - 5 < 0)
      {
        listPageStartPosition = 0;
        listLength = 5;
      }
      hienThiList(i);
      break;
    case HMI_REFRESH_SEGMENT_LIST:
      if (ProgramList.size() <= 5)
      {
        listLength = ProgramList.size();
      }
      else if (ProgramList.size() <= listPageStartPosition + 5)
      {
        listLength = ProgramList.size() - listPageStartPosition;
      }
      hienThiList(i);
      break;
    case HMI_GET_SEGMENT_DELAYOFF:
      if (data.pvData == NULL)
        continue;
      _dwin.HienThiNgay((ProgramList[*((uint8_t *)data.pvData) + listPageStartPosition].delayOffDay));
      _dwin.HienThiGio((ProgramList[*((uint8_t *)data.pvData) + listPageStartPosition].delayOffHour));
      _dwin.HienThiPhut((ProgramList[*((uint8_t *)data.pvData) + listPageStartPosition].delayOffMinute));
      break;
    case HMI_GET_CALIB:
      // _dwin.HienThiHeSoCalib(GetCalib(BaseProgram.programData.setPointTemp));
      break;
    case HMI_GET_SCAN_SSID_WIFI:
    {
      std::vector<String> vectorSSID;
      vectorSSID.reserve(4);
      vectorSSID.push_back("Scanning");
      for (int8_t i = 1; i < 4; i++)
      {
        vectorSSID.push_back("");
      }
      _dwin.HienThiListSSIDWifi(vectorSSID);

      if (WiFi.status() == WL_DISCONNECTED)
      {
        dataSent.event = eEVENT_DISCONNECT_WIFI;
        if (KetNoiWiFiQueue)
        {
          xQueueSend(KetNoiWiFiQueue, &dataSent, 0);
        }
      }

      int16_t n = WiFi.scanNetworks();
      Serial.printf("scan done %d\n", n);

      if (n == 0)
      {
        vectorSSID.at(0) = "no wifi found";
      }
      for (int8_t i = 0; i < 4; i++)
      {
        vectorSSID.at(i) = WiFi.SSID(i);
      }

      _dwin.HienThiListSSIDWifi(vectorSSID);
    }
    break;
    default:
      Serial.printf("%s nhận UNDEFINE event %u\n", __func__, data.event);
      break;
    }
  }
}

void TaskMonitorDebug(void *)
{
  TickType_t pxPreviousWakeTime;
  pxPreviousWakeTime = xTaskGetTickCount();
  char buffer[1024]; // Bộ nhớ lưu danh sách task
  while (1)
  {
    Serial.printf("Task Name\tState\tPrio\tStack Left\tTask Num\n");
    vTaskList(buffer);             // Lấy danh sách task
    Serial.printf("%s\n", buffer); // In ra Serial
    heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);
    vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(1000));
  }
}
void TaskHardWareTesting(void *)
{
  while (1)
  {
  }
}
