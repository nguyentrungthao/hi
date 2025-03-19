#include <WiFi.h>
// #include <HTTPClient.h>
#include "03_POSTGET.h"
#include <Arduino_JSON.h>

// const char *ssid = "LABone:KingLab-HUYlab";
// const char *password = "66668888";

POSTGET _POSTGET;
String MAC;

String MatKhau = "LABone@2024";
float Setpoint = 37;
float HeSoCalib = 0.1;
uint32_t ThoiGianTat = 13;

int32_t preTime;
size_t freeMemoryMin = 300000;
String LenhGuiXuongBoard = "";
String _LenhGuiXuongBoard = "";
int _CODE;     // Mã CODE gửi từ app xuống board để thực thi các tác vụ.
int _MODE = 0; // Chế độ làm việc của máy.
bool TrangThaiMay = 0;
uint8_t CuongDoWiFi;

// Tách chuỗi
//-----------------------------------------------------------------------------------------------------
String TachChuoi(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++)
    {
        if (data.charAt(i) == separator || i == maxIndex)
        {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void ThucThiTacVuTheoCODE(void)
{
    String Data1;
    char DuLieuXem[100];
    hmi_set_event_t event;
    switch (_CODE)
    {
    case 2:
        Serial.println("User bam 'xem'");
        _POSTGET.CapNhatCODETrongDatabaseTrenServer(_LenhGuiXuongBoard + "20");
        // String Data2 = MatKhau + ";" + String(HeSoCalib, 1) + ";" + String(Setpoint, 1) + ";" + String(ThoiGianTat) + ";" + "20:06:13 18/08/2024";
        snprintf(DuLieuXem, sizeof(DuLieuXem), "%s;%0.1f;%0.1f;%d;20:06:12 02/12/2024", "LABone@2024", HeSoCalib, BaseProgram.programData.setPointTemp, ThoiGianTat);
#if defined(debug)
        Serial.println(DuLieuXem);
#endif
        if (_POSTGET.POSTThongSoBoard(DuLieuXem))
        {
            _POSTGET.CapNhatCODETrongDatabaseTrenServer(_LenhGuiXuongBoard + "21");
        }

        break;
    case 3:
        Serial.println("User bam 'Cap nhat'");
        _POSTGET.CapNhatCODETrongDatabaseTrenServer(_LenhGuiXuongBoard + "30");
        Data1 = _POSTGET.GETThongSoBoard();

        MatKhau = TachChuoi(Data1, ';', 0);
        HeSoCalib = TachChuoi(Data1, ';', 1).toFloat();
        Setpoint = TachChuoi(Data1, ';', 2).toFloat();
        ThoiGianTat = TachChuoi(Data1, ';', 3).toInt();

        if (Setpoint != BaseProgram.programData.setPointTemp)
        {
            event.type = HMI_SET_SETPOINT_TEMP;
            event.f_value = Setpoint;
            BaseProgram.programData.setPointTemp = Setpoint;
            hmiSetEvent(event);
            _dwin.HienThiSetpointTemp(BaseProgram.programData.setPointTemp);
        }

        if (ThoiGianTat != 0)
        {
            event.type = HMI_SET_DELAYOFF;
            event.u32_value = ThoiGianTat * 60;
            hmiSetEvent(event);
            _dwin.HienThiSetpointTemp(BaseProgram.programData.setPointTemp);
            if (BaseProgram.delayOffState == false)
            {
                event.type = HMI_SET_DELAYOFF_ONOFF;
                hmiSetEvent(event);
            }
        }
        else
        {
            if (BaseProgram.delayOffState == true)
            {
                event.type = HMI_SET_DELAYOFF_ONOFF;
                hmiSetEvent(event);
            }
        }

        _POSTGET.CapNhatCODETrongDatabaseTrenServer(_LenhGuiXuongBoard + "31");

        break;
    case 4:
        Serial.println("Cap nhat firmware");
        break;
    case 5:
        Serial.println("Xem phien ban firmware");
        break;
    case 9:
        Serial.println("Cap nhat thoi gian thuc");
        event.u32_value = _POSTGET.LayRTCTuServerIoTVision();
        if (event.u32_value > 0)
        {
            event.type = HMI_SET_RTC;
            hmiSetEvent(event);
            _POSTGET.CapNhatCODETrongDatabaseTrenServer(_LenhGuiXuongBoard + "00");
        }
        break;
    case 11:
        Serial.println("Doi trang thai On/Off");
        if (TrangThaiMay == _LenhGuiXuongBoard.toInt())
        {
            TrangThaiMay = !TrangThaiMay;
            _LenhGuiXuongBoard = String(TrangThaiMay);
        }
        else
        {
            TrangThaiMay = (bool)_LenhGuiXuongBoard.toInt();
        }

        if (TrangThaiMay != BaseProgram.machineState)
        {
            event.type = HMI_SET_RUN_ONOFF;
            hmiSetEvent(event);
            _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);
        }

        _POSTGET.CapNhatCODETrongDatabaseTrenServer(String(TrangThaiMay) + "00");
        // snprintf(DuLieuXem, sizeof(DuLieuXem), "%d%u;%.1f;%.1f;%d;%d;%s", TrangThaiMay, CuongDoWiFi, Setpoint, 30.0, ThoiGianTat, 22, "21:34:13 01/12/2025");
        // Serial.println(DuLieuXem);
        // _POSTGET.POSTDuLieuBoard(DuLieuXem);

        break;
    case 13:
        Serial.println("Reset lai ESP");
        break;
    case 16:
        // TrangThaiMay = 0;
        // _POSTGET.CapNhatCODETrongDatabaseTrenServer("000");
        // snprintf(DuLieuXem, sizeof(DuLieuXem), "%d%d;%.1f;%.1f;%d;%d;%s", TrangThaiMay, 88, Setpoint, 30.0, ThoiGianTat, 22, "21:34:13 01/12/2025");
        // Serial.println(DuLieuXem);
        // _POSTGET.POSTDuLieuBoard(DuLieuXem);
        break;
    default:
        break;
    }
}

void setup_PostGet()
{
    // Serial.begin(115200);

    // Kết nối WiFi
    // WiFi.begin(ssid, password);
    // while (WiFi.status() != WL_CONNECTED)
    // {
    //     delay(1000);
    //     Serial.println("Connecting to WiFi...");
    // }
    MAC = WiFi.macAddress();
    MAC.replace(":", "");
    Serial.print("WiFi connected, MAC: ");
    Serial.println(MAC);

    // Tạo một task mới để thực hiện GET
    _POSTGET.CaiIDPostGet(MAC);
    // xTaskCreate(httpGetTask, "HTTP GET Task", 5120, NULL, 1, NULL);
}

void loop_PostGet()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        LenhGuiXuongBoard = _POSTGET.GETLenhGuiXuongBoard();
        if (LenhGuiXuongBoard.length() > 0)
        {
#pragma region Lấy lệnh gửi từ app xuống board thành công

            JSONVar json = JSON.parse(LenhGuiXuongBoard);
            String S = (const char*)json["S"];
            _LenhGuiXuongBoard = S.substring(0, 1);

#if defined(debug) && defined(DEBUG_POST_GET)
            Serial.print("Lệnh điều khiển ON/OFF gửi từ app xuống board: ");
            Serial.println(_LenhGuiXuongBoard);
#endif

            int CODE = S.substring(1).toInt();

#if defined(debug) && defined(DEBUG_POST_GET)
            Serial.print("CODE gửi xuống board: ");
            Serial.println(CODE);
#endif

#if defined(debug) && defined(DEBUG_POST_GET)
            if (CODE == 20 || CODE == 21)
                Serial.println("Đang lấy thông số của board gửi về server...");
            else if (CODE == 30 || CODE == 31)
                Serial.println("Đang cập nhật thông số từ app cài xuống board...");
            else if (CODE == 40 || CODE == 41)
                Serial.println("Đang cập nhật firmware cho board...");
            else if (CODE == 50 || CODE == 51)
                Serial.println("Đang lấy phiên bản firmware của board gửi về server...");
#endif

            if (CODE != 0 && CODE != 20 && CODE != 21 && CODE != 30 && CODE != 31 && CODE != 40 && CODE != 41 && CODE != 50 && CODE != 51)
            {
                _CODE = CODE;
                ThucThiTacVuTheoCODE();
                _CODE = 0;
            }
#pragma endregion Lấy lệnh gửi từ app xuống board thành công
        }
        else
        {
#if defined(debug) && defined(DEBUG_POST_GET)
            Serial.println("Lỗi GET lệnh từ app xuống board hoặc không có WIFI");
#endif
        }
    }
    delay(100);
    //-----------------------------------------------------------------------------------------
    // Chuỗi dulieu sẽ gồm K1MODERSSI;CO2CaiDat;NongDoCO2ThucTe;NhietDoCaiDat;NhietDoThucTe;ThoiGianONCaiDat;ThoiGianDaON;RTC
    // Ví dụ: 175;500;27.9;32.5;32.49;130;80.6;21:34:13 30/07/2023
    // nghĩa là
    // s00: K1 = 1
    // s01: Độ mạnh WiFI = 75%
    // s1: Tốc độ CO2 cài đặt = 500 (RPM)
    // s2: Tốc độ CO2 thực tế hiện tại = 27.9 (RPM)
    // s3: Nhiệt độ cài đặt: 32.5 (độ C)
    // s4: Nhiệt độ thực tế: 32.49 (độ C)
    // s5: Thời gian ON cài đặt = 130 (phút)
    // s6: Thời gian đã ON = 80,6 (phút )
    // s7: Thời gian thực RTC trên board = 30/07/2023 21:34:13
    if (millis() - preTime > 1000)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            char data[100];
            CuongDoWiFi = map(constrain(WiFi.RSSI(), -100, -30), -100, -30, 0, 100);

            if (!BaseProgram.delayOffState)
            {
                ThoiGianTat = 0;
                // _time.getRemainingTime();
                snprintf(data, sizeof(data), "%d%u;%.1f;%.1f;%.1f;%.1f;%d;%d;%02u:%02u:%02u %02u/%02u/%4u",
                    BaseProgram.machineState, CuongDoWiFi, BaseProgram.programData.setPointCO2, BaseProgram.CO2, BaseProgram.programData.setPointTemp, BaseProgram.temperature, ThoiGianTat,
                    (uint32_t)_time.getElapsedTime() / 60, RTCnow.hour(), RTCnow.minute(), RTCnow.second(), RTCnow.day(), RTCnow.month(), RTCnow.year());
            }
            else
            {
                ThoiGianTat = BaseProgram.programData.delayOffDay * 1440 + BaseProgram.programData.delayOffHour * 60 + BaseProgram.programData.delayOffMinute;
                Serial.print("ThoiGianTat ");
                Serial.println(ThoiGianTat);
                if (FlagNhietDoXacLap)
                {
                    snprintf(data, sizeof(data), "%d%u;%.1f;%.1f;%.1f;%.1f;%d;%d;%02u:%02u:%02u %02u/%02u/%4u",
                        BaseProgram.machineState, CuongDoWiFi, BaseProgram.programData.setPointCO2, BaseProgram.CO2, BaseProgram.programData.setPointTemp, BaseProgram.temperature, ThoiGianTat,
                        (uint32_t)_time.getRemainingTime() / 60, RTCnow.hour(), RTCnow.minute(), RTCnow.second(), RTCnow.day(), RTCnow.month(), RTCnow.year());
                }
                else
                {
                    snprintf(data, sizeof(data), "%d%u;%.1f;%.1f;%.1f;%.1f;%d;%d;%02u:%02u:%02u %02u/%02u/%4u",
                        BaseProgram.machineState, CuongDoWiFi, BaseProgram.programData.setPointCO2, BaseProgram.CO2, BaseProgram.programData.setPointTemp, BaseProgram.temperature, ThoiGianTat,
                        ThoiGianTat, RTCnow.hour(), RTCnow.minute(), RTCnow.second(), RTCnow.day(), RTCnow.month(), RTCnow.year());
                }

                Serial.print("ThoiGianChayConLai ");
                Serial.println((uint32_t)_time.getRemainingTime() / 60);
            }
            // Serial.println(data);
            _POSTGET.POSTDuLieuBoard(data);

            // snprintf(data, sizeof(data), "%s;%0.1f;%0.1f;%d;20:06:12 02/12/2024", "LABone@2024", HeSoCalib, BaseProgram.programData.setPointTemp, ThoiGianTat);
            // if (_POSTGET.POSTThongSoBoard(data)) {

            // }

            // Serial.printf("Minimum heap free size: %u bytes\n", esp_get_minimum_free_heap_size());
            // Serial.printf("Free heap size: %u bytes\n", esp_get_free_heap_size());
            preTime = millis();
        }
    }
}