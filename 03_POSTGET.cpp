#include "03_POSTGET.h"

#ifdef CODE_POST_GET_CU
#include "03_POSTGET.h"
#include <Arduino_JSON.h>  // Thư viện xử lý dữ liệu kiểu JSON
#include <WiFi.h> // Thư viện WiFi của ESP32.
#include <RTClib.h>

#define URL_GETLenhGuiXuongBoard "https://App.IoTVision.vn/api/LABone_KingIncu_DuLieuGuiXuongBoard?CheDo=1&key="
#define URL_CapNhatCODE "https://App.IoTVision.vn/api/LABone_KingIncu_DuLieuGuiXuongBoard"
#define URL_POSTDuLieuTuBoardLenServer "https://App.IoTVision.vn/api/LABone_KingIncu_DuLieu"
#define URL_POSTGETThongSoBoard "https://App.IoTVision.vn/api/LABone_KingIncu_ThongSoCaiDatBoard"
#define URL_GETThoiGian "https://App.IoTVision.vn/api/ThoiGian"

// Thiết lập thời gian timeout.
#define timeout 1000


// fixed max size for server response
#define _INSIZE 512

//=================================================================================
// Ví dụ, DuLieu = "1089;210;13.3;20:06:13 18/08/2023"
// 1089 => K1 = 1, MODE = 0 (MAN), RSSI WiFi = 89%
// Tốc độ lắc = 210 (RPM)
// Thời gian lắc = 13.3 (giờ)
// HH:MM:SS DD/MM/YYYY = 20:06:13 18/08/2023
//=================================================================================
void POSTGET::CaiIDPostGet(String ID)
{
    _ID = ID;
}

uint32_t POSTGET::LayRTCTuServerIoTVision()
{
    _http.begin(URL_GETThoiGian);
    _http.addHeader("Content-Type", "application/json");
    int httpCodeGet = _http.GET();
    if (httpCodeGet == HTTP_CODE_OK)
    {                           // needs a 200 to continue...
        int z = _http.getSize(); // how big is the payload?
        if (z < _INSIZE)
        { // payload must fit in the available space.
            String thoigianthuc = _http.getString();
            if (thoigianthuc.length() > 0)
            {
                // Chuyển chuỗi thoigianthuc sang dạng JSON
                JSONVar RTC = JSON.parse(thoigianthuc);

                // Chuyển RTC dạng JSON và kiểu dữ liệu
                // tự định nghĩa ThoiGian.
                //------------------------------------------------------
                // dt.Thu = (int)RTC["Thu"]; // CN = 0, Thứ 2 = 1,...
                // dt.Ngay = (int)RTC["Ngay"];
                // dt.Thang = (int)RTC["Thang"];
                // dt.Nam = (int)RTC["Nam"];
                // dt.Gio = (int)RTC["Gio"];
                // dt.Phut = (int)RTC["Phut"];
                // dt.Giay = (int)RTC["Giay"];

                // Lấy 2 số cuối của năm, vdu: 2020 thì chỉ lấy số 20.
                // dt.Nam -= 2000;
                //------------------------------------------------------
                DateTime dt((int)RTC["Nam"], (int)RTC["Thang"], (int)RTC["Ngay"], (int)RTC["Gio"], (int)RTC["Phut"], (int)RTC["Giay"]);
                _http.end();
                return dt.unixtime();
            }
        }
    }
    _http.end();
    return 0;
}

bool POSTGET::POSTDuLieuBoard(String DuLieu)
{
    if (_ID.isEmpty())
    {
        Serial.printf("\n\n\t _ID TRONG \n\n");
        return 0;
    }
    if (DuLieu.isEmpty())
    {
        Serial.printf("\n\n\t DuLieu TRONG \n\n");
    }
    try
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            // this->_http.setTimeout(timeout);  // Thiết lập thời gian timeout.
            this->_http.begin(URL_POSTDuLieuTuBoardLenServer);
            this->_http.addHeader("Content-Type", "application/json");
            String data = "{\"ID\":\"" + _ID + "\",\"S\":\"" + DuLieu + "\"}";
            this->_http.POST(data); // Send the request
            this->_http.end();

            return 1;
        }
        else
            return 0;
    }
    catch (String error)
    {
        return 0;
    }
}

bool POSTGET::POSTThongSoBoard(const String &DuLieu)
{
    if (_ID.isEmpty())
    {
        Serial.printf("\n\n\t _ID TRỐNG \n\n");
        return 0;
    }
    if (DuLieu.isEmpty())
    {
        Serial.printf("\n\n\t ThongSo TRỐNG \n\n");
    }
    try
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            // this->_http.setTimeout(timeout);  // Thiết lập thời gian timeout.
            this->_http.begin(URL_POSTGETThongSoBoard);
            this->_http.addHeader("Content-Type", "application/json");
            String data = "{\"ID\":\"" + _ID + "\",\"ThongSo\":\"" + DuLieu + "\"}";
            this->_http.POST(data); // Send the request
            this->_http.end();

            return 1;
        }
        else
            return 0;
    }
    catch (String error)
    {
        return 0;
    }
}

String POSTGET::GETLenhGuiXuongBoard()
{
    if (_ID.isEmpty())
    {
        Serial.printf("\n\n\t _ID TRỐNG \n\n");
        return "";
    }
    try
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            String url = URL_GETLenhGuiXuongBoard + _ID;
            // this->_http.setTimeout(timeout);  // Thiết lập thời gian timeout.
            this->_http.begin(url);
            this->_http.addHeader("Content-Type", "application/json");
#if defined(debug) && defined(POST_GET)
            unsigned long ms = millis();
#endif
            int httpCodeGet = this->_http.GET();
            if (httpCodeGet == HTTP_CODE_OK)
            { // needs a 200 to continue...
                String data = this->_http.getString();
                data.remove(0, 1);                 // Loại ký tự '[' tại đầu chuỗi.
                data.remove(data.length() - 1, 1); // Loại ký tự ']' tại cuối chuỗi.
                this->_http.end();

#if defined(debug) && defined(POST_GET)
                Serial.print(millis() - ms);
                Serial.print("(ms) for GET ");
                Serial.println(data);
#endif

                return data;
            }
            else
            {
#if defined(debug) && defined(POST_GET)
                Serial.println(this->_http.errorToString(httpCodeGet).c_str());
#endif
                this->_http.end();
                return "";
            }
        }
        else
        {
#if defined(debug) && defined(POST_GET)
            Serial.println("Mất kết nối WIFI!");
#endif
            return "";
        }
    }
    catch (String error)
    {
        return "";
    }
}

String POSTGET::GETThongSoBoard()
{
    if (_ID.isEmpty())
    {
        Serial.printf("\n\n\t _ID TRỐNG \n\n");
        return "";
    }
    try
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            String url = URL_POSTGETThongSoBoard + String("?ID=") + _ID;
            // this->_http.setTimeout(timeout);  // Thiết lập thời gian timeout.
            this->_http.begin(url);
            this->_http.addHeader("Content-Type", "application/json");
#if defined(debug) && defined(POST_GET)
            unsigned long ms = millis();
#endif
            int httpCodeGet = this->_http.GET();
            if (httpCodeGet == HTTP_CODE_OK)
            { // needs a 200 to continue...
                String data = this->_http.getString();
                data.remove(0, 1);                 // Loại ký tự '[' tại đầu chuỗi.
                data.remove(data.length() - 1, 1); // Loại ký tự ']' tại cuối chuỗi.
                this->_http.end();

#if defined(debug) && defined(POST_GET)
                Serial.print(millis() - ms);
                Serial.print("(ms) for GET ");
                Serial.println(data);
#endif

                return data;
            }
            else
            {
#if defined(debug) && defined(POST_GET)
                Serial.println(this->_http.errorToString(httpCodeGet).c_str());
#endif
                this->_http.end();
                return "";
            }
        }
        else
        {
#if defined(debug) && defined(POST_GET)
            Serial.println("Mất kết nối WIFI!");
#endif
            return "";
        }
    }
    catch (String error)
    {
        return "";
    }
}
//=================================================================================
// Ví dụ, DuLieu = "1011"
// K1 = 1, MODE = 0 (MAN), CODE = 11
//=================================================================================
void POSTGET::CapNhatCODETrongDatabaseTrenServer(String DuLieu)
{
    if (WiFi.status() == WL_CONNECTED)
    {
        // this->_http.setTimeout(timeout);  // Thiết lập thời gian timeout.
        // this->_http.addHeader("Content-Type", "application/json");
        this->_http.begin(URL_CapNhatCODE);
        this->_http.addHeader("Content-Type", "application/json");

        String data = "{\"ID\":\"" + _ID + "\",\"S\":\"" + DuLieu + "\"}";
        this->_http.POST(data); // Send the request
        this->_http.end();
    }
}

#else

#include <WiFi.h> // Thư viện WiFi của ESP32.

#define HOST "App.IoTVision.vn"
#define GET_PATH "/api/LABone_KingOven_DuLieuGuiXuongBoard?CheDo=1&key="
#define POST_PATH "/api/LABone_KingOven_DuLieu"
#define UPCODE_POST_PATH "/api/LABone_KingOven_DuLieuGuiXuongBoard"

#define HTTP_TIMEOUT 1500

POSTGET::POSTGET(void)
{
    client = &WF_Cli;
}

void POSTGET::SETLuongMang(http_stream_t stream)
{
    // this->stream = stream;
    // if (this->stream == HTTP_OVER_ETH) client = &ETH_Cli;
    // else client = &WF_Cli;
}

void POSTGET::Huy(void)
{
    client->stop();
}
//=================================================================================
// Ví dụ, DuLieu = "1089;210;13.3;20:06:13 18/08/2023"
// 1089 => K1 = 1, MODE = 0 (MAN), RSSI WiFi = 89%
// Tốc độ lắc = 210 (RPM)
// Thời gian lắc = 13.3 (giờ)
// HH:MM:SS DD/MM/YYYY = 20:06:13 18/08/2023
//=================================================================================
bool POSTGET::POSTDuLieuBoard(String _ID, String DuLieu)
{
#if defined(debug) && defined(DEBUG_POST_GET)
    if (_ID.isEmpty())
    {
        Serial.printf("\n\n\t _ID TRONG \n\n");
    }
    if (DuLieu.isEmpty())
    {
        Serial.printf("\n\n\t DuLieu TRONG \n\n");
    }
#endif
    if (WiFi.status() == WL_CONNECTED)
    {
        uint32_t time_start;
        String data = "{\"ID\": \"" + _ID + "\", \"S\": \"" + DuLieu + "\"}";

        String post_req = "POST " + String(POST_PATH) + " HTTP/1.1\r\n"
                                                        "Host: " +
                          String(HOST) + "\r\n"
                                         "Content-Type: application/json\r\n"
                                         "Content-Length: " +
                          String(data.length()) + "\r\n"
                                                  "Connection: close\r\n"
                                                  "\r\n" +
                          data + "\r\n";

        if (client->connect(HOST, 80))
        {
#if defined(debug) && defined(DEBUG_POST_GET)
            Serial.println("Connected to server, starting to post data!");
#endif
            client->print(post_req);
        }
        else
        {
#if defined(debug) && defined(DEBUG_POST_GET)
            Serial.println("connection failed");
#endif
            goto disconnect;
        }
        // #if defined(debug) && defined(DEBUG_POST_GET)
        time_start = millis();
        while (client->connected() && ((millis() - time_start) <= HTTP_TIMEOUT))
        {
            if (client->available())
            {
                char c = client->read();
                Serial.print(c);
            }
        }
        // #endif

        return 1;
    disconnect:
        client->stop();
#if defined(debug) && defined(DEBUG_POST_GET)
        Serial.println("Post data successful, Disconnected to server!");
#endif
        return 0;
    }
    return 0;
}

String POSTGET::GETLenhGuiXuongBoard(String _ID)
{
    if (WiFi.status() == WL_CONNECTED)
    {
        uint32_t time_start;
        int startIndex, endIndex;
        String post_req = "GET " + static_cast<String>(GET_PATH + _ID) + " HTTP/1.1\r\n"
                                                                         "Host: " +
                          static_cast<String>(HOST) + "\r\n"
                                                      "Content-Type: application/json\r\n"
                                                      "Connection: close\r\n"
                                                      "\r\n";
        String response_content;

        if (client->connect(HOST, 80))
        {
#if defined(debug) && defined(DEBUG_POST_GET)
            Serial.println("Connected to server, starting to get data!");
#endif
            client->print(post_req);
        }
        else
        {
#if defined(debug) && defined(DEBUG_POST_GET)
            Serial.println("connection failed");
#endif
            goto disconnect;
        }

        time_start = millis();
        while (client->connected() && ((millis() - time_start) <= HTTP_TIMEOUT))
        {
            if (client->readStringUntil('\n') == "\r")
                break;
        }
        time_start = millis();
        while (client->connected() && ((millis() - time_start) <= HTTP_TIMEOUT))
        {
            response_content += client->readStringUntil('\n');
        }

        startIndex = response_content.indexOf('['); //[{ nội dung   }]
        endIndex = response_content.indexOf(']');

        if (startIndex >= 0 && endIndex >= 0)
        {
            response_content = response_content.substring(startIndex + 1, endIndex);
        }
        else
        {
#if defined(debug) && defined(DEBUG_POST_GET)
            Serial.println("String error");
#endif
        }
    disconnect:
        client->stop();
#if defined(debug) && defined(DEBUG_POST_GET)
        Serial.print("dữ liệu nhận từ serve: ");
        Serial.println(response_content);
        Serial.println("Get data successful, Disconnected to server!");
#endif
        return response_content;
    }
    else
    {
#if defined(debug) && defined(DEBUG_POST_GET)
        Serial.println("Mất kết nối WIFI!");
#endif
        return "";
    }
    return "";
}

//=================================================================================
// Ví dụ, DuLieu = "1011"
// K1 = 1, MODE = 0 (MAN), CODE = 11
//=================================================================================
void POSTGET::CapNhatCODETrongDatabaseTrenServer(String _ID, String DuLieu)
{
    if (WiFi.status() == WL_CONNECTED)
    {
        uint32_t time_start;
        String data = "{\"ID\":\"" + _ID + "\",\"S\":\"" + DuLieu + "\"}";
        String post_req = "POST " + String(UPCODE_POST_PATH) + " HTTP/1.1\r\n"
                                                               "Host: " +
                          String(HOST) + "\r\n"
                                         "Content-Type: application/json\r\n"
                                         "Content-Length: " +
                          String(data.length()) + "\r\n"
                                                  "Connection: close\r\n"
                                                  "\r\n" +
                          data + "\r\n";

        if (client->connect(HOST, 80))
        {
#if defined(debug) && defined(DEBUG_POST_GET)
            Serial.println("Connected to server, starting to post data!");
#endif
            client->print(post_req);
        }
        else
        {
#if defined(debug) && defined(DEBUG_POST_GET)
            Serial.println("connection failed");
#endif
            goto disconnect;
        }
        // #if defined(debug) && defined(DEBUG_POST_GET)
        time_start = millis();
        while (client->connected() && ((millis() - time_start) <= HTTP_TIMEOUT))
        {
            if (client->available())
            {
                char c = client->read();
                Serial.print(c);
            }
        }
        // #endif

    disconnect:
        client->stop();
#if defined(debug) && defined(DEBUG_POST_GET)
        Serial.println("Post data successful, Disconnected to server!");
#endif
    }
}
#endif // CODE_POST_GET_CU
