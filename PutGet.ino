#include <WiFi.h>
#include <HTTPClient.h>
#define ARDUINOJSON_USE_LONG_LONG 1
#include <ArduinoJson.h>
#include <TimeLib.h>

// const char* ssid = "LABone:KingLab-HUYlab";
// const char* password = "66668888";
const char* firebaseHost = "https://angelic-bazaar-312410-default-rtdb.firebaseio.com";  // URL của Firebase
const char* authToken = "AAAA7Tz7Pi8:APA91bGRqR70tLBlE9ZfKXYoDt6JOQX8XQyPO7_eGZ-02MGmiPL34id2fvlVcCAlmW6ssGpIRIjJj4zt-OEEzCfL897K2YoHcu4dzdrFyT-inNTSDlEnUXfTKGwSvdbkOwKY_Asb9xrZ";  // Token Firebase

// Biến để điều khiển yêu cầu POST và GET
bool doPost = true;

void connectWiFi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    // Tạo task httpTask để xử lý POST và GET tuần tự
    xTaskCreate(httpTask, "httpTask", 5120, NULL, 5, NULL);
}

// Hàm POST JSON lên Firebase
float Index = 0;
void sendPostRequest() {
    HTTPClient http;
    String url = String(firebaseHost) + "/data.json";

    // Tạo dữ liệu JSON để gửi
    StaticJsonDocument<200> jsonDoc;
    jsonDoc["name"] = "AnhIncu";
    jsonDoc["state"] = BaseProgram.machineState;
    jsonDoc["setpoint"] = BaseProgram.programData.setPoint;
    jsonDoc["value"] = BaseProgram.temperature;
    jsonDoc["flap"] = BaseProgram.programData.flap;
    jsonDoc["fan"] = BaseProgram.programData.fanSpeed;
    jsonDoc["time"] = now();

    String jsonString;
    serializeJson(jsonDoc, jsonString);

    // Thiết lập URL và Header
    http.begin(url);
    http.addHeader("Authorization", String("key=") + authToken);
    http.addHeader("Content-Type", "application/json");

    // Gửi yêu cầu POST
    int httpResponseCode = http.PUT(jsonString);

    if (httpResponseCode > 0) {
        Serial.print("POST Response Code: ");
        Serial.println(httpResponseCode);
        Serial.print("Response: ");
        Serial.println(http.getString());
    } else {
        Serial.print("Error on sending POST: ");
        Serial.println(http.errorToString(httpResponseCode).c_str());
    }
    http.end();
}

// Hàm GET JSON từ Firebase
void sendGetRequest() {
    HTTPClient http;
    String url = String(firebaseHost) + "/data.json";

    http.begin(url);
    http.addHeader("Authorization", String("key=") + authToken);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.print("GET Response Code: ");
        Serial.println(httpResponseCode);
        Serial.print("Response: ");
        Serial.println(response);

        // Xử lý dữ liệu JSON nhận được
        StaticJsonDocument<200> jsonDoc;
        DeserializationError error = deserializeJson(jsonDoc, response);
        bool machineState = false;
        float setPoint;
        float flap;
        float fanSpeed;
        hmi_set_event_t event;
        if (!error) {
            const char* name = jsonDoc["name"];
            machineState = jsonDoc["state"];
            setPoint = jsonDoc["setpoint"];
            flap = jsonDoc["flap"];
            fanSpeed = jsonDoc["fan"];

            Serial.print("Name: ");
            Serial.println(name);
            Serial.print("State: ");
            Serial.println(machineState);
            if(machineState != BaseProgram.machineState) {
                event.type = HMI_SET_RUN_ONOFF;
                hmiSetEvent(event);
                _dwin.HienThiIconTrangThaiRun(BaseProgram.machineState);
            }

            Serial.print("flap: ");
            Serial.println(BaseProgram.programData.flap);
            if(flap != BaseProgram.programData.flap)
            {
                event.type = HMI_SET_FLAP;
                event.f_value = flap;
                hmiSetEvent(event);
                _dwin.HienThiGocFlap(BaseProgram.programData.flap);
            }

            Serial.print("fan: ");
            Serial.println(BaseProgram.programData.fanSpeed); 
            if(fanSpeed != BaseProgram.programData.fanSpeed)
            {
                event.type = HMI_SET_FAN;
                event.f_value = fanSpeed;
                hmiSetEvent(event);
                _dwin.HienThiTocDoQuat(BaseProgram.programData.fanSpeed);
            }
            Serial.print("setpoint: ");
            Serial.println(setPoint); 
            if(setPoint != BaseProgram.programData.setPoint)
            {
                event.type = HMI_SET_SETPOINT;
                event.f_value = setPoint;
                hmiSetEvent(event);
                _dwin.HienThiSetpoint(BaseProgram.programData.setPoint);
            }         

        } else {
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.f_str());
        }
    } else {
        Serial.print("Error on sending GET: ");
        Serial.println(http.errorToString(httpResponseCode).c_str());
    }
    http.end();
}

// Task để quản lý POST và GET tuần tự
void httpTask(void *parameter) {
    uint8_t count = 0;
    while (true) {
        if (WiFi.status() == WL_CONNECTED) {
            if (doPost) {
                if(count % 10 == 0) {
                    sendPostRequest();
                    count = 0;
                } 
            } else {
                sendGetRequest();
            }
            count++;
            // Đổi trạng thái để lần tiếp theo sẽ thực hiện GET nếu vừa POST và ngược lại
            doPost = !doPost;  
        } else {
            Serial.println("WiFi Disconnected");
            connectWiFi();
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);  // Thực hiện lại mỗi 500 mili giây
    }
}

// void setup() {
//     Serial.begin(115200);
//     connectWiFi();


// }

// void loop() {
//     // Không làm gì trong loop()
// }
