// Đường dẫn tải Firmware
const char* jsonUrl = "https://app.iottest.io.vn/firmware/checkforupdate/CheckForUpdates.json";

// Đường dẫn thư mục lưu firmware
const char* UPDATE_DIR = "/Update";

// Hàm in thông tin tải firmware lên HMI
#define HMI_PRINT(X) _dwin.HienThiThongTinUpdate(X)

// Hàm so sánh ngày tháng
bool compareDates(const String& date1, const String& date2) {
    int year1 = date1.substring(0, 2).toInt();
    int month1 = date1.substring(3, 5).toInt();
    int day1 = date1.substring(6, 8).toInt();

    int year2 = date2.substring(0, 2).toInt();
    int month2 = date2.substring(3, 5).toInt();
    int day2 = date2.substring(6, 8).toInt();

    // So sánh năm trước
    if (year1 != year2) {
        return year1 < year2; // So sánh năm
    }
    // Nếu năm bằng nhau, so sánh tháng
    if (month1 != month2) {
        return month1 < month2; // So sánh tháng
    }
    // Nếu tháng bằng nhau, so sánh ngày
    return day1 < day2; // So sánh ngày
}

// Hàm chuyển đổi Date biên dịch sang số hiệu version
String convertDateToVersion(const char* dateStr) {
    static const char* months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    int day, year;
    char monthStr[4];
    sscanf(dateStr, "%s %d %d", monthStr, &day, &year);

    int month = -1;
    for (int i = 0; i < 12; ++i) {
        if (strcmp(monthStr, months[i]) == 0) {
            month = i + 1;
            break;
        }
    }

    // Định dạng YY.MM.DD từ day, month, year
    char formattedDate[9];
    snprintf(formattedDate, sizeof(formattedDate), "%02d.%02d.%02d", year % 100, month, day);
    return String(formattedDate);
}

// Hàm kiểm tra phiên bản mới
bool isNewVersion(const String& newVersion) {
    // So sánh ngày của phiên bản mới với phiên bản hiện tại
    String currentVersion = convertDateToVersion(__DATE__);
    return compareDates(currentVersion, newVersion);
}

bool downloadUpdatesFromJson(fs::FS &fs) {

    HTTPClient http;
    http.begin(jsonUrl);
    int httpCode = http.GET();

    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("Failed to download JSON, HTTP code: %d\n", httpCode);
        HMI_PRINT("Failed to download JSON");
        return false;
    }

    // Đọc dữ liệu JSON
    String jsonData = http.getString();
    http.end();

    // Phân tích JSON
    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, jsonData);
    if (error) {
        Serial.print("Failed to parse JSON: ");
        Serial.println(error.c_str());
        return false;
    }

    const char* newVersion = doc["version"];
    Serial.println("Check version -> " + String(newVersion));
    HMI_PRINT("Check for updates !");
    if (!isNewVersion(String(newVersion))) {
        Serial.println("No new version available.");
        HMI_PRINT("No new version available.");
        return false;
    }

    JsonArray updates = doc["Updates"];
    bool allSuccess = true;

    for (JsonObject update : updates) {
        const char* name = update["Name"];
        String directory = String(UPDATE_DIR) + "/" + name;

        // Tạo thư mục nếu chưa có
        // if (!fs.exists(directory.c_str())) {
        //     fs.mkdir(directory.c_str());
        // }
        const char* directoryPath = directory.c_str();
        if (!fs.exists(directoryPath)) {
          if (strchr(directoryPath, '/')) {
            Serial.printf("Create missing folders of: %s\r\n", directoryPath);
            char *pathStr = strdup(directoryPath);
            if (pathStr) {
              char *ptr = strchr(pathStr, '/');
              while (ptr) {
                *ptr = 0;
                fs.mkdir(pathStr);
                *ptr = '/';
                ptr = strchr(ptr + 1, '/');
              }
            }
            free(pathStr);
          }
        }
    
        JsonArray urls = update["URLs"];

        for (const char* url : urls) {
            if(String(url).compareTo("") == 0) {
                continue;
            }
            String fileName = String(url).substring(String(url).lastIndexOf('/') + 1);
            String localFilePath = directory + "/" + fileName;
            HMI_PRINT("Downloading: " + fileName);
            if (!downloadFile(fs, url, localFilePath.c_str())) {
                Serial.printf("Failed to download file: %s\n", localFilePath.c_str());
                HMI_PRINT("Error downloading updates.");
                allSuccess = false;
            }
        }
    }

  return allSuccess;
}

// Hàm tải file từ URL và lưu vào đường dẫn filePath
bool downloadFile(fs::FS &fs, const char* url, const char* filePath) {
  HTTPClient http;
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("HTTP error while downloading %s: %d\n", filePath, httpCode);
    http.end();
    return false;
  }

  File file = fs.open(filePath, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file on SD card");
    http.end();
    return false;
  }

  WiFiClient* stream = http.getStreamPtr();
  int total_len = http.getSize();
  int downloaded_len = 0;

  uint8_t buff[128];
  unsigned long lastProgressUpdate = 0;

  while (http.connected() && (total_len < 0 || downloaded_len < total_len)) {
    size_t size = stream->available();
    if (size) {
      int c = stream->readBytes(buff, (size > sizeof(buff) ? sizeof(buff) : size));
      file.write(buff, c);

      downloaded_len += c;

      unsigned long currentTime = millis();
      if (currentTime - lastProgressUpdate >= 1000) {
        int progress = (total_len > 0) ? (downloaded_len * 100 / total_len) : 0;
        Serial.printf("Downloading %s: %d%%\n", filePath, progress);
        String fileName = String(filePath).substring(String(filePath).lastIndexOf('/') + 1);
        HMI_PRINT(fileName + ": " + String(progress) + "%");
        lastProgressUpdate = currentTime;
        delay(1);
      }
    }
  }

  file.close();
  http.end();

  if (total_len > 0 && downloaded_len < total_len) {
    Serial.printf("Incomplete download for file: %s\n", filePath);
    fs.remove(filePath); // Xóa file không hoàn chỉnh
    return false;
  }

  Serial.printf("File downloaded successfully: %s\n", filePath);
  return true;
}

// Hàm xóa tất cả các file trong thư mục cập nhật sau khi update hoàn tất
void clearUpdateFolder(fs::FS &fs, const char* base_dir) {
  File root = fs.open(base_dir);
  if (!root) {
    Serial.println("Failed to open update folder");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      fs.remove(file.path());
      Serial.printf("Deleted: %s\n", file.path());
    }
    file = root.openNextFile();
  }
}
