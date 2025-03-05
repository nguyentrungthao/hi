
// Định nghĩa các trạng thái
const char* UPDATE_STARTED   = "started";
const char* UPDATE_COMPLETED = "completed";
const char* FIRMWARE_PATH    = "/Update/Firmware/firmware.bin";
const char* HMI_PATH         = "/Update/HMI";

// Thư mục lưu trạng thái
const char* FIRMWARE_UPDATE_STATUS  = "/UpdateStatus/firmware_status.txt";
const char* HMI_UPDATE_STATUS       = "/UpdateStatus/hmi_status.txt";

// Hàm in thông tin Update lên HMI
#define HMI_PRINT(X) _dwin.HienThiThongTinUpdate(X)

// Hàm ghi trạng thái cập nhật vào file
void writeUpdateStatus(fs::FS &fs, const char* path, const char* status) {
    
    if (!fs.exists(path)) {
      if (strchr(path, '/')) {
        Serial.printf("Create missing folders of: %s\r\n", path);
        char *pathStr = strdup(path);
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

    File file = fs.open(path, FILE_WRITE);
    if (file) {
        file.println(status);
        file.close();
        Serial.printf("%s: %s", __func__, status);
    }
}

// Hàm đọc trạng thái cập nhật từ file
String readUpdateStatus(fs::FS &fs, const char* path) {
    if (fs.exists(path)) {
        File file = fs.open(path, FILE_READ);
        if (file) {
            String status = file.readStringUntil('\n');
            file.close();
            return status;
        }
    }
    else {
      Serial.printf("Không mở được %s\n", path);
    }
    return ""; // Trả về chuỗi trống nếu file không tồn tại hoặc không đọc được
}

// Hàm cập nhật firmware
void updateFirmware(fs::FS &fs) {
    writeUpdateStatus(fs, FIRMWARE_UPDATE_STATUS, UPDATE_STARTED); // Đánh dấu bắt đầu
    Serial.println("\nUpdating Firmware...");

    /* Thực hiện cập nhật firmware tại đây */
    HMI_PRINT("Updating firmware!");
    updateFromFS(fs);

    writeUpdateStatus(fs, FIRMWARE_UPDATE_STATUS, UPDATE_COMPLETED); // Đánh dấu hoàn thành
    Serial.println("\nFirmware Update Completed.");
    HMI_PRINT("\nFirmware Update Completed.");
    delay(1000);

}

// Hàm cập nhật HMI
void updateHMI(fs::FS &fs) {
    writeUpdateStatus(fs, HMI_UPDATE_STATUS, UPDATE_STARTED); // Đánh dấu bắt đầu
    Serial.println("\nUpdating HMI...");

    /* Thực hiện cập nhật HMI tại đây */
    HMI_PRINT("Updating HMI!");
    if(_dwin.updateHMI(fs, HMI_PATH))
    {
        Serial.println("UpdateHMI-->OK");
        clearUpdateFolder(fs, HMI_PATH);
    }
    else {
        Serial.println("UpdateHMI-->Fail");
        return;
    }

    writeUpdateStatus(fs, HMI_UPDATE_STATUS, UPDATE_COMPLETED); // Đánh dấu hoàn thành
    // Serial.println("HMI Update Completed.");
}

// Hàm kiểm tra và tiếp tục cập nhật nếu cần
void checkAndResumeUpdates(fs::FS &fs) {
    bool flagReset = false;
    if(readUpdateStatus(fs, FIRMWARE_UPDATE_STATUS).indexOf(UPDATE_STARTED) >= 0) {
        Serial.println("Resuming Firmware Update...");
        updateFirmware(fs);
        flagReset = true;
    }
    else {
        Serial.println("Firmware is up to date.");
    }

    if(readUpdateStatus(fs, HMI_UPDATE_STATUS).indexOf(UPDATE_STARTED) >= 0) {
        Serial.println("Resuming HMI Update...");
        updateHMI(fs);
        flagReset = true;
    }
    else {
        Serial.println("HMI is up to date.");
    }
    if(flagReset) {
        ESP.restart();
    }
}

// perform the actual update from a given stream
void performUpdate(Stream &updateSource, size_t updateSize) {
  if (Update.begin(updateSize)) {
    size_t written = Update.writeStream(updateSource);
    if (written == updateSize) {
      Serial.println("Written : " + String(written) + " successfully");
    } else {
      Serial.println("Written only : " + String(written) + "/" + String(updateSize) + ". Retry?");
    }
    if (Update.end()) {
      Serial.println("OTA done!");
      if (Update.isFinished()) {
        Serial.println("Update successfully completed. Rebooting.");
      } else {
        Serial.println("Update not finished? Something went wrong!");
      }
    } else {
      Serial.println("Error Occurred. Error #: " + String(Update.getError()));
    }

  } else {
    Serial.println("Not enough space to begin OTA");
  }
}

// check given FS for valid update.bin and perform update if available
void updateFromFS(fs::FS &fs) {
  File updateBin = fs.open(FIRMWARE_PATH);
  if (updateBin) {
    if (updateBin.isDirectory()) {
      Serial.println("Error, " + String(FIRMWARE_PATH) + " is not a file");
      updateBin.close();
      return;
    }

    size_t updateSize = updateBin.size();

    if (updateSize > 0) {
      Serial.println("Try to start update");
      performUpdate(updateBin, updateSize);
    } 
    else {
      Serial.println("Error, file is empty");
    }

    updateBin.close();

    // when finished remove the binary from sd card to indicate end of the process
    // fs.remove(filePath);
    fs.rename(FIRMWARE_PATH, (String(FIRMWARE_PATH)+".done").c_str());
  } 
  else {
    Serial.println("Could not load " + String(FIRMWARE_PATH) + " from sd root");
  }
}
