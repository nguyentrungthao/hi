
static int fileIndex = 0;                      // Chỉ số file hiện tại
static const int MAX_FILE_INDEX = 30;          // Giới hạn tối đa fileIndex
static const int32_t MAX_RECORDS_PER_FILE = 1048576; // Giới hạn số lượng bản ghi mỗi file
// const int MAX_RECORDS_PER_FILE = 1440; // Giới hạn số lượng bản ghi mỗi file
static int recordCount = 0; // Số lượng bản ghi đã ghi trong file hiện tại
#define LOG_DIR_PATH "/log"
#define LOG_STATE_DIR_PATH "/state"

// Kiểm tra và cập nhật fileIndex và recordCount khi hệ thống khởi động
void initLogFileState(fs::FS& fs)
{
  // Tải trạng thái từ file lưu trữ trạng thái
  char filename[20];
  snprintf(filename, sizeof(filename), "%s/state.bin", LOG_STATE_DIR_PATH);
  File stateFile = fs.open(filename, FILE_READ);
  if (stateFile)
  {
    stateFile.read((uint8_t*)&fileIndex, sizeof(fileIndex));
    stateFile.read((uint8_t*)&recordCount, sizeof(recordCount));
    stateFile.close();
    Serial.printf("Resuming from saved state: fileIndex=%d, recordCount=%d\n", fileIndex, recordCount);

    // Kiểm tra lại fileIndex và recordCount để đảm bảo dữ liệu hợp lệ
    snprintf(filename, sizeof(filename), "%s/data%d.csv", LOG_DIR_PATH, fileIndex);
    if (fs.exists(filename))
    {
      File file = fs.open(filename, FILE_READ);
      if (file)
      {
        int fileSize = file.size();
        int recordsInFile = fileSize;
        if (recordsInFile == recordCount && recordCount < MAX_RECORDS_PER_FILE)
        {
          return; // Nếu dữ liệu khớp, sử dụng trạng thái đã lưu
        }
      }
    }
  }

  // Nếu không có file trạng thái hợp lệ, thực hiện kiểm tra các file
  fileIndex = 0;
  recordCount = 0;
  for (int i = 0; i <= MAX_FILE_INDEX; i++)
  {
    snprintf(filename, sizeof(filename), "%s/data%d.csv", LOG_DIR_PATH, i);
    if (fs.exists(filename))
    {
      File file = fs.open(filename, FILE_READ);
      if (file)
      {
        int fileSize = file.size();
        int recordsInFile = fileSize;

        if (recordsInFile < MAX_RECORDS_PER_FILE)
        {
          // File này vẫn chưa đầy, tiếp tục ghi vào file này
          fileIndex = i;
          recordCount = recordsInFile;
          file.close();
          Serial.printf("Resuming from file: %s with %d records\n", filename, recordCount);
          return;
        }
        file.close();
      }
    }
  }

  // Tất cả các file đều đầy hoặc không có file nào, bắt đầu từ file đầu tiên
  fileIndex = 0;
  recordCount = 0;
  Serial.println("All files are full or no files found, starting from index 0");
}

// Lưu trạng thái sau mỗi lần ghi thành công
void saveState(fs::FS& fs)
{
  char filename[20];
  snprintf(filename, sizeof(filename), "%s/state.bin", LOG_STATE_DIR_PATH);

  if (!fs.exists(filename))
  {
    if (strchr(filename, '/'))
    {
      Serial.printf("Create missing folders of: %s\r\n", filename);
      char* pathStr = strdup(filename);
      if (pathStr)
      {
        char* ptr = strchr(pathStr, '/');
        while (ptr)
        {
          *ptr = 0;
          fs.mkdir(pathStr);
          *ptr = '/';
          ptr = strchr(ptr + 1, '/');
        }
      }
      free(pathStr);
    }
  }

  File stateFile = fs.open(filename, FILE_WRITE);
  if (stateFile)
  {
    stateFile.write((const uint8_t*)&fileIndex, sizeof(fileIndex));
    stateFile.write((const uint8_t*)&recordCount, sizeof(recordCount));
    stateFile.close();
    Serial.printf("State saved: fileIndex=%d, recordCount=%d\n", fileIndex, recordCount);
  }
}

// Xóa file cũ nếu tồn tại trước khi ghi file mới
void deleteExistingFile(fs::FS& fs, int index)
{
  char filename[20];
  snprintf(filename, sizeof(filename), "%s/data%d.csv", LOG_DIR_PATH, index);
  if (fs.exists(filename))
  {
    fs.remove(filename); // Xóa file cũ
    Serial.printf("Deleted existing file: %s\n", filename);
  }
}

// Ghi bản ghi và lưu trạng thái
void writeRecord(fs::FS& fs, RecordData_t record)
{
  char filename[20];
  char strData[200];
  snprintf(filename, sizeof(filename), "%s/data%d.csv", LOG_DIR_PATH, fileIndex);

  if (!fs.exists(LOG_DIR_PATH))
  {
    if (fs.mkdir(LOG_DIR_PATH))
    {
      Serial.println("Directory '/log' created successfully");
    }
    else
    {
      Serial.println("Failed to create directory '/log'");
      return;
    }
  }

  // Mở file để ghi
  File file;
  if (!fs.exists(filename))
  {
    strcpy(strData, "Time(D/M/Y H:M:S),TempSetpoint,Temperature(oC),CO2Setpoint,CO2Concentration(%),Fan(%)\n");
    file = fs.open(filename, FILE_APPEND);
    if (file)
    {
      file.write((const uint8_t*)strData, strlen(strData));
      file.close();
    }
    else
    {
      Serial.println("Failed to open file for writing");
      return;
    }
  }

  file = fs.open(filename, FILE_APPEND);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }


  sprintf(strData, "%02u/%02u/%4u %02u:%02u:%02u,%.1f,%.1f,%0.1f,%0.1f,%u\n",
    record.day, record.month, record.year + 2000, record.hour, record.minute, record.second,
    record.setpointTemp, record.valueTemp, record.setpointCO2, record.valueCO2, record.fan);
  recordCount += file.write((const uint8_t*)strData, strlen(strData));

  file.close();
  // Cập nhật trạng thái

  Serial.printf("%s: fileIndex: %d recordCount: %d byte\n", __func__, fileIndex, recordCount);
  // saveState(fs); // Lưu trạng thái mới sau khi ghi

  // Kiểm tra nếu file đã đầy
  if (recordCount >= MAX_RECORDS_PER_FILE)
  {
    fileIndex = (fileIndex + 1) % (MAX_FILE_INDEX + 1); // Xoay vòng fileIndex
    recordCount = 0;                                    // Reset lại số lượng bản ghi
    deleteExistingFile(fs, fileIndex);                  // Xóa file cũ nếu cần
    saveState(fs);                                      // Lưu trạng thái mới sau khi xoay file
  }
}

uint32_t getDirectorySize(fs::FS& fs, const char* path)
{
  uint32_t totalSize = 0;
  File root = fs.open(path);
  if (!root || !root.isDirectory())
  {
    Serial.printf("Failed to open directory: %s\n", path);
    return 0;
  }

  File file = root.openNextFile();
  while (file)
  {
    totalSize += file.size(); // Thêm kích thước của file hiện tại
    file.close();
    file = root.openNextFile(); // Mở file tiếp theo
  }
  root.close();
  return totalSize;
}

void copyFiles(fs::FS& destFS, fs::FS& srcFS)
{
  char strData[64];
  char strPath[50];
  time_t fileLastTime;

  File dir = srcFS.open(LOG_DIR_PATH);
  if (!dir || !dir.isDirectory())
  {
    Serial.println(String(__func__) + ": Thư mục " + String(LOG_DIR_PATH) + " không tồn tại trên srcFS");
    _dwin.HienThiPhanTramThanhLoading("Fail");
    _dwin.HienThiThanhLoading(0);
    return;
  }

  uint32_t dirSize = getDirectorySize(srcFS, LOG_DIR_PATH);

  // Sao chép nội dung từ filesystem sang USB
  uint32_t totalSize = 0;

  File srcFile = dir.openNextFile();
  while (srcFile)
  {
    String fileName = srcFile.name();
    fileLastTime = srcFile.getLastWrite();
    sprintf(strPath, "%s_%02u%02u%02u_%02u%02u%02u.csv", LOG_DIR_PATH, day(fileLastTime), month(fileLastTime), year(fileLastTime) % 2000, hour(fileLastTime), minute(fileLastTime), second(fileLastTime));

    Serial.print("Đang sao chép file: ");
    Serial.println(fileName);

    // Mở file trên USB để ghi

    File destFile = destFS.open(strPath, FILE_WRITE);
    if (!destFile)
    {
      Serial.println(String(__func__) + "Không thể mở file trên USB");
      srcFile.close();
      _dwin.HienThiPhanTramThanhLoading("Fail");
      _dwin.HienThiThanhLoading(0);
      return;
    }

    // strcpy(strData, "Time(D/M/Y H:M:S),Setpoint,Temperature,Fan(%),Flap(%),\n");
    // destFile.write((const uint8_t*)strData, strlen(strData));
    while (srcFile.available())
    {
      uint8_t data[8];
      uint8_t dataLen = 0;
      dataLen = srcFile.read((uint8_t*)data, sizeof(data));
      destFile.write((const uint8_t*)data, dataLen);
      totalSize += dataLen;
      _dwin.HienThiThanhLoading(totalSize * 99 / dirSize);
      _dwin.HienThiPhanTramThanhLoading(totalSize * 100 / dirSize);
      if (totalSize % 800 == 0)
      {
        delay(1);
      }
    }

    // Đóng file
    destFile.close();
    srcFile.close();

    // Mở file tiếp theo
    srcFile = dir.openNextFile();
  }
  dir.close();
  _dwin.HienThiPhanTramThanhLoading("Done");
  Serial.println("Hoàn tất sao chép các file.");
}