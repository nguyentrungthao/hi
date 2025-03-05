#include "FileHandle.h"

// Constructor, khởi tạo với hệ thống tệp tùy chọn (SPIFFS, SD, hoặc LittleFS)
FileHandle::FileHandle(fs::FS &filesystem) : fs(filesystem) {};

// Khởi tạo hệ thống tệp (chỉ cần với SPIFFS và LittleFS, SD tự động mount)
bool FileHandle::begin() {
    
    if (&fs == &SPIFFS) {
        if (!SPIFFS.begin(true)) {
            log_e("%s: Failed to mount file system", __func__);
            return false;
        }
    } 
    else if (&fs == &LittleFS) 
    {
        if (!LittleFS.begin(true)) {
            log_e("%s:Failed to mount LittleFS", __func__);
            return false;
        }
        else {
            log_i("%s:LittleFS mounted", __func__);
        }
    }
    else if (&fs == &SD) {
        if (!SD.begin()) {
            log_e("%s:Failed to initialize SD card", __func__);
            return false;
        }
    }
    else if(&fs == &FFat) {
        if (!FFat.begin()) {
            log_e("%s:Failed to mount file system", __func__);
            return false;
        }
    }
    return true;
}

// Thiết lập 1 hệ thống tệp khác 
void FileHandle::setFileSystem(fs::FS &filesystem)
{
    fs = filesystem;
}

void FileHandle::end(void)
{
    if (&fs == &SPIFFS) {
        SPIFFS.end();
    } 
    else if (&fs == &LittleFS) 
    {
        LittleFS.end();
    }
    else if (&fs == &SD) {
        SD.end();
    }
    else if (&fs == &FFat) {
        FFat.end();
    }
    else {
        log_e("%s: fail", __func__);
    }
}

// Ghi dữ liệu vào tệp
bool FileHandle::writeFile(const char* path, const char* message) {
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
    if (!file) {
        log_e("Failed to open file for writing");
        return false;
    }
    file.print(message);
    file.close();
    return true;
}

bool FileHandle::writeFile(const char* path, const uint8_t *buf, size_t size)
{
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
    if (!file) {
        log_e("Failed to open file for writing");
        return false;
    }
    file.write(buf, size);
    file.close();
    return true;
}

bool FileHandle::appendFile(const char* path, const uint8_t *buf, size_t size)
{
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
    File file = fs.open(path, FILE_APPEND);
    if (!file) {
        log_e("Failed to open file for writing");
        return false;
    }
    file.write(buf, size);
    file.close();
    return true;
}
// Đọc dữ liệu từ tệp
String FileHandle::readFile(const char* path) {
    File file = fs.open(path, FILE_READ);
    if (!file) {
        log_e("Failed to open file for reading");
        return "";
    }
    String content = "";
    while (file.available()) {
        content += (char)file.read();
    }
    file.close();
    return content;
}

size_t FileHandle::readFile(const char* path, uint8_t *buf, size_t size)
{
    File file = fs.open(path, FILE_READ);
    if (!file) {
        log_e("Failed to open file for reading");
        return 0;
    }
    size_t length = file.read(buf, size);
    if(length == -1) 
    {
        return 0;
    }
    file.close();
    return length;
}

size_t FileHandle::readFile(const char* path, uint8_t *buf, size_t size, uint32_t pos)
{
    File file = fs.open(path, FILE_READ);
    if (!file) {
        log_e("Failed to open file for reading");
        return 0;
    }
    file.seek(pos);
    size_t length = file.read(buf, size);
    if(length == -1) 
    {
        return 0;
    }
    file.close();
    return length;
}

// Xóa tệp
bool FileHandle::deleteFile(const char* path) {
    if (!fs.remove(path)) {
        log_e("Failed to delete file");
        return false;
    }
    return true;
}

size_t FileHandle::sizeFile(const char* path) {
    File file = fs.open(path, FILE_READ);
    if (!file) {
        log_e("%s: Failed", __func__);
        return 0;
    }
    size_t size = file.size();
    file.close();
    return size;
}

bool FileHandle::exists(const char* path)
{
    return fs.exists(path);
}

void FileHandle::renameFile(const char* path, const char* newPath)
{
    fs.rename(path, newPath);
}

time_t FileHandle::getLastWriteFile(const char* path)
{
    File file = fs.open(path, FILE_READ);
    time_t lastWrite = file.getLastWrite();
    file.close();
    return lastWrite;
}

bool FileHandle::createDir(const char* newDir)
{
    if (!fs.exists(newDir)) {
        if (fs.mkdir(newDir)) {
            Serial.println("Directory '" + String(newDir) + "' created successfully");
        } 
        else {
            Serial.println("Failed to create directory '" + String(newDir) + "'");
            return false;
        }
    } 
    else {
        Serial.println("Directory '" + String(newDir) + "' already exists");
    }
    return true;
}

int FileHandle::countFiles(const char* dir)
{
    File root = fs.open(dir); // Mở thư mục gốc hoặc thư mục cần đếm
    if (!root) {
        Serial.println("Không thể mở thư mục");
        return -1; // Trả về -1 nếu không thể mở thư mục
    }
    
    if (!root.isDirectory()) {
        Serial.println("Đường dẫn không phải là thư mục");
        return -1; // Trả về -1 nếu đường dẫn không phải thư mục
    }
    
    File file = root.openNextFile();
    int fileCount = 0;

    // Duyệt qua tất cả các tệp trong thư mục
    while (file) {
        if (!file.isDirectory()) { // Nếu là tệp (không phải thư mục con)
            fileCount++;
        }
        file.close();
        file = root.openNextFile(); // Mở tệp tiếp theo
    }
    root.close();
    return fileCount; // Trả về số lượng tệp
}
