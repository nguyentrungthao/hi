#ifndef _FILE_HANDLE_H_
#define _FILE_HANDLE_H_

#include "FS.h"
#include <SPIFFS.h>
#include <LittleFS.h>
#include <FFat.h>
#include <SD.h>
#include <SD_MMC.h>

class FileHandle {
public:
    // Constructor nhận tham số là một hệ thống tệp tùy chọn
    FileHandle(fs::FS &filesystem);

    // Khởi tạo hệ thống tệp
    bool begin();
    void end(void);

    // Thiết lập 1 hệ thống tệp khác 
    void setFileSystem(fs::FS &filesystem);

    // Các hàm đọc, ghi, xóa tệp
    bool writeFile(const char* path, const char* message);
    bool writeFile(const char* path, const uint8_t *buf, size_t size);
    bool appendFile(const char* path, const uint8_t *buf, size_t size);
    String readFile(const char* path);
    size_t readFile(const char* path, uint8_t *buf, size_t size);
    size_t readFile(const char* path, uint8_t *buf, size_t size, uint32_t pos);
    bool deleteFile(const char* path);
    size_t sizeFile(const char* path);
    bool exists(const char* path);
    void renameFile(const char* path, const char* newPath);
    time_t getLastWriteFile(const char* path);
    bool createDir(const char* newDir);
    int countFiles(const char* dir);
    // bool copyFileTo(fs::FS &filesystem, const char path);
private:
    fs::FS &fs; // Tham chiếu tới hệ thống tệp (có thể là SPIFFS, SD, LittleFS)
};

#endif