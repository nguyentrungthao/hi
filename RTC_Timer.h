#ifndef RTC_TIMER_H
#define RTC_TIMER_H

#include <Wire.h>
#include <RTClib.h>

class RTC_Timer : public RTC_DS3231 {
private:
  DateTime currentTime;      // Thời gian hiện tại

  // Timer variables
  DateTime startTime;
  time_t duration;
  bool isCountingDown;
  bool isRunning;
  time_t elapsedSeconds;

public:
  // Constructor
  RTC_Timer();

  // Hàm khởi tạo RTC
  void begin();

  // Lấy thời gian hiện tại từ RTC và cập nhật currentTime
  DateTime getCurrentTime();

  // Cập nhật thời gian trên RTC
  void updateRTC(DateTime newTime);

  // Các hàm lấy giá trị thời gian
  uint8_t getSecond();  // Giây
  uint8_t getMinute();  // Phút
  uint8_t getHour();    // Giờ
  uint8_t getDay();     // Ngày
  uint8_t getMonth();   // Tháng
  int getYear();        // Năm

  // Timer functions
  void start(time_t seconds, bool countDown = false);
  void stop();
  void reset();
  time_t getElapsedTime();
  time_t getRemainingTime();
  bool isFinished();
  bool isTimerRunning();
};

#endif
