#include "RTC_Timer.h"

// Constructor
RTC_Timer::RTC_Timer()
    : duration(0), isCountingDown(false), isRunning(false), elapsedSeconds(0) {}

// Hàm khởi tạo RTC
void RTC_Timer::begin()
{
  if (!RTC_DS3231::begin())
  {
    Serial.println("RTC not found!");
    while (1)
      ;
  }
  if (lostPower())
  {
    adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

// Lấy thời gian hiện tại từ RTC và cập nhật currentTime
DateTime RTC_Timer::getCurrentTime()
{
  currentTime = now(); // Gọi trực tiếp từ RTC_DS3231
  return currentTime;
}

// Cập nhật thời gian trên RTC
void RTC_Timer::updateRTC(DateTime newTime)
{
  DateTime oldTime = now();

  // Điều chỉnh thời gian Timer nếu RTC thay đổi
  if (isRunning)
  {
    time_t offset = (newTime - oldTime).totalseconds();
    startTime = startTime + TimeSpan(offset);
  }

  adjust(newTime); // Cập nhật thời gian mới
}

// Các hàm lấy giá trị thời gian
uint8_t RTC_Timer::getSecond()
{
  return currentTime.second();
}

uint8_t RTC_Timer::getMinute()
{
  return currentTime.minute();
}

uint8_t RTC_Timer::getHour()
{
  return currentTime.hour();
}

uint8_t RTC_Timer::getDay()
{
  return currentTime.day();
}

uint8_t RTC_Timer::getMonth()
{
  return currentTime.month();
}

int RTC_Timer::getYear()
{
  return currentTime.year();
}

// Timer functions
void RTC_Timer::start(time_t seconds, bool countDown)
{
  startTime = now();
  duration = seconds;
  isCountingDown = countDown;
  isRunning = true;
  elapsedSeconds = 0;
}

void RTC_Timer::stop()
{
  if (isRunning)
  {
    elapsedSeconds += (now() - startTime).totalseconds();
    isRunning = false;
  }
}

void RTC_Timer::reset()
{
  isRunning = false;
  elapsedSeconds = 0;
}

time_t RTC_Timer::getElapsedTime()
{
  if (isRunning)
  {
    return elapsedSeconds + (now() - startTime).totalseconds();
  }
  else
  {
    return elapsedSeconds;
  }
}

time_t RTC_Timer::getRemainingTime()
{
  if (isCountingDown)
  {
    time_t elapsed = getElapsedTime();
    return (elapsed < duration) ? (duration - elapsed) : 0;
  }
  return 0;
}

bool RTC_Timer::isFinished()
{
  return isCountingDown && (getRemainingTime() == 0);
}

bool RTC_Timer::isTimerRunning()
{
  return isRunning;
}

