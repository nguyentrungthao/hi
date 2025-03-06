/*
 TODO:Incubator IR CO2 sensor library ver 0.0

 Product: https://www.co2meter.com/products/incubator-ir-co2-sensor

  Người viết: Nguyễn Trung Thảo

 */

#ifndef SENSOR_CO2_H
#define SENSOR_CO2_H

#include "Arduino.h"
// #include "../00_Config.h"
#include "cstring"
#include "cstdlib"

// #ifdef KING_INCU_CO2_BOARD

#define CO2_LIB_VERSION "ver 1.0"

/*
  define nếu muốn log trạng thái
  comment nếu không muốn
*/
// #define DEBUG_SERIAL_ENABLE

// uart cho RS232
// version bộ board
// #define _RS232_TX_PIN 17
// #define _RS232_RX_PIN 16

//version 5 trong 1
// #define _RS232_TX_PIN 33
// #define _RS232_RX_PIN 34

//version đồ án 
// #define _RS232_TX_PIN 19
// #define _RS232_RX_PIN 18

// version S3
#define _RS232_TX_PIN 21
#define _RS232_RX_PIN 47

#define TXPin _RS232_TX_PIN
#define RXPin _RS232_RX_PIN

#ifdef DEBUG_SERIAL_ENABLE
#define dbSerialPrint(X) _DBSerial->print(X)
#define dbSerialPrintln(X) _DBSerial->println(X)
#define dbSerialBegin(X) _DBSerial->begin(X)
#else
#define dbSerialPrint(X) \
  do { \
  } while (0)
#define dbSerialPrintln(X) \
  do { \
  } while (0)
#define dbSerialBegin(X) \
  do { \
  } while (0)
#endif

/* default value in datasheet */
// frame data of RS232 8N1
#define BAUD_RATE_DEFAULT 9600

typedef enum {
  IRCO2_OK = 0x00u,
  IRCO2_ERR_VAL = 0x01u,
  IRCO2_ERR = 0x02u
} IRCO2_StatusTydef;

class IRCO2 {
public:
  IRCO2(HardwareSerial* serial = &Serial1, HardwareSerial* DBSerial = &Serial);

  IRCO2_StatusTydef init(uint32_t serialBaudRate = BAUD_RATE_DEFAULT, uint8_t serialTXPin = TXPin, uint8_t serialRXPin = RXPin, uint32_t DBserialBaudRate = 0);
  void SendCommand(const char* str);
  IRCO2_StatusTydef GetSensorFeedback();

  IRCO2_StatusTydef ReadData();

  float GetCO2Concentration();
  float GetTemperature();
  float GetAirPresure();
  uint32_t GetSerialIDSensor();
  uint32_t GetTimeStamp();

  IRCO2_StatusTydef ZeroPointAdjustment(uint32_t ZeroPoint);
  IRCO2_StatusTydef ChangeBaudRate(uint32_t baudRate);
  IRCO2_StatusTydef SpanPointAdjustment(uint32_t value);

  IRCO2_StatusTydef HumidityCompensationH2OPartialPressure(uint8_t Humidity);
  IRCO2_StatusTydef HumidityCompensationrHAndTemperature(uint8_t relativeHumidity, uint16_t temperature);

  IRCO2_StatusTydef SensorReset();
  IRCO2_StatusTydef SetFactoryDefault();

protected:
  HardwareSerial* _serial, * _DBSerial;
  uint32_t _serialIDSensor, _timestamp;
  int16_t CO2Concentration, temperature, airPressure, returnValue;
  IRCO2_StatusTydef ProcessString(char* string);
  IRCO2_StatusTydef CheckReturn(uint32_t value);
};

// #endif

#endif  // kết thúc file SENSOR_CO2_H
