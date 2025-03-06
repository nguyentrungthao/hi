#include "06_SenSorCO2.h"
/* default value in datasheet */

// #ifdef KING_INCU_CO2_BOARD

#define STX 0x02  // bit start communication
#define ETX 0x03  // bit end communication

/* command for sensor */
#define READ_DATA "1100"
#define CHANGE_BAURD_RATE "1302"
#define SENSOR_RESET "1908"
#define FACTORY_RESET "5005"
/* command for sensor */


IRCO2::IRCO2(HardwareSerial* serial, HardwareSerial* DBSerial) {
  this->_serial = serial;
  this->_DBSerial = DBSerial;
  this->CO2Concentration = -500;
  this->temperature = -200;
  this->airPressure = 800;
  this->returnValue = -1;
}

IRCO2_StatusTydef IRCO2::init(uint32_t serialBaudRate, uint8_t serialTXPin, uint8_t serialRXPin, uint32_t DBserialBaudRate) {
  char _buffer[40] = {};
  uint8_t i = 0, _try = 5;
  IRCO2_StatusTydef result = IRCO2_ERR;

  this->_serial->begin(serialBaudRate, SERIAL_8N1, serialRXPin, serialTXPin);

  if (DBserialBaudRate > 0) {
    dbSerialBegin(DBserialBaudRate);
  }
  // delay before communication
  delay(3);
  // be sure get ID sensor
  while (_try--) {
    //get ID sensor
    this->SendCommand((const char*)READ_DATA);
    delay(1000);

    while (_serial->available() > 0) {
      _buffer[i++] = _serial->read();
    }
    if (_buffer[i - 1] == ETX) {
      char* token = strtok(_buffer + 1, " ");
      _serialIDSensor = atoi(token);

      result = IRCO2_OK;
      dbSerialPrint("\nID Sensor: ");
      dbSerialPrintln(_serialIDSensor);
      dbSerialPrintln("init IRCO2 successfull");
      break;
    }
  }

  //delay before get data
  delay(10);

  return result;
}

void IRCO2::SendCommand(const char* str) {
  this->_serial->write(STX);
  this->_serial->print(str);
  this->_serial->write(ETX);
}

IRCO2_StatusTydef IRCO2::ReadData() {
  this->SendCommand((const char*)READ_DATA);
  return this->GetSensorFeedback();
}

IRCO2_StatusTydef IRCO2::GetSensorFeedback() {
  static char _buffer[40] = {};
  IRCO2_StatusTydef result = IRCO2_ERR;
  uint16_t i = 0;
  uint8_t c;

  while (_serial->available() > 0) {
    c = _serial->read();
    _buffer[i++] = c;
  }
  // dbSerialPrint("data CO2");
  // dbSerialPrintln(_buffer);
  if (_buffer[i - 1] == ETX) {
    result = this->ProcessString(_buffer);
  }
  memset(_buffer, 0, 40);
  return result;
}

float IRCO2::GetCO2Concentration() {
  float resultValue = this->CO2Concentration / 1000.0;
  this->CO2Concentration = -1;
  return resultValue;
}
float IRCO2::GetTemperature() {
  float resultValue = this->temperature / 10.0;
  this->temperature = -1;
  return resultValue;
}
float IRCO2::GetAirPresure() {
  float resultValue = this->airPressure;
  this->airPressure = -1;
  return resultValue;
}

uint32_t IRCO2::GetSerialIDSensor() {
  return this->_serialIDSensor;
}
uint32_t IRCO2::GetTimeStamp() {
  return this->_timestamp;
}

IRCO2_StatusTydef IRCO2::ZeroPointAdjustment(uint32_t ZeroPoint) {
  // wait 15 minute for const thermal and atmosphe 
  dbSerialPrintln("wait 15 minute for const thermal and atmosphe");
  // delay(15 * 60 * 1000);

  char cmd[15] = {};

  sprintf(cmd, "1203%ld\0", ZeroPoint);

  dbSerialPrint(cmd);
  this->SendCommand(cmd);

  return CheckReturn(0);
}
IRCO2_StatusTydef IRCO2::ChangeBaudRate(uint32_t baudRate) {

  char cmd[5] = { 0x31, 0x33, 0x30, 0x32, 0x00 };

  switch (baudRate) {
  case 57600:
    cmd[4] = 0x01;
    break;
  case 38400:
    cmd[4] = 0x02;
    break;
  case 19200:
    cmd[4] = 0x03;
    break;
  case 9600:
    cmd[4] = 0x04;
    break;
  case 4800:
    cmd[4] = 0x05;
    break;
  case 2400:
    cmd[4] = 0x06;
    break;
  }

  dbSerialPrintln(cmd);
  this->SendCommand(cmd);

  return CheckReturn(0);
}
IRCO2_StatusTydef IRCO2::SpanPointAdjustment(uint32_t value) {
  char cmd[15] = {};

  sprintf(cmd, "1405%d\0", value);

  dbSerialPrint(cmd);
  this->SendCommand(cmd);

  return CheckReturn(value);
}

IRCO2_StatusTydef IRCO2::HumidityCompensationH2OPartialPressure(uint8_t Humidity) {
  char cmd[10] = {};

  sprintf(cmd, "1706%d\0", Humidity);

  dbSerialPrintln(cmd);
  this->SendCommand(cmd);

  return CheckReturn(Humidity);
}
IRCO2_StatusTydef IRCO2::HumidityCompensationrHAndTemperature(uint8_t relativeHumidity, uint16_t temperature) {
  char cmd[15] = {};
  sprintf(cmd, "1809%d %d\0", relativeHumidity, temperature);

  dbSerialPrintln(cmd);
  this->SendCommand(cmd);


  return ((CheckReturn(relativeHumidity) == IRCO2_OK) && (CheckReturn(temperature) == IRCO2_OK)) ? IRCO2_OK : IRCO2_ERR;
}

IRCO2_StatusTydef IRCO2::SensorReset() {
  this->SendCommand(SENSOR_RESET);
  delay(15); // delay before read data
  return CheckReturn(0);
}
IRCO2_StatusTydef IRCO2::SetFactoryDefault() {
  this->SendCommand(FACTORY_RESET);
  delay(15); // delay before read data
  return CheckReturn(0);
}

IRCO2_StatusTydef IRCO2::ProcessString(char* string) {
  char* localString = string + 1;
  char* token = strtok(localString, " ");
  if (token != NULL) {
    // check string from sensor have IDsensor
    uint32_t IDSensor = atoi(token);
    if (IDSensor == _serialIDSensor) {
      uint8_t index = 0;
      token = strtok(NULL, " ");
      while (token != NULL) {
        switch (index) {
        case 0:
          _timestamp = atoi(token);
          index = 1;
          break;
        case 1:
          CO2Concentration = atoi(token);
          index = 2;
          break;
        case 2:
          temperature = atoi(token);
          index = 3;
          break;
        case 3:
          airPressure = atoi(token);
          break;
        }
        token = strtok(NULL, " ");
      }
      returnValue = 0;
    }
    else {
      returnValue = atoi(token);
    }
    return IRCO2_OK;
  }
  return IRCO2_ERR;
}

IRCO2_StatusTydef IRCO2::CheckReturn(uint32_t value) {
  if (value == returnValue) {
    returnValue = -1;
    return IRCO2_OK;
  }
  return IRCO2_ERR;
}

// #endif
