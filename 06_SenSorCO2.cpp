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
  delay(10);
  // be sure get ID sensor
  while (_try--) {
    if (ReadData() == IRCO2_OK) {
      //delay before get data
      delay(10);
      return IRCO2_OK;
    }
    delay(1000);
  }
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
  String str;
  delay(100);
  // Serial.print("data: ");
  while (_serial->available() > 0) {
    _buffer[i++] = _serial->read();
    delay(10);
  }
  // uint8_t length = str.length();
  // memcpy(_buffer, str.c_str(), length);
  // Serial.println(str);
  if (_buffer[0] == STX && _buffer[i - 1] == ETX) {
    result = this->ProcessString(_buffer);
  }
  else {
    ESP_LOGE(__FILE__,"CO2 sai frame %x %x\n", _buffer[0], _buffer[i - 1]);
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
  delay(15);  // delay before read data
  return CheckReturn(0);
}
IRCO2_StatusTydef IRCO2::SetFactoryDefault() {
  this->SendCommand(FACTORY_RESET);
  delay(15);  // delay before read data
  return CheckReturn(0);
}

IRCO2_StatusTydef IRCO2::ProcessString(char* string) {
  char* localString = string + 1;

  //* chuỗi mẫu trả về khi ra lệnh đọc cảm biến
  /*STX7 12345 1200 376 980ETX
    Decoded string : SensorID = 7
    Timestamp = 12345 => / 2 => 6172.5 s => 1.7 h
    CO2 concentration = 1.2 Vol. - %
    Sensor temperature = 37.6 °C
    Air pressure = 980 hPa*/

    //* chuỗi mẫu trả về khi ra lệnh cài thông số
    /*STX0ETX (adjustment successful)*/

    //tách số đầu tiên
  char* token = strtok(localString, " ");
  if (token == NULL) {
    return IRCO2_ERR;
  }
  // Serial.print(token);
  uint32_t firstReturnToken = atoi(token);

  // tách số thứ 2, nếu có chuỗi tức là lệnh đọc cảm biến, nếu không là giá trị phản hồi, 0 1 hoặc giá trị cài
  token = strtok(NULL, " ");
  // Serial.print(token);
  if (token == NULL) {
    returnValue = firstReturnToken;
    return IRCO2_OK;
  }

  uint8_t index = 0;
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
    // Serial.print(token);
  }
  returnValue = 0;
  return IRCO2_OK;
}

IRCO2_StatusTydef IRCO2::CheckReturn(uint32_t value) {
  if (value == returnValue) {
    returnValue = -1;
    return IRCO2_OK;
  }
  return IRCO2_ERR;
}

// #endif
