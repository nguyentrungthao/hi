#include "DWIN_custom.h"
#include <stdio.h>

#define CMD_HEAD1 0x5A
#define CMD_HEAD2 0xA5
#define CMD_WRITE 0x82
#define CMD_READ 0x83

#define MIN_ASCII 32
#define MAX_ASCII 255

#define CMD_READ_TIMEOUT 100
#define READ_TIMEOUT 100
#define READ_TIMEOUT_UPDATE 10000

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(FORCEHWSERIAL)
DWIN::DWIN(HardwareSerial& port, long baud) {
  port.begin(baud, SERIAL_8N1);
  init((Stream*)&port, false);
}

#elif defined(ARDUINO_ARCH_RP2040)
DWIN::DWIN(HardwareSerial& port, long baud, bool initSerial) {
  if (initSerial) {
    port.begin(baud, SERIAL_8N1);
    init((Stream*)&port, false);
  }
}

#elif ARDUINO_ARCH_STM32
DWIN::DWIN(HardwareSerial& port) {
  init((Stream*)&port, false);
}

#elif defined(ESP32)
DWIN::DWIN(HardwareSerial& port, uint8_t receivePin, uint8_t transmitPin, long baud)
  : listenerCallback(NULL),
  _baudrate(baud),
  _rxPin(receivePin),
  _txPin(transmitPin) {
  port.begin(baud, SERIAL_8N1, receivePin, transmitPin);
  init((HardwareSerial*)&port);
}

#elif defined(ESP8266)
DWIN::DWIN(uint8_t receivePin, uint8_t transmitPin, long baud) {
  localSWserial = new SoftwareSerial(receivePin, transmitPin);
  localSWserial->begin(baud);
  init((Stream*)localSWserial, true);
}
DWIN::DWIN(SoftwareSerial& port, long baud) {
  port.begin(baud);
  init((Stream*)&port, true);
}
DWIN::DWIN(Stream& port, long baud) {
  init(&port, true);
}

#else
DWIN::DWIN(uint8_t rx, uint8_t tx, long baud) {
  localSWserial = new SoftwareSerial(rx, tx);
  localSWserial->begin(baud);
  _baud = baud;
  init((Stream*)localSWserial, true);
}

#endif

#if defined(ESP32)
void DWIN::init(HardwareSerial* port) {
  this->_dwinSerial = port;
}
#else
void DWIN::init(Stream* port, bool isSoft) {
  this->_dwinSerial = port;
  this->_isSoft = isSoft;
}
#endif

void DWIN::echoEnabled(bool echoEnabled) {
  _echo = echoEnabled;
}
void DWIN::returnWord(bool retWord) {
  _retWord = retWord;
}

void DWIN::ackDisabled(bool noACK) {
  _noACK = noACK;
}

size_t DWIN::dwinWrite(const uint8_t* sendBuffer, size_t size) {
  size_t returnValue;
  if (sendBuffer) {
    _wait_for_respone = true;
    returnValue = _dwinSerial->write(sendBuffer, size);
  }
  return returnValue;
}

// Get Hardware Firmware Version of DWIN HMI
double DWIN::getHWVersion() {  //  HEX(5A A5 04 83 00 0F 01)
  uint8_t dataLen = 0x04;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_READ, 0x00, 0x0F, 0x01 };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  return readCMDLastByte();
  // return readCMDLastByteEvent();
}

double DWIN::getGUISoftVersion() {  //  HEX(5A A5 04 83 00 0F 01)
  uint8_t dataLen = 0x04;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_READ, 0x00, 0x0F, 0x01 };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  return readCMDLastByte(1);
  // return readCMDLastByteFromEvent(1);
}

// Restart DWIN HMI
void DWIN::restartHMI() {  // HEX(5A A5 07 82 00 04 55 aa 5a a5 )
  uint8_t dataLen = 0x07;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_WRITE, 0x00, 0x04, 0x55, 0xaa, CMD_HEAD1, CMD_HEAD2 };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  delay(10);
  readDWIN();
}

// SET DWIN Brightness
void DWIN::setBrightness(uint8_t brightness) {
  uint8_t dataLen = 0x04;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_WRITE, 0x00, 0x82, brightness };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  readDWIN();
}

// GET DWIN Brightness
uint8_t DWIN::getBrightness() {
  uint8_t dataLen = 0x04;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_READ, 0x00, 0x31, 0x01 };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  return readCMDLastByte();
  // return readCMDLastByteFromEvent();

}

// Change Page
void DWIN::setPage(uint8_t page) {
  // 5A A5 07 82 00 84 5a 01 00 02
  // uint8_t sendBuffer[3+dataLen] = {CMD_HEAD1, CMD_HEAD2, 0x07, CMD_WRITE, 0x00, 0x84, 0x5A, 0x01, 0x00, page};
  uint8_t dataLen = 7;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_WRITE, 0x00, 0x84, 0x5A, 0x01, 0x00, page };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  // Serial.println();
  // for(int i=0; i<sizeof(sendBuffer); i++) {
  //     Serial.print(String(sendBuffer[i], HEX) + " ");
  // }
  // Serial.println();
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  readDWIN();
}
// Play a sound
void DWIN::playSound(uint8_t soundID) {
  // 5A A5 07 82 00 A0 soundID 01 40 00
  uint8_t dataLen = 7;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_WRITE, 0x00, 0xA0, soundID, 0x01, 0x40, 0x00 };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  readDWIN();
}

// Get Current Page ID
uint8_t DWIN::getPage() {
  uint8_t dataLen = 0x04;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_READ, 0x00, 0x14, 0x01 };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  clearSerial();
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  return readCMDLastByte();
  // return readCMDLastByteFromEvent();

}
// set the hardware RTC The first two digits of the year are automatically added
void DWIN::setRTC(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
  // 5A A5 0B 82 00 9C 5A A5 year month day hour minute second
  uint8_t dataLen = 0x0B;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_WRITE, 0x00, 0x9C, 0x5A, 0xA5, year, month, day, hour, minute, second };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  readDWIN();
}
// upHMI_DATE the software RTC The first two digits of the year are automatically added
void DWIN::setRTCSOFT(uint8_t year, uint8_t month, uint8_t day, uint8_t weekday, uint8_t hour, uint8_t minute, uint8_t second) {
  // 5A A5 0B 82 0010 year month day weekday(0-6 0=Sunday) hour minute second 00
  uint8_t dataLen = 0x0B;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_WRITE, 0x00, 0x10, year, month, day, weekday, hour, minute, second, 0x00 };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  readDWIN();
}
// Set Text on VP Address
//5A A5 0F 82 80 00 41 42 20 20 20 41 42 FF FF AF FF 41 42 FF FF 01 20
void DWIN::setText(long address, String textData) {
  uint8_t ffEnding[2] = { 0xFF, 0xFF };
  int dataLen = textData.length();
  uint8_t dataCMD[dataLen];
  textData.getBytes(dataCMD, dataLen + 1);
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[8 + dataLen];
  uint8_t startCMD[] = { CMD_HEAD1, CMD_HEAD2, (uint8_t)(dataLen + 5), CMD_WRITE,
                         (uint8_t)((address >> 8) & 0xFF), (uint8_t)((address) & 0xFF) };

  memcpy(sendBuffer, startCMD, sizeof(startCMD));
  memcpy(sendBuffer + 6, dataCMD, sizeof(dataCMD));
  memcpy(sendBuffer + (6 + sizeof(dataCMD)), ffEnding, 2);  // add ending 0xFFFF
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  // for (int i = 0; i < sizeof(sendBuffer); i++) {
  //     Serial.print(sendBuffer[i], HEX);
  //     Serial.print(" ");  // In ra khoảng trắng giữa các byte
  // }
  // Serial.println();  // Xuống dòng sau khi in hết sendBuffer

  dwinWrite(sendBuffer, sizeof(sendBuffer));
  readDWIN();
}
// Set Byte Data on VP Address makes more sense alias of below
void DWIN::setVPByte(long address, uint8_t data) {
  setVP(address, data);
}
void DWIN::setVP(long address, uint8_t data) {
  // 0x5A, 0xA5, 0x05, 0x82, 0x40, 0x20, 0x00, state
  uint8_t dataLen = 0x05;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_WRITE, (uint8_t)((address >> 8) & 0xFF), (uint8_t)((address) & 0xFF), 0x00, data };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  readDWIN();
}

// Set WordData on VP Address
void DWIN::setVPWord(long address, int data) {
  // 0x5A, 0xA5, 0x05, 0x82, hiVPaddress, loVPaddress, hiData, loData
  uint8_t dataLen = 0x05;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_WRITE, (uint8_t)((address >> 8) & 0xFF), (uint8_t)((address) & 0xFF), (uint8_t)((data >> 8) & 0xFF), (uint8_t)((data) & 0xFF) };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  readDWIN();
}

// read WordData from VP Address you can read sequential multiple words returned in rx event
void DWIN::readVPWord(long address, uint8_t numWords) {
  // 0x5A, 0xA5, 0x04, 0x83, hiVPaddress, loVPaddress, 0x01 (1 vp to read)
  uint8_t dataLen = 0x04;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_READ, (uint8_t)((address >> 8) & 0xFF), (uint8_t)((address) & 0xFF), numWords };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
}

// read uint8_t from VP Address (if hiByte = true read HiByte of word)
uint8_t DWIN::readVPByte(long address, bool hiByte) {
  // 0x5A, 0xA5, 0x04, 0x83, hiVPaddress, loVPaddress, 0x01)
  uint8_t dataLen = 0x04;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_READ, (uint8_t)((address >> 8) & 0xFF), (uint8_t)((address) & 0xFF), 0x1 };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  return readCMDLastByte(hiByte);
  // return readCMDLastByteFromEvent(hiByte);
}

// read or write the NOR from/to VP must be on a even address 2 words are written or read
void DWIN::norReadWrite(bool write, long VPAddress, long NORAddress) {
  uint8_t readWrite;
  (write) ? (readWrite = 0xa5) : (readWrite = 0x5a);
  uint8_t dataLen = 0x0B;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_WRITE, 0x00, 0x08, readWrite, (uint8_t)((NORAddress >> 16) & 0xFF), (uint8_t)((NORAddress >> 8) & 0xFF),
                                      (uint8_t)((NORAddress) & 0xFF), (uint8_t)((VPAddress >> 8) & 0xFF), (uint8_t)((VPAddress) & 0xFF), 0x00, 0x02 };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  readDWIN();
  delay(30);  // DWIN Docs say - appropriate delay - is this it?
}
// beep Buzzer for 1 Sec
void DWIN::beepHMI(int time) {
  // 1s = 1000ms/8ms = 125 = 0x7D
  // 0x5A, 0xA5, 0x05, 0x82, 0x00, 0xA0, 0x00, 0x7D
  if (time < 8) {
    time = 8;
  }
  else {
    time = time / 8;
  }
  uint8_t dataLen = 0x05;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_WRITE, 0x00, 0xA0, (uint8_t)((time >> 8) & 0xFF), (uint8_t)((time) & 0xFF) };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  readDWIN();
}
// set text color (16-bit RGB) on controls which allow it ie. text control.
// changes the control sp address space (sp=description pointer) content see the DWIN docs.
void DWIN::setTextColor(long spAddress, long spOffset, long color) {
  // 0x5A, 0xA5, 0x05, hi spAddress, lo spAddress, hi color, lo color
  spAddress = (spAddress + spOffset);
  uint8_t dataLen = 0x05;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_WRITE, (uint8_t)((spAddress >> 8) & 0xFF), (uint8_t)((spAddress) & 0xFF), (uint8_t)((color >> 8) & 0xFF), (uint8_t)((color) & 0xFF) };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  readDWIN();
}

// set float value to 32bit DATA Variable
void DWIN::setFloatValue(long vpAddress, float fValue) {
  uint8_t hx[4] = { 0 };
  uint8_t* new_bytes = reinterpret_cast<uint8_t*>(&fValue);
  memcpy(hx, new_bytes, 4);
  uint8_t dataLen = 0x07;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_WRITE, (uint8_t)((vpAddress >> 8) & 0xFF), (uint8_t)((vpAddress) & 0xFF), hx[3], hx[2], hx[1], hx[0] };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  readDWIN();
}

// Send array to the display we dont need the 5A A5 or
// the size uint8_t hopefully we can work this out.
bool DWIN::sendArray(uint8_t dwinSendArray[], uint8_t arraySize) {
  uint8_t dataLen = arraySize;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen };
  memcpy(sendBuffer + 3, dwinSendArray, arraySize);
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  clearSerial();
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  flushSerial();
  // look for the ack. on write
  if (dwinSendArray[0] == CMD_WRITE) {
    if (readDWIN().indexOf("Fail") >= 0) {
      return 0;
    }
  }
  return 1;
}
/**
 *@brief Send int array to the display we dont need the 5A A5 or size - words only
 *
 * @param instruction lệnh ghi hay đọc
 * @param dwinIntArray mảng dữ liệu
 * @param arraySize số lượng phần tử trong mảng * 2 (vì mỗi phần tử là 1 word)
 */
 //
void DWIN::sendIntArray(uint16_t instruction, uint16_t dwinIntArray[], uint8_t arraySize) {

  // turn our int array to array of bytes
  uint8_t j = 0;
  uint8_t dwinSendByteArray[arraySize];
  for (int i = 0; i < (arraySize >> 1); i++) {
    dwinSendByteArray[j] = (uint8_t)((dwinIntArray[i] >> 8) & 0xFF);
    j++;
    dwinSendByteArray[j] = (uint8_t)((dwinIntArray[i]) & 0xFF);
    j++;
  }
  uint8_t dataLen = arraySize;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[4 + sizeof(dwinSendByteArray) + (dataLen - arraySize)] = { CMD_HEAD1, CMD_HEAD2, (uint8_t)((dataLen + 1)), (uint8_t)((instruction) & 0xFF) };
  memcpy(sendBuffer + 4, dwinSendByteArray, sizeof(dwinSendByteArray));
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  // Serial.print("Sending Array: ");
  // for(uint8_t i = 0; i < sizeof(sendBuffer); i++) {
  //     Serial.print(String(sendBuffer[i], HEX) + " ");
  // }
  // Serial.println();

  dwinWrite(sendBuffer, sizeof(sendBuffer));

  // look for the ack. on write
  if ((uint8_t)((instruction) & 0xFF) == CMD_WRITE) {  // or some others?
    readDWIN();
  }
}

// SET CallBack Event
void DWIN::hmiCallBack(hmiListener callBack) {
  listenerCallback = callBack;
}

// Listen For incoming callback  event from HMI
void DWIN::listen() {
  handle();
}

String DWIN::readDWIN() {
  flushSerial();
  String resp = "";
  if (_noACK) {
    return resp;  // using no response kernel
  }

  unsigned long startTime = millis();  // Start time for Timeout

  while ((millis() - startTime < READ_TIMEOUT)) {
    while (_dwinSerial->available() > 0) {
      int c = _dwinSerial->read();
      if (c < 0x10) {
        resp.concat(" 0" + String(c, HEX));
      }
      else {
        resp.concat(" " + String(c, HEX));
      }
      if (_dwinSerial->available() <= 0) {
        delayMicroseconds(100);
        if (_dwinSerial->available() <= 0) {
          if (_echo) {
            Serial.println("->> " + resp);
          }
          _wait_for_respone = false;
          return resp;
        }
        // break;
      }
    }
  }
  _wait_for_respone = false;
  if (_echo) {
    Serial.println("->> Fail. Time out");
  }
  return "Fail";
}

String DWIN::checkHex(uint8_t currentNo) {
  if (currentNo < 0x10) {
    return "0" + String(currentNo, HEX);
  }
  return String(currentNo, HEX);
}
// Hàm tính CRC-16 Modbus
uint16_t DWIN::calculateCRC(uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;  // Giá trị khởi tạo CRC

  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];  // XOR dữ liệu với CRC hiện tại

    for (uint8_t j = 0; j < 8; ++j) {  // Xử lý từng bit
      if (crc & 0x01) {
        crc >>= 1;      // Dịch phải
        crc ^= 0xA001;  // XOR với hằng số
      }
      else {
        crc >>= 1;  // Chỉ dịch phải
      }
    }
  }

  return crc;  // Trả về CRC tính toán
}

String DWIN::handle() {
  int lastBytes;
  int previousByte;
  String response;
  String address;
  String message;
  bool isSubstr = false;
  bool messageEnd = false;
  bool isFirstByte = false;

  if (_wait_for_respone) {
    return "";
  }
  while (_dwinSerial->available() > 0) {
    int inhex;
    if (!(_dwinSerial->read() == 0x5A && _dwinSerial->read() == 0xA5)) return "";

    inhex = _dwinSerial->read();  // độ dài chuỗi dữ liệu

    uint8_t cmd = _dwinSerial->read();
    switch (cmd) {
    case 0x83:
      isFirstByte = true;
      message = "";
      address = "";
      response = "";
      response.concat("5A A5 " + checkHex(inhex - 2) + " 83 ");

      if (_rawData.size() > 0) {
        _rawData.clear();
      }
      _rawData.push_back(0x83);
      for (int i = 2; i <= inhex; i++) {
        _rawData.push_back((uint8_t)_dwinSerial->read());
      }
      if (_crc) {
        uint32_t dataSize = _rawData.size();
        uint16_t receivedCRC;
        // Lấy 2 byte cuối của dữ liệu nhận được
        memcpy((uint8_t*)&receivedCRC, _rawData.data() + (dataSize - 2), 2);
        uint16_t calculatedCRC = calculateCRC(_rawData.data(), dataSize - 2);
        if (receivedCRC == calculatedCRC) {
          Serial.println("Serial check -> OK");
        }
        else {
          Serial.println("Serial check -> Fail");
          return "";
        }
      }
      break;
    case 0x82:
      return readDWIN();
      break;
    default:
    {
      String retunString(readCMDLastByteEvent(inhex, cmd));
      return retunString;
    }
    break;
    }

    int inhex2 = inhex;
    if (_crc) {
      inhex2 -= 2;
    }
    for (int i = 2; i <= inhex2; i++) {
      int inByte = _rawData[i - 1];
      if (i == (inhex2 - 1)) {
        previousByte = inByte;
      }
      response.concat(checkHex(inByte) + " ");
      if (i <= 3) {
        if ((i == 2) || (i == 3)) {
          address.concat(checkHex(inByte));
        }
        continue;
      }
      else {
        if (messageEnd == false) {
          if (isSubstr && inByte != MAX_ASCII && inByte >= MIN_ASCII) {
            message += char(inByte);
          }
          else {
            if (inByte == MAX_ASCII) {
              messageEnd = true;
            }
            isSubstr = true;
          }
        }
      }
      lastBytes = inByte;
    }
  }

  if (isFirstByte) {
    if (_retWord)
      lastBytes = (previousByte << 8) + lastBytes;
    if (listenerCallback != NULL) {
      listenerCallback(address, lastBytes, message, response);
    }

    EventInfo info;
    info.vpAddr = strtol(address.c_str(), NULL, 16);
    info.lastBytes = lastBytes;

    for (HmiEvent eventElemen : _eventList) {
      switch (eventElemen.eventType) {
      case DWIN_BUTTON:
        if (eventElemen.vpAddr == info.vpAddr && eventElemen.lastBytes == 0xffff) {
          eventElemen.callBack.buttonEvent(lastBytes, eventElemen.args);
          delay(1);
        }
        else if (eventElemen.vpAddr == info.vpAddr && eventElemen.lastBytes == info.lastBytes) {
          eventElemen.callBack.buttonEvent(lastBytes, eventElemen.args);
          delay(1);
        }
        break;
      case DWIN_TEXT:
        if (eventElemen.vpAddr == info.vpAddr) {
          eventElemen.callBack.textReceivedEvent(message, eventElemen.args);  // truyen text nhan duoc
        }
        break;
      default:
        Serial.println("The event has not been registered");
        break;
      }
    }
  }
  if (isFirstByte && _echo) {
    Serial.println("Address :0x" + address + " | Data :0x" + String(lastBytes, HEX) + " | Message : " + message + " | Response " + response);
    Serial.println("_rawData: ");
    for (uint8_t data : _rawData) {
      Serial.print(String(data, HEX) + " ");
    }
    Serial.println();
  }
  return response;
}

uint8_t DWIN::readCMDLastByteEvent(uint8_t length, uint8_t cmd) {
  flushSerial();
  unsigned long startTime = millis();  // Start time for Timeout
  if (_rawData.size() > 0 && _crc) {
    _rawData.clear();
  }
  while ((millis() - startTime < CMD_READ_TIMEOUT)) {
    if (_dwinSerial->available() > 0) {
      int inHex = length;  // Độ dài data
      for (uint8_t i = 1; i < inHex; i++) {
        _previousByte = _lastByte;
        _lastByte = _dwinSerial->read();
        _rawData.push_back(_lastByte);
      }
      if (_crc) {
        uint32_t dataSize = _rawData.size();
        uint16_t receivedCRC;
        // Lấy 2 byte cuối của dữ liệu nhận được
        memcpy((uint8_t*)&receivedCRC, _rawData.data() + dataSize - 2, 2);
        uint16_t calculatedCRC = calculateCRC(_rawData.data(), dataSize - 2);
        if (receivedCRC == calculatedCRC) {
          _previousByte = _rawData[dataSize - 4];
          _lastByte = _rawData[dataSize - 3];
          Serial.printf("Lastbyte: %u\n", _lastByte);
        }
        else {
          Serial.println("CRC incorrect");
        }
      }
      _rawData.clear();
      break;
    }
  }
}

uint8_t DWIN::readCMDLastByteFromEvent(bool hiByte) {
  delay(100);
  uint8_t lastByte = _lastByte;
  uint8_t previousByte = _previousByte;
  _lastByte = -1;
  _previousByte = -1;
  if (hiByte) {
    return _previousByte;
  }
  else {
    return _lastByte;
  }
}

uint8_t DWIN::readCMDLastByte(bool hiByte) {
  flushSerial();
  uint8_t lastByte = -1;
  uint8_t previousByte = -1;
  unsigned long startTime = millis();  // Start time for Timeout
  if (_rawData.size() > 0 && _crc) {
    _rawData.clear();
  }
  while ((millis() - startTime < CMD_READ_TIMEOUT)) {
    if (_dwinSerial->available() > 0) {
      if (_dwinSerial->read() == CMD_HEAD1 && _dwinSerial->read() == CMD_HEAD2) {
        int inHex = _dwinSerial->read();  // Độ dài data
        for (uint8_t i = 0; i < inHex; i++) {
          previousByte = lastByte;
          lastByte = _dwinSerial->read();
          _rawData.push_back(lastByte);
        }
        if (_crc) {
          uint32_t dataSize = _rawData.size();
          uint16_t receivedCRC;
          // Lấy 2 byte cuối của dữ liệu nhận được
          memcpy((uint8_t*)&receivedCRC, _rawData.data() + dataSize - 2, 2);
          uint16_t calculatedCRC = calculateCRC(_rawData.data(), dataSize - 2);
          if (receivedCRC == calculatedCRC) {
            previousByte = _rawData[dataSize - 4];
            lastByte = _rawData[dataSize - 3];
            // Serial.printf("Lastbyte: %u\n", lastByte);
          }
          else {
            Serial.println("CRC incorrect");
          }
        }
        _rawData.clear();
        break;
      }
      else {
        continue;
      }
    }
  }
  _wait_for_respone = false;

  if (hiByte) {
    return previousByte;
  }
  else {
    return lastByte;
  }
}

void DWIN::flushSerial() {
  Serial.flush();
  _dwinSerial->flush();
}

void DWIN::clearSerial() {
  while (_dwinSerial->available()) {
    _dwinSerial->read();
  }
}
#pragma region Custom

void DWIN::addButtonEvent(uint16_t vpAddr, int32_t lastBytes, HmiButtonEventCB_t ButtonEventCallback, void* args) {
  HmiEvent event;
  event.vpAddr = vpAddr;
  event.lastBytes = lastBytes;
  event.callBack.buttonEvent = ButtonEventCallback;
  event.eventType = DWIN_BUTTON;
  event.args = args;
  _eventList.push_back(event);
}

void DWIN::addTextReceivedEvent(uint16_t vpAddr, HmiTextReceivedEventCB_t TextEventCallback, void* args) {
  HmiEvent event;
  event.vpAddr = vpAddr;
  event.callBack.textReceivedEvent = TextEventCallback;
  event.eventType = DWIN_TEXT;
  event.args = args;
  _eventList.push_back(event);
}

void DWIN::setInt16Value(uint16_t vpAddress, int16_t value) {
  uint8_t dataLen = 0x05;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_WRITE, (uint8_t)((vpAddress >> 8) & 0xFF), (uint8_t)((vpAddress) & 0xFF), (int8_t)((value >> 8) & 0xFF), (int8_t)((value) & 0xFF) };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  readDWIN();
}

void DWIN::setInt32Value(uint16_t vpAddress, int32_t value) {
  uint8_t dataLen = 0x07;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_WRITE, (uint8_t)((vpAddress >> 8) & 0xFF), (uint8_t)((vpAddress) & 0xFF), (uint8_t)((value >> 24) & 0xFF), (int8_t)((value >> 16) & 0xFF), (int8_t)((value >> 8) & 0xFF), (int8_t)((value) & 0xFF) };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  readDWIN();
}

void DWIN::setUint16Value(uint16_t vpAddress, uint16_t value) {
  uint8_t dataLen = 0x05;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_WRITE, (uint8_t)((vpAddress >> 8) & 0xFF), (uint8_t)((vpAddress) & 0xFF), (uint8_t)((value >> 8) & 0xFF), (uint8_t)((value) & 0xFF) };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  readDWIN();
}

void DWIN::setUint32Value(uint16_t vpAddress, uint32_t value) {
  uint8_t dataLen = 0x07;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { CMD_HEAD1, CMD_HEAD2, dataLen, CMD_WRITE, (uint8_t)((vpAddress >> 8) & 0xFF), (uint8_t)((vpAddress) & 0xFF), (uint8_t)((value >> 24) & 0xFF), (uint8_t)((value >> 16) & 0xFF), (uint8_t)((value >> 8) & 0xFF), (uint8_t)((value) & 0xFF) };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer));
  readDWIN();
}

int16_t DWIN::getInt16Value(uint16_t vpAddress) {
  clearSerial();
  readVPWord(vpAddress, 1);
  flushSerial();
  uint8_t data[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  bool isFirstByte = false;
  unsigned long startTime = millis();  // Start time for Timeout
  while ((millis() - startTime < CMD_READ_TIMEOUT)) {
    while (_dwinSerial->available() > 0) {
      _dwinSerial->readBytes(data, sizeof(data) / sizeof(data[0]));
      if (data[0] == 0x5A && data[1] == 0xA5 && data[3] == 0x83 && ((uint16_t)data[4] << 8 | (uint16_t)data[5]) == vpAddress) {
        _wait_for_respone = false;
        return data[7] << 8 | data[8];
      }
    }
  }
  _wait_for_respone = false;
  return -1;
}

int32_t DWIN::getInt32Value(uint16_t vpAddress) {
  clearSerial();
  readVPWord(vpAddress, 2);
  flushSerial();

  uint8_t data[11] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  int32_t result;
  // memcpy(data, (const void*)-1, sizeof(data)/sizeof(data[0]));
  unsigned long startTime = millis();  // Start time for Timeout
  while ((millis() - startTime < CMD_READ_TIMEOUT)) {
    while (_dwinSerial->available() > 0) {
      _dwinSerial->readBytes(data, sizeof(data) / sizeof(data[0]));
      if (data[0] == 0x5A && data[1] == 0xA5 && data[3] == 0x83 && ((uint16_t)data[4] << 8 | (uint16_t)data[5]) == vpAddress) {
        _wait_for_respone = false;
        return data[7] << 24 | data[8] << 16 | data[9] << 8 | data[10];
      }
    }
  }
  _wait_for_respone = false;
  return -1;
}

uint16_t DWIN::getUint16Value(uint16_t vpAddress) {
  clearSerial();
  readVPWord(vpAddress, 1);
  flushSerial();

  uint8_t data[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  bool isFirstByte = false;
  unsigned long startTime = millis();  // Start time for Timeout
  while ((millis() - startTime < CMD_READ_TIMEOUT)) {
    while (_dwinSerial->available() > 0) {
      _dwinSerial->readBytes(data, sizeof(data) / sizeof(data[0]));
      if (data[0] == 0x5A && data[1] == 0xA5 && data[3] == 0x83 && ((uint16_t)data[4] << 8 | (uint16_t)data[5]) == vpAddress) {
        _wait_for_respone = false;
        return data[7] << 8 | data[8];
      }
    }
  }
  _wait_for_respone = false;
  return 0;
}

uint32_t DWIN::getUint32Value(uint16_t vpAddress) {
  clearSerial();
  readVPWord(vpAddress, 2);
  flushSerial();

  uint8_t data[11] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  int32_t result;
  unsigned long startTime = millis();  // Start time for Timeout
  while ((millis() - startTime < CMD_READ_TIMEOUT)) {
    while (_dwinSerial->available() > 0) {
      _dwinSerial->readBytes(data, sizeof(data) / sizeof(data[0]));
      if (data[0] == 0x5A && data[1] == 0xA5 && data[3] == 0x83 && ((uint16_t)data[4] << 8 | (uint16_t)data[5]) == vpAddress) {
        _wait_for_respone = false;
        return data[7] << 24 | data[8] << 16 | data[9] << 8 | data[10];
      }
    }
  }
  _wait_for_respone = false;
  return -1;
}

String DWIN::getText(uint16_t vpAddress, uint8_t lenText) {
  String message;
  bool isSubstr = false;
  bool messageEnd = false;
  bool isFirstByte = false;
  unsigned long startTime = millis();
  clearSerial();
  readVPWord(vpAddress, lenText / 2 + 1);
  flushSerial();
  while ((millis() - startTime < READ_TIMEOUT)) {
    while (_dwinSerial->available() > 0) {
      int inhex;
      if (_dwinSerial->read() == 0x5A && _dwinSerial->read() == 0xA5) {
        inhex = _dwinSerial->read();
        if (_dwinSerial->read() == 0x83) {
          isFirstByte = true;
        }
        else {
          continue;
        }
      }
      else {
        continue;
      }
      if (_crc) {
        inhex -= 2;
      }
      for (int i = 2; i <= inhex; i++) {
        int inByte = _dwinSerial->read();
        if (i >= 4) {
          if (messageEnd == false) {
            if (isSubstr && inByte != MAX_ASCII && inByte >= MIN_ASCII) {
              message += char(inByte);
            }
            else {
              if (inByte == MAX_ASCII) {
                messageEnd = true;
                clearSerial();
                _wait_for_respone = false;
                return message;
              }
              isSubstr = true;
            }
          }
        }
      }
    }
    if (isFirstByte)
      break;
  }
  _wait_for_respone = false;
  return "";
}

float DWIN::getFloatValue(uint16_t vpAddress) {
  clearSerial();
  readVPWord(vpAddress, 2);
  flushSerial();
  uint8_t dataLen = 11;
  uint8_t data[dataLen] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  bool isFirstByte = false;
  unsigned long startTime = millis();  // Start time for Timeout
  union {
    uint32_t rawData;
    float floatData;
  } hex2float;

  while ((millis() - startTime < CMD_READ_TIMEOUT)) {
    while (_dwinSerial->available() > 0) {
      _dwinSerial->readBytes(data, sizeof(data) / sizeof(data[0]));
      if (data[0] == 0x5A && data[1] == 0xA5 && data[3] == 0x83 && ((uint16_t)data[4] << 8 | (uint16_t)data[5]) == vpAddress) {
        _wait_for_respone = false;
        hex2float.rawData = data[7] << 24 | data[8] << 16 | data[9] << 8 | data[10];
        return hex2float.floatData;
      }
    }
  }
  _wait_for_respone = false;
  return -1;
}

void DWIN::setGraphYCentral(uint16_t spAddr, int value) {
  setVPWord(spAddr + 0x0005, value);
}

void DWIN::setGraphVDCentral(uint16_t spAddr, int value) {
  setVPWord(spAddr + 0x0006, value);
}

void DWIN::setGraphColor(uint16_t spAddr, int value) {
  setVPWord(spAddr + 0x0007, value);
}

void DWIN::setGraphRightToLeft(uint16_t spAddr) {
  setVPWord(spAddr, 0);
}

void DWIN::setGraphLeftToRight(uint16_t spAddr) {
  setVPWord(spAddr, 1);
}

void DWIN::setGraphMulY(uint16_t spAddr, int value) {
  setVPWord(spAddr + 0x0008, value);
}

void DWIN::sendGraphValue(int channel, int value) {
  uint8_t arr[] = { 0x82, 0x03, 0x10, 0x5A, 0xA5, 0x01, 0x00, (uint8_t)((channel) & 0xFF), 0x01,
    (uint8_t)((value >> 8) & 0xFF), (uint8_t)((value) & 0xFF) };
  sendArray(arr, sizeof(arr) / sizeof(arr[0]));
}

void DWIN::sendGraphValue(int channel, const int* values, size_t valueCount) {
  // Độ dài mảng cần gửi (bao gồm header và data)
  size_t dataSize = 9 + valueCount * 2;

  // Tạo mảng dữ liệu
  uint8_t arr[250];

  // Cấu hình header
  arr[0] = 0x82;                                  // Lệnh
  arr[1] = 0x03;                                  // Dữ liệu tiếp theo
  arr[2] = 0x10;                                  // Địa chỉ cao
  arr[3] = 0x5A;                                  // Địa chỉ thấp
  arr[4] = 0xA5;                                  // Constant
  arr[5] = 0x01;                                  // Ghi
  arr[6] = 0x00;                                  // Reserved
  arr[7] = static_cast<uint8_t>(channel & 0xFF);  // Channel
  arr[8] = static_cast<uint8_t>(valueCount);      // Số lượng data word

  // Thêm các giá trị data
  for (int i = 0; i < valueCount; ++i) {
    arr[9 + i * 2] = static_cast<uint8_t>((values[i] >> 8) & 0xFF);  // Byte cao
    arr[10 + i * 2] = static_cast<uint8_t>(values[i] & 0xFF);        // Byte thấp
  }

  // Gửi mảng qua hàm sendArray
  // int8_t countErr = 0;
  sendArray(arr, dataSize);
  // while(sendArray(arr, dataSize) == false) {
  //     countErr++;
  //     if(countErr >= 5) {
  //         return;
  //     }
  //     delay(5);
  // }
}

void DWIN::resetGraph(int channel) {
  uint8_t data_reset = 0x01;  // data_reset for channel 0
  data_reset += channel * 2;
  uint8_t dataLen = 0x05;
  if (_crc) {
    dataLen += 2;
  }
  uint8_t sendBuffer[3 + dataLen] = { 0x5A, 0xA5, dataLen, 0x82, 0x03, data_reset, 0x00, 0x00 };
  if (_crc) {
    uint16_t crc = calculateCRC(sendBuffer + 3, sizeof(sendBuffer) - 5);
    sendBuffer[sizeof(sendBuffer) - 2] = crc & 0xFF;
    sendBuffer[sizeof(sendBuffer) - 1] = (crc >> 8) & 0xFF;
  }
  dwinWrite(sendBuffer, sizeof(sendBuffer) / sizeof(sendBuffer[0]));
  readDWIN();
  flushSerial();
  clearSerial();
}

void DWIN::CRCEnabled(bool Enable) {
  _crc = Enable;
}

bool DWIN::isFirmwareFile(String& fileName) {
  if (fileName.endsWith(".bin"))
    return true;
  else if (fileName.endsWith(".BIN"))
    return true;
  else if (fileName.endsWith(".icl"))
    return true;
  else if (fileName.endsWith(".ICL"))
    return true;
  else if (fileName.endsWith(".DZK"))
    return true;
  else if (fileName.endsWith(".dzk"))
    return true;
  else if (fileName.endsWith(".HZK"))
    return true;
  else if (fileName.endsWith(".hzk"))
    return true;
  else if (fileName.endsWith(".LIB"))
    return true;
  else if (fileName.endsWith(".lib"))
    return true;
  else if (fileName.endsWith(".WAE"))
    return true;
  else if (fileName.endsWith(".wae"))
    return true;
  else if (fileName.endsWith(".UIC"))
    return true;
  else if (fileName.endsWith(".uic"))
    return true;
  else if (fileName.endsWith(".GIF"))
    return true;
  else if (fileName.endsWith(".gif"))
    return true;
  return false;
}

bool DWIN::updateHMI(fs::FS& filesystem, const char* dirPath) {
  /*
        Tắt Serial và bật lại để xóa Serial Event
        do Arduino không có hàm xóa trực tiếp.
        Sau khi cập nhật màn hình dù thành công hay không thì cũng phải restart lại ESP
    */
  _dwinSerial->end();
  delayMicroseconds(1000);
  _dwinSerial->begin(_baudrate, SERIAL_8N1, _rxPin, _txPin);
  echoEnabled(true);
  /*-----------------------------------------------------*/
  uint8_t hexCode[0xF3];
  int address = 0x8000;
  int i, j;
  uint32_t totalSize = 0;
  int fileIdx = 0;
  String fileName;
  Serial.println("Open: " + String(dirPath));

  /*
        Kiểm tra trong filesystem có tồn tại thư mục Update không ?
        nếu không tồn tại thì thoát khỏi hàm updateHMI
    */
  File root = filesystem.open(dirPath);
  if (!root || !root.isDirectory()) {
    Serial.println("Failed to update");
    return true;
  }

  /*
        Nếu tồn tại thư mục Update trong filesystem thì mở thư mục
        và duyệt qua các file cho đến khi tìm thấy một trong các file
        cập nhật màn hình VD: file .ICL, .BIN, .HZK.
        Thì tiến hành gửi lệnh cập nhật và truyền dữ liệu cập nhật đến DWIN HMI
        Lặp lại việc này cho đến khi nạp hết các file cập nhật vào màn hình
        sau đó restart màn hình để bắt đầu boot lại chương trình mới trên màn hình.
        Nếu không tìm thấy file cập nhật nào thì thoát khỏi hàm
    */
  File file = root.openNextFile();
  Serial.println("Start...");
  while (file) {
    fileName = file.name();
    if (isFirmwareFile(fileName) == false) {
      file.close();
      file = root.openNextFile();
    }
    else {
      break;
    }
  }
  if (!file) {
    Serial.println(" Not found HMI file");
    return true;
  }
  else {
    // Gửi lệnh bắt đầu cập nhật đến HMI
    uint8_t hexCodeFlash[] = { 0x82, 0x00, 0xfc, 0x55, 0xaa, 0x5a, 0xa5 };
    if (!this->sendArray(hexCodeFlash, sizeof(hexCodeFlash))) {
      Serial.println("Failed to update");
      return false;
    }
  }

  /*
        Bắt đầu gửi các mã HEX của các file cập nhật màn hình là các file trong DWIN_SET.
        Mỗi lần gửi 32Kb bắt đầu từ địa chỉ 0x8000 là địa chỉ trên RAM của màn hình.
        Gửi mỗi 240Byte/lần đủ 136 lần thì gửi 128Byte còn lại sau đó gửi lệnh ghi dữ liệu từ RAM vào Flash trên màn hình.
        Gửi lệnh hỏi DWIN đã ghi vào Flash chưa. Nếu DWIN phản hồi đã ghi thành công thì lặp lại quá trình trên cho đến khi hết file.
        Nếu kích thước của file cập nhật không chia hết cho 32Kb thì phần dư của nó cũng sẽ gửi bắt đầu tại địa chỉ 0x8000 trên RAM.
        Flash cũng có địa chỉ địa chỉ của Flash là từ 0-63 tức 64MB Flash được chia thành 64 stack
    */
  while (file) {

    fileName = String(file.name());
    fileIdx = fileName.toInt();
    size_t sizeFile = file.size();

    Serial.print(fileName + ": ");
    Serial.printf("%d Kb, %d byte. Update idx: %d.\n", sizeFile / 1024, sizeFile % 1024, fileIdx);
    Serial.printf("%dx32Kb + %dx240byte + %dbyte\n", sizeFile / 0x8000, sizeFile % 0x8000 / 240, sizeFile % 0x8000 % 240);

    i = 0;

    // Nếu kích thước file chia hết cho 32KB (0x8000)
    if (sizeFile / 0x8000 > 0) {
      // Phần cập nhật từng 32KB
      // sizeFile / 0x8000 = số lần cập nhật mỗi 32KB của file
      for (i = 0; i < sizeFile / 0x8000; i++) {
        // Cập nhật 136 * 240 Byte
        for (j = 0; j < 136; j++) {
          address = 0x8000 + j * 0x78;
          hexCode[0] = 0x82;
          hexCode[1] = (uint8_t)((address >> 8) & 0xFF);
          hexCode[2] = (uint8_t)((address) & 0xFF);
          totalSize += file.read((uint8_t*)(hexCode + 3), 240);

          if (!this->sendArray(hexCode, 0xF3)) {
            file.close();
            return false;
          }
        }

        // Cập nhật 128Kb còn lại tức là đã cập nhật (136*240 + 128) * i = 32Kb * i
        address = 0x8000 + j * 0x78;
        hexCode[0] = 0x82;
        hexCode[1] = (uint8_t)((address >> 8) & 0xFF);
        hexCode[2] = (uint8_t)((address) & 0xFF);
        totalSize += file.read((uint8_t*)(hexCode + 3), 128);

        if (!this->sendArray(hexCode, 0x83)) {
          file.close();
          return false;
        }

        // Gửi lệnh ghi dữ liệu từ RAM vào FLash
        int FlashAddrToWrite = fileIdx * 8 + i;
        {
          uint8_t hexCodeFlash[] = { 0x82, 0x00, 0xaa, 0x5a, 0x02, (uint8_t)((FlashAddrToWrite >> 8) & 0xFF), (uint8_t)((FlashAddrToWrite) & 0xFF), 0x80, 0x00, 0x17, 0x70, 0x00, 0x00, 0x00, 0x00 };

          if (!this->sendArray(hexCodeFlash, sizeof(hexCodeFlash))) {
            file.close();
            return false;
          }
        }
        delay(200);  // Chờ 200ms để màn hình ghi dữ liệu từ RAM vào Flash

        // Kiếm tra xem dữ liệu đã ghi xong chưa nếu xong màn hình sẽ phản hồi về mã "aa 01 00 02"
        // Nếu phản hồi về mã khác thì chờ 100ms ròi tiếp tra tiếp lần tiếp theo tối đa 3 lần
        // Mà không nhận được mã "aa 01 00 02" tức cập nhật thất bại
        for (int k = 0; true; k++) {
          uint8_t hexCodeFlash[] = { 0x83, 0x00, 0xaa, 0x01 };
          this->sendArray(hexCodeFlash, sizeof(hexCodeFlash));
          if (readDWIN().indexOf("aa 01 00 02") >= 0) {
            break;
          }
          else if (k >= 3) {
            file.close();
            return false;
          }
          delayMicroseconds(100000);
        }
      }
    }

    // Gửi những file có kích thước dưới 32Kb hoặc phần dư còn lại của file có kích thước > 32Kb
    j = 0;
    if ((sizeFile % 0x8000) / 240 > 0) {
      for (j = 0; j < (sizeFile % 0x8000) / 240; j++) {
        address = 0x8000 + j * 0x78;
        hexCode[0] = 0x82;
        hexCode[1] = (uint8_t)((address >> 8) & 0xFF);
        hexCode[2] = (uint8_t)((address) & 0xFF);

        totalSize += file.read((uint8_t*)(hexCode + 3), 240);
        // Serial.write(hexCode, 0xF3);
        if (!this->sendArray(hexCode, 0xF3)) {
          file.close();
          return false;
        }
      }
    }
    if ((sizeFile % 0x8000) % 240 > 0) {
      address = 0x8000 + j * 0x78;
      hexCode[0] = 0x82;
      hexCode[1] = (uint8_t)((address >> 8) & 0xFF);
      hexCode[2] = (uint8_t)((address) & 0xFF);
      totalSize += file.read((uint8_t*)(hexCode + 3), (sizeFile % 0x8000) % 240);
      if (!this->sendArray(hexCode, (sizeFile % 0x8000) % 240 + 3)) {
        file.close();
        return false;
      }
    }

    int FlashAddrToWrite = fileIdx * 8 + sizeFile / 0x8000;
    {
      uint8_t hexCodeFlash[] = { 0x82, 0x00, 0xaa, 0x5a, 0x02, (uint8_t)((FlashAddrToWrite >> 8) & 0xFF), (uint8_t)((FlashAddrToWrite) & 0xFF), 0x80, 0x00, 0x17, 0x70, 0x00, 0x00, 0x00, 0x00 };
      if (!this->sendArray(hexCodeFlash, sizeof(hexCodeFlash))) {
        file.close();
        return false;
      }
    }
    delay(200);
    for (int k = 0; true; k++) {
      uint8_t hexCodeFlash[] = { 0x83, 0x00, 0xaa, 0x01 };
      this->sendArray(hexCodeFlash, sizeof(hexCodeFlash));
      if (readDWIN().indexOf("aa 01 00 02") >= 0) {
        break;
      }
      else if (k >= 3) {
        file.close();
        return false;
      }
      delayMicroseconds(100000);
    }

    file.close();
    file = root.openNextFile();
    // Kiểm tra file tiếp theo trong thư mục Update có phải là file cập nhật màn hình không
    // Nếu là file cập nhật màn hình thì break khỏi vòng lặp để bắt đầu nạp file này vào màn hình
    // Nếu không còn file nào thì thoát và kết thúc quá trình nạp màn hình
    while (file) {
      fileName = file.name();
      if (isFirmwareFile(fileName) == false) {
        file.close();
        file = root.openNextFile();
      }
      else {
        break;
      }
    }
  }

  // Đóng các file đã mở trên filesystem
  if (root) {
    root.close();
  }
  if (file) {
    file.close();
  }

  // Restart lại màn hình
  Serial.println("RestartHMI");
  this->restartHMI();
  Serial.println("Done!");
  Serial.printf("Size: %d", totalSize);
  return true;
}

#pragma endregion Custom
