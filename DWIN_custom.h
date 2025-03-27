/*
* DWIN DGUS DWIN Library for Arduino
* This Library Supports all Basic Function
* Created by Tejeet ( tejeet@dwin.com.cn )
* UpHMI_DATEd by Satspares ( satspares@gmail.com )
* Please Checkout Latest Offerings FROM DWIN
* Here : https://www.dwin-global.com/
*/


#ifndef DWIN_H
#define DWIN_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#include  "HardwareSerial.h"
#include <FS.h>
#include <list>
using namespace std;
#else
#include "WProgram.h"
#endif

#ifndef ARDUINO_ARCH_RP2040 
#ifndef ESP32
#include <SoftwareSerial.h>
#endif   
#endif

#ifndef MAX_BUFFER_RESPONE 
#define MAX_BUFFER_RESPONE  255
#endif //MAX_BUFFER_RESPONE 

#define DWIN_DEFAULT_BAUD_RATE      115200
// #define DWIN_DEFAULT_BAUD_RATE      537600
#define ARDUINO_RX_PIN              10
#define ARDUINO_TX_PIN              11

//#define FORCEHWSERIAL

typedef struct {
    union {
        uint8_t pu8Data;
    };
    uint8_t sizeofData;
    uint16_t VPAddress;
} FrameQueueUart_t;

class DWIN {

public:
    // Using AVR Board with Hardware Serial
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(FORCEHWSERIAL)
    DWIN(HardwareSerial& port, long baud = DWIN_DEFAULT_BAUD_RATE);

    // Using Pico Board
#elif defined(ARDUINO_ARCH_RP2040)
    DWIN(HardwareSerial& port, long baud, bool initSerial);

    //Using STM32 Arduino
#elif ARDUINO_ARCH_STM32
    DWIN(HardwareSerial& port);

    // Using ESP32 Board
#elif defined(ESP32)
    DWIN(HardwareSerial& port, uint8_t receivePin, uint8_t transmitPin, long baud = DWIN_DEFAULT_BAUD_RATE);

    // Using ESP8266 Board
#elif defined(ESP8266)
    DWIN(uint8_t receivePin, uint8_t transmitPin, long baud = DWIN_DEFAULT_BAUD_RATE);
    DWIN(SoftwareSerial& port, long baud = DWIN_DEFAULT_BAUD_RATE);
    DWIN(Stream& port, long baud = DWIN_DEFAULT_BAUD_RATE);

    // Using Arduino Board
#else
    DWIN(uint8_t rx = ARDUINO_RX_PIN, uint8_t tx = ARDUINO_TX_PIN, long baud = DWIN_DEFAULT_BAUD_RATE);
#endif

    // PUBLIC Methods
    void initUartEvent();

    // dont look for the ack on no response kernels
    void ackDisabled(bool noACK);
    void echoEnabled(bool enabled);
    // if true return word from rx event not uint8_t
    void returnWord(bool retWord);
    // Listen Touch Events & Messages from HMI
    void listen();
    // Get Version
    double getHWVersion();
    //get GUI software version
    double getGUISoftVersion();
    // restart HMI
    void restartHMI();
    // set Particular Page
    void setPage(uint8_t pageID);
    // get Current Page ID
    uint8_t getPage();
    // set LCD Brightness
    void setBrightness(uint8_t pConstrast);
    // set LCD Brightness
    uint8_t getBrightness();
    // set Data on VP Address
    void setText(long address, String textData);
    // // Set Byte Data on VP Address makes more sense
    void setVPByte(long address, uint8_t data); //alias of below
    void setVP(long address, uint8_t data);
    // read uint8_t from VP Address if bool = true read HiByte
    uint8_t readVPByte(long address, bool = 0);
    // Set WordData on VP Address
    void setVPWord(long address, int data);
    // read WordData from VP Address you can read sequential multiple words (data returned in rx event) 
    void readVPWord(long address, uint8_t numWords);

    // read or write the NOR from/to VP must be on a even address 2 word are written or read
    void norReadWrite(bool write, long VPAddress, long NORAddress);
    // Play a sound
    void playSound(uint8_t soundID);
    // beep Buzzer
    void beepHMI(int time);
    // set the hardware RTC The first two digits of the year are automatically added
    void setRTC(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
    // upHMI_DATE the software RTC The first two digits of the year are automatically added
    void setRTCSOFT(uint8_t year, uint8_t month, uint8_t day, uint8_t weekday, uint8_t hour, uint8_t minute, uint8_t second);
    // set text color (16-bit RGB) on controls which allow it ie. text control.
    //  changes the control sp address space (sp=description pointer) content see the DWIN docs.  
    void setTextColor(long spAddress, long spOffset, long color);
    //set float value to 32bit DATA Variable Control  
    void setFloatValue(long vpAddress, float fValue);

    // Send array to the display we dont need the 5A A5 or 
    // the size uint8_t hopefully we can work this out.
    //uint8_t hmiArray[] = {0x83,0x10,0x00,0x1};        // Read 0x1000 one word returns in the rx event
    //uint8_t hmiArray[] = {0x82,0x88,0x00,0x55,0xAA};  // Write 0x8800
    //hmi.sendArray(hmiArray,sizeof(hmiArray));
    bool sendArray(uint8_t dwinSendArray[], uint8_t arraySize);

    // Send int array to the display we dont need the 5A A5 or size - words only
    // eg. Using Basic Graphic Control vp 0x5000 draw rectangle
    //  uint16_t intArrayRect[] = {0x5000,0x0003,0x0001,200,100,650,400,0xFFF0,0xFF00};
    //  Fill it with Yellow
    //  uint16_t intArrayFill[] = {0x5000,0x0004,0x0001,200,100,650,400,0xFFF0,0xFF00};
    //  display it
    //  hmi.sendIntArray(0x82,intArrayRect,sizeof(intArrayRect));
    //  hmi.sendIntArray(0x82,intArrayFill,sizeof(intArrayFill));
    void sendIntArray(uint16_t instruction, uint16_t dwinIntArray[], uint8_t arraySize);
    // Custom -----------------------------------------------------------------------------------------------------
    void setInt16Value(uint16_t vpAddress, int16_t value);  //ok
    void setInt32Value(uint16_t vpAddress, int32_t value);  // ok
    void setUint16Value(uint16_t vpAddress, uint16_t value);//ok
    void setUint32Value(uint16_t vpAddress, uint32_t value);//ok
    int16_t getInt16Value(uint16_t vpAddress);      // ok
    int32_t getInt32Value(uint16_t vpAddress);      // ok
    uint16_t getUint16Value(uint16_t vpAddress);    // ok
    uint32_t getUint32Value(uint16_t vpAddress);    // ok

    String getText(uint16_t vpAddress, uint8_t lenText); // ok
    float getFloatValue(uint16_t vpAddress);    // ok

    // Callback Function
    typedef void (*hmiListener) (String address, int lastBytes, String message, String response);

    // CallBack Method
    void hmiCallBack(hmiListener callBackFunction);

    // Init the serial port in setup useful for Pico boards
    void initSerial(HardwareSerial& port, long baud);

    // Custom

    void CRCEnabled(bool Enable = true);
    void setGraphYCentral(uint16_t spAddr, int value);
    void setGraphVDCentral(uint16_t spAddr, int value);
    void setGraphMulY(uint16_t spAddr, int value);
    void setGraphColor(uint16_t spAddr, int value);
    void setGraphRightToLeft(uint16_t spAddr);
    void setGraphLeftToRight(uint16_t spAddr);
    void sendGraphValue(int channel, int value);
    void sendGraphValue(int channel, const int* values, size_t valueCount);
    void resetGraph(int channel);

    // Chỉ trả về False khi xảy ra lỗi trong quá trình cập nhật
    // Các trường hợp không tìm thấy thư mục hoặc file cập nhật 
    // Tức là không có bản cập nhật mới sẽ trả về true
    bool updateHMI(fs::FS& filesystem, const char* dirPath);
private:

#ifndef ARDUINO_ARCH_RP2040 
#ifndef ESP32
    SoftwareSerial* localSWserial = nullptr;
#endif   
#endif

#if defined(ESP32)
    HardwareSerial* _dwinSerial;
#else 
    Stream* _dwinSerial;   // DWIN Serial interface
#endif
    uint8_t _rxPin, _txPin;
    long _baudrate;
    bool _isSoft;          // Is serial interface software
    long _baud;            // DWIN HMI Baud rate
    bool _echo = false;    // Response Command Show
    bool _isConnected;     // Flag set on successful communication
    bool _noACK = false;   // No ack used with no response kernel 
    bool _retWord = false; // return word from rx event when true 
    bool _crc = false;
    bool cbfunc_valid;
    bool _hiByte = false;
    uint8_t _lastByte = -1;
    uint8_t _previousByte = -1;
    bool _wait_for_respone = false;
    hmiListener listenerCallback;

    void init(HardwareSerial* port);
    void _createHmiListenTask(void* args);
    static void _hmiListenTask(void* args);
    static void _hmiUartEvent(void);


    uint8_t readCMDLastByte(bool hiByte = 0);
    uint8_t readCMDLastByteFromEvent(bool hiByte = 0);
    uint8_t readCMDLastByteEvent(uint8_t length, uint8_t cmd);
    size_t dwinWrite(const uint8_t* sendBuffer, size_t size);
    String readDWIN();
    String handle();
    String checkHex(uint8_t currentNo);
    bool isFirmwareFile(String& fileName);
    void flushSerial();
    void clearSerial();
    uint16_t calculateCRC(uint8_t* data, size_t length);

};



#endif // DWIN_H
