#if !SOC_USB_OTG_SUPPORTED || ARDUINO_USB_MODE
#error Device does not support USB_OTG or native USB CDC/JTAG is selected \
 Arduino -> tool -> USB mode -> USB OTG
#endif

#include <USB.h>
#include <USBMSC.h>
#include <SD.h>
#include <SPI.h>
#include <pins_arduino.h>

// USB Mass Storage Class (MSC) object
USBMSC msc;

// int sck = 12;
// int miso = 13;
// int mosi = 11;
// int cs = 10;

static int32_t onWrite(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
  uint32_t secSize = SD.sectorSize();
  if (!secSize) {
    return false;  // disk error
  }
  log_v("Write lba: %ld\toffset: %ld\tbufsize: %ld", lba, offset, bufsize);
  for (int x = 0; x < bufsize / secSize; x++) {
    uint8_t blkbuffer[secSize];
    memcpy(blkbuffer, (uint8_t *)buffer + secSize * x, secSize);
    if (!SD.writeRAW(blkbuffer, lba + x)) {
      return false;
    }
  }
  return bufsize;
}

static int32_t onRead(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
  uint32_t secSize = SD.sectorSize();
  if (!secSize) {
    return false;  // disk error
  }
  // log_v("Read lba: %ld\toffset: %ld\tbufsize: %ld\tsector: %lu", lba, offset, bufsize, secSize);
  for (int x = 0; x < bufsize / secSize; x++) {
    if (!SD.readRAW((uint8_t *)
buffer + (x * secSize), lba + x)) {
      return false;  // outside of volume boundary
    }
  }
  return bufsize;
}

static bool onStartStop(uint8_t power_condition, bool start, bool load_eject) {
  // log_i("Start/Stop power: %u\tstart: %d\teject: %d", power_condition, start, load_eject);
  return true;
}

static void usbEventCallback(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  Serial.println(__func__);
  if (event_base == ARDUINO_USB_EVENTS) {
    arduino_usb_event_data_t *data = (arduino_usb_event_data_t *)event_data;
    switch (event_id) {
      case ARDUINO_USB_STARTED_EVENT: Serial.println("USB PLUGGED"); break;
      case ARDUINO_USB_STOPPED_EVENT: Serial.println("USB UNPLUGGED"); break;
      case ARDUINO_USB_SUSPEND_EVENT: Serial.printf("USB SUSPENDED: remote_wakeup_en: %u\n", data->suspend.remote_wakeup_en); break;
      case ARDUINO_USB_RESUME_EVENT:  Serial.println("USB RESUMED"); break;

      default: break;
    }
  }
}

void sd2usbmsc_init() {
  // Serial.begin(115200);

  // Khởi tạo SD
  // Serial.println("Mounting SDcard");
  // SPI.begin(sck, miso, mosi, cs);
  // if (!SD.begin(cs)) {
  //     Serial.println("Card Mount Failed");
  //     return;
  // }

  Serial.println("Initializing MSC");
  // Initialize USB metadata and callbacks for MSC (Mass Storage Class)
  msc.vendorID("ESP32");
  msc.productID("USB_MSC");
  msc.productRevision("1.0");
  msc.onRead(onRead);
  msc.onWrite(onWrite);
  msc.onStartStop(onStartStop);
  msc.mediaPresent(true);
  msc.begin(SD.numSectors(), SD.sectorSize());

  Serial.println("Initializing USB");

  USB.begin();
  // USB.onEvent(usbEventCallback);

  // Serial.printf("Card Size: %lluMB\n", SD.totalBytes() / 1024 / 1024);
  // Serial.printf("Sector: %d\tCount: %d\n", SD.sectorSize(), SD.numSectors());
}

// void sd2usbmsc_deinit()
// {
//   USB.end();
//   msc.end();
// }
// void loop() {
//   delay(-1);
// }
