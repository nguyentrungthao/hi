// HeaterControl.cpp

#include "HeaterControl.h"

// Define the global objects
// SERVO _Servo;
HEATER _Heater;
DOOR _Door;

void CaiDatHeater() {
  if (BaseProgram.machineState == true) {
    _Heater.BatDieuKhienNhietDo();
  }
  else if (BaseProgram.machineState == false) {
    _Heater.TatDieuKhienNhietDo();
  }
  _Heater.CaiDatNhietDo(BaseProgram.programData.setPointTemp);
  _Heater.CaiTocDoQuat(BaseProgram.programData.fanSpeed);
  _Heater.CaiGiaTriOfset(GetCalib(BaseProgram.programData.setPointTemp));
  // _Servo.MoCuaXa(BaseProgram.programData.flap);
}

void CapNhatTrangThaiHeater() {
  static bool DoorState = false;
  static bool FanState = false;
  static bool HeaterState = false;
  static bool AcdetState = true;

  if (fabs(BaseProgram.temperature - _Heater.LayNhietDoLoc()) >= 0.05) {
    BaseProgram.temperature = _Heater.LayNhietDoLoc();
    if (BaseProgram.temperature > -10 && BaseProgram.temperature < 350) {
      _dwin.HienThiNhietDo(BaseProgram.temperature);
    }
    else {
      _dwin.HienThiNhietDo("err");
    }
    // Serial.println("Gui nhiet do len HMI");
  }

  if (_Door.TrangThai() == true) {  // 1 Mở
    if (DoorState == 0) {
      DoorState = 1;
      FanState = 0;
      _dwin.HienThiIconCua(DoorState);
      _dwin.HienThiIconQuat(FanState);
      Serial.println("DoorState 1, FanState 0");
    }
  }
  else if (BaseProgram.machineState == true) {
    if ((DoorState == 1 || FanState == 0)) {
      DoorState = 0;
      FanState = 1;
      _dwin.HienThiIconCua(DoorState);
      _dwin.HienThiIconQuat(FanState);
      Serial.println("DoorState 0, FanState 1");
    }
  }
  else if (BaseProgram.machineState == false) {
    if (DoorState == 1 || FanState == 1) {
      DoorState = 0;
      FanState = 0;
      _dwin.HienThiIconCua(DoorState);
      _dwin.HienThiIconQuat(FanState);
      Serial.println("DoorState 0, FanState 0");
    }
  }

  if (_Heater.TrangThaiThanhGiaNhiet() == HEATER_ON) {
    if (HeaterState == 0) {
      HeaterState = 1;
      _dwin.HienThiIconGiaNhiet(HeaterState);
      Serial.println("HeaterState");
    }
  }
  else {
    if (HeaterState == 1) {
      HeaterState = 0;
      _dwin.HienThiIconGiaNhiet(HeaterState);
      Serial.println("HeaterState");
    }
  }

  if (_Heater.CheckAcdet() == false && AcdetState == true) {
    AcdetState = false;
    _dwin.HienThiWarning("Power supply error", _HomePage);
  }
  else if (_Heater.CheckAcdet() == true) {
    AcdetState = true;
  }
}

void KhoiTaoHeater() {
  _Door.KhoiTao();
  _Heater.KhoiTao();
  // _Servo.KhoiTao();
  _Heater.TatDieuKhienNhietDo();
}

float GetCalib(float value) {
  for (CalibData_t HeSoCalib : DanhSachHeSoCalib) {
    if (fabs(HeSoCalib.Setpoint - value) < 0.02) {
      // Serial.printf("Setpoint: %.1f, value: %.2f\n", HeSoCalib.Setpoint, HeSoCalib.value);
      return HeSoCalib.value;
    }
  }
  return 0;
}