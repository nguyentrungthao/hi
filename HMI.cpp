
#include "HMI.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "HMIparam.h"
#include <RTClib.h>
#include <WiFi.h>

#define TAG "HMI"
#define MAX_SEMAPHORE 5
#define INIT_SEMAPHORE 2

#define MAX_INPUT_TEMPERATURE 180
#define MIN_INPUT_TEMPERATURE 20

#define MAX_INPUT_DAY_RTC 31
#define MIN_INPUT_DAY_RTC 1

#define MAX_INPUT_MONTH_RTC 12
#define MIN_INPUT_MONTH_RTC 1

#define MAX_INPUT_YEAR_RTC 2099
#define MIN_INPUT_YEAR_RTC 2000

#define MAX_INPUT_HOUR_RTC 23
#define MIN_INPUT_HOUR_RTC 0

#define MAX_INPUT_MINUTE_RTC 59
#define MIN_INPUT_MINUTE_RTC 0

#define MAX_INPUT_SECOND_RTC 59
#define MIN_INPUT_SECOND_RTC 0

#define MAX_INPUT_DAY_DELAYOFF 99
#define MIN_INPUT_DAY_DELAYOFF 0

#define MAX_INPUT_HOUR_DELAYOFF 23
#define MIN_INPUT_HOUR_DELAYOFF 0

#define MAX_INPUT_MINUTE_DELAYOFF 59
#define MIN_INPUT_MINUTE_DELAYOFF 0

#define MAX_INPUT_HOUR_STERILIZATION_DELAYOFF 2
#define MIN_INPUT_HOUR_STERILIZATION_DELAYOFF 0

#define MAX_INPUT_MINUTE_STERILIZATION_DELAYOFF 59
#define MIN_INPUT_MINUTE_STERILIZATION_DELAYOFF 0

static SemaphoreHandle_t _semaHmi;

HMI::HMI(HardwareSerial& port, uint8_t receivePin, uint8_t transmitPin, long baud) : DWIN(port, receivePin, transmitPin, baud),
_hmiSerial(&port),
_ChuoiBanPhimDangNhap(""),
_TenChuongTrinh(""),
_ChiMucChuongTrinhTruocDo(0xffff),
_ChiMucPhanDoanTruocDo(0xffff),
_CapslockEnable(false),
_ChuoiPassword("123456")
{
}

void HMI::KhoiTao(void)
{
    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueSetSetpointTemp, _NutCaiNhietDoSetpoint_, this);
    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueSetFanSpeed, _NutCaiTocDoQuat_, this);
    DWIN::addButtonEvent(_VPAddressSegmentSetpointButton, _AllKeyValue, _NutEditSetpointTrangSegment_, this);
    DWIN::addButtonEvent(_VPAddressSegmentFanSpeedButton, _AllKeyValue, _NutEditTocDoQuatTrangSegment_, this);
    // DWIN::addButtonEvent(_VPAddressSegmentFlapButton, _AllKeyValue, _NutEditFlapTrangSegment_, this);
    DWIN::addButtonEvent(_VPAddressSegmentTempMinButton, _AllKeyValue, _NutEditTempMinTrangSegment_, this);
    DWIN::addButtonEvent(_VPAddressSegmentTempMaxButton, _AllKeyValue, _NutEditTempMaxTrangSegment_, this);

    // DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueSegment, _SegmentButton_, this);
    DWIN::addButtonEvent(_VPAddressSegmentFunctionButton, _AllKeyValue, _CacNutThaoTacTrongTrangEditSegment_, this);

    DWIN::addButtonEvent(_VPAddressSegmentDelayOffButton, _AllKeyValue, _NutEditThoiGianTatTrangSegment_, this);
    DWIN::addButtonEvent(_VPAddressSetTimeButton, _KeyValueSetDay, _NutVaoTrangNhapNgay_, this);
    DWIN::addButtonEvent(_VPAddressSetTimeButton, _KeyValueSetMonth, _NutVaoTrangNhapThang_, this);
    DWIN::addButtonEvent(_VPAddressSetTimeButton, _KeyValueSetYear, _NutVaoTrangNhapNam_, this);
    DWIN::addButtonEvent(_VPAddressSetTimeButton, _KeyValueSetHour, _NutVaoTrangNhapGio_, this);
    DWIN::addButtonEvent(_VPAddressSetTimeButton, _KeyValueSetMinute, _NutVaoTrangNhapPhut_, this);
    DWIN::addButtonEvent(_VPAddressSetTimeButton, _KeyValueEnterSegmentDelayOff, _NutEnterTrongCaiDatThoiGianTatCuaSegment_, this);
    DWIN::addButtonEvent(_VPAddressSegmentSelectButton, _AllKeyValue, _NutChonSegment_, this);

    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueCalib, _NutVaoChucNangCalib_, this);
    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueEditCalibTemp, _NutEditCalib_, this);
    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueEnterCalibTemp, _NutEnterTrangCalib_, this);
    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueResetCalibTemp, _NutResetHeSoCalib_, this);

    // DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueSetFlap, _NutSetFlap_, this);
    // DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueEnterFlap, _NutEnterTrangFlap_, this);

    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueSetTimeRTC, _NutCaiDatThoiGianRTC_, this);
    DWIN::addButtonEvent(_VPAddressSetTimeButton, _KeyValueEnterSetRTC, _NutEnterTrangCaiRTC_, this);

    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueAlarm, _NutCaiCanhBaoNhietDo_, this);
    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueEnterSetAlarm, _NutEnterTrangCaiCanhBaoNhietDo_, this);
    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueSetAlarmBelow, _NutCaiCanhBaoNhietDoThap_, this);
    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueSetAlarmAbove, _NutCaiCanhBaoNhietDoCao_, this);

    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueProgram, _NutVaoChucNangProgram_, this);
    DWIN::addButtonEvent(_VPAddressSelectProgram, _AllKeyValue, _NutChonProgram_, this);
    DWIN::addButtonEvent(_VPAddressProgramFunction, _AllKeyValue, _CacNutThaoTacTrongTrangProgram_, this);

    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueSetDelayOff, _NutCaiThoiGianTatMay_, this);
    DWIN::addButtonEvent(_VPAddressSetTimeButton, _KeyValueEnterSetDelayOff, _NutEnterCaiThoiGianTatMay_, this);

    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueRunButton, _NutRun_, this);
    DWIN::addButtonEvent(_VPAddressSwitchDelayOff, _AllKeyValue, _NutThayDoiTrangThaiChucNangHenGioTat_, this);

    DWIN::addButtonEvent(_VPAddressEnterRunProgram, _AllKeyValue, _NutEnterTrongTrangProgram_, this);

    DWIN::addButtonEvent(_VPAddressEnterProgramLoop, _AllKeyValue, _NutEnterChayProgramVoiChuKyDuocChon_, this);
    DWIN::addButtonEvent(_VPAddressInfButton, _AllKeyValue, _NutInfTrongTrangChonChuKyChayProgram_, this);
    DWIN::addButtonEvent(_VPAddressSetProgramLoop, _AllKeyValue, _NutCaiChuKyChayProgram_, this);

    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueSterilization, _NutVaoChucNangTietTrung_, this);
    DWIN::addButtonEvent(_VPAddressCaiThoiGianTietTrung, _AllKeyValue, _NutCaiThoiGianTietTrung_, this);
    DWIN::addButtonEvent(_VPAddressSetTimeButton, _KeyValueEnterCaiThoiGianTietTrung, _NutEnterCaiThoiGianTietTrung_, this);
    DWIN::addButtonEvent(_VPAddressCaiNhietDoTietTrung, _AllKeyValue, _NutCaiNhietDoTietTrung_, this);
    DWIN::addButtonEvent(_VPAddressNextTrongTrangTietTrung, _AllKeyValue, _NutNextTrongCaiTietTrung_, this);
    DWIN::addButtonEvent(_VPAddressEnterCaiTietTrung, _AllKeyValue, _NutEnterCaiTietTrung_, this);

    DWIN::addButtonEvent(_VPAddressNumericKeypad, _AllKeyValue, _XuLyBanPhim_, this);
    DWIN::addButtonEvent(_VPAddressWarningReturn, _AllKeyValue, _NutTroVeTrongTrangWarning_, this);
    DWIN::addButtonEvent(_VPAddressResetGraph, _AllKeyValue, _NutXoaDoThi_, this);

    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueDataRecord, _NutDataRecord_, this);
    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueExport, _NutExportData_, this);

    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueUpdate, _NutVaoChucNangUpdate_, this);
    DWIN::addButtonEvent(_VPAddressCacNutChonPhuongThucUpdate, _AllKeyValue, _CacNutChonPhuongThucUpdate_, this);
    DWIN::addButtonEvent(_VPAddressCacNutTrangFOTA, _AllKeyValue, _CacNutTrangFOTA_, this);

    // DWIN::addButtonEvent(_VPAddressThanhCuonDoThi, _AllKeyValue, _ThanhCuonDoThi_, this);

    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueWifi, _NutVaoChucNangWiFi_, this); // truc them
    DWIN::addButtonEvent(_VPAddressCacNutTrangWiFi, _AllKeyValue, _CacNutTrangWiFi_, this);

    DWIN::addButtonEvent(_VPAddressCacNutNhan, _KeyValueAdminPassword, _NutVaoChucNangThayDoiAdminPassword_, this); //truc them
    DWIN::addButtonEvent(_VPAddressCacNutTrangAdminPassword, _AllKeyValue, _CacNutTrangThayDoiAdminPassword_, this);

    _semaHmi = xSemaphoreCreateCounting(MAX_SEMAPHORE, INIT_SEMAPHORE);
    _lock = xSemaphoreCreateMutex();
    _createHmiListenTask(this);

    _hmiSerial->onReceive(_hmiUartEvent, true);
    // DWIN::echoEnabled(true);
    DWIN::returnWord(true);
}

void HMI::_hmiUartEvent(void)
{
    xSemaphoreGive(_semaHmi);
    return;
}

void HMI::_hmiListenTask(void* args)
{
    HMI* hmiPtr = static_cast<HMI*>(args);
    for (;;)
    {
        if (xSemaphoreTake(_semaHmi, portMAX_DELAY) == pdTRUE)
        {
            // xSemaphoreTake(hmiPtr->_lock, portMAX_DELAY);
            hmiPtr->DWIN::listen();
            // xSemaphoreGive(hmiPtr->_lock);
        }
    }
}

void HMI::_createHmiListenTask(void* args)
{
    xTaskCreateUniversal(_hmiListenTask, "HMI Task", 5120, this, 15, &_hmiListenTaskHandle, -1);
    if (_hmiListenTaskHandle == NULL)
    {
        log_e(" -- HMI Task not Created!");
    }
}

void HMI::DangKyHamSetCallback(hmiSetData_t function)
{
    _hmiSetDataCallback = function;
}

void HMI::DangKyHamGetCallback(hmiGetData_t function)
{
    _hmiGetDataCallback = function;
}
#pragma region Keyboard
void HMI::_XuLyBanPhim_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    if (lastBytes == 0xF0)
    {
        hmiPtr->DWIN::setPage(hmiPtr->_set_event.pageAfterReturn);
    }
    else if (lastBytes == 0xF4) // Caps lock
    {
        hmiPtr->_CapslockEnable = !hmiPtr->_CapslockEnable;
        if (hmiPtr->_CapslockEnable)
        {
            hmiPtr->setVP(_VPAddressIconLowercaseKeyboard, 2);
        }
        else
        {
            hmiPtr->setVP(_VPAddressIconLowercaseKeyboard, 0);
        }
    }
    else if (lastBytes == 0x0D) // Nút Enter
    {
        // Loại bỏ khoảng trắng 2 đầu chuỗi.
        hmiPtr->_ChuoiBanPhimDangNhap.trim();

        if (hmiPtr->_set_event.displayType == HMI_PASSWORD)
        {
            if (hmiPtr->_ChuoiBanPhimDangNhap.compareTo(hmiPtr->_ChuoiPassword) == 0)
            {
                hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
                hmiPtr->setPage(hmiPtr->_set_event.pageAfterEnter);
                hmiPtr->_DemTrangThaiNext++;
            }
            else
            {
                hmiPtr->setText(_VPAddressKeyboardWarningText, "Incorrect");
            }
            return;
        }
        if (hmiPtr->_set_event.displayType == HMI_EXTERNAL_PASSWORD)
        {
            hmiPtr->_set_event.text = hmiPtr->_ChuoiBanPhimDangNhap;
            hmiPtr->setText(hmiPtr->_set_event.VPTextDisplayAfterEnter, hmiPtr->_ChuoiBanPhimDangNhap);
            hmiPtr->setPage(hmiPtr->_set_event.pageAfterEnter);
            hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
            return;
        }
        if (hmiPtr->_set_event.displayType == HMI_TEXT)
        {
            if (hmiPtr->_ChuoiBanPhimDangNhap == "" || hmiPtr->_ChuoiBanPhimDangNhap.indexOf(' ') == 0)
            {
                hmiPtr->setText(_VPAddressKeyboardWarningText, "Enter name");
                return;
            }
            hmiPtr->_set_event.text = hmiPtr->_ChuoiBanPhimDangNhap;
            hmiPtr->setText(hmiPtr->_set_event.VPTextDisplayAfterEnter, hmiPtr->_ChuoiBanPhimDangNhap);
            hmiPtr->setPage(hmiPtr->_set_event.pageAfterEnter);
            hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
            return;
        }
        float value = hmiPtr->_ChuoiBanPhimDangNhap.toFloat();
        if (value < hmiPtr->_set_event.minValue)
        {
            if (hmiPtr->_set_event.displayType == HMI_FLOAT)
            {
                hmiPtr->setText(_VPAddressKeyboardWarningText, "Min: " + String(hmiPtr->_set_event.minValue, 1));
            }
            else if (hmiPtr->_set_event.displayType == HMI_INT)
            {
                hmiPtr->setText(_VPAddressKeyboardWarningText, "Min: " + String(hmiPtr->_set_event.minValue, 0));
            }
        }
        else if (value > hmiPtr->_set_event.maxValue)
        {
            if (hmiPtr->_set_event.displayType == HMI_FLOAT)
            {
                hmiPtr->setText(_VPAddressKeyboardWarningText, "Max: " + String(hmiPtr->_set_event.maxValue, 1));
            }
            else if (hmiPtr->_set_event.displayType == HMI_INT)
            {
                hmiPtr->setText(_VPAddressKeyboardWarningText, "Max: " + String(hmiPtr->_set_event.maxValue, 0));
            }
        }
        else
        {
            switch (hmiPtr->_set_event.displayType)
            {
            case HMI_FLOAT:
                if (hmiPtr->_ChuoiBanPhimDangNhap.indexOf('+') >= 0)
                {
                    hmiPtr->DWIN::setText(hmiPtr->_set_event.VPTextDisplayAfterEnter, "+" + String(value, 1));
                }
                else
                {
                    hmiPtr->DWIN::setText(hmiPtr->_set_event.VPTextDisplayAfterEnter, String(value, 1));
                }
                break;
            case HMI_INT:
                if (hmiPtr->_ChuoiBanPhimDangNhap.indexOf('+') >= 0)
                {
                    hmiPtr->DWIN::setText(hmiPtr->_set_event.VPTextDisplayAfterEnter, "+" + String(value, 0));
                }
                else
                {
                    hmiPtr->DWIN::setText(hmiPtr->_set_event.VPTextDisplayAfterEnter, String(value, 0));
                }
                break;
            default:
                break;
            }
            hmiPtr->_set_event.f_value = value;
            hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
            hmiPtr->DWIN::setPage(hmiPtr->_set_event.pageAfterEnter);
            hmiPtr->_ChuoiBanPhimDangNhap = "";
        }
    }
    else if (((char)lastBytes == '.' && hmiPtr->_ChuoiBanPhimDangNhap.indexOf('.') != -1) || ((char)lastBytes == '.' && hmiPtr->_set_event.displayType != HMI_FLOAT))
    {
        return;
    }
    // 0xF2 là mã của phím xóa
    else if (lastBytes == 0xF2)
    {
        if ((hmiPtr->_ChuoiBanPhimDangNhap == "-" || hmiPtr->_ChuoiBanPhimDangNhap == "+") && (hmiPtr->_set_event.displayType == HMI_FLOAT || hmiPtr->_set_event.displayType == HMI_INT))
        {
            return;
        }
        else
        {
            hmiPtr->_ChuoiBanPhimDangNhap.remove(hmiPtr->_ChuoiBanPhimDangNhap.length() - 1);
        }
        hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    }
    else if (hmiPtr->_ChuoiBanPhimDangNhap.length() >= hmiPtr->_set_event.textLen)
    {
        return;
    }
    else if (hmiPtr->_set_event.displayType == HMI_INT || hmiPtr->_set_event.displayType == HMI_FLOAT)
    {
        hmiPtr->_ChuoiBanPhimDangNhap += String((char)lastBytes);
        if (hmiPtr->_ChuoiBanPhimDangNhap.toFloat() > hmiPtr->_set_event.maxValue)
        {
            // hmiPtr->_ChuoiBanPhimDangNhap.remove(hmiPtr->_ChuoiBanPhimDangNhap.length() - 1);
            if (hmiPtr->_set_event.displayType == HMI_INT)
            {
                if (hmiPtr->_ChuoiBanPhimDangNhap.indexOf('+') == 0)
                {
                    hmiPtr->_ChuoiBanPhimDangNhap = "+" + String(hmiPtr->_set_event.maxValue, 0);
                }
                else
                {
                    hmiPtr->_ChuoiBanPhimDangNhap = String(hmiPtr->_set_event.maxValue, 0);
                }
            }
            else if (hmiPtr->_set_event.displayType == HMI_FLOAT)
            {
                if (hmiPtr->_ChuoiBanPhimDangNhap.indexOf('+') == 0)
                {
                    hmiPtr->_ChuoiBanPhimDangNhap = "+" + String(hmiPtr->_set_event.maxValue, 1);
                }
                else
                {
                    hmiPtr->_ChuoiBanPhimDangNhap = String(hmiPtr->_set_event.maxValue, 1);
                }
            }
            hmiPtr->setText(_VPAddressKeyboardWarningText, "Max: " + hmiPtr->_ChuoiBanPhimDangNhap);
            ESP_LOGI(TAG, "Out of range.");
        }
        else if (hmiPtr->_ChuoiBanPhimDangNhap.toFloat() < hmiPtr->_set_event.minValue && hmiPtr->_set_event.minValue <= 0)
        {
            if (hmiPtr->_set_event.displayType == HMI_INT)
            {
                if (hmiPtr->_ChuoiBanPhimDangNhap.indexOf('+') == 0)
                {
                    hmiPtr->_ChuoiBanPhimDangNhap = "+" + String(hmiPtr->_set_event.minValue, 0);
                }
                else
                {
                    hmiPtr->_ChuoiBanPhimDangNhap = String(hmiPtr->_set_event.minValue, 0);
                }
            }
            else if (hmiPtr->_set_event.displayType == HMI_FLOAT)
            {
                if (hmiPtr->_ChuoiBanPhimDangNhap.indexOf('+') == 0)
                {
                    hmiPtr->_ChuoiBanPhimDangNhap = "+" + String(hmiPtr->_set_event.minValue, 1);
                }
                else
                {
                    hmiPtr->_ChuoiBanPhimDangNhap = String(hmiPtr->_set_event.minValue, 1);
                }
            }
            hmiPtr->setText(_VPAddressKeyboardWarningText, "Min: " + hmiPtr->_ChuoiBanPhimDangNhap);
            ESP_LOGI(TAG, "Out of range.");
        }
    }
    // Text
    else
    {
        if (hmiPtr->_CapslockEnable)
        {
            Serial.println(String(lastBytes, HEX));
            hmiPtr->_ChuoiBanPhimDangNhap += String((char)(lastBytes >> 8));
        }
        else
        {
            hmiPtr->_ChuoiBanPhimDangNhap += String((char)lastBytes);
        }
    }
    if (hmiPtr->_set_event.displayType == HMI_PASSWORD || hmiPtr->_set_event.displayType == HMI_EXTERNAL_PASSWORD)
    {
        String _strPass = "";
        for (int8_t i = 0; i < hmiPtr->_ChuoiBanPhimDangNhap.length(); i++)
        {
            _strPass += "*";
        }
        hmiPtr->DWIN::setText(hmiPtr->_set_event.VPTextDisplayWhenInput, _strPass);
    }
    else
    {
        hmiPtr->DWIN::setText(hmiPtr->_set_event.VPTextDisplayWhenInput, hmiPtr->_ChuoiBanPhimDangNhap);
    }
    Serial.println(hmiPtr->_ChuoiBanPhimDangNhap);
    return;
}
#pragma endregion

void HMI::_NutCaiNhietDoSetpoint_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = HMI_SET_SETPOINT;
    hmiPtr->_set_event.displayType = HMI_FLOAT;
    hmiPtr->_set_event.pageAfterReturn = _HomePage;
    hmiPtr->_set_event.pageAfterEnter = _HomePage;
    hmiPtr->_set_event.maxValue = 100;
    hmiPtr->_set_event.minValue = 20;
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressSetpointTempText;
    hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
    hmiPtr->_set_event.textLen = 5;
    hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 6);
    hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->DWIN::setPage(_TempNumericKeypadPage);
}

void HMI::_NutCaiTocDoQuat_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = HMI_SET_FAN;
    hmiPtr->_set_event.displayType = HMI_INT;
    hmiPtr->_set_event.pageAfterReturn = _HomePage;
    hmiPtr->_set_event.pageAfterEnter = _HomePage;
    hmiPtr->_set_event.maxValue = 100;
    hmiPtr->_set_event.minValue = 5;
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressFanSpeedText;
    hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
    hmiPtr->_set_event.textLen = 5;
    hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 6);
    hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->DWIN::setPage(_FanNumericKeypadPage);
}

void HMI::_NutCaiThoiGianTatMay_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_hmiGetDataCallback(HMI_GET_DELAYOFF, NULL);
    hmiPtr->_set_event.type = UNDEFINED;
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->setPage(_SetDelayOffPage);
}

void HMI::_NutEnterCaiThoiGianTatMay_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = HMI_SET_DELAYOFF;
    hmiPtr->_set_event.u32_value = hmiPtr->getText(_VPAddressDayText, 2).toInt() * 86400 +
        hmiPtr->getText(_VPAddressHourText, 2).toInt() * 3600 +
        hmiPtr->getText(_VPAddressMinuteText, 2).toInt() * 60;
    hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
    hmiPtr->setPage(_HomePage);
}

void HMI::_NutEditSetpointTrangSegment_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    ESP_LOGI(TAG, "Select %d\n", lastBytes);
    hmiPtr->_set_event.type = HMI_EDIT_SEG_SETPOINT;
    hmiPtr->_set_event.displayType = HMI_FLOAT;
    hmiPtr->_set_event.pageAfterReturn = _SegmentAdjPage;
    hmiPtr->_set_event.pageAfterEnter = _SegmentAdjPage;
    hmiPtr->_set_event.maxValue = 100;
    hmiPtr->_set_event.minValue = 20;
    hmiPtr->_set_event.textLen = 5;
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressSegmentSetpointText1 + lastBytes * 5;
    hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
    hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 6);
    hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->_set_event.indexList = lastBytes;
    if (hmiPtr->_hmiGetDataCallback(HMI_CHECK_LIST, (void*)&hmiPtr->_set_event.indexList) == true)
    {
        hmiPtr->DWIN::setPage(_TempNumericKeypadPage);
    }
}

void HMI::_NutEditTocDoQuatTrangSegment_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    ESP_LOGI(TAG, "Select %d\n", lastBytes);
    hmiPtr->_set_event.type = HMI_EDIT_SEG_FANSPEED;
    hmiPtr->_set_event.displayType = HMI_INT;
    hmiPtr->_set_event.pageAfterReturn = _SegmentAdjPage;
    hmiPtr->_set_event.pageAfterEnter = _SegmentAdjPage;
    hmiPtr->_set_event.maxValue = 100;
    hmiPtr->_set_event.minValue = 5;
    hmiPtr->_set_event.textLen = 5;
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressSegmentFanSpeedText1 + lastBytes * 5;
    hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
    hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 6);
    hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->_set_event.indexList = lastBytes;
    if (hmiPtr->_hmiGetDataCallback(HMI_CHECK_LIST, (void*)&hmiPtr->_set_event.indexList) == true)
    {
        hmiPtr->DWIN::setPage(_FanNumericKeypadPage);
    }
}

// void HMI::_NutEditFlapTrangSegment_(int32_t lastBytes, void* args)
// {
//     HMI* hmiPtr = (HMI*)args;
//     hmiPtr->_set_event.type = HMI_EDIT_SEG_AIRFLAP;
//     hmiPtr->_set_event.displayType = HMI_INT;
//     hmiPtr->_set_event.pageAfterReturn = _SegmentAdjPage;
//     hmiPtr->_set_event.pageAfterEnter = _SegmentAdjPage;
//     hmiPtr->_set_event.maxValue = 100;
//     hmiPtr->_set_event.minValue = 0;
//     hmiPtr->_set_event.textLen = 5;
//     hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressSegmentAirFlapText1 + lastBytes * 5;
//     hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
//     hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 6);
//     hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
//     hmiPtr->setText(_VPAddressKeyboardWarningText, "");
//     hmiPtr->_set_event.indexList = lastBytes;
//     if (hmiPtr->_hmiGetDataCallback(HMI_CHECK_LIST, (void*)&hmiPtr->_set_event.indexList) == true)
//     {
//         hmiPtr->DWIN::setPage(_FlapNumericKeypadPage);
//     }
// }

void HMI::_NutEditTempMinTrangSegment_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    ESP_LOGI(TAG, "Select %d\n", lastBytes);
    hmiPtr->_set_event.type = HMI_EDIT_SEG_TEMPMIN;
    hmiPtr->_set_event.displayType = HMI_FLOAT;
    hmiPtr->_set_event.pageAfterReturn = _SegmentAdjPage;
    hmiPtr->_set_event.pageAfterEnter = _SegmentAdjPage;
    hmiPtr->_set_event.maxValue = -0.1;
    hmiPtr->_set_event.minValue = -99;
    hmiPtr->_set_event.textLen = 5;
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressSegmentTempMinText1 + lastBytes * 5;
    hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
    hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 6);
    hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->_set_event.indexList = lastBytes;
    if (hmiPtr->_hmiGetDataCallback(HMI_CHECK_LIST, (void*)&hmiPtr->_set_event.indexList) == true)
    {
        hmiPtr->DWIN::setPage(_NumericKeypadPage);
    }
}
void HMI::_NutEditTempMaxTrangSegment_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    ESP_LOGI(TAG, "Select %d\n", lastBytes);
    hmiPtr->_set_event.type = HMI_EDIT_SEG_TEMPMAX;
    hmiPtr->_set_event.displayType = HMI_FLOAT;
    hmiPtr->_set_event.pageAfterReturn = _SegmentAdjPage;
    hmiPtr->_set_event.pageAfterEnter = _SegmentAdjPage;
    hmiPtr->_set_event.maxValue = 99;
    hmiPtr->_set_event.minValue = 0.1;
    hmiPtr->_set_event.textLen = 5;
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressSegmentTempMaxText1 + lastBytes * 5;
    hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
    hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 6);
    hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->_set_event.indexList = lastBytes;
    if (hmiPtr->_hmiGetDataCallback(HMI_CHECK_LIST, (void*)&hmiPtr->_set_event.indexList) == true)
    {
        hmiPtr->DWIN::setPage(_NumericKeypadPage);
    }
}

void HMI::_NutChonSegment_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    ESP_LOGI(TAG, "Select %d\n", lastBytes);
    hmiPtr->_set_event.indexList = lastBytes;
    if (hmiPtr->_hmiGetDataCallback(HMI_CHECK_LIST, (void*)&hmiPtr->_set_event.indexList) == true)
    {
        if (hmiPtr->_ChiMucPhanDoanTruocDo != lastBytes)
        {
            hmiPtr->setVP(_VPAddressSegmentIconTick1 + lastBytes, true);
            if (hmiPtr->_ChiMucPhanDoanTruocDo != 0xffff)
            {
                hmiPtr->setVP(_VPAddressSegmentIconTick1 + hmiPtr->_ChiMucPhanDoanTruocDo, false);
            }
            hmiPtr->_ChiMucPhanDoanTruocDo = lastBytes;
        }
        else
        {
            hmiPtr->setVP(_VPAddressSegmentIconTick1 + lastBytes, false);
            hmiPtr->_ChiMucPhanDoanTruocDo = 0xffff;
        }
    }
}

// void HMI::_SegmentButton_(int32_t lastBytes, void *args)
// {
//     HMI *hmiPtr = (HMI*)args;
//     hmiPtr->_hmiGetDataCallback(HMI_GET_SEGMENT_LIST, hmiPtr);
//     hmiPtr->DWIN::setPage(21);
// }

void HMI::_CacNutThaoTacTrongTrangEditSegment_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    switch (lastBytes)
    {
    case _KeyValueSegmentAddButton:
        hmiPtr->_set_event.type = HMI_ADD_SEG;
        hmiPtr->_set_event.indexList = hmiPtr->_ChiMucPhanDoanTruocDo;
        hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
        hmiPtr->_hmiGetDataCallback(HMI_REFRESH_SEGMENT_LIST, NULL);
        ESP_LOGI(TAG, "%s", __func__);
        break;
    case _KeyValueSegmentSubButton:
        hmiPtr->_set_event.type = HMI_SUB_SEG;
        hmiPtr->_set_event.indexList = hmiPtr->_ChiMucPhanDoanTruocDo;
        hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
        hmiPtr->_hmiGetDataCallback(HMI_REFRESH_SEGMENT_LIST, NULL);
        ESP_LOGI(TAG, "%s", __func__);
        break;
    case _KeyValueSegmentUpButton:
        hmiPtr->_hmiGetDataCallback(HMI_GET_NEXT_SEGMENT_LIST, hmiPtr);
        ESP_LOGI(TAG, "%s", __func__);
        break;
    case _KeyValueSegmentDownButton:
        hmiPtr->_hmiGetDataCallback(HMI_GET_BACK_SEGMENT_LIST, hmiPtr);
        ESP_LOGI(TAG, "%s", __func__);
        break;
    case _KeyValueSegmentEnterButton:
        hmiPtr->_set_event.type = HMI_SAVE_SEG;
        hmiPtr->_set_event.text = hmiPtr->getText(_VPAddressCurrentProgramNameText, 16);
        hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
        hmiPtr->setPage(_ProgramPage);
        hmiPtr->_hmiGetDataCallback(HMI_GET_PROGRAM_LIST, NULL);
        ESP_LOGI(TAG, "%s", __func__);
        break;
    case _KeyValueSegmentEditProgramNameButton:
        hmiPtr->_set_event.type = UNDEFINED;
        hmiPtr->_set_event.displayType = HMI_TEXT;
        hmiPtr->_set_event.pageAfterEnter = _SegmentAdjPage;
        hmiPtr->_set_event.pageAfterReturn = hmiPtr->_set_event.pageAfterEnter;
        hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressCurrentProgramNameText;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
        hmiPtr->_set_event.textLen = 10;
        hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(_VPAddressCurrentProgramNameText, 16);
        hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
        hmiPtr->setText(_VPAddressKeyboardWarningText, "");
        hmiPtr->setPage(_KeyboardPage);
        break;
    default:
        break;
    }
    return;
}

void HMI::HienThiDuLieuSegmentTrenHang(uint8_t row, uint8_t index, float setpoint, int delayOffDay, int delayOffHour, int delayOffMinute, int fanSpeed, int flap, float tempMin, float tempMax)
{
    this->setText(_VPAddressNumSegmentText1 + row * 2, String(index));
    this->setText(_VPAddressSegmentSetpointText1 + row * 5, String(setpoint, 1));
    char timeString[20];
    sprintf(timeString, "%02dd-%02d:%02d", delayOffDay, delayOffHour, delayOffMinute);
    this->setText(_VPAddressSegmentDelayOffText1 + row * 20, String(timeString));
    this->setText(_VPAddressSegmentFanSpeedText1 + row * 5, String(fanSpeed));
    // this->setText(_VPAddressSegmentAirFlapText1 + row * 5, String(flap));
    this->setText(_VPAddressSegmentTempMinText1 + row * 5, String(tempMin, 1));
    this->setText(_VPAddressSegmentTempMaxText1 + row * 5, String(tempMax, 1));
    this->setVP(_VPAddressSegmentIconTick1 + row, false);
    this->_ChiMucPhanDoanTruocDo = 0xffff;
}

void HMI::XoaDuLieuHienThiSegmentTrenHang(uint8_t row)
{
    this->setText(_VPAddressNumSegmentText1 + row * 2, "");
    this->setText(_VPAddressSegmentSetpointText1 + row * 5, "");
    this->setText(_VPAddressSegmentDelayOffText1 + row * 20, "");
    this->setText(_VPAddressSegmentFanSpeedText1 + row * 5, "");
    // this->setText(_VPAddressSegmentAirFlapText1 + row * 5, "");
    this->setText(_VPAddressSegmentTempMinText1 + row * 5, "");
    this->setText(_VPAddressSegmentTempMaxText1 + row * 5, "");
    this->setVP(_VPAddressSegmentIconTick1 + row, false);
    this->_ChiMucPhanDoanTruocDo = 0xffff;
}

void HMI::_NutEditThoiGianTatTrangSegment_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.indexList = lastBytes;
    hmiPtr->_set_event.type = UNDEFINED;
    if (hmiPtr->_hmiGetDataCallback(HMI_CHECK_LIST, (void*)&hmiPtr->_set_event.indexList) == true)
    {
        hmiPtr->_hmiGetDataCallback(HMI_GET_SEGMENT_DELAYOFF, (void*)&hmiPtr->_set_event.indexList);
        hmiPtr->setText(_VPAddressKeyboardWarningText, "");
        hmiPtr->DWIN::setPage(_SetSegmentDelayOffPage);
    }
}

void HMI::HienThiNgay(int ngay)
{
    this->setText(_VPAddressDayText, String(ngay));
}

void HMI::HienThiThang(int thang)
{
    this->setText(_VPAddressMonthText, String(thang));
}

void HMI::HienThiNam(int nam)
{
    this->setText(_VPAddressYearText, String(nam));
}

void HMI::HienThiGio(int gio)
{
    this->setText(_VPAddressHourText, String(gio));
}

void HMI::HienThiPhut(int phut)
{
    this->setText(_VPAddressMinuteText, String(phut));
}

void HMI::_NutVaoTrangNhapNam_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = UNDEFINED;
    hmiPtr->_set_event.displayType = HMI_INT;
    hmiPtr->_set_event.minValue = MIN_INPUT_YEAR_RTC;
    hmiPtr->_set_event.maxValue = MAX_INPUT_YEAR_RTC;
    hmiPtr->_set_event.pageAfterReturn = hmiPtr->_set_event.pageAfterEnter = hmiPtr->getPage();
    if (hmiPtr->_set_event.pageAfterEnter != _RTCPage)
    {
        return;
    }
    hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressYearText;
    hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 4);
    hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->_set_event.textLen = 4;
    hmiPtr->setPage(_NumericKeypadPage);
}

void HMI::_NutVaoTrangNhapThang_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = UNDEFINED;
    hmiPtr->_set_event.displayType = HMI_INT;
    hmiPtr->_set_event.minValue = MIN_INPUT_MONTH_RTC;
    hmiPtr->_set_event.maxValue = MAX_INPUT_MONTH_RTC;
    hmiPtr->_set_event.pageAfterReturn = hmiPtr->_set_event.pageAfterEnter = hmiPtr->getPage();
    if (hmiPtr->_set_event.pageAfterEnter != _RTCPage)
    {
        return;
    }
    hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressMonthText;
    hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 2);
    hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->_set_event.textLen = 2;
    hmiPtr->setPage(_NumericKeypadPage);
}

void HMI::_NutVaoTrangNhapNgay_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = UNDEFINED;
    hmiPtr->_set_event.displayType = HMI_INT;
    hmiPtr->_set_event.pageAfterReturn = hmiPtr->_set_event.pageAfterEnter = hmiPtr->getPage();
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressDayText;
    hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 2);
    hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->_set_event.textLen = 2;
    if (hmiPtr->_set_event.pageAfterEnter == _RTCPage)
    {
        hmiPtr->_set_event.minValue = MIN_INPUT_DAY_RTC;
        hmiPtr->_set_event.maxValue = MAX_INPUT_DAY_RTC;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
        hmiPtr->setPage(_NumericKeypadPage);
    }
    else if (hmiPtr->_set_event.pageAfterEnter == _SetDelayOffPage || hmiPtr->_set_event.pageAfterEnter == _SetSegmentDelayOffPage)
    {
        hmiPtr->_set_event.minValue = MIN_INPUT_DAY_DELAYOFF;
        hmiPtr->_set_event.maxValue = MAX_INPUT_DAY_DELAYOFF;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressDayText;
        // hmiPtr->setPage(150);
    }
    else
    {
        return;
    }
}

void HMI::_NutVaoTrangNhapGio_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = UNDEFINED;
    hmiPtr->_set_event.displayType = HMI_INT;
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressHourText;
    hmiPtr->_set_event.pageAfterReturn = hmiPtr->_set_event.pageAfterEnter = hmiPtr->getPage();
    hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 2);
    hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->_set_event.textLen = 2;
    if (hmiPtr->_set_event.pageAfterEnter == _RTCPage)
    {
        hmiPtr->_set_event.minValue = MIN_INPUT_HOUR_RTC;
        hmiPtr->_set_event.maxValue = MAX_INPUT_HOUR_RTC;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
        hmiPtr->setPage(_NumericKeypadPage);
    }
    else if (hmiPtr->_set_event.pageAfterEnter == _TrangCaiThoiGianTietTrung)
    {
        hmiPtr->_set_event.minValue = MIN_INPUT_HOUR_STERILIZATION_DELAYOFF;
        hmiPtr->_set_event.maxValue = MAX_INPUT_HOUR_STERILIZATION_DELAYOFF;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressHourText;
    }
    else if (hmiPtr->_set_event.pageAfterEnter == _SetDelayOffPage || hmiPtr->_set_event.pageAfterEnter == _SetSegmentDelayOffPage)
    {
        hmiPtr->_set_event.minValue = MIN_INPUT_HOUR_DELAYOFF;
        hmiPtr->_set_event.maxValue = MAX_INPUT_HOUR_DELAYOFF;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressHourText;
    }
    else
    {
        return;
    }
}

void HMI::_NutVaoTrangNhapPhut_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = UNDEFINED;
    hmiPtr->_set_event.displayType = HMI_INT;
    hmiPtr->_set_event.pageAfterReturn = hmiPtr->_set_event.pageAfterEnter = hmiPtr->getPage();
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressMinuteText;
    hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 2);
    hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->_set_event.textLen = 2;
    if (hmiPtr->_set_event.pageAfterEnter == _RTCPage)
    {
        hmiPtr->_set_event.minValue = MIN_INPUT_MINUTE_RTC;
        hmiPtr->_set_event.maxValue = MAX_INPUT_MINUTE_RTC;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
        hmiPtr->setPage(_NumericKeypadPage);
    }
    else if (hmiPtr->_set_event.pageAfterEnter == _TrangCaiThoiGianTietTrung)
    {
        hmiPtr->_set_event.minValue = MIN_INPUT_MINUTE_STERILIZATION_DELAYOFF;
        hmiPtr->_set_event.maxValue = MAX_INPUT_MINUTE_STERILIZATION_DELAYOFF;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressMinuteText;
    }
    else if (hmiPtr->_set_event.pageAfterEnter == _SetDelayOffPage || hmiPtr->_set_event.pageAfterEnter == _SetSegmentDelayOffPage)
    {
        hmiPtr->_set_event.minValue = MIN_INPUT_MINUTE_DELAYOFF;
        hmiPtr->_set_event.maxValue = MAX_INPUT_MINUTE_DELAYOFF;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressMinuteText;
    }
    else
    {
        return;
    }
}

void HMI::_NutEnterTrongCaiDatThoiGianTatCuaSegment_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;

    int Day = hmiPtr->getText(_VPAddressDayText, 2).toInt();
    if (Day > MAX_INPUT_DAY_DELAYOFF)
    {
        return;
    }
    hmiPtr->_set_event.type = HMI_EDIT_SEG_DELAYOFF_DAY;
    hmiPtr->_set_event.u32_value = Day;
    hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);

    int Hour = hmiPtr->getText(_VPAddressHourText, 2).toInt();
    if (Hour > MAX_INPUT_HOUR_DELAYOFF)
    {
        return;
    }
    hmiPtr->_set_event.type = HMI_EDIT_SEG_DELAYOFF_HOUR;
    hmiPtr->_set_event.u32_value = Hour;
    hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);

    int Minute = hmiPtr->getText(_VPAddressMinuteText, 2).toInt();
    if (Minute > MAX_INPUT_MINUTE_DELAYOFF)
    {
        return;
    }
    hmiPtr->_set_event.type = HMI_EDIT_SEG_DELAYOFF_MINUTE;
    hmiPtr->_set_event.u32_value = Minute;
    hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);

    char strTime[15];
    sprintf(strTime, "%02dd-%02d:%02d", Day, Hour, Minute);
    hmiPtr->setText(_VPAddressSegmentDelayOffText1 + hmiPtr->_set_event.indexList * 20, strTime);

    hmiPtr->setPage(_SegmentAdjPage);
}

void HMI::_NutVaoChucNangCalib_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = UNDEFINED;
    hmiPtr->_set_event.displayType = HMI_PASSWORD;
    hmiPtr->_set_event.pageAfterEnter = _CalibTempPage;
    hmiPtr->_set_event.pageAfterReturn = _CalibChoosePage;
    hmiPtr->_set_event.textLen = 10;
    hmiPtr->_ChuoiBanPhimDangNhap = "";
    hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressKeyboardInputText;
    hmiPtr->setText(_VPAddressKeyboardInputText, "Password ?");
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->_hmiGetDataCallback(HMI_GET_CALIB, NULL);
    hmiPtr->setPage(_KeyboardPage);
}

void HMI::_NutResetHeSoCalib_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = HMI_RESET_CALIB;
    hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
    hmiPtr->_hmiGetDataCallback(HMI_GET_CALIB, NULL);
    hmiPtr->setPage(_HomePage);
}

void HMI::_NutEditCalib_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = UNDEFINED;
    hmiPtr->_set_event.displayType = HMI_FLOAT;
    hmiPtr->_set_event.pageAfterEnter = _CalibTempPage;
    hmiPtr->_set_event.pageAfterReturn = _CalibChoosePage;
    hmiPtr->_set_event.maxValue = 900;
    hmiPtr->_set_event.minValue = 0;
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressStdTempText;
    hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
    hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 6);
    hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->_set_event.textLen = 5;
    hmiPtr->setPage(_NumericKeypadPage);
}

void HMI::_NutEnterTrangCalib_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = HMI_SET_CALIB;
    hmiPtr->_set_event.displayType = HMI_FLOAT;
    hmiPtr->_set_event.f_value = hmiPtr->getText(_VPAddressStdTempText, 6).toFloat();
    hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
    hmiPtr->setPage(_SettingsPage);
}

void HMI::_NutSetFlap_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = HMI_SET_FLAP;
    hmiPtr->_set_event.displayType = HMI_INT;
    hmiPtr->_set_event.pageAfterReturn = hmiPtr->_set_event.pageAfterEnter = _HomePage;
    hmiPtr->_set_event.maxValue = 100;
    hmiPtr->_set_event.minValue = 0;
    // hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressFlapText;
    hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
    hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 6);
    hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->_set_event.textLen = 5;
    hmiPtr->setPage(_FlapNumericKeypadPage);
}

// Khong dung
// void HMI::_NutEnterTrangFlap_(int32_t lastBytes, void *args)
// {
//     HMI *hmiPtr = (HMI*)args;
//     hmiPtr->_set_event.type = HMI_SET_FLAP;
//     hmiPtr->_set_event.displayType = HMI_INT;
//     hmiPtr->_set_event.f_value = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 6).toFloat();
//     hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);

//     hmiPtr->setPage(_SettingsPage);
// }

void HMI::_NutCaiDatThoiGianRTC_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_hmiGetDataCallback(HMI_GET_RTC, NULL); // Yêu cầu main gửi thời gian RTC lên HMI
    hmiPtr->setPage(_RTCPage);
}

void HMI::_NutEnterTrangCaiRTC_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    int Day = hmiPtr->getText(_VPAddressDayText, 2).toInt();
    int Month = hmiPtr->getText(_VPAddressMonthText, 2).toInt();
    int Year = hmiPtr->getText(_VPAddressYearText, 4).toInt();
    int Hour = hmiPtr->getText(_VPAddressHourText, 2).toInt();
    int Minute = hmiPtr->getText(_VPAddressMinuteText, 2).toInt();
    DateTime dt(Year, Month, Day, Hour, Minute, 0);
    hmiPtr->_set_event.type = HMI_SET_RTC;
    hmiPtr->_set_event.u32_value = dt.unixtime();
    Serial.printf("Unix time: %d\n", dt.unixtime());
    hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
    hmiPtr->setPage(_SettingsPage);
}

void HMI::_NutCaiCanhBaoNhietDo_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_hmiGetDataCallback(HMI_GET_ALARM, NULL); // Yêu cầu main gửi thông số Alarm đang lưu

    hmiPtr->setPage(_AlarmPage);
}

void HMI::HienThiNhietDoCanhBao(float NhietDuoi, float NhietTren)
{
    if (String(NhietDuoi).indexOf('-') >= 0)
    {
        this->setText(_VPAddressAlarmBelowTempText, String(NhietDuoi, 1));
    }
    else
    {
        this->setText(_VPAddressAlarmBelowTempText, "-" + String(NhietDuoi, 1));
    }
    this->setText(_VPAddressAlarmAboveTempText, "+" + String(NhietTren, 1));
}

void HMI::_NutCaiCanhBaoNhietDoThap_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = UNDEFINED;
    hmiPtr->_set_event.displayType = HMI_FLOAT;
    hmiPtr->_set_event.minValue = -99;
    hmiPtr->_set_event.maxValue = -0.1;
    hmiPtr->_set_event.pageAfterReturn = hmiPtr->_set_event.pageAfterEnter = _AlarmPage;
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressAlarmBelowTempText;
    hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
    hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 6);
    // Serial.println("Below" + hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->_set_event.textLen = 5;
    hmiPtr->setPage(_NumericKeypadPage);
}

void HMI::_NutCaiCanhBaoNhietDoCao_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = UNDEFINED;
    hmiPtr->_set_event.displayType = HMI_FLOAT;
    hmiPtr->_set_event.minValue = 0.1;
    hmiPtr->_set_event.maxValue = 99;
    hmiPtr->_set_event.pageAfterReturn = hmiPtr->_set_event.pageAfterEnter = _AlarmPage;
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressAlarmAboveTempText;
    hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
    hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(hmiPtr->_set_event.VPTextDisplayAfterEnter, 6);
    // Serial.println("Above" + hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->_set_event.textLen = 5;
    hmiPtr->setPage(_NumericKeypadPage);
}

void HMI::_NutEnterTrangCaiCanhBaoNhietDo_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;

    float value = hmiPtr->getText(_VPAddressAlarmBelowTempText, 5).toFloat();
    hmiPtr->_set_event.type = HMI_SET_ALARM_BELOW;
    hmiPtr->_set_event.f_value = value;
    hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);

    value = hmiPtr->getText(_VPAddressAlarmAboveTempText, 5).toFloat();
    hmiPtr->_set_event.type = HMI_SET_ALARM_ABOVE;
    hmiPtr->_set_event.f_value = value;
    hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);

    hmiPtr->setPage(_SettingsPage);
}

void HMI::HienThiTenChuongTrinhTrenHang(uint8_t row, uint8_t index, String name, uint8_t TotalSeg)
{
    this->setText(_VPAddressNumProgramText1 + row * 2, String(index));
    this->setText(_VPAddressProgramNameText1 + row * 20, name);
    this->setTextColor(_SPAddressProgramNameText1 + row * 0x0010, 0x03, _BlueColor);
    this->setText(_VPAddressTotalNumOfSegmentsText1 + row * 2, String(TotalSeg));
    this->_TenChuongTrinh = "";
    this->_TenChuongTrinhTruocDo = "";
}

void HMI::XoaDuLieuHienThiTenChuongTrinhTrenHang(uint8_t row)
{
    this->setText(_VPAddressNumProgramText1 + row * 2, "");
    this->setText(_VPAddressProgramNameText1 + row * 20, "");
    this->setText(_VPAddressTotalNumOfSegmentsText1 + row * 2, "");
}

void HMI::_NutVaoChucNangProgram_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_hmiGetDataCallback(HMI_GET_PROGRAM_LIST, NULL);
    hmiPtr->_TenChuongTrinh = "";
    hmiPtr->_TenChuongTrinhTruocDo = "";
    hmiPtr->setPage(_ProgramPage);
}

void HMI::_NutChonProgram_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;

    hmiPtr->_TenChuongTrinh = hmiPtr->getText(_VPAddressProgramNameText1 + lastBytes * 20, 20);
    if (hmiPtr->_TenChuongTrinhTruocDo != hmiPtr->_TenChuongTrinh)
    {
        hmiPtr->setTextColor(_SPAddressProgramNameText1 + lastBytes * 0x0010, 0x03, _RedColor);
        if (hmiPtr->_ChiMucChuongTrinhTruocDo != 0xFFFF && lastBytes != hmiPtr->_ChiMucChuongTrinhTruocDo)
        {
            hmiPtr->setTextColor(_SPAddressProgramNameText1 + hmiPtr->_ChiMucChuongTrinhTruocDo * 0x0010, 0x03, _BlueColor);
        }
        hmiPtr->_ChiMucChuongTrinhTruocDo = lastBytes;
        hmiPtr->_TenChuongTrinhTruocDo = hmiPtr->_TenChuongTrinh;
    }
    else
    {
        hmiPtr->_TenChuongTrinh = "";
        hmiPtr->_TenChuongTrinhTruocDo = "";
        hmiPtr->setTextColor(_SPAddressProgramNameText1 + hmiPtr->_ChiMucChuongTrinhTruocDo * 0x0010, 0x03, _BlueColor);
        hmiPtr->_ChiMucChuongTrinhTruocDo = 0xFFFF;
    }
    Serial.println(hmiPtr->_TenChuongTrinh);
}

void HMI::_CacNutThaoTacTrongTrangProgram_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    Serial.printf("Key value: %d\n", lastBytes);
    switch (lastBytes)
    {
    case _KeyValueProgramUp:
        hmiPtr->_hmiGetDataCallback(HMI_GET_NEXT_PROGRAM_LIST, NULL);

        break;
    case _KeyValueProgramDown:
        hmiPtr->_hmiGetDataCallback(HMI_GET_BACK_PROGRAM_LIST, NULL);

        break;
    case _KeyValueProgramAdd:
        hmiPtr->_set_event.type = HMI_ADD_PROGRAM;
        hmiPtr->_set_event.displayType = HMI_TEXT;
        hmiPtr->_set_event.textLen = 10;
        hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressCurrentProgramNameText;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
        hmiPtr->_set_event.pageAfterReturn = _ProgramPage;
        hmiPtr->_set_event.pageAfterEnter = _SegmentAdjPage;
        hmiPtr->_ChuoiBanPhimDangNhap = "";
        hmiPtr->setText(_VPAddressKeyboardInputText, "");
        hmiPtr->setText(_VPAddressKeyboardWarningText, "");
        // hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
        // hmiPtr->_hmiGetDataCallback(HMI_GET_PROGRAM_LIST, NULL);
        hmiPtr->DWIN::setPage(_KeyboardPage);
        hmiPtr->_TenChuongTrinh = "";
        break;
    case _KeyValueProgramDelete:
        hmiPtr->_set_event.type = HMI_DELETE_PROGRAM;
        if (hmiPtr->_TenChuongTrinh == "")
        {
            break;
        }
        hmiPtr->_set_event.text = hmiPtr->_TenChuongTrinh;
        hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
        hmiPtr->_hmiGetDataCallback(HMI_GET_PROGRAM_LIST, NULL);
        hmiPtr->_TenChuongTrinh = "";
        break;
    case _KeyValueProgramEdit:
        if (hmiPtr->_TenChuongTrinh != "")
        {
            hmiPtr->_set_event.type = HMI_SELECT_PROGRAM;
            hmiPtr->_set_event.text = hmiPtr->_TenChuongTrinh;
            hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
            hmiPtr->_hmiGetDataCallback(HMI_GET_SEGMENT_LIST, NULL);
            hmiPtr->setPage(_SegmentAdjPage);
        }
        break;
    case _KeyValueProgramView:
        if (hmiPtr->_TenChuongTrinh != "")
        {
            hmiPtr->_set_event.type = HMI_SELECT_PROGRAM;
            hmiPtr->_set_event.text = hmiPtr->_TenChuongTrinh;
            hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
            hmiPtr->_hmiGetDataCallback(HMI_GET_SEGMENT_LIST, NULL);
            hmiPtr->setPage(_SegmentViewPage);
        }
        break;
    case _KeyValueProgramQuick:
        hmiPtr->_set_event.type = HMI_RUN_QUICK_MODE;
        hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
        hmiPtr->setPage(_HomePage);
        break;
    default:
        break;
    }
}

void HMI::_NutThayDoiTrangThaiChucNangHenGioTat_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = HMI_SET_DELAYOFF_ONOFF;
    hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
}

void HMI::VeDoThi(float value, time_t time, int offset)
{
    value = value * 10;
    char timeText[10];
    if (_DuLieuDoThiNhietDo.minValue > value)
    {
        _DuLieuDoThiNhietDo.minValue = value;
        _DuLieuDoThiNhietDo.VDCentral = (_DuLieuDoThiNhietDo.minValue + _DuLieuDoThiNhietDo.maxValue) / 2;

        _DuLieuDoThiNhietDo.MulY = 150 * 256 / (_DuLieuDoThiNhietDo.maxValue - _DuLieuDoThiNhietDo.minValue);
        setGraphVDCentral(_SPAddressSmallGraph1, _DuLieuDoThiNhietDo.VDCentral);
        setGraphMulY(_SPAddressSmallGraph1, _DuLieuDoThiNhietDo.MulY);

        _DuLieuDoThiNhietDo.MulY = 190 * 256 / (_DuLieuDoThiNhietDo.maxValue - _DuLieuDoThiNhietDo.minValue);
        setGraphVDCentral(_SPAddressLargeGraph, _DuLieuDoThiNhietDo.VDCentral);
        setGraphMulY(_SPAddressLargeGraph, _DuLieuDoThiNhietDo.MulY);

        _DuLieuDoThiNhietDo.valueStep = (_DuLieuDoThiNhietDo.maxValue - _DuLieuDoThiNhietDo.minValue) / 5;
        setText(_VPAddressGraphYValueText1, String(_DuLieuDoThiNhietDo.minValue / 10, 1));
        for (float i = 1; i < 6; i++)
        {
            setText(_VPAddressGraphYValueText1 + i * 5, String((_DuLieuDoThiNhietDo.minValue + i * _DuLieuDoThiNhietDo.valueStep) / 10, 1));
        }
    }
    else if (_DuLieuDoThiNhietDo.maxValue < value)
    {
        _DuLieuDoThiNhietDo.maxValue = (value - _DuLieuDoThiNhietDo.minValue) * 1.25 + _DuLieuDoThiNhietDo.minValue;
        _DuLieuDoThiNhietDo.VDCentral = (_DuLieuDoThiNhietDo.minValue + _DuLieuDoThiNhietDo.maxValue) / 2;

        _DuLieuDoThiNhietDo.MulY = 150 * 256 / (_DuLieuDoThiNhietDo.maxValue - _DuLieuDoThiNhietDo.minValue);
        setGraphVDCentral(_SPAddressSmallGraph1, _DuLieuDoThiNhietDo.VDCentral);
        setGraphMulY(_SPAddressSmallGraph1, _DuLieuDoThiNhietDo.MulY);

        _DuLieuDoThiNhietDo.MulY = 190 * 256 / (_DuLieuDoThiNhietDo.maxValue - _DuLieuDoThiNhietDo.minValue);
        setGraphVDCentral(_SPAddressLargeGraph, _DuLieuDoThiNhietDo.VDCentral);
        setGraphMulY(_SPAddressLargeGraph, _DuLieuDoThiNhietDo.MulY);

        _DuLieuDoThiNhietDo.valueStep = (_DuLieuDoThiNhietDo.maxValue - _DuLieuDoThiNhietDo.minValue) / 5;
        setText(_VPAddressGraphYValueText1, String(_DuLieuDoThiNhietDo.minValue / 10, 1));
        for (float i = 1; i < 6; i++)
        {
            setText(_VPAddressGraphYValueText1 + i * 5, String((_DuLieuDoThiNhietDo.minValue + i * _DuLieuDoThiNhietDo.valueStep) / 10, 1));
        }
    }
    _DuLieuDoThiNhietDo.count++;
    if (_DuLieuDoThiNhietDo.count % 30 == 0)
    {
        if (_DuLieuDoThiNhietDo.flag == 0)
        {
            _DuLieuDoThiNhietDo.timeArr[(_DuLieuDoThiNhietDo.count / 30 - 1)] = time;
            sprintf(timeText, "%02u:%02u:%02u", hour(time), minute(time), second(time));
            setText(_VPAddressGraphXValueText1 + 10 * (_DuLieuDoThiNhietDo.count / 30 - 1), timeText);
        }
        // else if (flag == 1)
        // {
        //     for(int8_t i=0; i<8; i++)
        //     {
        //         timeArr[i] = timeArr[i+1];
        //         sprintf(timeText, "%02u:%02u:%02u", hour(timeArr[i]), minute(timeArr[i]), second(timeArr[i]));
        //         setText(_VPAddressGraphXValueText1 + 10*i, timeText);
        //     }
        //     timeArr[8] = time;
        //     sprintf(timeText, "%02u:%02u:%02u", hour(timeArr[8]), minute(timeArr[8]), second(timeArr[8]));
        //     setText(_VPAddressGraphXValueText1 + 10*8, timeText);
        // }
    }
    else if (_DuLieuDoThiNhietDo.flag == 1)
    {
        for (int8_t i = 0; i < 9; i++)
        {
            _DuLieuDoThiNhietDo.timeArr[i] = _DuLieuDoThiNhietDo.timeArr[i] + offset;
            sprintf(timeText, "%02u:%02u:%02u", hour(_DuLieuDoThiNhietDo.timeArr[i]), minute(_DuLieuDoThiNhietDo.timeArr[i]), second(_DuLieuDoThiNhietDo.timeArr[i]));
            setText(_VPAddressGraphXValueText1 + 10 * i, timeText);
        }
        if (second(time) != second(_DuLieuDoThiNhietDo.timeArr[8]))
        {
            Serial.println("update HMI_DATE RTC for graph");
            _DuLieuDoThiNhietDo.timeArr[8] = time;
            for (int8_t i = 7; i >= 0; i--)
            {
                _DuLieuDoThiNhietDo.timeArr[i] = _DuLieuDoThiNhietDo.timeArr[i + 1] - 30 * offset;
            }
        }
    }
    if (_DuLieuDoThiNhietDo.count / 30 == 9)
    {
        _DuLieuDoThiNhietDo.count = 0;
        _DuLieuDoThiNhietDo.flag = 1;
    }
    sendGraphValue(0, (int)value);
}

void HMI::XoaDoThi(void)
{
    _DuLieuDoThiNhietDo.count = 0;
    _DuLieuDoThiNhietDo.flag = 0;
    _DuLieuDoThiNhietDo.valueStep = 0;
    _DuLieuDoThiNhietDo.minValue = 200;
    _DuLieuDoThiNhietDo.maxValue = 400;
    for (int8_t i = 0; i < 9; i++)
    {
        setText(_VPAddressGraphXValueText1 + 10 * i, "");
    }
    resetGraph(0);

    _DuLieuDoThiNhietDo.VDCentral = (_DuLieuDoThiNhietDo.minValue + _DuLieuDoThiNhietDo.maxValue) / 2;

    _DuLieuDoThiNhietDo.MulY = 150 * 256 / (_DuLieuDoThiNhietDo.maxValue - _DuLieuDoThiNhietDo.minValue);
    setGraphVDCentral(_SPAddressSmallGraph1, _DuLieuDoThiNhietDo.VDCentral);
    setGraphMulY(_SPAddressSmallGraph1, _DuLieuDoThiNhietDo.MulY);

    _DuLieuDoThiNhietDo.MulY = 190 * 256 / (_DuLieuDoThiNhietDo.maxValue - _DuLieuDoThiNhietDo.minValue);
    setGraphVDCentral(_SPAddressLargeGraph, _DuLieuDoThiNhietDo.VDCentral);
    setGraphMulY(_SPAddressLargeGraph, _DuLieuDoThiNhietDo.MulY);

    _DuLieuDoThiNhietDo.valueStep = (_DuLieuDoThiNhietDo.maxValue - _DuLieuDoThiNhietDo.minValue) / 5;
    setText(_VPAddressGraphYValueText1, String(_DuLieuDoThiNhietDo.minValue / 10, 1));
    for (float i = 1; i < 6; i++)
    {
        setText(_VPAddressGraphYValueText1 + i * 5, String((_DuLieuDoThiNhietDo.minValue + i * _DuLieuDoThiNhietDo.valueStep) / 10, 1));
    }
}

void HMI::HienThiNhietDo(float GiaTri)
{
    setText(_VPAddressTemperatureText, String(GiaTri, 1));
}

void HMI::HienThiNhietDo(String text)
{
    setText(_VPAddressTemperatureText, text);
}

void HMI::HienThiSetpoint(float GiaTri)
{
    setText(_VPAddressSetpointTempText, String(GiaTri, 1));
}

void HMI::HienThiTocDoQuat(int GiaTri)
{
    setText(_VPAddressFanSpeedText, String(GiaTri));
}

// void HMI::HienThiGocFlap(int GiaTri)
// {
//     setText(_VPAddressFlapText, String(GiaTri));
// }

void HMI::HienThiThoiGianChay(int ngay, int gio, int phut, int giay)
{
    char text[15];
    sprintf(text, "%02ud-%02u:%02u:%02u", ngay, gio, phut, giay);
    setText(_VPAddressDelayOffText, text);
}

void HMI::HienThiThoiGianChay(String text)
{
    setText(_VPAddressDelayOffText, text);
}

void HMI::HienThiThoiGianRTC(int ngay, int thang, int nam, int gio, int phut, int giay)
{
    setRTCSOFT(nam, thang, ngay, 1, gio, phut, giay);
}

void HMI::HienThiIconQuat(bool TrangThai)
{
    setVP(_VPAddressIconQuat, TrangThai);
}

void HMI::HienThiIconCua(bool TrangThai)
{
    setVP(_VPAddressIconCua, TrangThai);
}

void HMI::HienThiIconGiaNhiet(bool TrangThai)
{
    setVP(_VPAddressIconGiaNhiet, TrangThai);
}
void HMI::HienThiIconTrangThaiRun(bool TrangThai)
{
    setVP(_VPAddressIconRun, TrangThai);
}
void HMI::HienThiIconOnOffDelayOff(bool TrangThai)
{
    setVP(_VPAddressIconSwitchDelayOff, TrangThai);
}
void HMI::HienThiChuongTrinhDangChay(String text)
{
    setText(_VPAddressProgramNumText, text);
}

void HMI::HienThiSegmentDangChay(int SegmentDangChay, int TongSoSegment)
{
    setText(_VPAddressSegmentNumText, String(SegmentDangChay) + "/" + String(TongSoSegment));
}

void HMI::HienThiSegmentDangChay(String text)
{
    setText(_VPAddressSegmentNumText, text);
}

void HMI::_NutRun_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    // static bool state = 0;
    // if(state == false)
    // {
    //     hmiPtr->_set_event.type = HMI_SET_RUN_ONOFF;
    //     hmiPtr->HienThiIconTrangThaiRun(true);
    //     state = true;
    // } else {
    //     hmiPtr->_set_event.type = HMI_SET_RUN_OFF;
    //     hmiPtr->HienThiIconTrangThaiRun(false);
    //     state = false;
    // }
    hmiPtr->_set_event.type = HMI_SET_RUN_ONOFF;
    hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
}

void HMI::Buzzer(int value)
{
    beepHMI(value);
}

void HMI::HienThiIconSegment(bool TrangThai)
{
    setVP(_VPAddressIconSegment, TrangThai);
}

void HMI::HienThiIconRemoveWater(bool TrangThai)
{
    setVP(_VPAddressIconTickRemoveWater, TrangThai);
}

void HMI::HienThiIconRemoveSample(bool TrangThai)
{
    setVP(_VPAddressIconTickRemoveSample, TrangThai);
}

void HMI::HienThiIconConfirm1(bool TrangThai)
{
    setVP(_VPAddressIconTickConfirm1, TrangThai);
}

void HMI::HienThiIconConfirm2(bool TrangThai)
{
    setVP(_VPAddressIconTickConfirm2, TrangThai);
}

void HMI::HienThiIconUSB(bool TrangThai)
{
    setVP(_VPAddressIconUSB, TrangThai);
}

void HMI::HienThiVongLapChuongTrinhConLai(int GiaTri, int Tong)
{
    setText(_VPAddressCurrentProgramLoopText, String(GiaTri) + "/" + String(Tong));
}

// truc them
void HMI::HienThiCheckAdminPasswordState(String text) {
    setText(_VPAddressTextPasswordCheckState, text);
}

void HMI::HienThiVongLapChuongTrinhConLai(String text)
{
    setText(_VPAddressCurrentProgramLoopText, text);
}

void HMI::HienThiHeSoCalib(float GiaTri)
{
    setText(_VPAddressCalibTempText, String(GiaTri, 1));
}

void HMI::HienThiWarning(String text, uint8_t TrangSauKhiReturn)
{
    _TrangSauKhiNhanReturnTrenWarning = TrangSauKhiReturn;
    setText(_VPAddressWarningText, text);
    setPage(_WarningPage);
}

void HMI::HienThiTenProgramDangEdit(String text)
{
    setText(_VPAddressCurrentProgramNameText, text);
}

void HMI::HienThiThanhLoading(uint8_t PhanTram)
{
    static uint8_t PhanTramHienThiTruocDo = 100;
    if (PhanTramHienThiTruocDo != PhanTram / 2)
    {
        PhanTramHienThiTruocDo = PhanTram / 2;
        setVP(_VPAddressIconLoading, PhanTramHienThiTruocDo);
    }
}

void HMI::HienThiPhanTramThanhLoading(uint8_t PhanTram)
{
    static uint8_t PhanTramHienThiTruocDo = 100;
    if (PhanTramHienThiTruocDo != PhanTram)
    {
        PhanTramHienThiTruocDo = PhanTram;
        setText(_VPAddressTextPhanTramThanhLoading, String(PhanTramHienThiTruocDo) + "%");
    }
}

void HMI::HienThiPhanTramThanhLoading(String text)
{
    setText(_VPAddressTextPhanTramThanhLoading, text);
}

void HMI::HienThiThongTinVersion(String text)
{
    setText(_VPAddressTextVersion, text);
}

void HMI::HienThiThongTinUpdate(String text)
{
    setText(_VPAddressTextCheckForUpdates, text);
    setPage(_CheckForUpdatePage);
}

void HMI::HienThiTrangThaiKetNoiWiFi(String text)
{
    setText(_VPAddressTextWiFiConnectState, text);
}

void HMI::HienThiSSIDWiFi(String text)
{
    setText(_VPAddressTextSSIDWifi, text);
}

void HMI::HienThiPasswordWiFi(String text)
{
    setText(_VPAddressTextPASSWifi, text);
}

void HMI::_NutEnterTrongTrangProgram_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    if (hmiPtr->_TenChuongTrinh != "")
    {
        //     hmiPtr->_set_event.type = HMI_RUN_PROGRAM_MODE;
        //     hmiPtr->_set_event.text = hmiPtr->_TenChuongTrinh;
        //     hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
        hmiPtr->_set_event.pageAfterReturn = _ProgramPage;
        hmiPtr->_ChuoiBanPhimDangNhap = "1";
        hmiPtr->setText(_VPAddressProgramLoopText, "1");
        hmiPtr->setPage(_ProgramLoopPage);
    }
}

void HMI::_NutEnterChayProgramVoiChuKyDuocChon_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    {
        String strLoop = hmiPtr->getText(_VPAddressProgramLoopText, 3);
        if (strLoop == "Inf")
        {
            hmiPtr->_set_event.f_value = 0;
        }
        else
        {
            hmiPtr->_set_event.f_value = strLoop.toInt();
        }
    }
    hmiPtr->_set_event.type = HMI_RUN_PROGRAM_MODE;
    hmiPtr->_set_event.text = hmiPtr->_TenChuongTrinh;
    hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
}

void HMI::_NutInfTrongTrangChonChuKyChayProgram_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->setText(_VPAddressProgramLoopText, "Inf");
}

void HMI::_NutCaiChuKyChayProgram_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = UNDEFINED;
    hmiPtr->_set_event.displayType = HMI_INT;
    hmiPtr->_set_event.maxValue = 255;
    hmiPtr->_set_event.minValue = 1;
    hmiPtr->_set_event.textLen = 3;
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressProgramLoopText;
    hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
    hmiPtr->_set_event.pageAfterReturn = _ProgramLoopPage;
    hmiPtr->_set_event.pageAfterEnter = _ProgramLoopPage;
    hmiPtr->_ChuoiBanPhimDangNhap = "1";
    hmiPtr->setText(_VPAddressKeyboardInputText, "1");
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->setPage(_NumericKeypadPage);
}

void HMI::_NutVaoChucNangTietTrung_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = HMI_RESET_STER;
    hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
    hmiPtr->_DemTrangThaiNext = 0;
    hmiPtr->_GioTietTrung = 2;
    hmiPtr->_PhutTietTrung = 0;
    hmiPtr->_NhietDoTietTrung = 180;
    if (hmiPtr->_PhutTietTrung < 10)
    {
        hmiPtr->setText(_VPAddressTrangCaiThoiGianTietTrung, "0" + String(hmiPtr->_GioTietTrung) + ":" + "0" + String(hmiPtr->_PhutTietTrung));
    }
    else
    {
        hmiPtr->setText(_VPAddressTrangCaiThoiGianTietTrung, "0" + String(hmiPtr->_GioTietTrung) + ":" + String(hmiPtr->_PhutTietTrung));
    }
    hmiPtr->setText(_VPAddressTextNhietDoTietTrung, String(hmiPtr->_NhietDoTietTrung, 0));
    // hmiPtr->setText(_VPAddressTextNhietDoTietTrung, "180");
    hmiPtr->setVP(_VPAddressIconTickRemoveWater, false);
    hmiPtr->setVP(_VPAddressIconTickRemoveSample, false);
    hmiPtr->setVP(_VPAddressIconTickConfirm1, false);
    hmiPtr->setVP(_VPAddressIconTickConfirm2, false);
    hmiPtr->setPage(_SterilizationPage);
}

void HMI::_NutCaiThoiGianTietTrung_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    String strHour;
    String strMinute;
    if (hmiPtr->_GioTietTrung < 10)
    {
        strHour = "0" + String(hmiPtr->_GioTietTrung);
    }
    if (hmiPtr->_PhutTietTrung < 10)
    {
        strMinute = "0" + String(hmiPtr->_PhutTietTrung);
    }
    hmiPtr->setText(_VPAddressHourText, String(hmiPtr->_GioTietTrung));
    hmiPtr->setText(_VPAddressMinuteText, String(hmiPtr->_PhutTietTrung));
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->setPage(_TrangCaiThoiGianTietTrung);
}

void HMI::_NutCaiNhietDoTietTrung_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = UNDEFINED;
    hmiPtr->_set_event.displayType = HMI_FLOAT;
    hmiPtr->_set_event.maxValue = 180;
    hmiPtr->_set_event.minValue = 20;
    hmiPtr->_set_event.pageAfterReturn = hmiPtr->_set_event.pageAfterEnter = _SterilizationPage;
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressTextNhietDoTietTrung;
    hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
    hmiPtr->_set_event.textLen = 5;
    hmiPtr->_ChuoiBanPhimDangNhap = hmiPtr->getText(_VPAddressTextNhietDoTietTrung, 5);
    hmiPtr->setText(_VPAddressKeyboardInputText, hmiPtr->_ChuoiBanPhimDangNhap);
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->setPage(_NumericKeypadPage);
}

void HMI::_NutNextTrongCaiTietTrung_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    if (hmiPtr->_DemTrangThaiNext < 2)
    {
        hmiPtr->setVP(_VPAddressIconTickRemoveWater + hmiPtr->_DemTrangThaiNext, 1);
        hmiPtr->_DemTrangThaiNext++;
    }
    else
    {
        hmiPtr->_ChuoiBanPhimDangNhap = "";
        hmiPtr->_set_event.displayType = HMI_PASSWORD;
        if (hmiPtr->_DemTrangThaiNext == 2)
        {
            hmiPtr->_set_event.textLen = 6;
            hmiPtr->_set_event.type = HMI_SET_ICON1;
            hmiPtr->_set_event.pageAfterEnter = _SterilizationPage;
            hmiPtr->_set_event.pageAfterReturn = _SterilizationPage;
            hmiPtr->setPage(_KeyboardPage);
        }
        else if (hmiPtr->_DemTrangThaiNext == 3)
        {
            hmiPtr->_set_event.textLen = 6;
            hmiPtr->_set_event.type = HMI_SET_ICON2;
            hmiPtr->_set_event.pageAfterEnter = _SterilizationPage;
            hmiPtr->_set_event.pageAfterReturn = _SterilizationPage;
            hmiPtr->setPage(_KeyboardPage);
        }
        hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressKeyboardInputText;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
        hmiPtr->setText(_VPAddressKeyboardInputText, "Password ?");
        hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    }
}

void HMI::_NutEnterCaiTietTrung_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    if (hmiPtr->_DemTrangThaiNext == 4)
    {
        hmiPtr->_set_event.type = HMI_SET_STER_TEMP;
        hmiPtr->_set_event.f_value = hmiPtr->getText(_VPAddressTextNhietDoTietTrung, 5).toFloat();
        hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
        hmiPtr->_set_event.type = HMI_SET_STER_TIME;
        if (hmiPtr->_GioTietTrung >= 2 && hmiPtr->_PhutTietTrung > 0)
        {
            hmiPtr->_GioTietTrung = 2;
            hmiPtr->_PhutTietTrung = 0;
        }
        hmiPtr->_set_event.u32_value = hmiPtr->_GioTietTrung * 3600 + hmiPtr->_PhutTietTrung * 60;
        hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
        hmiPtr->setPage(_HomePage);
    }
}

void HMI::_NutEnterCaiThoiGianTietTrung_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    String strHour, strMinute;
    hmiPtr->_GioTietTrung = hmiPtr->getText(_VPAddressHourText, 2).toInt();
    hmiPtr->_PhutTietTrung = hmiPtr->getText(_VPAddressMinuteText, 2).toInt();
    if (hmiPtr->_GioTietTrung >= 2 && hmiPtr->_PhutTietTrung >= 0)
    {
        hmiPtr->_GioTietTrung = 2;
        hmiPtr->_PhutTietTrung = 0;
    }
    if (hmiPtr->_GioTietTrung < 10)
    {
        strHour = "0" + String(hmiPtr->_GioTietTrung);
    }
    else {
        strHour = String(hmiPtr->_GioTietTrung);
    }
    if (hmiPtr->_PhutTietTrung < 10)
    {
        strMinute = "0" + String(hmiPtr->_PhutTietTrung);
    }
    else {
        strMinute = String(hmiPtr->_PhutTietTrung);
    }

    hmiPtr->setText(_VPAddressTrangCaiThoiGianTietTrung, strHour + ":" + strMinute);
    hmiPtr->setPage(_SterilizationPage);
}

void HMI::_NutTroVeTrongTrangWarning_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->setPage(hmiPtr->_TrangSauKhiNhanReturnTrenWarning);
}

void HMI::_NutXoaDoThi_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->XoaDoThi();
}

void HMI::_NutDataRecord_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->HienThiThanhLoading(0);
    hmiPtr->HienThiPhanTramThanhLoading(0);
    hmiPtr->setPage(_DataRecordPage);
}

void HMI::_NutExportData_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = HMI_EXPORT_DATA;
    hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
    hmiPtr->setPage(_DataRecordPage);
}

void HMI::_NutVaoChucNangUpdate_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = UNDEFINED;
    hmiPtr->_set_event.displayType = HMI_PASSWORD;
    hmiPtr->_set_event.pageAfterEnter = _UpdatePage;
    hmiPtr->_set_event.pageAfterReturn = _SettingsPage;
    hmiPtr->_set_event.textLen = 10;
    hmiPtr->_ChuoiBanPhimDangNhap = "";
    hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
    hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressKeyboardInputText;
    hmiPtr->setText(_VPAddressKeyboardInputText, "Password ?");
    hmiPtr->setText(_VPAddressKeyboardWarningText, "");
    hmiPtr->setPage(_KeyboardPage);
}

void HMI::_CacNutChonPhuongThucUpdate_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    switch (lastBytes)
    {
    case _KeyValueNutChonUpdateBangUSB:
        hmiPtr->_set_event.type = HMI_FIRMWARE_USB;
        hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
        break;
    case _KeyValueNutChonUpdateBangFOTA:
        // hmiPtr->_set_event.type = HMI_FIRMWARE_FOTA;
        if (WiFi.status() == WL_CONNECTED)
        {
            // Gửi yêu cầu Cập nhật firmware OTA
            hmiPtr->_set_event.type = HMI_FIRMWARE_FOTA;
            hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
            break;
        }
        hmiPtr->setPage(_FOTAPage);
        break;
    default:
        break;
    }
}

void HMI::_CacNutTrangFOTA_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    switch (lastBytes)
    {
    case _KeyValueNutSSIDFOTA:
        hmiPtr->_set_event.type = UNDEFINED;
        hmiPtr->_set_event.displayType = HMI_TEXT;
        hmiPtr->_set_event.textLen = 30;
        hmiPtr->_set_event.pageAfterReturn = _FOTAPage;
        hmiPtr->_set_event.pageAfterEnter = _FOTAPage;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
        hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressTextSSIDWifi;
        hmiPtr->_ChuoiBanPhimDangNhap = "";
        hmiPtr->setText(_VPAddressKeyboardInputText, "");
        hmiPtr->setText(_VPAddressKeyboardWarningText, "");
        hmiPtr->setPage(_KeyboardPage);
        break;
    case _KeyValueNutPASSFOTA:
        hmiPtr->_set_event.type = UNDEFINED;
        hmiPtr->_set_event.displayType = HMI_TEXT;
        hmiPtr->_set_event.textLen = 30;
        hmiPtr->_set_event.pageAfterReturn = _FOTAPage;
        hmiPtr->_set_event.pageAfterEnter = _FOTAPage;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
        hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressTextPASSWifi;
        hmiPtr->_ChuoiBanPhimDangNhap = "";
        hmiPtr->setText(_VPAddressKeyboardInputText, "");
        hmiPtr->setText(_VPAddressKeyboardWarningText, "");
        hmiPtr->setPage(_KeyboardPage);
        break;
    case _KeyValueNutUpdateFOTA:
        // Lấy SSID từ HMI
        hmiPtr->_set_event.type = HMI_SET_SSID;
        hmiPtr->_set_event.text = hmiPtr->getText(_VPAddressTextSSIDWifi, 30);
        if (hmiPtr->_set_event.text.compareTo("") == 0)
        {
            break;
        }
        hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);

        // Lấy PASS từ HMI
        hmiPtr->_set_event.type = HMI_SET_PASSWORD;
        hmiPtr->_set_event.text = hmiPtr->getText(_VPAddressTextPASSWifi, 15);
        if (hmiPtr->_set_event.text.compareTo("") == 0)
        {
            break;
        }
        hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);

        // Gửi yêu cầu Cập nhật firmware OTA
        hmiPtr->_set_event.type = HMI_FIRMWARE_FOTA;
        hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);

        break;
    default:
        break;
    }
}

void HMI::_ThanhCuonDoThi_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = HMI_SET_SCROLLCHART;
    hmiPtr->_set_event.f_value = (float)lastBytes;
    hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
}

// truc them
void HMI::_NutVaoChucNangWiFi_(int32_t lastBytes, void* args)
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = UNDEFINED;
    hmiPtr->_set_event.displayType = HMI_TEXT;
    hmiPtr->setPage(_WiFiPage);
}

// truc them
void HMI::_CacNutTrangWiFi_(int32_t lastBytes, void* arg)
{
    HMI* hmiPtr = (HMI*)arg;
    switch (lastBytes)
    {
    case _KeyValueNutWiFiID:
        hmiPtr->_set_event.type = UNDEFINED;
        hmiPtr->_set_event.displayType = HMI_TEXT;
        hmiPtr->_set_event.textLen = 30;
        hmiPtr->_set_event.pageAfterReturn = _WiFiPage;
        hmiPtr->_set_event.pageAfterEnter = _WiFiPage;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
        hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressTextSSIDWifi;
        hmiPtr->_ChuoiBanPhimDangNhap = "";
        hmiPtr->setText(_VPAddressKeyboardInputText, "");
        hmiPtr->setText(_VPAddressKeyboardWarningText, "");
        hmiPtr->setPage(_KeyboardPage);
        break;
        break;
    case _KeyValueNutPassWifi:
        hmiPtr->_set_event.type = UNDEFINED;
        // hmiPtr->_set_event.displayType = HMI_EXTERNAL_PASSWORD;
        hmiPtr->_set_event.displayType = HMI_TEXT;
        hmiPtr->_set_event.textLen = 30;
        hmiPtr->_set_event.pageAfterReturn = _WiFiPage;
        hmiPtr->_set_event.pageAfterEnter = _WiFiPage;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
        hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressTextPASSWifi;
        hmiPtr->_ChuoiBanPhimDangNhap = "";
        hmiPtr->setText(_VPAddressKeyboardInputText, "");
        hmiPtr->setText(_VPAddressKeyboardWarningText, "");
        hmiPtr->setPage(_KeyboardPage);
        break;

        break;
    case _KeyValueNutKetNoiWiFi:
        // Lấy SSID từ HMI
        hmiPtr->_set_event.type = HMI_SET_SSID;
        hmiPtr->_set_event.text = hmiPtr->getText(_VPAddressTextSSIDWifi, 30);
        if (hmiPtr->_set_event.text.compareTo("") == 0)
        {
            break;
        }
        hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);

        // Lấy PASS từ HMI
        hmiPtr->_set_event.type = HMI_SET_PASSWORD;
        hmiPtr->_set_event.text = hmiPtr->getText(_VPAddressTextPASSWifi, 15);
        if (hmiPtr->_set_event.text.compareTo("") == 0)
        {
            break;
        }
        hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);

        // Connect wifi
        hmiPtr->_set_event.type = HMI_CONNECT_OR_DISCONNECT_WIFI;
        hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
        break;
    default:
        break;
    }
}

void HMI::_NutVaoChucNangThayDoiAdminPassword_(int32_t lastBytes, void* args) // truc them
{
    HMI* hmiPtr = (HMI*)args;
    hmiPtr->_set_event.type = UNDEFINED;
    hmiPtr->_set_event.displayType = HMI_TEXT;
    hmiPtr->_set_event.pageAfterEnter = _SettingsPage;
    hmiPtr->_set_event.pageAfterReturn = _SettingsPage;
    hmiPtr->setPage(_AdminPasswordPage);
}
void HMI::_CacNutTrangThayDoiAdminPassword_(int32_t lastBytes, void* args) {
    HMI* hmiPtr = (HMI*)args;
    switch (lastBytes)
    {
    case _KeyValueNutCaiCurrentAdminPassword:
        hmiPtr->_set_event.type = UNDEFINED;
        hmiPtr->_set_event.displayType = HMI_TEXT;
        hmiPtr->_set_event.textLen = 10;
        hmiPtr->_set_event.pageAfterReturn = _AdminPasswordPage;
        hmiPtr->_set_event.pageAfterEnter = _AdminPasswordPage;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
        hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressTextCurrentAdminPassword;
        hmiPtr->_ChuoiBanPhimDangNhap = "";
        hmiPtr->setText(_VPAddressKeyboardInputText, "");
        hmiPtr->setText(_VPAddressKeyboardWarningText, "");
        hmiPtr->setPage(_KeyboardPage);
        break;
        break;
    case _KeyValueNutCaiNewAdminPassword:
        hmiPtr->_set_event.type = UNDEFINED;
        hmiPtr->_set_event.displayType = HMI_TEXT;
        hmiPtr->_set_event.textLen = 10;
        hmiPtr->_set_event.pageAfterReturn = _AdminPasswordPage;
        hmiPtr->_set_event.pageAfterEnter = _AdminPasswordPage;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
        hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressTextNewAdminPassword;
        hmiPtr->_ChuoiBanPhimDangNhap = "";
        hmiPtr->setText(_VPAddressKeyboardInputText, "");
        hmiPtr->setText(_VPAddressKeyboardWarningText, "");
        hmiPtr->setPage(_KeyboardPage);
        break;
    case _KeyValueNutcaiConfirmNewAdminPassword:
        hmiPtr->_set_event.type = UNDEFINED;
        hmiPtr->_set_event.displayType = HMI_TEXT;
        hmiPtr->_set_event.textLen = 10;
        hmiPtr->_set_event.pageAfterReturn = _AdminPasswordPage;
        hmiPtr->_set_event.pageAfterEnter = _AdminPasswordPage;
        hmiPtr->_set_event.VPTextDisplayWhenInput = _VPAddressKeyboardInputText;
        hmiPtr->_set_event.VPTextDisplayAfterEnter = _VPAddressTextConfirmNewAdminPassword;
        hmiPtr->_ChuoiBanPhimDangNhap = "";
        hmiPtr->setText(_VPAddressKeyboardInputText, "");
        hmiPtr->setText(_VPAddressKeyboardWarningText, "");
        hmiPtr->setPage(_KeyboardPage);
        break;

    case _KeyValueNutChangeAdminPassword:
        // Lấy old admin pass từ HMI 
        hmiPtr->HienThiCheckAdminPasswordState("");
        hmiPtr->_set_event.text = hmiPtr->getText(_VPAddressTextCurrentAdminPassword, 10);
        if (hmiPtr->_set_event.text.compareTo("") == 0) {
            hmiPtr->HienThiCheckAdminPasswordState("Missing current password");
            break;
        }
        if (hmiPtr->SoSanhPassWord(hmiPtr->_set_event.text) == 0) {
            hmiPtr->HienThiCheckAdminPasswordState("Incorrect password");
            break;
        }
        else {
            hmiPtr->HienThiCheckAdminPasswordState("Success");
        }
        // Lấy new admin pass từ HMI
        hmiPtr->_set_event.text = hmiPtr->getText(_VPAddressTextNewAdminPassword, 10);
        if (hmiPtr->_set_event.text.compareTo("") == 0) {
            hmiPtr->HienThiCheckAdminPasswordState("Missing new password");
            break;
        }
        hmiPtr->_set_event.text = hmiPtr->getText(_VPAddressTextConfirmNewAdminPassword, 10);
        if (hmiPtr->_set_event.text.compareTo("") == 0) {
            hmiPtr->HienThiCheckAdminPasswordState("Please confirm your new password");
            break;
        }
        if (hmiPtr->getText(_VPAddressTextNewAdminPassword, 10).compareTo(hmiPtr->getText(_VPAddressTextConfirmNewAdminPassword, 10)) != 0) {
            hmiPtr->HienThiCheckAdminPasswordState("Incorrect confirmed password");
            break;
        }
        // Change pass
        hmiPtr->ThayDoiUserAdminPassword(hmiPtr->_set_event.text);
        hmiPtr->_set_event.type = HMI_CHANGE_ADMIN_PASSWORD;
        hmiPtr->_hmiSetDataCallback(hmiPtr->_set_event);
        break;
    default:
        break;

        break;
    }
}
bool HMI::SoSanhPassWord(String EnteredPassword) {
    if (EnteredPassword.compareTo(this->_ChuoiPassword) == 0
        || EnteredPassword.indexOf(MANUFACTURER_PASSWORD) >= 0)
    {
        return 1;
    }
    return 0;
}
void HMI::ThayDoiUserAdminPassword(String EnteredPassword) {
    this->_ChuoiPassword = EnteredPassword;
}