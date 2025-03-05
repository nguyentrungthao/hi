#ifndef _HMI_PARAM_H_
#define _HMI_PARAM_H_

// Danh sach cac trang
#define _WarningPage 61
#define _HomePage 70
#define _SettingsPage 72
#define _ProgramLoopPage 74
#define _SterilizationPage 75
#define _TrangCaiThoiGianTietTrung 76
#define _RTCPage 77
#define _CalibTempPage 78
#define _ProgramPage 79
#define _AlarmPage 80
#define _SegmentAdjPage 81
#define _SetSegmentDelayOffPage 82
#define _SetDelayOffPage 83
#define _SegmentViewPage 84
#define _NumericKeypadPage 85
#define _KeyboardPage 86
#define _DataRecordPage 90
#define _UpdatePage 91
#define _FOTAPage 92
#define _CheckForUpdatePage 93
#define _WiFiPage 94
#define _TempNumericKeypadPage 95
#define _FanNumericKeypadPage 96
#define _CO2NumericKeypadPage 97
#define _AdminPasswordPage     98
#define _InfoPage 100
#define _CalibChoosePage 101
#define _CalibCO2Page 102

/* Sử dụng 0xFFFF để làm mã KeyValue cho việc cấu hình hàm addButtonEvent có nghĩa là với bất kỳ
/* giá trị KeyValue nào thì hàm đăng ký cũng được gọi
*/
#define _AllKeyValue 0xFFFF

// Địa chỉ VP dùng cho các nút nhấn Menu và nút nhấn cài đặt giá trị TRỪ nút Enter
#define _VPAddressCacNutNhan 0x5000

// Keyvalue của từng loại nút nhấn Menu hoặc nút nhấn cài đặt giá trị TRỪ nút Enter
#define _KeyValueSetSetpointTemp 0x01
#define _KeyValueSetFanSpeed 0x02
#define _KeyValueSetDelayOff 0x03
#define _KeyValueZoomGraph 0x04
#define _KeyValueSettings 0x05
#define _KeyValueProgram 0x06
#define _KeyValueAlarm 0x07
#define _KeyValueSterilization 0x08
#define _KeyValueDataRecord 0x09
#define _KeyValueCalib 0x0A
#define _KeyValueSetTimeRTC 0x0B
#define _KeyValueInformation 0x0C
//* key trong trang _CalibTempPage
#define _KeyValueEditCalibTemp 0x0D
#define _KeyValueEnterCalibTemp 0x0E

// #define _KeyValueSetFlap 0x0F
// #define _KeyValueEnterFlap 0x10
#define _KeyValueEnterSetRTC 0x11
#define _KeyValueEnterSetAlarm 0x12
#define _KeyValueSetAlarmBelow 0x13
#define _KeyValueSetAlarmAbove 0x14

#define _KeyValueRunButton 0x15
#define _KeyValueResetCalibTemp 0x16

#define _KeyValueExport 0x17

#define _KeyValueUpdate 0x18

#define _KeyValueWifi 0x19 // truc them
#define _KeyValueAdminPassword 0x1A
//---
#define _KeyValueSetSetpointCO2 0x1B
#define _KeyValueSwapGraph 0x1C
#define _KeyValueSetAlarmUnderCO2 0x1F
#define _KeyValueSetAlarmOverCO2 0x20
#define _KeyValueEditCalibCO2 0x21
#define _KeyValueResetCalibCO2 0x22
#define _KeyValueChooseCalibCO2 0x1D
#define _KeyValueChooseCalibTemp 0x1E
#define _KeyValueEnterCalibCO2 0x23



#pragma region VP các nút  Segment

#define _VPAddressSegmentSelectButton 0x5001
#define _VPAddressSegmentSetpointButton 0x5002
#define _VPAddressSegmentDelayOffButton 0x5003
#define _VPAddressSegmentFanSpeedButton 0x5004
// #define _VPAddressSegmentFlapButton 0x5005
#define _VPAddressSegmentCO2Button 0x5005
#define _VPAddressSegmentTempMinButton 0x5006
#define _VPAddressSegmentTempMaxButton 0x5007
#define _VPAddressSegmentCO2MinButton 0x501E
#define _VPAddressSegmentCO2MaxButton 0x501F


#define _VPAddressSegmentFunctionButton 0x5009
#define _KeyValueSegmentAddButton 1
#define _KeyValueSegmentSubButton 2
#define _KeyValueSegmentUpButton 3
#define _KeyValueSegmentDownButton 4
#define _KeyValueSegmentEditProgramNameButton 5
#define _KeyValueSegmentEnterButton 6

#define _VPAddressSetTimeButton 0x500C
#define _KeyValueSetDay 1
#define _KeyValueSetMonth 2
#define _KeyValueSetYear 3
#define _KeyValueSetHour 4
#define _KeyValueSetMinute 5
#define _KeyValueSetSecond 6
#define _KeyValueEnterSegmentDelayOff 7
#define _KeyValueEnterSetRTC 8
#define _KeyValueEnterSetDelayOff 9
#define _KeyValueEnterCaiThoiGianTietTrung 10

#pragma region VP cac nut trong Program

#define _VPAddressSelectProgram 0x500D
#define _KeyValueProgram1 1
#define _KeyValueProgram2 2
#define _KeyValueProgram3 3
#define _KeyValueProgram4 4
#define _KeyValueProgram5 5

#define _VPAddressProgramFunction 0x500E
#define _KeyValueProgramUp 1
#define _KeyValueProgramDown 2
#define _KeyValueProgramAdd 3
#define _KeyValueProgramDelete 4
#define _KeyValueProgramEdit 5
#define _KeyValueProgramView 6
#define _KeyValueProgramQuick 7

#define _VPAddressSwitchDelayOff 0x500F
#define _VPAddressEnterRunProgram 0x5010
#define _VPAddressEnterProgramLoop 0x5011
#define _VPAddressInfButton 0x5012
#define _VPAddressSetProgramLoop 0x5013

#pragma region VP cac nut trang Tiệt trùng

#define _VPAddressCaiThoiGianTietTrung 0x5014
#define _VPAddressCaiNhietDoTietTrung 0x5015
#define _VPAddressNextTrongTrangTietTrung 0x5016
#define _VPAddressEnterCaiTietTrung 0x5017

#pragma region VP cac nut trang Warning
#define _VPAddressWarningReturn 0x5018

#pragma region VP cac nut reset Graph
#define _VPAddressResetGraph 0x5019

#pragma region VP cac nut chuc nang Update
#define _VPAddressCacNutChonPhuongThucUpdate 0x501A
#define _KeyValueNutChonUpdateBangUSB 1
#define _KeyValueNutChonUpdateBangFOTA 2

#define _VPAddressCacNutTrangFOTA 0x501B
#define _KeyValueNutSSIDFOTA 1
#define _KeyValueNutPASSFOTA 2
#define _KeyValueNutUpdateFOTA 3

// truc them
#pragma region VP cac nut trang WiFi
#define _VPAddressCacNutTrangWiFi 0x501C
#define _KeyValueNutWiFiID 1
#define _KeyValueNutPassWifi 2
#define _KeyValueNutKetNoiWiFi 3

// Truc them
#pragma region VP cac nut trang Cai mau khau admin
#define _VPAddressCacNutTrangAdminPassword 0x501D
#define _KeyValueNutCaiCurrentAdminPassword 1
#define _KeyValueNutCaiNewAdminPassword 2
#define _KeyValueNutcaiConfirmNewAdminPassword 3
#define _KeyValueNutChangeAdminPassword 4

#pragma region NumericKeypad
#define _VPAddressNumericKeypad 0x5300

#pragma region ICON VP Address

#define _VPAddressIconRun 0x6000
#define _VPAddressIconQuat 0x6001
#define _VPAddressIconCua 0x6002
#define _VPAddressIconGiaNhiet 0x6003
#define _VPAddressIconSwitchDelayOff 0x6004
#define _VPAddressIconSegment 0x6005
#define _VPAddressIconTickRemoveWater 0x6006
#define _VPAddressIconTickRemoveSample 0x6007
#define _VPAddressIconTickConfirm1 0x6008
#define _VPAddressIconTickConfirm2 0x6009
#define _VPAddressIconLoading 0x600A
#define _VPAddressIonConnect 0x600B
#define _VPAddressIconWiFi 0x600C
#define _VPAddressIconLowercaseKeyboard 0x600D
#define _VPAddressIconUSB 0x600E

#pragma endregion

#pragma region Địa chỉ TEXT

#define _VPAddressSetpointTempText 0x8000    // Length text 5
#define _VPAddressFanSpeedText 0x8005    // Length text 5
#define _VPAddressDelayOffText 0x800A    // Length text 15
#define _VPAddressTemperatureText 0x8019 // Length text 5

#define _VPAddressDayText 0x8020    // length text 2
#define _VPAddressMonthText 0x8022  // length text 2
#define _VPAddressYearText 0x8024   // length text 4
#define _VPAddressHourText 0x8028   // length text 2
#define _VPAddressMinuteText 0x802A // length text 2

#define _VPAddressStdTempText 0x8100
// #define _VPAddressFlapText 0x8105
#define _VPAddressAlarmBelowTempText 0x810A
#define _VPAddressAlarmAboveTempText 0x810F
#define _VPAddressProgramNumText 0x8120
#define _VPAddressSegmentNumText 0x8130
#define _VPAddressCalibTempText 0x8140

#define _VPAddressProgramNameText1 0x8200
#define _VPAddressProgramNameText2 _VPAddressProgramNameText1 + 20 // 8214
#define _VPAddressProgramNameText3 _VPAddressProgramNameText2 + 20 // 8228
#define _VPAddressProgramNameText4 _VPAddressProgramNameText3 + 20 // 823C
#define _VPAddressProgramNameText5 _VPAddressProgramNameText4 + 20 // 8250

#define _VPAddressGraphYValueText1 0x8300
#define _VPAddressGraphYValueText2 0x8305
#define _VPAddressGraphYValueText3 0x830A
#define _VPAddressGraphYValueText4 0x830F
#define _VPAddressGraphYValueText5 0x8314
#define _VPAddressGraphYValueText6 0x8319

#define _VPAddressGraphXValueText1 0x8320
#define _VPAddressGraphXValueText2 0x832A
#define _VPAddressGraphXValueText3 0x8334
#define _VPAddressGraphXValueText4 0x833E
#define _VPAddressGraphXValueText5 0x8348
#define _VPAddressGraphXValueText6 0x8352
#define _VPAddressGraphXValueText7 0x835C
#define _VPAddressGraphXValueText8 0x8366
#define _VPAddressGraphXValueText9 0x8370

#define _VPAddressSegmentIconTick1 0x8990
#define _VPAddressSegmentIconTick2 0x8991
#define _VPAddressSegmentIconTick3 0x8992
#define _VPAddressSegmentIconTick4 0x8993
#define _VPAddressSegmentIconTick5 0x8994

#define _VPAddressNumSegmentText1 0x9000 // Có 5 ô, mỗi ô tối đa 2 ký tự
#define _VPAddressNumSegmentText2 _VPAddressNumSegmentText1 + 2
#define _VPAddressNumSegmentText3 _VPAddressNumSegmentText2 + 2
#define _VPAddressNumSegmentText4 _VPAddressNumSegmentText3 + 2
#define _VPAddressNumSegmentText5 _VPAddressNumSegmentText4 + 2

#define _VPAddressSegmentSetpointText1 0x900A // Có 5 ô, mỗi ô tối đa 5 ký tự
#define _VPAddressSegmentSetpointText2 _VPAddressSegmentSetpointText1 + 5
#define _VPAddressSegmentSetpointText3 _VPAddressSegmentSetpointText2 + 5
#define _VPAddressSegmentSetpointText4 _VPAddressSegmentSetpointText3 + 5
#define _VPAddressSegmentSetpointText5 _VPAddressSegmentSetpointText4 + 5

#define _VPAddressSegmentDelayOffText1 0x9023 // Có 5 ô, mỗi ô tối đa 20 ký tự
#define _VPAddressSegmentDelayOffText2 _VPAddressSegmentDelayOffText1 + 20
#define _VPAddressSegmentDelayOffText3 _VPAddressSegmentDelayOffText2 + 20
#define _VPAddressSegmentDelayOffText4 _VPAddressSegmentDelayOffText3 + 20
#define _VPAddressSegmentDelayOffText5 _VPAddressSegmentDelayOffText4 + 20

#define _VPAddressSegmentFanSpeedText1 0x9087 // Có 5 ô, mỗi ô tối đa 5 ký tự
#define _VPAddressSegmentFanSpeedText2 _VPAddressSegmentFanSpeedText1 + 5
#define _VPAddressSegmentFanSpeedText3 _VPAddressSegmentFanSpeedText2 + 5
#define _VPAddressSegmentFanSpeedText4 _VPAddressSegmentFanSpeedText3 + 5
#define _VPAddressSegmentFanSpeedText5 _VPAddressSegmentFanSpeedText4 + 5

// #define _VPAddressSegmentAirFlapText1 0x90A0
// #define _VPAddressSegmentAirFlapText2 _VPAddressSegmentAirFlapText1 + 5
// #define _VPAddressSegmentAirFlapText3 _VPAddressSegmentAirFlapText2 + 5
// #define _VPAddressSegmentAirFlapText4 _VPAddressSegmentAirFlapText3 + 5
// #define _VPAddressSegmentAirFlapText5 _VPAddressSegmentAirFlapText4 + 5

#define _VPAddressSegmentTempMinText1 0x90B9 // Có 5 ô, mỗi ô tối đa 5 ký tự
#define _VPAddressSegmentTempMinText2 _VPAddressSegmentTempMinText1 + 5
#define _VPAddressSegmentTempMinText3 _VPAddressSegmentTempMinText2 + 5
#define _VPAddressSegmentTempMinText4 _VPAddressSegmentTempMinText3 + 5
#define _VPAddressSegmentTempMinText5 _VPAddressSegmentTempMinText4 + 5

#define _VPAddressSegmentTempMaxText1 0x90D2 // Có 5 ô, mỗi ô tối đa 5 ký tự
#define _VPAddressSegmentTempMaxText2 _VPAddressSegmentTempMaxText1 + 5
#define _VPAddressSegmentTempMaxText3 _VPAddressSegmentTempMaxText2 + 5
#define _VPAddressSegmentTempMaxText4 _VPAddressSegmentTempMaxText3 + 5
#define _VPAddressSegmentTempMaxText5 _VPAddressSegmentTempMaxText4 + 5

#define _VPAddressNumProgramText1 0x90F7
#define _VPAddressNumProgramText2 _VPAddressNumProgramText1 + 2
#define _VPAddressNumProgramText3 _VPAddressNumProgramText2 + 2
#define _VPAddressNumProgramText4 _VPAddressNumProgramText3 + 2
#define _VPAddressNumProgramText5 _VPAddressNumProgramText4 + 2

#define _VPAddressTotalNumOfSegmentsText1 0x9101
#define _VPAddressTotalNumOfSegmentsText2 _VPAddressTotalNumOfSegmentsText1 + 2
#define _VPAddressTotalNumOfSegmentsText3 _VPAddressTotalNumOfSegmentsText2 + 2
#define _VPAddressTotalNumOfSegmentsText4 _VPAddressTotalNumOfSegmentsText3 + 2
#define _VPAddressTotalNumOfSegmentsText5 _VPAddressTotalNumOfSegmentsText4 + 2

#define _VPAddressKeyboardWarningText 0xA010
#define _VPAddressWarningText 0xA020            // 50 ky tu
#define _VPAddressCurrentProgramNameText 0xA052 // 20 ky tu
#define _VPAddressProgramLoopText 0xA072        // 3 ký tự
#define _VPAddressCurrentProgramLoopText 0xA075 // 10 ký tự

#define _VPAddressTrangCaiThoiGianTietTrung 0xA07F // 15 ký tự
#define _VPAddressTextNhietDoTietTrung 0xA094      // 5 ký tự

#define _VPAddressTextPhanTramThanhLoading 0xA09A // 5 ky tu

#define _VPAddressTextCheckForUpdates 0xA09F  // 30 ký tự
#define _VPAddressTextSSIDWifi 0xA0BD         // 30
#define _VPAddressTextPASSWifi 0xA0DB         // 15
#define _VPAddressTextWiFiConnectState 0xA0EA // 15 truc them

#define _VPAddressKeyboardInputText 0xA100 // 30 ky tu

#define _VPAddressTextCurrentAdminPassword 0xA11E    // 10 ky tu truc them
#define _VPAddressTextNewAdminPassword 0xA128        // 10 ky tu
#define _VPAddressTextConfirmNewAdminPassword 0xA132 // 10 ky tu
#define _VPAddressTextPasswordCheckState 0xA13C      // 40 ky tu truc them

#define _VPAddressTextVersion 0xAFF0 // 15 ky tu
//---
#define _VPAddressCO2Text 0xAFFF // 5 ky tu
#define _VPAddressSetpointCO2Text 0xB005 // 5 ký tự 
#define _VPAddressAlarmBelowCO2Text 0xB00A // 5 ký tự 
#define _VPAddressAlarmAboveCO2Text 0xB00F // 5 ký tự 

#define _VPAddressSegmentCO2Text1 0xB014
#define _VPAddressSegmentCO2Text2 (_VPAddressSegmentCO2Text1 + 5)
#define _VPAddressSegmentCO2Text3 (_VPAddressSegmentCO2Text2 + 5)
#define _VPAddressSegmentCO2Text4 (_VPAddressSegmentCO2Text3 + 5)
#define _VPAddressSegmentCO2Text5 (_VPAddressSegmentCO2Text4 + 5)

#define _VPAddressSegmentCO2MinText1 (_VPAddressSegmentCO2Text5 + 5)
#define _VPAddressSegmentCO2MinText2 (_VPAddressSegmentCO2MinText1 + 5)
#define _VPAddressSegmentCO2MinText3 (_VPAddressSegmentCO2MinText2 + 5)
#define _VPAddressSegmentCO2MinText4 (_VPAddressSegmentCO2MinText3 + 5)
#define _VPAddressSegmentCO2MinText5 (_VPAddressSegmentCO2MinText4 + 5)

#define _VPAddressSegmentCO2MaxText1 (_VPAddressSegmentCO2MinText5 + 5) // Có 5 ô, mỗi ô tối đa 5 ký tự
#define _VPAddressSegmentCO2MaxText2 (_VPAddressSegmentCO2MaxText1 + 5)
#define _VPAddressSegmentCO2MaxText3 (_VPAddressSegmentCO2MaxText2 + 5)
#define _VPAddressSegmentCO2MaxText4 (_VPAddressSegmentCO2MaxText3 + 5)
#define _VPAddressSegmentCO2MaxText5 (_VPAddressSegmentCO2MaxText4 + 5)

#define _VPAddressCalibCO2Text 0xB05F // 5 ký tự
#define _VPAddressStdCO2Text 0xB064 // 5 ký tự

#pragma region SP Address

#define _SPAddressProgramNameText1 0xB000
#define _SPAddressProgramNameText2 0xB010
#define _SPAddressProgramNameText3 0xB020
#define _SPAddressProgramNameText4 0xB030
#define _SPAddressProgramNameText5 0xB040

#define _SPAddressSmallGraph1 0xB100
#define _SPAddressLargeGraph 0xB110
#define _SPAddressSmallGraph2 0xB120
#define _SPAddressSmallGraphCO2 0xB130

#pragma region Mã màu

#define _RedColor 0xF800
#define _BlueColor 0x0014

#endif