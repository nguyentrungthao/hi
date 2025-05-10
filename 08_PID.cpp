#include "08_PID.h"
#include "Arduino.h"

// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------Hàm khởi tạo đối tượng PID--------//
PID::PID()
{
    this->xPIDParam.Kp = 1;
    this->xPIDParam.Ki = 0;
    this->xPIDParam.Kd = 0;
    this->Sample_time = 100;
    this->xPIDParam.WindupMax = 0;
    this->xPIDParam.WindupMin = 0;
    this->xPIDParam.OutMax = 0;
    this->xPIDParam.OutMin = 0;
    this->xPIDCalcu.ITerm = 0;
}
// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//

// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------Hàm khoi tạo đối tương PID có truyền thông số---------//
PID::PID(float Kp, float Ki = 0, float Kd = 0, float Sample_time = 1000)
{
    this->xPIDParam.Kp = Kp;
    this->xPIDParam.Ki = Ki;
    this->xPIDParam.Kd = Kd;
    this->Sample_time = Sample_time;
    this->xPIDParam.WindupMax = 0;
    this->xPIDParam.WindupMin = 0;
    this->xPIDParam.OutMax = 0;
    this->xPIDParam.OutMin = 0;
    this->xPIDCalcu.ITerm = 0;
    this->xPIDCalcu.PTerm = 0;
    this->xPIDCalcu.DTerm = 0;
}
// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//

// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------Hàm thay đổi hệ số PID------------//
void PID::setPIDparamters(float Kp, float Ki = 0, float Kd = 0)
{
    this->xPIDParam.Kp = Kp;
    this->xPIDParam.Ki = Ki;
    this->xPIDParam.Kd = Kd;
}
// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//

// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------Hàm tính toán PID-----------------//
float PID::getPIDcompute(float Error)
{
    if (std::isnan(Error) || std::isinf(Error))
    {
        Error = 0;
    }

    // xPIDCalcu.PTerm
    this->xPIDCalcu.PTerm = this->xPIDParam.Kp * Error;

    // xPIDCalcu.ITerm
    float deltaITerm = (this->xPIDParam.Ki * Error + this->xPIDCalcu.feedBackWindup * this->xPIDParam.Kw) * (this->Sample_time / 1000.0f);
    if (!std::isnan(deltaITerm) && !std::isinf(deltaITerm))
    {
        this->xPIDCalcu.ITerm += deltaITerm;
    }
    if (this->xPIDCalcu.ITerm > this->xPIDParam.WindupMax)
    {
        this->xPIDCalcu.ITerm = this->xPIDParam.WindupMax;
    }
    else if (this->xPIDCalcu.ITerm < this->xPIDParam.WindupMin)
    {
        this->xPIDCalcu.ITerm = this->xPIDParam.WindupMin;
    }

    // xPIDCalcu.DTerm with filtering
    if (this->Sample_time > 0)
    {
        float derivative = this->xPIDParam.Kd * (Error - this->LastError) / (this->Sample_time / 1000.0f);
        if (std::isnan(derivative) || std::isinf(derivative))
        {
            derivative = 0;
        }
        if (std::isnan(this->DTermFiltered) || std::isinf(this->DTermFiltered))
        {
            this->DTermFiltered = 0;
        }
        this->xPIDCalcu.DTerm = this->DTermFiltered * (1.0f - this->alpha) + derivative * this->alpha;
        this->DTermFiltered = this->xPIDCalcu.DTerm;

        // // Giới hạn xPIDCalcu.DTerm
        // const float DTermMin = -1000.0f;
        // const float DTermMax = 1000.0f;
        // if (this->xPIDCalcu.DTerm < DTermMin)
        // {
        //     this->xPIDCalcu.DTerm = DTermMin;
        // }
        // else if (this->xPIDCalcu.DTerm > DTermMax)
        // {
        //     this->xPIDCalcu.DTerm = DTermMax;
        // }
    }
    else
    {
        this->xPIDCalcu.DTerm = 0;
    }

    this->LastError = Error;


    // OUTPUT
    float unSaturationOutput = this->xPIDCalcu.PTerm + this->xPIDCalcu.ITerm + this->xPIDCalcu.DTerm;

    this->xPIDCalcu.Output = unSaturationOutput;
    if (this->xPIDCalcu.Output > this->xPIDParam.OutMax)
    {
        this->xPIDCalcu.Output = this->xPIDParam.OutMax;
    }
    else if (this->xPIDCalcu.Output < this->xPIDParam.OutMin)
    {
        this->xPIDCalcu.Output = this->xPIDParam.OutMin;
    }
    this->xPIDCalcu.feedBackWindup = this->xPIDCalcu.Output - unSaturationOutput;

    return this->xPIDCalcu.Output;
}

// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//

// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------Hàm set bão hòa khâu I------------//
void PID::setWindup(float MinValue, float MaxValue, float Kw)
{
    this->xPIDParam.WindupMax = MaxValue;
    this->xPIDParam.WindupMin = MinValue;
    this->xPIDParam.Kw = Kw;
}
// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//

// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------Hàm set bão hòa OUTPUT------------//
void PID::setOutput(float MinValue, float MaxValue)
{
    this->xPIDParam.OutMax = MaxValue;
    this->xPIDParam.OutMin = MinValue;
}
// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//

// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------Hàm set thời gian lấy mẫu---------//
void PID::setSampleTime(float value)
{
    this->Sample_time = value;
}
// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//

PIDParam_t PID::xGetParam() const {
    return xPIDParam;
}
PIDCalcu_t PID::xGetCalcu() const {
    return xPIDCalcu;
}
void PID::vSetParam(PIDParam_t xPIDParam) {
    this->xPIDParam = xPIDParam;
}

void PID::vResetPID() {
    xPIDCalcu.PTerm = 0;
    xPIDCalcu.ITerm = 0;
    xPIDCalcu.DTerm = 0;
    xPIDCalcu.feedBackWindup = 0;
    xPIDCalcu.Output = 0;
}

float PID::getWindupMax()
{
    return this->xPIDParam.WindupMax;
}
float PID::getWindupMin()
{
    return this->xPIDParam.WindupMin;
}
float PID::getSampleTime()
{
    return this->Sample_time;
}
float PID::getKp()
{
    return this->xPIDParam.Kp;
}
float PID::getKi()
{
    return this->xPIDParam.Ki;
}
float PID::getKd()
{
    return this->xPIDParam.Kd;
}
float PID::getOutputMin()
{
    return this->xPIDParam.OutMin;
}
float PID::getOutputMax()
{
    return this->xPIDParam.OutMax;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#ifdef CodePIDCu

void PID::KhoiTaoPID(float Kp, float Ki, float Kd, uint16_t GioiHanOUTPUT)
{
    this->xPIDParam.Kp = Kp;
    this->xPIDParam.Ki = Ki;
    this->xPIDParam.Kd = Kd;
    // this->currentTime = currentTime;
    // this->prevTime = currentTime;
    this->GioiHanOUTPUT = GioiHanOUTPUT;

    this->KhauP = 0.0;
    this->KhauI = 0.0;
    this->KhauD = 0.0;
    this->prevError = 0;
}

float PID::TinhToanPID(float TocDoThuc, float TocDoMongMuon, uint32_t sampleTime)
{
    // Serial.print("Current time: ");Serial.println(currentTime);

    float error = TocDoMongMuon - TocDoThuc;
    // Serial.print("error: ");Serial.println(error);

    // Khâu tỉ lệ (P)
    this->KhauP = Kp * error;
    // Serial.print("Khâu P: ");Serial.println(KhauP);

    // Tính khâu I và D.
    //---------------------------------------------
    // this->deltaT = deltaT;
    // float deltaTime = this->currentTime - this->prevTime;
    // Serial.print("deltaTime: ");Serial.println(deltaTime);
    float deltaError = error - this->prevError;
    // Serial.print("deltaError: ");Serial.println(deltaError);

    this->KhauI += this->xPIDParam.Ki * error * sampleTime;
    // Serial.print("Khâu I: ");Serial.println(KhauI);

    if (KhauI < 0)
        KhauI = 0;
    else if (KhauI > GioiHanKhauI)
        KhauI = GioiHanKhauI;

    // this->KhauD = 0.0;
    // if (deltaTime > 0)
    this->KhauD = deltaError / sampleTime;
    // Serial.print("Khâu D: ");Serial.println(KhauD);
    //---------------------------------------------

    // Lưu thời gian và sai số cho lần gọi PID tiếp theo
    // this->prevTime = sampleTime;
    this->prevError = error;

    // Tính giá trị output của PID.
    float output = this->KhauP + this->KhauI + this->xPIDParam.Kd * this->KhauD;
    // Serial.print("output: ");Serial.println(output);
    if (output > this->GioiHanOUTPUT)
        output = this->GioiHanOUTPUT;
    else if (output < 0.0)
        output = 0.0;

    return output;
}

void PID::CapNhapPID(float Kp, float Ki, float Kd)
{
    this->xPIDParam.Kp = Kp;
    this->xPIDParam.Ki = Ki;
    this->xPIDParam.Kd = Kd;
}
void PID::ResetKhauI(void)
{
    this->KhauI = 0;
}

#endif