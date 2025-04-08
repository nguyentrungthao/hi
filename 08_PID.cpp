#include "08_PID.h"
#include "Arduino.h"

// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------Hàm khởi tạo đối tượng PID--------//
PID::PID()
{
    this->Kp = 1;
    this->Ki = 0;
    this->Kd = 0;
    this->Sample_time = 100;
    this->WindupMax = 0;
    this->WindupMin = 0;
    this->OutMax = 0;
    this->OutMin = 0;
    this->ITerm = 0;
}
// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//

// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------Hàm khoi tạo đối tương PID có truyền thông số---------//
PID::PID(float Kp, float Ki = 0, float Kd = 0, float Sample_time = 1000)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->Sample_time = Sample_time;
    this->WindupMax = 0;
    this->WindupMin = 0;
    this->OutMax = 0;
    this->OutMin = 0;
    this->ITerm = 0;
    this->PTerm = 0;
    this->DTerm = 0;
}
// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//

// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------Hàm thay đổi hệ số PID------------//
void PID::setPIDparamters(float Kp, float Ki = 0, float Kd = 0)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
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

    // PTERM
    this->PTerm = this->Kp * Error;

    // ITERM
    float deltaITerm = (this->Ki * Error + this->feedBackWindup * Kw) * (this->Sample_time / 1000.0f);
    if (!std::isnan(deltaITerm) && !std::isinf(deltaITerm) && fabs(Error) < 2)
    {
        this->ITerm += deltaITerm;
    }
    if (this->ITerm > this->WindupMax)
    {
        this->ITerm = this->WindupMax;
    }
    else if (this->ITerm < this->WindupMin)
    {
        this->ITerm = this->WindupMin;
    }

    // DTERM with filtering
    if (this->Sample_time > 0)
    {
        float derivative = this->Kd * (Error - this->LastError) / (this->Sample_time / 1000.0f);
        if (std::isnan(derivative) || std::isinf(derivative))
        {
            derivative = 0;
        }
        if (std::isnan(this->DTermFiltered) || std::isinf(this->DTermFiltered))
        {
            this->DTermFiltered = 0;
        }
        this->DTerm = this->DTermFiltered * (1.0f - this->alpha) + derivative * this->alpha;
        this->DTermFiltered = this->DTerm;

        // Giới hạn DTerm
        const float DTermMin = -100.0f;
        const float DTermMax = 10.0f;
        if (this->DTerm < DTermMin)
        {
            this->DTerm = DTermMin;
        }
        else if (this->DTerm > DTermMax)
        {
            this->DTerm = DTermMax;
        }
    }
    else
    {
        this->DTerm = 0;
    }

    this->LastError = Error;

    // OUTPUT
    this->Output = this->PTerm + this->ITerm;

    if (this->Output > this->OutMax)
    {
        this->feedBackWindup = this->OutMax - this->Output;
        this->Output = this->OutMax;
    }
    else if (this->Output < this->OutMin)
    {
        this->feedBackWindup = this->OutMin - this->Output;
        this->Output = this->OutMin;
    }
    else
    {
        this->feedBackWindup = 0;
    }

    this->Output += this->DTerm;

    // Clamp output lần cuối
    if (this->Output > this->OutMax)
    {
        this->Output = this->OutMax;
    }
    else if (this->Output < this->OutMin)
    {
        this->Output = this->OutMin;
    }

    return this->Output;
}

// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//

// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------Hàm set bão hòa khâu I------------//
void PID::setWindup(float MinValue, float MaxValue, float Kw)
{
    this->WindupMax = MaxValue;
    this->WindupMin = MinValue;
    this->Kw = Kw;
}
// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//

// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------Hàm set bão hòa OUTPUT------------//
void PID::setOutput(float MinValue, float MaxValue)
{
    this->OutMax = MaxValue;
    this->OutMin = MinValue;
}
// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//

// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//
//----------------Hàm set thời gian lấy mẫu---------//
void PID::setSampleTime(float value)
{
    this->Sample_time = value;
}
// MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM//

float PID::getWindupMax()
{
    return this->WindupMax;
}
float PID::getWindupMin()
{
    return this->WindupMin;
}
float PID::getSampleTime()
{
    return this->Sample_time;
}
float PID::getKp()
{
    return this->Kp;
}
float PID::getKi()
{
    return this->Ki;
}
float PID::getKd()
{
    return this->Kd;
}
float PID::getOutputMin()
{
    return this->OutMin;
}
float PID::getOutputMax()
{
    return this->OutMax;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#ifdef CodePIDCu

void PID::KhoiTaoPID(float Kp, float Ki, float Kd, uint16_t GioiHanOUTPUT)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
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

    this->KhauI += this->Ki * error * sampleTime;
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
    float output = this->KhauP + this->KhauI + this->Kd * this->KhauD;
    // Serial.print("output: ");Serial.println(output);
    if (output > this->GioiHanOUTPUT)
        output = this->GioiHanOUTPUT;
    else if (output < 0.0)
        output = 0.0;

    return output;
}

void PID::CapNhapPID(float Kp, float Ki, float Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}
void PID::ResetKhauI(void)
{
    this->KhauI = 0;
}

#endif