#ifndef _PID_h
#define _PID_h

#include "Config.h"

typedef struct PIDCalcu_t {
  float PTerm;
  float ITerm;
  float DTerm;
  float feedBackWindup;
  float Output;
};
typedef struct PIDParam_t {
  float Kp;
  float Ki;
  float Kd;
  float Kw;
  float WindupMax;
  float WindupMin;
  float OutMax;
  float OutMin;
};

class PID {
private:
  PIDParam_t xPIDParam;

  PIDCalcu_t xPIDCalcu;
  
  float LastError = 0;
  float Sample_time = 1000.0;
  float DTermFiltered = 0;
  float alpha = 0.7f; // 1/(1+tD), tD hằng số thời gian lọc
public:
  PID();
  PID(float, float, float, float);
  void setPIDparamters(float, float, float);
  void setWindup(float, float, float);
  void setOutput(float, float);
  void setSampleTime(float);  //ms

  PIDParam_t xGetParam() const;
  PIDCalcu_t xGetCalcu() const;
  void vSetParam(PIDParam_t xPIDParam);

  void vResetPID();

  float getPIDcompute(float);
  float getWindupMax();
  float getWindupMin();
  float getSampleTime();
  float getKp();
  float getKi();
  float getKd();
  float getOutputMin();
  float getOutputMax();
};





//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#ifdef CodePIDCu

#include "Arduino.h"

// Các thông số PID mặc định
//-----------------------------------------
#define _GioiHanOUTPUT 4095
#define _Kp 0.00005
#define _Ki 0.00001
#define _Kd 0
//-----------------------------------------

class PID {
private:
  float Kp, Ki, Kd;
  float KhauP, KhauI, KhauD;
  float prevError;
  //uint32_t prevTime;
  //uint32_t currentTime;
  uint32_t GioiHanKhauI = 4095;
  uint16_t GioiHanOUTPUT;

public:
  void KhoiTaoPID(float Kp, float Ki, float Kd, uint16_t GioiHanOUTPUT);
  float TinhToanPID(float TocDoThuc, float TocDoMongMuon, uint32_t sampleTime);
  void CapNhapPID(float Kp, float Ki, float Kd);
  void ResetKhauI(void);
};
#endif


#endif
