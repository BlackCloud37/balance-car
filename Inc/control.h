#ifndef __CONTROL_H
#define __CONTROL_H

#include "filter.h"

extern unsigned int g_nMainEventCount; //���¼������������ж���
extern unsigned int g_nGetPulseCount;  //������������������ж���
extern unsigned int g_nSpeedControlCount;
extern float g_fCarAngle;
extern unsigned int g_nLeftMotorPulse;
extern int g_nSpeedTarget;
extern int g_nLeftMotorOutput;
extern int g_nSpeedControlPeriod;//�ٶȻ��������ڼ�����

void SpeedControlOutput(void);
void GetMpuData(void);
void AngleCalculate(void);
void GetMotorPulse(void);
int SpeedInnerControl(int nPulse, int nTarget, int nPwm, int nErrorPrev);
void SetMotorVoltageAndDirection(int nLeftMotorPwm,int nRightMotorPwm);
void MotorOutput(void);
void AngleControl(void);
void SpeedControl(void);

#endif
