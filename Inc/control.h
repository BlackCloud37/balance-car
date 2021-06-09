#ifndef __CONTROL_H
#define __CONTROL_H

#include "filter.h"

extern unsigned int g_nMainEventCount; //主事件计数，用在中断中
extern unsigned int g_nGetPulseCount;  //捕获脉冲计数，用在中断中
extern unsigned int g_nSpeedControlCount;
extern float g_fCarAngle;
extern int g_iLeftTurnRoundCnt, g_iRightTurnRoundCnt;
extern unsigned int g_nLeftPulseTotal, g_nRightPulseTotal;
extern int g_nSpeedTarget;
extern int g_nLeftMotorOutput;
extern int g_nSpeedControlPeriod;//速度环控制周期计算量

enum ACTION_MODE{
	STOP_MODE = 0, FORWARD_MODE, BACKWARD_MODE, LEFTMOVE_MODE, RIGHTMOVE_MODE, TAILING_MODE, SONIC_MODE
};
extern enum ACTION_MODE g_currentMode;

//extern char g_SonicDoing, g_SonicAction;
extern int g_iCurrentDeg;
float GetDirect(void);
void SpeedControlOutput(void);
void GetMpuData(void);
void AngleCalculate(void);
void GetMotorPulse(void);
int SpeedInnerControl(int nPulse, int nTarget, int nPwm, int nErrorPrev);
void SetMotorVoltageAndDirection(int nLeftMotorPwm,int nRightMotorPwm);
void MotorOutput(void);
void AngleControl(void);
void SpeedControl(void);
void SetMode(enum ACTION_MODE mode);
void RunMode(void);
void Steer(float direct, float speed);
int KeepDirect(int still);
#endif
