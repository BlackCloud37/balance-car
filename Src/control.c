
#include "control.h"
#include "filter.h"
#include "mpu6050.h"
#include "math.h"
#include "outputdata.h"
#include "tim.h"
#include "main.h"

#define MOTOR_OUT_DEAD_VAL       0	   //死区值
#define MOTOR_OUT_MAX           1000	   //占空比正最大值
#define MOTOR_OUT_MIN         (-1000)   //占空比负最大值

#define CAR_ANGLE_SET 0//目标角度
#define CAR_ANGLE_SPEED_SET 0//目标角速度

#define CAR_ZERO_ANGLE  (g_fCarAngleOffset)  //机械零点偏移值

#define CAR_SPEED_SET 0//小车目标速度
#define CAR_POSITION_MAX 900//路程（速度积分）上限
#define CAR_POSITION_MIN (-900)//路程（速度积分）下限
#define SPEED_CONTROL_PERIOD    25      //速度环控制周期


float g_fCarAngleOffset = 2;//每辆小车的机械零点都不一定相同
short x_nAcc,y_nAcc,z_nAcc;//加速度x轴、y轴、z轴数据
short x_nGyro,y_nGyro,z_nGyro;//陀螺仪x轴、y轴、z轴数据
float x_fAcc,y_fAcc,z_fAcc;//用于存储加速度x轴、y轴、z轴数据运算后的数据

float g_fAccAngle;//加速度传感器经过atan2()解算得到的角度
float g_fGyroAngleSpeed;//陀螺仪角速度
float g_fCarAngle;//小车倾角
float dt = 0.005;//互补滤波器控制周期

unsigned int g_nMainEventCount;//主事件计数，用在中断中
unsigned int g_nSpeedControlCount;//速度控制计数，用在中断中

unsigned int g_nLeftMotorPulse,g_nRightMotorPulse;//全局变量，保存左电机脉冲数值

int nPwmBais;//PWM增量
int nLeftMotorPwm,nRightMotorPwm;//左电机PWM输出总量，左电机PWM输出总量
int nLeftMotorErrorPrev,nRightMotorErrorPrev;//左电机上一次偏差，右电机上一次偏差

float g_fLeftMotorOut,g_fRightMotorOut;
float g_fAngleControlOut;


float g_fSpeedControlOut,g_fSpeedControlOutNew,g_fSpeedControlOutOld;//速度环输出
int g_nSpeedControlPeriod;//速度环控制周期计算量
float g_fCarSpeed;//小车实际速度
float g_fCarSpeedPrev;//小车前一次速度
float g_fCarPosition;//小车路程
long g_lLeftMotorPulseSigma;//左电机25ms内累计脉冲总和
long g_lRightMotorPulseSigma;//右电机25ms内累计脉冲总和
float g_fSpeedControlOut;//速度环输出

void GetMpuData(void)//读取MPU-6050数据
{
	MPU_Get_Accelerometer(&x_nAcc,&y_nAcc,&z_nAcc);//获取MPU6050加速度数据
	MPU_Get_Gyroscope(&x_nGyro,&y_nGyro,&z_nGyro); //获取MPU6050陀螺仪数据
}

void AngleCalculate(void)//角度计算
{
    //-------加速度数据处理--------------------------
    //量程为±2g时，灵敏度：16384 LSB/g
    x_fAcc = x_nAcc / 16384.0;
    y_fAcc = y_nAcc / 16384.0;
    z_fAcc = z_nAcc / 16384.0;

    g_fAccAngle = atan2(y_fAcc,z_fAcc) * 180.0 / 3.14;

    //-------陀螺仪数据处理-------------------------
    //范围为2000deg/s时，换算关系：16.4 LSB/(deg/s)
    g_fGyroAngleSpeed = x_nGyro / 16.4;  //计算角速度值                   

    //-------互补滤波---------------
    g_fCarAngle = ComplementaryFilter(g_fAccAngle, g_fGyroAngleSpeed, dt);

    g_fCarAngle = g_fCarAngle - CAR_ZERO_ANGLE;//减去机械零点偏移值

}

void GetMotorPulse(void)//读取电机脉冲
{
    g_nRightMotorPulse = (short)(__HAL_TIM_GET_COUNTER(&htim4));//获取计数器值
    g_nRightMotorPulse = (-g_nRightMotorPulse);
    __HAL_TIM_SET_COUNTER(&htim4,0);//TIM4计数器清零
    g_nLeftMotorPulse = (short)(__HAL_TIM_GET_COUNTER(&htim2));//获取计数器值
    __HAL_TIM_SET_COUNTER(&htim2,0);//TIM2计数器清零

    g_lLeftMotorPulseSigma += g_nLeftMotorPulse;//速度外环使用的脉冲累积
    g_lRightMotorPulseSigma += g_nRightMotorPulse;//速度外环使用的脉冲累积
}


int SpeedInnerControl(int nPulse, int nTarget, int nPwm, int nErrorPrev)//速度内环控制
{
	int nError;//偏差
	float fP = 10.0, fI = 0.9;//这里只用到PI，参数由电机的种类和负载决定

	nError = nPulse - nTarget;//偏差 = 目标速度 - 实际速度 
	
	nPwmBais = fP * (nError - nErrorPrev) + fI * nError;//增量式PI控制器
	
	nErrorPrev = nError;//保存上一次偏差
	
	nPwm += nPwmBais;//增量输出
	
	if(nPwm > 1000) nPwm = 1000;//输出饱和处理，限制上限，防止超出PWM量程
	if(nPwm <-1000) nPwm =-1000;
	
	//OutData[0]=(float)nPulse;//速度实际值
	//OutData[1]=(float)nTarget ;//速度目标值
	//OutData[2]=(float)nPwm;//PWM输出值
	
	return nPwm;//返回输出值
}

void SetMotorVoltageAndDirection(int nLeftMotorPwm,int nRightMotorPwm)//设置电机电压和方向
{
	if(nRightMotorPwm < 0)//反转
		{
			HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
			nRightMotorPwm = (-nRightMotorPwm);//如果计算值是负值，负值只是表示反转，先转负为正，因为PWM寄存器只能是正值
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, nRightMotorPwm);
		}else//正转
		{
			HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, nRightMotorPwm );
		}
	if(nLeftMotorPwm < 0)//反转
		{
			HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
			nLeftMotorPwm = (-nLeftMotorPwm);//如果计算值是负值，负值只是表示反转，先转负为正，因为PWM寄存器只能是正值
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, nLeftMotorPwm);
		}else//正转
		{
			HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, nLeftMotorPwm);
		}
		
}


void MotorOutput(void)//电机输出函数,将直立控制、速度控制、方向控制的输出量进行叠加,并加入死区常量，对输出饱和作出处理。
{

    g_fLeftMotorOut  = g_fAngleControlOut - g_fSpeedControlOut;//这里的电机输出等于角度环控制量 + 速度环外环,这里的 - g_fSpeedControlOut 是因为速度环的极性跟角度环不一样，角度环是负反馈，速度环是正反馈
    g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut;

    /*增加电机死区常数*/
    if((int)g_fLeftMotorOut>0)       g_fLeftMotorOut  += MOTOR_OUT_DEAD_VAL;
    else if((int)g_fLeftMotorOut<0)  g_fLeftMotorOut  -= MOTOR_OUT_DEAD_VAL;
    if((int)g_fRightMotorOut>0)      g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
    else if((int)g_fRightMotorOut<0) g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;

    /*输出饱和处理，防止超出PWM范围*/            
    if((int)g_fLeftMotorOut  > MOTOR_OUT_MAX)    g_fLeftMotorOut  = MOTOR_OUT_MAX;
    if((int)g_fLeftMotorOut  < MOTOR_OUT_MIN)    g_fLeftMotorOut  = MOTOR_OUT_MIN;
    if((int)g_fRightMotorOut > MOTOR_OUT_MAX)    g_fRightMotorOut = MOTOR_OUT_MAX;
    if((int)g_fRightMotorOut < MOTOR_OUT_MIN)    g_fRightMotorOut = MOTOR_OUT_MIN;

    SetMotorVoltageAndDirection((int)g_fLeftMotorOut,(int)g_fRightMotorOut);
}

void AngleControl(void)	 //角度环控制函数
{
	float fP = 65.0;//角度环P参数
	float fD = 2.3;//角度环D参数
	
	g_fAngleControlOut =  (CAR_ANGLE_SET - g_fCarAngle) * fP + (CAR_ANGLE_SPEED_SET - g_fGyroAngleSpeed) * fD;//PD控制器
}


// P0.8 I0.01
void SpeedControl(void)//速度外环控制函数
{
    float fP=12.25,fI=0.108; //速度环PI参数，    
    float fDelta;//临时变量，用于存储误差

    g_fCarSpeed = (g_lLeftMotorPulseSigma + g_lRightMotorPulseSigma ) / 2;//左轮和右轮的速度平均值等于小车速度
    g_lLeftMotorPulseSigma = g_lRightMotorPulseSigma = 0;      //全局变量，注意及时清零

    g_fCarSpeed = 0.7 * g_fCarSpeedPrev + 0.3 * g_fCarSpeed ;//低通滤波，使速度更平滑
    g_fCarSpeedPrev = g_fCarSpeed; //保存前一次速度  

    fDelta = CAR_SPEED_SET - g_fCarSpeed;//误差=目标速度-实际速度  
    g_fCarPosition += fDelta;//对速度误差进行积分   

    //设置积分上限设限
    if((int)g_fCarPosition > CAR_POSITION_MAX)    g_fCarPosition = CAR_POSITION_MAX;
    if((int)g_fCarPosition < CAR_POSITION_MIN)    g_fCarPosition = CAR_POSITION_MIN;

    g_fSpeedControlOutOld = g_fSpeedControlOutNew;//保存上一次输出

    g_fSpeedControlOutNew = fDelta * fP + g_fCarPosition * fI; //PI控制器，输出=误差*P+误差积分*I

}

void SpeedControlOutput(void)
{
  float fValue;
  fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld ;//速度计算量差值=本次速度计算量-上次速度计算量
  g_fSpeedControlOut = fValue * (g_nSpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_fSpeedControlOutOld;//速度计算量差值* 
}
