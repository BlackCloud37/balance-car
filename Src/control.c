
#include "control.h"
#include "filter.h"
#include "mpu6050.h"
#include "math.h"
#include "outputdata.h"
#include "tim.h"
#include "main.h"
#include "infrare.h"

#define MOTOR_OUT_DEAD_VAL       0	                           //死区值
#define MOTOR_OUT_MAX           1000	                         //占空比正最大值
#define MOTOR_OUT_MIN         (-1000)                          //占空比负最大值

#define CAR_ANGLE_SET 0                                        //目标角度
#define CAR_ANGLE_SPEED_SET 0                                  //目标角速度

#define CAR_ZERO_ANGLE  (g_fCarAngleOffset)                    //机械零点偏移值

#define CAR_SPEED_SET (g_iCarSpeedSet)                         //小车目标速度
#define CAR_POSITION_MAX 900                                   //路程（速度积分）上限
#define CAR_POSITION_MIN (-900)                                //路程（速度积分）下限
#define SPEED_CONTROL_PERIOD    25                             //速度环控制周期
#define PULSE_PER_CM            70                             //每厘米的pulse数目

float g_fCarAngleOffset = 2;                                   //每辆小车的机械零点都不一定相同
short x_nAcc,y_nAcc,z_nAcc;                                    //加速度x轴、y轴、z轴数据
short x_nGyro,y_nGyro,z_nGyro;                                 //陀螺仪x轴、y轴、z轴数据
float x_fAcc,y_fAcc,z_fAcc;                                    //用于存储加速度x轴、y轴、z轴数据运算后的数据
float g_fAccAngle;                                             //加速度传感器经过atan2()解算得到的角度
float g_fGyroAngleSpeed;                                       //陀螺仪角速度
float g_fCarAngle;                                             //小车倾角
float dt = 0.005;                                              //互补滤波器控制周期

unsigned int g_nMainEventCount;                                //主事件计数，用在中断中
unsigned int g_nSpeedControlCount;                             //速度控制计数，用在中断中

int nPwmBais;                                                  //PWM增量
int nLeftMotorPwm,nRightMotorPwm;                              //左电机PWM输出总量，左电机PWM输出总量
int nLeftMotorErrorPrev,nRightMotorErrorPrev;                  //左电机上一次偏差，右电机上一次偏差

float g_fLeftMotorOut,g_fRightMotorOut;
float g_fAngleControlOut;


float g_fSpeedControlOut,g_fSpeedControlOutNew,g_fSpeedControlOutOld; //速度环输出
int g_nSpeedControlPeriod;                                            //速度环控制周期计算量
float g_fCarSpeed;                                                    //小车实际速度
float g_fCarSpeedPrev;                                                //小车前一次速度
float g_fCarPosition;                                                 //小车路程
long g_lLeftMotorPulseSigma;                                          //左电机25ms内累计脉冲总和
long g_lRightMotorPulseSigma;                                         //右电机25ms内累计脉冲总和
float g_fSpeedControlOut;                                             //速度环输出


/* 运动控制 */
enum ACTION_MODE g_currentMode = STOP_MODE;
float g_fSpeed;
float g_fCarSpeed;
float g_iCarSpeedSet;// 速度
float g_fCarSpeedOld;
float g_fCarPosition;
//int g_nLeftMotorPulse, g_nRightMotorPulse;                   //全局变量，保存电机脉冲数值(即距离)
// 方向
float g_fDirection,  // 方向
	g_fDirectionOld, g_fDirectionNew, g_fDirectionOut;

int g_iLeftTurnRoundCnt = 0;
int g_iRightTurnRoundCnt = 0;



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
		int r = -((short)(__HAL_TIM_GET_COUNTER(&htim4)));//获取计数器值;
    __HAL_TIM_SET_COUNTER(&htim4,0);//TIM4计数器清零
    int l = (short)(__HAL_TIM_GET_COUNTER(&htim2));//获取计数器值
    __HAL_TIM_SET_COUNTER(&htim2,0);//TIM2计数器清零

    g_lLeftMotorPulseSigma += l;//速度外环使用的脉冲累积
    g_lRightMotorPulseSigma += r;//速度外环使用的脉冲累积
		g_iLeftTurnRoundCnt -= l;    // 运动距离控制
		g_iRightTurnRoundCnt -= r;   // 运动距离控制
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
	g_fLeftMotorOut  = g_fAngleControlOut - g_fSpeedControlOut - g_fDirection ;	//这里的电机输出等于角度环控制量 + 速度环外环,这里的 - g_fSpeedControlOut 是因为速度环的极性跟角度环不一样，角度环是负反馈，速度环是正反馈
	g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut + g_fDirection ;

	/*增加死区常数*/
	if((int)g_fLeftMotorOut>0)       g_fLeftMotorOut  += MOTOR_OUT_DEAD_VAL;
	else if((int)g_fLeftMotorOut<0)  g_fLeftMotorOut  -= MOTOR_OUT_DEAD_VAL;
	if((int)g_fRightMotorOut>0)      g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
	else if((int)g_fRightMotorOut<0) g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;

	/*输出饱和处理，防止超出PWM范围*/			
	if((int)g_fLeftMotorOut  > MOTOR_OUT_MAX)	g_fLeftMotorOut  = MOTOR_OUT_MAX;
	if((int)g_fLeftMotorOut  < MOTOR_OUT_MIN)	g_fLeftMotorOut  = MOTOR_OUT_MIN;
	if((int)g_fRightMotorOut > MOTOR_OUT_MAX)	g_fRightMotorOut = MOTOR_OUT_MAX;
	if((int)g_fRightMotorOut < MOTOR_OUT_MIN)	g_fRightMotorOut = MOTOR_OUT_MIN;
	
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
    float fP=10.25,fI=0.108; //速度环PI参数，    
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

// 根据min, max归一化输入输出
float Scale(float input, float inputMin, float inputMax, float outputMin, float outputMax) { 
  float output;
  if (inputMin < inputMax)
    output = (input - inputMin) / ((inputMax - inputMin) / (outputMax - outputMin));
  else
    output = (inputMin - input) / ((inputMin - inputMax) / (outputMax - outputMin));
  if (output > outputMax)
    output = outputMax;
  else if (output < outputMin)
    output = outputMin;
  return output;
}


// 运动(方向/速度)控制函数
void Steer(float direct, float speed)
{
	if(direct > 0)
		g_fDirection = Scale(direct, 0, 10, 0, 400);
	else
		g_fDirection = -Scale(direct, 0, -10, 0, 400);
	
	if(speed > 0)
		g_iCarSpeedSet = Scale(speed, 0, 10, 0, 70);
	else
		g_iCarSpeedSet = -Scale(speed, 0, -10, 0, 70);
}

void SetMode(enum ACTION_MODE mode) {
	g_currentMode = mode;
	switch(mode) {
		case FORWARD_MODE: {
			// < 0 < 0 stop
			g_iLeftTurnRoundCnt = 100 * PULSE_PER_CM;
			g_iRightTurnRoundCnt = 100 * PULSE_PER_CM;
			Steer(0, 3);
			break;
		}
		case BACKWARD_MODE: {
			// > 0 > 0 stop
			g_iLeftTurnRoundCnt = - 100 * PULSE_PER_CM;
			g_iRightTurnRoundCnt = - 100 * PULSE_PER_CM;
			Steer(0, -3);
			break;
		}
		case LEFTMOVE_MODE: {
			// < 0 < 0 stop
			Steer(-1, 3);
			g_iRightTurnRoundCnt = 85 * PULSE_PER_CM;
			g_iLeftTurnRoundCnt = g_iRightTurnRoundCnt - 24 * PULSE_PER_CM;
			break;
		}
		case RIGHTMOVE_MODE: {
			// < 0 < 0 stop
			Steer(1, 3);
			g_iLeftTurnRoundCnt = 85 * PULSE_PER_CM;
			g_iRightTurnRoundCnt = g_iLeftTurnRoundCnt - 24 * PULSE_PER_CM;
			break;
		}
		default: {
			Steer(0,0);
			break;
		}
	}
}
/*
int g_iIsCenter, g_iIsLeft; // 线在中间, 线在左边 
void Tailing(void) {
	int cnt = 0;
	if(La) cnt++;
	if(Lb) cnt++;
	if(Ra) cnt++;
	if(Rb) cnt++;
	
	float speed = 3, direct = 0;
	if (cnt == 4) {
		
		return;
	}
	
	if (cnt == 0) {
		// 没线了, 可能偏离轨道也可能在轨道上
		return;
	}
	else if (cnt >= 1) {
		if(Lb) {
			g_iIsCenter = 0;
			g_iIsLeft = 1;
		}
		else if(Rb) {
			g_iIsCenter = 0;
			g_iIsLeft = 0;
		}
		else if(La) {
			g_iIsCenter = 1;
			g_iIsLeft = 1;
		}
		else if(Ra) {
			g_iIsCenter = 1;
			g_iIsLeft = 0;
		}		
	}
	else {
		//
	}
	
	if (g_iIsCenter) {
		direct = 0;
	} else {
		if (g_iIsLeft) {
			direct = -5;
		} else {
			direct = 5;
		}
	}
	Steer(direct, speed);
}
*/

void Tailing(void) {
	// TODO: 增加异常检测情况判断
	int cnt = 0;
	float speed = 3, direct = 0;
	
	if(La) {
		cnt ++;
		direct -= 3;
	}
	if(Lb) {
		cnt ++;
		direct -= 5;
	}
	if(Ra) {
		cnt ++;
		direct += 3;
	}
	if(Rb) {
		cnt ++;
		direct += 5;
	}
	
	if (cnt == 4) {
		// 终点线?
		return;
	}
	Steer(direct, speed);
}

char g_SonicTodo[3], g_SonicDoing;
char g_SonicAction = 'f'; // 'l, r, f, b'
char g_SonicMem[2];

// return finished
int SonicFinished(void) {
  switch(g_SonicAction) {
		case 'f':{
			return 1;
		}
		case 'b':{
			if (g_iLeftTurnRoundCnt >= 0 && g_iRightTurnRoundCnt >= 0) {
				return 1;
			}
			return 0;
		}
		case 'l':{
			if (g_iRightTurnRoundCnt <= 0) {
				return 1;
			}
			return 0;
		}
		case 'r':{
			if (g_iLeftTurnRoundCnt <= 0) {
				return 1;
			}
			return 0;
		}
		default:{
			Steer(0,4);
			return 1;
		}
	}
}


void RunMode(void) {
	switch(g_currentMode) {
		case LEFTMOVE_MODE:
		case RIGHTMOVE_MODE:
		case FORWARD_MODE: {
			// < 0 < 0 stop
			if (g_iLeftTurnRoundCnt <= 0 && g_iRightTurnRoundCnt <= 0) 
				SetMode(STOP_MODE);
			break;
		}
		case BACKWARD_MODE: {
			// > 0 > 0 stop
			if (g_iLeftTurnRoundCnt >= 0 && g_iRightTurnRoundCnt >= 0) 
				SetMode(STOP_MODE);
			break;
		}
		case TAILING_MODE: {
			Tailing();
			break;
		}
		case SONIC_MODE: {
			if(Distance >= 0 && Distance <= 25) {
				// 行动队列为空
				if (!g_SonicTodo[0] && !g_SonicTodo[1] && !g_SonicTodo[2]) {
					if (Distance <= 10) {
						g_SonicTodo[0] = g_SonicTodo[1] = g_SonicTodo[2] = 'b';
					} else {
						char lastDirect = 'l';
						if (!g_SonicMem[0] && g_SonicMem[1])
							lastDirect = g_SonicMem[1];
						else
							lastDirect = g_SonicMem[0];
						
						if (lastDirect == 'l') {
							g_SonicTodo[0] = 'r';
							g_SonicTodo[1] = 'l';
							g_SonicTodo[2] = 'l';
						}
						else {
							g_SonicTodo[0] = 'l';
							g_SonicTodo[1] = 'r';
							g_SonicTodo[2] = 'r';
						}
						// 更新历史转弯记录
						g_SonicMem[0] = g_SonicMem[1];
						g_SonicMem[1] = g_SonicTodo[0];
					}
					g_SonicDoing = 0;
				}
				
				if(SonicFinished()) {
					g_SonicAction = g_SonicTodo[g_SonicDoing];
					if (g_SonicDoing == 1) {
						g_SonicMem[1] = g_SonicMem[0];
						g_SonicMem[0] = 0;
					}
					if (g_SonicDoing == 2) {
						g_SonicMem[1] = g_SonicAction; 
					}
					if (g_SonicAction == 'f') {
						Steer(0, 4);
					} 
					else if (g_SonicAction == 'b') {
						Steer(0, -4);
						g_iLeftTurnRoundCnt = g_iRightTurnRoundCnt = - 2 * PULSE_PER_CM;
					} 
					else if (g_SonicAction == 'r') {
						Steer(6, -1);
						g_iLeftTurnRoundCnt = 20 * PULSE_PER_CM;
					} 
					else if (g_SonicAction == 'l') {
						Steer(-6, -1);
						g_iRightTurnRoundCnt = 20 * PULSE_PER_CM;
					}
					g_SonicDoing++;
					g_SonicDoing = g_SonicDoing % 3;
				}
			} else {
				if (SonicFinished()) {
					g_SonicAction = 'f';
					g_SonicTodo[0] = g_SonicTodo[1] = g_SonicTodo[2] = g_SonicDoing = 0;
					Steer(0, 4);
				}
			}
			break;
		}
		default: {
			Steer(0,0);
			break;
		}
	}
}

