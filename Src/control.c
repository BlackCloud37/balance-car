
#include "control.h"
#include "filter.h"
#include "mpu6050.h"
#include "math.h"
#include "outputdata.h"
#include "tim.h"
#include "main.h"
#include "infrare.h"

#define MOTOR_OUT_DEAD_VAL       0	                           //����ֵ
#define MOTOR_OUT_MAX           1000	                         //ռ�ձ������ֵ
#define MOTOR_OUT_MIN         (-1000)                          //ռ�ձȸ����ֵ

#define CAR_ANGLE_SET (-1)                                     //Ŀ��Ƕ�
#define CAR_ANGLE_SPEED_SET 0                                  //Ŀ����ٶ�

#define CAR_ZERO_ANGLE  (g_fCarAngleOffset)                    //��е���ƫ��ֵ

#define CAR_SPEED_SET (g_iCarSpeedSet)                         //С��Ŀ���ٶ�
#define CAR_POSITION_MAX 900                                   //·�̣��ٶȻ��֣�����
#define CAR_POSITION_MIN (-900)                                //·�̣��ٶȻ��֣�����
#define SPEED_CONTROL_PERIOD    25                             //�ٶȻ���������
#define PULSE_PER_CM            70                             //ÿ���׵�pulse��Ŀ

float g_fCarAngleOffset = 0;                                   //ÿ��С���Ļ�е��㶼��һ����ͬ
short x_nAcc,y_nAcc,z_nAcc;                                    //���ٶ�x�ᡢy�ᡢz������
short x_nGyro,y_nGyro,z_nGyro;                                 //������x�ᡢy�ᡢz������
float x_fAcc,y_fAcc,z_fAcc;                                    //���ڴ洢���ٶ�x�ᡢy�ᡢz����������������
float g_fAccAngle;                                             //���ٶȴ���������atan2()����õ��ĽǶ�
float g_fGyroAngleSpeed;                                       //�����ǽ��ٶ�
float g_fCarAngle;                                             //С�����
float dt = 0.005;                                              //�����˲�����������

unsigned int g_nMainEventCount;                                //���¼������������ж���
unsigned int g_nSpeedControlCount;                             //�ٶȿ��Ƽ����������ж���

int nPwmBais;                                                  //PWM����
int nLeftMotorPwm,nRightMotorPwm;                              //����PWM�������������PWM�������
int nLeftMotorErrorPrev,nRightMotorErrorPrev;                  //������һ��ƫ��ҵ����һ��ƫ��

float g_fLeftMotorOut,g_fRightMotorOut;
float g_fAngleControlOut;


float g_fSpeedControlOut,g_fSpeedControlOutNew,g_fSpeedControlOutOld; //�ٶȻ����
int g_nSpeedControlPeriod;                                            //�ٶȻ��������ڼ�����
float g_fCarSpeed;                                                    //С��ʵ���ٶ�
float g_fCarSpeedPrev;                                                //С��ǰһ���ٶ�
float g_fCarPosition;                                                 //С��·��
long g_lLeftMotorPulseSigma;                                          //����25ms���ۼ������ܺ�
long g_lRightMotorPulseSigma;                                         //�ҵ��25ms���ۼ������ܺ�
float g_fSpeedControlOut;                                             //�ٶȻ����


/* �˶����� */
enum ACTION_MODE g_currentMode = STOP_MODE;
float g_fSpeed;
float g_fCarSpeed;
float g_iCarSpeedSet;// �ٶ�
float g_fCarSpeedOld;
float g_fCarPosition;
//int g_nLeftMotorPulse, g_nRightMotorPulse;                   //ȫ�ֱ�����������������ֵ(������)
// ����
float g_fDirection,  // ����
	g_fDirectionOld, g_fDirectionNew, g_fDirectionOut;

int g_iLeftTurnRoundCnt = 0;
int g_iRightTurnRoundCnt = 0;



void GetMpuData(void)//��ȡMPU-6050����
{
	MPU_Get_Accelerometer(&x_nAcc,&y_nAcc,&z_nAcc);//��ȡMPU6050���ٶ�����
	MPU_Get_Gyroscope(&x_nGyro,&y_nGyro,&z_nGyro); //��ȡMPU6050����������
}

void AngleCalculate(void)//�Ƕȼ���
{
    //-------���ٶ����ݴ���--------------------------
    //����Ϊ��2gʱ�������ȣ�16384 LSB/g
    x_fAcc = x_nAcc / 16384.0;
    y_fAcc = y_nAcc / 16384.0;
    z_fAcc = z_nAcc / 16384.0;

    g_fAccAngle = atan2(y_fAcc,z_fAcc) * 180.0 / 3.14;

    //-------���������ݴ���-------------------------
    //��ΧΪ2000deg/sʱ�������ϵ��16.4 LSB/(deg/s)
    g_fGyroAngleSpeed = x_nGyro / 16.4;  //������ٶ�ֵ                   

    //-------�����˲�---------------
    g_fCarAngle = ComplementaryFilter(g_fAccAngle, g_fGyroAngleSpeed, dt);

    g_fCarAngle = g_fCarAngle - CAR_ZERO_ANGLE;//��ȥ��е���ƫ��ֵ
}

void GetMotorPulse(void)//��ȡ�������
{
		int r = -((short)(__HAL_TIM_GET_COUNTER(&htim4)));//��ȡ������ֵ;
    __HAL_TIM_SET_COUNTER(&htim4,0);//TIM4����������
    int l = (short)(__HAL_TIM_GET_COUNTER(&htim2));//��ȡ������ֵ
    __HAL_TIM_SET_COUNTER(&htim2,0);//TIM2����������

    g_lLeftMotorPulseSigma += l;//�ٶ��⻷ʹ�õ������ۻ�
    g_lRightMotorPulseSigma += r;//�ٶ��⻷ʹ�õ������ۻ�
		g_iLeftTurnRoundCnt -= l;    // �˶��������
		g_iRightTurnRoundCnt -= r;   // �˶��������
}


int SpeedInnerControl(int nPulse, int nTarget, int nPwm, int nErrorPrev)//�ٶ��ڻ�����
{
	int nError;//ƫ��
	float fP = 10.0, fI = 0.9;//����ֻ�õ�PI�������ɵ��������͸��ؾ���

	nError = nPulse - nTarget;//ƫ�� = Ŀ���ٶ� - ʵ���ٶ� 
	
	nPwmBais = fP * (nError - nErrorPrev) + fI * nError;//����ʽPI������
	
	nErrorPrev = nError;//������һ��ƫ��
	
	nPwm += nPwmBais;//�������
	
	if(nPwm > 1000) nPwm = 1000;//������ʹ����������ޣ���ֹ����PWM����
	if(nPwm <-1000) nPwm =-1000;
	
	//OutData[0]=(float)nPulse;//�ٶ�ʵ��ֵ
	//OutData[1]=(float)nTarget ;//�ٶ�Ŀ��ֵ
	//OutData[2]=(float)nPwm;//PWM���ֵ
	
	return nPwm;//�������ֵ
}

void SetMotorVoltageAndDirection(int nLeftMotorPwm,int nRightMotorPwm)//���õ����ѹ�ͷ���
{
	if(nRightMotorPwm < 0)//��ת
		{
			HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
			nRightMotorPwm = (-nRightMotorPwm);//�������ֵ�Ǹ�ֵ����ֵֻ�Ǳ�ʾ��ת����ת��Ϊ������ΪPWM�Ĵ���ֻ������ֵ
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, nRightMotorPwm);
		}else//��ת
		{
			HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, nRightMotorPwm );
		}
	if(nLeftMotorPwm < 0)//��ת
		{
			HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
			nLeftMotorPwm = (-nLeftMotorPwm);//�������ֵ�Ǹ�ֵ����ֵֻ�Ǳ�ʾ��ת����ת��Ϊ������ΪPWM�Ĵ���ֻ������ֵ
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, nLeftMotorPwm);
		}else//��ת
		{
			HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, nLeftMotorPwm);
		}
}


void MotorOutput(void)//����������,��ֱ�����ơ��ٶȿ��ơ�������Ƶ���������е���,���������������������������������
{
	g_fLeftMotorOut  = g_fAngleControlOut - g_fSpeedControlOut - g_fDirection ;	//����ĵ��������ڽǶȻ������� + �ٶȻ��⻷,����� - g_fSpeedControlOut ����Ϊ�ٶȻ��ļ��Ը��ǶȻ���һ�����ǶȻ��Ǹ��������ٶȻ���������
	g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut + g_fDirection ;

	/*������������*/
	if((int)g_fLeftMotorOut>0)       g_fLeftMotorOut  += MOTOR_OUT_DEAD_VAL;
	else if((int)g_fLeftMotorOut<0)  g_fLeftMotorOut  -= MOTOR_OUT_DEAD_VAL;
	if((int)g_fRightMotorOut>0)      g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
	else if((int)g_fRightMotorOut<0) g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;

	/*������ʹ�����ֹ����PWM��Χ*/			
	if((int)g_fLeftMotorOut  > MOTOR_OUT_MAX)	g_fLeftMotorOut  = MOTOR_OUT_MAX;
	if((int)g_fLeftMotorOut  < MOTOR_OUT_MIN)	g_fLeftMotorOut  = MOTOR_OUT_MIN;
	if((int)g_fRightMotorOut > MOTOR_OUT_MAX)	g_fRightMotorOut = MOTOR_OUT_MAX;
	if((int)g_fRightMotorOut < MOTOR_OUT_MIN)	g_fRightMotorOut = MOTOR_OUT_MIN;
	
  SetMotorVoltageAndDirection((int)g_fLeftMotorOut,(int)g_fRightMotorOut);
}

void AngleControl(void)	 //�ǶȻ����ƺ���
{
	float fP = 65.0;//�ǶȻ�P����
	float fD = 2.3;//�ǶȻ�D����
	
	g_fAngleControlOut =  (CAR_ANGLE_SET - g_fCarAngle) * fP + (CAR_ANGLE_SPEED_SET - g_fGyroAngleSpeed) * fD;//PD������
}


// P0.8 I0.01
void SpeedControl(void)//�ٶ��⻷���ƺ���
{
    float fP=10.25,fI=0.108; //�ٶȻ�PI������    
    float fDelta;//��ʱ���������ڴ洢���

    g_fCarSpeed = (g_lLeftMotorPulseSigma + g_lRightMotorPulseSigma ) / 2;//���ֺ����ֵ��ٶ�ƽ��ֵ����С���ٶ�
    g_lLeftMotorPulseSigma = g_lRightMotorPulseSigma = 0;      //ȫ�ֱ�����ע�⼰ʱ����

    g_fCarSpeed = 0.7 * g_fCarSpeedPrev + 0.3 * g_fCarSpeed ;//��ͨ�˲���ʹ�ٶȸ�ƽ��
    g_fCarSpeedPrev = g_fCarSpeed; //����ǰһ���ٶ�  

    fDelta = CAR_SPEED_SET - g_fCarSpeed;//���=Ŀ���ٶ�-ʵ���ٶ�  
    g_fCarPosition += fDelta;//���ٶ������л���   

    //���û�����������
    if((int)g_fCarPosition > CAR_POSITION_MAX)    g_fCarPosition = CAR_POSITION_MAX;
    if((int)g_fCarPosition < CAR_POSITION_MIN)    g_fCarPosition = CAR_POSITION_MIN;

    g_fSpeedControlOutOld = g_fSpeedControlOutNew;//������һ�����

    g_fSpeedControlOutNew = fDelta * fP + g_fCarPosition * fI; //PI�����������=���*P+������*I

}

void SpeedControlOutput(void)
{
  float fValue;
  fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld ;//�ٶȼ�������ֵ=�����ٶȼ�����-�ϴ��ٶȼ�����
  g_fSpeedControlOut = fValue * (g_nSpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_fSpeedControlOutOld;//�ٶȼ�������ֵ* 
}

// ����min, max��һ���������
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


// �˶�(����/�ٶ�)���ƺ���
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

int g_iIsCenter, g_iIsLeft, g_oldWeight; // �����м�, ������� 
void Tailing(void) {
	int cnt = 0;
	if(La) cnt++;
	if(Lb) cnt++;
	if(Ra) cnt++;
	if(Rb) cnt++;
	
	int weight = 0;
	if (Lb) weight -= 2;
	if (La) weight -= 1;
	if (Ra) weight += 1;
	if (Rb) weight += 2;
	
	float speed = 0.8, direct = 0;
	
	if (cnt == 4) {
		SetMode(STOP_MODE); // TODO: switch to sonic mode
		return;
	}
	
	if (cnt == 0) {
		direct = g_oldWeight * 4;
	} else {
		direct = (weight - g_oldWeight) * 4;
		g_oldWeight = weight;
	}
	Steer(direct, speed);
}

/*
float g_fOldDirect = 0;
void Tailing(void) {
	// TODO: �����쳣�������ж�
	int cnt = 0;
	float speed = 1, direct = 0;
	
	if(La) {
		cnt ++;
		direct -= 3;
	}
	if(Lb) {
		cnt ++;
		direct -= 6;
	}
	if(Ra) {
		cnt ++;
		direct += 3;
	}
	if(Rb) {
		cnt ++;
		direct += 6;
	}
	
	if (cnt == 4) {
		// �յ���?
		return;
	}
	if (cnt == 0) {
		direct = g_fOldDirect;
	}
	g_fOldDirect = direct;
	Steer(direct, speed);
}
*/
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
				// �ж�����Ϊ��
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
						// ������ʷת���¼
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

