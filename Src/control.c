
#include "control.h"
#include "filter.h"
#include "mpu6050.h"
#include "math.h"
#include "outputdata.h"
#include "tim.h"
#include "main.h"

#define MOTOR_OUT_DEAD_VAL       0	   //����ֵ
#define MOTOR_OUT_MAX           1000	   //ռ�ձ������ֵ
#define MOTOR_OUT_MIN         (-1000)   //ռ�ձȸ����ֵ

#define CAR_ANGLE_SET 0//Ŀ��Ƕ�
#define CAR_ANGLE_SPEED_SET 0//Ŀ����ٶ�

#define CAR_ZERO_ANGLE  (g_fCarAngleOffset)  //��е���ƫ��ֵ

#define CAR_SPEED_SET 0//С��Ŀ���ٶ�
#define CAR_POSITION_MAX 900//·�̣��ٶȻ��֣�����
#define CAR_POSITION_MIN (-900)//·�̣��ٶȻ��֣�����
#define SPEED_CONTROL_PERIOD    25      //�ٶȻ���������


float g_fCarAngleOffset = 2;//ÿ��С���Ļ�е��㶼��һ����ͬ
short x_nAcc,y_nAcc,z_nAcc;//���ٶ�x�ᡢy�ᡢz������
short x_nGyro,y_nGyro,z_nGyro;//������x�ᡢy�ᡢz������
float x_fAcc,y_fAcc,z_fAcc;//���ڴ洢���ٶ�x�ᡢy�ᡢz����������������

float g_fAccAngle;//���ٶȴ���������atan2()����õ��ĽǶ�
float g_fGyroAngleSpeed;//�����ǽ��ٶ�
float g_fCarAngle;//С�����
float dt = 0.005;//�����˲�����������

unsigned int g_nMainEventCount;//���¼������������ж���
unsigned int g_nSpeedControlCount;//�ٶȿ��Ƽ����������ж���

unsigned int g_nLeftMotorPulse,g_nRightMotorPulse;//ȫ�ֱ�������������������ֵ

int nPwmBais;//PWM����
int nLeftMotorPwm,nRightMotorPwm;//����PWM�������������PWM�������
int nLeftMotorErrorPrev,nRightMotorErrorPrev;//������һ��ƫ��ҵ����һ��ƫ��

float g_fLeftMotorOut,g_fRightMotorOut;
float g_fAngleControlOut;


float g_fSpeedControlOut,g_fSpeedControlOutNew,g_fSpeedControlOutOld;//�ٶȻ����
int g_nSpeedControlPeriod;//�ٶȻ��������ڼ�����
float g_fCarSpeed;//С��ʵ���ٶ�
float g_fCarSpeedPrev;//С��ǰһ���ٶ�
float g_fCarPosition;//С��·��
long g_lLeftMotorPulseSigma;//����25ms���ۼ������ܺ�
long g_lRightMotorPulseSigma;//�ҵ��25ms���ۼ������ܺ�
float g_fSpeedControlOut;//�ٶȻ����

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
    g_nRightMotorPulse = (short)(__HAL_TIM_GET_COUNTER(&htim4));//��ȡ������ֵ
    g_nRightMotorPulse = (-g_nRightMotorPulse);
    __HAL_TIM_SET_COUNTER(&htim4,0);//TIM4����������
    g_nLeftMotorPulse = (short)(__HAL_TIM_GET_COUNTER(&htim2));//��ȡ������ֵ
    __HAL_TIM_SET_COUNTER(&htim2,0);//TIM2����������

    g_lLeftMotorPulseSigma += g_nLeftMotorPulse;//�ٶ��⻷ʹ�õ������ۻ�
    g_lRightMotorPulseSigma += g_nRightMotorPulse;//�ٶ��⻷ʹ�õ������ۻ�
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

    g_fLeftMotorOut  = g_fAngleControlOut - g_fSpeedControlOut;//����ĵ��������ڽǶȻ������� + �ٶȻ��⻷,����� - g_fSpeedControlOut ����Ϊ�ٶȻ��ļ��Ը��ǶȻ���һ�����ǶȻ��Ǹ��������ٶȻ���������
    g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut;

    /*���ӵ����������*/
    if((int)g_fLeftMotorOut>0)       g_fLeftMotorOut  += MOTOR_OUT_DEAD_VAL;
    else if((int)g_fLeftMotorOut<0)  g_fLeftMotorOut  -= MOTOR_OUT_DEAD_VAL;
    if((int)g_fRightMotorOut>0)      g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
    else if((int)g_fRightMotorOut<0) g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;

    /*������ʹ�����ֹ����PWM��Χ*/            
    if((int)g_fLeftMotorOut  > MOTOR_OUT_MAX)    g_fLeftMotorOut  = MOTOR_OUT_MAX;
    if((int)g_fLeftMotorOut  < MOTOR_OUT_MIN)    g_fLeftMotorOut  = MOTOR_OUT_MIN;
    if((int)g_fRightMotorOut > MOTOR_OUT_MAX)    g_fRightMotorOut = MOTOR_OUT_MAX;
    if((int)g_fRightMotorOut < MOTOR_OUT_MIN)    g_fRightMotorOut = MOTOR_OUT_MIN;

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
    float fP=12.25,fI=0.108; //�ٶȻ�PI������    
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
