#ifndef _INFRARE_H
#define _INFRARE_H

#define La HAL_GPIO_ReadPin(GPIOB, La_Pin)
#define Lb HAL_GPIO_ReadPin(GPIOB, Lb_Pin)
#define Ra HAL_GPIO_ReadPin(GPIOA, Ra_Pin)
#define Rb HAL_GPIO_ReadPin(GPIOA, Rb_Pin)

/*
typedef enum _infrared_channel_t{
	infrared_channel_La = (0x01 << 0),
	infrared_channel_Lb = (0x01 << 1),
	infrared_channel_Ra = (0x01 << 2),
	infrared_channel_Rb = (0x01 << 3),
}infrared_channel_t;


char InfraredDetect(void);
*/
void InfrareSelfCheck(void);
int IsInfrareOK(void);

#endif
