#ifndef SWITCH_H
#define SWITCH_H

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "motor.h"

#define SWITCH_X_REAR 				GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4) 
#define SWITCH_X_FRONT 				GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_5)	
#define SWITCH_X_PROTECT 			GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6) 

#define SWITCH_Y_REAR 				GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0) 
#define SWITCH_Y_FRONT 				GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_1) 
#define SWITCH_Y_PROTECT			GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2) 

#define SWITCH_Z_REAR 				GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_12)
#define SWITCH_Z_FRONT 				GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_13) 
#define SWITCH_Z_PROTECT			GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_14) 

#define SWITCH_T_REAR 		1
#define SWITCH_T_FRONT 	1
#define SWITCH_T_PROTECT	1

//Y轴限位开关中断初始化 两个机械开关（前后各一个），一个光电开关 用于保护电机
void DRV_SWITCH_X_Init(void);

void DRV_SWITCH_Y_Init(void);

void DRV_SWITCH_Z_Init(void);

void DRV_SWITCH_T_Init(void);

void DRV_switch_Init(void);

MOTOR_STATUS DRV_switch_judge(MOTOR *motor);

void DRV_switch_deal(CONTROL *control);


#endif   //SWITCH_H
