#include "switch.h"

//若不使用限位开关，则关闭掉宏定义
#define USE_PROTECT_SWITCH	0

MOTOR motor_temp;

void DRV_switch_Init(void)
{
	DRV_SWITCH_X_Init();
	DRV_SWITCH_Y_Init();
	DRV_SWITCH_Z_Init();
}

//Y轴限位开关中断初始化 两个机械开关（前后各一个），一个光电开关 用于保护电机
void DRV_SWITCH_X_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef   NVIC_InitStructure;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//配置74HC245M，全部配置为低电平
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);   //配置DIR1 为低电平表示B->A
	GPIO_ResetBits(GPIOB,GPIO_Pin_13);	 //使能OE1 低电平有效
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);	 //使能OE2 低电平有效
	GPIO_ResetBits(GPIOB,GPIO_Pin_15);	 //配置DIR2 低电平表示B->A
	
	//中断
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource4);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource5);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource7);
	
	/* 配置EXTI_Line4 5  机械开关上升沿*/  
	EXTI_InitStruct.EXTI_Line = EXTI_Line4 | EXTI_Line5 | EXTI_Line7;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;//中断线使能
  EXTI_Init(&EXTI_InitStruct);//配置
	
#if USE_PROTECT_SWITCH	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource6);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line6;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;//中断线使能
  EXTI_Init(&EXTI_InitStruct);//配置
#endif

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;//外部中断4
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断9-5
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//抢占优先级2
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
}

//Y轴限位开关中断初始化 两个机械开关（前后各一个），一个光电开关 用于保护电机
void DRV_SWITCH_Y_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef   NVIC_InitStructure;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
	//中断
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);//PE2 连接到中断线2
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource1);//PE3 连接到中断线3
	
	/* 配置EXTI_Line0 1  机械开关上升沿*/  
	EXTI_InitStruct.EXTI_Line = EXTI_Line0 | EXTI_Line1;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;//中断线使能
  EXTI_Init(&EXTI_InitStruct);//配置
	/*配置EXTI_Line2 光电开关  下降沿触发中断*/

#if USE_PROTECT_SWITCH	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource2);//PE4 连接到中断线4

	EXTI_InitStruct.EXTI_Line = EXTI_Line2;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;//中断线使能
  EXTI_Init(&EXTI_InitStruct);//配置
#endif
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//外部中断4
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;//抢占优先级2
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
}

//Y轴限位开关中断初始化 两个机械开关（前后各一个），一个光电开关 用于保护电机
void DRV_SWITCH_Z_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef   NVIC_InitStructure;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
	//中断
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource12);//PE2 连接到中断线2
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource13);//PE3 连接到中断线3
	
	/* 配置EXTI_Line12 13  机械开关上升沿*/  
	EXTI_InitStruct.EXTI_Line = EXTI_Line12 | EXTI_Line13;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;//中断线使能
  EXTI_Init(&EXTI_InitStruct);//配置
	/*配置EXTI_Line14 光电开关  下降沿触发中断*/
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//外部中断4
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
#if USE_PROTECT_SWITCH	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource14);//PE4 连接到中断线4

	EXTI_InitStruct.EXTI_Line = EXTI_Line14;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;//中断线使能
  EXTI_Init(&EXTI_InitStruct);//配置

#endif
}


//外部中断0服务程序
void EXTI0_IRQHandler(void)
{	
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)			//机械开关  后
	{	
			motor_temp.motor_type = MOTOR_Y;
			motor_temp.switch_rear = 1;
			DRV_motor_set_switch(motor_temp);
			motor_temp.switch_rear = 0;
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}
//外部中断01服务程序
void EXTI1_IRQHandler(void)
{	
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)			//机械开关 前
	{	
			motor_temp.motor_type = MOTOR_Y;
			motor_temp.switch_front = 1;
			DRV_motor_set_switch(motor_temp);
			motor_temp.switch_front = 0;
	}
	EXTI_ClearITPendingBit(EXTI_Line1);
}

//外部中断02服务程序
void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)			//保护开关
	{
			motor_temp.motor_type = MOTOR_Y;
			motor_temp.switch_protect = 1;
			DRV_motor_set_switch(motor_temp);
			motor_temp.switch_protect = 0;		
	}
	EXTI_ClearITPendingBit(EXTI_Line2);
}
//外部中断04服务程序
void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4) != RESET)			//机械开关  后
	{
			motor_temp.motor_type = MOTOR_X;
			motor_temp.switch_rear = 1;
			DRV_motor_set_switch(motor_temp);
			motor_temp.switch_rear = 0;
	}
	EXTI_ClearITPendingBit(EXTI_Line4);
}	
//外部中断5-9服务程序
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)				//机械开关  前
	{		
		motor_temp.motor_type = MOTOR_X;
		motor_temp.switch_front = 1;
		DRV_motor_set_switch(motor_temp);
		motor_temp.switch_front = 0;
	
		EXTI_ClearITPendingBit(EXTI_Line5);//清除LINE5上的中断标志位 
	}
	else if(EXTI_GetITStatus(EXTI_Line6) != RESET)   //光电开关
	{		
		if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6) == 1)
		{
			motor_temp.motor_type = MOTOR_X;
			motor_temp.switch_protect = 1;
			DRV_motor_set_switch(motor_temp);
			motor_temp.switch_protect = 0;
		}
		EXTI_ClearITPendingBit(EXTI_Line6);//清除LINE6上的中断标志位 
	}
	else if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line7);
	}
	else if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
	else if(EXTI_GetITStatus(EXTI_Line9) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line9);
	}

}

void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line10) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line10);
	}
	else if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line11);
	}
	else if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{		
			motor_temp.motor_type = MOTOR_Z;
			motor_temp.switch_rear = 1;
			DRV_motor_set_switch(motor_temp);
			motor_temp.switch_rear = 0;
		
			EXTI_ClearITPendingBit(EXTI_Line12);
	}
	else if(EXTI_GetITStatus(EXTI_Line13) != RESET)
	{					
			motor_temp.motor_type = MOTOR_Z;
			motor_temp.switch_front = 1;
			DRV_motor_set_switch(motor_temp);
			motor_temp.switch_front = 0;
		
		 EXTI_ClearITPendingBit(EXTI_Line13);
	}
	else if(EXTI_GetITStatus(EXTI_Line14) != RESET)
	{		
			motor_temp.motor_type = MOTOR_Z;
			motor_temp.switch_protect = 1;
			DRV_motor_set_switch(motor_temp);
			motor_temp.switch_protect = 0;
		
		 EXTI_ClearITPendingBit(EXTI_Line14);
	}
}


//判断当前的开关状态  返回状态有三种  STATUS_FRONT  STATUS_STOP  STATUS_REAR
MOTOR_STATUS DRV_switch_judge(MOTOR *motor)
{
	MOTOR_STATUS status = STATUS_STOP;
	switch(motor->motor_type)
	{
		case MOTOR_X:
		{
			//机械开关上升沿触发中断，闭合时候为高电平
			if(SWITCH_X_REAR == 0)
			{
				status = STATUS_REAR;
			} 
			else if (SWITCH_X_FRONT == 0)
			{
				status = STATUS_FRONT;
			}
			else
			{
				status = STATUS_STOP;
			}
		}
			break;
		case MOTOR_Y:
			//机械开关上升沿触发中断，闭合时候为高电平
			if(SWITCH_Y_REAR == 0)
			{
				status = STATUS_REAR;
			} 
			else if(SWITCH_Y_FRONT == 0)
			{
				status = STATUS_FRONT;
			}
			else
			{
				status = STATUS_STOP;
			}
			break;
		case MOTOR_Z:
			//机械开关上升沿触发中断，闭合时候为高电平
			if(SWITCH_Z_REAR == 0)
			{
				status = STATUS_REAR;
			} 
			else if(SWITCH_Z_FRONT == 0)
			{
				status = STATUS_FRONT;
			}
			else
			{
				status = STATUS_STOP;
			}
			break;
		case MOTOR_T:
			//机械开关上升沿触发中断，闭合时候为高电平
			if(SWITCH_T_REAR == 0)
			{
				status = STATUS_REAR;
			} 
			else if(SWITCH_T_FRONT == 0)
			{
				status = STATUS_FRONT;
			}
			else
			{
				status = STATUS_STOP;
			}
			break;
		default :
			//出错
			status = NON;
			break;
		
	}
	
	return status;
}


void DRV_switch_deal(CONTROL *control)
{
	
//	if((control->motor_x->status != STATUS_GO_AHEAD) && (control->motor_y->status != STATUS_GO_AHEAD) && (control->motor_z->status != STATUS_GO_AHEAD))
//			return ;
	
	if(control->motor_x->switch_front == 1)
	{
		control->motor_x->switch_front = 0;
		
		if(control->motor_x->motor_dir != MOTOR_POSTIVE)
		{
			control->motor_x->status = STATUS_STOP;
			control->motor_x->curren_location = 0;
			DRV_motor_stop(control->motor_x);
		}
	}
	else if(control->motor_x->switch_rear == 1)
	{
		control->motor_x->switch_rear = 0;
		
		if(control->motor_x->motor_dir != MOTOR_NEGATIVE)
		{
			control->motor_x->status = STATUS_STOP;
			DRV_motor_stop(control->motor_x);
		}
	}
	else if(control->motor_x->switch_protect == 1)
	{
		control->motor_x->switch_protect = 0;
		if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6) == 1)
		if(control->motor_x->motor_dir != MOTOR_POSTIVE)
		{
			control->motor_x->status = STATUS_STOP;
			control->motor_x->curren_location = 0;
			DRV_motor_stop(control->motor_x);
		}
	}
	
	
	if(control->motor_y->switch_front == 1)
	{
		control->motor_y->switch_front = 0;
		
		if(control->motor_y->motor_dir != MOTOR_POSTIVE)
		{
			control->motor_y->status = STATUS_STOP;
			control->motor_y->curren_location = 0;
			DRV_motor_stop(control->motor_y);
		}
	}
	else if(control->motor_y->switch_rear == 1)
	{
		control->motor_y->switch_rear = 0;
		
		if(control->motor_y->motor_dir != MOTOR_NEGATIVE)
		{
			control->motor_y->status = STATUS_STOP;
			DRV_motor_stop(control->motor_y);
		}
	}
	else if(control->motor_y->switch_protect == 1)
	{
		control->motor_y->switch_protect = 0;
		if(control->motor_y->motor_dir != MOTOR_POSTIVE)
		{
			control->motor_y->status = STATUS_STOP;
			control->motor_y->curren_location = 0;
			DRV_motor_stop(control->motor_y);
		}
	}
	
	
	
	if(control->motor_z->switch_front == 1)
	{
		control->motor_z->switch_front = 0;
		
		if(control->motor_z->motor_dir != MOTOR_POSTIVE)
		{
			control->motor_z->status = STATUS_STOP;
			control->motor_z->curren_location = 0;
			DRV_motor_stop(control->motor_z);
		}
	}
	else if(control->motor_z->switch_rear == 1)
	{
		control->motor_z->switch_rear = 0;
		
		if(control->motor_z->motor_dir != MOTOR_NEGATIVE)
		{
			control->motor_z->status = STATUS_STOP;
			DRV_motor_stop(control->motor_z);
		}
	}
	else if(control->motor_z->switch_protect == 1)
	{
		control->motor_z->switch_protect = 0;
		if(control->motor_z->motor_dir != MOTOR_POSTIVE)
		{
			control->motor_z->status = STATUS_STOP;
			control->motor_z->curren_location = 0;
			DRV_motor_stop(control->motor_z);
		}
	}
}







