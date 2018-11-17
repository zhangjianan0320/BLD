#include "switch.h"

//����ʹ����λ���أ���رյ��궨��
#define USE_PROTECT_SWITCH	0

MOTOR motor_temp;

void DRV_switch_Init(void)
{
	DRV_SWITCH_X_Init();
	DRV_SWITCH_Y_Init();
	DRV_SWITCH_Z_Init();
}

//Y����λ�����жϳ�ʼ�� ������е���أ�ǰ���һ������һ����翪�� ���ڱ������
void DRV_SWITCH_X_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef   NVIC_InitStructure;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//����74HC245M��ȫ������Ϊ�͵�ƽ
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);   //����DIR1 Ϊ�͵�ƽ��ʾB->A
	GPIO_ResetBits(GPIOB,GPIO_Pin_13);	 //ʹ��OE1 �͵�ƽ��Ч
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);	 //ʹ��OE2 �͵�ƽ��Ч
	GPIO_ResetBits(GPIOB,GPIO_Pin_15);	 //����DIR2 �͵�ƽ��ʾB->A
	
	//�ж�
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
	
	/* ����EXTI_Line4 5  ��е����������*/  
	EXTI_InitStruct.EXTI_Line = EXTI_Line4 | EXTI_Line5 | EXTI_Line7;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;//�ж���ʹ��
  EXTI_Init(&EXTI_InitStruct);//����
	
#if USE_PROTECT_SWITCH	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource6);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line6;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;//�ж���ʹ��
  EXTI_Init(&EXTI_InitStruct);//����
#endif

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;//�ⲿ�ж�4
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//�ⲿ�ж�9-5
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//��ռ���ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����
	
}

//Y����λ�����жϳ�ʼ�� ������е���أ�ǰ���һ������һ����翪�� ���ڱ������
void DRV_SWITCH_Y_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef   NVIC_InitStructure;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
	//�ж�
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
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);//PE2 ���ӵ��ж���2
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource1);//PE3 ���ӵ��ж���3
	
	/* ����EXTI_Line0 1  ��е����������*/  
	EXTI_InitStruct.EXTI_Line = EXTI_Line0 | EXTI_Line1;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;//�ж���ʹ��
  EXTI_Init(&EXTI_InitStruct);//����
	/*����EXTI_Line2 ��翪��  �½��ش����ж�*/

#if USE_PROTECT_SWITCH	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource2);//PE4 ���ӵ��ж���4

	EXTI_InitStruct.EXTI_Line = EXTI_Line2;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; //�½��ش���
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;//�ж���ʹ��
  EXTI_Init(&EXTI_InitStruct);//����
#endif
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//�ⲿ�ж�4
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;//��ռ���ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����
	
}

//Y����λ�����жϳ�ʼ�� ������е���أ�ǰ���һ������һ����翪�� ���ڱ������
void DRV_SWITCH_Z_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef   NVIC_InitStructure;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
	//�ж�
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
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource12);//PE2 ���ӵ��ж���2
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource13);//PE3 ���ӵ��ж���3
	
	/* ����EXTI_Line12 13  ��е����������*/  
	EXTI_InitStruct.EXTI_Line = EXTI_Line12 | EXTI_Line13;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;//�ж���ʹ��
  EXTI_Init(&EXTI_InitStruct);//����
	/*����EXTI_Line14 ��翪��  �½��ش����ж�*/
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//�ⲿ�ж�4
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����
	
#if USE_PROTECT_SWITCH	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource14);//PE4 ���ӵ��ж���4

	EXTI_InitStruct.EXTI_Line = EXTI_Line14;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; //�½��ش���
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;//�ж���ʹ��
  EXTI_Init(&EXTI_InitStruct);//����

#endif
}


//�ⲿ�ж�0�������
void EXTI0_IRQHandler(void)
{	
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)			//��е����  ��
	{	
			motor_temp.motor_type = MOTOR_Y;
			motor_temp.switch_rear = 1;
			DRV_motor_set_switch(motor_temp);
			motor_temp.switch_rear = 0;
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}
//�ⲿ�ж�01�������
void EXTI1_IRQHandler(void)
{	
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)			//��е���� ǰ
	{	
			motor_temp.motor_type = MOTOR_Y;
			motor_temp.switch_front = 1;
			DRV_motor_set_switch(motor_temp);
			motor_temp.switch_front = 0;
	}
	EXTI_ClearITPendingBit(EXTI_Line1);
}

//�ⲿ�ж�02�������
void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)			//��������
	{
			motor_temp.motor_type = MOTOR_Y;
			motor_temp.switch_protect = 1;
			DRV_motor_set_switch(motor_temp);
			motor_temp.switch_protect = 0;		
	}
	EXTI_ClearITPendingBit(EXTI_Line2);
}
//�ⲿ�ж�04�������
void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4) != RESET)			//��е����  ��
	{
			motor_temp.motor_type = MOTOR_X;
			motor_temp.switch_rear = 1;
			DRV_motor_set_switch(motor_temp);
			motor_temp.switch_rear = 0;
	}
	EXTI_ClearITPendingBit(EXTI_Line4);
}	
//�ⲿ�ж�5-9�������
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)				//��е����  ǰ
	{		
		motor_temp.motor_type = MOTOR_X;
		motor_temp.switch_front = 1;
		DRV_motor_set_switch(motor_temp);
		motor_temp.switch_front = 0;
	
		EXTI_ClearITPendingBit(EXTI_Line5);//���LINE5�ϵ��жϱ�־λ 
	}
	else if(EXTI_GetITStatus(EXTI_Line6) != RESET)   //��翪��
	{		
		if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6) == 1)
		{
			motor_temp.motor_type = MOTOR_X;
			motor_temp.switch_protect = 1;
			DRV_motor_set_switch(motor_temp);
			motor_temp.switch_protect = 0;
		}
		EXTI_ClearITPendingBit(EXTI_Line6);//���LINE6�ϵ��жϱ�־λ 
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


//�жϵ�ǰ�Ŀ���״̬  ����״̬������  STATUS_FRONT  STATUS_STOP  STATUS_REAR
MOTOR_STATUS DRV_switch_judge(MOTOR *motor)
{
	MOTOR_STATUS status = STATUS_STOP;
	switch(motor->motor_type)
	{
		case MOTOR_X:
		{
			//��е���������ش����жϣ��պ�ʱ��Ϊ�ߵ�ƽ
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
			//��е���������ش����жϣ��պ�ʱ��Ϊ�ߵ�ƽ
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
			//��е���������ش����жϣ��պ�ʱ��Ϊ�ߵ�ƽ
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
			//��е���������ش����жϣ��պ�ʱ��Ϊ�ߵ�ƽ
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
			//����
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







