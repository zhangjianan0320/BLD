#include "motor.h"
#include "timer.h"
#include "switch.h"
#include "string.h"

static CONTROL control;

static MOTOR motor_control_x;
static MOTOR motor_control_y;
static MOTOR motor_control_z;
static MOTOR motor_control_t;

CONTROL* DRV_Motor_Init(CONTROL *pControl)
{
	if(pControl == 0)
	{
		pControl = &control;
	}
	
	pControl->motor_x = &motor_control_x;
	pControl->motor_y = &motor_control_y;
	pControl->motor_z = &motor_control_z;
	pControl->motor_t = &motor_control_t;
	
	pControl->motor_x->motor_dir = MOTOR_POSTIVE;
	pControl->motor_x->motor_type = MOTOR_X;
	pControl->motor_x->acceleration_level = 1;
	pControl->motor_x->speed = 0;
	pControl->motor_x->step = 0;
	pControl->motor_x->curren_location = 0;
	pControl->motor_x->operat = NO_OPERAT;
	pControl->motor_x->switch_front = 0;
	pControl->motor_x->switch_rear = 0;
	pControl->motor_x->switch_rear = 0;
	
	pControl->motor_y->motor_dir = MOTOR_POSTIVE;
	pControl->motor_y->motor_type = MOTOR_Y;
	pControl->motor_y->acceleration_level = 1;
	pControl->motor_y->speed = 0;
	pControl->motor_y->step = 0;
	pControl->motor_y->curren_location = 0;
	pControl->motor_y->operat = NO_OPERAT;
	pControl->motor_y->switch_front = 0;
	pControl->motor_y->switch_rear = 0;
	pControl->motor_y->switch_rear = 0;
	
	pControl->motor_z->motor_dir = MOTOR_POSTIVE;
	pControl->motor_z->motor_type = MOTOR_Z;
	pControl->motor_z->acceleration_level = 1;
	pControl->motor_z->speed = 0;
	pControl->motor_z->step = 0;
	pControl->motor_z->curren_location = 0;
	pControl->motor_z->operat = NO_OPERAT;
	pControl->motor_z->switch_front = 0;
	pControl->motor_z->switch_rear = 0;
	pControl->motor_z->switch_rear = 0;
	
	pControl->motor_t->motor_dir = MOTOR_POSTIVE;
	pControl->motor_t->motor_type = MOTOR_T;
	pControl->motor_t->acceleration_level = 1;
	pControl->motor_t->speed = 0;
	pControl->motor_t->step = 0;
	pControl->motor_t->curren_location = 0;
	pControl->motor_t->operat = NO_OPERAT;
	pControl->motor_t->switch_front = 0;
	pControl->motor_t->switch_rear = 0;
	pControl->motor_t->switch_rear = 0;
	
	return &control;
}

void DRV_Motor_GPIO_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 |GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_SetBits(GPIOC,GPIO_Pin_3);		//x�� dir
	GPIO_SetBits(GPIOC,GPIO_Pin_2);		//y�� dir
	GPIO_SetBits(GPIOC,GPIO_Pin_1);		//z�� dir
	GPIO_SetBits(GPIOC,GPIO_Pin_0);		//t�� dir
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA,GPIO_Pin_1);		//x�� pwm����
	GPIO_SetBits(GPIOA,GPIO_Pin_7);		//y�� pwm����	
	GPIO_SetBits(GPIOA,GPIO_Pin_2);		//z�� pwm����
	GPIO_SetBits(GPIOD,GPIO_Pin_12);	//t�� pwm����
	
	
	
	GPIO_MOTOR_Y_PWN_OUT = 0;
	GPIO_MOTOR_Y_PWN_OUT = 1;
	GPIO_MOTOR_Y_PWN_OUT = 0;
	
	
	GPIO_MOTOR_Z_PWN_OUT = 0;
	GPIO_MOTOR_Z_PWN_OUT = 1;
	GPIO_MOTOR_Z_PWN_OUT = 0;
	
}

//�жϻص�����  switch type
void DRV_motor_set_switch(MOTOR motor)
{
	switch(motor.motor_type)
	{
		case MOTOR_X:
		{
			if(motor.switch_front == 1)
			{
				motor_control_x.switch_front = 1;
			}
			else if(motor.switch_rear == 1)
			{
				motor_control_x.switch_rear = 1;
			}
			else if(motor.switch_protect == 1)
			{
				motor_control_x.switch_protect = 1;
			}
			else
			{
				motor_control_x.switch_protect = 1;
			}
		}
			break;
		case MOTOR_Y:
		{
			if(motor.switch_front == 1)
			{
				motor_control_y.switch_front = 1;
			}
			else if(motor.switch_rear == 1)
			{
				motor_control_y.switch_rear = 1;
			}
			else if(motor.switch_protect == 1)
			{
				motor_control_y.switch_protect = 1;
			}
			else
			{
				motor_control_y.switch_protect = 1;
			}
		}
			break;
		case MOTOR_Z:
		{
			if(motor.switch_front == 1)
			{
				motor_control_z.switch_front = 1;
			}
			else if(motor.switch_rear == 1)
			{
				motor_control_z.switch_rear = 1;
			}
			else if(motor.switch_protect == 1)
			{
				motor_control_z.switch_protect = 1;
			}
			else
			{
				motor_control_z.switch_protect = 1;
			}
		}
			break;
		case MOTOR_T:
			{
			if(motor.switch_front == 1)
			{
				motor_control_t.switch_front = 1;
			}
			else if(motor.switch_rear == 1)
			{
				motor_control_t.switch_rear = 1;
			}
			else if(motor.switch_protect == 1)
			{
				motor_control_t.switch_protect = 1;
			}
			else
			{
				motor_control_t.switch_protect = 1;
			}
		}
			break;
		default:
			break;
	}
}

//���X����pwm�ľ��岽��
void DRV_PWM_X(void)
{
	if(GPIO_MOTOR_X_PWN_IN == 0)
	{
		GPIO_MOTOR_X_PWN_OUT = 1;
	}
	else
	{
		GPIO_MOTOR_X_PWN_OUT = 0;
	}
}

//���X����pwm�ľ��岽��
void DRV_PWM_Y(void)
{
	if(GPIO_MOTOR_Y_PWN_IN == 0)
	{
		GPIO_MOTOR_Y_PWN_OUT = 1;
	}
	else
	{
		GPIO_MOTOR_Y_PWN_OUT = 0;
	}
}

//���X����pwm�ľ��岽��
void DRV_PWM_Z(void)
{
	if(GPIO_MOTOR_Z_PWN_IN == 0)
	{
		GPIO_MOTOR_Z_PWN_OUT = 1;
	}
	else
	{
		GPIO_MOTOR_Z_PWN_OUT = 0;
	}
}

//���X����pwm�ľ��岽��
void DRV_PWM_T(void)
{
	if(GPIO_MOTOR_T_PWN_IN == 0)
	{
		GPIO_MOTOR_T_PWN_OUT = 1;
	}
	else
	{
		GPIO_MOTOR_T_PWN_OUT = 0;
	}
}

//���X�˶�������У���Ƿ��˵���ָ������
void DRV_Check_X(void)
{
	//������ʵ����
	if(GPIO_MOTOR_X_PWN_IN == 0)
	{
		//����ʵ�ʵ�λ��
		if(motor_control_x.motor_dir == MOTOR_POSTIVE)
		{
			motor_control_x.curren_location++;	
		}
		else
		{
			if(motor_control_x.curren_location >= 1)
				motor_control_x.curren_location--;
		}
		
		//������Ϊ0�����ʾ���޲���
		if(motor_control_x.step != 0)
		{
			//ÿ������ֻ����һ��
			--motor_control_x.step;
			if(motor_control_x.step == 0)
			{
				
				DRV_TimerX_Close();
			}
		}
	}
}

//���X�˶�������У���Ƿ��˵���ָ������
void DRV_Check_Y(void)
{
	//������ʵ����
	if(GPIO_MOTOR_Y_PWN_IN == 0)
	{
		//����ʵ�ʵ�λ��
		if(motor_control_y.motor_dir == MOTOR_POSTIVE)
		{
			motor_control_y.curren_location++;	
		}
		else
		{
			if(motor_control_y.curren_location >= 1)
				motor_control_y.curren_location--;
		}
		
		//������Ϊ0�����ʾ���޲���
		if(motor_control_y.step != 0)
		{
			//ÿ������ֻ����һ��
			--motor_control_y.step;
			if(motor_control_y.step == 0)
			{
				DRV_TimerY_Close();
			}
		}
	}
}

//���X�˶�������У���Ƿ��˵���ָ������
void DRV_Check_Z(void)
{
	//������ʵ����
	if(GPIO_MOTOR_Z_PWN_IN == 0)
	{
		//����ʵ�ʵ�λ��
		if(motor_control_z.motor_dir == MOTOR_POSTIVE)
		{
			motor_control_z.curren_location++;	
		}
		else
		{
			if(motor_control_z.curren_location >= 1)
				motor_control_z.curren_location--;
		}
		
		//������Ϊ0�����ʾ���޲���
		if(motor_control_z.step != 0)
		{
			//ÿ������ֻ����һ��
			--motor_control_z.step;
			if(motor_control_z.step == 0)
			{
				DRV_TimerZ_Close();
			}
		}
	}
}

//���X�˶�������У���Ƿ��˵���ָ������
void DRV_Check_T(void)
{
	//������ʵ����
	if(GPIO_MOTOR_T_PWN_IN == 0)
	{
		//����ʵ�ʵ�λ��
		if(motor_control_t.motor_dir == MOTOR_POSTIVE)
		{
			motor_control_t.curren_location++;	
		}
		else
		{
			if(motor_control_t.curren_location >= 1)
				motor_control_t.curren_location--;
		}
		
		//������Ϊ0�����ʾ���޲���
		if(motor_control_t.step != 0)
		{
			//ÿ������ֻ����һ��
			--motor_control_t.step;
			if(motor_control_t.step == 0)
			{
				DRV_TimerT_Close();
			}
		}
	}
}
void DRV_motor_run(MOTOR *motor)
{
	u16 time_arr = 0;
	u16 time_psc = 0;


	time_psc = 840;
	time_arr = (2495 - motor->speed * 25) * 840 / time_psc / 19 - 1;

	time_psc--;
	
	if(motor->motor_type == MOTOR_X)
	{
		GPIO_MOTOR_X_DIR = motor->motor_dir;
		//�ٶ�  ���ݼ��ٶȽ��м�������
		DRV_TimerX_Init(time_arr ,time_psc);
	}
	else if(motor->motor_type == MOTOR_Y)
	{
		GPIO_MOTOR_Y_DIR = motor->motor_dir;
		//�ٶ�
		DRV_TimerY_Init(time_arr ,time_psc);
	}
	else if(motor->motor_type == MOTOR_Z)
	{
		GPIO_MOTOR_Z_DIR = motor->motor_dir;
		//�ٶ�
		DRV_TimerZ_Init(time_arr ,time_psc);
	}
	else if(motor->motor_type == MOTOR_T)
	{
		GPIO_MOTOR_T_DIR = motor->motor_dir;
		//�ٶ�
		DRV_TimerT_Init(time_arr ,time_psc);
	}
}
void DRV_motor_stop(MOTOR *motor)
{
	if(motor->motor_type == MOTOR_X)
	{
		DRV_TimerX_Close();
	}
	else if(motor->motor_type == MOTOR_Y)
	{
		DRV_TimerY_Close();
	}
	else if(motor->motor_type == MOTOR_Z)
	{
		DRV_TimerZ_Close();
	}
	else if(motor->motor_type == MOTOR_T)
	{
		DRV_TimerT_Close();
	}
	else
	{}
}

//���Ƶ���������ٶȣ����в����������
void DRV_Motor_Control(MOTOR *motor)
{
	MOTOR_STATUS status;
	MOTOR m_temp;
	
	memcpy(&m_temp,motor,sizeof(m_temp));
	
	status = DRV_switch_judge(motor);
	
	//motor->acceleration_level = 10;
	if(motor->speed != 0)
	{
		//for(i = 0; i < motor->acceleration_level; i++)
		{
			//m_temp.speed = motor->speed / (float)(motor->acceleration_level - i);
			//�����½���ʱ�����һ�£��Ƿ��ܼ�������
			if(((m_temp.motor_dir == MOTOR_POSTIVE) && (status != STATUS_REAR)) || ((m_temp.motor_dir == MOTOR_NEGATIVE) && (status != STATUS_FRONT)))
			{
				m_temp.status = STATUS_GO_AHEAD;
				
				DRV_motor_run(motor);
			//	delay_ms(10);
			}
			else
				return ;
		}
	}
	else //����ٶ�Ϊ0�����ʾֹͣ
	{
		DRV_motor_stop(motor);
	}
}




void DRV_motor_deal(CONTROL *control)
{
	if(control->motor_x->operat == OPERAT)
	{
		control->motor_x->operat = NO_OPERAT;
		DRV_Motor_Control(control->motor_x);
	}
	
	if(control->motor_y->operat == OPERAT)
	{
		control->motor_y->operat = NO_OPERAT;
		DRV_Motor_Control(control->motor_y);
	}
	
	if(control->motor_z->operat == OPERAT)
	{
		control->motor_z->operat = NO_OPERAT;
		DRV_Motor_Control(control->motor_z);
	}
	
	if(control->motor_t->operat == OPERAT)
	{
		control->motor_t->operat = NO_OPERAT;
		DRV_Motor_Control(control->motor_t);
	}
	
}
//���ݹ����ٶȵȼ������ٶ�
float DRV_SPEED_LEVEL(u8 level)
{
	float speed = 0;
	switch(level)
	{
		case 1:
			speed = H_SPEED1;
			break;
		case 2:
			speed = H_SPEED2;
			break;
		case 3:
			speed = H_SPEED3;
			break;
		case 4:
			speed = H_SPEED4;
			break;
		case 5:
			speed = H_SPEED5;
			break;
		case 6:
			speed = H_SPEED6;
			break;
			
	}
	
	return speed;
}

