#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"

#define GPIO_MOTOR_X_DIR 			(PCout(3))
#define GPIO_MOTOR_X_PWN_OUT	(PAout(1))
#define GPIO_MOTOR_X_PWN_IN		(PAin(1))

#define GPIO_MOTOR_Y_DIR 			(PCout(2))
#define GPIO_MOTOR_Y_PWN_OUT	(PAout(7))
#define GPIO_MOTOR_Y_PWN_IN		(PAin(7))

#define GPIO_MOTOR_Z_DIR 			(PCout(1))
#define GPIO_MOTOR_Z_PWN_OUT	(PAout(2))
#define GPIO_MOTOR_Z_PWN_IN		(PAin(2))

#define GPIO_MOTOR_T_DIR 			(PCout(0))
#define GPIO_MOTOR_T_PWN_OUT	(PDout(12))
#define GPIO_MOTOR_T_PWN_IN		(PDin(12))


//6�ֹ����ٶȵȼ�
#define H_SPEED1	3.8
#define H_SPEED2	59.8
#define H_SPEED3	79.8
#define H_SPEED4	91.8
#define H_SPEED5	95.8
#define H_SPEED6	98.2

typedef enum __MOTOR_TYPE{
	
	MOTOR_X,
	MOTOR_Z,
	MOTOR_Y,
	MOTOR_T

}MOTOR_TYPE;

typedef enum __MOTOR_DIR{
	MOTOR_POSTIVE = 0,
	MOTOR_NEGATIVE = 1
}MOTOR_DIR;

typedef enum __MOTOR_STATUS{
	STATUS_FRONT,
	STATUS_GO_AHEAD,
	STATUS_RETREAT,
	STATUS_STOP,
	STATUS_REAR,
	NON
}MOTOR_STATUS;

typedef enum __MOTOR_OPERAT{
	NO_OPERAT,
	OPERAT
}MOTOR_OPERAT;

typedef struct __MOTOR{
	
	
	MOTOR_TYPE motor_type;		//������� X Y Z R��
	MOTOR_DIR motor_dir;			//������з���
	u8 acceleration_level;		//���ٶȵȼ�
	float speed;							//��������ٶ�
	u32 step;									//������в���
	MOTOR_OPERAT operat;			//��ʾ�Ƿ��е������
	MOTOR_STATUS status;			//�����ǰ״̬  0����ǰ�� 1��ǰ�� 2������ 3��ֹͣ 4����ĩ��
	u32 curren_location;			//����ĵ�ǰ����λ��
	u8 switch_front;
	u8 switch_rear;
	u8 switch_protect;
	
}MOTOR;

typedef struct __CONTROL{
	MOTOR *motor_x;
	MOTOR *motor_y;
	MOTOR *motor_z;
	MOTOR *motor_t;
}CONTROL;

CONTROL* DRV_Motor_Init(CONTROL *pControl);

void DRV_Motor_GPIO_Init(void);

void DRV_motor_set_switch(MOTOR motor);

void DRV_PWM_X(void);

void DRV_Check_X(void);

void DRV_PWM_Y(void);

void DRV_Check_Y(void);

void DRV_PWM_Z(void);

void DRV_Check_Z(void);

void DRV_PWM_T(void);

void DRV_Check_T(void);

void DRV_motor_run(MOTOR *motor);

void DRV_motor_stop(MOTOR *motor);

void DRV_Motor_Control(MOTOR *motor);

void DRV_motor_deal(CONTROL *control);

float DRV_SPEED_LEVEL(u8 level);

#endif
