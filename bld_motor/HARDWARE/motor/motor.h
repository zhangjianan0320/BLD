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


//6种归零速度等级
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
	
	
	MOTOR_TYPE motor_type;		//电机类型 X Y Z R等
	MOTOR_DIR motor_dir;			//电机运行方向
	u8 acceleration_level;		//加速度等级
	float speed;							//电机运行速度
	u32 step;									//电机运行步数
	MOTOR_OPERAT operat;			//表示是否有电机操作
	MOTOR_STATUS status;			//电机当前状态  0：最前端 1：前进 2：后退 3：停止 4：最末端
	u32 curren_location;			//电机的当前绝对位置
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
