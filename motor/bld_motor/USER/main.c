#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "motor.h"
#include "timer.h"
#include "switch.h"


/*测试配置时钟是否正确*/
void MCO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_MCO); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	RCC_MCO1Config(RCC_MCO1Source_PLLCLK,RCC_MCO1Div_4);
}

int main(void)
{ 
	
	CONTROL *control;
	
	delay_init(168);		  //初始化延时函数
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	uart_init(115200);
	
	control = DRV_Motor_Init((CONTROL *)0);
	DRV_queue_Init();
	
	DRV_Motor_GPIO_Init();
	DRV_switch_Init();
	
	while(1)
	{
		DRV_cmd_deal(control);
		DRV_motor_deal(control);
		DRV_switch_deal(control);
	}
}

