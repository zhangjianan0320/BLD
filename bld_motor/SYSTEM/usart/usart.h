#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "motor.h"

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);

void SendDataToPC(u8 *str);

void USART3_Send_One_Byte(u8 byte);

void USART3_Send_Str(u8 *str);

void USART3_CMD_Deal(void);


void DRV_queue_Init(void);

void DRV_queue_push(u8 data);

u8 DRV_queue_pull(u8 *buf);

u8 DRV_queue_check(void);

void DRV_queue_delete_frame_count(void);

int DRV_read_one_nums(u8 *data);

void DRV_cmd_deal(CONTROL *control);

void DRV_usart_send_location(CONTROL *control);


#endif


