#include "sys.h"
#include "usart.h"	
#include "string.h"
  
#if 0
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 


#define BUF_SIZE	100

#define FRAME_END	';'

//处理接受到的数据的结构体
typedef struct __QUEUE
{

	u16 head;															//接受数据头的位置
	u16 tail;															//接受数据未的位置
	u16 count;														//接收数据数量
	u8 buf[BUF_SIZE];											//存储接受到的数据
	u8 frame_count;												//帧数量
}QUEUE;



//bound:波特率
void uart_init(u32 bound)
{
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART1); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART1); 
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口1
	

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
	USART_Cmd(USART3, ENABLE);  //使能串口1 
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断
	
}
void SendDataToPC(u8 *str)
{
	USART3_Send_Str(str);
}
void USART3_Send_Str(u8 *str)
{
	while(*str != '\0')
		USART3_Send_One_Byte(*str++);
}

void USART3_Send_One_Byte(u8 byte)
{
	USART_SendData(USART3, byte);                                      
    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){}   
}


void USART3_IRQHandler(void)          	//串口1中断服务程序
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  
	{
		DRV_queue_push(USART_ReceiveData(USART3));	//读取接收到的数据	 
  } 
} 

QUEUE Global_Queue_usart;
//队列初始化
void DRV_queue_Init(void)
{
	
	Global_Queue_usart.head = 0;
	Global_Queue_usart.tail = 0;
	Global_Queue_usart.count = 0;
	memset(Global_Queue_usart.buf,0,sizeof(Global_Queue_usart.buf));
	Global_Queue_usart.frame_count = 0;
}
//进队列  且检测是否包含帧尾
void DRV_queue_push(u8 data)
{
	//如果buf满了以后，头指针向后移动一位
	if(Global_Queue_usart.count == BUF_SIZE)
	{
		Global_Queue_usart.head ++;
		if(Global_Queue_usart.head == BUF_SIZE)
		{
			Global_Queue_usart.head = 0;
		}
	}
	
	//如果队列满了，则head需要向后移动以为
	if(Global_Queue_usart.count != BUF_SIZE)
	{
		Global_Queue_usart.count ++;
	}
	
	Global_Queue_usart.buf[Global_Queue_usart.tail++] = data;
	//检测是否到队尾
	if(Global_Queue_usart.tail == (BUF_SIZE))
	{
		Global_Queue_usart.tail = 0;
	}
	
	
	//如果检测到帧尾，则表示收到一帧数据
	if(data == FRAME_END)
		Global_Queue_usart.frame_count++;

}
//出队列  若有数据则返回1，没有则返回0
u8 DRV_queue_pull(u8 *buf)
{
	if(Global_Queue_usart.count == 0)
		return 0;
	else
	{
		*buf = Global_Queue_usart.buf[Global_Queue_usart.head];
		Global_Queue_usart.buf[Global_Queue_usart.head] = 0;
		Global_Queue_usart.head++;
		//当队头到达数组末端时，返回数组开头位置为，循环取数
		if(Global_Queue_usart.head == BUF_SIZE)
			Global_Queue_usart.head = 0;
		Global_Queue_usart.count--;
		return 1;
	}
}
//检测帧数量
u8 DRV_queue_check(void)
{
	return Global_Queue_usart.frame_count;
}
//帧数量减一
void DRV_queue_delete_frame_count(void)
{
	if(Global_Queue_usart.frame_count != 0)
		Global_Queue_usart.frame_count--;
}

int DRV_read_one_nums(u8 *data)
{
	int i = 0;
	int temp = 0;
	u32 num;
	
	num = 0;
	
	while((data[i] != ',') && (data[i] != ';'))
	{
		i++;
	}
	temp = i;
	for(i = 0; i < temp; i++)
	{
		//如果数值不在0-9之间，则强制致为0
		if((data[i] < '0') || (data[i] > '9'))
			data[i] = '0';
		num *= 10;
		num += data[i] - '0';
		data[i] = '0';
	}
	data[i] = '0';
	return num;;
	
}

void DRV_cmd_deal(CONTROL *control)
{
	u8 deal_data[50] = {0};
	u8 data_count = 0;
	u8 offset = 0;       //用于计算处理数据的偏移位置
	u8 send[50] = "OK";
	u8 temp[20] = {0};
	u8 acces = 0;				//计算加速度的中间变量
	
	if(DRV_queue_check())
	{
		do{
				if(DRV_queue_pull(deal_data + data_count) == 0)
				{
					//没有数据时，直接舍弃退出
					DRV_queue_delete_frame_count();
						return;
				}
			}while(deal_data[data_count++] != FRAME_END);					//当检测到帧头时，退出循环
		
		DRV_queue_delete_frame_count();											//减少帧数量
			
		switch(deal_data[0])
		{
			case 'T':				//测试通信是否ok
			{
				if(strcmp((const char*)deal_data,"T?;") == 0)
					SendDataToPC(send);
			}
				break;
			case 'D':				//运行参数设置
			{
				SendDataToPC(send);
				switch(deal_data[1])
				{
					case 'X':
					{
						offset = 3;
						control->motor_x->speed = DRV_read_one_nums(deal_data + offset);
						control->motor_x->motor_dir= (MOTOR_DIR)DRV_read_one_nums(deal_data + offset);
						control->motor_x->step = DRV_read_one_nums(deal_data + offset);
					}
						break;
					case 'Y':
					{
						offset = 3;
						control->motor_y->speed = DRV_read_one_nums(deal_data + offset);
						control->motor_y->motor_dir= (MOTOR_DIR)DRV_read_one_nums(deal_data + offset);
						control->motor_y->step = DRV_read_one_nums(deal_data + offset);
					}
						break;
					case 'Z':
					{
						offset = 3;
						control->motor_z->speed = DRV_read_one_nums(deal_data + offset);
						control->motor_z->motor_dir= (MOTOR_DIR)DRV_read_one_nums(deal_data + offset);
						control->motor_z->step = DRV_read_one_nums(deal_data + offset);
					}
						break;
					case 'T':
						offset = 3;
						control->motor_t->speed = DRV_read_one_nums(deal_data + offset);
						control->motor_t->motor_dir= (MOTOR_DIR)DRV_read_one_nums(deal_data + offset);
						control->motor_t->step = DRV_read_one_nums(deal_data + offset);
						break;
				}
			}
				break;
			case 'A':
			{
				SendDataToPC(send);
				switch(deal_data[3])
				{
					case 'L':
						acces = 4;
					break;
					case 'M':
						acces = 3;
					break;
					case 'H':
						acces = 2;
					break;
					case 'G':
						acces = 1;
					break;
					default :
						acces = 1;
					break;
				}
				
				switch(deal_data[1])
				{
					case 'X':
					{
						control->motor_x->acceleration_level = acces;
					}
						break;
					case 'Y':
					{
						control->motor_y->acceleration_level = acces;
					}
						break;
					case 'Z':
					{
						control->motor_z->acceleration_level = acces;
					}
						break;
					case 'T':
					{
						control->motor_t->acceleration_level = acces;
					}
						break;
				}
			}
				break;
			case 'G':				//运行
			{
				SendDataToPC(send);
				offset = 1;
				while(deal_data[offset] != ';')
				{
					switch(deal_data[offset])
					{
						case 'X':
							control->motor_x->operat = OPERAT;
							break;
						case 'Y':
							control->motor_y->operat = OPERAT;
							break;
						case 'Z':
							control->motor_z->operat = OPERAT;
							break;
						case 'T':
							control->motor_t->operat = OPERAT;
							break;
						case 'O':
							control->motor_x->operat = OPERAT;
							control->motor_y->operat = OPERAT;
							control->motor_z->operat = OPERAT;
							control->motor_t->operat = OPERAT;
							break;
					}
					offset++;
				}
			}
				break;
			case 'P':				//暂停
			{
				SendDataToPC(send);
				offset = 1;
				while(deal_data[offset] != ';')
				{
					switch(deal_data[offset])
					{
						case 'X':
							control->motor_x->speed = 0;
							control->motor_x->operat = OPERAT;
							break;
						case 'Y':
							control->motor_y->speed = 0;
							control->motor_y->operat = OPERAT;
							break;
						case 'Z':
							control->motor_z->speed = 0;
							control->motor_z->operat = OPERAT;
							break;
						case 'T':
							control->motor_t->speed = 0;
							control->motor_t->operat = OPERAT;
							break;
						case 'A':
							control->motor_x->speed = 0;
							control->motor_y->speed = 0;
							control->motor_z->speed = 0;
							control->motor_t->speed = 0;
							control->motor_x->operat = OPERAT;
							control->motor_y->operat = OPERAT;
							control->motor_z->operat = OPERAT;
							control->motor_t->operat = OPERAT;
							break;
					}
					offset++;
				}
			}
				break;
			case 'H':				//归零指令
			{
				SendDataToPC(send);
					switch(deal_data[1])
					{
						case 'X':
							control->motor_x->speed = DRV_SPEED_LEVEL(DRV_read_one_nums(deal_data + 3));
							control->motor_x->motor_dir = MOTOR_NEGATIVE;
							control->motor_x->operat = OPERAT;
							break;
						case 'Y':
							control->motor_y->speed = DRV_SPEED_LEVEL(DRV_read_one_nums(deal_data + 3));
							control->motor_y->motor_dir = MOTOR_NEGATIVE;
							control->motor_y->operat = OPERAT;
							break;
						case 'Z':
							control->motor_z->speed = DRV_SPEED_LEVEL(DRV_read_one_nums(deal_data + 3));
							control->motor_z->motor_dir = MOTOR_NEGATIVE;
							control->motor_z->operat = OPERAT;
							break;
						case 'R':
							control->motor_t->speed = DRV_SPEED_LEVEL(DRV_read_one_nums(deal_data + 3));
							control->motor_t->motor_dir = MOTOR_NEGATIVE;
							control->motor_t->operat = OPERAT;
							break;
						case 'A':
							control->motor_x->speed = DRV_SPEED_LEVEL(DRV_read_one_nums(deal_data + 3));
							control->motor_y->speed = control->motor_x->speed;
							control->motor_z->speed = control->motor_x->speed;
							control->motor_t->speed = control->motor_x->speed;
							control->motor_x->motor_dir = MOTOR_NEGATIVE;
							control->motor_y->motor_dir = MOTOR_NEGATIVE;
							control->motor_z->motor_dir = MOTOR_NEGATIVE;
							control->motor_t->motor_dir = MOTOR_NEGATIVE;
							control->motor_x->operat = OPERAT;
							control->motor_y->operat = OPERAT;
							control->motor_z->operat = OPERAT;
							control->motor_t->operat = OPERAT;
							break;
					}			
			}
				break;
			case '?':				//查询指令
			{

				switch(deal_data[1])
				{
					case 'X':
						sprintf((char *)send,"X:%d",control->motor_x->curren_location);
						SendDataToPC((u8 *)send);
						break;
					case 'Y':
						sprintf((char *)send,"Y:%d",control->motor_y->curren_location);
						SendDataToPC(send);
						break;
					case 'Z':
						sprintf((char *)send,"Z:%d",control->motor_z->curren_location);
						SendDataToPC(send);
						break;
					case 'T':
						sprintf((char *)send,"T:%d",control->motor_t->curren_location);
						SendDataToPC(send);
						break;
					case 'A':
						sprintf((char *)send,"1:000000  2:000000  3:000000  4:000000  ");

						sprintf((char *)temp,"%d ",control->motor_x->curren_location);
						memmove(send+9-strlen((const char *)temp),temp,strlen((const char *)temp));

						sprintf((char *)temp,"%d ",control->motor_y->curren_location);
						memmove(send+19-strlen((const char *)temp),temp,strlen((const char *)temp));

						sprintf((char *)temp,"%d ",control->motor_z->curren_location);
						memmove(send+29-strlen((const char *)temp),temp,strlen((const char *)temp));

						sprintf((char *)temp,"%d ",control->motor_t->curren_location);
						memmove(send+39-strlen((const char *)temp),temp,strlen((const char *)temp));
					
						SendDataToPC((u8 *)send);
					break;
					default:
						break;
				}
			}
				break;
		}
	}
}

void DRV_usart_send_location(CONTROL *control)
{
	u8 send[50] = "OK;";
	u8 temp[10] = {0};
	
	sprintf((char *)send,"1:000000  2:000000  3:000000  4:000000  ");

	sprintf((char *)temp,"%d ",control->motor_x->curren_location);
	memmove(send+9-strlen((const char *)temp),temp,strlen((const char *)temp));

	sprintf((char *)temp,"%d ",control->motor_y->curren_location);
	memmove(send+19-strlen((const char *)temp),temp,strlen((const char *)temp));

	sprintf((char *)temp,"%d ",control->motor_z->curren_location);
	memmove(send+29-strlen((const char *)temp),temp,strlen((const char *)temp));

	sprintf((char *)temp,"%d ",control->motor_t->curren_location);
	memmove(send+39-strlen((const char *)temp),temp,strlen((const char *)temp));

	SendDataToPC((u8 *)send);
	
	return ;
}
