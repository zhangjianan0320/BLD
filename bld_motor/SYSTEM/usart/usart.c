#include "sys.h"
#include "usart.h"	
#include "string.h"
  
#if 0
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART3->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 


#define BUF_SIZE	100

#define FRAME_END	';'

//������ܵ������ݵĽṹ��
typedef struct __QUEUE
{

	u16 head;															//��������ͷ��λ��
	u16 tail;															//��������δ��λ��
	u16 count;														//������������
	u8 buf[BUF_SIZE];											//�洢���ܵ�������
	u8 frame_count;												//֡����
}QUEUE;



//bound:������
void uart_init(u32 bound)
{
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART1); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART1); 
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure);

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure); //��ʼ������1
	

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	
	USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���1 
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//��������ж�
	
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


void USART3_IRQHandler(void)          	//����1�жϷ������
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  
	{
		DRV_queue_push(USART_ReceiveData(USART3));	//��ȡ���յ�������	 
  } 
} 

QUEUE Global_Queue_usart;
//���г�ʼ��
void DRV_queue_Init(void)
{
	
	Global_Queue_usart.head = 0;
	Global_Queue_usart.tail = 0;
	Global_Queue_usart.count = 0;
	memset(Global_Queue_usart.buf,0,sizeof(Global_Queue_usart.buf));
	Global_Queue_usart.frame_count = 0;
}
//������  �Ҽ���Ƿ����֡β
void DRV_queue_push(u8 data)
{
	//���buf�����Ժ�ͷָ������ƶ�һλ
	if(Global_Queue_usart.count == BUF_SIZE)
	{
		Global_Queue_usart.head ++;
		if(Global_Queue_usart.head == BUF_SIZE)
		{
			Global_Queue_usart.head = 0;
		}
	}
	
	//����������ˣ���head��Ҫ����ƶ���Ϊ
	if(Global_Queue_usart.count != BUF_SIZE)
	{
		Global_Queue_usart.count ++;
	}
	
	Global_Queue_usart.buf[Global_Queue_usart.tail++] = data;
	//����Ƿ񵽶�β
	if(Global_Queue_usart.tail == (BUF_SIZE))
	{
		Global_Queue_usart.tail = 0;
	}
	
	
	//�����⵽֡β�����ʾ�յ�һ֡����
	if(data == FRAME_END)
		Global_Queue_usart.frame_count++;

}
//������  ���������򷵻�1��û���򷵻�0
u8 DRV_queue_pull(u8 *buf)
{
	if(Global_Queue_usart.count == 0)
		return 0;
	else
	{
		*buf = Global_Queue_usart.buf[Global_Queue_usart.head];
		Global_Queue_usart.buf[Global_Queue_usart.head] = 0;
		Global_Queue_usart.head++;
		//����ͷ��������ĩ��ʱ���������鿪ͷλ��Ϊ��ѭ��ȡ��
		if(Global_Queue_usart.head == BUF_SIZE)
			Global_Queue_usart.head = 0;
		Global_Queue_usart.count--;
		return 1;
	}
}
//���֡����
u8 DRV_queue_check(void)
{
	return Global_Queue_usart.frame_count;
}
//֡������һ
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
		//�����ֵ����0-9֮�䣬��ǿ����Ϊ0
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
	u8 offset = 0;       //���ڼ��㴦�����ݵ�ƫ��λ��
	u8 send[50] = "OK";
	u8 temp[20] = {0};
	u8 acces = 0;				//������ٶȵ��м����
	
	if(DRV_queue_check())
	{
		do{
				if(DRV_queue_pull(deal_data + data_count) == 0)
				{
					//û������ʱ��ֱ�������˳�
					DRV_queue_delete_frame_count();
						return;
				}
			}while(deal_data[data_count++] != FRAME_END);					//����⵽֡ͷʱ���˳�ѭ��
		
		DRV_queue_delete_frame_count();											//����֡����
			
		switch(deal_data[0])
		{
			case 'T':				//����ͨ���Ƿ�ok
			{
				if(strcmp((const char*)deal_data,"T?;") == 0)
					SendDataToPC(send);
			}
				break;
			case 'D':				//���в�������
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
			case 'G':				//����
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
			case 'P':				//��ͣ
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
			case 'H':				//����ָ��
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
			case '?':				//��ѯָ��
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
