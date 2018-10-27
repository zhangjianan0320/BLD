#ifndef __TIMER_H
#define __TIMER_H

#include "sys.h"
#include "usart.h"

void DRV_TimerX_Init(u16 arr,u16 psc);

void DRV_TimerX_Close(void);

void DRV_TimerY_Init(u16 arr,u16 psc);

void DRV_TimerY_Close(void);

void DRV_TimerZ_Init(u16 arr,u16 psc);

void DRV_TimerZ_Close(void);

void DRV_TimerT_Init(u16 arr,u16 psc);

void DRV_TimerT_Close(void);

#endif



