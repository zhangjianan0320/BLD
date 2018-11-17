/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId SD_TaskHandle;
osThreadId ETH_TaskHandle;
osThreadId USB_TaskHandle;
osThreadId UART_TaskHandle;
osThreadId LCD_TaskHandle;
osThreadId Watchdog_TaskHandle;
osThreadId IOExternal_TaskHandle;
osThreadId CAN_TaskHandle;
osThreadId Debug_TaskHandle;
osThreadId Flash_TaskHandle;
osThreadId AD_TaskHandle;
osThreadId ADC_TaskHandle;
osThreadId DAC_TaskHandle;
osThreadId LCD_TTL_TaskHandle;
osMessageQId LCD_QueueHandle;
osMessageQId FLASH_QueueHandle;
osMutexId SDRAM_MutexHandle;
osMutexId FLASH_MutexHandle;
osSemaphoreId LED_SemHandle;
osSemaphoreId ETH_SemHandle;
osSemaphoreId SD_SemHandle;
osSemaphoreId Uart_SemHandle;
osSemaphoreId LCD_SemHandle;
osSemaphoreId CAN_SemHandle;
osSemaphoreId AD1_TaskHandle;
osSemaphoreId AD2_TaskHandle;
osSemaphoreId AD3_TaskHandle;
osSemaphoreId ADC1_SemHandle;
osSemaphoreId ADC2_SemHandle;
osSemaphoreId DAC_SemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void SDWR_Task(void const * argument);
void COMETH_Task(void const * argument);
void COMUSB_Task(void const * argument);
void UART_Task_Func(void const * argument);
void LCDDispaly_Task(void const * argument);
void LED_Watch_Task(void const * argument);
void IOCON_Task_Func(void const * argument);
void CAN_Task_Func(void const * argument);
void Debug_Task_Func(void const * argument);
void Flash_Task_Func(void const * argument);
void AD_Task_Func(void const * argument);
void ADC_Inside_Task(void const * argument);
void DAC_OUT_Task(void const * argument);
void LCD_UART_Task(void const * argument);

extern void MX_FATFS_Init(void);
extern void MX_LIBJPEG_Init(void);
extern void MX_LWIP_Init(void);
extern void MX_MBEDTLS_Init(void);
extern void MX_PDM2PCM_Init(void);
extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of SDRAM_Mutex */
  osMutexDef(SDRAM_Mutex);
  SDRAM_MutexHandle = osMutexCreate(osMutex(SDRAM_Mutex));

  /* definition and creation of FLASH_Mutex */
  osMutexDef(FLASH_Mutex);
  FLASH_MutexHandle = osMutexCreate(osMutex(FLASH_Mutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of LED_Sem */
  osSemaphoreDef(LED_Sem);
  LED_SemHandle = osSemaphoreCreate(osSemaphore(LED_Sem), 1);

  /* definition and creation of ETH_Sem */
  osSemaphoreDef(ETH_Sem);
  ETH_SemHandle = osSemaphoreCreate(osSemaphore(ETH_Sem), 1);

  /* definition and creation of SD_Sem */
  osSemaphoreDef(SD_Sem);
  SD_SemHandle = osSemaphoreCreate(osSemaphore(SD_Sem), 1);

  /* definition and creation of Uart_Sem */
  osSemaphoreDef(Uart_Sem);
  Uart_SemHandle = osSemaphoreCreate(osSemaphore(Uart_Sem), 1);

  /* definition and creation of LCD_Sem */
  osSemaphoreDef(LCD_Sem);
  LCD_SemHandle = osSemaphoreCreate(osSemaphore(LCD_Sem), 1);

  /* definition and creation of CAN_Sem */
  osSemaphoreDef(CAN_Sem);
  CAN_SemHandle = osSemaphoreCreate(osSemaphore(CAN_Sem), 1);

  /* definition and creation of AD1_Task */
  osSemaphoreDef(AD1_Task);
  AD1_TaskHandle = osSemaphoreCreate(osSemaphore(AD1_Task), 1);

  /* definition and creation of AD2_Task */
  osSemaphoreDef(AD2_Task);
  AD2_TaskHandle = osSemaphoreCreate(osSemaphore(AD2_Task), 1);

  /* definition and creation of AD3_Task */
  osSemaphoreDef(AD3_Task);
  AD3_TaskHandle = osSemaphoreCreate(osSemaphore(AD3_Task), 1);

  /* definition and creation of ADC1_Sem */
  osSemaphoreDef(ADC1_Sem);
  ADC1_SemHandle = osSemaphoreCreate(osSemaphore(ADC1_Sem), 1);

  /* definition and creation of ADC2_Sem */
  osSemaphoreDef(ADC2_Sem);
  ADC2_SemHandle = osSemaphoreCreate(osSemaphore(ADC2_Sem), 1);

  /* definition and creation of DAC_Sem */
  osSemaphoreDef(DAC_Sem);
  DAC_SemHandle = osSemaphoreCreate(osSemaphore(DAC_Sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of SD_Task */
  osThreadDef(SD_Task, SDWR_Task, osPriorityNormal, 0, 1024);
  SD_TaskHandle = osThreadCreate(osThread(SD_Task), NULL);

  /* definition and creation of ETH_Task */
  osThreadDef(ETH_Task, COMETH_Task, osPriorityBelowNormal, 0, 1024);
  ETH_TaskHandle = osThreadCreate(osThread(ETH_Task), NULL);

  /* definition and creation of USB_Task */
  osThreadDef(USB_Task, COMUSB_Task, osPriorityNormal, 0, 1024);
  USB_TaskHandle = osThreadCreate(osThread(USB_Task), NULL);

  /* definition and creation of UART_Task */
  osThreadDef(UART_Task, UART_Task_Func, osPriorityAboveNormal, 0, 1024);
  UART_TaskHandle = osThreadCreate(osThread(UART_Task), NULL);

  /* definition and creation of LCD_Task */
  osThreadDef(LCD_Task, LCDDispaly_Task, osPriorityAboveNormal, 0, 2048);
  LCD_TaskHandle = osThreadCreate(osThread(LCD_Task), NULL);

  /* definition and creation of Watchdog_Task */
  osThreadDef(Watchdog_Task, LED_Watch_Task, osPriorityLow, 0, 128);
  Watchdog_TaskHandle = osThreadCreate(osThread(Watchdog_Task), NULL);

  /* definition and creation of IOExternal_Task */
  osThreadDef(IOExternal_Task, IOCON_Task_Func, osPriorityNormal, 0, 256);
  IOExternal_TaskHandle = osThreadCreate(osThread(IOExternal_Task), NULL);

  /* definition and creation of CAN_Task */
  osThreadDef(CAN_Task, CAN_Task_Func, osPriorityNormal, 0, 512);
  CAN_TaskHandle = osThreadCreate(osThread(CAN_Task), NULL);

  /* definition and creation of Debug_Task */
  osThreadDef(Debug_Task, Debug_Task_Func, osPriorityNormal, 0, 256);
  Debug_TaskHandle = osThreadCreate(osThread(Debug_Task), NULL);

  /* definition and creation of Flash_Task */
  osThreadDef(Flash_Task, Flash_Task_Func, osPriorityAboveNormal, 0, 2048);
  Flash_TaskHandle = osThreadCreate(osThread(Flash_Task), NULL);

  /* definition and creation of AD_Task */
  osThreadDef(AD_Task, AD_Task_Func, osPriorityHigh, 0, 4096);
  AD_TaskHandle = osThreadCreate(osThread(AD_Task), NULL);

  /* definition and creation of ADC_Task */
  osThreadDef(ADC_Task, ADC_Inside_Task, osPriorityAboveNormal, 0, 2048);
  ADC_TaskHandle = osThreadCreate(osThread(ADC_Task), NULL);

  /* definition and creation of DAC_Task */
  osThreadDef(DAC_Task, DAC_OUT_Task, osPriorityAboveNormal, 0, 1024);
  DAC_TaskHandle = osThreadCreate(osThread(DAC_Task), NULL);

  /* definition and creation of LCD_TTL_Task */
  osThreadDef(LCD_TTL_Task, LCD_UART_Task, osPriorityIdle, 0, 512);
  LCD_TTL_TaskHandle = osThreadCreate(osThread(LCD_TTL_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of LCD_Queue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(LCD_Queue, 1024, uint16_t);
  LCD_QueueHandle = osMessageCreate(osMessageQ(LCD_Queue), NULL);

  /* definition and creation of FLASH_Queue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(FLASH_Queue, 1024, uint16_t);
  FLASH_QueueHandle = osMessageCreate(osMessageQ(FLASH_Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* init code for LIBJPEG */
  MX_LIBJPEG_Init();

  /* MX_LWIP_Init() is generated within mbedtls_net_init() function in net_cockets.c file */
  /* Up to user to call mbedtls_net_init() function in MBEDTLS initialization step */

  /* Up to user define the empty MX_MBEDTLS_Init() function located in mbedtls.c file */
  MX_MBEDTLS_Init();

  /* init code for PDM2PCM */
  MX_PDM2PCM_Init();

  /* init code for USB_HOST */
  MX_USB_HOST_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_SDWR_Task */
/**
* @brief Function implementing the SD_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SDWR_Task */
void SDWR_Task(void const * argument)
{
  /* USER CODE BEGIN SDWR_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SDWR_Task */
}

/* USER CODE BEGIN Header_COMETH_Task */
/**
* @brief Function implementing the ETH_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_COMETH_Task */
void COMETH_Task(void const * argument)
{
  /* USER CODE BEGIN COMETH_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END COMETH_Task */
}

/* USER CODE BEGIN Header_COMUSB_Task */
/**
* @brief Function implementing the USB_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_COMUSB_Task */
void COMUSB_Task(void const * argument)
{
  /* USER CODE BEGIN COMUSB_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END COMUSB_Task */
}

/* USER CODE BEGIN Header_UART_Task_Func */
/**
* @brief Function implementing the UART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_Task_Func */
void UART_Task_Func(void const * argument)
{
  /* USER CODE BEGIN UART_Task_Func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UART_Task_Func */
}

/* USER CODE BEGIN Header_LCDDispaly_Task */
/**
* @brief Function implementing the LCD_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LCDDispaly_Task */
void LCDDispaly_Task(void const * argument)
{
  /* USER CODE BEGIN LCDDispaly_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LCDDispaly_Task */
}

/* USER CODE BEGIN Header_LED_Watch_Task */
/**
* @brief Function implementing the Watchdog_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED_Watch_Task */
void LED_Watch_Task(void const * argument)
{
  /* USER CODE BEGIN LED_Watch_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LED_Watch_Task */
}

/* USER CODE BEGIN Header_IOCON_Task_Func */
/**
* @brief Function implementing the IOExternal_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IOCON_Task_Func */
void IOCON_Task_Func(void const * argument)
{
  /* USER CODE BEGIN IOCON_Task_Func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IOCON_Task_Func */
}

/* USER CODE BEGIN Header_CAN_Task_Func */
/**
* @brief Function implementing the CAN_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task_Func */
void CAN_Task_Func(void const * argument)
{
  /* USER CODE BEGIN CAN_Task_Func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN_Task_Func */
}

/* USER CODE BEGIN Header_Debug_Task_Func */
/**
* @brief Function implementing the Debug_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Debug_Task_Func */
void Debug_Task_Func(void const * argument)
{
  /* USER CODE BEGIN Debug_Task_Func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Debug_Task_Func */
}

/* USER CODE BEGIN Header_Flash_Task_Func */
/**
* @brief Function implementing the Flash_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Flash_Task_Func */
void Flash_Task_Func(void const * argument)
{
  /* USER CODE BEGIN Flash_Task_Func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Flash_Task_Func */
}

/* USER CODE BEGIN Header_AD_Task_Func */
/**
* @brief Function implementing the AD_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AD_Task_Func */
void AD_Task_Func(void const * argument)
{
  /* USER CODE BEGIN AD_Task_Func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AD_Task_Func */
}

/* USER CODE BEGIN Header_ADC_Inside_Task */
/**
* @brief Function implementing the ADC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADC_Inside_Task */
void ADC_Inside_Task(void const * argument)
{
  /* USER CODE BEGIN ADC_Inside_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ADC_Inside_Task */
}

/* USER CODE BEGIN Header_DAC_OUT_Task */
/**
* @brief Function implementing the DAC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DAC_OUT_Task */
void DAC_OUT_Task(void const * argument)
{
  /* USER CODE BEGIN DAC_OUT_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DAC_OUT_Task */
}

/* USER CODE BEGIN Header_LCD_UART_Task */
/**
* @brief Function implementing the LCD_TTL_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LCD_UART_Task */
void LCD_UART_Task(void const * argument)
{
  /* USER CODE BEGIN LCD_UART_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LCD_UART_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
