/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LED_ON_CMD   			 				1
#define LED_OFF_CMD  			 				2
#define LED_TOGGLE_CMD   					3
#define LED_TOGGLE_STOP_CMD   		4
#define LED_READ_STATUS   				5
#define RTC_PRINT_DATETIME    		6

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

osThreadId Task3Handle;
osThreadId Task1Handle;
osThreadId Task2Handle;
osThreadId Task4Handle;
osMessageQId cmd_queueHandle;
osTimerId myTimer01Handle;
/* USER CODE BEGIN PV */
osMessageQId write_queueHandle;

RTC_TimeTypeDef sTime;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
void Task3_cmd_processing(void const * argument);
void Task1_Display_menu(void const * argument);
void Task2_cmd_handling(void const * argument);
void Task4_Uart_write(void const * argument);
void Callback01(void const * argument);

/* USER CODE BEGIN PFP */



uint8_t getCommandCode(uint8_t *buffer);
void getArguments(uint8_t *buffer);


 void make_led_on(void);
 void make_led_off(void);
 void led_toggle_start(void);
 void led_toggle_stop(void);
 void read_led_status(char task_msg[]);
 void read_rtc_inf(char* task_msg);
 void printf_error_message(char* task_msg);
 
 

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char RxData[2];
uint8_t command_buffer[20];
uint8_t command_len=0;
char message[100]={0};
typedef struct APP_CMD
	{
		uint8_t COMMAND_NUM;
		uint8_t COMMAND_ARG[10];
	}APP_CMD_t;


char menu[]={
	"\
	\r\n LED_ON   			 				:1\
	\r\n LED_OFF   			 				:2\
	\r\n LED_TOGGLE   					:3\
	\r\n LED_TOGGLE_OFF   			:4\
	\r\n LED_READ_STATUS    		:5\
	\r\n RTC_PRINT_DATETIME    	:6\
	\r\nType your option here : " 
           };
	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t pxHigherPriorityTaskWoken =pdFALSE;
	uint16_t data_byte;
	
	data_byte = USART2->DR;//data byte received from users
	command_buffer[command_len++]=data_byte & 0xFF;
	
			command_len=0;
			//then user is finished entering the data
			//lets Notify the command-handling
				xTaskNotifyFromISR(Task2Handle,0,eNoAction,&pxHigherPriorityTaskWoken);
				xTaskNotifyFromISR(Task1Handle,0,eNoAction,&pxHigherPriorityTaskWoken);
		
		UART_Start_Receive_IT(&huart2,(uint8_t*)RxData,1);
		//if the above RTos apis wake up any higher Piority task,then yield processing to th
		//higher piorrity task which is just wkae up
		if(pxHigherPriorityTaskWoken)
			{
				taskYIELD();
			}	
}	
//helper function

uint8_t getCommandCode(uint8_t *buffer)
{
	return buffer[0]-48;
}	
void getArguments(uint8_t *buffer)
{
	
}	

 void make_led_on(void)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
	
}	
 void make_led_off(void)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
}		
void led_toggle_start(void)
{
		if(myTimer01Handle==NULL)
		{
      	//let create the software timer
	     osTimerDef(myTimer01, Callback01);
       myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);
       //start software timer
	     osTimerStart(myTimer01Handle, 500);
		}
   else
    {	
			 osTimerStart(myTimer01Handle, 500);
    }			
}	

void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */
  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
  /* USER CODE END Callback01 */
}
 void led_toggle_stop(void)
{
		osTimerStop(myTimer01Handle);
}
	
 void read_led_status(char task_msg[])
	{
		sprintf(task_msg,"\r\nLed status is : %d \r\n",HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15));
	  xQueueSend(write_queueHandle,(void*)&task_msg,portMAX_DELAY);
	}	
 void read_rtc_inf(char* task_msg)
	{
			uint8_t Seconds=0;
			uint8_t Minutes=0;
			uint8_t Hours=0;
			
			HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BCD);
			
			Hours=sTime.Hours ;
			Minutes=sTime.Minutes;
			Seconds=sTime.Seconds;
		  sprintf(task_msg,"\r\n Time : \n%d:%d:%d \r\n",Hours,Minutes,Seconds);
			xQueueSend(write_queueHandle,(void*)&task_msg,portMAX_DELAY);
		
	}	
void printf_error_message(char* task_msg)
	{

		sprintf(task_msg,"\r\nInvalid command received\r\n"); 
		
	  xQueueSend(write_queueHandle,(void*)&task_msg,portMAX_DELAY);
	
	
	}	
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	
	UART_Start_Receive_IT(&huart2,(uint8_t*)RxData,1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, Callback01);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	

  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of cmd_queue */
  osMessageQDef(cmd_queue, 10, APP_CMD_t*);
  cmd_queueHandle = osMessageCreate(osMessageQ(cmd_queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	 osMessageQDef(write_queue, 10, sizeof(char*));
  write_queueHandle = osMessageCreate(osMessageQ(write_queue), NULL);
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task3 */
  osThreadDef(Task3, Task3_cmd_processing, osPriorityNormal, 0, 500);
  Task3Handle = osThreadCreate(osThread(Task3), NULL);

  /* definition and creation of Task1 */
  osThreadDef(Task1, Task1_Display_menu, osPriorityBelowNormal, 0, 500);
  Task1Handle = osThreadCreate(osThread(Task1), NULL);

  /* definition and creation of Task2 */
  osThreadDef(Task2, Task2_cmd_handling, osPriorityNormal, 0, 500);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

  /* definition and creation of Task4 */
  osThreadDef(Task4, Task4_Uart_write, osPriorityNormal, 0, 500);
  Task4Handle = osThreadCreate(osThread(Task4), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task3_cmd_processing */
/**
  * @brief  Function implementing the Task3 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task3_cmd_processing */
void Task3_cmd_processing(void const * argument)
{
  /* USER CODE BEGIN 5 */
	APP_CMD_t *new_cmd;
	char task_msg[50];
	
  /* Infinite loop */
  for(;;)
  {
     xQueueReceive(cmd_queueHandle,(void*)&new_cmd,portMAX_DELAY);
		
		if(new_cmd->COMMAND_NUM == LED_ON_CMD)
			{
				make_led_on();
			}else if(new_cmd->COMMAND_NUM == LED_OFF_CMD)
			{
				make_led_off();
			}else if(new_cmd->COMMAND_NUM == LED_TOGGLE_CMD)
			{
				led_toggle_start();
			}else if(new_cmd->COMMAND_NUM == LED_TOGGLE_STOP_CMD)
			{
				led_toggle_stop();
			}else if(new_cmd->COMMAND_NUM == LED_READ_STATUS)
			{
				read_led_status(task_msg);
			}else if(new_cmd->COMMAND_NUM == RTC_PRINT_DATETIME)
			{
				read_rtc_inf(task_msg);
			}else
			{
					printf_error_message(task_msg);
			}	
			vPortFree(new_cmd);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task1_Display_menu */
/**
* @brief Function implementing the Task1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task1_Display_menu */
void Task1_Display_menu(void const * argument)
{
  /* USER CODE BEGIN Task1_Display_menu */
	char *pData=menu;
  /* Infinite loop */
  for(;;)
  {
		xQueueSend(write_queueHandle,&pData,portMAX_DELAY);
		//lets wait until someone notifies
    xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
  }
  /* USER CODE END Task1_Display_menu */
}

/* USER CODE BEGIN Header_Task2_cmd_handling */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task2_cmd_handling */
void Task2_cmd_handling(void const * argument)
{
  /* USER CODE BEGIN Task2_cmd_handling */
	uint8_t command_code =0;
	APP_CMD_t *new_cmd;
  /* Infinite loop */
  for(;;)
  {
    xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		
			new_cmd=(APP_CMD_t*)pvPortMalloc(sizeof(APP_CMD_t));
		//1.send command to queue
		taskENTER_CRITICAL();
		command_code=getCommandCode(command_buffer);
		new_cmd=(APP_CMD_t*)pvPortMalloc(sizeof(APP_CMD_t));
		new_cmd->COMMAND_NUM = command_code;
		getArguments(new_cmd->COMMAND_ARG);
		taskEXIT_CRITICAL();
		xQueueSend(cmd_queueHandle,&new_cmd,portMAX_DELAY);
  }
  /* USER CODE END Task2_cmd_handling */
}

/* USER CODE BEGIN Header_Task4_Uart_write */
/**
* @brief Function implementing the Task4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task4_Uart_write */
void Task4_Uart_write(void const * argument)
{
  /* USER CODE BEGIN Task4_Uart_write */
	char *pDataRec=NULL;
  /* Infinite loop */
  for(;;)
  {
		xQueueReceive(write_queueHandle,&pDataRec,portMAX_DELAY);
		HAL_UART_Transmit(&huart2,(uint8_t*)pDataRec,sizeof(menu),500);
  }
  /* USER CODE END Task4_Uart_write */
}

/* Callback01 function */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
