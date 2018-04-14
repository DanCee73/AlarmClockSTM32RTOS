
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_host.h"

/* USER CODE BEGIN Includes */
#include "seg.h"
#include "debug.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
RTC_TimeTypeDef current_time;

TaskHandle_t taskH_display_time = NULL;
TaskHandle_t taskH_startup_task = NULL;

TimerHandle_t second = NULL;

uint16_t segdis[] = {SEGDIG_0, SEGDIG_1, SEGDIG_2, SEGDIG_3, SEGDIG_4, SEGDIG_5, SEGDIG_6, SEGDIG_7, SEGDIG_8, SEGDIG_9};
uint16_t anodes[] = {ANODE_A_Pin, ANODE_B_Pin};

BaseType_t sec_exp = NULL;
BaseType_t pwr_reset = pdTRUE;
BaseType_t in_task = pdFALSE;

SemaphoreHandle_t sem_hr_task = NULL;
SemaphoreHandle_t sem_min_task = NULL;
SemaphoreHandle_t sem_reset_state = NULL;
SemaphoreHandle_t sem_kill_task = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void prvDisplayTime(void *p);
static void prvPwrOn(void *p);
static void prvSecExp(TimerHandle_t xTimer);

static void prvIncHr(void *p);
static void prvIncMin(void *p);
static void prvKillTask(void *p);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DAC_Init();
  MX_SPI1_Init();
  MX_SDIO_SD_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(ANODE_BASE, ALL_ANODE, GPIO_PIN_SET);
  current_time.Hours = (uint8_t) 12;
  current_time.Minutes = (uint8_t) 00;
  current_time.Seconds = 0b00000000;

  HAL_RTC_SetTime(&hrtc, &current_time, RTC_FORMAT_BIN);
  xTaskCreate(prvDisplayTime, "TimeDisplay", configMINIMAL_STACK_SIZE, NULL, 3 , &taskH_display_time);
  xTaskCreate(prvPwrOn, "PwrOn", configMINIMAL_STACK_SIZE, NULL, 4, &taskH_startup_task);
  xTaskCreate(prvIncHr, "AddHour", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
  xTaskCreate(prvIncMin, "AddMin", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
  xTaskCreate(prvKillTask, "Kill_task", configMINIMAL_STACK_SIZE, NULL, 6, NULL);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  sem_reset_state = xSemaphoreCreateBinary();
  sem_hr_task = xSemaphoreCreateBinary();
  sem_min_task = xSemaphoreCreateBinary();
  sem_kill_task = xSemaphoreCreateBinary();

  xSemaphoreGive(sem_reset_state);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  second = xTimerCreate("segment_update", pdMS_TO_TICKS(1000), pdFALSE, ( void * ) 0, prvSecExp);
  xTimerStart(second, portMAX_DELAY);

  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 124;
  hrtc.Init.SynchPrediv = 7999;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PC7   ------> I2S3_MCK
     PB6   ------> I2C1_SCL
     PB9   ------> I2C1_SDA
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|SEG_a_Pin|SEG_b_Pin|SEG_c_Pin 
                          |SEG_d_Pin|SEG_e_Pin|SEG_f_Pin|SEG_g_Pin 
                          |SEG_h_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(alarm_set_GPIO_Port, alarm_set_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ANODE_A_Pin|ANODE_B_Pin|ANODE_C_Pin|ANODE_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin SEG_a_Pin SEG_b_Pin SEG_c_Pin 
                           SEG_d_Pin SEG_e_Pin SEG_f_Pin SEG_g_Pin 
                           SEG_h_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|SEG_a_Pin|SEG_b_Pin|SEG_c_Pin 
                          |SEG_d_Pin|SEG_e_Pin|SEG_f_Pin|SEG_g_Pin 
                          |SEG_h_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin alarm_set_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|alarm_set_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : hr_btn_Pin min_btn_Pin set_alarm_btn_Pin */
  GPIO_InitStruct.Pin = hr_btn_Pin|min_btn_Pin|set_alarm_btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ANODE_A_Pin ANODE_B_Pin ANODE_C_Pin ANODE_D_Pin */
  GPIO_InitStruct.Pin = ANODE_A_Pin|ANODE_B_Pin|ANODE_C_Pin|ANODE_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_MCK_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_MCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
static void prvDisplayTime(void *p)
{
	(void) p;
	uint8_t hh = 0;
	uint8_t mm = 0;
	uint8_t ss = 0;
	uint8_t old_ss;
	int hours_tens;
	int hours_ones;
	int mins_tens;
	int mins_ones;

	char numbuf[4];

	while(1)
	{

		xTimerReset(second, portMAX_DELAY); //Reset the timer if it hasn't already been reset

		//While the timer hasn't expired, run the code below, otherwise on expiry, increment "number" and reset timer
		if(sec_exp == pdFALSE)
		{
			HAL_RTC_GetTime(&hrtc, &current_time, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, NULL, RTC_FORMAT_BIN);
			hh = current_time.Hours;
			mm = current_time.Minutes;
			ss = current_time.Seconds;

			hours_tens = hh / 10;
			hours_ones = hh % 10;
			mins_tens = mm / 10;
			mins_ones = mm % 10;

			//THOUSANDS DIGIT
			//Clear all anodes
			HAL_GPIO_WritePin(ANODE_A_GPIO_Port, ALL_ANODE, GPIO_PIN_RESET);
			//Clear all cathodes, set cathodes for tens spot, then set anode A
			HAL_GPIO_WritePin(GPIOE_BASE, ALLSEG, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE_BASE, segdis[hours_tens], GPIO_PIN_SET);
			HAL_GPIO_WritePin(ANODE_BASE, ANODE_D_Pin, GPIO_PIN_SET);
			HAL_Delay(1);

			//HUNDREDS DIGIT
			//Clear all anodes
			HAL_GPIO_WritePin(ANODE_A_GPIO_Port, ALL_ANODE, GPIO_PIN_RESET);
			//Clear all cathodes, set cathodes for tens spot, then set anode B
			HAL_GPIO_WritePin(GPIOE_BASE, ALLSEG, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE_BASE, segdis[hours_ones], GPIO_PIN_SET);
			HAL_GPIO_WritePin(ANODE_BASE, ANODE_C_Pin, GPIO_PIN_SET);
			HAL_Delay(1);

			//TENS DIGIT
			//Clear all anodes
			HAL_GPIO_WritePin(ANODE_A_GPIO_Port, ALL_ANODE, GPIO_PIN_RESET);
			//Clear all cathodes, set cathodes for tens spot, then set anode A
			HAL_GPIO_WritePin(GPIOE_BASE, ALLSEG, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE_BASE, segdis[mins_tens], GPIO_PIN_SET);
			HAL_GPIO_WritePin(ANODE_BASE, ANODE_B_Pin, GPIO_PIN_SET);
			HAL_Delay(1);

			//ONES DIGIT
			//Clear all anodes
			HAL_GPIO_WritePin(ANODE_A_GPIO_Port, ALL_ANODE, GPIO_PIN_RESET);
			//Clear all cathodes, set cathodes for tens spot, then set anode B
			HAL_GPIO_WritePin(GPIOE_BASE, ALLSEG, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE_BASE, segdis[mins_ones], GPIO_PIN_SET);
			HAL_GPIO_WritePin(ANODE_BASE, ANODE_A_Pin, GPIO_PIN_SET);
			HAL_Delay(1);

			if(old_ss != ss)
				{
					itoa(hh, numbuf, 10);
					_write(numbuf);
					_write(":");
					itoa(mm, numbuf, 10);
					_write(numbuf);
					_write(":");
					itoa(ss, numbuf, 10);
					_writeln(numbuf);
					old_ss = ss;
				}
		}

		else
		{
			//convert to string for debugging
			itoa(hh, numbuf, 10);
			_write(numbuf);
			_write(":");
			itoa(mm, numbuf, 10);
			_write(numbuf);
			_write(":");
			itoa(ss, numbuf, 10);
			_writeln(numbuf);
			_writeln(" ");

			sec_exp = pdFALSE;
			xTimerReset(second, portMAX_DELAY);
		}
	}
}

static void prvSecExp(TimerHandle_t xTimer)
{
	(void) xTimer;
	sec_exp = pdTRUE;
}

static void prvPwrOn(void *p)
{
	(void) p;
	int i;
	/// TODO: Default running task on powerup. Like the flashing 12:00 you see on clocks when the power goes out
	while(1)
	{
		if(xSemaphoreTake(sem_reset_state, portMAX_DELAY) == pdTRUE)
		{
			i = 0;
			while(i < 100)
			{
				//THOUSANDS DIGIT
				//Clear all anodes
				HAL_GPIO_WritePin(ANODE_A_GPIO_Port, ALL_ANODE, GPIO_PIN_RESET);
				//Clear all cathodes, set cathodes for tens spot, then set anode A
				HAL_GPIO_WritePin(GPIOE_BASE, ALLSEG, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE_BASE, segdis[1], GPIO_PIN_SET);
				HAL_GPIO_WritePin(ANODE_BASE, ANODE_D_Pin, GPIO_PIN_SET);
				HAL_Delay(1);

				//HUNDREDS DIGIT
				//Clear all anodes
				HAL_GPIO_WritePin(ANODE_A_GPIO_Port, ALL_ANODE, GPIO_PIN_RESET);
				//Clear all cathodes, set cathodes for tens spot, then set anode B
				HAL_GPIO_WritePin(GPIOE_BASE, ALLSEG, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE_BASE, segdis[2], GPIO_PIN_SET);
				HAL_GPIO_WritePin(ANODE_BASE, ANODE_C_Pin, GPIO_PIN_SET);
				HAL_Delay(1);

				//TENS DIGIT
				//Clear all anodes
				HAL_GPIO_WritePin(ANODE_A_GPIO_Port, ALL_ANODE, GPIO_PIN_RESET);
				//Clear all cathodes, set cathodes for tens spot, then set anode A
				HAL_GPIO_WritePin(GPIOE_BASE, ALLSEG, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE_BASE, segdis[0], GPIO_PIN_SET);
				HAL_GPIO_WritePin(ANODE_BASE, ANODE_B_Pin, GPIO_PIN_SET);
				HAL_Delay(1);

				//ONES DIGIT
				//Clear all anodes
				HAL_GPIO_WritePin(ANODE_A_GPIO_Port, ALL_ANODE, GPIO_PIN_RESET);
				//Clear all cathodes, set cathodes for tens spot, then set anode B
				HAL_GPIO_WritePin(GPIOE_BASE, ALLSEG, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE_BASE, segdis[0], GPIO_PIN_SET);
				HAL_GPIO_WritePin(ANODE_BASE, ANODE_A_Pin, GPIO_PIN_SET);
				HAL_Delay(1);
				i++;
			}

			i = 0;
			HAL_GPIO_WritePin(ANODE_A_GPIO_Port, ALL_ANODE, GPIO_PIN_RESET);

			while(i < 400)
			{
				HAL_Delay(1);
				i++;
			}

			if(pwr_reset == pdTRUE)
				xSemaphoreGive(sem_reset_state);
			/*
			 * else
				vTaskDelete(NULL);
			*/
		}
	}
}

static void prvIncHr(void *p)
{
	(void) p;
	RTC_TimeTypeDef buf_time = current_time;

	while(1)
	{
		if(xSemaphoreTake(sem_hr_task, portMAX_DELAY) == pdTRUE)
		{
			in_task = pdTRUE;
			HAL_RTC_GetTime(&hrtc, &buf_time, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, NULL, RTC_FORMAT_BIN);
			buf_time.Hours++;
			if (buf_time.Hours > 23)
				buf_time.Hours = 0;
			buf_time.Seconds = 0;
			HAL_RTC_SetTime(&hrtc, &buf_time, RTC_FORMAT_BIN);
			vTaskDelay(pdMS_TO_TICKS(350));
			in_task= pdFALSE;
		}
	}
}

static void prvIncMin(void *p)
{
	(void) p;
	RTC_TimeTypeDef buf_time = current_time;

	while(1)
	{
		if(xSemaphoreTake(sem_min_task, portMAX_DELAY) == pdTRUE)
		{
			in_task = pdTRUE;
			HAL_RTC_GetTime(&hrtc, &buf_time, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, NULL, RTC_FORMAT_BIN);
			buf_time.Minutes++;
			if (buf_time.Minutes > 59)
				buf_time.Minutes = 0;
			buf_time.Seconds = 0;
			HAL_RTC_SetTime(&hrtc, &buf_time, RTC_FORMAT_BIN);
			vTaskDelay(pdMS_TO_TICKS(350));
			in_task= pdFALSE;
		}
	}
}

static void prvKillTask(void *p)
{
	(void) p;
	RTC_TimeTypeDef buf_time = current_time;

	while(1)
	{
		if(xSemaphoreTake(sem_kill_task, portMAX_DELAY) == pdTRUE)
		{
			HAL_GPIO_WritePin(ANODE_A_GPIO_Port, ALL_ANODE, GPIO_PIN_RESET);
			vTaskDelete(taskH_startup_task);
			buf_time.Hours = 00;
			buf_time.Minutes = 00;
			HAL_RTC_SetTime(&hrtc, &buf_time, RTC_FORMAT_BIN);

		}
	}
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();

  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
