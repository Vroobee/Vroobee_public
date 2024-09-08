/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"

#include "usbd_cdc_if.h"
#include "fatfs_sd.h"
#include "eeprom.h"

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
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//FATFS variables
FATFS fs;
FIL fil;
FRESULT fresult;
char buffer [256];

//uSD card capacity related variables
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

uint8_t m_state, m_init;
uint8_t sd_update;
uint32_t m_current_number, m_session_number;

uint16_t VirtAddVarTab[NB_OF_VAR];
uint16_t VarDataTab[NB_OF_VAR] = {0};
uint16_t VarDataTabRead[NB_OF_VAR];
uint16_t VarIndex,VarDataTmp = 0;

uint8_t file_update = 0;

typedef struct TTime{
	uint32_t us; 			//100us - 1/10ms
	uint32_t mils;
	uint32_t sec;
	uint16_t min;
	uint16_t hour;
} TTime;
TTime Time;

bool state = true;

char file_name [64];
char tbuf [64] = {'0'};

void led_blink (uint8_t state);

void send_uart(char *string);
void send_usb(char *string);
int bufsize (char *buf);
void bufclear (void);

void measurement (void);

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
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_FLASH_Unlock();


  //EEPROM Init
  if( EE_Init() != HAL_OK){
	  Error_Handler();
  }

  // Fill EEPROM variables addresses
  for(VarIndex = 1; VarIndex <= NB_OF_VAR; VarIndex++){
	  VirtAddVarTab[VarIndex-1] = VarIndex;
  }

  //read EEPROM
  for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++){
	  if((EE_ReadVariable(VirtAddVarTab[VarIndex], &VarDataTabRead[VarIndex])) != HAL_OK){
		  Error_Handler();
  	  }
  }

  m_session_number = VarDataTabRead[0];

  if (m_session_number < 256){
	  VarDataTab[0] = m_session_number + 1;
  } else {
	  m_session_number = 0;
	  VarDataTab[0] = m_session_number + 1;
  }

  for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++){
	  if((EE_WriteVariable(VirtAddVarTab[VarIndex],  VarDataTab[NB_OF_VAR-VarIndex-1])) != HAL_OK){
  		  Error_Handler();
  	  }
  }

  HAL_Delay(1000);

  send_uart("\r\n***** uSD_CARD INIT ***** \r\n \r\n");
  send_usb("\r\n***** uSD_CARD INIT ***** \r\n \r\n");


  fresult = f_mount(&fs, "", 1);
  if (fresult != FR_OK) {
	  led_blink(0);
	  send_uart ("ERROR MOUNTING SD CARD !!!\r\n \r\n");
	  send_usb ("ERROR MOUNTING SD CARD !!!\r\n \r\n");
  } else {
	  led_blink(1);
	  send_uart ("uSD CARD MOUNTED SUCCESFULLY!!!\r\n \r\n");
	  send_usb ("uSD CARD MOUNTED SUCCESFULLY!!!\r\n \r\n");
  }

  HAL_Delay(100);

  send_uart(":: uSD_CARD INFO ::\r\n");
  send_usb(":: uSD_CARD INFO ::\r\n");


  f_getfree ("", &fre_clust, &pfs);
  total = (uint32_t) (( pfs -> n_fatent - 2) * pfs -> csize * 0.5);
  sprintf (buffer, "uSD CARD total size: \t%lu bytes / %lu mb\r\n", total, total/1024);
  send_uart(buffer);
  send_usb(buffer);
  bufclear();

  free_space = (uint32_t) ( fre_clust * pfs -> csize * 0.5);
  sprintf (buffer, "uSD CARD free space: \t%lu bytes / %lu mb\r\n", free_space, free_space/1024);
  send_uart(buffer);
  send_usb(buffer);
  bufclear();

  sprintf (buffer, "\r\nSession no.: \t%ld\r\n", m_session_number);
  send_uart(buffer);
  send_usb(buffer);
  bufclear();

  send_uart("************************* \r\n");
  send_usb("************************* \r\n");


  HAL_Delay(100);


  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, ENABLE);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  switch (m_state){
	  case 1:{
		  if (m_init == 0){
				HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, DISABLE);

				sprintf(file_name, "measure_%ld_%ld.txt", m_session_number, m_current_number);

				fresult = f_open (&fil, file_name, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

				m_current_number++;
				m_init = 1;

				Time.us = 0;
				Time.mils = 0;
				Time.sec = 0;
				Time.min = 0;

	  			send_uart("\r\n*** FILE INFO ***\r\n");
	  			send_usb("\r\n*** FILE INFO ***\r\n");

	  			sprintf(buffer, "file created: %s\r\n", file_name);
	  			send_uart(buffer);
	  			send_usb(buffer);
	  			bufclear();

	  			//SD CARD INFO WRITE
	  			fresult = f_puts ("*****\tFILE\tINFO\t*****\n", &fil);
	  			fresult = f_lseek (&fil, fil.fsize);
	  			sprintf(tbuf, "SSN:\t%ld\n", m_session_number);
	  			fresult = f_puts (tbuf, &fil);
	  			fresult = f_lseek (&fil, fil.fsize);
	  			sprintf(tbuf, "MSNT:\t%ld\n", m_current_number - 1);
	  			fresult = f_puts (tbuf, &fil);
	  			fresult = f_lseek (&fil, fil.fsize);
	  			fresult = f_puts ("LSB:\t0.10\t[ms]\n", &fil);
	  			//fresult = f_lseek (&fil, fil.fsize);
	  			//fresult = f_puts ("*****\t*****\t*****\t*****\n", &fil);
	  			fresult = f_lseek (&fil, fil.fsize);
	  			fresult = f_puts ("-----\t-----\t-----\t-----\n", &fil);
	  			fresult = f_lseek (&fil, fil.fsize);
	  			fresult = f_puts ("D_IN1\tD_IN2\tD_IN3\tD_IN4\n", &fil);
	  			fresult = f_lseek (&fil, fil.fsize);
	  			fresult = f_puts ("-----\t-----\t-----\t-----\n", &fil);

	  			f_getfree ("", &fre_clust, &pfs);
	  			total = (uint32_t) (( pfs -> n_fatent - 2) * pfs -> csize * 0.5);
	  			sprintf (buffer, "uSD CARD total size: \t%lu bytes / %lu mb\r\n", total, total/1024);
	  			send_uart(buffer);
	  			send_usb(buffer);
	  			bufclear();

	  			free_space = (uint32_t) ( fre_clust * pfs -> csize * 0.5);
	  			sprintf (buffer, "uSD CARD free space: \t%lu bytes / %lu mb\r\n", free_space, free_space/1024);
	  			send_uart(buffer);
	  			send_usb(buffer);
	  			bufclear();



	  			send_uart("\r\n---> MEASUREMENT START \r\n");
	  			send_usb("\r\n---> MEASUREMENT START \r\n");
	 	  }

		  if (sd_update) {
	 		  measurement();
	 	  }
	 	  break;
	  }
	  case 2: {
		  f_close(&fil);

		  m_init = 0;
	 	  m_state = 0;

  		  send_uart("MEASUREMENT STOPPED <---\r\n");
  		  send_usb("MEASUREMENT STOPPED <---\r\n");

		  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, ENABLE);

	 	  break;
	  }
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, USER_LED_Pin|SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_SW_Pin D_IN1_Pin D_IN2_Pin D_IN3_Pin
                           D_IN4_Pin */
  GPIO_InitStruct.Pin = USER_SW_Pin|D_IN1_Pin|D_IN2_Pin|D_IN3_Pin
                          |D_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void led_blink (uint8_t state){
	if (state == 1){
		for (uint8_t i = 0; i < 10; i++){
			HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, DISABLE);
			HAL_Delay(50);
			HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, ENABLE);
			HAL_Delay(50);
		}
	} else {
		for (uint8_t i = 0; i < 4; i++){
			HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, DISABLE);
			HAL_Delay(250);
			HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, ENABLE);
			HAL_Delay(250);
		}
	}
}

void measurement (void){
		switch (sd_update){
		case 1:
			sprintf(tbuf, "%ld\n", Time.us);
			break;
		case 2:
			sprintf(tbuf, "\t%ld\n", Time.us);
			break;
		case 3:
			sprintf(tbuf, "\t\t%ld\n", Time.us);
			break;
		case 4:
			sprintf(tbuf, "\t\t\t%ld\n", Time.us);
			break;
		}

		fresult = f_lseek (&fil, fil.fsize);
		fresult = f_puts (tbuf, &fil);

		sd_update = 0;
}


void send_uart(char *string){
	uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart1, (uint8_t*) string, len, 2000);
}

void send_usb(char *string){
	uint8_t len = strlen(string);
    CDC_Transmit_FS((uint8_t*) string, len);
}

int bufsize (char *buf){
	int i = 0;
	while (*buf++ != '\0') i++;
	return i;
}

void bufclear (void){
	for (int i = 0; i < 256; i++){
		buffer [i] = '\0';
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim->Instance == TIM2){
		Time.us ++;
	} else {
		Time.us = 0;
		Time.mils = 0;
		Time.sec = 0;
		Time.min = 0;
		Time.hour = 0;
	}

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	switch (GPIO_Pin){
	case USER_SW_Pin:
		if (!HAL_GPIO_ReadPin(USER_SW_GPIO_Port, USER_SW_Pin)){
			m_state ++;
		}
		break;
	case D_IN1_Pin:
		if (!HAL_GPIO_ReadPin(D_IN1_GPIO_Port, D_IN1_Pin)){
			sd_update = 1;
		}
		break;
	case D_IN2_Pin:
		if (!HAL_GPIO_ReadPin(D_IN2_GPIO_Port, D_IN2_Pin)){
			sd_update = 2;
		}
		break;
	case D_IN3_Pin:
		if (!HAL_GPIO_ReadPin(D_IN3_GPIO_Port, D_IN3_Pin)){
			sd_update = 3;
		}
		break;
	case D_IN4_Pin:
		if (!HAL_GPIO_ReadPin(D_IN4_GPIO_Port, D_IN4_Pin)){
			sd_update = 4;
		}
		break;
	}
}


/* USER CODE END 4 */

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
