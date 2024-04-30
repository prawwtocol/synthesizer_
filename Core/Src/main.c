/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------==------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*configuration parameters for synthesizer*/
/*нужно инициализировать какими-то дефолтными значениями, чтобы при первом включении микруха выставилась*/
typedef struct __config_typedef{
	uint32_t frequency;
	uint8_t power_mode;
}config_typedef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMSIZE 35
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
uint8_t Rx_buf[CMSIZE];   //buffer for received command
uint8_t Tx_buf[CMSIZE];   //
uint8_t Tx_buf_size;
uint8_t freq_buf[5]; //buffer for frequency value in format xxx.x
volatile uint8_t frequency_received = 0;  //flag special for frequency value receiving
uint8_t Rx_index = 0;     //index for end of command word
volatile uint8_t msg_received = 0;     //0 if command word not received, 1 if command received
uint8_t command = 0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_NVIC_Init(void);
void load_config_in_flash(config_typedef *pll_config);  //takes as argument a pointer on structure
void read_config_from_flash(config_typedef *pll_config);//takes as argument a pointer on structure
void transmit_SPI_data(void);
void receive_SPI_data(void);



int main(void)
{
  config_typedef config = {
		  .frequency = 0xaaaaaaaa,
		  .power_mode = 0x01
  };
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* Initialize interrupts */
  MX_NVIC_Init();


 // read_config_from_flash(&config);// read configuration saved in flash after power on
  /*сделали функцию конфигурации в зависимости от запомненных значений*/


  //далее инициализировали прием команд с прерыванием
  HAL_UART_Receive_IT(&huart2, &Rx_buf[Rx_index], 1); //initialize first symbol receive
  /* USER CODE END 2 */

  while (1)
  {
	  switch (command) {
	  case 1: //freq
		  Tx_buf[Tx_buf_size] = "Enter frequency (format xxx.x)";
		  HAL_UART_Transmit(&huart2,  Tx_buf, Tx_buf_size, HAL_MAX_DELAY);
		  HAL_UART_Receive(&huart2, Rx_buf, 5, 50000); //timeout = 50 s

		  if (HAL_UART_Receive==HAL_TIMEOUT){
			  Tx_buf_size = 35;
			  Tx_buf[Tx_buf_size] = "timeout, frequency will not change";
			  HAL_UART_Transmit(&huart2,  Tx_buf, Tx_buf_size, HAL_MAX_DELAY);
		  }


			//функция, вызывающая обработку команды freq:
				  /*
				   * передаем сообщение "Enter frequency" с МК,
				   * ждем значение частоты
				   *
				   */
	   memset(Tx_buf, 0, CMSIZE); //reset TxBuf after message was sent
	   command = 0;  //reset indicator so in next cycle NOOP if new command wasn't received
	  // а если прерывание произошло тут, то в следующем цикле будет отработана команда пришедшая
	  break;

	  case 2: //temp
		  /* функция достает значение температуры ядра
	      * функция отправляет значение температуры ядра
	      *
		  * */
	  	  break;
	  case 3: //Lock_detect
		  /*достаем по SPI=
			*
		    * */
	  	  break;
	  case 4: //Power_down
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); //resetting pin turn synthesizer in power down mode
	           /*
			   * записываем во флеш то, что выключили, чтобы при пропадании питания и включении микруха так же была pwr_down
			   *
			   * */
	  	  break;
	  case 5: //power_up
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); //resetting pin turn synthesizer in power down mode
			  /*
			   * записываем во флеш то, что включили, чтобы при пропадании питания и включении микруха так же была pwr_down
			   *
			   * */
	  	  break;
	  case 6: //frequency value

		  /*конвертировали в число
			   * посчитали по формуле в нужные парамеры
			   * записали во флеш
			   * запустили функцию конфигурации по SP
			   * */
	  	  break;
	  default:
		  break;
  }

}
}


/*callback function to process receive operation*/
/*calls with interrupt after receiving 1 byte of message*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

  // check for command termination character
  if ((Rx_buf[Rx_index] != '\r') | (Rx_buf[Rx_index] != '\n')) {
    // Continue receiving if the termination character is not received
	  Rx_index++;//первый символ уже принят, произошло прерывание, которое привело нас сюда
      HAL_UART_Receive_IT(&huart2, &Rx_buf[Rx_index], 1);  // Receive next byte
  } else {

    // Reset the index for the next command, set flag for received data
	  Rx_index = 0;
	  msg_received = 1;

}

}

void USART2_IRQHandler(void)
{

  HAL_UART_IRQHandler(&huart2); //check for some errors, calls callback function

  /*check for which command was received*/
  if (msg_received == 1) {    //check for received messages
      if (strcmp(Rx_buf, "freq") == 0){   //check for which command was received
      Tx_buf_size = sizeof("Enter frequency (format xxx.x)");
	  command = 1; //

	  msg_received = 0; //reset flag at the end of ISR
	  memset(Rx_buf, 0, CMSIZE); //reset RxBuf at the end of ISR
	  return;
  }
	  else if (strcmp(Rx_buf, "temp") == 0){

	  command =2;

	  msg_received = 0; //reset flag at the end of ISR
	  memset(Rx_buf, 0, CMSIZE); //reset RxBuf at the end of ISR
	  return;
	  }
	  else if (strcmp(Rx_buf, "Lock_detect") == 0) {

	  command = 3;
	  msg_received = 0; //reset flag at the end of ISR
	  memset(Rx_buf, 0, CMSIZE); //reset RxBuf at the end of ISR
	 return;
	  }
	  else if(strcmp(Rx_buf,"pwr_down")==0)	  {

	  command = 4;

		  msg_received = 0; //reset flag at the end of ISR
		  memset(Rx_buf, 0, CMSIZE); //reset RxBuf at the end of ISR
	 return;
	  }
	  else if(strcmp(Rx_buf,"pwr_up")==0)	  {

	  command = 5;
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); //resetting pin turn synthesizer in power down mode
		  /*
		   * записываем во флеш то, что включили, чтобы при пропадании питания и включении микруха так же была pwr_down
		   *
		   * */
		  msg_received = 0; //reset flag at the end of ISR
		  memset(Rx_buf, 0, CMSIZE); //reset RxBuf at the end of ISR
	 return;	//возврат из обработчика прерываний
	  }

	  else if (frequency_received == 1){ //check for frequency value info

	  command = 6;

	       msg_received = 0; //reset flag at the end of ISR
		   memset(Rx_buf, 0, CMSIZE); //reset RxBuf at the end of ISR
	  }
	  else {
		  msg_received = 0; //reset flag at the end of ISR
		  memset(Rx_buf, 0, CMSIZE); //reset RxBuf at the end of ISR
	 return;
		  /**/
	  }
  }
  else { //if msg_received == 0
	  return;//return from ISR and wait next byte

  }

}


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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* NVIC Configuration */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/*  ADC1 Initialization Function*/
static void MX_ADC1_Init(void)
{


  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START; //?
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

}


  /*  SPI1 Initialization Function*/

static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }


}


  /*  USART2 Initialization Function*/

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
    Error_Handler();
  }

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LE_Pin|CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0
                           PC1 PC2 PC3 PC6
                           PC7 PC8 PC9 PC10
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA8
                           PA9 PA10 PA11 PA12
                           PA13 PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LE_Pin CE_Pin */
  GPIO_InitStruct.Pin = LE_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB3 PB4 PB5
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
