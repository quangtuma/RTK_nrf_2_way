/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "NRF24.h"
#include "NRF24_reg_addresses.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NRF_SIZE 32
#define MAX_SIZE 1000
#define GGA_SIZE 250

#define INTERVAL_MODE_5 2 // = 1s

#define TX
#define Quectel
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

uint8_t Send_Large_Data(uint8_t* data, uint16_t size);
int16_t Get_End_RTCM_Index(uint8_t* packet, uint16_t size, uint16_t start);
int16_t Get_End_GGA_Index(uint8_t* data, uint16_t size, uint16_t start);

#ifdef RX
void handleUARTError(UART_HandleTypeDef *huart);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef TX
uint8_t nrf_tx_data[MAX_SIZE] = {};
uint8_t ack_Tx[NRF_SIZE];
#endif

#ifdef RX

uint8_t ack_Rx[NRF_SIZE] = { "Received" };
volatile uint8_t gga_index = 0;      // Tracks the current index for incoming data
volatile uint8_t data_ready = 0;    // Flag to signal that a string is ready
#endif

uint8_t nrf_rx_packet[NRF_SIZE];
uint8_t nrf_rx_data[MAX_SIZE];
uint16_t nrf_packet_id = 0;

uint8_t rover_gga_message[GGA_SIZE];
uint8_t rover_mode;
uint8_t interval_mode_5 = INTERVAL_MODE_5;

uint8_t GGA_set_command[18] = { 0x24, 0x50, 0x41, 0x49, 0x52, 0x30, 0x36, 0x32, 0x2C, 0x30, 0x2C, 0x30, 0x31, 0x2A, 0x30, 0x46, 0x0D, 0x0A }; //$PAIR062,0,01*0F
uint8_t GGA_start[6] = { 0x24, 0x47, 0x4E, 0x47, 0x47, 0x41 };

uint8_t ret = 0;
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  csn_high();
  nrf24_init();
  nrf24_tx_pwr(_0dbm);
  nrf24_data_rate(_1mbps);
  nrf24_set_channel(78);
  nrf24_set_crc(en_crc, _1byte);
  nrf24_pipe_pld_size(0, NRF_SIZE);

  uint8_t addr_base[5] = { 0x10, 0x21, 0x32, 0x43, 0x54 };
  uint8_t addr_rover[5] = { 0x10, 0x21, 0x32, 0x43, 0x53 };

  HAL_Delay(1000); // waiting initial Station

#ifdef TX

  nrf24_open_tx_pipe(addr_base);
  nrf24_open_rx_pipe(0, addr_rover);

#ifdef Quectel

  HAL_UART_Transmit(&huart2, GGA_set_command, sizeof(GGA_set_command) / sizeof(uint8_t), 100);
  HAL_Delay(500); // waiting initial GGA Command

#endif

#endif

#ifdef RX

  nrf24_open_tx_pipe(addr_rover);
  nrf24_open_rx_pipe(0, addr_base);

  HAL_UART_Receive_IT(&huart2, &rover_gga_message[gga_index], 1);

#endif

  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // TEST GIT
#ifdef TX

	  HAL_UART_Receive(&huart2, (uint8_t *)&nrf_tx_data, MAX_SIZE, 250);
	  // get end of data

	  int16_t rtcm_end_id = Get_End_RTCM_Index(nrf_tx_data, MAX_SIZE, 0);
	  if (rtcm_end_id > 0){
		  // separate data and send chunk
		  uint8_t check[rtcm_end_id]; // to extend to send
		  memcpy(check, nrf_tx_data, sizeof(check));

		  nrf24_stop_listen();

		  Send_Large_Data(check, rtcm_end_id);

		  memset(nrf_tx_data, 0, sizeof(nrf_tx_data));

		  HAL_Delay(1);
	  }

	  nrf24_listen();

	  if (nrf24_data_available()){

		  nrf24_receive(nrf_rx_packet, sizeof(nrf_rx_packet));

		  memcpy(&rover_gga_message[nrf_packet_id], nrf_rx_packet, sizeof(nrf_rx_packet));
		  nrf_packet_id += NRF_SIZE;

		  int16_t gga_end_id = Get_End_GGA_Index(rover_gga_message, GGA_SIZE, 0);

		  if (gga_end_id > 0){

			  HAL_UART_Transmit(&huart2, rover_gga_message, gga_end_id, 100);

			  memset(rover_gga_message, 0, sizeof(rover_gga_message));

			  nrf_packet_id = 0;
		  }

		  memset(nrf_rx_packet, 0, sizeof(nrf_rx_packet));

		  HAL_Delay(1);
	  }

#endif
#ifdef RX
	  // NRF Receiver Processing

	  HAL_UART_Receive(&huart2, (uint8_t *)&rover_gga_message, GGA_SIZE, 250);

	  int16_t end_gga_index = Get_End_GGA_Index(rover_gga_message, sizeof(rover_gga_message), 0);

	  if (end_gga_index > 0){

		  nrf24_stop_listen();

		  Send_Large_Data(rover_gga_message, end_gga_index);

		  int comma_count = 0;
		  for(int j = 0; j < GGA_SIZE; j++){
			  if (rover_gga_message[j] == 0x2C){ // 2C is ','
				  comma_count++;
				  if (comma_count == 6) // from $GNGGA to Rover Mode
				  {
					  rover_mode = rover_gga_message[j + 1];
					  break;
				  }
			  }
		  }

		  memset(rover_gga_message, 0, sizeof(rover_gga_message));
	  }

	  nrf24_listen();

	  if (nrf24_data_available()){
		  nrf24_receive(nrf_rx_packet, sizeof(nrf_rx_packet));

		  //HAL_UART_Transmit(&huart2, nrf_rx_packet, NRF_SIZE, 10);

		  memcpy(&nrf_rx_data[nrf_packet_id], nrf_rx_packet, sizeof(nrf_rx_packet) / sizeof(uint8_t));
		  nrf_packet_id += NRF_SIZE;

		  int16_t end_rtcm_id = Get_End_RTCM_Index(nrf_rx_data, MAX_SIZE, 0);

		  if (end_rtcm_id > 0){

			  HAL_UART_Transmit(&huart2, nrf_rx_data, end_rtcm_id, 250);

			  memset(nrf_rx_data, 0, sizeof(nrf_rx_data));

			  nrf_packet_id = 0;
		  }

		  memset(nrf_rx_packet, 0, sizeof(nrf_rx_packet));
	  }

	  // UART (Rover GGA) Receiver Processing
	  /*if (data_ready) {

		  data_ready = 0;  // Reset flag

		  nrf24_stop_listen();

		  int16_t end_gga_index = Get_End_GGA_Index(rover_gga_message, sizeof(rover_gga_message), 0);

		  Send_Large_Data(rover_gga_message, end_gga_index);

		  int comma_count = 0;
		  for(int j = 0; j < GGA_SIZE; j++){
			  if (rover_gga_message[j] == 0x2C){ // 2C is ','
				  comma_count++;
				  if (comma_count == 6) // from $GNGGA to Rover Mode
				  {
					  rover_mode = rover_gga_message[j + 1];
					  HAL_UART_Receive_IT(&huart2, &rover_gga_message[gga_index], 1);
					  break;
				  }
			  }
		  }
		  memset(rover_gga_message, 0, sizeof(rover_gga_message));
	  }*/
#endif
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1599;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2599;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CE_Pin|CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CE_Pin CSN_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int16_t Get_End_RTCM_Index(uint8_t* data, uint16_t size, uint16_t start)
{
	uint8_t d3_count = 0;

	int16_t rtcm_end_id = 0;

	if (start == 0)
		start = 1;

	for (int i = start; i < size; i++){
		if (data[i] == 0xD3)
			d3_count++;

		if(d3_count >=5 && data[i-1] == 0x0D && data[i] == 0x0A)
		{
			rtcm_end_id = i + 1;
			break;
		}
	}

	return rtcm_end_id;
}

int16_t Get_End_GGA_Index(uint8_t* data, uint16_t size, uint16_t start)
{
	// check start gga

	int16_t gga_end_id = 0;

	if (start == 0)
		start = 1;

	for (int i = start; i < size; i++){
		if(data[i-1] == 0x0D && data[i] == 0x0A)
		{
			gga_end_id = i + 1;
			break;
		}
	}

	return gga_end_id;
}

#ifdef RX
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		switch (rover_mode)
		{
			case '0':
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			break;
			case '4':
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				break;
			case '5':
				interval_mode_5--;
				if (interval_mode_5 == 0){
					interval_mode_5 = INTERVAL_MODE_5;
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				}
				break;
			default:
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				break;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == USART2){
    	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE) ||
			__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) ||
			__HAL_UART_GET_FLAG(huart, UART_FLAG_NE)) {

			// Handle UART errors (e.g., reset UART, alert user)
			handleUARTError(huart);
		}
		// Check if the buffer is full
		if (rover_gga_message[gga_index] == 0x0A
				&& rover_gga_message[gga_index - 1] == 0x0D) {
			data_ready = 1;  // Set flag to indicate data reception is complete
			gga_index = 0;
		}
		else {
	    	gga_index++;
			// Continue receiving next byte if buffer is not yet full
			HAL_UART_Receive_IT(&huart2, &rover_gga_message[gga_index], 1);
		}
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Handle error if UART has a problem (e.g., disconnection)
        handleUARTError(huart);
    }
}

void handleUARTError(UART_HandleTypeDef *huart) {
    // Disable UART interrupts to stop triggering further
    __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);

    // Optionally, reset UART (if needed)
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    // Re-enable UART for reception
    HAL_UART_Receive_IT(huart, &rover_gga_message[gga_index], 1);
}
#endif

uint8_t Send_Large_Data(uint8_t* data, uint16_t size)
{
	uint8_t ret = 0;
	uint8_t chunk[NRF_SIZE];

	for (uint16_t i = 0; i < size; i += NRF_SIZE) {

		uint16_t packet_size = 0;

		if (size - i < NRF_SIZE){
			packet_size = size - i;
		}
		else{
			packet_size = NRF_SIZE;
		}

		memcpy(chunk, data + i, packet_size * sizeof(uint8_t));

		// Send the 32-byte chunk

		//HAL_UART_Transmit(&huart2, chunk, NRF_SIZE, 5);
		ret = nrf24_transmit(chunk, sizeof(chunk));
		while(ret == 1){
			// TODO: timer break while
			HAL_Delay(1);

			ret = nrf24_transmit(chunk, sizeof(chunk));
		}

		for(uint8_t j = 0; j < NRF_SIZE; j++){
			chunk[j] = '\0';
		}

		// Short delay if necessary to avoid overload
		HAL_Delay(1);
	}
	return ret;
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
