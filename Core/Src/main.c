#include "main.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// Global variables
uint8_t hostCommand[] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
uint8_t bmsResponse[65];
char v_i_Str[50];
volatile uint8_t dataReady = 0;
uint32_t lastDataTime = 0;
const uint32_t TIMEOUT = 600;  // 2 seconds timeout
uint8_t TxData[8] = {0};  // CAN data payload of 8 bytes

uint8_t min_current = 0x32;
uint8_t max_current = 0xA0;
/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_TxHeaderTypeDef TxHeader;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART1_UART_Init(void);

int _write(int file, char *ptr, int len) {
    if (HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY) == HAL_OK) {
        return len;
    }
    return 0;
}


int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_FDCAN1_Init();
  FDCAN1_FilterConfig();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart1, bmsResponse, sizeof(bmsResponse));
  lastDataTime = HAL_GetTick(); // Initialize the last data time

  while (1)
  {
	  transmitBMSCommand();

	         if (dataReady && (HAL_GetTick() - lastDataTime < TIMEOUT)) {
	             transmitDataOverUSART1();
	             transmitDataOverCAN();
	             memset(v_i_Str, 0, sizeof(v_i_Str)); // Clear buffer
	             dataReady = 0; // Clear the flag

	         }
	         else {
	             CheckForTimeout();
	         }
  }
  /* USER CODE END 3 */
}

void transmitBMSCommand(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);			//LED indication HIgh
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);			//DE pin set High
	HAL_UART_Transmit(&huart1, hostCommand, sizeof(hostCommand),10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);		//LED indication LOW
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);		//DE pin set Low
	HAL_Delay(500);
}

void transmitDataOverUSART1(void) {

		int len;
		uint16_t voltage = (uint16_t)bmsResponse[4] << 8 | bmsResponse[5];
		len = snprintf(v_i_Str, sizeof(v_i_Str), "Voltage: %u", voltage);
	    HAL_UART_Transmit(&huart2, (uint8_t*)v_i_Str, len, 1000); // Transmit voltage

		uint16_t current = (uint16_t)bmsResponse[6] << 8 | bmsResponse[7];
	    len = snprintf(v_i_Str, sizeof(v_i_Str), "Current: %u", current);
	    HAL_UART_Transmit(&huart2, (uint8_t*)v_i_Str, len, 1000); // Transmit current

		uint16_t SoC = (uint16_t)bmsResponse[23];
		len = snprintf(v_i_Str, sizeof(v_i_Str), "SOC: %u\n", SoC);
		HAL_UART_Transmit(&huart2, (uint8_t*)v_i_Str, len, 1000); // Transmit current

		uint16_t TotalCells = (uint16_t)bmsResponse[25];
		len = snprintf(v_i_Str, sizeof(v_i_Str), "No. of Cells: %u\n", TotalCells);
		HAL_UART_Transmit(&huart2, (uint8_t*)v_i_Str, len, 1000); // Transmit current

}


//void transmitDataOverCAN(void) {
//    uint8_t TxData[8] = {0};  // CAN data payload of 8 bytes
//    uint32_t msgId = 0x200;  // Starting message ID for data messages
//
//    // Prepare CAN header
//    TxHeader.Identifier = msgId;
//    TxHeader.IdType = FDCAN_STANDARD_ID;
//    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
//    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
//    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
//    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
//    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//    TxHeader.MessageMarker = 0;
//
//    // Voltage and Current data packing
//    uint16_t voltage = (uint16_t)bmsResponse[4] << 8 | bmsResponse[5];
//    uint16_t current = (uint16_t)bmsResponse[6] << 8 | bmsResponse[7];
//    TxData[0] = (voltage >> 8) & 0xFF;
//    TxData[1] = voltage & 0xFF;
//    TxData[2] = (current >> 8) & 0xFF;
//    TxData[3] = current & 0xFF;
//
//    // State of Charge data, assuming single-byte SoC
//    uint8_t SoC = bmsResponse[23];  // Corrected to single-byte handling
//    TxData[4] = SoC;
//
//    // Debugging outputs to check data integrity
//    printf("Preparing to send CAN data: Voltage=%u, Current=%u, SOC=%u\n", voltage, current, SoC);
//
//    // Send CAN message
//    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {
//        printf("Failed to send data over CAN.\n");
//    } else {
//        printf("Data sent over CAN successfully.\n");
//    }
//}

void transmitDataOverCAN(void) {

	int totalCellVolt = (bmsResponse[25]*3.65)*100;
	    uint16_t scaled_Volt_Value = totalCellVolt/10;
	    uint8_t scaled_Volt_HighByte = (scaled_Volt_Value >> 8) & 0xFF;  // High byte
	    uint8_t scaled_Volt_LowByte = scaled_Volt_Value & 0xFF;
	    uint32_t msgId = 0x200;

    uint8_t broadcastAdd = 0xFF;
    uint8_t relayPowerON = 3;

    //     Prepare CAN header
    TxHeader.Identifier = msgId;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    switch (bmsResponse[23]>90){
    case 1:
    	TxData[0] = broadcastAdd;
    	TxData[1] = relayPowerON;
    	TxData[2] = scaled_Volt_HighByte;		// voltage high byte
    	TxData[3] = scaled_Volt_LowByte;		// voltage low byte
    	TxData[4] = 0x00;
    	TxData[5] = min_current;
    	printf("executing Case_1 as SoC = %d\n", bmsResponse[23]);
    }

    switch (bmsResponse[23]<90){
    case 1:
    	TxData[0] = broadcastAdd;
    	TxData[1] = relayPowerON;
    	TxData[2] = scaled_Volt_HighByte;		// voltage high byte
    	TxData[3] = scaled_Volt_LowByte;		// voltage low byte
    	TxData[4] = 0x00;
    	TxData[5] = max_current;
    	printf("executing Case_2 as SoC = %d\n", bmsResponse[23]);
    }
       // Transmit CAN frame
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) == HAL_OK) {
            // Handle possible error
            // Error handling code here
        	printf("Data sent over CAN successfully.\n");
        }
}

/*void checkCANTransmission(void) {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        // Compare received data with expected data
        if (RxData[0] == bmsResponse[4] && RxData[1] == bmsResponse[5] &&
            RxData[2] == bmsResponse[6] && RxData[3] == bmsResponse[7] &&
            RxData[4] == bmsResponse[23] && RxData[5] == 0 && RxData[6] == 0 && RxData[7] == 0) {
            // Data is correct
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Set a success LED or similar indicator
        	printf("Data sent over CAN successfully.\n");
        } else {
            // Data is incorrect
        	printf("Data sent over CAN successfully.\n");

        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Set a failure indicator
        }
    }
}*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
        dataReady = 1; // Data is ready
        lastDataTime = HAL_GetTick(); // Update last data time
    }
}

void CheckForTimeout() {
    if (HAL_GetTick() - lastDataTime >= TIMEOUT) {

        HAL_UART_DMAStop(&huart1);
        __HAL_UART_FLUSH_DRREGISTER(&huart1);
        HAL_UART_Receive_DMA(&huart1, bmsResponse, sizeof(bmsResponse));
        lastDataTime = HAL_GetTick(); // Reset the timer
    }
}


void FDCAN1_FilterConfig(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x200; // First ID of the range
//    sFilterConfig.FilterID2 = 0x110; // Last ID of the range

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        /* Filter configuration Error */
        Error_Handler();
    }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 8;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 7;
  hfdcan1.Init.NominalTimeSeg2 = 8;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  // Start FDCAN1
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
      Error_Handler();
  }
  /* USER CODE END FDCAN1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
