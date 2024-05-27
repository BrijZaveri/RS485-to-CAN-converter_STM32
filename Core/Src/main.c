#include "main.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// BMS RS-485 matrix commands
uint8_t hostCommand_1[HOST_COMMAND_1_SIZE] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
uint8_t hostCommand_2[HOST_COMMAND_2_SIZE] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};

uint8_t bmsResponse[65];		//buffer to store BMS response we get by sending the values defined in "hostCommand" variable
char v_i_Str[50];		//buffer to store Voltage, current, SoC, No.of cells values

//current limit for charging and discharging
uint8_t min_current = 0x32;		//50*0.1 = 5amps
uint8_t max_current = 0xA0;		//160*0.1 = 16 amps

//GLobal variables
volatile uint8_t dataReady = 0;		//dataready flag
uint32_t lastDataTime = 0;			// capture time of last data received
const uint32_t TIMEOUT = 600;  		// 0.6 seconds timeout
uint8_t TxData[8] = {0};  			// Buffer to store CAN data payload of 8 bytes


//**************************Printf functionality********************
int _write(int file, char *ptr, int len) {
    if (HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY) == HAL_OK) {
        return len;
    }
    return 0;
}
//**************************Printf functionality********************
<<<<<<< HEAD
=======


>>>>>>> origin/main

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
  HAL_UART_Receive_DMA(&huart1, bmsResponse, sizeof(bmsResponse));			//from CAN-transciever's UART terminals
  lastDataTime = HAL_GetTick(); // Initialize the last data time

  while (1)
  {
	  transmitBMSCommand();

<<<<<<< HEAD
		 if (dataReady && (HAL_GetTick() - lastDataTime < TIMEOUT)) {
			 transmitDataOverUSART2();
			 transmitDataOverCAN();
			 memset(v_i_Str, 0, sizeof(v_i_Str)); // Clear buffer
			 dataReady = 0; // Clear the flag
=======
	         if (dataReady && (HAL_GetTick() - lastDataTime < TIMEOUT)) {
	             transmitDataOverUSART2();
	             transmitDataOverCAN();
	             memset(v_i_Str, 0, sizeof(v_i_Str)); // Clear buffer
	             dataReady = 0; // Clear the flag
>>>>>>> origin/main

		 }
		 else {
			 CheckForTimeout();
		 }
  }
}

<<<<<<< HEAD
=======
void transmitBMSCommand(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);			//LED indication HIgh
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);			//DE pin set High
	HAL_UART_Transmit(&huart1, hostCommand, sizeof(hostCommand),10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);		//LED indication LOW
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);		//DE pin set Low
	HAL_Delay(500);
}

void transmitDataOverUSART2(void) {

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

void transmitDataOverCAN(void) {

    int totalCellVolt = (bmsResponse[25] * 3.65) * 100;
    uint16_t scaled_Volt_Value = totalCellVolt / 10;
    uint8_t scaled_Volt_HighByte = (scaled_Volt_Value >> 8) & 0xFF;  // High byte
    uint8_t scaled_Volt_LowByte = scaled_Volt_Value & 0xFF;
    uint32_t msgId = 0x200;

    uint8_t broadcastAdd = 0xFF;
    uint8_t relayPowerON = 3;
    uint8_t TxData[8];
    uint8_t min_current = 0x10;  // Assuming min_current is defined somewhere
    uint8_t max_current = 0x20;  // Assuming max_current is defined somewhere

    // Prepare CAN header
    TxHeader.Identifier = msgId;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // Check SoC (State of Charge) and set TxData accordingly
    if (bmsResponse[23] > 90) { // when SoC is greater than 90%
        TxData[0] = broadcastAdd;
        TxData[1] = relayPowerON;
        TxData[2] = scaled_Volt_HighByte; // voltage high byte
        TxData[3] = scaled_Volt_LowByte;  // voltage low byte
        TxData[4] = 0x00;
        TxData[5] = min_current;
        printf("Executing Case_1 as SoC = %d\n", bmsResponse[23]);
    } else { // when SoC is less than or equal to 90%
        TxData[0] = broadcastAdd;
        TxData[1] = relayPowerON;
        TxData[2] = scaled_Volt_HighByte; // voltage high byte
        TxData[3] = scaled_Volt_LowByte;  // voltage low byte
        TxData[4] = 0x00;
        TxData[5] = max_current;
        printf("Executing Case_2 as SoC = %d\n", bmsResponse[23]);
    }

    // Transmit CAN frame
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) == HAL_OK) {
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

>>>>>>> origin/main
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
        dataReady = 1;                 // Data is ready
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
