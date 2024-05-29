#include "main.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// BMS RS-485 matrix commands
uint8_t hostCommand_1[7] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
uint8_t hostCommand_2[7] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};
uint8_t bmsResponse_1[BMS_RESPONSE_SIZE];		//buffer to store BMS response we get by sending the values defined in "hostCommand_11" variable
uint8_t bmsResponse_2[BMS_RESPONSE_SIZE];		//buffer to store BMS response we get by sending the values defined in "hostCommand_2" variable
uint16_t Cell_String[Cell_String_SIZE];
char v_i_Str[50];				//buffer to store Voltage, current, SoC, No.of cells values
uint16_t voltage,current,SoC,TotalCells;
uint16_t Cell_temp[Cell_TEMP_SIZE];
//current limit for charging and discharging
uint8_t min_current = 0x32;		//50*0.1 = 5amps
uint8_t max_current = 0xA0;		//160*0.1 = 16 amps
volatile uint8_t dataReady[2];		//dataready flaTotalCellsg
volatile uint8_t hostCommand = 1; // Flag to indicate current command (1 or 2)
uint32_t lastDataTime = 0;			// capture time of last data received
const uint32_t TIMEOUT = 1000;  		// 0.5 seconds timeout
uint8_t TxData[8] = {0};  			// Buffer to store CAN data payload of 8 bytes
volatile uint8_t canConnected = 0;  // Flag to indicate CAN connection status


//**************************Printf functionality********************
int _write(int file, char *ptr, int len) {
    if (HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY) == HAL_OK) {
        return len;
    }
    return 0;
}
//**************************Printf functionality********************

void startReception(void)
{
//    memset(bmsResponse_1, 0, sizeof(bmsResponse_1)); // Clear b/
//    memset(bmsResponse_2, 0, sizeof(bmsResponse_2)); // Clear b/
    if (hostCommand == 1) {
        HAL_UART_Receive_DMA(&huart1, bmsResponse_1, sizeof(bmsResponse_1));
    } else {
        HAL_UART_Receive_DMA(&huart1, bmsResponse_2, sizeof(bmsResponse_2));
    }
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

  startReception();
  lastDataTime = HAL_GetTick(); // Initialize the last data time

  while (1)
  {
	  transmitBMSCommand();

	      if ((dataReady[0] || dataReady[1]) && (HAL_GetTick() - lastDataTime < TIMEOUT)) {
	        transmitDataOverUSART2();
	        transmitDataOverCAN();
	        memset(v_i_Str, 0, sizeof(v_i_Str)); // Clear buffer


	        dataReady[0] = 0;		//Clear the flag
	        dataReady[1] = 0;		//Clear the flag

	        hostCommand = (hostCommand == 1) ? 2 : 1;	        // Toggle between commands
	      } else {
	        CheckForTimeout();
	      }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    if (hostCommand == 1) {
      dataReady[0] = 1;  //Data is ready
    } else {
    	dataReady[1] = 1;  //Data is ready
    }
    lastDataTime = HAL_GetTick(); // Update last data time
  }
}

void CheckForTimeout() {
  if (HAL_GetTick() - lastDataTime >= TIMEOUT) {
    HAL_UART_DMAStop(&huart1);
    __HAL_UART_FLUSH_DRREGISTER(&huart1);
    HAL_UART_Receive_DMA(&huart1, hostCommand == 1 ? bmsResponse_1 : bmsResponse_2, sizeof(bmsResponse_1));
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
