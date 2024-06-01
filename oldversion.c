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

volatile uint8_t dataReady[2];		//dataready flaTotalCellsg
volatile uint8_t hostCommand = 1; // Flag to indicate current command (1 or 2)
uint32_t lastDataTime = 0;			// capture time of last data received
const uint32_t TIMEOUT = 5000;  		// 5 seconds timeout
uint8_t TxData[8] = {0};  			// Buffer to store CAN data payload of 8 bytes
volatile bool canConnected = true;  // Flag to indicate CAN connection status
volatile uint8_t timeoutCount = 0; // Counter for consecutive timeouts
volatile bool timeoutResolved = false;  // Flag to indicate if the timeout was resolved
uint32_t timeoutStartTime = 0;  // Start time for timeout check
volatile bool handleChargeVoltage = false;  // Flag to indicate if chargeVoltageBased should be called

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
		}
		else {
			CheckForTimeout();
		}
		handleTimeout();  // Check and handle the timeout in each iteration
		if (handleChargeVoltage) {
			for (int i = 0; i < 5; i++) {
				chargeVoltageBased();
			}
			handleChargeVoltage = false;  // Reset the flag after handling charge voltage
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
        canConnected = true;  // Set CAN connection flag to true on data reception
        timeoutCount = 0;  // Reset the timeout counter
        timeoutResolved = true;  // Mark timeout as resolved
	}
}

void CheckForTimeout() {
	if (HAL_GetTick() - lastDataTime >= TIMEOUT) {
		canConnected = false;          // Set CAN connection flag to false if timeout occurs
		HAL_UART_DMAStop(&huart1);
		__HAL_UART_FLUSH_DRREGISTER(&huart1);
		HAL_UART_Receive_DMA(&huart1, hostCommand == 1 ? bmsResponse_1 : bmsResponse_2, sizeof(bmsResponse_1));
		lastDataTime = HAL_GetTick(); // Reset the timer
		timeoutStartTime = HAL_GetTick();  // Start the timeout handling period
		timeoutResolved = false;  // Reset timeout resolved flag
	}
}

void handleTimeout() {
	if (!timeoutResolved && (HAL_GetTick() - timeoutStartTime >= 5000)) {
		// If after 5 seconds canConnected is still false, execute chargeVoltageBased
		for (int i = 0; i < 5; i++) {
			chargeVoltageBased();
		}
		timeoutResolved = true;  // Prevent further execution of chargeVoltageBased
	}
}