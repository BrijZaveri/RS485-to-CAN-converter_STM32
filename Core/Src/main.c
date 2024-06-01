#include "main.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// Buffers
char v_i_Str[BUFFER_SIZE];
uint8_t TxData[TX_DATA_SIZE] = {0};

// BMS RS-485 commands
const uint8_t hostCommand_1[COMMAND_SIZE] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
const uint8_t hostCommand_2[COMMAND_SIZE] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};

// BMS response buffers
uint8_t bmsResponse_1[BMS_RESPONSE_SIZE];
uint8_t bmsResponse_2[BMS_RESPONSE_SIZE];

// Cell data
uint16_t Cell_String[CELL_STRING_SIZE];
uint16_t voltage, current, SoC, TotalCells;
uint16_t Cell_temp[CELL_TEMP_SIZE];

// Flags and counters
volatile uint8_t dataReady[2] = {0};
volatile uint8_t hostCommand = 1;
uint32_t lastDataTime = 0;
volatile bool canConnected = true;
volatile uint8_t timeoutCount = 0;
volatile bool timeoutResolved = false;
volatile bool handleChargeVoltage = false;

// System states
typedef enum {
    STATE_WAIT_RESPONSE,
    STATE_PROCESS_RESPONSE,
    STATE_TIMEOUT,
    STATE_CHARGE_VOLTAGE
} SystemState;

SystemState currentState = STATE_WAIT_RESPONSE;

// Printf functionality
int _write(int file, char *ptr, int len) {
    if (HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY) == HAL_OK) {
        return len;
    }
    return 0;
}

// Start UART reception
void startReception(void) {
    if (hostCommand == 1) {
        HAL_UART_Receive_DMA(&huart1, bmsResponse_1, sizeof(bmsResponse_1));
    } else {
        HAL_UART_Receive_DMA(&huart1, bmsResponse_2, sizeof(bmsResponse_2));
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_FDCAN1_Init();
    FDCAN1_FilterConfig();
    MX_USART1_UART_Init();

    startReception();
    lastDataTime = HAL_GetTick();

    while (1) {
        transmitBMSCommand();
        handleStateMachine();
        HAL_Delay(DELAY_MS);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
//    	if (hostCommand == 1) {
//    			dataReady[0] = 1;  //Data is ready
//    		} else {
//    			dataReady[1] = 1;  //Data is ready
//    		}
        dataReady[hostCommand == 1 ? 0 : 1] = 1;			//Data is ready
		lastDataTime = HAL_GetTick(); 						// Update last data time
        canConnected = true;  								// Set CAN connection flag to true on data reception
        timeoutCount = 0;  									// Reset the timeout counter
    }
}

// Handle state machine
void handleStateMachine(void) {

    switch (currentState) {
        case STATE_WAIT_RESPONSE:
    		if ((dataReady[0] || dataReady[1]) && (HAL_GetTick() - lastDataTime < TIMEOUT_MS)) {
                currentState = STATE_PROCESS_RESPONSE;
            } else {
                currentState = STATE_TIMEOUT;
            }
            break;

        case STATE_PROCESS_RESPONSE:
            transmitDataOverUSART2();
            transmitDataOverCAN();
            memset(v_i_Str, 0, sizeof(v_i_Str));

            dataReady[0] = 0;
            dataReady[1] = 0;
            hostCommand = (hostCommand == 1) ? 2 : 1;  				// Toggle between commands
            startReception();
            lastDataTime = HAL_GetTick();

            currentState = STATE_WAIT_RESPONSE;
            break;

        case STATE_TIMEOUT:
            timeoutCount++;
            if (timeoutCount >= MAX_TIMEOUT_COUNT) {
                handleChargeVoltage = true;
            }
            chargeVoltageBased();

            HAL_UART_DMAStop(&huart1);
            __HAL_UART_FLUSH_DRREGISTER(&huart1);
            startReception();
            lastDataTime = HAL_GetTick();

            currentState = STATE_WAIT_RESPONSE;
            break;

        case STATE_CHARGE_VOLTAGE:
            chargeVoltageBased();
            handleChargeVoltage = false;
            currentState = STATE_WAIT_RESPONSE;
            break;

        default:
            currentState = STATE_WAIT_RESPONSE;
            break;
    }
}
