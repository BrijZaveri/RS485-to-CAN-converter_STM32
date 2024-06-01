/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * COPYRIGHT (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "config.h"
#include <stdio.h>
#include <charger_can_comm.h>
#include <batt_rs485_comm.h>
#include <stdbool.h>

// Constants
#define BUFFER_SIZE 50
#define TX_DATA_SIZE 8
#define COMMAND_SIZE 7
#define BMS_RESPONSE_SIZE 70
#define CELL_STRING_SIZE 70
#define CELL_TEMP_SIZE 11
#define TEMP_OFFSET 2731
#define NUM_CELL_TEMPS 5
#define NUM_CELL_STRINGS 23
#define UART_TIMEOUT 1000
#define TIMEOUT_MS 3000
#define MAX_TIMEOUT_COUNT 3
#define TIMEOUT_INIT 1000  // Timeout for initial response in milliseconds
#define MAX_TIMEOUT_COUNT 3
#define DELAY_MS 100
#define CELL_STRING_SIZE 70
#define CELL_TEMP_SIZE 11

// Maximum output voltages
#define MAX_VOLTAGE_23S 8400
#define MAX_VOLTAGE_19S 6935
#define MAX_VOLTAGE_16S 5840
#define MAX_VOLTAGE_15S 5475

// Global variables
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

// Buffers
extern char v_i_Str[BUFFER_SIZE];
extern uint8_t TxData[TX_DATA_SIZE];
extern const uint8_t hostCommand_1[COMMAND_SIZE];
extern const uint8_t hostCommand_2[COMMAND_SIZE];
extern uint8_t bmsResponse_1[BMS_RESPONSE_SIZE];
extern uint8_t bmsResponse_2[BMS_RESPONSE_SIZE];
extern uint16_t Cell_String[CELL_STRING_SIZE];
extern uint16_t voltage, current, SoC, TotalCells;
extern uint16_t Cell_temp[CELL_TEMP_SIZE];

// Flags and counters
extern volatile uint8_t dataReady[2];
extern volatile uint8_t hostCommand;
extern uint32_t lastDataTime;
extern volatile bool canConnected;
extern volatile uint8_t timeoutCount;
extern volatile bool timeoutResolved;
extern volatile bool handleChargeVoltage;

// Charger logic variables
extern uint8_t broadcastAdd;
extern uint8_t relayPowerON;
extern uint8_t max_current;
extern uint8_t min_current;
extern uint8_t currentForLessThan3V;
extern uint8_t derated_current;

// Function prototypes
void SystemClock_Config(void);
void startReception(void);
void handleStateMachine(void);
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
