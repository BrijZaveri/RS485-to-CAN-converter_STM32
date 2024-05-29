/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

// Constants
#define BMS_RESPONSE_SIZE 70
#define Cell_String_SIZE 70
#define Cell_TEMP_SIZE 11
#define TEMP_OFFSET 2731
#define NUM_CELL_TEMPS 5
#define NUM_CELL_STRINGS 23
#define UART_TIMEOUT 1000

// Define constants for maximum output voltages
#define MAX_VOLTAGE_23S 8400
#define MAX_VOLTAGE_19S 6935
#define MAX_VOLTAGE_16S 5840
#define MAX_VOLTAGE_15S 5475

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "config.h"
#include <charger_can_comm.h>
#include <batt_rs485_comm.h>
#include <stdbool.h>

// Global variables
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern uint8_t hostCommand_1[7];
extern uint8_t hostCommand_2[7];
extern uint8_t bmsResponse_1[];
extern uint8_t bmsResponse_2[];
extern volatile uint8_t hostCommand;
extern char v_i_Str[50];
extern uint16_t Cell_String[Cell_String_SIZE];
extern uint16_t voltage,current,SoC,TotalCells;
extern uint16_t Cell_temp[];
extern volatile uint8_t dataReady[2];
extern uint32_t lastDataTime;
extern const uint32_t TIMEOUT;
extern uint8_t TxData[8];
extern volatile uint8_t canConnected;
// Function prototypes
void SystemClock_Config(void);
void CheckForTimeout(void);
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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
