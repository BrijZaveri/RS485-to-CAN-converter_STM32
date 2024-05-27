/*
 * config.h
 *
 *  Created on: May 24, 2024
 *      Author: Brij Zaveri
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "stm32g0xx_hal.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_TxHeaderTypeDef TxHeader;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

/* Private function prototypes -----------------------------------------------*/

//void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_FDCAN1_Init(void);
void FDCAN1_FilterConfig(void);

#endif /* INC_CONFIG_H_ */
