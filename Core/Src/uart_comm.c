/*
 * uart_comm.c
 *
 *  Created on: May 24, 2024
 *      Author: Brij Zaveri
 */

#include "main.h"
#include "uart_comm.h"

void transmitBMSCommand(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, hostCommand_1, HOST_COMMAND_1_SIZE, 10); // Correct size calculation
//    HAL_UART_Transmit(&huart1, hostCommand_1, sizeof(hostCommand_1), 10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(500);
}

void transmitDataOverUSART2(void)
{
    int len;
    uint16_t voltage = (uint16_t)bmsResponse[4] << 8 | bmsResponse[5];
    len = snprintf(v_i_Str, sizeof(v_i_Str), "Voltage: %u\r\n", voltage);
    HAL_UART_Transmit(&huart2, (uint8_t*)v_i_Str, len, 1000);

    uint16_t current = (uint16_t)bmsResponse[6] << 8 | bmsResponse[7];
    len = snprintf(v_i_Str, sizeof(v_i_Str), "Current: %u\r\n", current);
    HAL_UART_Transmit(&huart2, (uint8_t*)v_i_Str, len, 1000);

    uint16_t SoC = (uint16_t)bmsResponse[23];
    len = snprintf(v_i_Str, sizeof(v_i_Str), "SOC: %u\r\n", SoC);
    HAL_UART_Transmit(&huart2, (uint8_t*)v_i_Str, len, 1000);

    uint16_t TotalCells = (uint16_t)bmsResponse[25];
    len = snprintf(v_i_Str, sizeof(v_i_Str), "No. of Cells: %u\r\n", TotalCells);
    HAL_UART_Transmit(&huart2, (uint8_t*)v_i_Str, len, 1000);
}
