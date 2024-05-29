/*
 * uart_comm.c
 *
 *  Created on: May 24, 2024
 *      Author: Brij Zaveri
 */
/* this part of code will the send query command to battery via rs485 using USART_1 pins and store the response in bmsResponse_1 buffer*/
/* MCU(rs485) <-> Battery(rs485) >> */

#include <batt_rs485_comm.h>
#include "main.h"

void transmitBMSCommand(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	if (hostCommand == 1) {
		HAL_UART_Transmit(&huart1, hostCommand_1, sizeof(hostCommand_1), HAL_MAX_DELAY);
	} else {
		HAL_UART_Transmit(&huart1, hostCommand_2, sizeof(hostCommand_2), HAL_MAX_DELAY);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_Delay(200);
}

void transmitData(const char* label, uint16_t value, UART_HandleTypeDef* huart)
{
    int len = snprintf(v_i_Str, sizeof(v_i_Str), "%s: %u\r\n", label, value);
    if (len > 0 && len < sizeof(v_i_Str)) {
        HAL_UART_Transmit(huart, (uint8_t*)v_i_Str, len, UART_TIMEOUT);
    } else {
        // Handle snprintf error if needed
    }
}

//The bmsResponse_1 array contains the response data from the BMS.
//When you call transmitBMSCellTemps(bmsResponse_1, &huart2);, bmsResponse_1 is passed to the response parameter of the function.
//Inside the function, response acts as an alias for bmsResponse_1.
//Any manipulation of response inside the function is actually manipulating the data in bmsResponse_1.

void transmitBMSCellTemps(uint8_t* response, UART_HandleTypeDef* huart)
{
    for (uint8_t i = 0; i < NUM_CELL_TEMPS; ++i) {
        uint16_t cell_temp = (response[27 + 2 * i] << 8) | response[28 + 2 * i];
        Cell_temp[i] = cell_temp - TEMP_OFFSET;
        char label[20];
        snprintf(label, sizeof(label), "Cell_temp_%d", i + 1);
        transmitData(label, Cell_temp[i], huart);
    }
}

void transmitBMSCellStrings(uint8_t* response, UART_HandleTypeDef* huart)
{
    for (uint8_t i = 0; i < NUM_CELL_STRINGS; ++i) {
        uint16_t cell_string = (response[4 + 2 * i] << 8) | response[5 + 2 * i];
        Cell_String[i] = cell_string;
        char label[20];
        snprintf(label, sizeof(label), "Cell_String_%d", i + 1);
        transmitData(label, Cell_String[i], huart);
    }
}

void transmitDataOverUSART2(void)
{
    voltage = bmsResponse_1[4] << 8 | bmsResponse_1[5];
    transmitData("Voltage", voltage, &huart2);

    current = bmsResponse_1[6] << 8 | bmsResponse_1[7];
    transmitData("Current", current, &huart2);

    SoC = bmsResponse_1[23];
    transmitData("SOC", SoC, &huart2);

    TotalCells = bmsResponse_1[25];
    transmitData("No. of Cells", TotalCells, &huart2);

    transmitBMSCellTemps(bmsResponse_1, &huart2);

//    if (hostCommand == 2) {
        transmitBMSCellStrings(bmsResponse_2, &huart2);
//    }
}
