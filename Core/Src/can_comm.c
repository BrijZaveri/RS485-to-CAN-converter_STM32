/*
 * can_comm.c
 *
 *  Created on: May 24, 2024
 *      Author: Brij Zaveri
 */


#include "can_comm.h"
#include "main.h"

void transmitDataOverCAN(void)
{
    int totalCellVolt = (bmsResponse[25] * 3.65) * 100;
    uint16_t scaled_Volt_Value = totalCellVolt / 10;
    uint8_t scaled_Volt_HighByte = (scaled_Volt_Value >> 8) & 0xFF;
    uint8_t scaled_Volt_LowByte = scaled_Volt_Value & 0xFF;
    uint32_t msgId = 0x200;

    uint8_t broadcastAdd = 0xFF;
    uint8_t relayPowerON = 3;
    uint8_t min_current = 0x10;
    uint8_t max_current = 0x20;

    TxHeader.Identifier = msgId;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (bmsResponse[23] > 90)
    {
        TxData[0] = broadcastAdd;
        TxData[1] = relayPowerON;
        TxData[2] = scaled_Volt_HighByte;
        TxData[3] = scaled_Volt_LowByte;
        TxData[4] = 0x00;
        TxData[5] = min_current;
        printf("Executing Case_1 as SoC = %d\r\n", bmsResponse[23]);
    }
    else
    {
        TxData[0] = broadcastAdd;
        TxData[1] = relayPowerON;
        TxData[2] = scaled_Volt_HighByte;
        TxData[3] = scaled_Volt_LowByte;
        TxData[4] = 0x00;
        TxData[5] = max_current;
        printf("Executing Case_2 as SoC = %d\r", bmsResponse[23]);
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) == HAL_OK)
    {
        printf("Data sent over CAN successfully.\r\n");
        printf("\n");

    }
}
