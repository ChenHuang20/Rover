/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file can1.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 *
 ***************************************************************************/

#include "can1.h"

#include "scheduler.h"
#include "stm32f4xx_hal.h"

enum state_e {
    STATE_NULL = 0,
    STATE_UPDATED,
    STATE_BUSY
};

struct {
    int16_t data[4];
    enum state_e state;
} _can1 = {
    .state = STATE_NULL
};

CAN_HandleTypeDef hcan1;

static CanTxMsgTypeDef Tx1Message;
static CanRxMsgTypeDef Rx1Message;

void can1_config()
{
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;

    // PD0     ------> CAN1_RX
    // PD1     ------> CAN1_TX 
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;

    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // CAN1 interrupt Init
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 5;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SJW = CAN_SJW_1TQ;
    hcan1.Init.BS1 = CAN_BS1_3TQ;
    hcan1.Init.BS2 = CAN_BS2_5TQ;
    hcan1.Init.TTCM = DISABLE;
    hcan1.Init.ABOM = ENABLE;
    hcan1.Init.AWUM = DISABLE;
    hcan1.Init.NART = DISABLE;
    hcan1.Init.RFLM = DISABLE;
    hcan1.Init.TXFP = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    //can1 &can2 use same filter config
    CAN_FilterConfTypeDef  CAN_FilterConfigStructure;

    CAN_FilterConfigStructure.FilterNumber         = 0;
    CAN_FilterConfigStructure.FilterMode           = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale          = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh         = 0x0000;
    CAN_FilterConfigStructure.FilterIdLow          = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh     = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow      = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.BankNumber           = 14; //can1(0-13)and can2(14-27)each get half filter
    CAN_FilterConfigStructure.FilterActivation     = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure) != HAL_OK) {
    }

    hcan1.pTxMsg = &Tx1Message;
    hcan1.pRxMsg = &Rx1Message;
    HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    __HAL_RCC_CAN1_CLK_ENABLE();
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
    if (hcan->Instance == CAN1) {
        __HAL_RCC_CAN1_CLK_ENABLE();
    }
}

void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);

    if (_can1.state != STATE_BUSY) {
        _can1.state = STATE_BUSY;

        switch (hcan1.pRxMsg->StdId) {
            case 0x201:
                _can1.data[0] = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
                break;

			case 0x202:
                _can1.data[1] = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
                break;

			case 0x203:
                _can1.data[2] = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
                break;

			case 0x204:
                _can1.data[3] = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
                break;

			default:
                break;
        }

        _can1.state = STATE_UPDATED;
    }

    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
}

int can1_read(uint8_t *buf, int len)
{
    if (_can1.state == STATE_UPDATED) {
        _can1.state = STATE_BUSY;

        buf[0] = _can1.data[0] >> 8;
        buf[1] = _can1.data[0];
        buf[2] = _can1.data[1] >> 8;
        buf[3] = _can1.data[1];
        buf[4] = _can1.data[2] >> 8;
        buf[5] = _can1.data[2];
        buf[6] = _can1.data[3] >> 8;
        buf[7] = _can1.data[3];

        _can1.state = STATE_NULL;

        return 0;

    } else {
        return -1;
    }
}

int can1_write(uint8_t *buf, int len)
{
    uint32_t std_id = (uint32_t)buf[0] << 24 | (uint32_t)buf[1] << 16 | (uint32_t)buf[2] << 8 | (uint32_t)buf[3];

	hcan1.pTxMsg->StdId   = std_id;
	hcan1.pTxMsg->IDE     = CAN_ID_STD;
	hcan1.pTxMsg->RTR     = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC     = 0x08;
	hcan1.pTxMsg->Data[0] = buf[4];
	hcan1.pTxMsg->Data[1] = buf[5];
	hcan1.pTxMsg->Data[2] = buf[6];
	hcan1.pTxMsg->Data[3] = buf[7];
	hcan1.pTxMsg->Data[4] = buf[8];
	hcan1.pTxMsg->Data[5] = buf[9];
	hcan1.pTxMsg->Data[6] = buf[10];
	hcan1.pTxMsg->Data[7] = buf[11];

    HAL_CAN_Transmit(&hcan1, 1000);

    return 0;
}
