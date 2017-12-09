/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file can2.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 *
 ***************************************************************************/

#include "can2.h"
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
} _can2 = {
    .state = STATE_NULL
};

CAN_HandleTypeDef hcan2;

static CanTxMsgTypeDef Tx2Message;
static CanRxMsgTypeDef Rx2Message;

void can2_config()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;

    //PB12     ------> CAN2_RX
    //PB13     ------> CAN2_TX 

    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // CAN1 interrupt Init
    hcan2.Instance = CAN2;
    hcan2.Init.Prescaler = 5;
    hcan2.Init.Mode = CAN_MODE_NORMAL;
    hcan2.Init.SJW = CAN_SJW_4TQ;
    hcan2.Init.BS1 = CAN_BS1_3TQ;
    hcan2.Init.BS2 = CAN_BS2_5TQ;
    hcan2.Init.TTCM = DISABLE;
    hcan2.Init.ABOM = ENABLE;
    hcan2.Init.AWUM = DISABLE;
    hcan2.Init.NART = DISABLE;
    hcan2.Init.RFLM = DISABLE;
    hcan2.Init.TXFP = DISABLE;
    if (HAL_CAN_Init(&hcan2) != HAL_OK)
    {
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

    CAN_FilterConfigStructure.FilterNumber = 14;
    if (HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfigStructure) != HAL_OK) {
    }

    hcan2.pTxMsg = &Tx2Message;
    hcan2.pRxMsg = &Rx2Message;
    HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);

    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);

    __HAL_RCC_CAN2_CLK_ENABLE();
}


void CAN2_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan2);

    if (_can2.state != STATE_BUSY) {
        _can2.state = STATE_BUSY;

        switch (hcan2.pRxMsg->StdId) {
            case 0x206:
                _can2.data[0] = (int16_t)(hcan2.pRxMsg->Data[2]<<8 | hcan2.pRxMsg->Data[3]);
                break;
			default:
                break;
        }

        _can2.state = STATE_UPDATED;
    }

    __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
}

int can2_read(uint8_t *buf, int len)
{
    if (_can2.state == STATE_UPDATED) {
        _can2.state = STATE_BUSY;

        buf[0] = _can2.data[0] >> 8;
        buf[1] = _can2.data[0];
        buf[2] = _can2.data[1] >> 8;
        buf[3] = _can2.data[1];
        buf[4] = _can2.data[2] >> 8;
        buf[5] = _can2.data[2];
        buf[6] = _can2.data[3] >> 8;
        buf[7] = _can2.data[3];

        _can2.state = STATE_NULL;

        return 0;

    } else {
        return -1;
    }
}

int can2_write(uint8_t *buf, int len)
{
    uint32_t std_id = (uint32_t)buf[0] << 24 | (uint32_t)buf[1] << 16 | (uint32_t)buf[2] << 8 | (uint32_t)buf[3];

	hcan2.pTxMsg->StdId   = std_id;
	hcan2.pTxMsg->IDE     = CAN_ID_STD;
	hcan2.pTxMsg->RTR     = CAN_RTR_DATA;
	hcan2.pTxMsg->DLC     = 0x08;
	hcan2.pTxMsg->Data[0] = buf[4];
	hcan2.pTxMsg->Data[1] = buf[5];
	hcan2.pTxMsg->Data[2] = buf[6];
	hcan2.pTxMsg->Data[3] = buf[7];
	hcan2.pTxMsg->Data[4] = buf[8];
	hcan2.pTxMsg->Data[5] = buf[9];
	hcan2.pTxMsg->Data[6] = buf[10];
	hcan2.pTxMsg->Data[7] = buf[11];

    HAL_CAN_Transmit(&hcan2, 1000);

    return 0;
}




