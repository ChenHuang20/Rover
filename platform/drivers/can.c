/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file can1.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089@qq.com>
 *
 ***************************************************************************/

#include "can.h"
#include "topics.h"
#include "stm32f4xx_hal.h"
#include "scheduler.h"

enum state_e {
    STATE_NULL = 0,
    STATE_UPDATED,
    STATE_BUSY
};

struct {
    int16_t data[10];
    enum state_e state;
} _can1 = {
    .state = STATE_NULL
};

struct {
    int16_t data[4];
    enum state_e state;
} _can2 = {
    .state = STATE_NULL
};

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void can_filter_config(CAN_HandleTypeDef* _hcan)
{
    //can1 &can2 use same filter config
    CAN_FilterConfTypeDef  CAN_FilterConfigStructure;
    static CanTxMsgTypeDef Tx1Message;
    static CanRxMsgTypeDef Rx1Message;
    static CanTxMsgTypeDef Tx2Message;
    static CanRxMsgTypeDef Rx2Message;

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

    if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK) {
    }
    //filter config for can2
    //can1(0-13),can2(14-27)
    CAN_FilterConfigStructure.FilterNumber = 14;
    if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK) {
    }

    if (_hcan == &hcan1) {
        _hcan->pTxMsg = &Tx1Message;
        _hcan->pRxMsg = &Rx1Message;
		HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
    }

    if (_hcan == &hcan2){
        _hcan->pTxMsg = &Tx2Message;
        _hcan->pRxMsg = &Rx2Message;
		HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
	}
}

void can1_config()
{
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;

		//427:
    // PD0     ------> CAN1_RX
    // PD1     ------> CAN1_TX
		//405:
		// PA11    ------> CAN1_RX
		// PA12    ------> CAN1_TX
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

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    __HAL_RCC_CAN1_CLK_ENABLE();
    
    can_filter_config(&hcan1);
}

void can2_config()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOB_CLK_ENABLE();

		//427&405:
    //PB12     ------> CAN2_RX
    //PB13     ------> CAN2_TX 
	
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // CAN2 interrupt Init
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
    
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);

    __HAL_RCC_CAN2_CLK_ENABLE();

    can_filter_config(&hcan2);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
    if(canHandle->Instance==CAN1) {
        //Peripheral clock enable
        HAL_RCC_CAN1_CLK_ENABLED++;
        if(HAL_RCC_CAN1_CLK_ENABLED==1) {
            __HAL_RCC_CAN1_CLK_ENABLE();
        }
    }
    else if(canHandle->Instance==CAN2) {
        //Peripheral clock enable
        __HAL_RCC_CAN2_CLK_ENABLE();
        HAL_RCC_CAN1_CLK_ENABLED++;
        if(HAL_RCC_CAN1_CLK_ENABLED==1) {
            __HAL_RCC_CAN1_CLK_ENABLE();
        }
    }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

    if(canHandle->Instance==CAN1) {
        //Peripheral clock disable
        HAL_RCC_CAN1_CLK_ENABLED--;
        if(HAL_RCC_CAN1_CLK_ENABLED==0) {
          __HAL_RCC_CAN1_CLK_DISABLE();
        }

    //PD0     ------> CAN1_RX
    //PD1     ------> CAN1_TX 
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    //Peripheral interrupt Deinit
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);

    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    }
    else if(canHandle->Instance==CAN2) {
        // Peripheral clock disable
        __HAL_RCC_CAN2_CLK_DISABLE();
        HAL_RCC_CAN1_CLK_ENABLED--;
        if(HAL_RCC_CAN1_CLK_ENABLED==0) {
            __HAL_RCC_CAN1_CLK_DISABLE();
        }
        
    //PB12     ------> CAN2_RX
    //PB13     ------> CAN2_TX 
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    //Peripheral interrupt Deinit
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);

    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  }
}

void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);

    if (_can1.state != STATE_BUSY) {
        _can1.state = STATE_BUSY;
        //0x201~0x204反馈速度   0x205~0x206反馈机械角度
        switch (hcan1.pRxMsg->StdId) {
      case 0x201:
                _can1.data[0] = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
                _can1.data[6] = (int16_t)(hcan1.pRxMsg->Data[4]<<8 | hcan1.pRxMsg->Data[5]);

                if(_motor_trigger.msg_cnt++ <= 10)
                {
                _motor_trigger.angle = (uint16_t)(hcan1.pRxMsg->Data[0] << 8 | hcan1.pRxMsg->Data[1]);
                _motor_trigger.offset_angle = _motor_trigger.angle;
                }
                else //+-4096
                {
                _motor_trigger.speed_rpm  = (int16_t)(hcan1.pRxMsg->Data[2] << 8 | hcan1.pRxMsg->Data[3]);
                _tri_rates.trigger = _motor_trigger.speed_rpm / 60 / 96;//(rad/s)
                _motor_trigger.given_current = (int16_t)(hcan1.pRxMsg->Data[4] << 8 | hcan1.pRxMsg->Data[5]) / -5;
                _motor_trigger.hall = hcan1.pRxMsg->Data[6];
                _motor_trigger.last_angle = _motor_trigger.angle;
                _motor_trigger.angle = (uint16_t)(hcan1.pRxMsg->Data[0] << 8 | hcan1.pRxMsg->Data[1]);

                if (_motor_trigger.angle - _motor_trigger.last_angle > 4096)
                _motor_trigger.round_cnt--;

                else if (_motor_trigger.angle - _motor_trigger.last_angle < -4096)
                _motor_trigger.round_cnt++;

                _motor_trigger.total_ecd = _motor_trigger.round_cnt * 8192 + _motor_trigger.angle - _motor_trigger.offset_angle;
                _motor_trigger.total_angle = _motor_trigger.total_ecd * 360 / 8192;

                }

                break;

			case 0x202:
                _can1.data[1] = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
                _can1.data[7] = (int16_t)(hcan1.pRxMsg->Data[4]<<8 | hcan1.pRxMsg->Data[5]);
                break;

			case 0x203:
                _can1.data[2] = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
                _can1.data[8] = (int16_t)(hcan1.pRxMsg->Data[4]<<8 | hcan1.pRxMsg->Data[5]);
                break;

			case 0x204:
                _can1.data[3] = (int16_t)(hcan1.pRxMsg->Data[2]<<8 | hcan1.pRxMsg->Data[3]);
                _can1.data[9] = (int16_t)(hcan1.pRxMsg->Data[4]<<8 | hcan1.pRxMsg->Data[5]);
                break;

			case 0x205:
                _can1.data[4] = (int16_t)(hcan1.pRxMsg->Data[0]<<8 | hcan1.pRxMsg->Data[1]);
                break;

			case 0x206:
                _can1.data[5] = (int16_t)(hcan1.pRxMsg->Data[0]<<8 | hcan1.pRxMsg->Data[1]);

			default:
                break;
        }

        _can1.state = STATE_UPDATED;
    }

    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
}

void CAN2_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan2);

    if (_can2.state != STATE_BUSY) {
        _can2.state = STATE_BUSY;

        switch (hcan2.pRxMsg->StdId) {
			default:
                break;
        }

        _can2.state = STATE_UPDATED;
    }

    __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
}

int can1_read(uint8_t *buf, int len)
{
    if (_can1.state == STATE_UPDATED) {
        _can1.state = STATE_BUSY;

		if(len == 8)
		{
			buf[0] = _can1.data[0] >> 8;
			buf[1] = _can1.data[0];
			buf[2] = _can1.data[1] >> 8;
			buf[3] = _can1.data[1];
			buf[4] = _can1.data[2] >> 8;
			buf[5] = _can1.data[2];
			buf[6] = _can1.data[3] >> 8;
			buf[7] = _can1.data[3];
		}
		else if(len == 4)
		{
			buf[0] = _can1.data[4] >> 8;
			buf[1] = _can1.data[4];
			buf[2] = _can1.data[5] >> 8;
			buf[3] = _can1.data[5];
		}
        _can1.state = STATE_NULL;

        return 0;

    } else {
        return -1;
    }
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

    HAL_CAN_Transmit(&hcan1, 4);

    return 0;
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

    HAL_CAN_Transmit(&hcan2, 4);

    return 0;
}
